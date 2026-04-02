#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    # Standard ROS yaw extraction from quaternion.
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class StraightLineCompensatorNode(Node):
    def __init__(self) -> None:
        super().__init__("straight_line_compensator")

        self.declare_parameter("input_cmd_topic", "/cmd_vel")
        self.declare_parameter("output_cmd_topic", "/cmd_vel_compensated")
        self.declare_parameter("odom_topic", "/odom_raw")
        self.declare_parameter("enabled", True)

        self.declare_parameter("linear_speed_min", 0.03)
        self.declare_parameter("angular_deadband", 0.05)
        self.declare_parameter("turn_memory_angular_threshold", 0.20)

        self.declare_parameter("kp", 1.8)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.20)
        self.declare_parameter("max_angular_correction", 0.50)
        self.declare_parameter("integral_limit", 0.40)

        self.declare_parameter("caster_comp_enabled", True)
        self.declare_parameter("caster_comp_gain", 0.12)
        self.declare_parameter("caster_comp_decay_sec", 0.60)
        self.declare_parameter("caster_comp_max", 0.25)
        self.declare_parameter("caster_comp_max_age_sec", 2.0)
        self.declare_parameter("caster_comp_forward_only", True)

        self.declare_parameter("odom_timeout_sec", 0.40)
        self.declare_parameter("reset_heading_on_reverse", True)

        self.input_cmd_topic = str(self.get_parameter("input_cmd_topic").value)
        self.output_cmd_topic = str(self.get_parameter("output_cmd_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.enabled = bool(self.get_parameter("enabled").value)

        self.linear_speed_min = max(0.0, float(self.get_parameter("linear_speed_min").value))
        self.angular_deadband = max(0.0, float(self.get_parameter("angular_deadband").value))
        self.turn_memory_angular_threshold = max(
            0.0, float(self.get_parameter("turn_memory_angular_threshold").value)
        )

        self.kp = float(self.get_parameter("kp").value)
        self.ki = float(self.get_parameter("ki").value)
        self.kd = float(self.get_parameter("kd").value)
        self.max_angular_correction = max(0.0, float(self.get_parameter("max_angular_correction").value))
        self.integral_limit = max(0.0, float(self.get_parameter("integral_limit").value))

        self.caster_comp_enabled = bool(self.get_parameter("caster_comp_enabled").value)
        self.caster_comp_gain = max(0.0, float(self.get_parameter("caster_comp_gain").value))
        self.caster_comp_decay_sec = max(0.05, float(self.get_parameter("caster_comp_decay_sec").value))
        self.caster_comp_max = max(0.0, float(self.get_parameter("caster_comp_max").value))
        self.caster_comp_max_age_sec = max(0.0, float(self.get_parameter("caster_comp_max_age_sec").value))
        self.caster_comp_forward_only = bool(self.get_parameter("caster_comp_forward_only").value)

        self.odom_timeout_sec = max(0.05, float(self.get_parameter("odom_timeout_sec").value))
        self.reset_heading_on_reverse = bool(self.get_parameter("reset_heading_on_reverse").value)

        self.current_yaw: Optional[float] = None
        self.current_yaw_rate = 0.0
        self.last_odom_time = None

        self.hold_active = False
        self.target_yaw = 0.0
        self.integral_error = 0.0
        self.last_control_time = None
        self.hold_start_time = None
        self.last_linear_sign = 0
        self.last_turn_sign = 0
        self.last_turn_time = None

        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 20)
        self.cmd_sub = self.create_subscription(Twist, self.input_cmd_topic, self.on_cmd, 20)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)

        self._stale_odom_warned = False

        self.get_logger().info(
            "Straight-line compensator started: "
            f"input={self.input_cmd_topic}, output={self.output_cmd_topic}, odom={self.odom_topic}, "
            f"enabled={self.enabled}, kp={self.kp:.3f}, ki={self.ki:.3f}, kd={self.kd:.3f}, "
            f"max_corr={self.max_angular_correction:.3f}, caster_comp={self.caster_comp_enabled}"
        )

    def on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.current_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.current_yaw_rate = float(msg.twist.twist.angular.z)
        self.last_odom_time = self.get_clock().now()
        self._stale_odom_warned = False

    def reset_hold(self) -> None:
        self.hold_active = False
        self.integral_error = 0.0
        self.last_control_time = None
        self.hold_start_time = None
        self.last_linear_sign = 0

    def odom_is_fresh(self) -> bool:
        if self.last_odom_time is None:
            return False
        age = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1.0e9
        return age <= self.odom_timeout_sec

    def on_cmd(self, msg: Twist) -> None:
        now = self.get_clock().now()
        out = Twist()
        out.linear = msg.linear
        out.angular = msg.angular

        if not self.enabled:
            self.reset_hold()
            self.cmd_pub.publish(out)
            return

        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)

        if abs(angular_z) >= self.turn_memory_angular_threshold:
            self.last_turn_sign = 1 if angular_z > 0.0 else -1
            self.last_turn_time = now

        moving_straight = abs(linear_x) >= self.linear_speed_min and abs(angular_z) <= self.angular_deadband

        if not moving_straight:
            self.reset_hold()
            self.cmd_pub.publish(out)
            return

        if self.current_yaw is None or not self.odom_is_fresh():
            self.reset_hold()
            if not self._stale_odom_warned:
                self.get_logger().warn("Odom unavailable/stale; straight-line compensation bypassed")
                self._stale_odom_warned = True
            self.cmd_pub.publish(out)
            return

        linear_sign = 1 if linear_x > 0.0 else -1

        if not self.hold_active:
            self.hold_active = True
            self.target_yaw = self.current_yaw
            self.integral_error = 0.0
            self.last_control_time = now
            self.hold_start_time = now
            self.last_linear_sign = linear_sign
        elif self.reset_heading_on_reverse and linear_sign != self.last_linear_sign:
            self.target_yaw = self.current_yaw
            self.integral_error = 0.0
            self.last_control_time = now
            self.hold_start_time = now
            self.last_linear_sign = linear_sign

        if self.last_control_time is None:
            dt = 0.0
        else:
            dt = (now - self.last_control_time).nanoseconds / 1.0e9
        self.last_control_time = now

        yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
        if dt > 0.0:
            self.integral_error += yaw_error * dt
            self.integral_error = clamp(self.integral_error, -self.integral_limit, self.integral_limit)

        correction = self.kp * yaw_error + self.ki * self.integral_error - self.kd * self.current_yaw_rate
        correction += self.compute_caster_countersteer(now, linear_x)
        correction = clamp(correction, -self.max_angular_correction, self.max_angular_correction)

        out.angular.z = correction
        self.cmd_pub.publish(out)

    def compute_caster_countersteer(self, now, linear_x: float) -> float:
        if not self.caster_comp_enabled or self.last_turn_sign == 0:
            return 0.0
        if self.last_turn_time is None or self.hold_start_time is None:
            return 0.0
        if self.caster_comp_forward_only and linear_x < 0.0:
            return 0.0

        turn_age = (now - self.last_turn_time).nanoseconds / 1.0e9
        if turn_age > self.caster_comp_max_age_sec:
            return 0.0

        hold_age = (now - self.hold_start_time).nanoseconds / 1.0e9
        if hold_age < 0.0:
            return 0.0

        decay = math.exp(-hold_age / self.caster_comp_decay_sec)
        countersteer = -float(self.last_turn_sign) * self.caster_comp_gain * decay
        return clamp(countersteer, -self.caster_comp_max, self.caster_comp_max)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = StraightLineCompensatorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
