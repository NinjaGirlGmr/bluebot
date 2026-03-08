#!/usr/bin/env python3

import copy

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class CmdVelSafetyGate(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_safety_gate")

        self.declare_parameter("input_cmd_topic", "/cmd_vel")
        self.declare_parameter("output_cmd_topic", "/cmd_vel_safe")
        self.declare_parameter("drop_topic", "/drop_detected")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("release_hold_sec", 0.8)

        self.input_cmd_topic = str(self.get_parameter("input_cmd_topic").value)
        self.output_cmd_topic = str(self.get_parameter("output_cmd_topic").value)
        self.drop_topic = str(self.get_parameter("drop_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.command_timeout_sec = float(self.get_parameter("command_timeout_sec").value)
        self.release_hold_sec = float(self.get_parameter("release_hold_sec").value)

        self.drop_detected = False
        self.release_hold_until_ns = 0
        self.last_cmd = Twist()
        self.last_cmd_stamp_ns = 0

        self.cmd_sub = self.create_subscription(Twist, self.input_cmd_topic, self.on_cmd, 20)
        self.drop_sub = self.create_subscription(Bool, self.drop_topic, self.on_drop, 20)
        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 20)

        timer_period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.05
        self.timer = self.create_timer(timer_period, self.on_timer)

        self.get_logger().info(
            f"CmdVelSafetyGate in={self.input_cmd_topic} out={self.output_cmd_topic} drop={self.drop_topic}"
        )

    def on_cmd(self, msg: Twist) -> None:
        self.last_cmd = copy.deepcopy(msg)
        self.last_cmd_stamp_ns = self.get_clock().now().nanoseconds

    def on_drop(self, msg: Bool) -> None:
        prev = self.drop_detected
        self.drop_detected = bool(msg.data)
        now_ns = self.get_clock().now().nanoseconds

        if self.drop_detected and not prev:
            self.get_logger().warn("Drop hazard active. Forcing /cmd_vel_safe = 0.")
        elif (not self.drop_detected) and prev:
            self.release_hold_until_ns = now_ns + int(self.release_hold_sec * 1e9)
            self.get_logger().info("Drop hazard cleared. Holding stop briefly before release.")

    def on_timer(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        stop_for_drop = self.drop_detected or (now_ns < self.release_hold_until_ns)
        stale_cmd = (now_ns - self.last_cmd_stamp_ns) > int(self.command_timeout_sec * 1e9)

        out = Twist()
        if not stop_for_drop and not stale_cmd:
            out = copy.deepcopy(self.last_cmd)

        self.cmd_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelSafetyGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ignore errors triggered during global context shutdown.
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
