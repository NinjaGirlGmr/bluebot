#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class DriveCommandNode(Node):
    def __init__(self) -> None:
        super().__init__("drive_command_node")

        self.declare_parameter("command_topic", "/bluebot_v2/drive_command")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_speed", 0.20)
        self.declare_parameter("angular_speed", 1.00)
        self.declare_parameter("default_duration_sec", 0.0)

        self.command_topic = str(self.get_parameter("command_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.default_duration_sec = max(0.0, float(self.get_parameter("default_duration_sec").value))

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.cmd_sub = self.create_subscription(String, self.command_topic, self.on_command, 10)

        self.stop_timer: Optional[object] = None

        self.get_logger().info(
            "Drive command node ready. Commands: forward, backward, left, right, stop"
        )
        self.get_logger().info(
            f"Listening on {self.command_topic}, publishing Twist on {self.cmd_vel_topic}"
        )

    def on_command(self, msg: String) -> None:
        text = msg.data.strip().lower()
        if not text:
            self.get_logger().warn("Ignoring empty command")
            return

        parts = text.split()
        command = parts[0]

        duration_sec = self.default_duration_sec
        if len(parts) >= 2:
            try:
                duration_sec = max(0.0, float(parts[1]))
            except ValueError:
                self.get_logger().warn(
                    f"Invalid duration in '{msg.data}'. Use: '<command> [duration_sec]'"
                )
                return

        twist = Twist()

        if command == "forward":
            twist.linear.x = self.linear_speed
        elif command == "backward":
            twist.linear.x = -self.linear_speed
        elif command == "left":
            twist.angular.z = self.angular_speed
        elif command == "right":
            twist.angular.z = -self.angular_speed
        elif command == "stop":
            pass
        else:
            self.get_logger().warn(
                f"Unknown command '{command}'. Valid commands: forward, backward, left, right, stop"
            )
            return

        self.cmd_pub.publish(twist)

        if command == "stop":
            self.cancel_stop_timer()
            return

        if duration_sec > 0.0:
            self.schedule_stop(duration_sec)

    def schedule_stop(self, duration_sec: float) -> None:
        self.cancel_stop_timer()
        self.stop_timer = self.create_timer(duration_sec, self.on_stop_timer)

    def cancel_stop_timer(self) -> None:
        if self.stop_timer is not None:
            self.stop_timer.cancel()
            self.stop_timer = None

    def on_stop_timer(self) -> None:
        self.cancel_stop_timer()
        self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriveCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
