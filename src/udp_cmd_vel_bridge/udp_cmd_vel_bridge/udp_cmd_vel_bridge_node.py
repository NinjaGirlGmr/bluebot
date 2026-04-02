import json
import socket
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class UdpCmdVelBridge(Node):
    def __init__(self) -> None:
        super().__init__("udp_cmd_vel_bridge")

        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("listen_port", 8766)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("timeout_s", 0.5)

        listen_host = self.get_parameter("listen_host").get_parameter_value().string_value
        listen_port = (
            self.get_parameter("listen_port").get_parameter_value().integer_value
        )
        cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )
        self.timeout_s = (
            self.get_parameter("timeout_s").get_parameter_value().double_value
        )

        self.publisher_ = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.last_packet_time = 0.0
        self.last_nonzero_sent = False
        self._shutdown = False
        self._last_sender: Optional[str] = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((listen_host, int(listen_port)))
        self.sock.settimeout(0.2)

        self.receiver_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.receiver_thread.start()

        self.timer = self.create_timer(0.05, self._timeout_check)
        self.get_logger().info(
            f"Listening for UDP teleop on {listen_host}:{listen_port}, publishing to {cmd_vel_topic}"
        )

    def _recv_loop(self) -> None:
        while not self._shutdown:
            try:
                payload, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                if not self._shutdown:
                    self.get_logger().error("UDP socket closed unexpectedly")
                break

            try:
                message = json.loads(payload.decode("utf-8"))
                linear_x = float(message.get("linear_x", 0.0))
                angular_z = float(message.get("angular_z", 0.0))
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
                self.get_logger().warning(f"Ignoring malformed UDP packet from {addr}: {exc}")
                continue

            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            self.publisher_.publish(twist)
            self.last_packet_time = time.monotonic()
            self.last_nonzero_sent = abs(linear_x) > 1e-4 or abs(angular_z) > 1e-4

            sender = f"{addr[0]}:{addr[1]}"
            if sender != self._last_sender:
                self._last_sender = sender
                self.get_logger().info(f"Receiving teleop packets from {sender}")

    def _timeout_check(self) -> None:
        if self.last_packet_time <= 0.0:
            return

        if time.monotonic() - self.last_packet_time <= self.timeout_s:
            return

        if not self.last_nonzero_sent:
            return

        self.publisher_.publish(Twist())
        self.last_nonzero_sent = False
        self.get_logger().warn("UDP teleop timeout, publishing zero cmd_vel")

    def destroy_node(self) -> bool:
        self._shutdown = True
        try:
            self.sock.close()
        except OSError:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = UdpCmdVelBridge()
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
