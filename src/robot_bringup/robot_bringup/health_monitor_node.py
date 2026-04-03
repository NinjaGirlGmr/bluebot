#!/usr/bin/env python3

from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


@dataclass
class TopicWatch:
    topic: str
    type_name: str
    timeout_sec: float
    last_msg_time_ns: Optional[int] = None


class HealthMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__('health_monitor')

        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('topic_grace_period_sec', 10.0)
        self.declare_parameter('node_grace_period_sec', 15.0)
        self.declare_parameter('hardware_id', 'robot_bringup')
        self.declare_parameter('topic_names', [
            '/scan',
            '/imu/data_raw',
            '/imu/orientation',
            '/odom',
        ])
        self.declare_parameter('topic_types', [
            'sensor_msgs/msg/LaserScan',
            'sensor_msgs/msg/Imu',
            'sensor_msgs/msg/Imu',
            'nav_msgs/msg/Odometry',
        ])
        self.declare_parameter('topic_timeouts_sec', [2.0, 2.0, 2.0, 3.0])
        self.declare_parameter('expected_nodes', [
            'rplidar_node',
            'a471_serial_node',
            'imu_node',
            'robot_localization_filter',
            'slam_toolbox',
        ])

        self._publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._topic_grace_period_sec = float(self.get_parameter('topic_grace_period_sec').value)
        self._node_grace_period_sec = float(self.get_parameter('node_grace_period_sec').value)
        self._hardware_id = str(self.get_parameter('hardware_id').value)
        self._expected_nodes = list(self.get_parameter('expected_nodes').value)

        topic_names = list(self.get_parameter('topic_names').value)
        topic_types = list(self.get_parameter('topic_types').value)
        topic_timeouts_sec = list(self.get_parameter('topic_timeouts_sec').value)

        if len(topic_names) != len(topic_types) or len(topic_names) != len(topic_timeouts_sec):
            raise ValueError(
                'topic_names, topic_types, and topic_timeouts_sec must have the same length'
            )

        if self._publish_rate_hz <= 0.0:
            self._publish_rate_hz = 1.0
        if self._topic_grace_period_sec < 0.0:
            self._topic_grace_period_sec = 0.0
        if self._node_grace_period_sec < 0.0:
            self._node_grace_period_sec = 0.0

        self._startup_time_ns = self.get_clock().now().nanoseconds
        self._topic_watches: List[TopicWatch] = []
        self._last_reported_levels: Dict[str, int] = {}

        for idx, topic_name in enumerate(topic_names):
            watch = TopicWatch(
                topic=str(topic_name),
                type_name=str(topic_types[idx]),
                timeout_sec=float(topic_timeouts_sec[idx]),
            )
            self._topic_watches.append(watch)

            msg_type = get_message(watch.type_name)
            self.create_subscription(
                msg_type,
                watch.topic,
                lambda _msg, topic_idx=idx: self._on_topic(topic_idx),
                qos_profile_sensor_data,
            )

        self._diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0 / self._publish_rate_hz, self._publish_health)

        self.get_logger().info(
            'Health monitor started. '
            f'topics={topic_names}, expected_nodes={self._expected_nodes}, '
            f'publish_rate_hz={self._publish_rate_hz:.2f}'
        )

    def _on_topic(self, topic_idx: int) -> None:
        self._topic_watches[topic_idx].last_msg_time_ns = self.get_clock().now().nanoseconds

    def _kv(self, key: str, value: str) -> KeyValue:
        kv = KeyValue()
        kv.key = key
        kv.value = value
        return kv

    def _publish_health(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        uptime_sec = max(0.0, (now_ns - self._startup_time_ns) / 1.0e9)
        statuses: List[DiagnosticStatus] = []

        # Topic freshness checks.
        for watch in self._topic_watches:
            status = DiagnosticStatus()
            status.name = f'Health/Topic{watch.topic}'
            status.hardware_id = self._hardware_id
            status.values = [
                self._kv('topic', watch.topic),
                self._kv('type', watch.type_name),
                self._kv('timeout_sec', f'{watch.timeout_sec:.3f}'),
            ]

            if watch.last_msg_time_ns is None:
                if uptime_sec < self._topic_grace_period_sec:
                    status.level = DiagnosticStatus.WARN
                    status.message = 'Waiting for first message (startup grace period)'
                else:
                    status.level = DiagnosticStatus.ERROR
                    status.message = 'No messages received'
                status.values.append(self._kv('age_sec', 'n/a'))
            else:
                age_sec = max(0.0, (now_ns - watch.last_msg_time_ns) / 1.0e9)
                status.values.append(self._kv('age_sec', f'{age_sec:.3f}'))

                if age_sec <= watch.timeout_sec:
                    status.level = DiagnosticStatus.OK
                    status.message = 'Topic healthy'
                else:
                    status.level = DiagnosticStatus.ERROR
                    status.message = f'Topic stale ({age_sec:.2f}s since last message)'

            statuses.append(status)

        # Node liveness checks.
        graph_nodes = self.get_node_names_and_namespaces()
        names = {name for (name, _ns) in graph_nodes}
        fully_qualified = {
            (f'{ns}/{name}' if ns != '/' else f'/{name}')
            for (name, ns) in graph_nodes
        }

        for expected in self._expected_nodes:
            expected_name = str(expected)
            is_present = (
                expected_name in names
                or expected_name in fully_qualified
                or f'/{expected_name}' in fully_qualified
            )

            status = DiagnosticStatus()
            status.name = f'Health/Node/{expected_name}'
            status.hardware_id = self._hardware_id
            status.values = [self._kv('node', expected_name)]

            if is_present:
                status.level = DiagnosticStatus.OK
                status.message = 'Node present'
            else:
                if uptime_sec < self._node_grace_period_sec:
                    status.level = DiagnosticStatus.WARN
                    status.message = 'Node not found yet (startup grace period)'
                else:
                    status.level = DiagnosticStatus.ERROR
                    status.message = 'Node missing'

            statuses.append(status)

        summary = DiagnosticStatus()
        summary.name = 'Health/Summary'
        summary.hardware_id = self._hardware_id
        summary.level = DiagnosticStatus.OK
        for item in statuses:
            summary.level = max(summary.level, item.level)

        if summary.level == DiagnosticStatus.OK:
            summary.message = 'All monitored systems healthy'
        elif summary.level == DiagnosticStatus.WARN:
            summary.message = 'Monitoring warnings present'
        else:
            summary.message = 'Monitoring errors present'

        summary.values = [
            self._kv('uptime_sec', f'{uptime_sec:.1f}'),
            self._kv('checks_total', str(len(statuses))),
            self._kv('checks_ok', str(sum(1 for s in statuses if s.level == DiagnosticStatus.OK))),
            self._kv('checks_warn', str(sum(1 for s in statuses if s.level == DiagnosticStatus.WARN))),
            self._kv('checks_error', str(sum(1 for s in statuses if s.level == DiagnosticStatus.ERROR))),
        ]

        all_statuses = [summary] + statuses
        self._emit_status_transitions(all_statuses)

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = all_statuses
        self._diagnostics_pub.publish(msg)

    def _emit_status_transitions(self, statuses: List[DiagnosticStatus]) -> None:
        for status in statuses:
            prev_level = self._last_reported_levels.get(status.name)
            self._last_reported_levels[status.name] = status.level
            if prev_level is None or prev_level == status.level:
                continue

            event = f'{status.name}: {status.message}'
            if status.level == DiagnosticStatus.OK:
                self.get_logger().info(f'Recovered: {event}')
            elif status.level == DiagnosticStatus.WARN:
                self.get_logger().warning(event)
            else:
                self.get_logger().error(event)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = HealthMonitorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
