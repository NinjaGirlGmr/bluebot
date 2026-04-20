import math
from typing import List

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

DEFAULT_COVARIANCE = [
    0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942,
]
MIN_VALID_VARIANCE = 1.0e-9


class IsaacToNav2Pose(Node):
    def __init__(self) -> None:
        super().__init__('isaac_to_nav2_pose')

        self.declare_parameter('output_topic', '/initialpose')
        self.declare_parameter('set_pose_topic', '/set_pose')
        self.declare_parameter('pose_stamped_topic', '/grid_search_pose')
        self.declare_parameter(
            'pose_with_covariance_topic', '/localization_result')
        self.declare_parameter('enable_pose_stamped_input', True)
        self.declare_parameter('enable_pose_with_covariance_input', True)
        self.declare_parameter('publish_set_pose', True)
        self.declare_parameter('output_frame_id', 'map')
        self.declare_parameter('pose_stamped_covariance', DEFAULT_COVARIANCE)
        self.declare_parameter('use_current_output_stamp', False)
        self.declare_parameter('fallback_initial_pose_enabled', True)
        self.declare_parameter('fallback_initial_pose_wait_sec', 6.0)
        self.declare_parameter('fallback_initial_pose_publish_count', 5)
        self.declare_parameter('fallback_initial_pose_publish_period_sec', 0.5)
        self.declare_parameter('fallback_initial_pose_x', 0.0)
        self.declare_parameter('fallback_initial_pose_y', 0.0)
        self.declare_parameter('fallback_initial_pose_yaw', 0.0)

        output_topic = self.get_parameter('output_topic').value
        set_pose_topic = self.get_parameter('set_pose_topic').value
        pose_stamped_topic = self.get_parameter('pose_stamped_topic').value
        pose_with_covariance_topic = self.get_parameter(
            'pose_with_covariance_topic').value
        self.enable_pose_stamped_input = self.get_parameter(
            'enable_pose_stamped_input').value
        self.enable_pose_with_covariance_input = self.get_parameter(
            'enable_pose_with_covariance_input').value
        self.publish_set_pose = self.get_parameter('publish_set_pose').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        self.use_current_output_stamp = self.get_parameter(
            'use_current_output_stamp').value
        self.fallback_initial_pose_enabled = self.get_parameter(
            'fallback_initial_pose_enabled').value
        self.fallback_initial_pose_wait_sec = float(self.get_parameter(
            'fallback_initial_pose_wait_sec').value)
        self.fallback_initial_pose_publish_count = int(self.get_parameter(
            'fallback_initial_pose_publish_count').value)
        self.fallback_initial_pose_publish_period_sec = float(self.get_parameter(
            'fallback_initial_pose_publish_period_sec').value)
        self.fallback_initial_pose_x = float(self.get_parameter(
            'fallback_initial_pose_x').value)
        self.fallback_initial_pose_y = float(self.get_parameter(
            'fallback_initial_pose_y').value)
        self.fallback_initial_pose_yaw = float(self.get_parameter(
            'fallback_initial_pose_yaw').value)

        self.pose_stamped_covariance = self._load_covariance_parameter()
        self.received_input = False
        self._invalid_covariance_warned = False
        self._fallback_start_timer = None
        self._fallback_publish_timer = None
        self._fallback_remaining_publishes = 0

        initialpose_qos = QoSProfile(depth=10)
        initialpose_qos.reliability = QoSReliabilityPolicy.RELIABLE
        initialpose_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, output_topic, initialpose_qos)
        self.set_pose_pub = None
        if self.publish_set_pose:
            set_pose_qos = QoSProfile(depth=10)
            set_pose_qos.reliability = QoSReliabilityPolicy.RELIABLE
            set_pose_qos.durability = QoSDurabilityPolicy.VOLATILE
            self.set_pose_pub = self.create_publisher(
                PoseWithCovarianceStamped, set_pose_topic, set_pose_qos)

        if self.enable_pose_stamped_input:
            self.create_subscription(
                PoseStamped,
                pose_stamped_topic,
                self.pose_stamped_callback,
                10)

        if self.enable_pose_with_covariance_input:
            self.create_subscription(
                PoseWithCovarianceStamped,
                pose_with_covariance_topic,
                self.pose_with_covariance_callback,
                10)

        if (not self.enable_pose_stamped_input and
                not self.enable_pose_with_covariance_input):
            self.get_logger().warning(
                'Both inputs are disabled; bridge will never publish.')

        if self.fallback_initial_pose_enabled:
            self._fallback_start_timer = self.create_timer(
                self.fallback_initial_pose_wait_sec,
                self._start_fallback_publish
            )

        self.get_logger().info(
            'Bridge ready. output_topic=%s pose_stamped_topic=%s '
            'pose_with_covariance_topic=%s output_frame_id=%s '
            'publish_set_pose=%s set_pose_topic=%s '
            'fallback_initial_pose_enabled=%s fallback_wait_sec=%.2f'
            % (
                output_topic,
                pose_stamped_topic,
                pose_with_covariance_topic,
                self.output_frame_id if self.output_frame_id else '<unchanged>',
                self.publish_set_pose,
                set_pose_topic,
                self.fallback_initial_pose_enabled,
                self.fallback_initial_pose_wait_sec,
            )
        )

    def _load_covariance_parameter(self) -> List[float]:
        covariance = self.get_parameter('pose_stamped_covariance').value
        if len(covariance) != 36:
            self.get_logger().warning(
                'pose_stamped_covariance must contain 36 values; '
                'using default covariance.')
            return DEFAULT_COVARIANCE
        return [float(value) for value in covariance]

    def _apply_frame_id(self, frame_id: str) -> str:
        if self.output_frame_id:
            return self.output_frame_id
        return frame_id

    def _has_valid_pose_covariance(self, covariance: List[float]) -> bool:
        if len(covariance) != 36:
            return False
        if any(not math.isfinite(value) for value in covariance):
            return False
        # x, y, yaw variances must be positive for downstream filters.
        return (
            covariance[0] > MIN_VALID_VARIANCE
            and covariance[7] > MIN_VALID_VARIANCE
            and covariance[35] > MIN_VALID_VARIANCE
        )

    def _mark_input_received(self) -> None:
        if self.received_input:
            return
        self.received_input = True
        if self._fallback_start_timer is not None:
            self._fallback_start_timer.cancel()
            self._fallback_start_timer = None
        if self._fallback_publish_timer is not None:
            self._fallback_publish_timer.cancel()
            self._fallback_publish_timer = None

    def _start_fallback_publish(self) -> None:
        if self._fallback_start_timer is not None:
            self._fallback_start_timer.cancel()
            self._fallback_start_timer = None
        if self.received_input:
            return
        self._fallback_remaining_publishes = max(
            1, self.fallback_initial_pose_publish_count)
        self.get_logger().warning(
            'No localization input received within %.2fs; publishing '
            'fallback initial pose (x=%.3f, y=%.3f, yaw=%.3f rad).'
            % (
                self.fallback_initial_pose_wait_sec,
                self.fallback_initial_pose_x,
                self.fallback_initial_pose_y,
                self.fallback_initial_pose_yaw,
            )
        )
        self._publish_fallback_initial_pose()
        if self._fallback_remaining_publishes > 0:
            self._fallback_publish_timer = self.create_timer(
                self.fallback_initial_pose_publish_period_sec,
                self._publish_fallback_initial_pose
            )

    def _publish_with_stamp(
            self,
            output: PoseWithCovarianceStamped,
            mirror_set_pose: bool = True) -> None:
        if self.use_current_output_stamp:
            output.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(output)
        if mirror_set_pose and self.set_pose_pub is not None:
            self.set_pose_pub.publish(output)

    def _publish_fallback_initial_pose(self) -> None:
        if self.received_input:
            if self._fallback_publish_timer is not None:
                self._fallback_publish_timer.cancel()
                self._fallback_publish_timer = None
            return

        output = PoseWithCovarianceStamped()
        output.header.frame_id = self._apply_frame_id('map')
        output.pose.pose.position.x = self.fallback_initial_pose_x
        output.pose.pose.position.y = self.fallback_initial_pose_y
        output.pose.pose.orientation.z = math.sin(
            self.fallback_initial_pose_yaw / 2.0)
        output.pose.pose.orientation.w = math.cos(
            self.fallback_initial_pose_yaw / 2.0)
        output.pose.covariance = self.pose_stamped_covariance
        self._publish_with_stamp(output, mirror_set_pose=False)

        self._fallback_remaining_publishes -= 1
        if self._fallback_remaining_publishes <= 0:
            if self._fallback_publish_timer is not None:
                self._fallback_publish_timer.cancel()
                self._fallback_publish_timer = None

    def pose_stamped_callback(self, msg: PoseStamped) -> None:
        self._mark_input_received()
        output = PoseWithCovarianceStamped()
        output.header = msg.header
        output.header.frame_id = self._apply_frame_id(msg.header.frame_id)
        output.pose.pose = msg.pose
        output.pose.covariance = self.pose_stamped_covariance
        self._publish_with_stamp(output)

    def pose_with_covariance_callback(
            self, msg: PoseWithCovarianceStamped) -> None:
        self._mark_input_received()
        output = PoseWithCovarianceStamped()
        output.header = msg.header
        output.header.frame_id = self._apply_frame_id(msg.header.frame_id)
        output.pose = msg.pose
        if not self._has_valid_pose_covariance(output.pose.covariance):
            output.pose.covariance = self.pose_stamped_covariance
            if not self._invalid_covariance_warned:
                self.get_logger().warning(
                    'Received PoseWithCovariance with invalid x/y/yaw variances; '
                    'using configured default covariance for /initialpose output.'
                )
                self._invalid_covariance_warned = True
        self._publish_with_stamp(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IsaacToNav2Pose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
