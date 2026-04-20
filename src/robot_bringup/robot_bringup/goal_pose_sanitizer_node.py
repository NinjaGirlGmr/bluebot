#!/usr/bin/env python3

import math

from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class GoalPoseSanitizerNode(Node):
    def __init__(self) -> None:
        super().__init__('goal_pose_sanitizer')

        self.declare_parameter('input_topic', '/goal_pose')
        self.declare_parameter('output_topic', '/goal_pose_sanitized')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('transform_timeout_sec', 0.2)

        self._input_topic = str(self.get_parameter('input_topic').value)
        self._output_topic = str(self.get_parameter('output_topic').value)
        self._target_frame = str(self.get_parameter('target_frame').value)
        self._transform_timeout_sec = float(
            self.get_parameter('transform_timeout_sec').value
        )

        if self._input_topic == self._output_topic:
            raise RuntimeError('input_topic and output_topic must be different')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._goal_sub = self.create_subscription(
            PoseStamped, self._input_topic, self._on_goal_pose, 10
        )
        self._goal_pub = self.create_publisher(PoseStamped, self._output_topic, 10)

        self.get_logger().info(
            'Goal pose sanitizer ready. '
            f'input_topic={self._input_topic} output_topic={self._output_topic} '
            f'target_frame={self._target_frame} transform_timeout_sec={self._transform_timeout_sec:.2f}'
        )

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        source_frame = msg.header.frame_id.strip() if msg.header.frame_id else ''
        if not source_frame:
            source_frame = self._target_frame

        output = PoseStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self._target_frame

        if source_frame == self._target_frame:
            output.pose = msg.pose
            self._goal_pub.publish(output)
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=self._transform_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'Unable to transform goal from "{source_frame}" to "{self._target_frame}": {exc}'
            )
            return

        output.pose = self._transform_pose_2d(msg.pose, transform)
        self._goal_pub.publish(output)

    @staticmethod
    def _transform_pose_2d(pose: Pose, transform: TransformStamped) -> Pose:
        tx = float(transform.transform.translation.x)
        ty = float(transform.transform.translation.y)

        t_q = transform.transform.rotation
        tf_yaw = _quat_to_yaw(t_q.x, t_q.y, t_q.z, t_q.w)
        cos_yaw = math.cos(tf_yaw)
        sin_yaw = math.sin(tf_yaw)

        px = float(pose.position.x)
        py = float(pose.position.y)
        p_q = pose.orientation
        pyaw = _quat_to_yaw(p_q.x, p_q.y, p_q.z, p_q.w)

        x_map = tx + (cos_yaw * px) - (sin_yaw * py)
        y_map = ty + (sin_yaw * px) + (cos_yaw * py)
        yaw_map = tf_yaw + pyaw
        qx, qy, qz, qw = _yaw_to_quat(yaw_map)

        out = Pose()
        out.position.x = x_map
        out.position.y = y_map
        out.position.z = float(pose.position.z + transform.transform.translation.z)
        out.orientation.x = qx
        out.orientation.y = qy
        out.orientation.z = qz
        out.orientation.w = qw
        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = GoalPoseSanitizerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
