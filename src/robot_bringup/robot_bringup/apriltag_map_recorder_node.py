#!/usr/bin/env python3

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Tuple

import rclpy
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


QuaternionTuple = Tuple[float, float, float, float]
Vector3Tuple = Tuple[float, float, float]


@dataclass
class TagAggregate:
    family: str
    tag_id: int
    observations: int = 0
    sum_x: float = 0.0
    sum_y: float = 0.0
    sum_z: float = 0.0
    latest_qx: float = 0.0
    latest_qy: float = 0.0
    latest_qz: float = 0.0
    latest_qw: float = 1.0
    first_seen_sec: float = 0.0
    last_seen_sec: float = 0.0


class AprilTagMapRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__('apriltag_map_recorder')

        self.declare_parameter('detections_topic', '/tag_detections')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('output_yaml', '/tmp/apriltag_map_landmarks.yaml')
        self.declare_parameter('write_period_sec', 5.0)
        self.declare_parameter('min_observations', 1)
        self.declare_parameter('use_latest_tf_on_failure', True)
        self.declare_parameter('write_on_shutdown', True)
        self.declare_parameter('log_new_tags', True)

        self._detections_topic = str(self.get_parameter('detections_topic').value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._output_yaml = str(self.get_parameter('output_yaml').value)
        self._write_period_sec = float(self.get_parameter('write_period_sec').value)
        self._min_observations = int(self.get_parameter('min_observations').value)
        self._use_latest_tf_on_failure = bool(
            self.get_parameter('use_latest_tf_on_failure').value
        )
        self._write_on_shutdown = bool(self.get_parameter('write_on_shutdown').value)
        self._log_new_tags = bool(self.get_parameter('log_new_tags').value)

        if self._write_period_sec <= 0.0:
            self._write_period_sec = 5.0
        if self._min_observations < 1:
            self._min_observations = 1

        self._tag_data: Dict[str, TagAggregate] = {}
        self._dirty = False
        self._last_warn_ns: Dict[str, int] = {}

        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            AprilTagDetectionArray,
            self._detections_topic,
            self._on_detections,
            qos_profile_sensor_data,
        )
        self.create_timer(self._write_period_sec, self._on_write_timer)

        self.get_logger().info(
            'AprilTag map recorder started. '
            f'detections_topic={self._detections_topic}, map_frame={self._map_frame}, '
            f'output_yaml={self._output_yaml}'
        )

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        for detection in msg.detections:
            source_frame = detection.pose.header.frame_id or msg.header.frame_id
            if not source_frame:
                self._warn_throttled(
                    'missing_source_frame',
                    'Skipping detection with empty source frame_id.',
                )
                continue

            pose_stamp = detection.pose.header.stamp
            if self._is_zero_stamp(pose_stamp):
                pose_stamp = msg.header.stamp
            stamp_time = self._stamp_to_time(pose_stamp)
            source_pose = detection.pose.pose.pose

            transformed = self._transform_pose_to_map(
                source_frame=source_frame,
                stamp_time=stamp_time,
                position=(
                    float(source_pose.position.x),
                    float(source_pose.position.y),
                    float(source_pose.position.z),
                ),
                orientation=(
                    float(source_pose.orientation.x),
                    float(source_pose.orientation.y),
                    float(source_pose.orientation.z),
                    float(source_pose.orientation.w),
                ),
            )

            if transformed is None:
                continue

            pos_map, quat_map = transformed
            now_sec = self._now_sec()
            family = detection.family if detection.family else 'unknown'
            key = f'{family}:{int(detection.id)}'

            aggregate = self._tag_data.get(key)
            if aggregate is None:
                aggregate = TagAggregate(
                    family=family,
                    tag_id=int(detection.id),
                    first_seen_sec=now_sec,
                )
                self._tag_data[key] = aggregate
                if self._log_new_tags:
                    self.get_logger().info(
                        f'New tag observed: family={aggregate.family}, id={aggregate.tag_id}'
                    )

            aggregate.observations += 1
            aggregate.sum_x += pos_map[0]
            aggregate.sum_y += pos_map[1]
            aggregate.sum_z += pos_map[2]
            aggregate.latest_qx = quat_map[0]
            aggregate.latest_qy = quat_map[1]
            aggregate.latest_qz = quat_map[2]
            aggregate.latest_qw = quat_map[3]
            aggregate.last_seen_sec = now_sec

            if aggregate.first_seen_sec <= 0.0:
                aggregate.first_seen_sec = now_sec

            self._dirty = True

    def _transform_pose_to_map(
        self,
        source_frame: str,
        stamp_time: Time,
        position: Vector3Tuple,
        orientation: QuaternionTuple,
    ) -> Tuple[Vector3Tuple, QuaternionTuple] | None:
        transform = None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                source_frame,
                stamp_time,
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            if self._use_latest_tf_on_failure:
                try:
                    transform = self._tf_buffer.lookup_transform(
                        self._map_frame,
                        source_frame,
                        Time(),
                        timeout=Duration(seconds=0.05),
                    )
                except TransformException:
                    self._warn_throttled(
                        f'tf_lookup_latest_{source_frame}',
                        f'Failed transform {source_frame}->{self._map_frame}: {exc}',
                    )
                    return None
            else:
                self._warn_throttled(
                    f'tf_lookup_{source_frame}',
                    f'Failed transform {source_frame}->{self._map_frame}: {exc}',
                )
                return None

        tf_t = transform.transform.translation
        tf_q = transform.transform.rotation
        tf_translation: Vector3Tuple = (float(tf_t.x), float(tf_t.y), float(tf_t.z))
        tf_quaternion: QuaternionTuple = (
            float(tf_q.x),
            float(tf_q.y),
            float(tf_q.z),
            float(tf_q.w),
        )

        rotated_position = self._quat_rotate_vector(tf_quaternion, position)
        position_out: Vector3Tuple = (
            rotated_position[0] + tf_translation[0],
            rotated_position[1] + tf_translation[1],
            rotated_position[2] + tf_translation[2],
        )
        orientation_out = self._quat_normalize(
            self._quat_multiply(tf_quaternion, orientation)
        )
        return (position_out, orientation_out)

    def _on_write_timer(self) -> None:
        self.write_yaml_snapshot(force=False)

    def write_yaml_snapshot(self, force: bool) -> None:
        if not force and not self._dirty:
            return

        output_path = Path(self._output_yaml).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)

        tags = []
        for item in self._sorted_tag_items():
            if item.observations < self._min_observations:
                continue
            inv_count = 1.0 / float(item.observations)
            tags.append({
                'family': item.family,
                'id': item.tag_id,
                'observations': item.observations,
                'first_seen_sec': round(item.first_seen_sec, 6),
                'last_seen_sec': round(item.last_seen_sec, 6),
                'position': {
                    'x': round(item.sum_x * inv_count, 6),
                    'y': round(item.sum_y * inv_count, 6),
                    'z': round(item.sum_z * inv_count, 6),
                },
                'orientation': {
                    'x': round(item.latest_qx, 6),
                    'y': round(item.latest_qy, 6),
                    'z': round(item.latest_qz, 6),
                    'w': round(item.latest_qw, 6),
                },
            })

        payload = {
            'map_frame': self._map_frame,
            'generated_at_sec': round(self._now_sec(), 6),
            'detections_topic': self._detections_topic,
            'min_observations': self._min_observations,
            'tags': tags,
        }

        try:
            with output_path.open('w', encoding='utf-8') as f:
                yaml.safe_dump(payload, f, sort_keys=False)
            self._dirty = False
            self.get_logger().info(
                f'Wrote {len(tags)} tag landmarks to {output_path}'
            )
        except OSError as exc:
            self.get_logger().error(f'Failed writing YAML to {output_path}: {exc}')

    def _sorted_tag_items(self) -> list[TagAggregate]:
        return sorted(
            self._tag_data.values(),
            key=lambda item: (item.family, item.tag_id),
        )

    def _stamp_to_time(self, stamp_msg) -> Time:
        if self._is_zero_stamp(stamp_msg):
            return Time()
        return Time.from_msg(stamp_msg)

    @staticmethod
    def _is_zero_stamp(stamp_msg) -> bool:
        return int(stamp_msg.sec) == 0 and int(stamp_msg.nanosec) == 0

    def _now_sec(self) -> float:
        now = self.get_clock().now().nanoseconds
        return float(now) / 1.0e9

    def _warn_throttled(self, key: str, msg: str, period_sec: float = 5.0) -> None:
        now_ns = self.get_clock().now().nanoseconds
        prev_ns = self._last_warn_ns.get(key)
        if prev_ns is not None and now_ns < prev_ns + int(period_sec * 1.0e9):
            return
        self._last_warn_ns[key] = now_ns
        self.get_logger().warning(msg)

    @staticmethod
    def _quat_multiply(a: QuaternionTuple, b: QuaternionTuple) -> QuaternionTuple:
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (
            (aw * bx) + (ax * bw) + (ay * bz) - (az * by),
            (aw * by) - (ax * bz) + (ay * bw) + (az * bx),
            (aw * bz) + (ax * by) - (ay * bx) + (az * bw),
            (aw * bw) - (ax * bx) - (ay * by) - (az * bz),
        )

    @staticmethod
    def _quat_conjugate(q: QuaternionTuple) -> QuaternionTuple:
        qx, qy, qz, qw = q
        return (-qx, -qy, -qz, qw)

    @classmethod
    def _quat_rotate_vector(cls, q: QuaternionTuple, v: Vector3Tuple) -> Vector3Tuple:
        vx, vy, vz = v
        v_quat: QuaternionTuple = (vx, vy, vz, 0.0)
        rotated = cls._quat_multiply(
            cls._quat_multiply(q, v_quat),
            cls._quat_conjugate(q),
        )
        return (rotated[0], rotated[1], rotated[2])

    @staticmethod
    def _quat_normalize(q: QuaternionTuple) -> QuaternionTuple:
        qx, qy, qz, qw = q
        norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
        if norm <= 1.0e-9:
            return (0.0, 0.0, 0.0, 1.0)
        inv_norm = 1.0 / norm
        return (qx * inv_norm, qy * inv_norm, qz * inv_norm, qw * inv_norm)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AprilTagMapRecorderNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if node._write_on_shutdown:
                node.write_yaml_snapshot(force=True)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
