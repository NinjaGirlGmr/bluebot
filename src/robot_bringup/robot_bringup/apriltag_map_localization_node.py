#!/usr/bin/env python3

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple
import math

from geometry_msgs.msg import PoseWithCovarianceStamped
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
import rclpy
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
class TagLandmark:
    family: str
    tag_id: int
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    observations: int


@dataclass
class CandidatePose:
    map_to_base_translation: Vector3Tuple
    map_to_base_quaternion: QuaternionTuple
    distance_m: float
    stamp: Time
    source_frame: str
    tag_family: str
    tag_id: int


class AprilTagMapLocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__('apriltag_map_localization')

        self.declare_parameter('detections_topic', '/tag_detections')
        self.declare_parameter('output_pose_topic', '/apriltag/map_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('landmarks_file', '')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('auto_landmarks_from_map', True)
        self.declare_parameter('use_landmark_yaw', True)
        self.declare_parameter('min_detection_range_m', 0.05)
        self.declare_parameter('max_detection_range_m', 4.5)
        self.declare_parameter('base_position_stddev_m', 0.05)
        self.declare_parameter('base_yaw_stddev_rad', 0.15)
        self.declare_parameter('position_stddev_per_meter', 0.03)
        self.declare_parameter('yaw_stddev_per_meter', 0.07)
        self.declare_parameter('tf_lookup_timeout_sec', 0.08)
        self.declare_parameter('use_latest_tf_on_failure', True)
        self.declare_parameter('debug_log_matches', False)

        self._detections_topic = str(self.get_parameter('detections_topic').value)
        self._output_pose_topic = str(self.get_parameter('output_pose_topic').value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._landmarks_file_param = str(self.get_parameter('landmarks_file').value)
        self._map_yaml = str(self.get_parameter('map_yaml').value)
        self._auto_landmarks_from_map = bool(
            self.get_parameter('auto_landmarks_from_map').value
        )
        self._use_landmark_yaw = bool(self.get_parameter('use_landmark_yaw').value)
        self._min_detection_range_m = float(self.get_parameter('min_detection_range_m').value)
        self._max_detection_range_m = float(self.get_parameter('max_detection_range_m').value)
        self._base_position_stddev_m = float(
            self.get_parameter('base_position_stddev_m').value
        )
        self._base_yaw_stddev_rad = float(self.get_parameter('base_yaw_stddev_rad').value)
        self._position_stddev_per_meter = float(
            self.get_parameter('position_stddev_per_meter').value
        )
        self._yaw_stddev_per_meter = float(self.get_parameter('yaw_stddev_per_meter').value)
        self._tf_lookup_timeout_sec = float(self.get_parameter('tf_lookup_timeout_sec').value)
        self._use_latest_tf_on_failure = bool(
            self.get_parameter('use_latest_tf_on_failure').value
        )
        self._debug_log_matches = bool(self.get_parameter('debug_log_matches').value)

        if self._min_detection_range_m < 0.0:
            self._min_detection_range_m = 0.0
        if self._max_detection_range_m < self._min_detection_range_m:
            self._max_detection_range_m = self._min_detection_range_m
        if self._base_position_stddev_m <= 0.0:
            self._base_position_stddev_m = 0.05
        if self._base_yaw_stddev_rad <= 0.0:
            self._base_yaw_stddev_rad = 0.15
        if self._position_stddev_per_meter < 0.0:
            self._position_stddev_per_meter = 0.0
        if self._yaw_stddev_per_meter < 0.0:
            self._yaw_stddev_per_meter = 0.0
        if self._tf_lookup_timeout_sec <= 0.0:
            self._tf_lookup_timeout_sec = 0.08

        self._landmarks_by_family_id: Dict[tuple[str, int], TagLandmark] = {}
        self._landmarks_by_id: Dict[int, TagLandmark] = {}
        self._resolved_landmarks_file = self._resolve_landmarks_file_path()
        self._load_landmarks(self._resolved_landmarks_file)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, self._output_pose_topic, 10)
        self.create_subscription(
            AprilTagDetectionArray,
            self._detections_topic,
            self._on_detections,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'AprilTag map localization started. '
            f'detections_topic={self._detections_topic}, output_pose_topic={self._output_pose_topic}, '
            f'landmarks_file="{self._resolved_landmarks_file}"'
        )

    def _resolve_landmarks_file_path(self) -> str:
        explicit = self._landmarks_file_param.strip()
        if explicit:
            return explicit

        if not self._auto_landmarks_from_map:
            return ''

        map_yaml = self._map_yaml.strip()
        if not map_yaml.endswith('.yaml'):
            return ''

        return f'{map_yaml[:-5]}.apriltags.yaml'

    def _load_landmarks(self, landmarks_file: str) -> None:
        self._landmarks_by_family_id.clear()
        self._landmarks_by_id.clear()

        if not landmarks_file:
            return

        file_path = Path(landmarks_file).expanduser()
        if not file_path.is_file():
            self.get_logger().warning(f'Landmarks file not found: {file_path}')
            return

        with file_path.open('r', encoding='utf-8') as f:
            loaded = yaml.safe_load(f) or {}

        raw_tags = loaded.get('tags', [])
        if not isinstance(raw_tags, list):
            self.get_logger().warning(
                f'Landmarks file "{file_path}" has invalid "tags" format; expected a list.'
            )
            return

        loaded_count = 0
        for idx, raw_tag in enumerate(raw_tags):
            if not isinstance(raw_tag, dict):
                self.get_logger().warning(f'Skipping non-dict landmark at index {idx}')
                continue

            try:
                family = str(raw_tag.get('family', 'unknown')).strip() or 'unknown'
                tag_id = int(raw_tag['id'])
                position = raw_tag.get('position', {}) or {}
                orientation = raw_tag.get('orientation', {}) or {}
                observations = int(raw_tag.get('observations', 0))

                landmark = TagLandmark(
                    family=family,
                    tag_id=tag_id,
                    x=float(position.get('x', 0.0)),
                    y=float(position.get('y', 0.0)),
                    z=float(position.get('z', 0.0)),
                    qx=float(orientation.get('x', 0.0)),
                    qy=float(orientation.get('y', 0.0)),
                    qz=float(orientation.get('z', 0.0)),
                    qw=float(orientation.get('w', 1.0)),
                    observations=observations,
                )
            except (TypeError, ValueError, KeyError) as exc:
                self.get_logger().warning(f'Skipping invalid landmark at index {idx}: {exc}')
                continue

            self._landmarks_by_family_id[(landmark.family, landmark.tag_id)] = landmark
            current = self._landmarks_by_id.get(landmark.tag_id)
            if current is None or landmark.observations >= current.observations:
                self._landmarks_by_id[landmark.tag_id] = landmark
            loaded_count += 1

        self.get_logger().info(f'Loaded {loaded_count} tag landmarks from {file_path}')

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        if not self._landmarks_by_id and not self._landmarks_by_family_id:
            return

        best_candidate: Optional[CandidatePose] = None
        for detection in msg.detections:
            candidate = self._candidate_from_detection(msg, detection)
            if candidate is None:
                continue

            if best_candidate is None or candidate.distance_m < best_candidate.distance_m:
                best_candidate = candidate

        if best_candidate is None:
            return

        position_stddev = self._base_position_stddev_m + (
            self._position_stddev_per_meter * best_candidate.distance_m
        )
        yaw_stddev = self._base_yaw_stddev_rad + (
            self._yaw_stddev_per_meter * best_candidate.distance_m
        )
        pos_var = position_stddev * position_stddev
        yaw_var = yaw_stddev * yaw_stddev

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = best_candidate.stamp.to_msg()
        pose_msg.header.frame_id = self._map_frame
        pose_msg.pose.pose.position.x = best_candidate.map_to_base_translation[0]
        pose_msg.pose.pose.position.y = best_candidate.map_to_base_translation[1]
        pose_msg.pose.pose.position.z = best_candidate.map_to_base_translation[2]
        pose_msg.pose.pose.orientation.x = best_candidate.map_to_base_quaternion[0]
        pose_msg.pose.pose.orientation.y = best_candidate.map_to_base_quaternion[1]
        pose_msg.pose.pose.orientation.z = best_candidate.map_to_base_quaternion[2]
        pose_msg.pose.pose.orientation.w = best_candidate.map_to_base_quaternion[3]
        pose_msg.pose.covariance[0] = pos_var
        pose_msg.pose.covariance[7] = pos_var
        pose_msg.pose.covariance[14] = 1.0e3
        pose_msg.pose.covariance[21] = 1.0e3
        pose_msg.pose.covariance[28] = 1.0e3
        pose_msg.pose.covariance[35] = yaw_var
        self._pose_pub.publish(pose_msg)

        if self._debug_log_matches:
            self.get_logger().info(
                f'Published map pose from tag {best_candidate.tag_family}:{best_candidate.tag_id} '
                f'(range={best_candidate.distance_m:.2f}m, source_frame={best_candidate.source_frame})'
            )

    def _candidate_from_detection(self, msg: AprilTagDetectionArray, detection) -> Optional[CandidatePose]:
        tag_id = int(detection.id)
        tag_family = str(detection.family).strip()
        landmark = self._lookup_landmark(tag_id=tag_id, tag_family=tag_family)
        if landmark is None:
            return None

        source_frame = detection.pose.header.frame_id or msg.header.frame_id
        if not source_frame:
            return None

        stamp_msg = detection.pose.header.stamp
        if self._is_zero_stamp(stamp_msg):
            stamp_msg = msg.header.stamp
        stamp_time = self._stamp_to_time(stamp_msg)

        pos = detection.pose.pose.pose.position
        det_translation = (float(pos.x), float(pos.y), float(pos.z))
        distance_m = math.sqrt(
            (det_translation[0] * det_translation[0])
            + (det_translation[1] * det_translation[1])
            + (det_translation[2] * det_translation[2])
        )
        if distance_m < self._min_detection_range_m or distance_m > self._max_detection_range_m:
            return None

        det_orientation_msg = detection.pose.pose.pose.orientation
        det_quaternion = self._quat_normalize(
            (
                float(det_orientation_msg.x),
                float(det_orientation_msg.y),
                float(det_orientation_msg.z),
                float(det_orientation_msg.w),
            )
        )
        map_quaternion = self._quat_normalize(
            (landmark.qx, landmark.qy, landmark.qz, landmark.qw)
        )
        if not self._use_landmark_yaw:
            map_quaternion = (0.0, 0.0, 0.0, 1.0)

        timeout = Duration(seconds=self._tf_lookup_timeout_sec)
        base_from_source = None
        try:
            base_from_source = self._tf_buffer.lookup_transform(
                self._base_frame,
                source_frame,
                stamp_time,
                timeout=timeout,
            )
        except TransformException as exc:
            if not self._use_latest_tf_on_failure:
                self.get_logger().debug(
                    f'lookup_transform failed for {source_frame}->{self._base_frame}: {exc}'
                )
                return None
            try:
                base_from_source = self._tf_buffer.lookup_transform(
                    self._base_frame,
                    source_frame,
                    Time(),
                    timeout=timeout,
                )
            except TransformException as latest_exc:
                self.get_logger().debug(
                    f'latest lookup_transform failed for {source_frame}->{self._base_frame}: '
                    f'{latest_exc}'
                )
                return None

        tf_t = base_from_source.transform.translation
        tf_q = base_from_source.transform.rotation
        base_source_translation = (float(tf_t.x), float(tf_t.y), float(tf_t.z))
        base_source_quaternion = self._quat_normalize(
            (float(tf_q.x), float(tf_q.y), float(tf_q.z), float(tf_q.w))
        )

        base_tag_translation, base_tag_quaternion = self._compose_transform(
            base_source_translation,
            base_source_quaternion,
            det_translation,
            det_quaternion,
        )
        tag_base_translation, tag_base_quaternion = self._inverse_transform(
            base_tag_translation,
            base_tag_quaternion,
        )
        map_base_translation, map_base_quaternion = self._compose_transform(
            (landmark.x, landmark.y, landmark.z),
            map_quaternion,
            tag_base_translation,
            tag_base_quaternion,
        )

        return CandidatePose(
            map_to_base_translation=map_base_translation,
            map_to_base_quaternion=map_base_quaternion,
            distance_m=distance_m,
            stamp=stamp_time if not self._is_zero_time(stamp_time) else self.get_clock().now(),
            source_frame=source_frame,
            tag_family=tag_family if tag_family else landmark.family,
            tag_id=tag_id,
        )

    def _lookup_landmark(self, tag_id: int, tag_family: str) -> Optional[TagLandmark]:
        if tag_family:
            landmark = self._landmarks_by_family_id.get((tag_family, tag_id))
            if landmark is not None:
                return landmark
        return self._landmarks_by_id.get(tag_id)

    @staticmethod
    def _stamp_to_time(stamp_msg) -> Time:
        if AprilTagMapLocalizationNode._is_zero_stamp(stamp_msg):
            return Time()
        return Time.from_msg(stamp_msg)

    @staticmethod
    def _is_zero_stamp(stamp_msg) -> bool:
        return int(stamp_msg.sec) == 0 and int(stamp_msg.nanosec) == 0

    @staticmethod
    def _is_zero_time(time_obj: Time) -> bool:
        return int(time_obj.nanoseconds) == 0

    @staticmethod
    def _quat_normalize(quat: QuaternionTuple) -> QuaternionTuple:
        qx, qy, qz, qw = quat
        norm = (qx * qx) + (qy * qy) + (qz * qz) + (qw * qw)
        if norm <= 1.0e-12:
            return (0.0, 0.0, 0.0, 1.0)
        inv = 1.0 / math.sqrt(norm)
        return (qx * inv, qy * inv, qz * inv, qw * inv)

    @staticmethod
    def _quat_multiply(q1: QuaternionTuple, q2: QuaternionTuple) -> QuaternionTuple:
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (
            (w1 * x2) + (x1 * w2) + (y1 * z2) - (z1 * y2),
            (w1 * y2) - (x1 * z2) + (y1 * w2) + (z1 * x2),
            (w1 * z2) + (x1 * y2) - (y1 * x2) + (z1 * w2),
            (w1 * w2) - (x1 * x2) - (y1 * y2) - (z1 * z2),
        )

    @staticmethod
    def _quat_conjugate(quat: QuaternionTuple) -> QuaternionTuple:
        x, y, z, w = quat
        return (-x, -y, -z, w)

    def _quat_inverse(self, quat: QuaternionTuple) -> QuaternionTuple:
        return self._quat_normalize(self._quat_conjugate(quat))

    def _quat_rotate_vector(self, quat: QuaternionTuple, vector: Vector3Tuple) -> Vector3Tuple:
        vx, vy, vz = vector
        vec_quat: QuaternionTuple = (vx, vy, vz, 0.0)
        q_norm = self._quat_normalize(quat)
        q_inv = self._quat_inverse(q_norm)
        rotated = self._quat_multiply(self._quat_multiply(q_norm, vec_quat), q_inv)
        return (rotated[0], rotated[1], rotated[2])

    def _compose_transform(
        self,
        t_ab: Vector3Tuple,
        q_ab: QuaternionTuple,
        t_bc: Vector3Tuple,
        q_bc: QuaternionTuple,
    ) -> Tuple[Vector3Tuple, QuaternionTuple]:
        rotated_t_bc = self._quat_rotate_vector(q_ab, t_bc)
        t_ac: Vector3Tuple = (
            t_ab[0] + rotated_t_bc[0],
            t_ab[1] + rotated_t_bc[1],
            t_ab[2] + rotated_t_bc[2],
        )
        q_ac = self._quat_normalize(self._quat_multiply(q_ab, q_bc))
        return (t_ac, q_ac)

    def _inverse_transform(
        self,
        translation: Vector3Tuple,
        quaternion: QuaternionTuple,
    ) -> Tuple[Vector3Tuple, QuaternionTuple]:
        q_inv = self._quat_inverse(quaternion)
        neg_translation = (-translation[0], -translation[1], -translation[2])
        t_inv = self._quat_rotate_vector(q_inv, neg_translation)
        return (t_inv, q_inv)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AprilTagMapLocalizationNode()
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
