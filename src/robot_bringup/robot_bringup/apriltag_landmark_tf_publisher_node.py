#!/usr/bin/env python3

from dataclasses import dataclass
from pathlib import Path
import re
from typing import Dict, Tuple

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import yaml


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


class AprilTagLandmarkTFPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__('apriltag_landmark_tf_publisher')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('landmarks_file', '')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('auto_landmarks_from_map', True)
        self.declare_parameter('child_frame_prefix', 'apriltag_landmark')
        self.declare_parameter('publish_landmarks_topic', True)
        self.declare_parameter('landmarks_topic', '/apriltag/landmarks')
        self.declare_parameter('landmarks_publish_period_sec', 1.0)

        self._map_frame = str(self.get_parameter('map_frame').value)
        self._landmarks_file_param = str(self.get_parameter('landmarks_file').value)
        self._map_yaml = str(self.get_parameter('map_yaml').value)
        self._auto_landmarks_from_map = bool(
            self.get_parameter('auto_landmarks_from_map').value
        )
        self._child_frame_prefix = str(self.get_parameter('child_frame_prefix').value).strip('/')
        self._publish_landmarks_topic = bool(
            self.get_parameter('publish_landmarks_topic').value
        )
        self._landmarks_topic = str(self.get_parameter('landmarks_topic').value).strip()
        self._landmarks_publish_period_sec = float(
            self.get_parameter('landmarks_publish_period_sec').value
        )

        self._landmarks_by_family_id: Dict[Tuple[str, int], TagLandmark] = {}
        self._landmarks_by_id: Dict[int, TagLandmark] = {}
        self._resolved_landmarks_file = self._resolve_landmarks_file_path()
        self._load_landmarks(self._resolved_landmarks_file)

        self._landmarks_pub = None
        self._landmarks_timer = None
        if self._publish_landmarks_topic and self._landmarks_topic:
            qos = QoSProfile(depth=1)
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self._landmarks_pub = self.create_publisher(PoseArray, self._landmarks_topic, qos)

        self._tf_broadcaster = StaticTransformBroadcaster(self)
        transforms = self._build_transforms()
        now = self.get_clock().now().to_msg()
        if transforms:
            self._tf_broadcaster.sendTransform(transforms)
            self._publish_landmarks_pose_array(stamp=now)
            self.get_logger().info(
                f'Published {len(transforms)} static landmark transforms from '
                f'"{self._resolved_landmarks_file}"'
            )
        else:
            self._publish_landmarks_pose_array(stamp=now)
            self.get_logger().warning(
                f'No landmark transforms were published. '
                f'landmarks_file="{self._resolved_landmarks_file}"'
            )

        if (
            self._landmarks_pub is not None
            and self._landmarks_publish_period_sec > 0.0
            and self._landmarks_by_family_id
        ):
            self._landmarks_timer = self.create_timer(
                self._landmarks_publish_period_sec,
                self._on_landmarks_timer,
            )
            self.get_logger().info(
                f'Publishing /apriltag/landmarks every '
                f'{self._landmarks_publish_period_sec:.2f}s'
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

        self.get_logger().info(
            f'Loaded {len(self._landmarks_by_family_id)} landmark entries from {file_path}'
        )

    def _publish_landmarks_pose_array(self, stamp=None) -> None:
        if self._landmarks_pub is None:
            return

        if stamp is None:
            stamp = self.get_clock().now().to_msg()

        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = self._map_frame
        for landmark in self._landmarks_by_family_id.values():
            pose = Pose()
            pose.position.x = landmark.x
            pose.position.y = landmark.y
            pose.position.z = landmark.z
            qx, qy, qz, qw = self._quat_normalize(
                (landmark.qx, landmark.qy, landmark.qz, landmark.qw)
            )
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            pose_array.poses.append(pose)
        self._landmarks_pub.publish(pose_array)

    def _on_landmarks_timer(self) -> None:
        self._publish_landmarks_pose_array()

    def _build_transforms(self) -> list[TransformStamped]:
        transforms: list[TransformStamped] = []
        now = self.get_clock().now().to_msg()

        for landmark in self._landmarks_by_family_id.values():
            frame_suffix = self._sanitize_frame_component(f'{landmark.family}_{landmark.tag_id}')
            child_frame = (
                f'{self._child_frame_prefix}/{frame_suffix}'
                if self._child_frame_prefix
                else frame_suffix
            )

            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = self._map_frame
            tf_msg.child_frame_id = child_frame
            tf_msg.transform.translation.x = landmark.x
            tf_msg.transform.translation.y = landmark.y
            tf_msg.transform.translation.z = landmark.z

            qx, qy, qz, qw = self._quat_normalize(
                (landmark.qx, landmark.qy, landmark.qz, landmark.qw)
            )
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            transforms.append(tf_msg)

        return transforms

    @staticmethod
    def _sanitize_frame_component(raw: str) -> str:
        cleaned = re.sub(r'[^A-Za-z0-9_]+', '_', raw)
        cleaned = re.sub(r'_+', '_', cleaned).strip('_')
        return cleaned or 'tag'

    @staticmethod
    def _quat_normalize(quat: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        qx, qy, qz, qw = quat
        norm = (qx * qx) + (qy * qy) + (qz * qz) + (qw * qw)
        if norm <= 1.0e-12:
            return (0.0, 0.0, 0.0, 1.0)
        inv = 1.0 / (norm ** 0.5)
        return (qx * inv, qy * inv, qz * inv, qw * inv)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AprilTagLandmarkTFPublisherNode()
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
