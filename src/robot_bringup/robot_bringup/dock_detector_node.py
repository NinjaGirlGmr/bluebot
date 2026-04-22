#!/usr/bin/env python3
# Copyright (c) 2024 Bluebot
# SPDX-License-Identifier: Apache-2.0

"""
dock_detector_node — bridges AprilTag detections to opennav_docking.

Subscribes to /tag_detections and publishes PoseStamped to
/detected_dock_pose for any tag whose ID is in the dock_tag_ids list.
The docking server (SimpleChargingDock, use_external_detection_pose=true)
consumes this topic during the INITIAL_PERCEPTION and CONTROLLING phases.

Only the best (first) matching detection per frame is published.
"""

from geometry_msgs.msg import PoseStamped
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class DockDetectorNode(Node):
    def __init__(self):
        super().__init__('dock_detector_node')

        self.declare_parameter('detections_topic', '/tag_detections')
        self.declare_parameter('detected_dock_pose_topic', '/detected_dock_pose')
        self.declare_parameter('dock_tag_ids', [9, 25])
        self.declare_parameter('dock_tag_family', 'tag36h11')
        # Minimum distance from camera to dock tag (metres). Detections closer
        # than this are treated as zero-pose pipeline artefacts and dropped.
        self.declare_parameter('min_detection_distance', 0.05)

        detections_topic = self.get_parameter(
            'detections_topic').get_parameter_value().string_value
        dock_pose_topic = self.get_parameter(
            'detected_dock_pose_topic').get_parameter_value().string_value
        self._dock_tag_ids = set(
            self.get_parameter('dock_tag_ids').get_parameter_value().integer_array_value)
        self._dock_tag_family = self.get_parameter(
            'dock_tag_family').get_parameter_value().string_value
        self._min_dist = self.get_parameter(
            'min_detection_distance').get_parameter_value().double_value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(PoseStamped, dock_pose_topic, qos)
        self._sub = self.create_subscription(
            AprilTagDetectionArray,
            detections_topic,
            self._on_detections,
            qos,
        )

        self.get_logger().info(
            f'[dock_detector] Listening on {detections_topic} → {dock_pose_topic}  '
            f'dock tag IDs: {sorted(self._dock_tag_ids)}'
        )

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        for detection in msg.detections:
            if detection.family != self._dock_tag_family:
                continue
            if detection.id not in self._dock_tag_ids:
                continue
            p = detection.pose.pose.pose.position
            dist = math.sqrt(p.x ** 2 + p.y ** 2 + p.z ** 2)
            if dist < self._min_dist:
                self.get_logger().warn(
                    f'[dock_detector] Dropping zero-pose detection for tag {detection.id} '
                    f'(dist={dist:.4f} m < min={self._min_dist} m)',
                    throttle_duration_sec=2.0,
                )
                continue
            out = PoseStamped()
            out.header = detection.pose.header
            out.pose = detection.pose.pose.pose
            self._pub.publish(out)
            return  # publish first matching detection only


def main(args=None):
    rclpy.init(args=args)
    node = DockDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
