#!/usr/bin/env python3

import math
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


def quaternion_to_pitch(x: float, y: float, z: float, w: float) -> float:
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        return math.copysign(math.pi / 2.0, sinp)
    return math.asin(sinp)


class DropDetector(Node):
    def __init__(self) -> None:
        super().__init__("drop_detector")

        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("odometry_topic", "/visual_slam/tracking/odometry")
        self.declare_parameter("drop_topic", "/drop_detected")

        self.declare_parameter("roi_bottom_fraction", 0.55)
        self.declare_parameter("roi_center_width_fraction", 0.40)
        self.declare_parameter("min_depth_m", 0.10)
        self.declare_parameter("max_depth_m", 5.00)
        self.declare_parameter("min_valid_ratio", 0.25)
        self.declare_parameter("valid_ratio_hysteresis", 0.10)
        self.declare_parameter("drop_depth_threshold_m", 1.40)
        self.declare_parameter("depth_hysteresis_m", 0.20)
        self.declare_parameter("pitch_compensation_gain", 0.25)
        self.declare_parameter("consecutive_trigger_frames", 3)
        self.declare_parameter("consecutive_clear_frames", 5)

        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.odometry_topic = str(self.get_parameter("odometry_topic").value)
        self.drop_topic = str(self.get_parameter("drop_topic").value)

        self.roi_bottom_fraction = float(self.get_parameter("roi_bottom_fraction").value)
        self.roi_center_width_fraction = float(self.get_parameter("roi_center_width_fraction").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.min_valid_ratio = float(self.get_parameter("min_valid_ratio").value)
        self.valid_ratio_hysteresis = float(self.get_parameter("valid_ratio_hysteresis").value)
        self.drop_depth_threshold_m = float(self.get_parameter("drop_depth_threshold_m").value)
        self.depth_hysteresis_m = float(self.get_parameter("depth_hysteresis_m").value)
        self.pitch_compensation_gain = float(self.get_parameter("pitch_compensation_gain").value)
        self.consecutive_trigger_frames = int(
            self.get_parameter("consecutive_trigger_frames").value
        )
        self.consecutive_clear_frames = int(self.get_parameter("consecutive_clear_frames").value)

        self.current_pitch_rad = 0.0
        self.drop_detected = False
        self.trigger_count = 0
        self.clear_count = 0

        self.drop_pub = self.create_publisher(Bool, self.drop_topic, 10)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.on_depth, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odometry_topic, self.on_odom, 10)

        self.get_logger().info(
            f"DropDetector depth={self.depth_topic} odom={self.odometry_topic} out={self.drop_topic}"
        )

    def on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.current_pitch_rad = quaternion_to_pitch(q.x, q.y, q.z, q.w)

    def on_depth(self, msg: Image) -> None:
        depth = self.decode_depth(msg)
        if depth is None:
            return

        h, w = depth.shape
        roi_y0 = int(max(0, min(h - 1, h * self.roi_bottom_fraction)))
        roi_half_w = int(max(1, w * self.roi_center_width_fraction * 0.5))
        center_x = w // 2
        roi_x0 = max(0, center_x - roi_half_w)
        roi_x1 = min(w, center_x + roi_half_w)

        roi = depth[roi_y0:h, roi_x0:roi_x1]
        if roi.size == 0:
            return

        valid = np.isfinite(roi) & (roi >= self.min_depth_m) & (roi <= self.max_depth_m)
        valid_ratio = float(np.count_nonzero(valid)) / float(roi.size)
        median_depth = float(np.median(roi[valid])) if np.any(valid) else float("inf")

        # Positive pitch means nose-up, so allow a slightly larger depth threshold.
        pitch_adjust = self.pitch_compensation_gain * self.current_pitch_rad
        depth_set_thresh = max(0.2, self.drop_depth_threshold_m + pitch_adjust)
        depth_clear_thresh = max(0.2, depth_set_thresh - self.depth_hysteresis_m)
        valid_clear_thresh = min(1.0, self.min_valid_ratio + self.valid_ratio_hysteresis)

        hazard_candidate = (valid_ratio < self.min_valid_ratio) or (median_depth > depth_set_thresh)
        clear_candidate = (valid_ratio > valid_clear_thresh) and (median_depth < depth_clear_thresh)

        if not self.drop_detected:
            if hazard_candidate:
                self.trigger_count += 1
                self.clear_count = 0
            else:
                self.trigger_count = 0
            if self.trigger_count >= self.consecutive_trigger_frames:
                self.drop_detected = True
                self.get_logger().warn(
                    "DROP DETECTED valid_ratio=%.3f median_depth=%.3f pitch=%.3f"
                    % (valid_ratio, median_depth, self.current_pitch_rad)
                )
        else:
            if clear_candidate:
                self.clear_count += 1
                self.trigger_count = 0
            else:
                self.clear_count = 0
            if self.clear_count >= self.consecutive_clear_frames:
                self.drop_detected = False
                self.get_logger().info(
                    "Drop cleared valid_ratio=%.3f median_depth=%.3f pitch=%.3f"
                    % (valid_ratio, median_depth, self.current_pitch_rad)
                )

        self.drop_pub.publish(Bool(data=self.drop_detected))

    def decode_depth(self, msg: Image) -> Optional[np.ndarray]:
        if msg.width == 0 or msg.height == 0 or not msg.data:
            return None

        if msg.encoding == "16UC1":
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            depth_m = arr.astype(np.float32) * 0.001
            depth_m[arr == 0] = np.nan
            return depth_m

        if msg.encoding == "32FC1":
            depth_m = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            return depth_m

        self.get_logger().warn(
            f"Unsupported depth encoding '{msg.encoding}'. Expected 16UC1 or 32FC1."
        )
        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DropDetector()
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
