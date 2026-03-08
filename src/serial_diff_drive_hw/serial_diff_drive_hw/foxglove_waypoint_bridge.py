#!/usr/bin/env python3

from typing import Dict

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class FoxgloveWaypointBridge(Node):
    STATUS_LABELS: Dict[int, str] = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
    }

    def __init__(self) -> None:
        super().__init__("foxglove_waypoint_bridge")

        self.declare_parameter("input_topic", "/foxglove/waypoints")
        self.declare_parameter("action_name", "/navigate_through_poses")
        self.declare_parameter("default_frame_id", "map")
        self.declare_parameter("status_topic", "/foxglove/waypoints/status")
        self.declare_parameter("wait_for_server_sec", 2.0)
        self.declare_parameter("feedback_log_period_sec", 1.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.action_name = str(self.get_parameter("action_name").value)
        self.default_frame_id = str(self.get_parameter("default_frame_id").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.wait_for_server_sec = float(self.get_parameter("wait_for_server_sec").value)
        self.feedback_log_period_ns = int(
            float(self.get_parameter("feedback_log_period_sec").value) * 1e9
        )

        self.action_client = ActionClient(self, NavigateThroughPoses, self.action_name)
        self.goal_count = 0
        self.last_feedback_log_ns = 0

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.input_sub = self.create_subscription(PoseArray, self.input_topic, self.on_waypoints, 10)

        self.publish_status(
            f"ready: publish geometry_msgs/PoseArray to {self.input_topic}"
        )
        self.get_logger().info(
            "FoxgloveWaypointBridge input=%s action=%s default_frame=%s status=%s"
            % (self.input_topic, self.action_name, self.default_frame_id, self.status_topic)
        )

    def publish_status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))

    def on_waypoints(self, msg: PoseArray) -> None:
        if len(msg.poses) == 0:
            text = "ignored empty waypoint list"
            self.get_logger().warn(text)
            self.publish_status(text)
            return

        if not self.action_client.wait_for_server(timeout_sec=self.wait_for_server_sec):
            text = f"action server unavailable: {self.action_name}"
            self.get_logger().error(text)
            self.publish_status(text)
            return

        frame_id = msg.header.frame_id.strip() if msg.header.frame_id else ""
        if not frame_id:
            frame_id = self.default_frame_id

        goal_msg = NavigateThroughPoses.Goal()
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        for pose in msg.poses:
            stamped = PoseStamped()
            stamped.header.frame_id = frame_id
            stamped.header.stamp = stamp
            stamped.pose = pose
            goal_msg.poses.append(stamped)

        self.goal_count += 1
        text = f"sending goal #{self.goal_count} with {len(goal_msg.poses)} waypoints in frame '{frame_id}'"
        self.get_logger().info(text)
        self.publish_status(text)

        send_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.on_feedback
        )
        send_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            text = "waypoint goal rejected"
            self.get_logger().warn(text)
            self.publish_status(text)
            return

        self.get_logger().info("waypoint goal accepted")
        self.publish_status("waypoint goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_feedback(self, feedback_msg) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self.feedback_log_period_ns > 0 and (
            now_ns - self.last_feedback_log_ns < self.feedback_log_period_ns
        ):
            return
        self.last_feedback_log_ns = now_ns

        feedback = feedback_msg.feedback
        parts = []
        if hasattr(feedback, "number_of_poses_remaining"):
            parts.append(f"remaining={feedback.number_of_poses_remaining}")
        if hasattr(feedback, "distance_remaining"):
            parts.append(f"distance={feedback.distance_remaining:.2f}m")
        if hasattr(feedback, "number_of_recoveries"):
            parts.append(f"recoveries={feedback.number_of_recoveries}")
        if hasattr(feedback, "navigation_time"):
            nav_time_sec = feedback.navigation_time.sec + feedback.navigation_time.nanosec / 1e9
            parts.append(f"time={nav_time_sec:.1f}s")

        text = "feedback: " + (", ".join(parts) if parts else "active")
        self.get_logger().info(text)
        self.publish_status(text)

    def on_result(self, future) -> None:
        wrapped_result = future.result()
        status = int(wrapped_result.status)
        status_label = self.STATUS_LABELS.get(status, str(status))
        result = wrapped_result.result

        error_code = getattr(result, "error_code", None)
        error_msg = getattr(result, "error_msg", "")

        if status == GoalStatus.STATUS_SUCCEEDED and (error_code is None or int(error_code) == 0):
            text = "waypoint goal succeeded"
            self.get_logger().info(text)
            self.publish_status(text)
            return

        text = f"waypoint goal finished with status={status_label}"
        if error_code is not None:
            text += f" error_code={int(error_code)}"
        if error_msg:
            text += f" error_msg={error_msg}"
        self.get_logger().warn(text)
        self.publish_status(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FoxgloveWaypointBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
