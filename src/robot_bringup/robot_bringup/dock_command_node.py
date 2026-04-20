#!/usr/bin/env python3
# Copyright (c) 2024 Bluebot
# SPDX-License-Identifier: Apache-2.0

"""
dock_command_node — external docking trigger.

Subscribes to /dock_command (std_msgs/String).
Payload is a dock ID string matching a key in dock_database.yaml,
e.g. "25" or "26".

On receipt, sends a DockRobot action goal to /dock_robot with:
  - use_dock_id = True
  - navigate_to_staging_pose = True
  - dock_id = <payload>

A new goal is rejected while a previous one is still active.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from opennav_docking_msgs.action import DockRobot
from std_msgs.msg import String


class DockCommandNode(Node):
    def __init__(self):
        super().__init__('dock_command_node')

        self.declare_parameter('dock_command_topic', '/dock_command')
        self.declare_parameter('dock_action_server', 'dock_robot')
        self.declare_parameter('navigate_to_staging_pose', True)
        self.declare_parameter('max_staging_time', 120.0)

        topic = self.get_parameter('dock_command_topic').get_parameter_value().string_value
        action_server = self.get_parameter('dock_action_server').get_parameter_value().string_value

        self._action_client = ActionClient(self, DockRobot, action_server)
        self._active_goal = None

        self._sub = self.create_subscription(String, topic, self._on_command, 10)
        self.get_logger().info(
            f'[dock_command_node] Listening on {topic}, '
            f'sending goals to {action_server}')

    def _on_command(self, msg: String):
        dock_id = msg.data.strip()
        if not dock_id:
            self.get_logger().warn('[dock_command_node] Received empty dock_id — ignoring.')
            return

        if self._active_goal is not None:
            self.get_logger().warn(
                f'[dock_command_node] Dock goal already active — ignoring request for "{dock_id}".')
            return

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                '[dock_command_node] dock_robot action server not available.')
            return

        navigate = self.get_parameter('navigate_to_staging_pose').get_parameter_value().bool_value
        max_staging = self.get_parameter('max_staging_time').get_parameter_value().double_value

        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = dock_id
        goal.navigate_to_staging_pose = navigate
        goal.max_staging_time = float(max_staging)

        self.get_logger().info(
            f'[dock_command_node] Sending dock goal — id: "{dock_id}"  '
            f'navigate_to_staging: {navigate}')

        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('[dock_command_node] Goal rejected by docking server.')
            self._active_goal = None
            return

        self.get_logger().info('[dock_command_node] Goal accepted.')
        self._active_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        pass  # could log docking state machine phase here if needed

    def _on_result(self, future):
        result = future.result().result
        status = future.result().status

        import action_msgs.msg as action_msgs
        if status == action_msgs.GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'[dock_command_node] Docking succeeded '
                f'(retries: {result.num_retries}).')
        else:
            self.get_logger().warn(
                f'[dock_command_node] Docking did not succeed — '
                f'status: {status}  error_code: {result.error_code}  '
                f'retries: {result.num_retries}.')

        self._active_goal = None


def main(args=None):
    rclpy.init(args=args)
    node = DockCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
