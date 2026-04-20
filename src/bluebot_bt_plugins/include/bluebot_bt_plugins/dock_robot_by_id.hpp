// Copyright (c) 2024 Bluebot
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <string>
#include <memory>

#include "opennav_docking_msgs/action/dock_robot.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace bluebot_bt_plugins
{

/**
 * BT Action node that drives the robot to a named dock and docks it.
 *
 * Input ports:
 *   - dock_id            (string):  Key from dock_database.yaml (e.g. "25", "26")
 *   - navigate_to_staging (bool):   Let Nav2 navigate to staging pose first (default true)
 *   - max_staging_time   (float):   Seconds allowed for staging navigation (default 1000)
 *
 * Output ports:
 *   - success    (bool):    True if docking completed successfully
 *   - error_code (uint16):  Error code from the docking server
 *   - num_retries (uint16): How many dock attempts were made
 */
class DockRobotById
  : public nav2_behavior_tree::BtActionNode<
      opennav_docking_msgs::action::DockRobot>
{
public:
  using Action = opennav_docking_msgs::action::DockRobot;

  DockRobotById(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts();
};

}  // namespace bluebot_bt_plugins
