// Copyright (c) 2024 Bluebot
// SPDX-License-Identifier: Apache-2.0

#include "bluebot_bt_plugins/dock_robot_by_id.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace bluebot_bt_plugins
{

DockRobotById::DockRobotById(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{}

BT::PortsList DockRobotById::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<std::string>(
      "dock_id", "", "Dock ID key from dock_database.yaml (e.g. \"25\" or \"26\")"),
    BT::InputPort<bool>(
      "navigate_to_staging_pose", true,
      "Have Nav2 drive to the dock staging pose before docking"),
    BT::InputPort<float>(
      "max_staging_time", 1000.0f,
      "Max seconds allowed to navigate to the staging pose"),

    BT::OutputPort<bool>("success", "True when docking completed successfully"),
    BT::OutputPort<uint16_t>("error_code", "Error code returned by the docking server"),
    BT::OutputPort<uint16_t>("num_retries", "Number of docking attempts made"),
  });
}

void DockRobotById::on_tick()
{
  std::string dock_id;
  getInput("dock_id", dock_id);
  if (dock_id.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
      "[DockRobotById] dock_id input port is empty — aborting dock goal.");
    return;
  }

  bool navigate_to_staging;
  float max_staging_time;
  getInput("navigate_to_staging_pose", navigate_to_staging);
  getInput("max_staging_time", max_staging_time);

  goal_.use_dock_id = true;
  goal_.dock_id = dock_id;
  goal_.navigate_to_staging_pose = navigate_to_staging;
  goal_.max_staging_time = max_staging_time;

  RCLCPP_INFO(node_->get_logger(),
    "[DockRobotById] Sending dock goal — id: '%s'  navigate_to_staging: %s",
    dock_id.c_str(), navigate_to_staging ? "true" : "false");
}

BT::NodeStatus DockRobotById::on_success()
{
  setOutput("success", result_.result->success);
  setOutput("error_code", result_.result->error_code);
  setOutput("num_retries", result_.result->num_retries);

  RCLCPP_INFO(node_->get_logger(),
    "[DockRobotById] Docking succeeded (retries: %u).",
    result_.result->num_retries);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus DockRobotById::on_aborted()
{
  setOutput("success", false);
  if (result_.result) {
    setOutput("error_code", result_.result->error_code);
    setOutput("num_retries", result_.result->num_retries);
    RCLCPP_WARN(node_->get_logger(),
      "[DockRobotById] Docking aborted — error_code: %u  retries: %u",
      result_.result->error_code, result_.result->num_retries);
  } else {
    RCLCPP_WARN(node_->get_logger(), "[DockRobotById] Docking aborted (no result).");
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus DockRobotById::on_cancelled()
{
  RCLCPP_INFO(node_->get_logger(), "[DockRobotById] Docking cancelled.");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bluebot_bt_plugins

// ── Plugin registration ───────────────────────────────────────────────────────
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<bluebot_bt_plugins::DockRobotById>(
        name, "dock_robot", config);
    };

  factory.registerBuilder<bluebot_bt_plugins::DockRobotById>(
    "DockRobotById", builder);
}
