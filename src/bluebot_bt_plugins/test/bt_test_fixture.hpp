// Copyright (c) 2024 Bluebot
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/blackboard.h"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

namespace bluebot_bt_plugins::test
{

class BtTestFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = rclcpp::Node::make_shared(
      "bt_test_node_" + std::to_string(counter_++));

    blackboard_ = BT::Blackboard::create();
    blackboard_->set<rclcpp::Node::SharedPtr>("node", node_);
    blackboard_->set<std::chrono::milliseconds>(
      "server_timeout", std::chrono::milliseconds(2000));
    blackboard_->set<std::chrono::milliseconds>(
      "bt_loop_duration", std::chrono::milliseconds(10));
  }

  void TearDown() override
  {
    tree_.reset();
    node_.reset();
  }

  void buildTree(const std::string & xml)
  {
    tree_ = std::make_unique<BT::Tree>(
      factory_.createTreeFromText(xml, blackboard_));
  }

  BT::NodeStatus tickUntilDone(int max_ticks = 200)
  {
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    for (int i = 0; i < max_ticks && status == BT::NodeStatus::RUNNING; ++i) {
      status = tree_->tickRoot();
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return status;
  }

  rclcpp::Node::SharedPtr   node_;
  BT::Blackboard::Ptr       blackboard_;
  BT::BehaviorTreeFactory   factory_;
  std::unique_ptr<BT::Tree> tree_;

private:
  static inline int counter_ = 0;
};

class ROS2Environment : public ::testing::Environment
{
public:
  void SetUp()    override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

}  // namespace bluebot_bt_plugins::test
