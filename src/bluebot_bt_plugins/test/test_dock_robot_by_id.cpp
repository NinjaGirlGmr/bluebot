// Copyright (c) 2024 Bluebot
// SPDX-License-Identifier: Apache-2.0

#include "bt_test_fixture.hpp"
#include "bluebot_bt_plugins/dock_robot_by_id.hpp"

#include <memory>
#include <string>

#include "opennav_docking_msgs/action/dock_robot.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace bluebot_bt_plugins::test
{

using Action = opennav_docking_msgs::action::DockRobot;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

// ── Fake docking action server ─────────────────────────────────────────────
class FakeDockingServer
{
public:
  explicit FakeDockingServer(rclcpp::Node::SharedPtr node, bool should_abort = false)
  : should_abort_(should_abort)
  {
    server_ = rclcpp_action::create_server<Action>(
      node, "dock_robot",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandle> gh) {
        call_count_++;
        last_dock_id_ = gh->get_goal()->dock_id;
        last_use_dock_id_ = gh->get_goal()->use_dock_id;
        last_navigate_to_staging_ = gh->get_goal()->navigate_to_staging_pose;

        auto result = std::make_shared<Action::Result>();
        result->success = !should_abort_;
        result->error_code = should_abort_ ? 1u : 0u;
        result->num_retries = 0u;

        if (should_abort_) {
          gh->abort(result);
        } else {
          gh->succeed(result);
        }
      });
  }

  int  call_count_{0};
  std::string last_dock_id_;
  bool last_use_dock_id_{false};
  bool last_navigate_to_staging_{false};

private:
  bool should_abort_;
  rclcpp_action::Server<Action>::SharedPtr server_;
};

// ── Test fixture ───────────────────────────────────────────────────────────
class DockRobotByIdTest : public BtTestFixture
{
protected:
  void SetUp() override
  {
    BtTestFixture::SetUp();
    factory_.registerBuilder<bluebot_bt_plugins::DockRobotById>(
      "DockRobotById",
      [](const std::string & name, const BT::NodeConfiguration & cfg) {
        return std::make_unique<bluebot_bt_plugins::DockRobotById>(
          name, "dock_robot", cfg);
      });
  }

  static constexpr const char * DOCK_ID = "25";

  std::string makeXml(
    const std::string & dock_id,
    bool navigate = true,
    float max_staging_time = 1000.0f)
  {
    return std::string(R"(
      <root main_tree_to_execute="T">
        <BehaviorTree ID="T">
          <DockRobotById
            action_name="dock_robot"
            server_timeout="2000"
            dock_id=")") + dock_id + R"("
            navigate_to_staging_pose=")" + (navigate ? "true" : "false") + R"("
            max_staging_time=")" + std::to_string(max_staging_time) + R"("
            success="{success}"
            error_code="{error_code}"
            num_retries="{num_retries}"/>
        </BehaviorTree>
      </root>)";
  }
};

// ── Tests ──────────────────────────────────────────────────────────────────

TEST_F(DockRobotByIdTest, HappyPath_ReturnsSuccess)
{
  auto server = std::make_shared<FakeDockingServer>(node_);
  buildTree(makeXml(DOCK_ID));
  EXPECT_EQ(tickUntilDone(), BT::NodeStatus::SUCCESS);
}

TEST_F(DockRobotByIdTest, GoalPopulatedFromInputPorts)
{
  auto server = std::make_shared<FakeDockingServer>(node_);
  buildTree(makeXml(DOCK_ID, true));
  tickUntilDone();

  EXPECT_EQ(server->call_count_, 1);
  EXPECT_EQ(server->last_dock_id_, DOCK_ID);
  EXPECT_TRUE(server->last_use_dock_id_);
  EXPECT_TRUE(server->last_navigate_to_staging_);
}

TEST_F(DockRobotByIdTest, NavigateToStagingFalse_PropagatedToGoal)
{
  auto server = std::make_shared<FakeDockingServer>(node_);
  buildTree(makeXml(DOCK_ID, false));
  tickUntilDone();

  EXPECT_FALSE(server->last_navigate_to_staging_);
}

TEST_F(DockRobotByIdTest, OutputPortsSetOnSuccess)
{
  auto server = std::make_shared<FakeDockingServer>(node_);
  buildTree(makeXml(DOCK_ID));
  tickUntilDone();

  bool success = false;
  ASSERT_NO_THROW(success = blackboard_->get<bool>("success"));
  EXPECT_TRUE(success);

  uint16_t error_code = 99u;
  ASSERT_NO_THROW(error_code = blackboard_->get<uint16_t>("error_code"));
  EXPECT_EQ(error_code, 0u);
}

TEST_F(DockRobotByIdTest, ReturnsFailureWhenAborted)
{
  auto server = std::make_shared<FakeDockingServer>(node_, /*should_abort=*/true);
  buildTree(makeXml(DOCK_ID));
  EXPECT_EQ(tickUntilDone(), BT::NodeStatus::FAILURE);

  bool success = true;
  ASSERT_NO_THROW(success = blackboard_->get<bool>("success"));
  EXPECT_FALSE(success);
}

TEST_F(DockRobotByIdTest, FailsWhenActionServerUnavailable)
{
  // No fake server — action server is absent
  buildTree(R"(
    <root main_tree_to_execute="T">
      <BehaviorTree ID="T">
        <DockRobotById
          action_name="dock_robot"
          server_timeout="300"
          dock_id="25"/>
      </BehaviorTree>
    </root>)");

  EXPECT_EQ(tickUntilDone(100), BT::NodeStatus::FAILURE);
}

TEST_F(DockRobotByIdTest, AlternateDockId_SentCorrectly)
{
  auto server = std::make_shared<FakeDockingServer>(node_);
  buildTree(makeXml("26"));
  tickUntilDone();

  EXPECT_EQ(server->last_dock_id_, "26");
}

}  // namespace bluebot_bt_plugins::test
