#include <gtest/gtest.h>

#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "../src/safety_monitor_node.cpp"

namespace {

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SafetyMonitorNodeShellTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) rclcpp::shutdown();
  }
};

TEST_F(SafetyMonitorNodeShellTest, ConfigureActivateAndCleanupResources) {
  auto node = std::make_shared<adas::system::SafetyMonitorNode>(rclcpp::NodeOptions());

  CallbackReturn callback_result = CallbackReturn::FAILURE;
  const auto& inactive = node->configure(callback_result);
  ASSERT_EQ(callback_result, CallbackReturn::SUCCESS);
  ASSERT_EQ(inactive.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_FALSE(node->get_publishers_info_by_topic("/adas/system/safety_status").empty());
  EXPECT_FALSE(node->get_subscriptions_info_by_topic(
                         "/adas/localization/kinematic_state")
                   .empty());

  callback_result = CallbackReturn::FAILURE;
  const auto& active = node->activate(callback_result);
  ASSERT_EQ(callback_result, CallbackReturn::SUCCESS);
  ASSERT_EQ(active.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  callback_result = CallbackReturn::FAILURE;
  const auto& deactivated = node->deactivate(callback_result);
  ASSERT_EQ(callback_result, CallbackReturn::SUCCESS);
  ASSERT_EQ(deactivated.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  callback_result = CallbackReturn::FAILURE;
  const auto& unconfigured = node->cleanup(callback_result);
  EXPECT_EQ(callback_result, CallbackReturn::SUCCESS);
  EXPECT_EQ(unconfigured.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

}  // namespace
