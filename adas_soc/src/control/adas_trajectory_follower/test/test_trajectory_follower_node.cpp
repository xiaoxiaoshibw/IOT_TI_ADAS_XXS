#include <gtest/gtest.h>

#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "../src/trajectory_follower_node.cpp"

namespace {

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

TEST(TrajectoryFollowerNode, InvalidLqrGridStepFailsConfigure) {
  if (!rclcpp::ok()) {
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  auto node = std::make_shared<adas::control::TrajectoryFollowerNode>(rclcpp::NodeOptions());
  const auto mode_result =
      node->set_parameter(rclcpp::Parameter("lateral_controller_mode", "lqr"));
  ASSERT_TRUE(mode_result.successful);
  const auto step_result =
      node->set_parameter(rclcpp::Parameter("lqr.v_grid_step", 0.0));
  ASSERT_TRUE(step_result.successful);

  CallbackReturn callback_result = CallbackReturn::SUCCESS;
  const auto& state = node->configure(callback_result);
  EXPECT_EQ(callback_result, CallbackReturn::FAILURE);
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

}  // namespace
