#include <gtest/gtest.h>

#include <memory>

#include "../src/can_gateway_node.cpp"

namespace {

class CanGatewayNodeShellTest : public ::testing::Test {
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

TEST_F(CanGatewayNodeShellTest, ConstructsSocketCanResources) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("transport", "sim");
  options.append_parameter_override("can_interface", "vcan0");
  options.append_parameter_override(
      "sequence_persist_path", "/tmp/adas_can_gateway_node_shell_seq.bin");
  options.append_parameter_override(
      "hil_session_persist_path", "/tmp/adas_can_gateway_node_shell_hil.bin");

  auto node = std::make_shared<adas::can_gateway::CanGatewayNode>(options);
  EXPECT_FALSE(node->get_publishers_info_by_topic("/adas/mcu/status").empty());
  EXPECT_FALSE(node->get_publishers_info_by_topic("/adas/mcu/actuation_feedback").empty());
  EXPECT_FALSE(node->get_subscriptions_info_by_topic("/adas/control/gate/control_cmd").empty());
  EXPECT_FALSE(node->get_subscriptions_info_by_topic("/adas/system/safety_status").empty());
}

}  // namespace
