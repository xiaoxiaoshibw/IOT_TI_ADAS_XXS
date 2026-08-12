#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "adas_global_planner/route_adapter.hpp"
#include "adas_msgs/msg/global_route.hpp"
#include "adas_msgs/msg/route_point.hpp"
#include "nav_msgs/msg/path.hpp"

namespace ap = adas::planning;

namespace {

adas_msgs::msg::GlobalRoute make_route(const std::string& run_id,
                                       std::uint32_t route_id,
                                       std::uint8_t status,
                                       const std::string& frame = "map",
                                       int point_count = 3) {
  adas_msgs::msg::GlobalRoute route;
  route.header.frame_id = frame;
  route.header.stamp = rclcpp::Time(100, 0);
  route.frame_id = frame;
  route.run_id = run_id;
  route.route_id = route_id;
  route.status = status;
  route.length = 12.5F;
  route.map_id = "Town04";
  route.map_hash = "hash";
  route.goal_id = "goal-1";
  for (int i = 0; i < point_count; ++i) {
    adas_msgs::msg::RoutePoint rp;
    rp.x = static_cast<float>(i * 1.0);
    rp.y = 0.0F;
    rp.yaw = 0.0F;
    route.points.push_back(rp);
  }
  return route;
}

}  // namespace

TEST(RouteAdapter, AcceptsValidRouteWithMatchingRunId) {
  rclcpp::init(0, nullptr);
  ap::RouteAdapterNode node;
  node.test_set_bound_run_id("11111111-2222-4333-8444-555555555555");
  node.test_reset_state();
  const auto route = make_route("11111111-2222-4333-8444-555555555555", 1U,
                                adas_msgs::msg::GlobalRoute::STATUS_VALID);
  node.test_feed(route);
  EXPECT_TRUE(node.test_published_once());
  EXPECT_EQ(node.test_last_route_id(), 1U);
  EXPECT_EQ(node.test_last_status(),
            adas_msgs::msg::GlobalRoute::STATUS_VALID);
  rclcpp::shutdown();
}

TEST(RouteAdapter, RejectsEmptyRunId) {
  rclcpp::init(0, nullptr);
  ap::RouteAdapterNode node;
  node.test_set_bound_run_id("11111111-2222-4333-8444-555555555555");
  node.test_reset_state();
  const auto route = make_route("", 1U,
                                adas_msgs::msg::GlobalRoute::STATUS_VALID);
  node.test_feed(route);
  EXPECT_FALSE(node.test_published_once());
  rclcpp::shutdown();
}

TEST(RouteAdapter, RejectsMismatchedRunId) {
  rclcpp::init(0, nullptr);
  ap::RouteAdapterNode node;
  node.test_set_bound_run_id("11111111-2222-4333-8444-555555555555");
  node.test_reset_state();
  const auto route = make_route("aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee", 1U,
                                adas_msgs::msg::GlobalRoute::STATUS_VALID);
  node.test_feed(route);
  EXPECT_FALSE(node.test_published_once());
  rclcpp::shutdown();
}

TEST(RouteAdapter, RejectsRouteIdZero) {
  rclcpp::init(0, nullptr);
  ap::RouteAdapterNode node;
  node.test_set_bound_run_id("11111111-2222-4333-8444-555555555555");
  node.test_reset_state();
  const auto route = make_route("11111111-2222-4333-8444-555555555555", 0U,
                                adas_msgs::msg::GlobalRoute::STATUS_VALID);
  node.test_feed(route);
  EXPECT_FALSE(node.test_published_once());
  rclcpp::shutdown();
}

TEST(RouteAdapter, RejectsFrameMismatch) {
  rclcpp::init(0, nullptr);
  ap::RouteAdapterNode node;
  node.test_set_bound_run_id("11111111-2222-4333-8444-555555555555");
  auto route = make_route("11111111-2222-4333-8444-555555555555", 1U,
                          adas_msgs::msg::GlobalRoute::STATUS_VALID);
  route.header.frame_id = "odom";  // 与 frame_id="map" 不一致
  node.test_reset_state();
  node.test_feed(route);
  EXPECT_FALSE(node.test_published_once());
  rclcpp::shutdown();
}

TEST(RouteAdapter, RejectsTooFewPoints) {
  rclcpp::init(0, nullptr);
  ap::RouteAdapterNode node;
  node.test_set_bound_run_id("11111111-2222-4333-8444-555555555555");
  node.test_reset_state();
  const auto route = make_route("11111111-2222-4333-8444-555555555555", 1U,
                                adas_msgs::msg::GlobalRoute::STATUS_VALID,
                                "map", /*point_count=*/1);
  node.test_feed(route);
  EXPECT_FALSE(node.test_published_once());
  rclcpp::shutdown();
}

TEST(RouteAdapter, RejectsNonFinitePoints) {
  rclcpp::init(0, nullptr);
  ap::RouteAdapterNode node;
  node.test_set_bound_run_id("11111111-2222-4333-8444-555555555555");
  node.test_reset_state();
  auto route = make_route("11111111-2222-4333-8444-555555555555", 1U,
                          adas_msgs::msg::GlobalRoute::STATUS_VALID);
  route.points[1].x = std::numeric_limits<float>::infinity();
  node.test_feed(route);
  EXPECT_FALSE(node.test_published_once());
  rclcpp::shutdown();
}

TEST(RouteAdapter, NonValidStatusPublishesClear) {
  rclcpp::init(0, nullptr);
  ap::RouteAdapterNode node;
  node.test_set_bound_run_id("11111111-2222-4333-8444-555555555555");
  node.test_reset_state();
  const auto invalid = make_route("11111111-2222-4333-8444-555555555555", 2U,
                                  adas_msgs::msg::GlobalRoute::STATUS_INVALID);
  node.test_feed(invalid);
  EXPECT_TRUE(node.test_published_once());
  EXPECT_EQ(node.test_last_status(),
            adas_msgs::msg::GlobalRoute::STATUS_INVALID);
  const auto valid = make_route("11111111-2222-4333-8444-555555555555", 3U,
                                adas_msgs::msg::GlobalRoute::STATUS_VALID);
  node.test_feed(valid);
  EXPECT_EQ(node.test_last_route_id(), 3U);
  EXPECT_EQ(node.test_last_status(),
            adas_msgs::msg::GlobalRoute::STATUS_VALID);
  rclcpp::shutdown();
}

TEST(RouteAdapter, DuplicateHeartbeatDoesNotRepublish) {
  rclcpp::init(0, nullptr);
  ap::RouteAdapterNode node;
  node.test_set_bound_run_id("11111111-2222-4333-8444-555555555555");
  node.test_reset_state();
  const auto route = make_route("11111111-2222-4333-8444-555555555555", 4U,
                                adas_msgs::msg::GlobalRoute::STATUS_VALID);
  node.test_feed(route);
  EXPECT_TRUE(node.test_published_once());
  for (int i = 0; i < 3; ++i) node.test_feed(route);
  EXPECT_EQ(node.test_last_route_id(), 4U);
  EXPECT_EQ(node.test_last_status(),
            adas_msgs::msg::GlobalRoute::STATUS_VALID);
  rclcpp::shutdown();
}

TEST(RouteAdapter, TwoDistinctValidRouteIdsBothPublish) {
  rclcpp::init(0, nullptr);
  ap::RouteAdapterNode node;
  node.test_set_bound_run_id("11111111-2222-4333-8444-555555555555");
  node.test_reset_state();
  const auto first = make_route("11111111-2222-4333-8444-555555555555", 7U,
                                adas_msgs::msg::GlobalRoute::STATUS_VALID);
  node.test_feed(first);
  EXPECT_EQ(node.test_last_route_id(), 7U);
  const auto second = make_route("11111111-2222-4333-8444-555555555555", 8U,
                                 adas_msgs::msg::GlobalRoute::STATUS_VALID);
  node.test_feed(second);
  EXPECT_EQ(node.test_last_route_id(), 8U);
  rclcpp::shutdown();
}

TEST(RouteAdapter, BoundRunIdEmptyAlwaysRejects) {
  rclcpp::init(0, nullptr);
  ap::RouteAdapterNode node;
  node.test_reset_state();
  const auto route = make_route("11111111-2222-4333-8444-555555555555", 1U,
                                adas_msgs::msg::GlobalRoute::STATUS_VALID);
  node.test_feed(route);
  EXPECT_FALSE(node.test_published_once());
  rclcpp::shutdown();
}

TEST(RouteAdapterDecision, PureFunctionMatrix) {
  const std::string bound = "11111111-2222-4333-8444-555555555555";
  // 空 run_id 消息 → 拒。
  {
    auto route = make_route("", 1U,
                            adas_msgs::msg::GlobalRoute::STATUS_VALID);
    const auto d = ap::evaluate_route_for_adapter(bound, route);
    EXPECT_FALSE(d.accept);
  }
  // 错配 → 拒。
  {
    auto route = make_route("aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee", 1U,
                            adas_msgs::msg::GlobalRoute::STATUS_VALID);
    const auto d = ap::evaluate_route_for_adapter(bound, route);
    EXPECT_FALSE(d.accept);
  }
  // route_id=0 → 拒。
  {
    auto route = make_route(bound, 0U,
                            adas_msgs::msg::GlobalRoute::STATUS_VALID);
    const auto d = ap::evaluate_route_for_adapter(bound, route);
    EXPECT_FALSE(d.accept);
  }
  // 正常 VALID → 接受且非 clear。
  {
    auto route = make_route(bound, 1U,
                            adas_msgs::msg::GlobalRoute::STATUS_VALID);
    const auto d = ap::evaluate_route_for_adapter(bound, route);
    EXPECT_TRUE(d.accept);
    EXPECT_FALSE(d.publish_clear);
  }
  // FAILED → 接受但 clear。
  {
    auto route = make_route(bound, 2U,
                            adas_msgs::msg::GlobalRoute::STATUS_FAILED);
    const auto d = ap::evaluate_route_for_adapter(bound, route);
    EXPECT_TRUE(d.accept);
    EXPECT_TRUE(d.publish_clear);
  }
  // frame 错配 → 拒。
  {
    auto route = make_route(bound, 3U,
                            adas_msgs::msg::GlobalRoute::STATUS_VALID, "odom");
    route.header.frame_id = "map";  // 与 frame_id="odom" 不一致
    const auto d = ap::evaluate_route_for_adapter(bound, route);
    EXPECT_FALSE(d.accept);
  }
  // 空 bound → 拒(分机拓扑无握手)。
  {
    auto route = make_route(bound, 4U,
                            adas_msgs::msg::GlobalRoute::STATUS_VALID);
    const auto d = ap::evaluate_route_for_adapter("", route);
    EXPECT_FALSE(d.accept);
  }
}
