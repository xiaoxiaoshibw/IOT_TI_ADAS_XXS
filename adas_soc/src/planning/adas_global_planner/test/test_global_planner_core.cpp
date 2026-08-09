#include <gtest/gtest.h>

#include "adas_global_planner/global_planner_core.hpp"
#include "adas_global_planner/semantic_route.hpp"

namespace ap = adas::planning;

namespace {

ap::LaneSegment lane(std::int64_t id, double x0, double y0, double x1, double y1,
                     bool junction = false) {
  return {id, {{x0, y0, 0.0}, {x1, y1, 0.0}}, 13.9, junction};
}

ap::LaneGraph simple_graph() {
  ap::LaneGraph graph;
  EXPECT_TRUE(graph.add_lane(lane(1, 0.0, 0.0, 10.0, 0.0)));
  EXPECT_TRUE(graph.add_lane(lane(2, 10.0, 0.0, 20.0, 0.0)));
  EXPECT_TRUE(graph.add_lane(lane(3, 20.0, 0.0, 30.0, 0.0)));
  EXPECT_TRUE(graph.add_connection({1, 2, ap::Maneuver::kStraight, 0.0}));
  EXPECT_TRUE(graph.add_connection({2, 3, ap::Maneuver::kStraight, 0.0}));
  return graph;
}

}  // namespace

TEST(LaneGraph, RejectsInvalidDataAndDuplicateEdges) {
  ap::LaneGraph graph;
  EXPECT_FALSE(graph.add_lane(lane(0, 0.0, 0.0, 1.0, 0.0)));
  EXPECT_TRUE(graph.add_lane(lane(1, 0.0, 0.0, 10.0, 0.0)));
  EXPECT_FALSE(graph.add_lane(lane(1, 0.0, 1.0, 10.0, 1.0)));
  EXPECT_TRUE(graph.add_lane(lane(2, 10.0, 0.0, 20.0, 0.0)));
  EXPECT_TRUE(graph.add_connection({1, 2, ap::Maneuver::kStraight, 0.0}));
  EXPECT_FALSE(graph.add_connection({1, 2, ap::Maneuver::kStraight, 0.0}));
  EXPECT_FALSE(graph.add_connection({2, 99, ap::Maneuver::kStraight, 0.0}));
}

TEST(GlobalPlanner, FindsConnectedRoute) {
  auto graph = simple_graph();
  const ap::GlobalPlannerCore planner;
  const auto route = planner.plan(graph, 1, 3);
  ASSERT_TRUE(route.valid) << route.failure_reason;
  ASSERT_EQ(route.segments.size(), 3U);
  EXPECT_EQ(route.segments[0].lane_id, 1);
  EXPECT_EQ(route.segments[1].lane_id, 2);
  EXPECT_EQ(route.segments[2].lane_id, 3);
  EXPECT_DOUBLE_EQ(route.total_cost_m, 30.0);
}

TEST(GlobalPlanner, UsesCoordinateSnapping) {
  auto graph = simple_graph();
  const ap::GlobalPlannerCore planner;
  const auto route = planner.plan(graph, 2.0, 0.5, 28.0, -0.4);
  ASSERT_TRUE(route.valid) << route.failure_reason;
  EXPECT_EQ(route.start_lane_id, 1);
  EXPECT_EQ(route.goal_lane_id, 3);
}

TEST(GlobalPlanner, RejectsUnreachableGoal) {
  auto graph = simple_graph();
  ASSERT_TRUE(graph.add_lane(lane(4, 0.0, 20.0, 10.0, 20.0)));
  const ap::GlobalPlannerCore planner;
  const auto route = planner.plan(graph, 1, 4);
  EXPECT_FALSE(route.valid);
  EXPECT_EQ(route.failure_reason, "goal is unreachable");
}

TEST(GlobalPlanner, PrefersStraightRouteOverLaneChangeShortcut) {
  ap::LaneGraph graph;
  ASSERT_TRUE(graph.add_lane(lane(1, 0.0, 0.0, 10.0, 0.0)));
  ASSERT_TRUE(graph.add_lane(lane(2, 10.0, 0.0, 20.0, 0.0)));
  ASSERT_TRUE(graph.add_lane(lane(3, 20.0, 0.0, 30.0, 0.0)));
  ASSERT_TRUE(graph.add_lane(lane(4, 10.0, 3.5, 18.0, 3.5)));
  ASSERT_TRUE(graph.add_connection({1, 2, ap::Maneuver::kStraight, 0.0}));
  ASSERT_TRUE(graph.add_connection({2, 3, ap::Maneuver::kStraight, 0.0}));
  ASSERT_TRUE(graph.add_connection({1, 4, ap::Maneuver::kLaneChangeLeft, 0.0}));
  ASSERT_TRUE(graph.add_connection({4, 3, ap::Maneuver::kLaneChangeRight, 0.0}));
  const ap::GlobalPlannerCore planner;
  const auto route = planner.plan(graph, 1, 3);
  ASSERT_TRUE(route.valid);
  ASSERT_EQ(route.segments.size(), 3U);
  EXPECT_EQ(route.segments[1].lane_id, 2);
}

TEST(GlobalPlanner, HandlesStartAlreadyOnGoalLane) {
  auto graph = simple_graph();
  const ap::GlobalPlannerCore planner;
  const auto route = planner.plan(graph, 2, 2);
  ASSERT_TRUE(route.valid);
  ASSERT_EQ(route.segments.size(), 1U);
  EXPECT_EQ(route.segments.front().lane_id, 2);
}

TEST(SemanticRoute, StopsAtProjectedGoalInsteadOfLaneEnd) {
  ap::GlobalRoute route;
  route.valid = true;
  ap::RouteSegment segment;
  segment.lane_id = 1;
  segment.speed_limit_mps = 13.9;
  segment.centerline = {{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}};
  route.segments.push_back(segment);

  // The click is 2m off-center. The safe arrival point must be its lane
  // projection at x=80, not the raw click and not the lane end at x=100.
  const auto semantic = ap::build_semantic_route(route, 10.0, 0.0, 80.0, 2.0);
  ASSERT_TRUE(semantic.valid) << semantic.failure_reason;
  ASSERT_GE(semantic.points.size(), 2U);
  EXPECT_NEAR(semantic.points.front().pose.x, 10.0, 1e-9);
  EXPECT_NEAR(semantic.points.back().pose.x, 80.0, 1e-9);
  EXPECT_NEAR(semantic.points.back().pose.y, 0.0, 1e-9);
  EXPECT_NEAR(semantic.length_m, 70.0, 1e-9);
  EXPECT_TRUE(semantic.points.back().stop);
}
