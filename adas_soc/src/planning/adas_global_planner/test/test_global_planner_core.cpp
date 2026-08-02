#include <gtest/gtest.h>

#include "adas_global_planner/global_planner_core.hpp"

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
