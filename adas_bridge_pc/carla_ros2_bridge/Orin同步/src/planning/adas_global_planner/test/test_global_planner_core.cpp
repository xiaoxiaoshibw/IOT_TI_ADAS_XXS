#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "adas_global_planner/global_planner_core.hpp"
#include "adas_global_planner/route_adapter.hpp"
#include "adas_global_planner/route_validator.hpp"
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

TEST(GlobalPlanner, SkipsLaneChangeWhenStartLaneHasInsufficientDistance) {
  ap::LaneGraph graph;
  ASSERT_TRUE(graph.add_lane(lane(1, 0.0, 0.0, 20.0, 0.0)));
  ASSERT_TRUE(graph.add_lane(lane(2, 0.0, 3.5, 20.0, 3.5)));
  ASSERT_TRUE(graph.add_lane(lane(3, 20.0, 0.0, 30.0, 0.0)));
  ASSERT_TRUE(graph.add_lane(lane(4, 30.0, 0.0, 40.0, 0.0)));
  ASSERT_TRUE(graph.add_connection({1, 2, ap::Maneuver::kLaneChangeLeft, 0.0}));
  ASSERT_TRUE(graph.add_connection({2, 4, ap::Maneuver::kStraight, 0.0}));
  ASSERT_TRUE(graph.add_connection({1, 3, ap::Maneuver::kStraight, 20.0}));
  ASSERT_TRUE(graph.add_connection({3, 4, ap::Maneuver::kStraight, 0.0}));
  const ap::GlobalPlannerCore planner;

  const auto route = planner.plan(graph, 19.5, 0.0, 39.0, 0.0);

  ASSERT_TRUE(route.valid) << route.failure_reason;
  ASSERT_EQ(route.segments.size(), 3U);
  EXPECT_EQ(route.segments[1].lane_id, 3);
}

double maximum_gap(const ap::SemanticRouteResult& route) {
  double result = 0.0;
  for (std::size_t index = 1U; index < route.points.size(); ++index) {
    result = std::max(
        result,
        std::hypot(route.points[index].pose.x - route.points[index - 1U].pose.x,
                   route.points[index].pose.y - route.points[index - 1U].pose.y));
  }
  return result;
}

TEST(SemanticRoute, TrimsStartAndGoalAndCarriesConstraints) {
  auto graph = simple_graph();
  const ap::GlobalPlannerCore planner;
  const auto route = planner.plan(graph, 2.0, 0.2, 27.0, -0.1);
  ASSERT_TRUE(route.valid) << route.failure_reason;
  const auto semantic = ap::build_semantic_route(route, 2.0, 0.2, 27.0, -0.1);
  ASSERT_TRUE(semantic.valid) << semantic.failure_reason;
  ASSERT_GE(semantic.points.size(), 4U);
  EXPECT_NEAR(semantic.points.front().pose.x, 2.0, 1e-9);
  EXPECT_NEAR(semantic.points.back().pose.x, 27.0, 1e-9);
  EXPECT_NEAR(semantic.length_m, 25.0, 1e-9);
  EXPECT_EQ(semantic.points.front().lane_id, 1);
  EXPECT_EQ(semantic.points.back().lane_id, 3);
  EXPECT_DOUBLE_EQ(semantic.points.front().speed_limit_mps, 13.9);
  EXPECT_TRUE(semantic.points.back().stop);
}

TEST(SemanticRoute, RejectsGoalBehindStartOnDirectedLane) {
  auto graph = simple_graph();
  const ap::GlobalPlannerCore planner;
  const auto route = planner.plan(graph, 8.0, 0.0, 2.0, 0.0);
  ASSERT_TRUE(route.valid);
  const auto semantic = ap::build_semantic_route(route, 8.0, 0.0, 2.0, 0.0);
  EXPECT_FALSE(semantic.valid);
  EXPECT_EQ(semantic.failure_reason,
            "goal lies behind start on the same directed lane");
}

TEST(SemanticRoute, PreservesTurnManeuverAndDecodesRoadId) {
  ap::LaneGraph graph;
  constexpr std::int64_t first_id = (std::int64_t{42} << 24) | 1;
  constexpr std::int64_t turn_id = (std::int64_t{43} << 24) | 2;
  ASSERT_TRUE(graph.add_lane(lane(first_id, 0.0, 0.0, 10.0, 0.0)));
  ASSERT_TRUE(graph.add_lane(lane(turn_id, 10.0, 0.0, 10.0, 10.0, true)));
  ASSERT_TRUE(graph.add_connection({first_id, turn_id, ap::Maneuver::kLeft, 0.0}));
  const ap::GlobalPlannerCore planner;
  const auto route = planner.plan(graph, first_id, turn_id);
  ASSERT_TRUE(route.valid);
  const auto semantic = ap::build_semantic_route(route, 1.0, 0.0, 10.0, 9.0);
  ASSERT_TRUE(semantic.valid) << semantic.failure_reason;
  EXPECT_EQ(semantic.points.back().road_id, 43);
  const auto turn = std::find_if(semantic.points.begin(), semantic.points.end(),
                                 [](const auto& point) {
                                   return point.maneuver == ap::Maneuver::kLeft;
                                 });
  EXPECT_NE(turn, semantic.points.end());
}

TEST(SemanticRoute, Town04LaneChangeDoesNotJumpToRemoteCenterlineFront) {
  ap::GlobalRoute route;
  route.valid = true;
  route.segments = {
      {12750716925, ap::Maneuver::kStraight, 20.0, 20.0, 13.9, false,
       {{384.5449, 77.7402, -1.57846},
        {384.3917, 57.7676, -1.57846}}},
      {12750716926, ap::Maneuver::kLaneChangeLeft, 22.0, 50.0, 13.9, false,
       {{388.0449, 77.7402, -1.57846},
        {387.8916, 57.7407, -1.57846},
        {387.8763, 55.7408, -1.57846}}},
  };

  const auto semantic = ap::build_semantic_route(
      route, 384.3988, 58.6916, 387.8763, 55.7408);

  EXPECT_FALSE(semantic.valid);
  EXPECT_EQ(semantic.failure_reason,
            "lane-change has no safe common forward connection interval");
}

TEST(SemanticRoute, JoinsSuccessorWithoutDuplicateOrGap) {
  ap::GlobalRoute route;
  route.valid = true;
  route.segments = {
      {1, ap::Maneuver::kStraight, 10.0, 10.0, 13.9, false,
       {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}}},
      {2, ap::Maneuver::kStraight, 10.0, 20.0, 13.9, false,
       {{10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}}},
  };
  const auto semantic = ap::build_semantic_route(route, 0.0, 0.0, 20.0, 0.0);
  ASSERT_TRUE(semantic.valid) << semantic.failure_reason;
  EXPECT_LE(maximum_gap(semantic), 3.0);
  for (std::size_t index = 1U; index < semantic.points.size(); ++index) {
    EXPECT_GT(semantic.points[index].pose.x,
              semantic.points[index - 1U].pose.x);
  }
  EXPECT_TRUE(ap::validate_route(semantic.points).valid);
}

TEST(SemanticRoute, CropsOverlappingSuccessorAtForwardProjection) {
  ap::GlobalRoute route;
  route.valid = true;
  route.segments = {
      {1, ap::Maneuver::kStraight, 20.0, 20.0, 13.9, false,
       {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}}},
      {2, ap::Maneuver::kStraight, 20.0, 40.0, 13.9, false,
       {{10.0, 0.0, 0.0}, {30.0, 0.0, 0.0}}},
  };
  const auto semantic = ap::build_semantic_route(route, 0.0, 0.0, 30.0, 0.0);
  ASSERT_TRUE(semantic.valid) << semantic.failure_reason;
  EXPECT_NEAR(semantic.length_m, 30.0, 1e-6);
  EXPECT_LE(maximum_gap(semantic), 3.0);
  for (std::size_t index = 1U; index < semantic.points.size(); ++index) {
    EXPECT_GT(semantic.points[index].pose.x,
              semantic.points[index - 1U].pose.x);
  }
}

TEST(SemanticRoute, BuildsContinuousTown04StyleLaneChange) {
  ap::GlobalRoute route;
  route.valid = true;
  route.segments = {
      {12750716925, ap::Maneuver::kStraight, 30.0, 30.0, 13.9, false,
       {{384.6218, 87.7402, -1.57846},
        {384.3917, 57.7676, -1.57846}}},
      {12750716926, ap::Maneuver::kLaneChangeLeft, 22.0, 60.0, 13.9, false,
       {{388.0449, 77.7402, -1.57846},
        {387.8916, 57.7407, -1.57846},
        {387.8763, 55.7408, -1.57846}}},
  };
  const auto semantic = ap::build_semantic_route(
      route, 384.6218, 87.7402, 387.8763, 55.7408);
  ASSERT_TRUE(semantic.valid) << semantic.failure_reason;
  EXPECT_LE(maximum_gap(semantic), 3.0);
  EXPECT_TRUE(ap::validate_route(semantic.points).valid);
  EXPECT_TRUE(std::any_of(semantic.points.begin(), semantic.points.end(),
                          [](const auto& point) {
                            return point.maneuver ==
                                   ap::Maneuver::kLaneChangeLeft;
                          }));
  EXPECT_FALSE(std::any_of(semantic.points.begin(), semantic.points.end(),
                           [](const auto& point) {
                             return std::hypot(point.pose.x - 388.0449,
                                               point.pose.y - 77.7402) < 1e-3;
                           }));
}

TEST(SemanticRoute, CorrectsCenterlineOppositeToTravelYaw) {
  ap::GlobalRoute route;
  route.valid = true;
  route.segments = {
      {1, ap::Maneuver::kStraight, 10.0, 10.0, 13.9, false,
       {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}}},
      {2, ap::Maneuver::kStraight, 10.0, 20.0, 13.9, false,
       {{20.0, 0.0, 0.0}, {10.0, 0.0, 0.0}}},
  };
  const auto semantic = ap::build_semantic_route(route, 0.0, 0.0, 20.0, 0.0);
  ASSERT_TRUE(semantic.valid) << semantic.failure_reason;
  EXPECT_TRUE(ap::validate_route(semantic.points).valid);
}

TEST(SemanticRoute, RejectsDisconnectedSuccessor) {
  ap::GlobalRoute route;
  route.valid = true;
  route.segments = {
      {1, ap::Maneuver::kStraight, 10.0, 10.0, 13.9, false,
       {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}}},
      {2, ap::Maneuver::kStraight, 10.0, 20.0, 13.9, false,
       {{100.0, 0.0, 0.0}, {110.0, 0.0, 0.0}}},
  };
  const auto semantic = ap::build_semantic_route(route, 0.0, 0.0, 110.0, 0.0);
  EXPECT_FALSE(semantic.valid);
  EXPECT_TRUE(semantic.points.empty());
}

TEST(RouteValidator, RejectsGapReverseAndMissingStop) {
  std::vector<ap::SemanticRoutePoint> points(2);
  points[0].pose = {0.0, 0.0, 0.0};
  points[1].pose = {4.0, 0.0, 0.0};
  points[0].lane_id = points[1].lane_id = 1;
  points[0].speed_limit_mps = points[1].speed_limit_mps = 13.9;
  points[1].stop = true;
  auto validation = ap::validate_route(points);
  EXPECT_FALSE(validation.valid);
  EXPECT_EQ(validation.reason, "adjacent point gap exceeds limit");

  points[1].pose = {-1.0, 0.0, 0.0};
  validation = ap::validate_route(points);
  EXPECT_FALSE(validation.valid);
  EXPECT_EQ(validation.reason, "reverse progress exceeds limit");

  points[1].pose = {1.0, 0.0, 0.0};
  points[1].stop = false;
  validation = ap::validate_route(points);
  EXPECT_FALSE(validation.valid);
  EXPECT_EQ(validation.reason, "route endpoint is not STOP");
}

TEST(RouteFrameContract, RejectsEmptyOrMismatchedFrames) {
  EXPECT_FALSE(ap::global_route_frame_valid("", "map"));
  EXPECT_FALSE(ap::global_route_frame_valid("map", "odom"));
  EXPECT_TRUE(ap::global_route_frame_valid("map", "map"));
}
