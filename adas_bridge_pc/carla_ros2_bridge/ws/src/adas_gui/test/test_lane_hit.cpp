// lane_hit.hpp 最近车道点查询的单元测试（无 Qt，用最小点/车道桩）。
#include <gtest/gtest.h>

#include <vector>

#include "lane_hit.hpp"

namespace {

struct StubPoint {
  double px, py;
  double x() const { return px; }
  double y() const { return py; }
};

struct StubLane {
  std::vector<StubPoint> centerline;
};

using adas::gui::LaneHit;
using adas::gui::nearest_lane_point;

TEST(LaneHit, EmptyLanesInvalid) {
  std::vector<StubLane> lanes;
  EXPECT_FALSE(nearest_lane_point(lanes, 0.0, 0.0, 8.0).valid);
}

TEST(LaneHit, ProjectsOntoSegmentInterior) {
  // 水平线段 (0,0)-(10,0)，查询 (5,3) → 最近点 (5,0)，距离 3
  std::vector<StubLane> lanes{{{{0, 0}, {10, 0}}}};
  const LaneHit hit = nearest_lane_point(lanes, 5.0, 3.0, 8.0);
  ASSERT_TRUE(hit.valid);
  EXPECT_NEAR(hit.x, 5.0, 1e-9);
  EXPECT_NEAR(hit.y, 0.0, 1e-9);
  EXPECT_NEAR(hit.distance, 3.0, 1e-9);
}

TEST(LaneHit, ClampsToSegmentEndpoint) {
  // 查询点在线段延长线外 → 最近点吸到端点
  std::vector<StubLane> lanes{{{{0, 0}, {10, 0}}}};
  const LaneHit hit = nearest_lane_point(lanes, 14.0, 3.0, 8.0);
  ASSERT_TRUE(hit.valid);
  EXPECT_NEAR(hit.x, 10.0, 1e-9);
  EXPECT_NEAR(hit.y, 0.0, 1e-9);
  EXPECT_NEAR(hit.distance, 5.0, 1e-9);
}

TEST(LaneHit, BeyondMaxDistanceRejected) {
  std::vector<StubLane> lanes{{{{0, 0}, {10, 0}}}};
  const LaneHit hit = nearest_lane_point(lanes, 5.0, 8.5, 8.0);
  EXPECT_FALSE(hit.valid);
  // 距离仍返回，供 UI 提示"离车道 X m"
  EXPECT_NEAR(hit.distance, 8.5, 1e-9);
}

TEST(LaneHit, PicksNearestAcrossLanes) {
  std::vector<StubLane> lanes{
      {{{0, 0}, {10, 0}}},    // 距 (5,3) = 3
      {{{0, 4}, {10, 4}}},    // 距 (5,3) = 1 ← 更近
  };
  const LaneHit hit = nearest_lane_point(lanes, 5.0, 3.0, 8.0);
  ASSERT_TRUE(hit.valid);
  EXPECT_NEAR(hit.y, 4.0, 1e-9);
  EXPECT_NEAR(hit.distance, 1.0, 1e-9);
}

TEST(LaneHit, SinglePointLaneIgnored) {
  // 单点车道构不成线段，不参与命中（车道图采样正常 ≥2 点）
  std::vector<StubLane> lanes{{{{5, 5}}}};
  EXPECT_FALSE(nearest_lane_point(lanes, 5.0, 5.0, 8.0).valid);
}

}  // namespace
