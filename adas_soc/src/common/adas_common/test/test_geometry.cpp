// adas_common 几何工具单测
#include <gtest/gtest.h>

#include "adas_common/geometry.hpp"

namespace ac = adas::common;

namespace {

ac::Trajectory make_straight_x(double length, double step, double v) {
  ac::Trajectory traj;
  for (double s = 0.0; s <= length + 1e-9; s += step) {
    ac::TrajPoint p;
    p.x = s;
    p.y = 0.0;
    p.yaw = 0.0;
    p.velocity_mps = v;
    traj.push_back(p);
  }
  return traj;
}

}  // namespace

TEST(NormalizeAngle, WrapsIntoPlusMinusPi) {
  EXPECT_NEAR(ac::normalize_angle(0.0), 0.0, 1e-12);
  EXPECT_NEAR(ac::normalize_angle(ac::kPi / 2.0), ac::kPi / 2.0, 1e-12);
  EXPECT_NEAR(ac::normalize_angle(-ac::kPi / 2.0), -ac::kPi / 2.0, 1e-12);
  // ±π 边界与大角度：只要求落在 [-π,π] 且三角等价（边界符号不做约定）
  for (double x : {3.0 * ac::kPi, -3.0 * ac::kPi, 7.5, -123.4, ac::kPi, -ac::kPi}) {
    const double n = ac::normalize_angle(x);
    EXPECT_GE(n, -ac::kPi - 1e-12);
    EXPECT_LE(n, ac::kPi + 1e-12);
    EXPECT_NEAR(std::sin(n), std::sin(x), 1e-9);
    EXPECT_NEAR(std::cos(n), std::cos(x), 1e-9);
  }
}

TEST(FindNearestIndex, PicksClosestPoint) {
  const auto traj = make_straight_x(10.0, 1.0, 5.0);
  EXPECT_EQ(ac::find_nearest_index(traj, 3.2, 0.5), 3u);
  EXPECT_EQ(ac::find_nearest_index(traj, -5.0, 0.0), 0u);
  EXPECT_EQ(ac::find_nearest_index(traj, 100.0, 0.0), traj.size() - 1);
}

TEST(PointAtArclength, InterpolatesAlongPath) {
  const auto traj = make_straight_x(10.0, 1.0, 5.0);
  const auto p = ac::point_at_arclength(traj, 0, 2.5);
  EXPECT_NEAR(p.x, 2.5, 1e-9);
  EXPECT_NEAR(p.y, 0.0, 1e-9);
  EXPECT_NEAR(p.velocity_mps, 5.0, 1e-9);
}

TEST(PointAtArclength, ClampsAtEnd) {
  const auto traj = make_straight_x(10.0, 1.0, 5.0);
  const auto p = ac::point_at_arclength(traj, 0, 999.0);
  EXPECT_NEAR(p.x, 10.0, 1e-9);
}

TEST(PointAtArclength, EmptyAndZeroDistance) {
  const ac::Trajectory empty;
  const auto p0 = ac::point_at_arclength(empty, 0, 1.0);
  EXPECT_EQ(p0.x, 0.0);
  const auto traj = make_straight_x(10.0, 1.0, 5.0);
  const auto p1 = ac::point_at_arclength(traj, 4, 0.0);
  EXPECT_NEAR(p1.x, 4.0, 1e-9);
}

TEST(SignedLateralOffset, LeftPositive) {
  const auto traj = make_straight_x(10.0, 1.0, 5.0);
  EXPECT_GT(ac::signed_lateral_offset(traj, 5.0, 1.0), 0.9);   // 左侧（+y）为正
  EXPECT_LT(ac::signed_lateral_offset(traj, 5.0, -1.0), -0.9); // 右侧为负
  EXPECT_NEAR(ac::signed_lateral_offset(traj, 5.0, 0.0), 0.0, 1e-9);
}
