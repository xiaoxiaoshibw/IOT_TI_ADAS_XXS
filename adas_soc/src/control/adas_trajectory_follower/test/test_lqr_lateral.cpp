// LqrLateral 单测
#include <gtest/gtest.h>

#include <cmath>

#include "adas_common/geometry.hpp"
#include "adas_trajectory_follower/lqr_lateral.hpp"

namespace ct = adas::control;
namespace ac = adas::common;

namespace {

ac::Trajectory straight_x(double length, double step, double v) {
  ac::Trajectory traj;
  for (double s = 0.0; s <= length + 1e-9; s += step) {
    ac::TrajPoint p;
    p.x = s;
    p.velocity_mps = v;
    traj.push_back(p);
  }
  return traj;
}

ct::ControlInput input_on(const ac::Trajectory& traj, double x, double y, double yaw, double v) {
  ct::ControlInput in;
  in.trajectory = &traj;
  in.state.pose = ac::Pose2d{x, y, yaw};
  in.state.velocity_mps = v;
  in.dt = 0.02;
  return in;
}

}  // namespace

TEST(LqrLateral, OnPathZeroSteer) {
  ct::LqrLateral lqr((ct::LqrLateralParams()));
  const auto traj = straight_x(100.0, 1.0, 10.0);
  EXPECT_NEAR(lqr.run(input_on(traj, 10.0, 0.0, 0.0, 10.0)).steering_tire_angle_rad, 0.0,
              1e-9);
}

TEST(LqrLateral, OffsetSteersTowardPathSymmetric) {
  ct::LqrLateral l1((ct::LqrLateralParams()));
  ct::LqrLateral l2((ct::LqrLateralParams()));
  const auto traj = straight_x(100.0, 1.0, 10.0);
  const auto right = l1.run(input_on(traj, 10.0, -1.0, 0.0, 10.0));
  const auto left = l2.run(input_on(traj, 10.0, 1.0, 0.0, 10.0));
  EXPECT_GT(right.steering_tire_angle_rad, 0.01);   // 右偏 → 左打舵
  EXPECT_LT(left.steering_tire_angle_rad, -0.01);
  EXPECT_NEAR(right.steering_tire_angle_rad, -left.steering_tire_angle_rad, 1e-9);
}

TEST(LqrLateral, SteerClamped) {
  ct::LqrLateralParams p;
  p.max_steer_rad = 0.3;
  ct::LqrLateral lqr(p);
  const auto traj = straight_x(100.0, 1.0, 10.0);
  const auto cmd = lqr.run(input_on(traj, 10.0, -8.0, 1.0, 10.0));
  EXPECT_LE(std::fabs(cmd.steering_tire_angle_rad), 0.3 + 1e-9);
}

TEST(LqrLateral, CurvatureFeedforwardOnCircle) {
  ct::LqrLateralParams p;
  ct::LqrLateral lqr(p);
  // 常曲率圆弧轨迹（R=50），车在轨迹上 → 输出应接近前馈 atan(L·κ)
  ac::Trajectory traj;
  const double k = 0.02;
  double x = 0.0, y = 0.0, yaw = 0.0;
  for (int i = 0; i < 100; ++i) {
    ac::TrajPoint pt;
    pt.x = x;
    pt.y = y;
    pt.yaw = yaw;
    pt.curvature = k;
    pt.velocity_mps = 10.0;
    traj.push_back(pt);
    yaw = ac::normalize_angle(yaw + k * 1.0);
    x += std::cos(yaw);
    y += std::sin(yaw);
  }
  // 稳态巡圆：转角回读 = 前馈值（3 状态执行反馈项归零）
  auto in = input_on(traj, traj[10].x, traj[10].y, traj[10].yaw, 10.0);
  in.steering_angle_rad = std::atan(p.wheelbase_m * k);
  const auto cmd = lqr.run(in);
  // 前视项会在前馈之上叠加少量预打舵（设计行为），容差放宽
  EXPECT_NEAR(cmd.steering_tire_angle_rad, std::atan(p.wheelbase_m * k), 0.05);
  EXPECT_GT(cmd.steering_tire_angle_rad, 0.0);  // 方向必须正确（左弯左打舵）
}

TEST(LqrLateral, ClosedLoopConvergesFromOffset) {
  ct::LqrLateralParams params;
  ct::LqrLateral lqr(params);
  const auto traj = straight_x(400.0, 1.0, 15.0);
  double x = 0.0, y = 1.0, yaw = 0.0;
  const double v = 15.0, dt = 0.02, wheelbase = 2.7;
  for (int i = 0; i < 500; ++i) {  // 10s
    const auto cmd = lqr.run(input_on(traj, x, y, yaw, v));
    yaw = ac::normalize_angle(yaw + v / wheelbase * std::tan(cmd.steering_tire_angle_rad) * dt);
    x += v * std::cos(yaw) * dt;
    y += v * std::sin(yaw) * dt;
  }
  EXPECT_LT(std::fabs(y), 0.05);
  EXPECT_LT(std::fabs(yaw), 0.02);
}

TEST(LqrLateral, GainSchedulingMonotonicBehavior) {
  // 同一误差下，高速时反馈应更温和（转角更小）——增益调度生效的可观察结果
  ct::LqrLateral slow((ct::LqrLateralParams()));
  ct::LqrLateral fast((ct::LqrLateralParams()));
  const auto traj = straight_x(200.0, 1.0, 10.0);
  const auto cmd_slow = slow.run(input_on(traj, 10.0, -0.5, 0.0, 5.0));
  const auto cmd_fast = fast.run(input_on(traj, 10.0, -0.5, 0.0, 25.0));
  EXPECT_GT(cmd_slow.steering_tire_angle_rad, cmd_fast.steering_tire_angle_rad);
}
