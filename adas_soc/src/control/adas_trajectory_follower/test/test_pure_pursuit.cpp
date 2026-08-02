// PurePursuitLateral 单测
#include <gtest/gtest.h>

#include <cmath>

#include "adas_common/geometry.hpp"
#include "adas_trajectory_follower/pure_pursuit_lateral.hpp"

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

TEST(PurePursuit, OnPathGivesZeroSteer) {
  ct::PurePursuitLateral pp(ct::PurePursuitParams{});
  const auto traj = straight_x(100.0, 1.0, 10.0);
  const auto cmd = pp.run(input_on(traj, 10.0, 0.0, 0.0, 10.0));
  EXPECT_NEAR(cmd.steering_tire_angle_rad, 0.0, 1e-9);
}

TEST(PurePursuit, RightOffsetSteersLeft) {
  ct::PurePursuitLateral pp(ct::PurePursuitParams{});
  const auto traj = straight_x(100.0, 1.0, 10.0);
  // 自车在路径右侧（y=-1）→ 应左打舵（正）
  const auto cmd = pp.run(input_on(traj, 10.0, -1.0, 0.0, 10.0));
  EXPECT_GT(cmd.steering_tire_angle_rad, 0.01);
  // 左侧对称
  ct::PurePursuitLateral pp2(ct::PurePursuitParams{});
  const auto cmd2 = pp2.run(input_on(traj, 10.0, 1.0, 0.0, 10.0));
  EXPECT_LT(cmd2.steering_tire_angle_rad, -0.01);
  EXPECT_NEAR(cmd.steering_tire_angle_rad, -cmd2.steering_tire_angle_rad, 1e-9);
}

TEST(PurePursuit, SteerClampedToMax) {
  ct::PurePursuitParams params;
  params.max_steer_rad = 0.3;
  ct::PurePursuitLateral pp(params);
  const auto traj = straight_x(100.0, 1.0, 10.0);
  const auto cmd = pp.run(input_on(traj, 10.0, -8.0, 1.5, 10.0));
  EXPECT_LE(std::fabs(cmd.steering_tire_angle_rad), 0.3 + 1e-9);
}

TEST(PurePursuit, EmptyTrajectoryHoldsLastSteer) {
  ct::PurePursuitLateral pp(ct::PurePursuitParams{});
  const auto traj = straight_x(100.0, 1.0, 10.0);
  const auto cmd1 = pp.run(input_on(traj, 10.0, -1.0, 0.0, 10.0));
  ac::Trajectory empty;
  auto in = input_on(empty, 10.0, -1.0, 0.0, 10.0);
  const auto cmd2 = pp.run(in);
  EXPECT_NEAR(cmd2.steering_tire_angle_rad, cmd1.steering_tire_angle_rad, 1e-12);
}

// 小闭环：PurePursuit 驱动运动学自行车模型，从 1m 偏移收敛回直线轨迹
TEST(PurePursuit, ClosedLoopConvergesToPath) {
  ct::PurePursuitParams params;
  ct::PurePursuitLateral pp(params);
  const auto traj = straight_x(400.0, 1.0, 15.0);

  double x = 0.0, y = 1.0, yaw = 0.0;
  const double v = 15.0, dt = 0.02, wheelbase = 2.7;
  for (int i = 0; i < 500; ++i) {  // 10s
    const auto cmd = pp.run(input_on(traj, x, y, yaw, v));
    yaw = ac::normalize_angle(yaw + v / wheelbase * std::tan(cmd.steering_tire_angle_rad) * dt);
    x += v * std::cos(yaw) * dt;
    y += v * std::sin(yaw) * dt;
  }
  EXPECT_LT(std::fabs(y), 0.05);      // 收敛回中心线
  EXPECT_LT(std::fabs(yaw), 0.02);
}
