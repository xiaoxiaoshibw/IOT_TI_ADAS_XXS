// PidLongitudinal 单测
#include <gtest/gtest.h>

#include <cmath>

#include "adas_trajectory_follower/pid_longitudinal.hpp"

namespace ct = adas::control;
namespace ac = adas::common;

namespace {

ac::Trajectory profile_x(double length, double step, double v) {
  ac::Trajectory traj;
  for (double s = 0.0; s <= length + 1e-9; s += step) {
    ac::TrajPoint p;
    p.x = s;
    p.velocity_mps = v;
    traj.push_back(p);
  }
  return traj;
}

ct::ControlInput input_at(const ac::Trajectory& traj, double x, double v) {
  ct::ControlInput in;
  in.trajectory = &traj;
  in.state.pose = ac::Pose2d{x, 0.0, 0.0};
  in.state.velocity_mps = v;
  in.dt = 0.02;
  return in;
}

}  // namespace

TEST(PidLongitudinal, AcceleratesWhenBelowRef) {
  ct::PidLongitudinal pid(ct::PidLongitudinalParams{});
  const auto traj = profile_x(200.0, 1.0, 15.0);
  const auto cmd = pid.run(input_at(traj, 10.0, 10.0));
  EXPECT_GT(cmd.acceleration_mps2, 0.5);
  EXPECT_EQ(pid.state(), ct::PidLongitudinal::State::kRunning);
}

TEST(PidLongitudinal, BrakesWhenAboveRef) {
  ct::PidLongitudinal pid(ct::PidLongitudinalParams{});
  const auto traj = profile_x(200.0, 1.0, 10.0);
  const auto cmd = pid.run(input_at(traj, 10.0, 15.0));
  EXPECT_LT(cmd.acceleration_mps2, -0.5);
}

TEST(PidLongitudinal, OutputWithinLimits) {
  ct::PidLongitudinalParams params;
  params.max_accel_mps2 = 2.0;
  params.max_decel_mps2 = 3.0;
  ct::PidLongitudinal pid(params);
  const auto fast = profile_x(200.0, 1.0, 40.0);
  EXPECT_LE(pid.run(input_at(fast, 10.0, 0.0)).acceleration_mps2, 2.0 + 1e-9);
  pid.reset();
  const auto slow = profile_x(200.0, 1.0, 0.0);
  EXPECT_GE(pid.run(input_at(slow, 10.0, 40.0)).acceleration_mps2, -3.0 - 1e-9);
}

TEST(PidLongitudinal, StopsAndHoldsAtLowSpeed) {
  ct::PidLongitudinalParams params;
  ct::PidLongitudinal pid(params);
  const auto stop_profile = profile_x(200.0, 1.0, 0.0);  // 全程零速参考
  // 低速 + 零速参考 → 进入 STOPPED，输出驻车制动
  const auto cmd = pid.run(input_at(stop_profile, 10.0, 0.3));
  EXPECT_EQ(pid.state(), ct::PidLongitudinal::State::kStopped);
  EXPECT_NEAR(cmd.acceleration_mps2, params.stop_hold_accel_mps2, 1e-9);
  EXPECT_NEAR(cmd.velocity_mps, 0.0, 1e-9);
  // 参考恢复 → 退出 STOPPED
  const auto go_profile = profile_x(200.0, 1.0, 5.0);
  pid.run(input_at(go_profile, 10.0, 0.0));
  EXPECT_EQ(pid.state(), ct::PidLongitudinal::State::kRunning);
}

TEST(PidLongitudinal, NoStopWhileMovingFast) {
  ct::PidLongitudinal pid(ct::PidLongitudinalParams{});
  const auto stop_profile = profile_x(200.0, 1.0, 0.0);
  // 高速接近零速参考：应保持 RUNNING（PID 制动），不能直接切驻车制动
  pid.run(input_at(stop_profile, 10.0, 15.0));
  EXPECT_EQ(pid.state(), ct::PidLongitudinal::State::kRunning);
}

TEST(PidLongitudinal, EmptyTrajectorySoftBrake) {
  ct::PidLongitudinalParams params;
  ct::PidLongitudinal pid(params);
  ct::ControlInput in;
  ac::Trajectory empty;
  in.trajectory = &empty;
  in.state.velocity_mps = 10.0;
  in.dt = 0.02;
  EXPECT_NEAR(pid.run(in).acceleration_mps2, params.stop_hold_accel_mps2, 1e-9);
}
