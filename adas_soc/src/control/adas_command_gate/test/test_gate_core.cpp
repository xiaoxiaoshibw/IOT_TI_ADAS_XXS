// GateCore 单测
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "adas_command_gate/gate_core.hpp"

namespace ct = adas::control;
namespace ac = adas::common;

namespace {

ct::GateInputs fresh_follower(double now_s, double steer, double accel, double speed) {
  ct::GateInputs in;
  in.now_s = now_s;
  in.dt = 0.02;
  in.ego_speed_mps = speed;
  in.odom_received = true;
  in.odom_stamp_s = now_s;
  in.follower_received = true;
  in.follower_stamp_s = now_s - 0.01;
  in.follower_cmd.lateral.steering_tire_angle_rad = steer;
  in.follower_cmd.longitudinal.acceleration_mps2 = accel;
  in.follower_cmd.longitudinal.velocity_mps = speed;
  return in;
}

}  // namespace

TEST(GateCore, PassesFreshFollowerCommand) {
  ct::GateCore gate((ct::GateParams()));
  const auto d = gate.update(fresh_follower(100.0, 0.01, 0.1, 10.0));
  EXPECT_EQ(d.source, ct::GateSource::kFollower);
  EXPECT_NEAR(d.cmd.lateral.steering_tire_angle_rad, 0.01, 1e-9);
}

TEST(GateCore, FollowerTimeoutSwitchesToBuiltinStop) {
  ct::GateCore gate((ct::GateParams()));
  auto in = fresh_follower(100.0, 0.01, 0.1, 10.0);
  in.follower_stamp_s = 100.0 - 1.0;  // 1s 前 → 超时
  const auto d = gate.update(in);
  EXPECT_EQ(d.source, ct::GateSource::kBuiltinStop);
  EXPECT_LT(d.cmd.longitudinal.acceleration_mps2, 0.0);  // 在制动
  EXPECT_EQ(d.reason, "follower_timeout");
}

TEST(GateCore, PlannedStopMarksTerminalRoute) {
  ct::GateCore gate((ct::GateParams()));
  auto in = fresh_follower(100.0, 0.01, 0.1, 10.0);
  in.follower_stamp_s = 99.0;
  in.navigation_planned_stop = true;
  const auto d = gate.update(in);
  EXPECT_EQ(d.source, ct::GateSource::kBuiltinStop);
  EXPECT_EQ(d.reason, "planned_stop_route_terminal");
}

TEST(GateCore, NeverReceivedGivesBuiltinStop) {
  ct::GateCore gate((ct::GateParams()));
  ct::GateInputs in;
  in.now_s = 100.0;
  in.dt = 0.02;
  const auto d = gate.update(in);
  EXPECT_EQ(d.source, ct::GateSource::kBuiltinStop);
  EXPECT_EQ(d.reason, "follower_never_received");
}

TEST(GateCore, MrmRequestOverridesFreshFollower) {
  ct::GateCore gate((ct::GateParams()));
  auto in = fresh_follower(100.0, 0.01, 1.0, 10.0);
  in.mrm_stop_requested = true;
  const auto d = gate.update(in);
  EXPECT_EQ(d.source, ct::GateSource::kBuiltinStop);
}

TEST(GateCore, AebBrakeTakeMaxOnlyIncreases) {
  ct::GateParams p;
  ct::GateCore gate(p);
  // AEB 请求 -6，follower 请求 +0.5 → 取 AEB
  auto in = fresh_follower(100.0, 0.0, 0.5, 10.0);
  in.aeb_emergency = true;
  in.aeb_received = true;
  in.aeb_stamp_s = 100.0;
  in.aeb_cmd.longitudinal.acceleration_mps2 = -6.0;
  auto d = gate.update(in);
  EXPECT_EQ(d.source, ct::GateSource::kAeb);
  EXPECT_LT(d.cmd.longitudinal.acceleration_mps2, -3.0);  // 紧急方向不受 jerk 限
  // follower 已在 -7 制动、AEB 只要 -6 → 保持 follower（只增不减）
  ct::GateCore gate2(p);
  auto in2 = fresh_follower(100.0, 0.0, -7.0, 10.0);
  in2.aeb_emergency = true;
  in2.aeb_received = true;
  in2.aeb_stamp_s = 100.0;
  in2.aeb_cmd.longitudinal.acceleration_mps2 = -6.0;
  const auto d2 = gate2.update(in2);
  EXPECT_EQ(d2.source, ct::GateSource::kFollower);
}

TEST(GateCore, SpeedDependentSteerLimit) {
  ct::GateParams p;  // 30m/s 时表值 0.12
  ct::GateCore gate(p);
  // 先把速率限幅"喂饱"：连续多拍逼近，最终应钳在 0.12
  ct::GateDecision d;
  for (int i = 0; i < 200; ++i) {
    d = gate.update(fresh_follower(100.0 + 0.02 * i, 0.5, 0.0, 30.0));
  }
  EXPECT_NEAR(d.cmd.lateral.steering_tire_angle_rad, 0.12, 1e-6);
  EXPECT_TRUE(d.limited);
}

TEST(GateCore, SteerRateLimited) {
  ct::GateParams p;
  ct::GateCore gate(p);
  // 第一拍就要满舵：单拍变化不能超 rate*dt（10m/s → 0.5rad/s * 0.02 = 0.01）
  const auto d = gate.update(fresh_follower(100.0, 0.6, 0.0, 10.0));
  EXPECT_LE(std::fabs(d.cmd.lateral.steering_tire_angle_rad), 0.011);
}

TEST(GateCore, JerkLimitedForNormalSource) {
  ct::GateParams p;
  p.max_jerk_mps3 = 5.0;
  ct::GateCore gate(p);
  // follower 第一拍就要 +3 加速：单拍变化 ≤ 5*0.02 = 0.1
  const auto d = gate.update(fresh_follower(100.0, 0.0, 3.0, 10.0));
  EXPECT_LE(d.cmd.longitudinal.acceleration_mps2, 0.1 + 1e-9);
}

TEST(GateCore, SourceSwitchIsContinuous) {
  ct::GateCore gate((ct::GateParams()));
  // 正常跟随若干拍建立转角
  ct::GateDecision d;
  for (int i = 0; i < 100; ++i) {
    d = gate.update(fresh_follower(100.0 + 0.02 * i, 0.2, 0.5, 10.0));
  }
  const double steer_before = d.cmd.lateral.steering_tire_angle_rad;
  // follower 猝死 → builtin_stop：转角不许突跳（衰减 + 速率限幅）
  ct::GateInputs in;
  in.now_s = 103.0;
  in.dt = 0.02;
  in.ego_speed_mps = 10.0;
  const auto d2 = gate.update(in);
  EXPECT_EQ(d2.source, ct::GateSource::kBuiltinStop);
  EXPECT_NEAR(d2.cmd.lateral.steering_tire_angle_rad, steer_before, 0.02);
}

TEST(GateCore, IllegalFilterTableRejected) {
  ct::GateParams p;
  p.speed_points_mps = {10.0, 0.0};  // 非递增
  p.steer_lim_rad = {0.5, 0.5};
  p.steer_rate_lim_rps = {0.5, 0.5};
  EXPECT_THROW(ct::GateCore{p}, std::invalid_argument);
}

TEST(GateCoreFreshness, AebRemainsEmergencyFor50MsWithoutNewFrame) {
  ct::GateCore gate((ct::GateParams()));
  auto in = fresh_follower(100.0, 0.0, 0.5, 10.0);
  in.aeb_emergency = true;
  in.aeb_received = true;
  in.aeb_stamp_s = 100.0;
  in.aeb_cmd.longitudinal.acceleration_mps2 = -6.0;
  ASSERT_EQ(gate.update(in).source, ct::GateSource::kAeb);

  in.now_s = 100.05;
  in.follower_stamp_s = in.now_s;
  EXPECT_EQ(gate.update(in).source, ct::GateSource::kAeb);
}

TEST(GateCoreFreshness, AebFallsBackAfter150MsWithoutNewFrame) {
  ct::GateCore gate((ct::GateParams()));
  auto in = fresh_follower(100.0, 0.0, 0.5, 10.0);
  in.aeb_emergency = true;
  in.aeb_received = true;
  in.aeb_stamp_s = 100.0;
  in.aeb_cmd.longitudinal.acceleration_mps2 = -6.0;
  ASSERT_EQ(gate.update(in).source, ct::GateSource::kAeb);

  in.now_s = 100.15;
  in.follower_stamp_s = in.now_s;
  const auto d = gate.update(in);
  EXPECT_EQ(d.source, ct::GateSource::kFollower);
  EXPECT_NE(d.reason, "aeb_emergency_override");
}

TEST(GateCoreFreshness, FreshOdomEnablesSpeedDependentSteerLimit) {
  ct::GateParams p;
  ct::GateCore gate(p);
  ct::GateDecision d;
  for (int i = 0; i < 200; ++i) {
    d = gate.update(fresh_follower(100.0 + 0.02 * i, 0.5, 0.0, 30.0));
  }
  EXPECT_NEAR(d.cmd.lateral.steering_tire_angle_rad, 0.12, 1e-6);
}

TEST(GateCoreFreshness, StaleOdomFallsBackToMostConservativeSteerLimit) {
  ct::GateParams p;
  p.steer_rate_lim_rps = {100.0, 100.0, 100.0, 100.0};
  ct::GateCore gate(p);
  auto in = fresh_follower(100.0, 0.5, 0.0, 0.0);
  gate.update(in);

  in.now_s = 100.2;
  in.follower_stamp_s = in.now_s;
  in.odom_received = false;
  const auto d = gate.update(in);
  EXPECT_TRUE(d.limited);
  EXPECT_LE(std::fabs(d.cmd.lateral.steering_tire_angle_rad), 0.12);
}
