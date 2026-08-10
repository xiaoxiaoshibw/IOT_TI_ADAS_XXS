// ArbiterCore 单测
#include <gtest/gtest.h>

#include <cmath>

#include "adas_redundancy/arbiter_core.hpp"

namespace as = adas::system;
namespace ac = adas::common;

namespace {

as::StackObservation stack(double now, double steer, double accel, bool nominal,
                           bool aeb = false) {
  as::StackObservation s;
  s.received = true;
  s.stamp_s = now - 0.005;
  s.cmd.lateral.steering_tire_angle_rad = steer;
  s.cmd.longitudinal.acceleration_mps2 = accel;
  s.nominal = nominal;
  s.aeb_active = aeb;
  return s;
}

as::ArbiterInputs both_nominal(double now) {
  as::ArbiterInputs in;
  in.now_s = now;
  in.dt = 0.01;
  in.primary = stack(now, 0.05, 0.5, true);
  in.backup = stack(now, 0.05, 0.5, true);
  return in;
}

}  // namespace

TEST(ArbiterCore, PrefersNominalPrimary) {
  as::ArbiterCore core((as::ArbiterParams()));
  const auto d = core.update(both_nominal(100.0));
  EXPECT_TRUE(d.valid);
  EXPECT_EQ(d.role, as::ActiveRole::kPrimary);
}

TEST(ArbiterCore, PrimaryLostSwitchesToBackup) {
  as::ArbiterCore core((as::ArbiterParams()));
  core.update(both_nominal(100.0));
  auto in = both_nominal(100.5);
  in.primary.stamp_s = 100.0;  // 0.5s 前 → 超时
  const auto d = core.update(in);
  EXPECT_EQ(d.role, as::ActiveRole::kBackup);
  EXPECT_TRUE(d.takeover_guard);
}

TEST(ArbiterCore, PrimaryDegradedSwitchesToNominalBackup) {
  as::ArbiterCore core((as::ArbiterParams()));
  core.update(both_nominal(100.0));
  // 主栈仍在发流但已降级（builtin_stop），备栈正常 → 无缝切备
  auto in = both_nominal(100.1);
  in.primary.nominal = false;
  const auto d = core.update(in);
  EXPECT_EQ(d.role, as::ActiveRole::kBackup);
  EXPECT_EQ(d.reason, "primary_degraded");
}

TEST(ArbiterCore, BothDegradedFollowsPrimaryStop) {
  as::ArbiterCore core((as::ArbiterParams()));
  core.update(both_nominal(100.0));
  auto in = both_nominal(100.1);
  in.primary.nominal = false;
  in.backup.nominal = false;
  const auto d = core.update(in);
  EXPECT_TRUE(d.valid);
  EXPECT_EQ(d.role, as::ActiveRole::kPrimary);  // 双降级：跟主栈停车
}

TEST(ArbiterCore, AllLostInvalid) {
  as::ArbiterCore core((as::ArbiterParams()));
  core.update(both_nominal(100.0));
  as::ArbiterInputs in;
  in.now_s = 101.0;
  in.dt = 0.01;
  const auto d = core.update(in);
  EXPECT_FALSE(d.valid);
  EXPECT_EQ(d.reason, "all_sources_lost");
}

TEST(ArbiterCore, TakeoverTransientIsRateLimited) {
  as::ArbiterParams p;
  p.guard_steer_rate_rps = 0.25;
  as::ArbiterCore core(p);
  // 主栈转角 0.0 稳定输出
  for (int i = 0; i < 10; ++i) {
    auto in = both_nominal(100.0 + 0.01 * i);
    in.primary.cmd.lateral.steering_tire_angle_rad = 0.0;
    core.update(in);
  }
  // 主失联，备栈要 0.3rad → 接管瞬态单拍变化 ≤ 0.25*0.01
  auto in = both_nominal(100.2);
  in.primary.stamp_s = 100.0;
  in.backup.cmd.lateral.steering_tire_angle_rad = 0.3;
  const auto d = core.update(in);
  EXPECT_EQ(d.role, as::ActiveRole::kBackup);
  EXPECT_LE(std::fabs(d.cmd.lateral.steering_tire_angle_rad), 0.25 * 0.01 + 1e-9);
}

TEST(ArbiterCore, AebTakeoverAllowsFastBrakeDeepening) {
  as::ArbiterParams p;
  p.guard_accel_rate_mps3 = 4.0;
  p.aeb_brake_rate_mps3 = 12.0;
  as::ArbiterCore core(p);
  for (int i = 0; i < 10; ++i) {
    core.update(both_nominal(100.0 + 0.01 * i));
  }
  // 主失联；备栈 AEB 紧急制动 -8 → 制动加深速率放开（12 而非 4）
  auto in = both_nominal(100.2);
  in.primary.stamp_s = 100.0;
  in.backup.cmd.longitudinal.acceleration_mps2 = -8.0;
  in.backup.aeb_active = true;
  const auto d = core.update(in);
  const double da = d.cmd.longitudinal.acceleration_mps2 - 0.5;  // 上一输出 0.5
  EXPECT_LT(da, -4.0 * 0.01);            // 快于普通瞬态速率
  EXPECT_GE(da, -12.0 * 0.01 - 1e-9);    // 但不超 AEB 放开速率
}

TEST(ArbiterCore, RecoveryRequiresStableWindow) {
  as::ArbiterParams p;
  p.recover_stable_s = 0.5;
  as::ArbiterCore core(p);
  core.update(both_nominal(100.0));
  // 切到备
  auto in = both_nominal(100.1);
  in.primary.nominal = false;
  core.update(in);
  ASSERT_EQ(core.role(), as::ActiveRole::kBackup);
  // 主恢复正常 0.2s：仍在备
  for (int i = 0; i < 20; ++i) {
    core.update(both_nominal(100.2 + 0.01 * i));
  }
  EXPECT_EQ(core.role(), as::ActiveRole::kBackup);
  // 恢复满 0.5s → 回切主
  for (int i = 0; i < 40; ++i) {
    core.update(both_nominal(100.4 + 0.01 * i));
  }
  EXPECT_EQ(core.role(), as::ActiveRole::kPrimary);
}
