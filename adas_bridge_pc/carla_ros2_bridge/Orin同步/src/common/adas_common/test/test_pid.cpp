// adas_common PID 单测
#include <gtest/gtest.h>

#include "adas_common/pid.hpp"

namespace ac = adas::common;

TEST(Pid, ProportionalOnly) {
  ac::Pid pid({2.0, 0.0, 0.0}, -10.0, 10.0, -1.0, 1.0);
  EXPECT_NEAR(pid.update(1.5, 0.02), 3.0, 1e-9);
}

TEST(Pid, OutputClamped) {
  ac::Pid pid({100.0, 0.0, 0.0}, -1.0, 1.0, -1.0, 1.0);
  EXPECT_NEAR(pid.update(5.0, 0.02), 1.0, 1e-9);
  EXPECT_NEAR(pid.update(-5.0, 0.02), -1.0, 1e-9);
}

TEST(Pid, IntegralAccumulatesAndClamps) {
  ac::Pid pid({0.0, 1.0, 0.0}, -10.0, 10.0, -0.5, 0.5);
  // 持续正误差：积分应累积但不超过上限 0.5
  double out = 0.0;
  for (int i = 0; i < 1000; ++i) {
    out = pid.update(1.0, 0.01);
  }
  EXPECT_NEAR(out, 0.5, 1e-9);
}

TEST(Pid, AntiWindupWhenSaturated) {
  // 输出饱和期间积分不应无界增长；退出饱和后应快速响应反向误差
  ac::Pid pid({1.0, 10.0, 0.0}, -1.0, 1.0, -100.0, 100.0);
  for (int i = 0; i < 500; ++i) {
    pid.update(2.0, 0.01);
  }
  // 饱和期间积分被回退：远小于无抗饱和时的 2.0*0.01*500=10
  EXPECT_LT(pid.integral(), 1.0);
}

TEST(Pid, ResetClearsState) {
  ac::Pid pid({1.0, 1.0, 1.0}, -10.0, 10.0, -1.0, 1.0);
  pid.update(1.0, 0.01);
  pid.reset();
  EXPECT_NEAR(pid.integral(), 0.0, 1e-12);
}

TEST(Pid, ZeroDtReturnsLastOutput) {
  ac::Pid pid({2.0, 0.0, 0.0}, -10.0, 10.0, -1.0, 1.0);
  const double a = pid.update(1.0, 0.02);
  EXPECT_NEAR(pid.update(99.0, 0.0), a, 1e-12);
}

// === Commit 6b: 积分冻结带 (|error| < freeze_band 时跳过积分) =================

TEST(Pid, FreezeBandSkipsIntegration) {
  // freeze_band=0.5, ki=10.0：持续 |error|=0.1（小于 band）应不累计积分。
  ac::Pid pid({0.0, 10.0, 0.0}, -10.0, 10.0, -100.0, 100.0, 0.5);
  for (int i = 0; i < 1000; ++i) {
    pid.update(0.1, 0.01);
  }
  EXPECT_NEAR(pid.integral(), 0.0, 1e-9);
}

TEST(Pid, FreezeBandAllowsLargeErrorIntegration) {
  // 同一 PID，|error|=1.0（> 0.5）应正常累计积分。ki=10, out_max=10 →
  // 抗饱和把 integral 钳在 ≈ out_max/ki = 1.0（实际 0.99，因刚好到达 out_max
  // 那一拍回退一次 dt）。
  ac::Pid pid({0.0, 10.0, 0.0}, -10.0, 10.0, -100.0, 100.0, 0.5);
  for (int i = 0; i < 1000; ++i) {
    pid.update(1.0, 0.01);
  }
  EXPECT_NEAR(pid.integral(), 0.99, 1e-6);
}

TEST(Pid, ZeroFreezeBandDisablesFreezing) {
  // 默认 freeze_band=0 → 行为与 Commit 6b 前一致：小误差也累计
  ac::Pid pid({0.0, 10.0, 0.0}, -10.0, 10.0, -100.0, 100.0, 0.0);
  for (int i = 0; i < 100; ++i) {
    pid.update(0.1, 0.01);
  }
  EXPECT_NEAR(pid.integral(), 0.1, 1e-6);
}
