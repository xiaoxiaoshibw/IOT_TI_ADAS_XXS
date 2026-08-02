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
