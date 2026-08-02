// adas_common 变化率限幅器单测
#include <gtest/gtest.h>

#include "adas_common/rate_limiter.hpp"

namespace ac = adas::common;

TEST(RateLimiter, LimitsRise) {
  ac::RateLimiter rl(1.0);  // 1 单位/秒
  EXPECT_NEAR(rl.update(10.0, 0.1), 0.1, 1e-9);
  EXPECT_NEAR(rl.update(10.0, 0.1), 0.2, 1e-9);
}

TEST(RateLimiter, AsymmetricFall) {
  ac::RateLimiter rl(1.0, 5.0, 1.0);  // 升 1/s、降 5/s，初值 1
  EXPECT_NEAR(rl.update(-10.0, 0.1), 0.5, 1e-9);
}

TEST(RateLimiter, ReachesTargetWithinRate) {
  ac::RateLimiter rl(10.0);
  EXPECT_NEAR(rl.update(0.5, 0.1), 0.5, 1e-9);  // 一步可达
}

TEST(RateLimiter, ResetAndTightenRates) {
  ac::RateLimiter rl(10.0);
  rl.reset(2.0);
  EXPECT_NEAR(rl.value(), 2.0, 1e-12);
  rl.set_rates(0.1, 0.1);  // 接管瞬态收紧
  EXPECT_NEAR(rl.update(10.0, 1.0), 2.1, 1e-9);
}
