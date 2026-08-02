#include <gtest/gtest.h>

#include "adas_common/cycle_time_monitor.hpp"

TEST(CycleTimeMonitor, FirstTickUsesNominalPeriod) {
  adas::common::CycleTimeMonitor monitor(50.0);
  EXPECT_DOUBLE_EQ(monitor.tick_seconds(100.0), 0.02);
  EXPECT_EQ(monitor.snapshot().samples, 0U);
}

TEST(CycleTimeMonitor, MeasuresJitterAndAverage) {
  adas::common::CycleTimeMonitor monitor(50.0);
  monitor.tick_seconds(10.0);
  EXPECT_NEAR(monitor.tick_seconds(10.018), 0.018, 1e-12);
  EXPECT_NEAR(monitor.tick_seconds(10.040), 0.022, 1e-12);
  const auto stats = monitor.snapshot();
  EXPECT_EQ(stats.samples, 2U);
  EXPECT_NEAR(stats.average_raw_s, 0.020, 1e-12);
  EXPECT_NEAR(stats.max_abs_jitter_s, 0.002, 1e-12);
}

TEST(CycleTimeMonitor, ClampsSchedulerStallAndBackwardTime) {
  adas::common::CycleTimeMonitor monitor(50.0, 0.5, 2.0);
  monitor.tick_seconds(1.0);
  EXPECT_DOUBLE_EQ(monitor.tick_seconds(1.2), 0.04);
  EXPECT_DOUBLE_EQ(monitor.tick_seconds(1.1), 0.02);
  EXPECT_EQ(monitor.snapshot().clamped_samples, 2U);
}

TEST(CycleTimeMonitor, RejectsInvalidConfiguration) {
  EXPECT_THROW(adas::common::CycleTimeMonitor(0.0), std::invalid_argument);
  EXPECT_THROW(adas::common::CycleTimeMonitor(50.0, 2.0, 1.0), std::invalid_argument);
}

