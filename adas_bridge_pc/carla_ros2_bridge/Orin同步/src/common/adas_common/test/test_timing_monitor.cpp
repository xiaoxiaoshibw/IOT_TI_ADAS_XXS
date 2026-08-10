#include <gtest/gtest.h>

#include <chrono>
#include <limits>

#include "adas_common/parameter_validation.hpp"
#include "adas_common/timing_monitor.hpp"

TEST(TimingMonitor, TracksSamplesAndThresholds) {
  adas::common::TimingMonitor monitor(10.0);
  monitor.record_ms(2.0);
  monitor.record(std::chrono::milliseconds(8));

  const auto snapshot = monitor.snapshot();
  EXPECT_EQ(snapshot.samples, 2U);
  EXPECT_DOUBLE_EQ(snapshot.last_ms, 8.0);
  EXPECT_DOUBLE_EQ(snapshot.average_ms, 5.0);
  EXPECT_DOUBLE_EQ(snapshot.max_ms, 8.0);
  EXPECT_TRUE(snapshot.warning);
  EXPECT_FALSE(snapshot.error);

  monitor.record_ms(10.0);
  EXPECT_TRUE(monitor.snapshot().error);
}

TEST(TimingMonitor, ResetClearsRuntimeStatistics) {
  adas::common::TimingMonitor monitor(20.0);
  monitor.record_ms(4.0);
  monitor.reset();

  const auto snapshot = monitor.snapshot();
  EXPECT_EQ(snapshot.samples, 0U);
  EXPECT_DOUBLE_EQ(snapshot.last_ms, 0.0);
  EXPECT_DOUBLE_EQ(snapshot.max_ms, 0.0);
  EXPECT_DOUBLE_EQ(snapshot.budget_ms, 20.0);
}

TEST(ParameterValidation, RejectsInvalidValues) {
  EXPECT_THROW(adas::common::require_positive("rate_hz", 0.0), std::invalid_argument);
  EXPECT_THROW(adas::common::require_finite("value", std::numeric_limits<double>::infinity()),
               std::invalid_argument);
  EXPECT_THROW(adas::common::require_range("brake", 1.1, 0.0, 1.0), std::invalid_argument);
  EXPECT_THROW(adas::common::require_timeout_exceeds_period("timeout", 0.02, "rate", 50.0),
               std::invalid_argument);
}

TEST(ParameterValidation, AcceptsValidTiming) {
  EXPECT_NO_THROW(adas::common::require_timeout_exceeds_period("timeout", 0.2, "rate", 50.0));
}
