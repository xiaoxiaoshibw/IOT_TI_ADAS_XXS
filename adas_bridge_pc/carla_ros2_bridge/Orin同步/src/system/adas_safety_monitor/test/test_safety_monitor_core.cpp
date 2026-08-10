// SafetyMonitorCore 单测
#include <gtest/gtest.h>

#include <algorithm>

#include "adas_safety_monitor/safety_monitor_core.hpp"

namespace as = adas::system;

namespace {

as::ChannelStamps all_fresh(double now) {
  as::ChannelStamps s;
  s.odom = now - 0.01;
  s.objects = now - 0.02;
  s.trajectory = now - 0.05;
  s.follower_cmd = now - 0.01;
  s.gate_cmd = now - 0.01;
  return s;
}

as::SafetyMonitorParams hysteresis_params() {
  as::SafetyMonitorParams p;
  p.startup_grace_s = 0.0;
  p.stale_warn_threshold_s = 2.0;
  p.stale_mrm_threshold_s = 1.0;
  p.confirm_frames_to_escalate = 3;
  p.confirm_frames_to_recover = 3;
  return p;
}

}  // namespace

TEST(SafetyMonitor, GraceWindowAlwaysOk) {
  as::SafetyMonitorCore core({}, 100.0);  // 宽限 5s
  as::ChannelStamps never;                // 全部从未收到
  EXPECT_EQ(core.evaluate(103.0, never).overall, as::SafetyLevel::kOk);
}

TEST(SafetyMonitor, AllFreshOk) {
  as::SafetyMonitorCore core({}, 100.0);
  EXPECT_EQ(core.evaluate(110.0, all_fresh(110.0)).overall, as::SafetyLevel::kOk);
}

TEST(SafetyMonitor, StaleObjectsComfortMrm) {
  as::SafetyMonitorCore core(hysteresis_params(), 100.0);
  auto s = all_fresh(110.0);
  s.objects = 107.0;  // > MRM 边界
  core.evaluate(110.0, s);
  core.evaluate(110.1, s);
  const auto r = core.evaluate(110.2, s);
  EXPECT_EQ(r.overall, as::SafetyLevel::kMrmComfort);
  ASSERT_EQ(r.failed_components.size(), 1u);
  EXPECT_EQ(r.failed_components[0], "objects");
}

TEST(SafetyMonitor, StaleTrajectoryComfortMrm) {
  as::SafetyMonitorCore core(hysteresis_params(), 100.0);
  auto s = all_fresh(110.0);
  s.trajectory = 107.0;
  core.evaluate(110.0, s);
  core.evaluate(110.1, s);
  EXPECT_EQ(core.evaluate(110.2, s).overall, as::SafetyLevel::kMrmComfort);
}

TEST(SafetyMonitor, StaleGateCmdEmergency) {
  as::SafetyMonitorCore core({}, 100.0);
  auto s = all_fresh(110.0);
  s.gate_cmd = 109.0;
  const auto r = core.evaluate(110.0, s);
  EXPECT_EQ(r.overall, as::SafetyLevel::kMrmEmergency);
  EXPECT_NE(std::find(r.failed_components.begin(), r.failed_components.end(), "gate_cmd"),
            r.failed_components.end());
}

TEST(SafetyMonitor, RecoveryReturnsOk) {
  as::SafetyMonitorCore core(hysteresis_params(), 100.0);
  auto s = all_fresh(110.0);
  s.objects = 107.0;
  EXPECT_EQ(core.evaluate(110.0, s).overall, as::SafetyLevel::kWarn);
  EXPECT_EQ(core.evaluate(110.1, s).overall, as::SafetyLevel::kWarn);
  EXPECT_EQ(core.evaluate(110.2, s).overall, as::SafetyLevel::kMrmComfort);
  // MRM → WARN and WARN → OK each require three fresh frames.
  EXPECT_EQ(core.evaluate(111.0, all_fresh(111.0)).overall, as::SafetyLevel::kMrmComfort);
  EXPECT_EQ(core.evaluate(111.1, all_fresh(111.1)).overall, as::SafetyLevel::kMrmComfort);
  EXPECT_EQ(core.evaluate(111.2, all_fresh(111.2)).overall, as::SafetyLevel::kWarn);
  EXPECT_EQ(core.evaluate(111.3, all_fresh(111.3)).overall, as::SafetyLevel::kWarn);
  EXPECT_EQ(core.evaluate(111.4, all_fresh(111.4)).overall, as::SafetyLevel::kWarn);
  EXPECT_EQ(core.evaluate(111.5, all_fresh(111.5)).overall, as::SafetyLevel::kOk);
}

TEST(SafetyMonitorHysteresis, SingleWarnFrameDoesNotEscalate) {
  as::SafetyMonitorCore core(hysteresis_params(), 100.0);
  auto s = all_fresh(110.0);
  s.objects = 108.9;  // 1.1s age: WARN boundary crossed, MRM boundary not crossed.
  EXPECT_EQ(core.evaluate(110.0, s).overall, as::SafetyLevel::kWarn);
  EXPECT_EQ(core.evaluate(110.1, all_fresh(110.1)).overall, as::SafetyLevel::kWarn);
}

TEST(SafetyMonitorHysteresis, ThreeConsecutiveMrmFramesEscalate) {
  as::SafetyMonitorCore core(hysteresis_params(), 100.0);
  auto s = all_fresh(110.0);
  s.objects = 107.0;
  EXPECT_EQ(core.evaluate(110.0, s).overall, as::SafetyLevel::kWarn);
  EXPECT_EQ(core.evaluate(110.1, s).overall, as::SafetyLevel::kWarn);
  EXPECT_EQ(core.evaluate(110.2, s).overall, as::SafetyLevel::kMrmComfort);
}

TEST(SafetyMonitorHysteresis, WarnNeedsThreeFreshFramesToRecover) {
  as::SafetyMonitorCore core(hysteresis_params(), 100.0);
  auto s = all_fresh(110.0);
  s.objects = 108.9;
  EXPECT_EQ(core.evaluate(110.0, s).overall, as::SafetyLevel::kWarn);
  EXPECT_EQ(core.evaluate(110.1, all_fresh(110.1)).overall, as::SafetyLevel::kWarn);
  EXPECT_EQ(core.evaluate(110.2, all_fresh(110.2)).overall, as::SafetyLevel::kWarn);
  EXPECT_EQ(core.evaluate(110.3, all_fresh(110.3)).overall, as::SafetyLevel::kOk);
}

TEST(SafetyMonitorHysteresis, MrmNeedsThreeFreshFramesToDemoteToWarn) {
  as::SafetyMonitorCore core(hysteresis_params(), 100.0);
  auto s = all_fresh(110.0);
  s.objects = 107.0;
  core.evaluate(110.0, s);
  core.evaluate(110.1, s);
  ASSERT_EQ(core.evaluate(110.2, s).overall, as::SafetyLevel::kMrmComfort);
  EXPECT_EQ(core.evaluate(111.0, all_fresh(111.0)).overall, as::SafetyLevel::kMrmComfort);
  EXPECT_EQ(core.evaluate(111.1, all_fresh(111.1)).overall, as::SafetyLevel::kMrmComfort);
  EXPECT_EQ(core.evaluate(111.2, all_fresh(111.2)).overall, as::SafetyLevel::kWarn);
}

TEST(SafetyMonitorHysteresis, IntermittentJitterNeverEscalatesToMrm) {
  as::SafetyMonitorCore core(hysteresis_params(), 100.0);
  for (int i = 0; i < 100; ++i) {
    const double now = 110.0 + static_cast<double>(i) * 0.1;
    auto s = all_fresh(now);
    if ((i % 2) == 0) s.objects = now - 1.1;
    EXPECT_LT(static_cast<uint8_t>(core.evaluate(now, s).overall),
              static_cast<uint8_t>(as::SafetyLevel::kMrmComfort));
  }
}
