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
  as::SafetyMonitorCore core({}, 100.0);
  auto s = all_fresh(110.0);
  s.objects = 109.0;  // 1s 前 → 超 0.6s
  const auto r = core.evaluate(110.0, s);
  EXPECT_EQ(r.overall, as::SafetyLevel::kMrmComfort);
  ASSERT_EQ(r.failed_components.size(), 1u);
  EXPECT_EQ(r.failed_components[0], "objects");
}

TEST(SafetyMonitor, StaleTrajectoryComfortMrm) {
  as::SafetyMonitorCore core({}, 100.0);
  auto s = all_fresh(110.0);
  s.trajectory = 109.0;
  EXPECT_EQ(core.evaluate(110.0, s).overall, as::SafetyLevel::kMrmComfort);
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
  as::SafetyMonitorCore core({}, 100.0);
  auto s = all_fresh(110.0);
  s.objects = 109.0;
  EXPECT_EQ(core.evaluate(110.0, s).overall, as::SafetyLevel::kMrmComfort);
  // 通道恢复 → OK（自动恢复语义）
  EXPECT_EQ(core.evaluate(111.0, all_fresh(111.0)).overall, as::SafetyLevel::kOk);
}
