#include <gtest/gtest.h>

#include "adas_can_gateway/feedback_monitor.hpp"

namespace acg = adas::can_gateway;

namespace {

acg::FeedbackMonitorConfig config() {
  acg::FeedbackMonitorConfig result;
  result.steer_tolerance_deg = 5.0;
  result.accel_tolerance_mps2 = 2.0;
  result.deviation_hold_s = 0.5;
  result.feedback_timeout_s = 0.2;
  result.recovery_hold_s = 1.0;
  return result;
}

}  // namespace

TEST(FeedbackMonitor, MatchingFeedbackNeverFaults) {
  acg::FeedbackMonitor monitor(config());
  for (int i = 0; i <= 200; ++i) {
    monitor.update(i * 0.01, true, true, true, 10.0, 9.0, 1.0, 0.8);
  }
  EXPECT_FALSE(monitor.any_fault());
}

TEST(FeedbackMonitor, TransientDeviationWithinHoldIsAbsorbed) {
  acg::FeedbackMonitor monitor(config());
  // 0.4s 的偏差（如 slew 暂态），低于 0.5s 保持时间
  for (int i = 0; i < 40; ++i)
    monitor.update(i * 0.01, true, true, true, 20.0, 0.0, 0.0, 0.0);
  for (int i = 40; i < 60; ++i)
    monitor.update(i * 0.01, true, true, true, 20.0, 19.0, 0.0, 0.0);
  EXPECT_FALSE(monitor.deviation_fault());
}

TEST(FeedbackMonitor, SustainedSteerDeviationLatches) {
  acg::FeedbackMonitor monitor(config());
  for (int i = 0; i <= 60; ++i)
    monitor.update(i * 0.01, true, true, true, 20.0, 2.0, 0.0, 0.0);
  EXPECT_TRUE(monitor.deviation_fault());
  // 偏差消失也不允许自动恢复（执行器失配需人工复位）
  for (int i = 61; i <= 300; ++i)
    monitor.update(i * 0.01, true, true, true, 20.0, 20.0, 0.0, 0.0);
  EXPECT_TRUE(monitor.deviation_fault());
}

TEST(FeedbackMonitor, SustainedAccelDeviationLatches) {
  acg::FeedbackMonitor monitor(config());
  for (int i = 0; i <= 60; ++i)
    monitor.update(i * 0.01, true, true, true, 0.0, 0.0, 3.0, 0.0);
  EXPECT_TRUE(monitor.deviation_fault());
}

TEST(FeedbackMonitor, IntentionalDeviationInMrmDoesNotFault) {
  acg::FeedbackMonitor monitor(config());
  // MRM/AEB 下 MCU 有意偏离命令：deviation_eligible=false，但链路仍活跃
  for (int i = 0; i <= 200; ++i)
    monitor.update(i * 0.01, true, false, true, 20.0, 0.0, 2.0, -5.0);
  EXPECT_FALSE(monitor.any_fault());
}

TEST(FeedbackMonitor, FeedbackTimeoutFaultsAndRecoversAfterStableHold) {
  acg::FeedbackMonitor monitor(config());
  monitor.update(0.0, true, true, true, 0.0, 0.0, 0.0, 0.0);
  for (int i = 1; i <= 30; ++i)
    monitor.update(i * 0.01, true, true, false, 0.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(monitor.timeout_fault());
  // 反馈恢复后需要稳定保持 recovery_hold_s 才清除
  for (int i = 31; i <= 80; ++i)
    monitor.update(i * 0.01, true, true, true, 0.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(monitor.timeout_fault());
  for (int i = 81; i <= 140; ++i)
    monitor.update(i * 0.01, true, true, true, 0.0, 0.0, 0.0, 0.0);
  EXPECT_FALSE(monitor.timeout_fault());
}

TEST(FeedbackMonitor, TimeoutRecoversEvenAfterAuthorityWasDropped) {
  acg::FeedbackMonitor monitor(config());
  // 超时故障触发后网关撤权，MCU 离开 ACTIVE → deviation_eligible=false，
  // 但链路仍活跃：反馈恢复必须仍能清除超时故障，不得死锁。
  for (int i = 0; i <= 30; ++i)
    monitor.update(i * 0.01, true, true, false, 0.0, 0.0, 0.0, 0.0);
  ASSERT_TRUE(monitor.timeout_fault());
  for (int i = 31; i <= 200; ++i)
    monitor.update(i * 0.01, true, false, true, 0.0, 0.0, 0.0, 0.0);
  EXPECT_FALSE(monitor.timeout_fault());
}

TEST(FeedbackMonitor, ResetClearsLatchedDeviationFault) {
  acg::FeedbackMonitor monitor(config());
  for (int i = 0; i <= 60; ++i)
    monitor.update(i * 0.01, true, true, true, 20.0, 2.0, 0.0, 0.0);
  ASSERT_TRUE(monitor.deviation_fault());
  // 显式软复位（reset_actuator_fault 服务）取代进程重启
  monitor.reset();
  EXPECT_FALSE(monitor.any_fault());
  // 复位后监控从干净状态继续：匹配反馈不再误报
  for (int i = 61; i <= 120; ++i)
    monitor.update(i * 0.01, true, true, true, 20.0, 20.0, 0.0, 0.0);
  EXPECT_FALSE(monitor.any_fault());
}

TEST(FeedbackMonitor, ResetClearsTimeoutFaultAndReEvaluatesCleanly) {
  acg::FeedbackMonitor monitor(config());
  for (int i = 0; i <= 30; ++i)
    monitor.update(i * 0.01, true, true, false, 0.0, 0.0, 0.0, 0.0);
  ASSERT_TRUE(monitor.timeout_fault());
  monitor.reset();
  EXPECT_FALSE(monitor.timeout_fault());
}

TEST(FeedbackMonitor, ResetDoesNotMaskAStillPresentDeviation) {
  acg::FeedbackMonitor monitor(config());
  for (int i = 0; i <= 60; ++i)
    monitor.update(i * 0.01, true, true, true, 20.0, 2.0, 0.0, 0.0);
  ASSERT_TRUE(monitor.deviation_fault());
  monitor.reset();
  ASSERT_FALSE(monitor.deviation_fault());
  // 失配未排除：复位后应在 deviation_hold_s 内重新锁存，不被永久掩盖
  for (int i = 61; i <= 130; ++i)
    monitor.update(i * 0.01, true, true, true, 20.0, 2.0, 0.0, 0.0);
  EXPECT_TRUE(monitor.deviation_fault());
}

TEST(FeedbackMonitor, LatchedDeviationSurvivesInactivePhases) {
  acg::FeedbackMonitor monitor(config());
  for (int i = 0; i <= 60; ++i)
    monitor.update(i * 0.01, true, true, true, 20.0, 2.0, 0.0, 0.0);
  ASSERT_TRUE(monitor.deviation_fault());
  // 链路整体停用（如上游停车、MCU 掉线）期间锁存不丢失
  for (int i = 61; i <= 200; ++i)
    monitor.update(i * 0.01, false, false, true, 0.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(monitor.deviation_fault());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
