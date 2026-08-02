#include <gtest/gtest.h>

#include "adas_can_gateway/startup_gate.hpp"

namespace acg = adas::can_gateway;

namespace {

acg::StartupGate make_gate(double stable_s = 0.5) {
  acg::StartupGateConfig config;
  config.inputs_stable_s = stable_s;
  return acg::StartupGate(config);
}

// 以 10ms 节拍推进门控。
void run_ticks(acg::StartupGate& gate, double& now_s, int ticks,
               bool inputs_fresh, bool deadline_ok) {
  for (int i = 0; i < ticks; ++i) {
    now_s += 0.01;
    gate.update(now_s, inputs_fresh, deadline_ok);
  }
}

}  // namespace

TEST(StartupGate, ColdStartHoldsStandbyUntilInputsStable) {
  auto gate = make_gate();
  double now_s = 100.0;
  gate.update(now_s, true, true);
  EXPECT_FALSE(gate.active());
  // 490ms 仍不放行
  run_ticks(gate, now_s, 49, true, true);
  EXPECT_FALSE(gate.active());
  // 满 500ms 后在节拍边界统一放行
  run_ticks(gate, now_s, 1, true, true);
  EXPECT_TRUE(gate.active());
  EXPECT_EQ(gate.rearm_count(), 0U);
}

TEST(StartupGate, FreshnessDropoutRestartsStableWindow) {
  auto gate = make_gate();
  double now_s = 0.0;
  gate.update(now_s, true, true);
  run_ticks(gate, now_s, 30, true, true);   // 300ms 新鲜
  run_ticks(gate, now_s, 1, false, true);   // 一拍陈旧
  run_ticks(gate, now_s, 50, true, true);   // 重启后的窗口从首个新鲜拍起算
  EXPECT_FALSE(gate.active());
  run_ticks(gate, now_s, 1, true, true);
  EXPECT_TRUE(gate.active());
}

TEST(StartupGate, DeadlineMissDuringStartupRestartsStableWindow) {
  auto gate = make_gate();
  double now_s = 0.0;
  gate.update(now_s, true, true);
  run_ticks(gate, now_s, 40, true, true);
  run_ticks(gate, now_s, 1, true, false);   // 稳定窗内发送超期
  run_ticks(gate, now_s, 50, true, true);
  EXPECT_FALSE(gate.active());
  run_ticks(gate, now_s, 1, true, true);
  EXPECT_TRUE(gate.active());
  EXPECT_FALSE(gate.ever_active() && !gate.active());
}

TEST(StartupGate, DeadlineMissWhileActiveDemotesAndRequiresFullWindow) {
  auto gate = make_gate();
  double now_s = 0.0;
  gate.update(now_s, true, true);
  run_ticks(gate, now_s, 50, true, true);
  ASSERT_TRUE(gate.active());
  // 模拟线程暂停后的首个节拍：deadline 超期
  run_ticks(gate, now_s, 1, true, false);
  EXPECT_FALSE(gate.active());
  EXPECT_TRUE(gate.ever_active());
  EXPECT_TRUE(gate.last_rearm_was_deadline());
  EXPECT_EQ(gate.rearm_count(), 1U);
  // 重新走完整稳定窗
  run_ticks(gate, now_s, 50, true, true);
  EXPECT_FALSE(gate.active());
  run_ticks(gate, now_s, 1, true, true);
  EXPECT_TRUE(gate.active());
}

TEST(StartupGate, StaleInputsWhileActiveDemote) {
  auto gate = make_gate();
  double now_s = 0.0;
  gate.update(now_s, true, true);
  run_ticks(gate, now_s, 50, true, true);
  ASSERT_TRUE(gate.active());
  run_ticks(gate, now_s, 1, false, true);
  EXPECT_FALSE(gate.active());
  EXPECT_TRUE(gate.ever_active());
  EXPECT_FALSE(gate.last_rearm_was_deadline());
}

TEST(StartupGate, ActiveDurationTracksArmingWindow) {
  auto gate = make_gate();
  double now_s = 0.0;
  gate.update(now_s, true, true);
  run_ticks(gate, now_s, 50, true, true);
  ASSERT_TRUE(gate.active());
  EXPECT_DOUBLE_EQ(gate.active_duration_s(now_s), 0.0);
  EXPECT_LT(gate.active_duration_s(now_s + 0.5), 1.0);
  EXPECT_GE(gate.active_duration_s(now_s + 1.0), 1.0);
  // 未 ACTIVE 时时长为 0
  gate.update(now_s + 1.0, true, false);
  EXPECT_DOUBLE_EQ(gate.active_duration_s(now_s + 1.0), 0.0);
}
