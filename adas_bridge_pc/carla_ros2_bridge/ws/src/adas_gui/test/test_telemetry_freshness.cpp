// telemetry_freshness.hpp 单测：markFresh / ageMs / isFresh 三段语义。
#include <gtest/gtest.h>

#include <QCoreApplication>
#include <QThread>

#include "telemetry_freshness.hpp"

namespace {

using adas::gui::TelemetryFreshness;

// 让 markFresh 与 isFresh 之间至少隔 5 ms，保证 ageMs() 跨过一个非零区间。
void sleep_ms(int ms) {
  QThread::msleep(static_cast<unsigned long>(ms));
}

class TelemetryFreshnessTest : public ::testing::Test {
 protected:
  void SetUp() override {
    TelemetryFreshness::instance().resetForTest();
  }
  void TearDown() override {
    TelemetryFreshness::instance().resetForTest();
  }
};

TEST_F(TelemetryFreshnessTest, UnreceivedChannelReturnsMinusOne) {
  auto& f = TelemetryFreshness::instance();
  EXPECT_EQ(f.ageMs(TelemetryFreshness::Mcu), -1);
  EXPECT_FALSE(f.isFresh(TelemetryFreshness::Mcu, 1000));
}

TEST_F(TelemetryFreshnessTest, MarkFreshMakesIsFreshTrue) {
  auto& f = TelemetryFreshness::instance();
  f.markFresh(TelemetryFreshness::Ego);
  // 1000 ms 限值下，刚 mark 的通道必然 fresh
  EXPECT_TRUE(f.isFresh(TelemetryFreshness::Ego, 1000));
  EXPECT_GE(f.ageMs(TelemetryFreshness::Ego), 0);
}

TEST_F(TelemetryFreshnessTest, StaleAfterLimit) {
  auto& f = TelemetryFreshness::instance();
  f.markFresh(TelemetryFreshness::Aeb);
  // 等 30 ms 后用 10 ms 限值判定，应该已 stale
  sleep_ms(30);
  EXPECT_FALSE(f.isFresh(TelemetryFreshness::Aeb, 10));
}

TEST_F(TelemetryFreshnessTest, ChannelsAreIndependent) {
  auto& f = TelemetryFreshness::instance();
  f.markFresh(TelemetryFreshness::Mcu);
  // 其他通道不应被影响
  EXPECT_TRUE(f.isFresh(TelemetryFreshness::Mcu, 1000));
  EXPECT_FALSE(f.isFresh(TelemetryFreshness::Actuation, 1000));
  EXPECT_FALSE(f.isFresh(TelemetryFreshness::Ego, 1000));
  EXPECT_FALSE(f.isFresh(TelemetryFreshness::Gate, 1000));
  EXPECT_FALSE(f.isFresh(TelemetryFreshness::Safety, 1000));
  EXPECT_FALSE(f.isFresh(TelemetryFreshness::Lane, 1000));
  EXPECT_FALSE(f.isFresh(TelemetryFreshness::Map, 1000));
  EXPECT_FALSE(f.isFresh(TelemetryFreshness::Route, 1000));
  EXPECT_FALSE(f.isFresh(TelemetryFreshness::Dtc, 1000));
}

TEST_F(TelemetryFreshnessTest, RereadFreshnessAfterLimitExceeds) {
  auto& f = TelemetryFreshness::instance();
  f.markFresh(TelemetryFreshness::Nav);
  sleep_ms(30);
  EXPECT_FALSE(f.isFresh(TelemetryFreshness::Nav, 10));
  f.markFresh(TelemetryFreshness::Nav);
  // 重新 mark 后立即 fresh
  EXPECT_TRUE(f.isFresh(TelemetryFreshness::Nav, 1000));
}

}  // namespace
