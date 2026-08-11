#include <gtest/gtest.h>

#include <QApplication>
#include <QDateTime>
#include <QLabel>
#include <QWidget>

#include "safety_panel.hpp"
#include "telemetry_freshness.hpp"
#include "theme.hpp"

namespace adas::gui {
namespace {

QApplication& test_application() {
  qputenv("QT_QPA_PLATFORM", "offscreen");
  static int argc = 1;
  static char application_name[] = "test_safety_panel";
  static char* argv[] = {application_name, nullptr};
  static QApplication application(argc, argv);
  return application;
}

class SafetyPanelStaleTest : public ::testing::Test {
 protected:
  void SetUp() override {
    (void)test_application();
    TelemetryFreshness::instance().resetForTest();
  }

  void TearDown() override { TelemetryFreshness::instance().resetForTest(); }
};

TEST_F(SafetyPanelStaleTest, McuTimeoutMakesLiveAndCachedLinkStateUnknown) {
  SafetyPanel panel;
  GuiMcuStatus status;
  status.system_state = 2U;
  status.active_source = 1U;
  status.primary_fresh = true;
  status.feedback_age_s = 0.1F;
  status.protocol_version = 3U;
  status.protocol_version_ok = true;

  panel.onMcuStatus(status);

  auto* live = panel.findChild<QLabel*>(QStringLiteral("safetyLiveIndicator"));
  auto* can = panel.findChild<QLabel*>(QStringLiteral("safetyCanValue"));
  auto* source = panel.findChild<QLabel*>(QStringLiteral("safetySourceValue"));
  auto* primary = panel.findChild<QLabel*>(QStringLiteral("safetyPrimaryValue"));
  auto* protocol = panel.findChild<QLabel*>(QStringLiteral("safetyProtocolValue"));
  auto* handover = panel.findChild<QLabel*>(QStringLiteral("safetyHandoverValue"));
  auto* manual = panel.findChild<QLabel*>(QStringLiteral("safetyManualValue"));
  ASSERT_NE(live, nullptr);
  ASSERT_NE(can, nullptr);
  ASSERT_NE(source, nullptr);
  ASSERT_NE(primary, nullptr);
  ASSERT_NE(protocol, nullptr);
  ASSERT_NE(handover, nullptr);
  ASSERT_NE(manual, nullptr);
  EXPECT_EQ(live->text(), QStringLiteral("LIVE"));
  EXPECT_TRUE(live->property("telemetryFresh").toBool());
  EXPECT_EQ(can->text(), QStringLiteral("● 正常"));
  EXPECT_TRUE(can->property("linkOk").toBool());

  // 直接清空事实表，确定性模拟超过 freshness 阈值，无需在测试中 sleep。
  TelemetryFreshness::instance().resetForTest();
  panel.onStaleCheck(QDateTime::currentMSecsSinceEpoch());

  EXPECT_EQ(live->text(), QStringLiteral("STALE"));
  EXPECT_FALSE(live->property("telemetryFresh").toBool());
  EXPECT_EQ(can->text(), QStringLiteral("○ 未知"));
  EXPECT_FALSE(can->property("telemetryFresh").toBool());
  EXPECT_FALSE(can->property("linkOk").toBool());
  EXPECT_EQ(source->text(), QStringLiteral("--"));
  EXPECT_EQ(primary->text(), QStringLiteral("○ 未知"));
  EXPECT_EQ(protocol->text(), QStringLiteral("--"));
  EXPECT_EQ(handover->text(), QStringLiteral("--"));
  EXPECT_EQ(manual->text(), QStringLiteral("○ 未知"));
  EXPECT_TRUE(can->styleSheet().contains(QString::fromLatin1(theme::kStale)));
  EXPECT_TRUE(source->styleSheet().contains(QString::fromLatin1(theme::kStale)));

  // 新鲜消息到达后，LIVE 与缓存字段应恢复为当前消息，而不是保持灰态。
  panel.onMcuStatus(status);
  EXPECT_EQ(live->text(), QStringLiteral("LIVE"));
  EXPECT_EQ(can->text(), QStringLiteral("● 正常"));
  EXPECT_NE(source->text(), QStringLiteral("--"));
  EXPECT_TRUE(protocol->text().startsWith(QStringLiteral("v3")));
}

TEST_F(SafetyPanelStaleTest, FreshMcuStillReportsRealCanFailure) {
  SafetyPanel panel;
  GuiMcuStatus status;
  status.feedback_age_s = 0.8F;
  status.protocol_version_ok = true;

  panel.onMcuStatus(status);

  auto* can = panel.findChild<QLabel*>(QStringLiteral("safetyCanValue"));
  ASSERT_NE(can, nullptr);
  EXPECT_EQ(can->text(), QStringLiteral("○ 异常"));
  EXPECT_TRUE(can->property("telemetryFresh").toBool());
  EXPECT_FALSE(can->property("linkOk").toBool());
}

TEST_F(SafetyPanelStaleTest, ActuationTimeoutClearsBothBarsInsteadOfShowingFullBrake) {
  SafetyPanel panel;
  GuiActuation actuation;
  actuation.steer = 0.12F;
  actuation.throttle = 0.67F;
  actuation.brake = 0.24F;

  panel.onActuation(actuation);

  auto* throttle = panel.findChild<QWidget*>(QStringLiteral("safetyThrottleBar"));
  auto* brake = panel.findChild<QWidget*>(QStringLiteral("safetyBrakeBar"));
  auto* steer = panel.findChild<QLabel*>(QStringLiteral("safetySteerValue"));
  ASSERT_NE(throttle, nullptr);
  ASSERT_NE(brake, nullptr);
  ASSERT_NE(steer, nullptr);
  EXPECT_EQ(throttle->property("displayValue").toInt(), 67);
  EXPECT_EQ(brake->property("displayValue").toInt(), 24);
  EXPECT_TRUE(brake->property("telemetryFresh").toBool());

  TelemetryFreshness::instance().resetForTest();
  panel.onStaleCheck(QDateTime::currentMSecsSinceEpoch());

  EXPECT_EQ(throttle->property("displayValue").toInt(), 0);
  EXPECT_EQ(brake->property("displayValue").toInt(), 0);
  EXPECT_FALSE(throttle->property("telemetryFresh").toBool());
  EXPECT_FALSE(brake->property("telemetryFresh").toBool());
  EXPECT_EQ(steer->text(), QStringLiteral("-- %"));
  EXPECT_TRUE(steer->styleSheet().contains(QString::fromLatin1(theme::kStale)));
  EXPECT_TRUE(brake->toolTip().contains(QStringLiteral("未知")));
}

TEST_F(SafetyPanelStaleTest, AuxiliaryCachedTelemetryIsGreyedWhenItExpires) {
  SafetyPanel panel;
  panel.onBehavior(1, 8.0, 0);
  panel.onGate(1, false, QString());
  panel.onAeb(1, 4.0, 0.0);
  panel.onSafety(0, QString());
  panel.onEgo(1.0, 2.0, 0.1, 5.0, 0.02);
  panel.onLaneState(0.15, true);

  TelemetryFreshness::instance().resetForTest();
  panel.onStaleCheck(QDateTime::currentMSecsSinceEpoch());

  const QStringList names = {
      QStringLiteral("safetyBehaviorValue"), QStringLiteral("safetyGateValue"),
      QStringLiteral("safetyAebValue"), QStringLiteral("safetyTtcValue"),
      QStringLiteral("safetyChainValue"), QStringLiteral("safetySpeedValue"),
      QStringLiteral("safetyPoseValue"), QStringLiteral("safetyLateralValue")};
  for (const QString& name : names) {
    auto* value = panel.findChild<QLabel*>(name);
    ASSERT_NE(value, nullptr) << name.toStdString();
    EXPECT_TRUE(value->styleSheet().contains(QString::fromLatin1(theme::kStale)))
        << name.toStdString();
  }
  EXPECT_EQ(panel.findChild<QLabel*>(QStringLiteral("safetySpeedValue"))->text(),
            QStringLiteral("-- km/h"));
  EXPECT_EQ(panel.findChild<QLabel*>(QStringLiteral("safetyPoseValue"))->text(),
            QStringLiteral("--"));
}

}  // namespace
}  // namespace adas::gui
