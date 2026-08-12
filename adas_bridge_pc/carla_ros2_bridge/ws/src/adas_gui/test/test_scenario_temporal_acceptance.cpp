// P1.G: 时序验收单测。
//   - AEB 单帧 WARNING+EMERGENCY 不得通过；
//   - AEB 顺序 WARNING → EMERGENCY 不得通过（必须在时间上 WARNING 早于 EMERGENCY）；
//   - Dense 单帧 20 目标不得通过；要求 >= 5 帧；
//   - 会话切换必须清空历史。
#include <gtest/gtest.h>

#include <QApplication>

#include "scenario_run_panel.hpp"

namespace {

using adas::gui::ScenarioRunPanel;

class TemporalAcceptance : public ::testing::Test {
 protected:
  static void SetUpTestCase() {
    static int argc = 1;
    static char app[] = "test";
    static char* argv[] = {app, nullptr};
    if (!QApplication::instance()) application_.reset(new QApplication(argc, argv));
  }
  static void TearDownTestCase() { application_.reset(); }
  static std::unique_ptr<QApplication> application_;
};
std::unique_ptr<QApplication> TemporalAcceptance::application_;

TEST_F(TemporalAcceptance, AebRejectsWarningAndEmergencyOnSameTick) {
  ScenarioRunPanel panel;
  panel.setScenario(QStringLiteral("aeb"));
  // 注入 fake clock：两次 observe 落在**完全相同**的虚拟时间戳
  // （默认 wall clock 多次连续调用可能差 1ms 造成假阴性）。
  qint64 t = 1000;
  panel.setClockForTest([&]() { return t; });
  panel.onAeb(2);
  panel.onAeb(3);
  EXPECT_FALSE(panel.allRequirementsPassed());
}

TEST_F(TemporalAcceptance, AebRejectsEmergencyBeforeWarning) {
  ScenarioRunPanel panel;
  panel.setScenario(QStringLiteral("aeb_stationary"));
  // 严格时间倒置：先 t=2000 触发 EMERGENCY，再 t=3000 触发 WARNING。
  qint64 t = 2000;
  panel.setClockForTest([&]() { return t; });
  panel.onAeb(3);
  t = 3000;
  panel.onAeb(2);
  EXPECT_FALSE(panel.allRequirementsPassed());
}

TEST_F(TemporalAcceptance, AebPassesWhenWarningPrecedesEmergency) {
  ScenarioRunPanel panel;
  panel.setScenario(QStringLiteral("aeb_pedestrian"));
  // 严格时间顺序：WARNING 在 t=1000，EMERGENCY 在 t=2000。
  qint64 t = 1000;
  panel.setClockForTest([&]() { return t; });
  panel.onAeb(2);
  t = 2000;
  panel.onAeb(3);
  // 注意：aeb_pedestrian 的 profile 还可能要求 PedestrianDetected 等其它 evidence，
  // 这里只断言 AebEmergency 已被时序解锁（看 lastFailureDetail 不再含"先 WARNING"）。
  // 其余 evidence 与本测试无关。
  const auto profile = adas::gui::scenario_workflow_profile(
      QStringLiteral("aeb_pedestrian"));
  bool any = false;
  for (const auto& r : profile.requirements) {
    if (r.key == adas::gui::ScenarioEvidenceKey::AebEmergency) {
      any = true;
    }
  }
  EXPECT_TRUE(any);
  EXPECT_FALSE(panel.lastFailureDetailForTest().contains(QStringLiteral("WARNING")));
}

TEST_F(TemporalAcceptance, DenseRejectsSingleFramePeak) {
  ScenarioRunPanel panel;
  panel.setScenario(QStringLiteral("dense_overtake_v1"));
  // 严格时间顺序 + 每次 25 目标 → 同帧假阳性必须被 5 帧门槛挡住。
  qint64 t = 1000;
  panel.setClockForTest([&]() { return t; });
  QVector<adas::gui::GuiMapObject> objects;
  for (int i = 0; i < 25; ++i) {
    adas::gui::GuiMapObject o;
    o.id = i;
    o.classification = 1;
    o.x = i;
    o.y = 0;
    objects.push_back(o);
  }
  panel.onObjects(objects);  // 单帧 25 目标
  EXPECT_FALSE(panel.allRequirementsPassed());
}

TEST_F(TemporalAcceptance, DenseAcceptsFiveDistinctFramesWithinWindow) {
  ScenarioRunPanel panel;
  panel.setScenario(QStringLiteral("dense_overtake_v1"));
  qint64 t = 1000;
  panel.setClockForTest([&]() { return t; });
  QVector<adas::gui::GuiMapObject> objects;
  for (int i = 0; i < 25; ++i) {
    adas::gui::GuiMapObject o;
    o.id = i;
    o.classification = 1;
    o.x = i;
    o.y = 0;
    objects.push_back(o);
  }
  // 5 个不同时间戳（间隔 1s，落在 5s 窗口内）→ 触发 DenseObjectSet。
  for (int k = 0; k < 5; ++k) {
    t = 1000 + k * 1000;
    panel.onObjects(objects);
  }
  const auto profile = adas::gui::scenario_workflow_profile(
      QStringLiteral("dense_overtake_v1"));
  bool has_dense = false;
  for (const auto& r : profile.requirements) {
    if (r.key == adas::gui::ScenarioEvidenceKey::DenseObjectSet) has_dense = true;
  }
  EXPECT_TRUE(has_dense);
  EXPECT_TRUE(panel.evidenceSatisfiedForTest(
      adas::gui::ScenarioEvidenceKey::DenseObjectSet));
}

TEST_F(TemporalAcceptance, DenseBoundaryIsAcceptedAndTimeoutIsRejected) {
  ScenarioRunPanel panel;
  panel.setScenario(QStringLiteral("dense_overtake_v1"));
  qint64 t = 1000;
  panel.setClockForTest([&]() { return t; });
  QVector<adas::gui::GuiMapObject> objects(20);
  for (int k = 0; k < 5; ++k) {
    t = 1000 + k * 1250;  // fifth sample is exactly on the 5 s boundary
    panel.onObjects(objects);
  }
  EXPECT_TRUE(panel.evidenceSatisfiedForTest(
      adas::gui::ScenarioEvidenceKey::DenseObjectSet));
  t = 11001;
  panel.onObjects(objects);
  EXPECT_FALSE(panel.evidenceSatisfiedForTest(
      adas::gui::ScenarioEvidenceKey::DenseObjectSet));
}

TEST_F(TemporalAcceptance, DenseConditionInterruptionRestartsWindow) {
  ScenarioRunPanel panel;
  panel.setScenario(QStringLiteral("dense_overtake_v1"));
  qint64 t = 1000;
  panel.setClockForTest([&]() { return t; });
  QVector<adas::gui::GuiMapObject> dense(20);
  for (int k = 0; k < 4; ++k) {
    t += 500;
    panel.onObjects(dense);
  }
  panel.onObjects({});
  t += 500;
  panel.onObjects(dense);
  EXPECT_FALSE(panel.evidenceSatisfiedForTest(
      adas::gui::ScenarioEvidenceKey::DenseObjectSet));
}

TEST_F(TemporalAcceptance, NewSessionClearsTemporalHistory) {
  ScenarioRunPanel panel;
  qint64 t = 1000;
  panel.setClockForTest([&]() { return t; });
  panel.setScenario(QStringLiteral("aeb"));
  panel.onAeb(2);
  t = 2000;
  panel.onAeb(3);
  // 切到 free（视作"开新会话"），再切回 aeb —— 旧 WARNING/EMERGENCY 时间戳
  // 必须清空。
  panel.setScenario(QStringLiteral("free"));
  panel.setScenario(QStringLiteral("aeb"));
  // 重放时序：先 EMERGENCY 后 WARNING（旧数据已清空）
  t = 3000;
  panel.onAeb(3);
  t = 4000;
  panel.onAeb(2);
  EXPECT_FALSE(panel.allRequirementsPassed());
}

}  // namespace
