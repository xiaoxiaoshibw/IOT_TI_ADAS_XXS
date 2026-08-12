#ifndef ADAS_GUI__SCENARIO_RUN_PANEL_HPP_
#define ADAS_GUI__SCENARIO_RUN_PANEL_HPP_

#include <QDateTime>
#include <QMap>
#include <QSet>
#include <QVector>
#include <QWidget>

#include <functional>
#include <chrono>

#include "map_view.hpp"
#include "scenario_workflow.hpp"

class QLabel;
class QHBoxLayout;

namespace adas::gui {

class ScenarioRunPanel : public QWidget {
  Q_OBJECT

 public:
  explicit ScenarioRunPanel(QWidget* parent = nullptr);
  QString scenarioId() const { return profile_.id; }
  bool allRequirementsPassed() const;
  bool evidenceSatisfiedForTest(ScenarioEvidenceKey key) const {
    return satisfied(key);
  }

  // P1.G: 测试可读取的失败摘要，便于时序断言。
  QString lastFailureDetailForTest() const { return last_failure_detail_; }

  // P1.G: 可注入时钟——生产使用 steady_clock，测试
  // 注入 std::function<qint64()> 让两次相邻 observe() 落在不同的虚拟时刻，
  // 避免"同一毫秒内连续 observe WARNING+EMERGENCY"的假阳性。
  void setClockForTest(std::function<qint64()> clock) { clock_ = std::move(clock); }
  static qint64 default_clock() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }

 public slots:
  void setScenario(const QString& scenario_id);
  void setRunning(bool running);
  void onMapReady(bool ready);
  void onEgo(double speed_mps);
  void onLane(bool valid);
  void onNavigation(int state);
  void onObjects(const QVector<adas::gui::GuiMapObject>& objects);
  void onLead(bool valid);
  void onBehavior(int state);
  void onAeb(int state);
  void onSafety(int level);

 signals:
  void acceptanceChanged(bool passed, const QString& summary);

 private:
  void resetEvidence();
  void observe(ScenarioEvidenceKey key);
  void setLive(ScenarioEvidenceKey key, bool value);
  bool satisfied(ScenarioEvidenceKey key) const;
  void rebuildRequirements();
  void refresh();

  ScenarioWorkflowProfile profile_;
  bool running_{false};
  bool last_passed_{false};
  QString last_failure_detail_;
  QSet<int> observed_;
  QMap<int, bool> live_;
  // P1.G: 每个 evidence key 的最近一次观察时间（毫秒，单调时钟）；
  // 验收阶段要求"先 WARNING 后 EMERGENCY"等时序约束。
  QMap<int, qint64> observed_at_ms_;
  // P1.G: dense_overtake_v1 需要 >= kDenseMinFrames 个 DenseObjectSet 帧，
  // 至少 kDenseWindowMs 毫秒跨度。
  QVector<qint64> dense_object_set_at_ms_;
  static constexpr int kDenseMinFrames = 5;
  static constexpr qint64 kDenseWindowMs = 5000;
  // P1.G: 单调时钟来源；默认取系统 wall clock；测试可注入。
  std::function<qint64()> clock_;
  QMap<int, QLabel*> requirement_labels_;
  QLabel* family_label_{nullptr};
  QLabel* title_label_{nullptr};
  QLabel* hint_label_{nullptr};
  QLabel* state_label_{nullptr};
  QHBoxLayout* requirements_layout_{nullptr};
};

}  // namespace adas::gui

#endif  // ADAS_GUI__SCENARIO_RUN_PANEL_HPP_
