#ifndef ADAS_GUI__SCENARIO_RUN_PANEL_HPP_
#define ADAS_GUI__SCENARIO_RUN_PANEL_HPP_

#include <QMap>
#include <QSet>
#include <QWidget>

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
  QSet<int> observed_;
  QMap<int, bool> live_;
  QMap<int, QLabel*> requirement_labels_;
  QLabel* family_label_{nullptr};
  QLabel* title_label_{nullptr};
  QLabel* hint_label_{nullptr};
  QLabel* state_label_{nullptr};
  QHBoxLayout* requirements_layout_{nullptr};
};

}  // namespace adas::gui

#endif  // ADAS_GUI__SCENARIO_RUN_PANEL_HPP_
