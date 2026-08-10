#include "scenario_run_panel.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

#include "theme.hpp"

namespace adas::gui {

namespace {
int key_value(ScenarioEvidenceKey key) { return static_cast<int>(key); }
}

ScenarioRunPanel::ScenarioRunPanel(QWidget* parent) : QWidget(parent) {
  setObjectName(QStringLiteral("scenarioRunPanel"));
  setStyleSheet(QStringLiteral(
      "QWidget#scenarioRunPanel{background:%1;border:1px solid %2;border-radius:7px;}")
                    .arg(theme::kMdSurfaceContainerLow, theme::kCardBorder));
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(10, 7, 10, 7);
  root->setSpacing(4);
  auto* top = new QHBoxLayout();
  top->setSpacing(7);
  family_label_ = new QLabel();
  family_label_->setStyleSheet(
      QStringLiteral("color:%1;background:%2;border-radius:4px;padding:2px 6px;"
                     "font-size:9px;font-weight:800;")
          .arg(theme::kMdOnPrimaryContainer, theme::kMdPrimaryContainer));
  title_label_ = new QLabel();
  title_label_->setStyleSheet(QStringLiteral("color:%1;font-size:12px;font-weight:700;")
                                  .arg(theme::kTextPrimary));
  hint_label_ = new QLabel();
  hint_label_->setStyleSheet(QStringLiteral("color:%1;font-size:10px;")
                                 .arg(theme::kTextSecondary));
  state_label_ = new QLabel(QStringLiteral("待启动"));
  state_label_->setStyleSheet(QStringLiteral("color:%1;font-size:10px;font-weight:700;")
                                  .arg(theme::kTextSecondary));
  top->addWidget(family_label_);
  top->addWidget(title_label_);
  top->addWidget(hint_label_, 1);
  top->addWidget(state_label_);
  root->addLayout(top);
  requirements_layout_ = new QHBoxLayout();
  requirements_layout_->setSpacing(5);
  root->addLayout(requirements_layout_);
  setScenario(QStringLiteral("free"));
}

void ScenarioRunPanel::setScenario(const QString& scenario_id) {
  profile_ = scenario_workflow_profile(scenario_id);
  family_label_->setText(profile_.family);
  title_label_->setText(profile_.objective);
  hint_label_->setText(profile_.operator_hint);
  resetEvidence();
  rebuildRequirements();
  refresh();
}

void ScenarioRunPanel::setRunning(bool running) {
  if (running && !running_) resetEvidence();
  running_ = running;
  refresh();
}

void ScenarioRunPanel::resetEvidence() {
  observed_.clear();
  live_.clear();
  last_passed_ = false;
}

void ScenarioRunPanel::observe(ScenarioEvidenceKey key) {
  observed_.insert(key_value(key));
  refresh();
}

void ScenarioRunPanel::setLive(ScenarioEvidenceKey key, bool value) {
  live_.insert(key_value(key), value);
  refresh();
}

bool ScenarioRunPanel::satisfied(ScenarioEvidenceKey key) const {
  const int value = key_value(key);
  const auto live = live_.constFind(value);
  if (live != live_.cend()) return live.value() || observed_.contains(value);
  return observed_.contains(value);
}

void ScenarioRunPanel::onMapReady(bool ready) {
  setLive(ScenarioEvidenceKey::MapReady, ready);
}

void ScenarioRunPanel::onEgo(double speed_mps) {
  if (std::isfinite(speed_mps) && speed_mps > 0.5) {
    observe(ScenarioEvidenceKey::EgoMoving);
  }
}

void ScenarioRunPanel::onLane(bool valid) {
  if (valid) observe(ScenarioEvidenceKey::LaneValid);
}

void ScenarioRunPanel::onNavigation(int state) {
  if (state == 3) observe(ScenarioEvidenceKey::NavigationDriving);
}

void ScenarioRunPanel::onObjects(const QVector<GuiMapObject>& objects) {
  if (objects.size() >= 20) observe(ScenarioEvidenceKey::DenseObjectSet);
  if (std::any_of(objects.cbegin(), objects.cend(), [](const GuiMapObject& object) {
        return object.classification == 3;
      })) {
    observe(ScenarioEvidenceKey::PedestrianDetected);
  }
}

void ScenarioRunPanel::onLead(bool valid) {
  if (valid) observe(ScenarioEvidenceKey::LeadDetected);
}

void ScenarioRunPanel::onBehavior(int state) {
  if (state == 1) observe(ScenarioEvidenceKey::FollowLead);
  if (state == 2 || state == 3 || state == 4) {
    observe(ScenarioEvidenceKey::OvertakeDecision);
  }
}

void ScenarioRunPanel::onAeb(int state) {
  if (state == 2) observe(ScenarioEvidenceKey::AebWarning);
  if (state >= 3) observe(ScenarioEvidenceKey::AebEmergency);
}

void ScenarioRunPanel::onSafety(int level) {
  setLive(ScenarioEvidenceKey::SafetyHealthy, level < 2);
}

bool ScenarioRunPanel::allRequirementsPassed() const {
  return std::all_of(profile_.requirements.cbegin(), profile_.requirements.cend(),
                     [this](const ScenarioRequirement& requirement) {
                       return satisfied(requirement.key);
                     });
}

void ScenarioRunPanel::rebuildRequirements() {
  while (auto* item = requirements_layout_->takeAt(0)) {
    delete item->widget();
    delete item;
  }
  requirement_labels_.clear();
  for (const auto& requirement : profile_.requirements) {
    auto* label = new QLabel(requirement.label);
    label->setToolTip(requirement.label);
    requirement_labels_.insert(key_value(requirement.key), label);
    requirements_layout_->addWidget(label, 1);
  }
}

void ScenarioRunPanel::refresh() {
  int passed_count = 0;
  for (const auto& requirement : profile_.requirements) {
    const bool passed = satisfied(requirement.key);
    if (passed) ++passed_count;
    auto* label = requirement_labels_.value(key_value(requirement.key), nullptr);
    if (!label) continue;
    label->setText(QStringLiteral("%1 %2")
                       .arg(passed ? QStringLiteral("✓") : QStringLiteral("○"),
                            requirement.label));
    label->setStyleSheet(
        QStringLiteral("color:%1;background:%2;border:1px solid %3;border-radius:4px;"
                       "padding:2px 5px;font-size:9px;")
            .arg(passed ? theme::kOk : theme::kTextSecondary,
                 theme::kMdSurfaceContainerHigh,
                 passed ? theme::kOk : theme::kCardBorder));
  }
  const bool passed = !profile_.requirements.isEmpty() &&
                      passed_count == profile_.requirements.size();
  if (!running_) {
    state_label_->setText(QStringLiteral("待启动"));
    state_label_->setStyleSheet(QStringLiteral("color:%1;font-size:10px;font-weight:700;")
                                    .arg(theme::kTextSecondary));
  } else if (passed) {
    state_label_->setText(QStringLiteral("验收通过 %1/%2")
                              .arg(passed_count).arg(profile_.requirements.size()));
    state_label_->setStyleSheet(QStringLiteral("color:%1;font-size:10px;font-weight:800;")
                                    .arg(theme::kOk));
  } else {
    state_label_->setText(QStringLiteral("运行验证 %1/%2")
                              .arg(passed_count).arg(profile_.requirements.size()));
    state_label_->setStyleSheet(QStringLiteral("color:%1;font-size:10px;font-weight:700;")
                                    .arg(theme::kWarn));
  }
  if (passed != last_passed_) {
    last_passed_ = passed;
    emit acceptanceChanged(
        passed, QStringLiteral("%1：%2/%3 项")
                    .arg(profile_.objective).arg(passed_count)
                    .arg(profile_.requirements.size()));
  }
}

}  // namespace adas::gui
