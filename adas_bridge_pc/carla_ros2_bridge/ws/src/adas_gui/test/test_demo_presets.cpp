// demo_presets.hpp 单测：预设取值契约（与 known_scenarios/towns/control_sources 对齐）。
#include <gtest/gtest.h>

#include <set>

#include <QFile>
#include <QJsonDocument>

#include "demo_presets.hpp"
#include "launch_config.hpp"
#include "scenario_workflow.hpp"

namespace {

using adas::gui::demo_presets;

TEST(DemoPresets, ScenarioAndTownWithinBridgeContract) {
  const auto scenarios = adas::gui::known_scenarios();
  const auto towns = adas::gui::known_towns();
  const auto sources = adas::gui::known_control_sources();
  const auto presets = demo_presets();
  ASSERT_FALSE(presets.empty());
  for (const auto& p : presets) {
    EXPECT_TRUE(scenarios.contains(p.scenario))
        << "preset '" << p.label.toStdString() << "' has unknown scenario '"
        << p.scenario.toStdString() << "'";
    EXPECT_TRUE(towns.contains(p.town))
        << "preset '" << p.label.toStdString() << "' has unknown town '"
        << p.town.toStdString() << "'";
    EXPECT_TRUE(sources.contains(p.control_source))
        << "preset '" << p.label.toStdString() << "' has unknown control_source '"
        << p.control_source.toStdString() << "'";
  }
}

TEST(DemoPresets, LabelsAreUnique) {
  // 防止两个按钮 label 重复导致按钮互相覆盖
  const auto presets = demo_presets();
  std::set<QString> seen;
  for (const auto& p : presets) {
    EXPECT_TRUE(seen.insert(p.label).second)
        << "duplicate preset label: " << p.label.toStdString();
  }
}

TEST(DemoPresets, CatalogPresetAndWorkflowCoverSameTenScenarios) {
  const auto catalog = adas::gui::load_scenario_catalog();
  const auto presets = demo_presets();
  const auto workflow_ids = adas::gui::scenario_workflow_ids();
  ASSERT_EQ(catalog.size(), 10);
  ASSERT_EQ(presets.size(), 10U);
  ASSERT_EQ(workflow_ids.size(), 10);

  QStringList catalog_ids;
  QStringList preset_ids;
  for (const auto& entry : catalog) catalog_ids << entry.id;
  for (const auto& preset : presets) preset_ids << preset.scenario;

  EXPECT_EQ(preset_ids, catalog_ids);
  EXPECT_EQ(workflow_ids, catalog_ids);
  std::set<QString> unique_ids(workflow_ids.cbegin(), workflow_ids.cend());
  EXPECT_EQ(unique_ids.size(), 10U);
}

TEST(DemoPresets, EveryAutomaticWorkflowRequiresNavigationDrivingEvidence) {
  using adas::gui::ScenarioEvidenceKey;
  for (const auto& profile : adas::gui::scenario_workflow_profiles()) {
    bool requires_driving_evidence = false;
    for (const auto& requirement : profile.requirements) {
      if (requirement.key == ScenarioEvidenceKey::NavigationDriving) {
        requires_driving_evidence = true;
        break;
      }
    }
    if (profile.requires_navigation) {
      EXPECT_TRUE(requires_driving_evidence)
          << profile.id.toStdString()
          << " enables automatic navigation without DRIVING evidence";
    } else {
      EXPECT_FALSE(requires_driving_evidence)
          << profile.id.toStdString()
          << " must remain a manual-navigation workflow";
    }
  }
}

TEST(DemoPresets, SlowTruckUsesDeterministicCarlaTruckBlueprint) {
  const auto entry = adas::gui::scenario_catalog_entry("acc_slow_truck");
  QFile file(entry.file);
  ASSERT_TRUE(file.open(QIODevice::ReadOnly));
  const QJsonDocument document = QJsonDocument::fromJson(file.readAll());
  ASSERT_TRUE(document.isObject());
  const QJsonArray actors =
      document.object().value(QStringLiteral("actors")).toArray();
  ASSERT_EQ(actors.size(), 1);
  EXPECT_EQ(actors.at(0).toObject().value(QStringLiteral("blueprint")).toString(),
            QStringLiteral("vehicle.carlamotors.carlacola"));
}

}  // namespace
