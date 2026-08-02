// demo_presets.hpp 单测：预设取值契约（与 known_scenarios/towns/control_sources 对齐）。
#include <gtest/gtest.h>

#include <set>

#include "demo_presets.hpp"
#include "launch_config.hpp"

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

}  // namespace
