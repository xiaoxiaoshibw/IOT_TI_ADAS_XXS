#ifndef ADAS_GUI__DEMO_PRESETS_HPP_
#define ADAS_GUI__DEMO_PRESETS_HPP_

// 一键演示场景预设。纯逻辑（仅 QString 列表），gtest 覆盖。
//
// 预设只负责"启动哪个 CARLA 场景 + 在哪个 town + 用哪个控制源"。
// 导航目标必须由用户在地图上点击（MapView::goalRequested）独立设置，
// 避免"场景"与"导航"两个概念耦合（之前 forward_m/lateral_m 会在用户
// arm 期间静默覆盖手动选点）。
//
// scenario / town 取值须落在 launch_config.hpp 的 known_scenarios()/known_towns()
// （即 scenarios.py 的 ORDER），改动两侧同步。

#include <vector>

#include <QString>

#include "launch_config.hpp"

namespace adas::gui {

struct DemoPreset {
  QString label;            // 按钮显示名
  QString scenario;         // 来自 known_scenarios() / scenarios.py ORDER
  QString town;             // 来自 known_towns()
  QString control_source;   // ros2 / can / can_cpp
  QString note;             // 悬停提示 / 讲解要点
};

// 演示预设直接从 catalog 生成，保证新增场景无需再手工维护第二份按钮清单。
// catalog 不可用时才回退旧 ID 列表。
inline std::vector<DemoPreset> demo_presets() {
  std::vector<DemoPreset> result;
  const auto catalog = load_scenario_catalog();
  if (!catalog.isEmpty()) {
    result.reserve(static_cast<std::size_t>(catalog.size()));
    for (const auto& entry : catalog) {
      result.push_back({entry.display_name, entry.id, entry.default_town,
                        QStringLiteral("ros2"), entry.description});
    }
    return result;
  }
  for (const auto& id : legacy_scenarios()) {
    result.push_back({id, id, QStringLiteral("Town04"),
                      QStringLiteral("ros2"), QStringLiteral("兼容场景")});
  }
  return result;
}

}  // namespace adas::gui

#endif  // ADAS_GUI__DEMO_PRESETS_HPP_
