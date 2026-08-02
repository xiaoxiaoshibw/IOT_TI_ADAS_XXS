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

// 演示预设清单。默认 Town04（scenarios.py 的高速环路出生点），控制源 ros2。
inline std::vector<DemoPreset> demo_presets() {
  return {
      {QStringLiteral("LKA 直行"), QStringLiteral("lka"),
       QStringLiteral("Town04"), QStringLiteral("ros2"),
       QStringLiteral("无前车巡航；观察横向误差有界、弯道前馈")},
      {QStringLiteral("ACC 跟车"), QStringLiteral("acc"),
       QStringLiteral("Town04"), QStringLiteral("ros2"),
       QStringLiteral("前车 8→5→10→7 m/s 变速；观察时距自适应跟随")},
      {QStringLiteral("ACC 停车再走"), QStringLiteral("acc_stop_and_go"),
       QStringLiteral("Town04"), QStringLiteral("ros2"),
       QStringLiteral("前车停→走；观察自车起停跟随时距与起步响应")},
      {QStringLiteral("ACC 慢速卡车"), QStringLiteral("acc_slow_truck"),
       QStringLiteral("Town04"), QStringLiteral("ros2"),
       QStringLiteral("慢速前车 3 m/s；观察长跟时距、无误加速")},
      {QStringLiteral("AEB 前车急停"), QStringLiteral("aeb"),
       QStringLiteral("Town04"), QStringLiteral("ros2"),
       QStringLiteral("t=15s 前车全力急刹；观察 AEB WARNING→BRAKING")},
      {QStringLiteral("AEB 静止障碍"), QStringLiteral("aeb_stationary"),
       QStringLiteral("Town04"), QStringLiteral("ros2"),
       QStringLiteral("前车起步即静止；观察对静态障碍的 AEB 触发与 TTC")},
      {QStringLiteral("AEB 横穿行人"), QStringLiteral("aeb_pedestrian"),
       QStringLiteral("Town04"), QStringLiteral("ros2"),
       QStringLiteral("逼近 40m 时行人右侧横穿；观察行人识别 + 紧急制动")},
  };
}

}  // namespace adas::gui

#endif  // ADAS_GUI__DEMO_PRESETS_HPP_
