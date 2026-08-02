#ifndef ADAS_GUI__HEALTH_MODEL_HPP_
#define ADAS_GUI__HEALTH_MODEL_HPP_

#include <cstdint>

namespace adas::gui {

// GUI 与 ROS2 状态桥统一使用的健康状态。Unknown 表示尚无判定证据，
// Offline 表示已有 ROS graph/新鲜度证据确认组件不可用。
enum class HealthState : std::uint8_t {
  Unknown,
  Starting,
  Healthy,
  Degraded,
  Fault,
  Offline
};

inline const char* health_state_name(HealthState state) {
  switch (state) {
    case HealthState::Unknown: return "未知";
    case HealthState::Starting: return "启动中";
    case HealthState::Healthy: return "健康";
    case HealthState::Degraded: return "降级";
    case HealthState::Fault: return "故障";
    case HealthState::Offline: return "离线";
  }
  return "未知";
}

inline const char* health_state_color(HealthState state) {
  switch (state) {
    case HealthState::Healthy: return "#2e7d32";
    case HealthState::Starting:
    case HealthState::Degraded: return "#f9a825";
    case HealthState::Fault: return "#c62828";
    case HealthState::Unknown:
    case HealthState::Offline: return "#616161";
  }
  return "#616161";
}

inline bool health_is_problem(HealthState state) {
  return state == HealthState::Degraded || state == HealthState::Fault ||
         state == HealthState::Offline;
}

inline int health_severity(HealthState state) {
  switch (state) {
    case HealthState::Fault: return 3;
    case HealthState::Offline: return 2;
    case HealthState::Degraded: return 1;
    default: return 0;
  }
}

}  // namespace adas::gui

#endif  // ADAS_GUI__HEALTH_MODEL_HPP_
