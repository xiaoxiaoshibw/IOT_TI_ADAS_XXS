#ifndef ADAS_GUI__FORMAT_HPP_
#define ADAS_GUI__FORMAT_HPP_

// GUI 显示层的纯格式化逻辑（无 Qt/ROS 依赖，可主机 gtest）。
// 状态、来源与故障码取值与 MCU/include/adas_can_protocol.h、safety.h 一致。

#include <cstdint>
#include <string>

namespace adas::gui {

inline const char* state_name(std::uint8_t state) {
  switch (state) {
    case 0U: return "INIT";
    case 1U: return "STANDBY";
    case 2U: return "ACTIVE";
    case 3U: return "DEGRADED";
    case 4U: return "MRM";
    case 5U: return "EMERGENCY_BRAKE";
    case 6U: return "FAILSAFE";
    case 7U: return "FAULT_LOCK";
    default: return "UNKNOWN";
  }
}

// 状态颜色：红=不可逆/紧急，黄=降级/受控停车，绿=正常控车，灰=未控车。
inline const char* state_color(std::uint8_t state) {
  switch (state) {
    case 2U: return "#2e7d32";                    // ACTIVE
    case 3U: case 4U: return "#f9a825";           // DEGRADED / MRM
    case 5U: case 6U: case 7U: return "#c62828";  // EMERGENCY/FAILSAFE/LOCK
    default: return "#616161";                    // INIT / STANDBY / unknown
  }
}

inline const char* source_name(std::uint8_t source) {
  switch (source) {
    case 0U: return "NONE";
    case 1U: return "PRIMARY";
    case 2U: return "BACKUP";
    case 9U: return "MCU_WATCHDOG";
    default: return "UNKNOWN";
  }
}

inline std::string format_age(float age_s) {
  if (age_s < 0.0f) return "never";
  if (age_s < 1.0f) {
    return std::to_string(static_cast<int>(age_s * 1000.0f + 0.5f)) + " ms";
  }
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.1f s", static_cast<double>(age_s));
  return buffer;
}

// NavigationStatus.state → 显示名（与 adas_msgs/NavigationStatus 常量一致）。
inline const char* nav_state_name(std::uint8_t state) {
  switch (state) {
    case 0U: return "IDLE";
    case 1U: return "WAITING_FOR_MAP";
    case 2U: return "PLANNING";
    case 3U: return "DRIVING";
    case 4U: return "ARRIVED";
    case 5U: return "FAILED";
    case 6U: return "CANCELED";
    default: return "UNKNOWN";
  }
}

// BehaviorState.state → 显示名（与 adas_msgs/BehaviorState 常量一致）。
inline const char* behavior_state_name(std::uint8_t state) {
  switch (state) {
    case 0U: return "车道保持";
    case 1U: return "跟车";
    case 2U: return "等待超车";
    case 3U: return "超车中";
    case 4U: return "超车返回";
    case 5U: return "停车中";
    case 6U: return "紧急";
    default: return "未知";
  }
}

// GateStatus.selected_source → 显示名。
inline const char* gate_source_name(std::uint8_t source) {
  switch (source) {
    case 0U: return "跟随器";
    case 1U: return "AEB";
    case 2U: return "内建停车";
    default: return "未知";
  }
}

// AebStatus.state → 显示名。
inline const char* aeb_state_name(std::uint8_t state) {
  switch (state) {
    case 0U: return "未激活";
    case 1U: return "监控";
    case 2U: return "预警";
    case 3U: return "紧急制动";
    default: return "未知";
  }
}

// SafetyStatus.overall → 显示名。
inline const char* safety_level_name(std::uint8_t level) {
  switch (level) {
    case 0U: return "OK";
    case 1U: return "WARN";
    case 2U: return "MRM(舒适)";
    case 3U: return "MRM(紧急)";
    default: return "未知";
  }
}

// FC_* 位图 → 可读原因串（与网关 degrade_reason_text 同一命名）。
inline std::string fault_bits_text(std::uint16_t fault_code) {
  static constexpr const char* kNames[] = {
      "primary_timeout", "backup_timeout", "primary_seq_stall", "backup_seq_stall",
      "all_sources_lost", "crc_errors", "soc_fault", "loop_overrun",
      "estop_active", "protocol_mismatch", "fault_lock", "can_bus_off",
      "watchdog_reset", "self_test_failed",
  };
  std::string result;
  for (unsigned bit = 0U; bit < sizeof(kNames) / sizeof(kNames[0]); ++bit) {
    if ((fault_code & (1U << bit)) == 0U) continue;
    if (!result.empty()) result += '+';
    result += kNames[bit];
  }
  if (result.empty()) result = "none";
  return result;
}

}  // namespace adas::gui

#endif  // ADAS_GUI__FORMAT_HPP_
