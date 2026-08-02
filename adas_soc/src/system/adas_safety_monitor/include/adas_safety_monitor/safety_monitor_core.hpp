// adas_safety_monitor/safety_monitor_core.hpp
// 整机安全监控核心（对标 Apollo monitor + Autoware MRM handler 的角色，无 ROS 依赖）。
// 监控关键链路 topic 存活（时间戳新鲜度），聚合出整机安全级别：
//   OK → WARN → MRM_COMFORT（功能链退化 → gate 切 builtin_stop 舒适停车）
//             → MRM_EMERGENCY（gate 输出断流 → 车辆接口看门狗兜底，此处仅取证）
// 启动宽限期内不判罚（节点顺序拉起时输入天然缺失）。
#ifndef ADAS_SAFETY_MONITOR__SAFETY_MONITOR_CORE_HPP_
#define ADAS_SAFETY_MONITOR__SAFETY_MONITOR_CORE_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace adas::system {

struct SafetyMonitorParams {
  double startup_grace_s{5.0};
  double odom_timeout_s{0.3};
  double objects_timeout_s{0.6};
  double trajectory_timeout_s{0.6};
  double follower_cmd_timeout_s{0.4};
  double gate_cmd_timeout_s{0.4};
};

// 各通道最近一次收到的时刻（秒；从未收到 = -1e18）
struct ChannelStamps {
  double odom{-1e18};
  double objects{-1e18};
  double trajectory{-1e18};
  double follower_cmd{-1e18};
  double gate_cmd{-1e18};
};

enum class SafetyLevel : uint8_t {
  kOk = 0,
  kWarn = 1,
  kMrmComfort = 2,
  kMrmEmergency = 3,
};

struct SafetyResult {
  SafetyLevel overall{SafetyLevel::kOk};
  std::vector<std::string> failed_components;
};

class SafetyMonitorCore {
 public:
  SafetyMonitorCore(const SafetyMonitorParams& params, double start_time_s);
  SafetyResult evaluate(double now_s, const ChannelStamps& stamps) const;

 private:
  SafetyMonitorParams params_;
  double start_time_s_;
};

}  // namespace adas::system

#endif  // ADAS_SAFETY_MONITOR__SAFETY_MONITOR_CORE_HPP_
