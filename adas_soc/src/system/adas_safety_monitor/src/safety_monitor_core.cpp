// 安全监控核心实现
#include "adas_safety_monitor/safety_monitor_core.hpp"

namespace adas::system {

SafetyMonitorCore::SafetyMonitorCore(const SafetyMonitorParams& params, double start_time_s)
    : params_(params), start_time_s_(start_time_s) {}

SafetyResult SafetyMonitorCore::evaluate(double now_s, const ChannelStamps& stamps) const {
  SafetyResult res;
  if (now_s - start_time_s_ < params_.startup_grace_s) {
    return res;  // 启动宽限：OK
  }

  const auto stale = [now_s](double stamp, double timeout) {
    return now_s - stamp > timeout;
  };

  // 下发链断流 → 最高级（车辆接口看门狗是执行者，这里负责取证与上报）
  if (stale(stamps.gate_cmd, params_.gate_cmd_timeout_s)) {
    res.failed_components.push_back("gate_cmd");
    res.overall = SafetyLevel::kMrmEmergency;
  }

  // 功能链任一环退化 → 请求舒适停车
  if (stale(stamps.odom, params_.odom_timeout_s)) {
    res.failed_components.push_back("odometry");
  }
  if (stale(stamps.objects, params_.objects_timeout_s)) {
    res.failed_components.push_back("objects");
  }
  if (stale(stamps.trajectory, params_.trajectory_timeout_s)) {
    res.failed_components.push_back("trajectory");
  }
  if (stale(stamps.follower_cmd, params_.follower_cmd_timeout_s)) {
    res.failed_components.push_back("follower_cmd");
  }
  if (res.overall != SafetyLevel::kMrmEmergency && !res.failed_components.empty()) {
    res.overall = SafetyLevel::kMrmComfort;
  }
  return res;
}

}  // namespace adas::system
