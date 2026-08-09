// 安全监控核心实现
#include "adas_safety_monitor/safety_monitor_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace adas::system {

SafetyMonitorCore::SafetyMonitorCore(const SafetyMonitorParams& params, double start_time_s)
    : params_(params), start_time_s_(start_time_s) {
  if (!std::isfinite(start_time_s_)) {
    throw std::invalid_argument("start_time_s must be finite");
  }
  if (!std::isfinite(params_.stale_warn_threshold_s) ||
      !std::isfinite(params_.stale_mrm_threshold_s) ||
      params_.stale_warn_threshold_s <= 0.0 || params_.stale_mrm_threshold_s <= 0.0) {
    throw std::invalid_argument("stale thresholds must be finite and > 0");
  }
  if (params_.confirm_frames_to_escalate <= 0 || params_.confirm_frames_to_recover <= 0) {
    throw std::invalid_argument("confirmation frame counts must be > 0");
  }
}

SafetyResult SafetyMonitorCore::evaluate(double now_s, const ChannelStamps& stamps) const {
  SafetyResult res;
  if (!std::isfinite(now_s)) {
    throw std::invalid_argument("now_s must be finite");
  }
  if (now_s - start_time_s_ < params_.startup_grace_s) {
    stale_mrm_frames_ = 0;
    fresh_recovery_frames_ = 0;
    stable_level_ = SafetyLevel::kOk;
    return res;  // 启动宽限：OK
  }

  const auto age_s = [now_s](double stamp) {
    return stamp < -1e17 ? std::numeric_limits<double>::infinity() : now_s - stamp;
  };
  const auto stale = [&age_s](double stamp, double timeout) {
    return age_s(stamp) > timeout;
  };

  // A single timeout is a degraded observation.  MRM requires a consecutive
  // confirmation window, which prevents one scheduler/DDS hiccup from
  // oscillating the vehicle into a recovery maneuver.
  const double warn_multiplier =
      std::min(params_.stale_warn_threshold_s, params_.stale_mrm_threshold_s);
  const double mrm_multiplier =
      std::max(params_.stale_warn_threshold_s, params_.stale_mrm_threshold_s);
  const auto beyond_warn = [&age_s, warn_multiplier](double stamp, double timeout) {
    return age_s(stamp) > warn_multiplier * timeout;
  };
  const auto beyond_mrm = [&age_s, mrm_multiplier](double stamp, double timeout) {
    return age_s(stamp) > mrm_multiplier * timeout;
  };

  const bool gate_stale = stale(stamps.gate_cmd, params_.gate_cmd_timeout_s);
  const bool odom_warn = beyond_warn(stamps.odom, params_.odom_timeout_s);
  const bool objects_warn = beyond_warn(stamps.objects, params_.objects_timeout_s);
  const bool trajectory_warn = beyond_warn(stamps.trajectory, params_.trajectory_timeout_s);
  const bool follower_warn = beyond_warn(stamps.follower_cmd, params_.follower_cmd_timeout_s);
  const bool odom_mrm = beyond_mrm(stamps.odom, params_.odom_timeout_s);
  const bool objects_mrm = beyond_mrm(stamps.objects, params_.objects_timeout_s);
  const bool trajectory_mrm = beyond_mrm(stamps.trajectory, params_.trajectory_timeout_s);
  const bool follower_mrm = beyond_mrm(stamps.follower_cmd, params_.follower_cmd_timeout_s);
  const bool any_warn = odom_warn || objects_warn || trajectory_warn || follower_warn;
  const bool any_mrm = odom_mrm || objects_mrm || trajectory_mrm || follower_mrm;

  if (gate_stale) {
    res.failed_components.push_back("gate_cmd");
  }

  // 下发链断流 → 最高级（车辆接口看门狗是执行者，这里负责取证与上报）
  if (gate_stale) {
    res.overall = SafetyLevel::kMrmEmergency;
  }

  // 功能链的诊断信息 continues to report the concrete channels currently
  // outside their normal timeout, even while the aggregate state is held by
  // hysteresis.
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

  if (any_mrm) {
    ++stale_mrm_frames_;
    fresh_recovery_frames_ = 0;
  } else {
    stale_mrm_frames_ = 0;
  }

  if (res.overall != SafetyLevel::kMrmEmergency) {
    if (stable_level_ == SafetyLevel::kMrmComfort) {
      if (!any_warn) {
        ++fresh_recovery_frames_;
        if (fresh_recovery_frames_ >= params_.confirm_frames_to_recover) {
          stable_level_ = SafetyLevel::kWarn;
          fresh_recovery_frames_ = 0;
        }
      } else {
        fresh_recovery_frames_ = 0;
      }
    } else if (any_mrm && stale_mrm_frames_ >= params_.confirm_frames_to_escalate) {
      stable_level_ = SafetyLevel::kMrmComfort;
      fresh_recovery_frames_ = 0;
    } else if (stable_level_ == SafetyLevel::kWarn) {
      if (!any_warn) {
        ++fresh_recovery_frames_;
        if (fresh_recovery_frames_ >= params_.confirm_frames_to_recover) {
          stable_level_ = SafetyLevel::kOk;
          fresh_recovery_frames_ = 0;
        }
      } else {
        fresh_recovery_frames_ = 0;
      }
    } else if (stale_mrm_frames_ >= params_.confirm_frames_to_escalate) {
      stable_level_ = SafetyLevel::kMrmComfort;
      fresh_recovery_frames_ = 0;
    } else if (any_warn) {
      stable_level_ = SafetyLevel::kWarn;
      fresh_recovery_frames_ = 0;
    } else {
      stable_level_ = SafetyLevel::kOk;
      fresh_recovery_frames_ = 0;
    }
    res.overall = stable_level_;
  }
  return res;
}

}  // namespace adas::system
