// 命令门控核心实现
#include "adas_command_gate/gate_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace adas::control {

namespace ac = adas::common;

GateCore::GateCore(const GateParams& params)
    : params_(params),
      steer_lim_(params.speed_points_mps, params.steer_lim_rad),
      steer_rate_lim_(params.speed_points_mps, params.steer_rate_lim_rps) {
  if (!std::isfinite(params_.aeb_stale_timeout_s) || params_.aeb_stale_timeout_s <= 0.0 ||
      !std::isfinite(params_.odom_stale_timeout_s) || params_.odom_stale_timeout_s <= 0.0) {
    throw std::invalid_argument("AEB and odometry stale timeouts must be > 0");
  }
}

bool GateCore::process_aeb_override(const GateInputs& in, GateDecision* decision) {
  if (in.aeb_received && std::isfinite(in.aeb_stamp_s)) {
    // Do not let an older replayed frame move the tracked receive time
    // backwards.  This keeps the freshness check deterministic in SIL.
    last_aeb_stamp_ = std::max(last_aeb_stamp_, in.aeb_stamp_s);
  }
  const double aeb_age_s = in.now_s - last_aeb_stamp_;
  const bool aeb_fresh = in.aeb_received && std::isfinite(aeb_age_s) && aeb_age_s >= 0.0 &&
                         aeb_age_s <= params_.aeb_stale_timeout_s;
  if (!in.aeb_emergency || !aeb_fresh ||
      in.aeb_cmd.longitudinal.acceleration_mps2 >=
          decision->cmd.longitudinal.acceleration_mps2) {
    return false;
  }
  decision->cmd.longitudinal = in.aeb_cmd.longitudinal;
  decision->source = GateSource::kAeb;
  decision->reason = "aeb_emergency_override";
  return true;
}

GateDecision GateCore::update(const GateInputs& in) {
  GateDecision d;

  // ── 1. 源选择 ──
  const bool follower_fresh =
      in.follower_received && (in.now_s - in.follower_stamp_s) < params_.follower_timeout_s;
  if (in.force_builtin_stop) {
    d.source = GateSource::kBuiltinStop;
    d.reason = "forced_by_service";
  } else if (in.mrm_stop_requested) {
    d.source = GateSource::kBuiltinStop;
    d.reason = "mrm_requested";
  } else if (!follower_fresh) {
    d.source = GateSource::kBuiltinStop;
    if (in.navigation_planned_stop) {
      d.reason = in.follower_received ? "planned_stop_route_terminal" : "planned_stop_no_route";
    } else {
      d.reason = in.follower_received ? "follower_timeout" : "follower_never_received";
    }
  } else {
    d.source = GateSource::kFollower;
    d.cmd = in.follower_cmd;
  }

  // builtin_stop 命令生成：转角从上一输出向 0 衰减 + 恒定舒适减速
  if (d.source == GateSource::kBuiltinStop) {
    const double alpha =
        params_.steer_decay_tau_s <= 0.0 ? 1.0 : in.dt / (params_.steer_decay_tau_s + in.dt);
    d.cmd.lateral.steering_tire_angle_rad = last_steer_ * (1.0 - alpha);
    d.cmd.lateral.rotation_rate_rad_s = 0.0;
    d.cmd.longitudinal.velocity_mps = 0.0;
    d.cmd.longitudinal.acceleration_mps2 = -params_.stop_decel_mps2;
  }

  // ── 2. AEB 紧急纵向覆盖：制动只增不减，横向保持当前源 ──
  const bool emergency_braking = process_aeb_override(in, &d);

  // ── 3. 滤波限幅 ──
  bool limited = false;
  // 3a. 速度相关转角限幅
  const bool odom_fresh = in.odom_received && std::isfinite(in.odom_stamp_s) &&
                          in.now_s >= in.odom_stamp_s &&
                          (in.now_s - in.odom_stamp_s) <= params_.odom_stale_timeout_s;
  const double conservative_steer_lim =
      *std::min_element(params_.steer_lim_rad.begin(), params_.steer_lim_rad.end());
  const double conservative_steer_rate_lim =
      *std::min_element(params_.steer_rate_lim_rps.begin(), params_.steer_rate_lim_rps.end());
  const double steer_lim = odom_fresh ? steer_lim_(in.ego_speed_mps) : conservative_steer_lim;
  double steer = std::clamp(d.cmd.lateral.steering_tire_angle_rad, -steer_lim, steer_lim);
  if (steer != d.cmd.lateral.steering_tire_angle_rad) {
    limited = true;
  }
  // 3b. 转角速率限幅（相对上一输出帧——跨源连续，切源不突跳）
  const double steer_rate_lim = odom_fresh ? steer_rate_lim_(in.ego_speed_mps)
                                           : conservative_steer_rate_lim;
  const double max_dsteer = steer_rate_lim * in.dt;
  const double dsteer = std::clamp(steer - last_steer_, -max_dsteer, max_dsteer);
  if (std::fabs(dsteer - (steer - last_steer_)) > 1e-12) {
    limited = true;
  }
  steer = last_steer_ + dsteer;

  // 3c. 加速度限幅
  double accel = std::clamp(d.cmd.longitudinal.acceleration_mps2, -params_.max_decel_mps2,
                            params_.max_accel_mps2);
  if (accel != d.cmd.longitudinal.acceleration_mps2) {
    limited = true;
  }
  // 3d. jerk 限幅：常规源双向；紧急制动（aeb/builtin）放开"制动加深"方向
  const double max_da = params_.max_jerk_mps3 * in.dt;
  double da = accel - last_accel_;
  const bool brake_deepening = da < 0.0 && emergency_braking;
  if (!brake_deepening) {
    const double da_clamped = std::clamp(da, -max_da, max_da);
    if (std::fabs(da_clamped - da) > 1e-12) {
      limited = true;
    }
    da = da_clamped;
  }
  accel = last_accel_ + da;

  d.cmd.lateral.steering_tire_angle_rad = steer;
  d.cmd.longitudinal.acceleration_mps2 = accel;
  d.limited = limited;

  last_steer_ = steer;
  last_accel_ = accel;
  return d;
}

}  // namespace adas::control
