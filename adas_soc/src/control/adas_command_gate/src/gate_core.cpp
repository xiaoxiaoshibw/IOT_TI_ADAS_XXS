// 命令门控核心实现
#include "adas_command_gate/gate_core.hpp"

#include <algorithm>
#include <cmath>

namespace adas::control {

namespace ac = adas::common;

GateCore::GateCore(const GateParams& params)
    : params_(params),
      steer_lim_(params.speed_points_mps, params.steer_lim_rad),
      steer_rate_lim_(params.speed_points_mps, params.steer_rate_lim_rps) {}

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
  bool emergency_braking = false;
  if (in.aeb_emergency &&
      in.aeb_cmd.longitudinal.acceleration_mps2 < d.cmd.longitudinal.acceleration_mps2) {
    d.cmd.longitudinal = in.aeb_cmd.longitudinal;
    d.source = GateSource::kAeb;
    d.reason = "aeb_emergency_override";
    emergency_braking = true;
  }

  // ── 3. 滤波限幅 ──
  bool limited = false;
  // 3a. 速度相关转角限幅
  const double steer_lim = steer_lim_(in.ego_speed_mps);
  double steer = std::clamp(d.cmd.lateral.steering_tire_angle_rad, -steer_lim, steer_lim);
  if (steer != d.cmd.lateral.steering_tire_angle_rad) {
    limited = true;
  }
  // 3b. 转角速率限幅（相对上一输出帧——跨源连续，切源不突跳）
  const double max_dsteer = steer_rate_lim_(in.ego_speed_mps) * in.dt;
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
