// 冗余仲裁核心实现
#include "adas_redundancy/arbiter_core.hpp"

#include <algorithm>
#include <cmath>

namespace adas::system {

ArbiterCore::ArbiterCore(const ArbiterParams& params) : params_(params) {}

ArbiterDecision ArbiterCore::update(const ArbiterInputs& in) {
  ArbiterDecision d;

  const auto fresh = [&](const StackObservation& s) {
    return s.received && (in.now_s - s.stamp_s) < params_.cmd_timeout_s;
  };
  const bool p_fresh = fresh(in.primary);
  const bool b_fresh = fresh(in.backup);
  const bool p_nominal = p_fresh && in.primary.nominal;
  const bool b_nominal = b_fresh && in.backup.nominal;

  // 主栈"连续正常"计时（回切滞回：主恢复后须稳定 recover_stable_s）
  if (p_nominal) {
    if (primary_nominal_since_s_ < -1e17) {
      primary_nominal_since_s_ = in.now_s;
    }
  } else {
    primary_nominal_since_s_ = -1e18;
  }
  const bool p_stable =
      p_nominal && (in.now_s - primary_nominal_since_s_) >= params_.recover_stable_s;

  // ── 选源 ──
  ActiveRole target = ActiveRole::kNone;
  if (role_ == ActiveRole::kPrimary || role_ == ActiveRole::kNone) {
    // 当前主选（或初始）：主正常留主；主降级/失联 → 备正常切备；否则任一新鲜者
    if (p_nominal) {
      target = ActiveRole::kPrimary;
    } else if (b_nominal) {
      target = ActiveRole::kBackup;
      d.reason = p_fresh ? "primary_degraded" : "primary_lost";
    } else if (p_fresh) {
      target = ActiveRole::kPrimary;  // 双降级：跟随主栈（其 builtin_stop 停车）
    } else if (b_fresh) {
      target = ActiveRole::kBackup;
      d.reason = "primary_lost_backup_degraded";
    }
  } else {  // 当前备选：主须"稳定正常"才回切（防抖动来回切）
    if (p_stable) {
      target = ActiveRole::kPrimary;
      d.reason = "primary_recovered";
    } else if (b_fresh) {
      target = ActiveRole::kBackup;
    } else if (p_fresh) {
      target = ActiveRole::kPrimary;
      d.reason = "backup_lost";
    }
  }

  if (target == ActiveRole::kNone) {
    // 双源失联：停发（车辆接口看门狗 200ms 后全力制动兜底）
    role_ = ActiveRole::kNone;
    d.valid = false;
    d.reason = "all_sources_lost";
    return d;
  }

  // 切源 → 开启接管瞬态窗
  if (target != role_ && role_ != ActiveRole::kNone) {
    guard_until_s_ = in.now_s + params_.takeover_guard_s;
  }
  role_ = target;

  const StackObservation& src =
      role_ == ActiveRole::kPrimary ? in.primary : in.backup;
  common::ControlData cmd = src.cmd;

  // ── 输出平滑（跨源连续）──
  const bool guard = in.now_s < guard_until_s_;
  const double steer_rate =
      guard ? params_.guard_steer_rate_rps : params_.normal_steer_rate_rps;
  double accel_rate =
      guard ? params_.guard_accel_rate_mps3 : params_.normal_accel_rate_mps3;

  if (has_output_) {
    const double max_ds = steer_rate * in.dt;
    cmd.lateral.steering_tire_angle_rad =
        last_steer_ + std::clamp(cmd.lateral.steering_tire_angle_rad - last_steer_,
                                 -max_ds, max_ds);
    // AEB-seed 特例：新源紧急制动中，制动加深方向放开速率
    const double da = cmd.longitudinal.acceleration_mps2 - last_accel_;
    const double down_rate =
        (src.aeb_active && da < 0.0) ? params_.aeb_brake_rate_mps3 : accel_rate;
    cmd.longitudinal.acceleration_mps2 =
        last_accel_ + std::clamp(da, -down_rate * in.dt, accel_rate * in.dt);
  }
  last_steer_ = cmd.lateral.steering_tire_angle_rad;
  last_accel_ = cmd.longitudinal.acceleration_mps2;
  has_output_ = true;

  d.valid = true;
  d.cmd = cmd;
  d.role = role_;
  d.takeover_guard = guard;
  return d;
}

}  // namespace adas::system
