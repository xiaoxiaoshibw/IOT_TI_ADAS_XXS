// Pure Pursuit 横向控制器实现
#include "adas_trajectory_follower/pure_pursuit_lateral.hpp"

#include <algorithm>
#include <cmath>

#include "adas_common/geometry.hpp"

namespace adas::control {

namespace ac = adas::common;

PurePursuitLateral::PurePursuitLateral(const PurePursuitParams& params) : params_(params) {}

void PurePursuitLateral::seed_from_steering_report(double steering_tire_angle_rad) {
  last_steer_ = std::clamp(steering_tire_angle_rad,
                           -params_.max_steer_rad, params_.max_steer_rad);
  seeded_ = true;
}

double PurePursuitLateral::compute_lookahead(double speed_mps,
                                             double preview_curvature) const {
  // base：高速直道保持约 12 m（避免蛇形）；低速不低于 min_lookahead_m（避免抖）。
  const double base =
      std::clamp(params_.base_speed_coeff * speed_mps + params_.base_speed_offset_m,
                 params_.min_lookahead_m, params_.max_lookahead_high_m);
  // 紧弯压缩：1 / (1 + curve_gain * |k|) ∈ [curve_factor_min, 1]
  const double abs_k = std::fabs(preview_curvature);
  const double curve_factor = std::clamp(1.0 / (1.0 + params_.curve_gain * abs_k),
                                         params_.curve_factor_min, 1.0);
  return base * curve_factor;
}

common::LateralCommandData PurePursuitLateral::run(const ControlInput& input) {
  common::LateralCommandData cmd;
  if (input.trajectory == nullptr || input.trajectory->size() < 2) {
    cmd.steering_tire_angle_rad = last_steer_;  // 保持上一帧
    return cmd;
  }
  const auto& traj = *input.trajectory;
  const auto& pose = input.state.pose;

  // Commit 4 — 首次 run() 时把 last_steer_ 用实际方向盘角度播种，避免
  // rotation_rate_rad_s 第一拍出现 (steer - 0.0) / dt 尖峰。
  if (!seeded_) {
    last_steer_ = std::clamp(input.steering_angle_rad,
                             -params_.max_steer_rad, params_.max_steer_rad);
    seeded_ = true;
  }

  // 自适应前视：base(speed) × curve_factor(curvature)。
  // preview curvature 取最近点前方一段距离上的平均 |k|（不只看当前点）。
  const std::size_t nearest = ac::find_nearest_index(traj, pose.x, pose.y);
  // 临时预算：先用 base 估算 preview，再重新采样以避免循环依赖。两次扫描代价
  // 可忽略（< 200 点），换来精确的 curvature envelope。
  const double rough_lookahead = compute_lookahead(input.state.velocity_mps, 0.0);
  const double preview_target =
      std::clamp(rough_lookahead, params_.min_lookahead_m, params_.max_lookahead_high_m);
  // 在 s ∈ [nearest, nearest + preview_target] 内对 |k| 取最大值
  double preview_max_k = 0.0;
  double arc = 0.0;
  std::size_t preview_idx = nearest;
  for (std::size_t i = nearest; i + 1 < traj.size() && arc < preview_target; ++i) {
    preview_max_k = std::max(preview_max_k, std::fabs(traj[i].curvature));
    arc += std::hypot(traj[i + 1].x - traj[i].x, traj[i + 1].y - traj[i].y);
    preview_idx = i + 1;
  }
  // 也算入下一段的曲率（避免漏掉弯道起点）
  if (preview_idx < traj.size()) {
    preview_max_k = std::max(preview_max_k,
                             std::fabs(traj[preview_idx].curvature));
  }
  const double lookahead =
      compute_lookahead(input.state.velocity_mps, preview_max_k);
  const ac::TrajPoint target = ac::point_at_arclength(traj, nearest, lookahead);

  // Pure Pursuit 几何：delta = atan(2 L sin(alpha) / Ld)
  const double alpha =
      ac::normalize_angle(std::atan2(target.y - pose.y, target.x - pose.x) - pose.yaw);
  const double raw =
      std::atan2(2.0 * params_.wheelbase_m * std::sin(alpha), lookahead);
  const double steer = std::clamp(raw, -params_.max_steer_rad, params_.max_steer_rad);

  cmd.steering_tire_angle_rad = steer;
  cmd.rotation_rate_rad_s = input.dt > 0.0 ? (steer - last_steer_) / input.dt : 0.0;
  last_steer_ = steer;
  return cmd;
}

}  // namespace adas::control
