// Pure Pursuit 横向控制器实现
#include "adas_trajectory_follower/pure_pursuit_lateral.hpp"

#include <algorithm>
#include <cmath>

#include "adas_common/geometry.hpp"

namespace adas::control {

namespace ac = adas::common;

PurePursuitLateral::PurePursuitLateral(const PurePursuitParams& params) : params_(params) {}

common::LateralCommandData PurePursuitLateral::run(const ControlInput& input) {
  common::LateralCommandData cmd;
  if (input.trajectory == nullptr || input.trajectory->size() < 2) {
    cmd.steering_tire_angle_rad = last_steer_;  // 保持上一帧（调用方已保证不常态发生）
    return cmd;
  }
  const auto& traj = *input.trajectory;
  const auto& pose = input.state.pose;

  // 前视点：最近点前方 Ld 弧长处
  const double lookahead = std::clamp(params_.lookahead_gain_s * input.state.velocity_mps,
                                      params_.min_lookahead_m, params_.max_lookahead_m);
  const std::size_t nearest = ac::find_nearest_index(traj, pose.x, pose.y);
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
