// PID 纵向控制器实现
#include "adas_trajectory_follower/pid_longitudinal.hpp"

#include <algorithm>
#include <cmath>

#include "adas_common/geometry.hpp"

namespace adas::control {

namespace ac = adas::common;

PidLongitudinal::PidLongitudinal(const PidLongitudinalParams& params)
    : params_(params),
      pid_({params.kp, params.ki, params.kd}, -params.max_decel_mps2, params.max_accel_mps2,
           -params.integral_limit, params.integral_limit) {}

void PidLongitudinal::reset() {
  pid_.reset();
  state_ = State::kRunning;
}

common::LongitudinalCommandData PidLongitudinal::run(const ControlInput& input) {
  common::LongitudinalCommandData cmd;
  if (input.trajectory == nullptr || input.trajectory->size() < 2) {
    cmd.acceleration_mps2 = params_.stop_hold_accel_mps2;  // 无参考 → 缓制动
    return cmd;
  }
  const auto& traj = *input.trajectory;
  const double v = input.state.velocity_mps;

  // 速度参考：最近点前方 preview_time*v 弧长处（低速时至少看 1m）
  const std::size_t nearest =
      ac::find_nearest_index(traj, input.state.pose.x, input.state.pose.y);
  const double preview = std::max(params_.preview_time_s * v, 1.0);
  const ac::TrajPoint ref = ac::point_at_arclength(traj, nearest, preview);
  const double v_ref = std::max(0.0, ref.velocity_mps);

  // 状态机
  if (state_ == State::kRunning) {
    if (v_ref < params_.stop_speed_mps && v < params_.stop_speed_mps) {
      state_ = State::kStopped;
      pid_.reset();
    }
  } else {  // kStopped
    if (v_ref > params_.start_speed_mps) {
      state_ = State::kRunning;
      pid_.reset();
    }
  }

  if (state_ == State::kStopped) {
    cmd.velocity_mps = 0.0;
    cmd.acceleration_mps2 = params_.stop_hold_accel_mps2;
    return cmd;
  }

  // RUNNING：剖面加速度前馈 + PID 反馈
  const double a_ff = ref.acceleration_mps2;
  const double a_fb = pid_.update(v_ref - v, input.dt);
  cmd.velocity_mps = v_ref;
  cmd.acceleration_mps2 =
      std::clamp(a_ff + a_fb, -params_.max_decel_mps2, params_.max_accel_mps2);
  return cmd;
}

}  // namespace adas::control
