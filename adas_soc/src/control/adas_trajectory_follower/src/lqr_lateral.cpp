// LQR 横向控制器实现（3 状态：横向误差 / 航向误差 / 转向执行状态）
//
// 二状态版本（M7 初版）忽略转向执行一阶迟滞，离线急弯与在线（含 DDS 往返延迟）
// 均发散——迟滞在环内引入 ~90° 相位损失，误差反馈变成激励。修复：把执行
// 转角 δ_act 增广为第三状态（用 steering_report 回读闭环）：
//   e_lat' = e_lat + v·e_yaw·dt
//   e_yaw' = e_yaw + (v/L)·δ_act·dt − v·κ·dt
//   δ_act' = (1−α)·δ_act + α·u，α = dt/(τ+dt)（与执行器一阶惯性同构）
// 曲率前馈进入稳态偏置：u = δ_ff − K·[e_lat, e_yaw, δ_act − δ_ff]
#include "adas_trajectory_follower/lqr_lateral.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "adas_common/geometry.hpp"

namespace adas::control {

namespace ac = adas::common;

namespace {

using Mat3 = std::array<double, 9>;  // 行主序
using Vec3 = std::array<double, 3>;

Mat3 mat_mul(const Mat3& a, const Mat3& b) {
  Mat3 c{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k) {
        s += a[i * 3 + k] * b[k * 3 + j];
      }
      c[i * 3 + j] = s;
    }
  }
  return c;
}

Mat3 mat_transpose(const Mat3& a) {
  Mat3 t{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      t[j * 3 + i] = a[i * 3 + j];
    }
  }
  return t;
}

Vec3 mat_vec(const Mat3& a, const Vec3& v) {
  Vec3 r{};
  for (int i = 0; i < 3; ++i) {
    r[i] = a[i * 3 + 0] * v[0] + a[i * 3 + 1] * v[1] + a[i * 3 + 2] * v[2];
  }
  return r;
}

}  // namespace

LqrLateral::LqrLateral(const LqrLateralParams& params) : params_(params) {
  for (double v = params_.v_grid_min; v <= params_.v_grid_max + 1e-9;
       v += params_.v_grid_step) {
    Gain g{};
    solve_gain(v, g);
    gains_.push_back(g);
  }
}

// 离散 Riccati 定点迭代（3 状态、标量控制）：
//   P ← Q + Aᵀ(P − P B Bᵀ P / (R + BᵀPB))A；K = BᵀPA / (R + BᵀPB)
void LqrLateral::solve_gain(double v, Gain& gain) const {
  const double dt = params_.dt_s;
  const double alpha =
      params_.steer_tau_s <= 0.0 ? 1.0 : dt / (params_.steer_tau_s + dt);
  const Mat3 a = {1.0, v * dt, 0.0,
                  0.0, 1.0,    v * dt / params_.wheelbase_m,
                  0.0, 0.0,    1.0 - alpha};
  const Vec3 b = {0.0, 0.0, alpha};
  const Mat3 at = mat_transpose(a);

  Mat3 p = {params_.q_lat, 0, 0, 0, params_.q_yaw, 0, 0, 0, 0.0};
  for (int iter = 0; iter < params_.riccati_iters; ++iter) {
    const Vec3 pb = mat_vec(p, b);              // P B
    const double s = params_.r_steer + b[0] * pb[0] + b[1] * pb[1] + b[2] * pb[2];
    // M = P − (P B)(P B)ᵀ / s
    Mat3 m = p;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        m[i * 3 + j] -= pb[i] * pb[j] / s;
      }
    }
    Mat3 next = mat_mul(mat_mul(at, m), a);
    next[0] += params_.q_lat;
    next[4] += params_.q_yaw;
    double diff = 0.0;
    for (int i = 0; i < 9; ++i) {
      diff += std::fabs(next[i] - p[i]);
    }
    p = next;
    if (diff < 1e-10) {
      break;
    }
  }
  const Vec3 pb = mat_vec(p, b);
  const double s = params_.r_steer + b[0] * pb[0] + b[1] * pb[1] + b[2] * pb[2];
  const Vec3 pa_row = mat_vec(mat_transpose(mat_mul(p, a)), b);  // (BᵀPA)ᵀ = AᵀPᵀB
  gain.k_lat = pa_row[0] / s;
  gain.k_yaw = pa_row[1] / s;
  gain.k_steer = pa_row[2] / s;
}

common::LateralCommandData LqrLateral::run(const ControlInput& input) {
  common::LateralCommandData cmd;
  if (input.trajectory == nullptr || input.trajectory->size() < 2) {
    cmd.steering_tire_angle_rad = last_steer_;
    return cmd;
  }
  const auto& traj = *input.trajectory;
  const auto& pose = input.state.pose;
  const double v = std::max(input.state.velocity_mps, 0.1);

  const std::size_t nearest = ac::find_nearest_index(traj, pose.x, pose.y);
  const ac::TrajPoint ref =
      ac::point_at_arclength(traj, nearest, std::max(params_.preview_s * v, 0.5));

  const double e_lat = ac::signed_lateral_offset(traj, pose.x, pose.y);
  const double e_yaw = ac::normalize_angle(pose.yaw - ref.yaw);

  // 增益调度插值
  const double v_clamped = std::clamp(v, params_.v_grid_min, params_.v_grid_max);
  const double idx_f = (v_clamped - params_.v_grid_min) / params_.v_grid_step;
  const std::size_t i0 = std::min(static_cast<std::size_t>(idx_f), gains_.size() - 1);
  const std::size_t i1 = std::min(i0 + 1, gains_.size() - 1);
  const double t = std::clamp(idx_f - static_cast<double>(i0), 0.0, 1.0);
  const double k_lat = gains_[i0].k_lat + t * (gains_[i1].k_lat - gains_[i0].k_lat);
  const double k_yaw = gains_[i0].k_yaw + t * (gains_[i1].k_yaw - gains_[i0].k_yaw);
  const double k_steer = gains_[i0].k_steer + t * (gains_[i1].k_steer - gains_[i0].k_steer);

  // u = δ_ff − K·[e_lat, e_yaw, δ_act − δ_ff]（执行状态用转角回读闭环）
  const double delta_ff = std::atan(params_.wheelbase_m * ref.curvature);
  const double raw = delta_ff - k_lat * e_lat - k_yaw * e_yaw -
                     k_steer * (input.steering_angle_rad - delta_ff);
  const double steer = std::clamp(raw, -params_.max_steer_rad, params_.max_steer_rad);

  cmd.steering_tire_angle_rad = steer;
  cmd.rotation_rate_rad_s = input.dt > 0.0 ? (steer - last_steer_) / input.dt : 0.0;
  last_steer_ = steer;
  return cmd;
}

}  // namespace adas::control
