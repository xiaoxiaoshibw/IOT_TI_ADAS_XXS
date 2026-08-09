// 轨迹规划核心实现
#include "adas_trajectory_planner/trajectory_planner_core.hpp"

#include <algorithm>
#include <cmath>

#include "adas_common/geometry.hpp"

namespace adas::planning {

namespace ac = adas::common;

TrajectoryPlannerCore::TrajectoryPlannerCore(const PlannerParams& params) : params_(params) {}

common::Trajectory TrajectoryPlannerCore::plan(const common::KinematicState& ego,
                                               const common::LaneStateData& lane,
                                               const LeadInfo& lead,
                                               double cruise_override_mps,
                                               int target_lane) {
  common::Trajectory traj;
  if (!lane.valid) {
    return traj;  // 空轨迹 = 无可行参考，下游超时降级
  }

  // 1. 由自车位姿反解车道中心线上的投影点：
  //    路径切向 = 自车航向 - 航向误差；自车在路径左侧 lateral_offset 处，
  //    投影点 = 自车位置 - 法向(-sin,cos) * lateral_offset
  const double path_yaw = ac::normalize_angle(ego.pose.yaw - lane.heading_error);
  const double nx = -std::sin(path_yaw);
  const double ny = std::cos(path_yaw);
  double px = ego.pose.x - nx * lane.lateral_offset;
  double py = ego.pose.y - ny * lane.lateral_offset;

  // 2. 位置无关限速：巡航（可被 behavior 覆盖）∧ 曲率限速
  double v_free = cruise_override_mps >= 0.0 ? cruise_override_mps : params_.cruise_speed_mps;
  const double abs_k = std::fabs(lane.curvature);
  if (abs_k > 1e-6) {
    v_free = std::min(v_free, std::sqrt(params_.max_lat_accel_mps2 / abs_k));
  }
  v_free = std::max(0.0, v_free);
  const double v_lead = lead.present ? std::max(0.0, lead.speed_mps) : 0.0;

  // 3. 轨迹长度：时间视界 × 当前速度，限 [min, max]
  const double length = std::clamp(std::max(ego.velocity_mps, v_free) * params_.horizon_s,
                                   params_.min_length_m, params_.max_length_m);

  // 变道横移（M4）：固定锚点五次多项式（详见头文件注释）。
  const double lat_target = -static_cast<double>(target_lane) * params_.lane_width_m;
  // 目标车道变化（含变道中途改目标 = 中止回线）→ 开启新过渡，锚定当前状态
  if (std::fabs(lat_target - last_target_lat_) > 0.2) {
    lc_active_ = true;
    lc_from_lat_ = lane.lateral_offset;
    lc_target_lat_ = lat_target;
    lc_start_x_ = ego.pose.x;
    lc_start_y_ = ego.pose.y;
    lc_len_ = std::max(params_.lane_change_min_len_m,
                       std::max(ego.velocity_mps, 5.0) * params_.lane_change_time_s);
  }
  last_target_lat_ = lat_target;
  double lc_progress = 0.0;
  if (lc_active_) {
    lc_progress = ac::distance2d(ego.pose.x, ego.pose.y, lc_start_x_, lc_start_y_);
    if (lc_progress >= lc_len_) {
      lc_active_ = false;  // 过渡完成 → 常量偏移
    }
  }
  const auto lateral_shift_at = [&](double s) {
    if (!lc_active_) {
      return lat_target;
    }
    const double u = std::clamp((lc_progress + s) / lc_len_, 0.0, 1.0);
    const double blend = 10.0 * u * u * u - 15.0 * u * u * u * u + 6.0 * u * u * u * u * u;
    return lc_from_lat_ + (lc_target_lat_ - lc_from_lat_) * blend;
  };

  // 位置相关限速：跟车逼近曲线（对标 IDM 稳态：gap → d0 + T·v_lead）。
  // 关键：包络随前车共移——剖面点 (s, t) 处前车已前移 v_lead·t，
  // 否则冻结包络会给前视点/加速度前馈引入系统性下坡偏置，稳态时距偏大。
  const auto v_allow_at = [&](double s, double t_est) {
    double v_allow = v_free;
    if (lead.present) {
      const double follow_dist = lead.gap_m + v_lead * t_est - params_.follow_standstill_m -
                                 params_.follow_time_gap_s * v_lead - s;
      const double v2 = v_lead * v_lead + 2.0 * params_.max_decel_mps2 * follow_dist;
      v_allow = std::min(v_allow, v2 > 0.0 ? std::sqrt(v2) : 0.0);
    }
    return v_allow;
  };

  // 4. 沿常曲率弧采样，速度按加减速度限制逼近逐点限速
  double yaw = path_yaw;
  double v = std::max(0.0, ego.velocity_mps);
  double t = 0.0;
  double s = 0.0;
  const int n = std::max(2, static_cast<int>(length / params_.step_m) + 1);
  traj.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    const double ds = params_.step_m;
    // 推进后的速度：向下一点限速收敛且不越过（下一点时刻用当前速度估计）
    const double t_next_est = t + ds / std::max(v, 0.1);
    const double v_allow_next = v_allow_at(s + ds, t_next_est);
    double v_next = v;
    if (v < v_allow_next) {
      v_next = std::min(v_allow_next, std::sqrt(v * v + 2.0 * params_.max_accel_mps2 * ds));
    } else if (v > v_allow_next) {
      const double v2 = v * v - 2.0 * params_.max_decel_mps2 * ds;
      v_next = std::max(v_allow_next, v2 > 0.0 ? std::sqrt(v2) : 0.0);
    }

    // 沿基准弧法向叠加变道横移；航向按横移斜率修正
    const double shift = lateral_shift_at(s);
    const double shift_slope = (lateral_shift_at(s + ds) - shift) / ds;
    ac::TrajPoint p;
    p.x = px - std::sin(yaw) * shift;
    p.y = py + std::cos(yaw) * shift;
    p.yaw = ac::normalize_angle(yaw + std::atan(shift_slope));
    p.curvature = lane.curvature;
    p.velocity_mps = v;
    p.time_from_start_s = t;
    // 剖面加速度 = 段运动学一致值（纵向前馈）：a = (v'² − v²) / 2ds
    p.acceleration_mps2 = (v_next * v_next - v * v) / (2.0 * ds);
    traj.push_back(p);

    // 时间推进用段平均速度（低速地板防除零）
    const double v_seg = std::max(0.5 * (v + v_next), 0.1);
    t += ds / v_seg;
    v = v_next;
    s += ds;
    yaw = ac::normalize_angle(yaw + lane.curvature * ds);
    px += std::cos(yaw) * ds;
    py += std::sin(yaw) * ds;
  }
  return traj;
}

common::Trajectory TrajectoryPlannerCore::plan_global_route(
    const common::KinematicState& ego, const common::Trajectory& route,
    double cruise_speed_mps, double goal_stop_distance_m,
    bool stop_at_route_end, const LeadInfo& lead) const {
  common::Trajectory result;
  if (route.size() < 2U) return result;

  const double cruise = std::max(0.0, cruise_speed_mps);
  const double max_decel = std::max(params_.max_decel_mps2, 1e-3);
  const double max_lat_accel = std::max(params_.max_lat_accel_mps2, 1e-3);
  const std::size_t count = route.size();

  // Commit 2 — lead-vehicle pre-cap inputs (computed once outside the loop).
  const double v_lead = lead.present ? std::max(0.0, lead.speed_mps) : 0.0;
  // t_est proxy: assume ego's current speed all the way through. Actual
  // realized t_est will be similar or larger whenego slows for bends, so the
  // lead co-move shrinks and the cap relaxes — the correct physical direction.
  const double t_est_denom = std::max(ego.velocity_mps, 0.5);

  std::vector<double> ds(count, 0.0);
  std::vector<double> station(count, 0.0);
  std::vector<double> curvature(count, 0.0);
  const auto yaw_delta = [](double from, double to) {
    return std::atan2(std::sin(to - from), std::cos(to - from));
  };
  for (std::size_t i = 1U; i < count; ++i) {
    ds[i] = std::max(0.01, ac::distance2d(route[i].x, route[i].y,
                                           route[i - 1U].x, route[i - 1U].y));
    station[i] = station[i - 1U] + ds[i];
  }
  for (std::size_t i = 0U; i < count; ++i) {
    if (i == 0U) {
      curvature[i] = yaw_delta(route[0U].yaw, route[1U].yaw) / ds[1U];
    } else if (i + 1U == count) {
      curvature[i] = yaw_delta(route[i - 1U].yaw, route[i].yaw) / ds[i];
    } else {
      const double span = ds[i] + ds[i + 1U];
      curvature[i] = yaw_delta(route[i - 1U].yaw, route[i + 1U].yaw) / span;
    }
    if (!std::isfinite(curvature[i])) curvature[i] = 0.0;
  }

  std::vector<double> cap(count, cruise);
  for (std::size_t i = 0U; i < count; ++i) {
    const double remaining = stop_at_route_end
                                 ? std::max(0.0, station.back() - station[i] -
                                                     std::max(0.0, goal_stop_distance_m))
                                 : 0.0;
    const double stop_cap = stop_at_route_end
                                ? std::sqrt(2.0 * max_decel * remaining)
                                : cruise;
    const double curve_cap = std::sqrt(max_lat_accel / std::max(std::fabs(curvature[i]),
                                                                 1e-6));
    cap[i] = std::min({cruise, stop_cap, curve_cap});
    // Commit 2 — 4th cap: lead-vehicle. 当 lead.present 时把巡航剖面压到
    // v_lead ≤ cap_curve：包络随前车共移，路线长 ≥ 短时距时不至于到门口才减速。
    if (lead.present && lead.gap_m > 0.0) {
      const double t_est = station[i] / t_est_denom;
      const double follow_dist = lead.gap_m + v_lead * t_est -
                                 params_.global_route_follow_standstill_m -
                                 params_.global_route_follow_time_gap_s * v_lead -
                                 station[i];
      const double v2 = v_lead * v_lead + 2.0 * max_decel * follow_dist;
      const double lead_cap_local = v2 > 0.0 ? std::sqrt(v2) : 0.0;
      // 不能超过前车实速——物理上到达前车速度后无法再跟随
      const double lead_cap = std::min(lead_cap_local, v_lead);
      cap[i] = std::min(cap[i], lead_cap);
    }
  }

  // Backward pass makes the vehicle slow down before, rather than inside, a
  // bend. A forward-only profile sees the low speed cap too late by one
  // sample and is especially unsafe with the 2 m CARLA map sampling.
  for (std::size_t i = count - 1U; i > 0U; --i) {
    const double reachable = std::sqrt(cap[i] * cap[i] + 2.0 * max_decel * ds[i]);
    cap[i - 1U] = std::min(cap[i - 1U], reachable);
  }

  std::vector<double> speed(count, 0.0);
  speed[0] = std::min(std::max(0.0, ego.velocity_mps), cap[0]);
  for (std::size_t i = 1U; i < count; ++i) {
    const double accel_reachable =
        std::sqrt(speed[i - 1U] * speed[i - 1U] +
                  2.0 * std::max(params_.max_accel_mps2, 1e-3) * ds[i]);
    speed[i] = std::min(cap[i], accel_reachable);
  }

  result.reserve(count);
  double elapsed = 0.0;
  for (std::size_t i = 0U; i < count; ++i) {
    auto point = route[i];
    point.curvature = curvature[i];
    point.velocity_mps = speed[i];
    point.time_from_start_s = elapsed;
    if (i > 0U) {
      point.acceleration_mps2 =
          (speed[i] * speed[i] - speed[i - 1U] * speed[i - 1U]) /
          (2.0 * ds[i]);
      elapsed += ds[i] / std::max(0.5 * (speed[i] + speed[i - 1U]), 0.1);
      point.time_from_start_s = elapsed;
    }
    result.push_back(point);
  }
  return result;
}

}  // namespace adas::planning
