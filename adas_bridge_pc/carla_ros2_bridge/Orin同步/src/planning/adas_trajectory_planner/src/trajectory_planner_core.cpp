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

  // Commit 6a — 单段速度变化率软上限（仅对加速度方向夹紧）。减速方向
  // 保持 max_decel_mps2 的运动学上限，否则静止前车/终点停车剖面会
  // 因为"舒适限速"而停不住。max_dv_pos = accel_rate_limit * ds / v_prev。
  const double accel_rate = std::max(params_.accel_rate_limit, 1e-3);
  for (std::size_t i = 1; i < traj.size(); ++i) {
    const double v_prev = traj[i - 1].velocity_mps;
    const double v_curr = traj[i].velocity_mps;
    const double ds = std::hypot(traj[i].x - traj[i - 1].x,
                                  traj[i].y - traj[i - 1].y);
    if (ds < 1e-6) continue;
    const double max_dv_pos = accel_rate * ds / std::max(v_prev, 0.5);
    if (v_curr > v_prev + max_dv_pos) {
      traj[i].velocity_mps = v_prev + max_dv_pos;
      traj[i].acceleration_mps2 = (traj[i].velocity_mps * traj[i].velocity_mps -
                                   v_prev * v_prev) / (2.0 * ds);
    }
  }
  return traj;
}

common::Trajectory TrajectoryPlannerCore::avoid_obstacles_laterally(
    const common::Trajectory& reference,
    const std::vector<StaticObstacle>& obstacles) const {
  if (!params_.lateral_avoidance_enabled || reference.size() < 2U) return reference;

  double reference_length = 0.0;
  for (std::size_t i = 1; i < reference.size(); ++i) {
    reference_length += ac::distance2d(reference[i - 1].x, reference[i - 1].y,
                                       reference[i].x, reference[i].y);
  }
  const double max_offset = 0.5 * params_.lane_width_m;
  double offset = 0.0;
  for (const auto& obstacle : obstacles) {
    if (!obstacle.is_static || !std::isfinite(obstacle.longitudinal_m) ||
        !std::isfinite(obstacle.lateral_m) || obstacle.longitudinal_m < 0.0 ||
        obstacle.longitudinal_m > reference_length ||
        std::fabs(obstacle.lateral_m) > max_offset) {
      continue;
    }
    // Minimal framework: move to the positive lateral side with one lane-half
    // of clearance, then clamp to the supported half-lane limit.
    offset = std::clamp(obstacle.lateral_m + max_offset, -max_offset, max_offset);
    break;  // deliberately one static obstacle only in the first iteration
  }
  if (std::fabs(offset) <= 1e-9) return reference;

  common::Trajectory shifted = reference;
  for (auto& point : shifted) {
    point.x -= std::sin(point.yaw) * offset;
    point.y += std::cos(point.yaw) * offset;
  }
  return shifted;
}

common::Trajectory TrajectoryPlannerCore::plan_global_route(
    const common::KinematicState& ego, const common::Trajectory& route,
    double cruise_speed_mps, double goal_stop_distance_m,
    bool stop_at_route_end, const LeadInfo& lead,
    const std::vector<StaticObstacle>& obstacles) const {
  common::Trajectory result;
  const common::Trajectory working_route =
      params_.lateral_avoidance_enabled ? avoid_obstacles_laterally(route, obstacles) : route;
  const auto& planning_route = working_route;
  if (planning_route.size() < 2U) return result;

  const double cruise = std::max(0.0, cruise_speed_mps);
  const double max_decel = std::max(params_.max_decel_mps2, 1e-3);
  const double max_lat_accel = std::max(params_.max_lat_accel_mps2, 1e-3);
  const std::size_t count = planning_route.size();

  // Commit 2 — lead-vehicle pre-cap inputs (computed once outside the loop).
  const double v_lead = lead.present ? std::max(0.0, lead.speed_mps) : 0.0;
  // t_est proxy: assume ego's current speed all the way through. Actual
  // realized t_est will be similar or larger whenego slows for bends, so the
  // lead co-move shrinks and the cap relaxes — the correct physical direction.
  const double t_est_denom = std::max(ego.velocity_mps, 0.5);

  std::vector<double> ds(count, 0.0);
  std::vector<double> station(count, 0.0);
  for (std::size_t i = 1U; i < count; ++i) {
    ds[i] = std::max(0.01, ac::distance2d(planning_route[i].x, planning_route[i].y,
                                           planning_route[i - 1U].x,
                                           planning_route[i - 1U].y));
    station[i] = station[i - 1U] + ds[i];
  }

  // Commit 3 — yaw unwrap pass. 逐点累计 yaw 增量，避免 ±π 跨界后
  // atan2(sin, cos) 把整段曲率符号翻转。原始 yaw 序列来自 map 中心线采样，
  // CARLA 的 2 m 步长偶尔跨过整圈时尤其重要。
  std::vector<double> yaw_unwrap(count, 0.0);
  yaw_unwrap[0U] = planning_route[0U].yaw;
  for (std::size_t i = 1U; i < count; ++i) {
    const double delta = std::atan2(
        std::sin(planning_route[i].yaw - planning_route[i - 1U].yaw),
        std::cos(planning_route[i].yaw - planning_route[i - 1U].yaw));
    yaw_unwrap[i] = yaw_unwrap[i - 1U] + delta;
  }

  // 中央差分曲率（基于解包后的 yaw）
  std::vector<double> curvature(count, 0.0);
  for (std::size_t i = 0U; i < count; ++i) {
    if (i == 0U) {
      curvature[i] = (yaw_unwrap[1U] - yaw_unwrap[0U]) / ds[1U];
    } else if (i + 1U == count) {
      curvature[i] = (yaw_unwrap[i] - yaw_unwrap[i - 1U]) / ds[i];
    } else {
      const double span = ds[i] + ds[i + 1U];
      curvature[i] = (yaw_unwrap[i + 1U] - yaw_unwrap[i - 1U]) / span;
    }
    if (!std::isfinite(curvature[i])) curvature[i] = 0.0;
  }

  // 5 点加权平滑 (1-2-4-2-1)/10。边缘透传。锐化 2 m 步长带来的非物理尖峰：
  // 单点突变被 ±2 点平均掉，整段弯道曲率更接近真实几何。
  std::vector<double> curvature_smooth(count, 0.0);
  for (std::size_t i = 0U; i < count; ++i) {
    if (i < 2U || i + 2U >= count) {
      curvature_smooth[i] = curvature[i];
      continue;
    }
    curvature_smooth[i] = (curvature[i - 2U] + 2.0 * curvature[i - 1U] +
                           4.0 * curvature[i] + 2.0 * curvature[i + 1U] +
                           curvature[i + 2U]) /
                          10.0;
  }

  // 在平滑后的曲率上计算限速，使用前方 8-15 m 的弯道包络
  // （不是只看当前点），避免单点异常把整段速度压到零。
  // envelope_m 在 yaml 中可调，默认 12 m（中央值）。
  const double envelope_m = std::clamp(
      params_.global_route_curvature_envelope_m, 8.0, 15.0);
  const double ds_avg = (count > 1U && station.back() > 0.0)
                            ? station.back() / static_cast<double>(count - 1U)
                            : 1.0;
  const std::size_t lookahead_pts =
      std::max<std::size_t>(1U, static_cast<std::size_t>(
                                     std::round(envelope_m / std::max(ds_avg, 1e-3))));

  std::vector<double> cap(count, cruise);
  for (std::size_t i = 0U; i < count; ++i) {
    const double remaining = stop_at_route_end
                                 ? std::max(0.0, station.back() - station[i] -
                                                     std::max(0.0, goal_stop_distance_m))
                                 : 0.0;
    const double stop_cap = stop_at_route_end
                                ? std::sqrt(2.0 * max_decel * remaining)
                                : cruise;
    // Envelope-based curve cap: 包络前方 lookahead_pts 点的最大 |k|，
    // 然后 v = sqrt(max_lat_accel / max_k)。
    const std::size_t cap_end = std::min(count, i + 1U + lookahead_pts);
    double max_abs_k = 0.0;
    for (std::size_t j = i; j < cap_end; ++j) {
      max_abs_k = std::max(max_abs_k, std::fabs(curvature_smooth[j]));
    }
    const double curve_cap =
        std::sqrt(max_lat_accel / std::max(max_abs_k, 1e-6));
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

  // Commit 3 — acceleration-continuity post-pass. 按 max_decel * ds 约束
  // 段间加速度跳跃，避免相邻轨迹点从 15 m/s 突然掉到 3 m/s 的跃变
  // （PID 会把这种跃变当作阶跃干扰而起跳）。双向限幅（不只是减速侧）。
  for (std::size_t i = 1U; i < count; ++i) {
    const double max_dv = std::sqrt(speed[i - 1U] * speed[i - 1U] +
                                    2.0 * max_decel * ds[i]) -
                          speed[i - 1U];
    const double dv = speed[i] - speed[i - 1U];
    if (dv > max_dv) {
      speed[i] = speed[i - 1U] + max_dv;
    } else if (dv < -max_dv) {
      speed[i] = speed[i - 1U] - max_dv;
    }
  }

  result.reserve(count);
  double elapsed = 0.0;
  for (std::size_t i = 0U; i < count; ++i) {
    auto point = planning_route[i];
    point.curvature = curvature_smooth[i];
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

  // Commit 6a — 单段速度变化率软上限（仅加速方向夹紧，减速交给 max_decel）。
  // 在 Commit 3 的 backward pass + continuity post-pass 之后再做一次软夹紧。
  const double accel_rate = std::max(params_.accel_rate_limit, 1e-3);
  for (std::size_t i = 1U; i < result.size(); ++i) {
    const double v_prev = result[i - 1U].velocity_mps;
    const double v_curr = result[i].velocity_mps;
    if (ds[i] < 1e-6) continue;
    const double max_dv_pos = accel_rate * ds[i] / std::max(v_prev, 0.5);
    if (v_curr > v_prev + max_dv_pos) {
      result[i].velocity_mps = v_prev + max_dv_pos;
      result[i].acceleration_mps2 = (result[i].velocity_mps * result[i].velocity_mps -
                                    v_prev * v_prev) / (2.0 * ds[i]);
    }
  }
  return result;
}

}  // namespace adas::planning
