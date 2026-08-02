// 仿真车辆核心实现
#include "adas_sim_vehicle/sim_vehicle_core.hpp"

#include <algorithm>
#include <cmath>

#include "adas_common/geometry.hpp"

namespace adas::sim {

namespace ac = adas::common;

SimVehicleCore::SimVehicleCore(const VehicleParams& params,
                               const std::vector<TrackSegment>& segments, double lane_width_m,
                               double sample_step_m)
    : params_(params), lane_width_(lane_width_m) {
  // 由分段圆弧生成中心线折线（起点 (0,0) 朝 +x）
  double cx = 0.0;
  double cy = 0.0;
  double cyaw = 0.0;
  for (const auto& seg : segments) {
    track_length_m_ += seg.length_m;
  }
  for (const auto& seg : segments) {
    const int n = std::max(1, static_cast<int>(seg.length_m / sample_step_m));
    const double ds = seg.length_m / n;
    for (int i = 0; i < n; ++i) {
      ac::TrajPoint p;
      p.x = cx;
      p.y = cy;
      p.yaw = cyaw;
      p.curvature = seg.curvature;
      centerline_.push_back(p);
      cyaw = ac::normalize_angle(cyaw + seg.curvature * ds);
      cx += std::cos(cyaw) * ds;
      cy += std::sin(cyaw) * ds;
    }
  }
  // 末点收尾
  ac::TrajPoint tail;
  tail.x = cx;
  tail.y = cy;
  tail.yaw = cyaw;
  tail.curvature = segments.empty() ? 0.0 : segments.back().curvature;
  centerline_.push_back(tail);
}

void SimVehicleCore::set_initial_state(double station_m, double lateral_m, double speed_mps) {
  const ac::TrajPoint p = ac::point_at_arclength(centerline_, 0, station_m);
  // 左偏为正：沿路径法向 (-sin, cos) 方向偏移放置
  x_ = p.x - std::sin(p.yaw) * lateral_m;
  y_ = p.y + std::cos(p.yaw) * lateral_m;
  yaw_ = p.yaw;
  v_ = std::max(0.0, speed_mps);
  steer_ = 0.0;
  yaw_rate_ = 0.0;
}

void SimVehicleCore::set_lead_script(const LeadScript& script) {
  lead_script_ = script;
  lead_station_ = script.initial_station_m;
  lead_v_ = std::max(0.0, script.initial_speed_mps);
}

void SimVehicleCore::set_pedestrian_script(const PedestrianScript& script) {
  ped_script_ = script;
  ped_walking_ = false;
  ped_done_ = false;
  ped_lateral_ = script.start_lateral_m;
}

void SimVehicleCore::set_adjacent_car_script(const AdjacentCarScript& script) {
  adj_script_ = script;
  adj_station_ = script.initial_station_m;
}

void SimVehicleCore::step(const common::ActuationData& actuation, double dt) {
  if (dt <= 0.0) {
    return;
  }
  sim_time_ += dt;

  // ── 行人推进（触发后沿法向匀速横穿）──
  if (ped_script_.enabled && !ped_done_) {
    if (!ped_walking_) {
      // 自车逼近横穿点到触发距离 → 开始横穿（用自车最近点弧长近似自车 station）
      const std::size_t ego_idx = ac::find_nearest_index(centerline_, x_, y_);
      const double ego_station =
          centerline_.empty() ? 0.0
                              : track_length_m_ * static_cast<double>(ego_idx) /
                                    static_cast<double>(centerline_.size() - 1);
      if (ped_script_.station_m - ego_station < ped_script_.trigger_ego_gap_m) {
        ped_walking_ = true;
      }
    }
    if (ped_walking_) {
      const double dir = ped_script_.end_lateral_m > ped_script_.start_lateral_m ? 1.0 : -1.0;
      ped_lateral_ += dir * ped_script_.speed_mps * dt;
      if ((dir > 0.0 && ped_lateral_ >= ped_script_.end_lateral_m) ||
          (dir < 0.0 && ped_lateral_ <= ped_script_.end_lateral_m)) {
        ped_done_ = true;
      }
    }
  }

  // ── 邻道车推进（spawn 后恒速）──
  if (adj_script_.enabled && sim_time_ >= adj_script_.spawn_time_s) {
    adj_station_ += adj_script_.speed_mps * dt;
  }

  // ── 前车推进（沿中心线，事件表变速）──
  if (lead_script_.enabled) {
    double target_v = lead_script_.initial_speed_mps;
    for (const auto& ev : lead_script_.events) {
      if (sim_time_ >= ev.first) {
        target_v = ev.second;
      }
    }
    target_v = std::max(0.0, target_v);
    const double dv = std::clamp(target_v - lead_v_, -lead_script_.accel_mps2 * dt,
                                 lead_script_.accel_mps2 * dt);
    lead_v_ = std::max(0.0, lead_v_ + dv);
    lead_station_ += lead_v_ * dt;
  }
  // 转向执行一阶惯性
  const double steer_target =
      std::clamp(actuation.steer, -1.0, 1.0) * params_.max_steer_rad;
  const double alpha =
      params_.steer_tau_s <= 0.0 ? 1.0 : dt / (params_.steer_tau_s + dt);
  steer_ += alpha * (steer_target - steer_);

  // 纵向：油门/制动 + 阻力；不支持倒车
  const double throttle = std::clamp(actuation.throttle, 0.0, 1.0);
  const double brake = std::clamp(actuation.brake, 0.0, 1.0);
  double accel = throttle * params_.max_accel_mps2 - brake * params_.max_decel_mps2;
  if (v_ > 0.0) {
    accel -= params_.drag_accel_mps2;
  }
  v_ = std::max(0.0, v_ + accel * dt);

  // 运动学自行车模型
  yaw_rate_ = v_ / params_.wheelbase_m * std::tan(steer_);
  yaw_ = ac::normalize_angle(yaw_ + yaw_rate_ * dt);
  x_ += v_ * std::cos(yaw_) * dt;
  y_ += v_ * std::sin(yaw_) * dt;
}

common::KinematicState SimVehicleCore::state() const {
  common::KinematicState s;
  s.pose = common::Pose2d{x_, y_, yaw_};
  s.velocity_mps = v_;
  s.yaw_rate_rps = yaw_rate_;
  return s;
}

LeadState SimVehicleCore::lead_state() const {
  LeadState ls;
  if (!lead_script_.enabled || centerline_.size() < 2 || lead_station_ > track_length_m_) {
    return ls;  // 未启用或驶出赛道末端 → present=false
  }
  const ac::TrajPoint p = ac::point_at_arclength(centerline_, 0, lead_station_);
  ls.present = true;
  ls.x = p.x;
  ls.y = p.y;
  ls.yaw = p.yaw;
  ls.v_mps = lead_v_;
  ls.station_m = lead_station_;
  return ls;
}

LeadState SimVehicleCore::adjacent_car_state() const {
  LeadState ls;
  if (!adj_script_.enabled || sim_time_ < adj_script_.spawn_time_s ||
      centerline_.size() < 2 || adj_station_ > track_length_m_) {
    return ls;  // present=false
  }
  const ac::TrajPoint p = ac::point_at_arclength(centerline_, 0, adj_station_);
  ls.present = true;
  ls.x = p.x - std::sin(p.yaw) * adj_script_.lateral_m;
  ls.y = p.y + std::cos(p.yaw) * adj_script_.lateral_m;
  ls.yaw = p.yaw;
  ls.v_mps = adj_script_.speed_mps;
  ls.station_m = adj_station_;
  return ls;
}

PedestrianState SimVehicleCore::pedestrian_state() const {
  PedestrianState ps;
  if (!ped_script_.enabled || ped_done_ || centerline_.size() < 2) {
    return ps;  // present=false
  }
  const ac::TrajPoint p = ac::point_at_arclength(centerline_, 0, ped_script_.station_m);
  // 沿路径法向 (-sin, cos) 偏移（左正）
  ps.present = true;
  ps.x = p.x - std::sin(p.yaw) * ped_lateral_;
  ps.y = p.y + std::cos(p.yaw) * ped_lateral_;
  const double dir = ped_script_.end_lateral_m > ped_script_.start_lateral_m ? 1.0 : -1.0;
  ps.yaw = ac::normalize_angle(p.yaw + dir * ac::kPi / 2.0);  // 行走方向 = 法向
  ps.v_mps = ped_walking_ ? ped_script_.speed_mps : 0.0;
  ps.lateral_m = ped_lateral_;
  return ps;
}

common::LaneStateData SimVehicleCore::lane_state() const {
  common::LaneStateData ls;
  if (centerline_.size() < 2) {
    return ls;  // valid=false
  }
  const std::size_t idx = ac::find_nearest_index(centerline_, x_, y_);
  // 接近赛道末端（最后两点）视为驶出赛道
  if (idx + 2 >= centerline_.size()) {
    return ls;  // valid=false
  }
  ls.valid = true;
  ls.lateral_offset = ac::signed_lateral_offset(centerline_, x_, y_);
  ls.heading_error = ac::normalize_angle(yaw_ - centerline_[idx].yaw);
  ls.curvature = centerline_[idx].curvature;
  ls.lane_width = lane_width_;
  return ls;
}

}  // namespace adas::sim
