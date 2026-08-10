// 仿真车辆核心实现
#include "adas_sim_vehicle/sim_vehicle_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

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
  ScriptedActor actor;
  actor.id = 1;
  actor.classification = ScriptedActorClass::Car;
  actor.initial_station_m = script.initial_station_m;
  actor.initial_speed_mps = script.initial_speed_mps;
  actor.accel_limit_mps2 = script.accel_mps2;
  actor.speed_profile = {{0.0, script.initial_speed_mps}};
  actor.speed_profile.insert(actor.speed_profile.end(), script.events.begin(),
                             script.events.end());
  upsert_legacy_actor(actor, script.enabled);
}

void SimVehicleCore::set_pedestrian_script(const PedestrianScript& script) {
  ScriptedActor actor;
  actor.id = 2;
  actor.classification = ScriptedActorClass::Pedestrian;
  actor.initial_station_m = script.station_m;
  actor.initial_lateral_m = script.start_lateral_m;
  actor.initial_speed_mps = script.speed_mps;
  actor.accel_limit_mps2 = std::max(script.speed_mps, 0.1);
  actor.speed_profile = {{0.0, script.speed_mps}};
  actor.trigger_ego_gap_m = script.trigger_ego_gap_m;
  actor.crossing_end_lateral_m = script.end_lateral_m;
  actor.crossing_speed_mps = script.speed_mps;
  upsert_legacy_actor(actor, script.enabled);
}

void SimVehicleCore::set_adjacent_car_script(const AdjacentCarScript& script) {
  ScriptedActor actor;
  actor.id = 3;
  actor.classification = ScriptedActorClass::Car;
  actor.initial_station_m = script.initial_station_m;
  actor.initial_lateral_m = script.lateral_m;
  actor.initial_speed_mps = script.speed_mps;
  actor.accel_limit_mps2 = 2.0;
  actor.speed_profile = {{0.0, script.speed_mps}};
  actor.spawn_time_s = script.spawn_time_s;
  upsert_legacy_actor(actor, script.enabled);
}

void SimVehicleCore::upsert_legacy_actor(const ScriptedActor& actor, bool enabled) {
  std::vector<ScriptedActor> configs;
  configs.reserve(actors_.size() + 1);
  for (const auto& runtime : actors_) {
    if (runtime.config.id != actor.id) configs.push_back(runtime.config);
  }
  if (enabled) configs.push_back(actor);
  set_scripted_actors(configs);
}

void SimVehicleCore::set_scripted_actors(const std::vector<ScriptedActor>& actors) {
  if (actors.size() > kMaxScriptedActors) {
    throw std::invalid_argument("scripted actor count exceeds safety limit");
  }
  std::unordered_set<std::uint32_t> ids;
  std::vector<ActorRuntime> next;
  next.reserve(actors.size());
  for (const auto& actor : actors) {
    if (actor.id == 0 || !ids.insert(actor.id).second) {
      throw std::invalid_argument("scripted actor IDs must be positive and unique");
    }
    if (!std::isfinite(actor.initial_station_m) ||
        !std::isfinite(actor.initial_lateral_m) ||
        !std::isfinite(actor.initial_speed_mps) || actor.initial_speed_mps < 0.0 ||
        !std::isfinite(actor.accel_limit_mps2) || actor.accel_limit_mps2 <= 0.0 ||
        actor.spawn_time_s < 0.0 ||
        (actor.disappear_time_s >= 0.0 &&
         actor.disappear_time_s <= actor.spawn_time_s) ||
        actor.speed_profile.empty()) {
      throw std::invalid_argument("invalid scripted actor scalar fields");
    }
    double previous_time = -1.0;
    for (const auto& point : actor.speed_profile) {
      if (!std::isfinite(point.first) || !std::isfinite(point.second) ||
          point.first <= previous_time || point.second < 0.0) {
        throw std::invalid_argument("invalid scripted actor speed profile");
      }
      previous_time = point.first;
    }
    if (actor.speed_profile.front().first != 0.0 ||
        actor.speed_profile.front().second != actor.initial_speed_mps) {
      throw std::invalid_argument("speed profile must start at initial speed at t=0");
    }
    ActorRuntime runtime;
    runtime.config = actor;
    runtime.station_m = actor.initial_station_m;
    runtime.lateral_m = actor.initial_lateral_m;
    runtime.speed_mps = actor.initial_speed_mps;
    next.push_back(std::move(runtime));
  }
  std::sort(next.begin(), next.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.config.id < rhs.config.id;
  });
  actors_ = std::move(next);
}

void SimVehicleCore::step(const common::ActuationData& actuation, double dt) {
  if (dt <= 0.0) {
    return;
  }
  sim_time_ += dt;

  // ── 通用 actor 推进：固定车道车辆/骑行者沿 station，行人沿 lateral ──
  const double ego_station = ego_station_m();
  for (auto& actor : actors_) {
    const auto& config = actor.config;
    if (actor.done || sim_time_ < config.spawn_time_s) continue;
    if (config.disappear_time_s >= 0.0 && sim_time_ >= config.disappear_time_s) {
      actor.done = true;
      continue;
    }
    if (config.classification == ScriptedActorClass::Pedestrian) {
      if (!actor.crossing_started &&
          (config.trigger_ego_gap_m < 0.0 ||
           config.initial_station_m - ego_station <= config.trigger_ego_gap_m)) {
        actor.crossing_started = true;
      }
      if (actor.crossing_started) {
        const double direction = config.crossing_end_lateral_m >=
                                         config.initial_lateral_m
                                     ? 1.0
                                     : -1.0;
        actor.lateral_m += direction * config.crossing_speed_mps * dt;
        if ((direction > 0.0 && actor.lateral_m >= config.crossing_end_lateral_m) ||
            (direction < 0.0 && actor.lateral_m <= config.crossing_end_lateral_m)) {
          actor.done = true;
        }
      }
      continue;
    }

    double target_speed = config.initial_speed_mps;
    const double actor_time = sim_time_ - config.spawn_time_s;
    for (const auto& point : config.speed_profile) {
      if (actor_time >= point.first) target_speed = point.second;
    }
    if (config.hard_brake_start_s >= 0.0 &&
        actor_time >= config.hard_brake_start_s &&
        actor_time < config.hard_brake_end_s) {
      target_speed = 0.0;
    }
    const double limit = config.accel_limit_mps2 * dt;
    actor.speed_mps = std::max(
        0.0, actor.speed_mps + std::clamp(target_speed - actor.speed_mps,
                                           -limit, limit));
    actor.station_m += actor.speed_mps * dt;
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

double SimVehicleCore::ego_station_m() const {
  if (centerline_.size() < 2) return 0.0;
  const std::size_t index = ac::find_nearest_index(centerline_, x_, y_);
  return track_length_m_ * static_cast<double>(index) /
         static_cast<double>(centerline_.size() - 1);
}

common::TrajPoint SimVehicleCore::point_at_station(double station_m) const {
  if (centerline_.empty()) return {};
  if (station_m >= 0.0) {
    return ac::point_at_arclength(centerline_, 0,
                                  std::min(station_m, track_length_m_));
  }
  auto point = centerline_.front();
  point.x += std::cos(point.yaw) * station_m;
  point.y += std::sin(point.yaw) * station_m;
  return point;
}

std::vector<ScriptedObjectState> SimVehicleCore::snapshot_objects() const {
  std::vector<ScriptedObjectState> result;
  result.reserve(actors_.size());
  if (centerline_.size() < 2) return result;
  for (const auto& actor : actors_) {
    const auto& config = actor.config;
    if (actor.done || sim_time_ < config.spawn_time_s ||
        (config.disappear_time_s >= 0.0 && sim_time_ >= config.disappear_time_s) ||
        actor.station_m > track_length_m_) {
      continue;
    }
    const auto point = point_at_station(actor.station_m);
    ScriptedObjectState state;
    state.id = config.id;
    state.classification = config.classification;
    state.x = point.x - std::sin(point.yaw) * actor.lateral_m;
    state.y = point.y + std::cos(point.yaw) * actor.lateral_m;
    state.yaw = point.yaw;
    state.v_mps = actor.speed_mps;
    state.station_m = actor.station_m;
    state.lateral_m = actor.lateral_m;
    if (config.classification == ScriptedActorClass::Pedestrian) {
      const double direction = config.crossing_end_lateral_m >=
                                       config.initial_lateral_m
                                   ? 1.0
                                   : -1.0;
      state.yaw = ac::normalize_angle(point.yaw + direction * ac::kPi / 2.0);
      state.v_mps = actor.crossing_started ? config.crossing_speed_mps : 0.0;
    }
    result.push_back(state);
  }
  return result;
}

LeadState SimVehicleCore::lead_state() const {
  LeadState state;
  const auto objects = snapshot_objects();
  const auto found = std::find_if(objects.begin(), objects.end(),
                                  [](const auto& object) { return object.id == 1; });
  if (found == objects.end()) return state;
  state.present = true;
  state.x = found->x;
  state.y = found->y;
  state.yaw = found->yaw;
  state.v_mps = found->v_mps;
  state.station_m = found->station_m;
  return state;
}

LeadState SimVehicleCore::adjacent_car_state() const {
  LeadState state;
  const auto objects = snapshot_objects();
  const auto found = std::find_if(objects.begin(), objects.end(),
                                  [](const auto& object) { return object.id == 3; });
  if (found == objects.end()) return state;
  state.present = true;
  state.x = found->x;
  state.y = found->y;
  state.yaw = found->yaw;
  state.v_mps = found->v_mps;
  state.station_m = found->station_m;
  return state;
}

PedestrianState SimVehicleCore::pedestrian_state() const {
  PedestrianState state;
  const auto objects = snapshot_objects();
  const auto found = std::find_if(objects.begin(), objects.end(),
                                  [](const auto& object) { return object.id == 2; });
  if (found == objects.end()) return state;
  state.present = true;
  state.x = found->x;
  state.y = found->y;
  state.yaw = found->yaw;
  state.v_mps = found->v_mps;
  state.lateral_m = found->lateral_m;
  return state;
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
