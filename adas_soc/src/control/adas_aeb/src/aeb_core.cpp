// AEB 核心实现
#include "adas_aeb/aeb_core.hpp"

#include <algorithm>
#include <cmath>

#include "adas_common/geometry.hpp"

namespace adas::control {

namespace ac = adas::common;

namespace {
constexpr uint8_t kClassPedestrian = 3;  // adas_msgs/TrackedObject CLASS_PEDESTRIAN
}

AebCore::AebCore(const AebParams& params) : params_(params) {}

AebResult AebCore::update(const common::KinematicState& ego,
                          const std::vector<AebObject>& objects) {
  AebResult res;

  // 低速不激活（起步/静止阶段防误触发；停车由 ACC/gate 负责维持）
  if (ego.velocity_mps < params_.min_active_speed_mps) {
    trigger_count_ = 0;
    clear_count_ = 0;
    latched_ = false;
    res.state = AebState::kInactive;
    return res;
  }

  // ── 冲突扫描：自车(恒速+恒横摆) × 目标(恒速直线) 同步外推 ──
  double best_ttc = 1e9;
  uint8_t best_class = 0;
  const double hit_dist = params_.corridor_half_width_m + params_.obj_radius_m;
  for (const auto& obj : objects) {
    // Transform the object's current position into the ego frame before
    // prediction.  A vehicle well behind the ego must not cause an AEB
    // decision merely because its world-frame trajectory intersects the
    // ego's future path.
    const double dx0 = obj.x - ego.pose.x;
    const double dy0 = obj.y - ego.pose.y;
    const double initial_longitudinal =
        std::cos(ego.pose.yaw) * dx0 + std::sin(ego.pose.yaw) * dy0;
    if (initial_longitudinal < -params_.rear_filter_m || initial_longitudinal > 50.0) {
      continue;
    }

    const double ovx = std::cos(obj.yaw) * obj.v_mps;
    const double ovy = std::sin(obj.yaw) * obj.v_mps;
    double ex = ego.pose.x;
    double ey = ego.pose.y;
    double eyaw = ego.pose.yaw;
    for (double t = 0.0; t <= params_.horizon_s + 1e-9; t += params_.step_s) {
      const double ox = obj.x + ovx * t;
      const double oy = obj.y + ovy * t;
      const double dx = ox - ex;
      const double dy = oy - ey;
      const double longitudinal = std::cos(eyaw) * dx + std::sin(eyaw) * dy;
      const double lateral = -std::sin(eyaw) * dx + std::cos(eyaw) * dy;
      if (longitudinal < -params_.rear_filter_m || longitudinal > 50.0 ||
          std::abs(lateral) > params_.corridor_width_m) {
        // The target may enter the corridor later (e.g. a crossing
        // pedestrian), so this is evaluated for every prediction sample.
        eyaw = ac::normalize_angle(eyaw + ego.yaw_rate_rps * params_.step_s);
        ex += ego.velocity_mps * std::cos(eyaw) * params_.step_s;
        ey += ego.velocity_mps * std::sin(eyaw) * params_.step_s;
        continue;
      }
      if (ac::distance2d(ex, ey, ox, oy) < hit_dist) {
        if (t < best_ttc) {
          best_ttc = t;
          best_class = obj.classification;
        }
        break;  // 该目标最早冲突已找到
      }
      // 自车推进一步
      eyaw = ac::normalize_angle(eyaw + ego.yaw_rate_rps * params_.step_s);
      ex += ego.velocity_mps * std::cos(eyaw) * params_.step_s;
      ey += ego.velocity_mps * std::sin(eyaw) * params_.step_s;
    }
  }

  res.ttc_s = best_ttc;
  const bool has_conflict = best_ttc < 1e8;
  if (has_conflict) {
    const double dist_to_conflict = std::max(ego.velocity_mps * best_ttc, 0.1);
    res.required_decel_mps2 =
        ego.velocity_mps * ego.velocity_mps / (2.0 * dist_to_conflict);
  }

  // class-aware 触发阈值
  const double ttc_thr = best_class == kClassPedestrian ? params_.ttc_emergency_ped_s
                                                        : params_.ttc_emergency_car_s;
  const bool danger = has_conflict && best_ttc < ttc_thr;

  // 连帧确认 / 连帧释放
  if (danger) {
    trigger_count_ = std::min(trigger_count_ + 1, params_.trigger_frames);
    clear_count_ = 0;
  } else {
    clear_count_ = std::min(clear_count_ + 1, params_.release_frames);
    trigger_count_ = 0;
  }
  if (!latched_ && trigger_count_ >= params_.trigger_frames) {
    latched_ = true;
  }
  if (latched_ && clear_count_ >= params_.release_frames) {
    latched_ = false;
  }

  if (latched_) {
    res.state = AebState::kEmergency;
    res.emergency_active = true;
    res.brake_accel_mps2 = -params_.emergency_decel_mps2;
    res.reason = best_class == kClassPedestrian ? "pedestrian_in_path" : "obstacle_in_path";
  } else if (danger) {
    res.state = AebState::kWarning;  // 确认窗内
    res.reason = "confirming";
  } else if (has_conflict) {
    res.state = AebState::kWarning;
    res.reason = "conflict_beyond_threshold";
  } else {
    res.state = AebState::kMonitoring;
  }
  return res;
}

}  // namespace adas::control
