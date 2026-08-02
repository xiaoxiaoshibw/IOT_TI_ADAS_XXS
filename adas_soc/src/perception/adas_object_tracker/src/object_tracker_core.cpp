// 目标跟踪核心实现
#include "adas_object_tracker/object_tracker_core.hpp"

#include <algorithm>
#include <cmath>

#include "adas_common/geometry.hpp"

namespace adas::perception {

namespace ac = adas::common;

ObjectTrackerCore::ObjectTrackerCore(const TrackerParams& params) : params_(params) {}

TrackerOutput ObjectTrackerCore::update(double now_s, const std::vector<RawObject>& raw,
                                        const common::KinematicState& ego,
                                        const common::LaneStateData& lane) {
  TrackerOutput out;

  // 路径切向坐标系（自车参考线投影处）
  const double path_yaw = ac::normalize_angle(ego.pose.yaw - lane.heading_error);
  const double cos_p = std::cos(path_yaw);
  const double sin_p = std::sin(path_yaw);

  for (const auto& obj : raw) {
    // 跨帧 track（id 关联；沿用发布方 id，感知真值/桥保证稳定）
    auto it = tracks_.find(obj.id);
    if (it == tracks_.end()) {
      it = tracks_
               .emplace(obj.id, Track(params_.v_filter_tau_s, params_.a_filter_tau_s))
               .first;
    }
    Track& trk = it->second;
    const double dt = trk.has_prev ? (now_s - trk.last_t) : 0.0;
    const double v_f = trk.v_filter.update(obj.v_mps, dt > 0.0 ? dt : 1e-2);
    double a_est = 0.0;
    if (trk.has_prev && dt > 1e-4) {
      a_est = trk.a_filter.update((obj.v_mps - trk.last_v) / dt, dt);
    } else {
      a_est = trk.a_filter.update(0.0, 1e-2);
    }
    trk.last_v = obj.v_mps;
    trk.last_t = now_s;
    trk.has_prev = true;

    // 目标在路径坐标系中的位置
    const double dx = obj.x - ego.pose.x;
    const double dy = obj.y - ego.pose.y;
    const double lon = cos_p * dx + sin_p * dy;          // 前方为正
    const double lat_rel = -sin_p * dx + cos_p * dy;     // 左为正
    // 车道中心线在前方 lon 处相对切线的横向偏移 ≈ 0.5*k*lon²（二阶修正）
    const double lane_lat_at_lon = 0.5 * lane.curvature * lon * lon;
    const double lat_from_center = (lane.lateral_offset + lat_rel) - lane_lat_at_lon;

    // 自车当前车道带中心：超车变道后选举跟随自车所在车道（±1 邻道范围）
    double ego_lane_center = 0.0;
    if (lane.lane_width > 1e-3) {
      const double idx =
          std::round(lane.lateral_offset / lane.lane_width);
      ego_lane_center = std::clamp(idx, -1.0, 1.0) * lane.lane_width;
    }

    TrackedObjectData t;
    t.raw = obj;
    t.v_filtered_mps = v_f;
    t.a_est_mps2 = a_est;
    t.gap_m = lon;
    t.lat_m = lat_from_center;
    t.ahead = lon > 0.0;
    t.in_ego_lane = lane.valid &&
                    std::fabs(lat_from_center - ego_lane_center) <
                        (lane.lane_width / 2.0) * params_.lane_margin_factor;
    out.objects.push_back(t);
  }

  // 清理过期 track
  for (auto it = tracks_.begin(); it != tracks_.end();) {
    if (now_s - it->second.last_t > params_.track_stale_s) {
      it = tracks_.erase(it);
    } else {
      ++it;
    }
  }

  // 主前车选举：自车道内、前方、范围内、最近
  int primary = -1;
  double best_gap = params_.max_lead_range_m;
  for (const auto& t : out.objects) {
    if (t.ahead && t.in_ego_lane && t.gap_m < best_gap) {
      best_gap = t.gap_m;
      primary = static_cast<int>(t.raw.id);
    }
  }
  out.primary_lead_id = primary;
  if (primary >= 0) {
    for (const auto& t : out.objects) {
      if (static_cast<int>(t.raw.id) == primary) {
        out.primary_lead_gap_m = t.gap_m;
        out.primary_lead_speed_mps = std::max(0.0, t.v_filtered_mps);
        break;
      }
    }
  }
  out.lead_swapped = (primary != last_primary_);
  last_primary_ = primary;
  return out;
}

}  // namespace adas::perception
