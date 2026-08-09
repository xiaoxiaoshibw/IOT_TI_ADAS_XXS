// 目标跟踪核心实现
#include "adas_object_tracker/object_tracker_core.hpp"

#include <algorithm>
#include <cmath>
#include <set>

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
    // Commit 5 — 车道中心线在前方 lon 处相对切线的横向偏移用圆弧几何替代
    // 0.5*k*lon²。圆弧公式 R*(1 - cos(lon/R)) 在 lon=R*π 时取最大值 2R，
    // 本身就比二阶展开稳定；不再额外夹紧 lon（夹紧会破坏真实中心线几何）。
    const double abs_k = std::fabs(lane.curvature);
    double lane_lat_at_lon = 0.0;
    if (abs_k > 1e-6) {
      const double R = 1.0 / abs_k;
      // dlat 符号 = lane.curvature 符号（左正右负对应左/右弯）
      lane_lat_at_lon = std::copysign(R * (1.0 - std::cos(lon / R)),
                                      lane.curvature);
    } else {
      lane_lat_at_lon = 0.0;
    }
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

  // 清理过期 track（包括 candidate_frames_ 的对应计数）
  for (auto it = tracks_.begin(); it != tracks_.end();) {
    if (now_s - it->second.last_t > params_.track_stale_s) {
      candidate_frames_.erase(it->first);
      it = tracks_.erase(it);
    } else {
      ++it;
    }
  }

  // Commit 5 — sticky 主前车选举（详见 select_primary_lead 注释）
  const int primary = select_primary_lead(out.objects);

  out.primary_lead_id = primary;
  if (primary >= 0 && lost_frames_ == 0) {
    // 正常路径：主车在本帧可见且合格，从 out.objects 提取实时 gap/speed
    for (const auto& t : out.objects) {
      if (static_cast<int>(t.raw.id) == primary) {
        out.primary_lead_gap_m = t.gap_m;
        out.primary_lead_speed_mps = std::max(0.0, t.v_filtered_mps);
        // 记住当前主车的 gap/speed，为短暂丢失时复用
        last_primary_gap_m_ = t.gap_m;
        last_primary_speed_mps_ = std::max(0.0, t.v_filtered_mps);
        break;
      }
    }
  } else if (primary >= 0 && lost_frames_ > 0) {
    // 短暂保留期：复用最后已知 gap/speed
    out.primary_lead_gap_m = last_primary_gap_m_;
    out.primary_lead_speed_mps = last_primary_speed_mps_;
  } else {
    out.primary_lead_gap_m = 0.0;
    out.primary_lead_speed_mps = 0.0;
  }
  const bool swapped = (primary != last_primary_);
  out.lead_swapped = swapped;
  last_primary_ = primary;
  return out;
}

bool ObjectTrackerCore::qualify_for_lead(const TrackedObjectData& t) const {
  return t.ahead && t.in_ego_lane &&
         t.gap_m > 0.0 &&
         t.gap_m < params_.max_lead_range_m;
}

int ObjectTrackerCore::select_primary_lead(const std::vector<TrackedObjectData>& objects) {
  // 1. 重置候选计数：对当前帧中存在的目标，合格则 +1，不合格则清零。
  std::set<uint32_t> present_ids;
  for (const auto& t : objects) {
    const uint32_t id = t.raw.id;
    present_ids.insert(id);
    if (qualify_for_lead(t)) {
      candidate_frames_[id]++;
    } else {
      candidate_frames_[id] = 0;
    }
  }
  // 2. 清掉本帧不存在的目标计数（避免长时间不出现还占着计数）
  for (auto it = candidate_frames_.begin(); it != candidate_frames_.end();) {
    if (present_ids.find(it->first) == present_ids.end()) {
      it = candidate_frames_.erase(it);
    } else {
      ++it;
    }
  }

  // 3. 第一次选择（last_primary_ == -1）：直接选最近的合格候选，不走 sticky。
  if (last_primary_ == -1) {
    int best = -1;
    double best_gap = params_.max_lead_range_m;
    for (const auto& t : objects) {
      if (!qualify_for_lead(t)) continue;
      if (t.gap_m < best_gap) {
        best_gap = t.gap_m;
        best = static_cast<int>(t.raw.id);
      }
    }
    return best;
  }

  // 4. 当前主车存在 + 合格 → 沿用，并重置 lost 计数。
  for (const auto& t : objects) {
    if (static_cast<int>(t.raw.id) == last_primary_ && qualify_for_lead(t)) {
      lost_frames_ = 0;
      int primary = last_primary_;
      // 候选抢占：另一目标 frames ≥ confirm_frames 且 gap 小 ≥ gain_m
      for (const auto& t2 : objects) {
        const uint32_t id = t2.raw.id;
        if (static_cast<int>(id) == primary) continue;
        const int cnt = candidate_frames_[id];
        if (cnt < params_.confirm_frames) continue;
        if (t2.gap_m + params_.gain_m > last_primary_gap_m_) continue;
        primary = static_cast<int>(id);
        // 把原主车计数清零，避免反复横跳
        candidate_frames_[static_cast<uint32_t>(last_primary_)] = 0;
        break;
      }
      return primary;
    }
  }

  // 5. 主车在本帧不可见：短暂保留期 → 复用其 gap/speed。
  if (lost_frames_ < params_.retain_frames_on_lost) {
    lost_frames_++;
    return last_primary_;
  }
  // 6. 真正清空：把残留状态归零，下一帧从 step 3 的"首次选择"重新开始。
  lost_frames_ = 0;
  last_primary_gap_m_ = 0.0;
  last_primary_speed_mps_ = 0.0;
  for (auto& kv : candidate_frames_) kv.second = 0;
  return -1;
}

}  // namespace adas::perception
