// adas_object_tracker/object_tracker_core.hpp
// 多目标跟踪 + 主前车选举（对标 openpilot radard 的角色，无 ROS 依赖）。
// 职责：
//   1. 对每个原始目标做速度低通 + 加速度差分估计（跨帧状态按 id 维护）
//   2. 车道归属判定：目标投影到自车参考路径坐标系（含曲率二阶修正）
//   3. 主前车选举：自车道内最近前方目标；选举切换时置 lead_swapped 标志
//      （继承旧栈教训：lead swap 必须显式让下游知道，防滤波状态串车）
#ifndef ADAS_OBJECT_TRACKER__OBJECT_TRACKER_CORE_HPP_
#define ADAS_OBJECT_TRACKER__OBJECT_TRACKER_CORE_HPP_

#include <cstdint>
#include <map>
#include <vector>

#include "adas_common/low_pass_filter.hpp"
#include "adas_common/types.hpp"

namespace adas::perception {

struct RawObject {
  uint32_t id{0};
  uint8_t classification{0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v_mps{0.0};  // 目标自身纵向速度
};

struct TrackedObjectData {
  RawObject raw;
  double v_filtered_mps{0.0};
  double a_est_mps2{0.0};
  double gap_m{0.0};     // 沿路径纵向间距（前方为正）
  double lat_m{0.0};     // 相对车道中心线横向位置（左正）
  bool in_ego_lane{false};
  bool ahead{false};
};

struct TrackerParams {
  double lane_margin_factor{0.9};   // in-lane 阈值 = lane_width/2 * factor
  double max_lead_range_m{120.0};   // 超出此距离不参与选举
  double v_filter_tau_s{0.3};
  double a_filter_tau_s{0.5};
  double track_stale_s{0.5};        // 超时未更新的 track 清理
};

struct TrackerOutput {
  std::vector<TrackedObjectData> objects;
  int primary_lead_id{-1};
  double primary_lead_gap_m{0.0};
  double primary_lead_speed_mps{0.0};
  bool lead_swapped{false};  // 本帧主前车 id 发生变化（含出现/消失）
};

class ObjectTrackerCore {
 public:
  explicit ObjectTrackerCore(const TrackerParams& params);

  TrackerOutput update(double now_s, const std::vector<RawObject>& raw,
                       const common::KinematicState& ego, const common::LaneStateData& lane);

 private:
  struct Track {
    common::LowPassFilter v_filter;
    common::LowPassFilter a_filter;
    double last_v{0.0};
    double last_t{0.0};
    bool has_prev{false};
    Track(double v_tau, double a_tau) : v_filter(v_tau), a_filter(a_tau) {}
  };

  TrackerParams params_;
  std::map<uint32_t, Track> tracks_;
  int last_primary_{-1};
};

}  // namespace adas::perception

#endif  // ADAS_OBJECT_TRACKER__OBJECT_TRACKER_CORE_HPP_
