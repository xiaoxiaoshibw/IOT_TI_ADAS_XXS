// adas_trajectory_planner/trajectory_planner_core.hpp
// 局部轨迹生成核心（无 ROS 依赖）：
//   由 lane_state（横向偏移/航向误差/曲率）重建局部参考线（常曲率弧），
//   叠加速度剖面（巡航限速 / 曲率限速 / 加减速度限制），输出 odom 系轨迹。
// M1 仅车道保持；M2 起 IDM 跟车剖面、M4 起变道横移在此扩展。
#ifndef ADAS_TRAJECTORY_PLANNER__TRAJECTORY_PLANNER_CORE_HPP_
#define ADAS_TRAJECTORY_PLANNER__TRAJECTORY_PLANNER_CORE_HPP_

#include <vector>

#include "adas_common/types.hpp"

namespace adas::planning {

inline int guarded_target_lane(int requested, bool left_available,
                               bool right_available) {
  if (requested < 0 && !left_available) return 0;
  if (requested > 0 && !right_available) return 0;
  return requested;
}

struct PlannerParams {
  double horizon_s{8.0};           // 时间视界
  double min_length_m{30.0};       // 轨迹最短长度
  double max_length_m{120.0};      // 轨迹最长长度（点数上限的另一面）
  double step_m{1.0};              // 采样步长
  double cruise_speed_mps{15.0};   // 巡航速度（M2 起由 behavior 覆盖）
  double max_lat_accel_mps2{2.5};  // 弯道限速：v = sqrt(a_lat_max / |k|)
  double max_accel_mps2{1.5};      // 剖面纵向加速上限
  double max_decel_mps2{2.0};      // 剖面纵向减速上限（舒适值，非 AEB）
  // Commit 6a — 段间速度变化率的"舒适"上限。比 max_accel/mps2 更保守，
  // 是对前馈加速度的"软上限"，避免轨迹给出 0.5 m/s² → 3 m/s² 的阶跃式跳变。
  double accel_rate_limit{1.5};    // 单段最大 |dv|/(ds/v) [m/s²]
  // 跟车（ACC）逼近曲线参数：稳态时距 ≈ time_gap + standstill/v
  double follow_time_gap_s{1.1};   // 恒定时距
  double follow_standstill_m{4.0}; // 静止安全距离（停在前车后方此距离）
  // Commit 2 — 全局路线专属跟车参数：与 lane-state plan() 的 follow_*
  // 解耦——路线规划跟随前车更早减速，停车时距更保守，避免在目的地前才察觉。
  double global_route_follow_time_gap_s{1.4};
  double global_route_follow_standstill_m{4.0};
  // Commit 3 — 弯道曲率前视包络（米）。curve_cap 取前方 [8, 15] m 内最大 |k|，
  // 避免单点曲率异常把整段速度压到零；该参数是 [8, 15] 区间内的具体值，
  // 12 是中央默认值（用户验收后再评估 10 或 14）。
  double global_route_curvature_envelope_m{12.0};
  // 变道（M4）：五次多项式横移过渡
  double lane_change_time_s{3.0};  // 过渡时长（长度 = v × 时长，有下限）
  double lane_change_min_len_m{25.0};
  double lane_width_m{3.5};
  bool lateral_avoidance_enabled{false};
};

// 主前车信息（由 object_tracker 选举结果投影）
struct LeadInfo {
  bool present{false};
  double gap_m{0.0};       // 沿路径纵向间距
  double speed_mps{0.0};   // 滤波后前车速度（≥0）
};

struct StaticObstacle {
  double longitudinal_m{0.0};
  double lateral_m{0.0};
  bool is_static{true};
};

class TrajectoryPlannerCore {
 public:
  explicit TrajectoryPlannerCore(const PlannerParams& params);

  // lane.valid 为 false 时返回空轨迹（下游按超时降级）。
  // lead.present 时速度剖面叠加跟车逼近曲线：
  //   v_cap(s) = sqrt(max(0, v_lead² + 2·b·(gap − d0 − T·v_lead − s)))
  //   稳态收敛于 gap = d0 + T·v_lead（前车静止时停在其后 d0 处）。
  // cruise_override_mps ≥ 0 时覆盖参数巡航速度（behavior 下发；0 = 停车剖面）。
  // target_lane：0=本车道；-1=左邻道。变道过渡为**固定锚点**五次多项式：
  //   目标车道变化时记录起点（位置+横向），横移量按「起点以来已行驶距离+s」
  //   推进曲线——若每拍从自车当前横向重新锚定，滚动重规划会让横移渐近减速、
  //   永远到不了目标车道（实测卡在 83%），此为 M4 调参实录。
  common::Trajectory plan(const common::KinematicState& ego,
                          const common::LaneStateData& lane,
                          const LeadInfo& lead = LeadInfo(),
                          double cruise_override_mps = -1.0,
                          int target_lane = 0);

  // Commit 2 — 为全局路线生成可跟踪的速度剖面。全局路线来自地图中心线，
  // 不能只按巡航速度前进：必须同时满足弯道横向加速度、终点停车距离和
  // 可用减速度约束。当 lead.present 时叠加第 4 个限速（lead cap）：仅
  // ego-side 包络（不涉及 AEB，AEB 由独立通道处理）；跟车距离增量随前车
  // 共移：follow_dist = lead.gap + v_lead·t_est − standstill − T·v_lead − s，
  // 保证稳态收敛到 gap = standstill + T·v_lead（前车静止时停在其后 standstill）。
  common::Trajectory plan_global_route(const common::KinematicState& ego,
                                       const common::Trajectory& route,
                                       double cruise_speed_mps,
                                       double goal_stop_distance_m,
                                       bool stop_at_route_end = true,
                                       const LeadInfo& lead = LeadInfo(),
                                       const std::vector<StaticObstacle>& obstacles = {}) const;

  common::Trajectory avoid_obstacles_laterally(
      const common::Trajectory& reference,
      const std::vector<StaticObstacle>& obstacles) const;

 private:
  PlannerParams params_;
  // 变道过渡状态（跨拍）
  bool lc_active_{false};
  double lc_from_lat_{0.0};
  double lc_target_lat_{0.0};
  double lc_start_x_{0.0};
  double lc_start_y_{0.0};
  double lc_len_{0.0};
  double last_target_lat_{0.0};
};

}  // namespace adas::planning

#endif  // ADAS_TRAJECTORY_PLANNER__TRAJECTORY_PLANNER_CORE_HPP_
