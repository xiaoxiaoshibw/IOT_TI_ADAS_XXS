// adas_behavior_planner/behavior_core.hpp
// 行为状态机（无 ROS 依赖）。状态数值与 adas_msgs/BehaviorState 常量一致。
// M2：LANE_FOLLOW / FOLLOW_LEAD / STOPPING
// M4：OVERTAKE_WAIT / OVERTAKE_ACTIVE / OVERTAKE_RETURN
//   与旧栈最大差异：超车只改「目标车道」，变道轨迹由轨迹层生成——
//   行为层不再直接操纵控制器目标偏移（旧栈 target_lane_offset 补丁的架构化替代）。
#ifndef ADAS_BEHAVIOR_PLANNER__BEHAVIOR_CORE_HPP_
#define ADAS_BEHAVIOR_PLANNER__BEHAVIOR_CORE_HPP_

#include <cstdint>
#include <vector>

namespace adas::planning {

enum class BehaviorKind : int {
  kLaneFollow = 0,
  kFollowLead = 1,
  kOvertakeWait = 2,
  kOvertakeActive = 3,
  kOvertakeReturn = 4,
  kStopping = 5,
  kApproachingStop = 7,
  kStoppingAtStop = 8,
  kWaitingAtLight = 9,
  kEnteringJunction = 10,
};

struct BehaviorParams {
  double cruise_speed_mps{15.0};
  double follow_enter_range_m{80.0};
  double follow_exit_range_m{95.0};
  int exit_hysteresis_frames{5};
  // ── 超车（M4）──
  bool overtake_enabled{true};
  double overtake_slow_ratio{0.6};      // 前车速 < 巡航×比值 → 视为慢车
  double overtake_trigger_gap_m{30.0};  // 已逼近到该距离才考虑超车
  int overtake_wait_frames{10};         // 慢车持续 N 帧才进入 WAIT
  int clear_confirm_frames{5};          // 邻道连续 M 帧清空才变道
  double clear_front_m{60.0};           // 邻道清空校验：前向窗口
  double clear_rear_m{15.0};            // 邻道清空校验：后向窗口
  double pass_margin_m{8.0};            // 被超车辆落后自车此距离 → 回线
  double lane_attain_tol_m{0.6};        // 须先横移到目标车道中心此范围内才算变道到位
  double return_done_lat_m{0.5};        // 回到本车道中心此范围内 → 完成
  double abort_lat_limit_m{1.4};        // 仍在本车道内（|lat|<此值）才允许中止
  int target_lane{-1};                  // 超车用车道：-1=左邻道
  double lane_width_m{3.5};
  double stop_sign_approach_m{15.0};
  double stop_sign_stop_duration_s{2.0};
  double traffic_light_approach_m{30.0};
  double junction_approach_m{30.0};
};

enum class MapSignType : uint8_t {
  kStopSign = 0,
  kTrafficLight = 1,
  kJunction = 2,
};

struct MapSignLite {
  MapSignType type{MapSignType::kJunction};
  double distance_m{0.0};
  bool traffic_light_red{false};
  int64_t lane_id{0};
};

// 路径坐标系下的轻量目标（由 TrackedObjectArray 投影字段构建）
struct ObjectLite {
  uint32_t id{0};
  double lon_m{0.0};   // 自车前方为正
  double lat_m{0.0};   // 相对本车道中心线，左正
  double v_mps{0.0};
};

struct BehaviorInput {
  std::vector<ObjectLite> objects;
  int primary_lead_id{-1};
  double lead_gap_m{0.0};
  double lead_speed_mps{0.0};
  double ego_speed_mps{0.0};
  double ego_lateral_m{0.0};   // 自车相对本车道中心线横向
  bool target_lane_available{false};  // 目标邻道存在且当前允许变道
  double now_s{0.0};
  std::vector<MapSignLite> map_signs;
  bool mrm_stop{false};
};

struct BehaviorOutput {
  BehaviorKind state{BehaviorKind::kLaneFollow};
  double target_speed_mps{0.0};
  int target_lane{0};          // 0=本车道；-1=左邻道
};

class BehaviorCore {
 public:
  explicit BehaviorCore(const BehaviorParams& params);
  BehaviorOutput update(const BehaviorInput& in);
  BehaviorKind state() const { return state_; }

 private:
  bool adjacent_lane_clear(const BehaviorInput& in) const;

  BehaviorParams params_;
  BehaviorKind state_{BehaviorKind::kLaneFollow};
  int exit_count_{0};
  int slow_count_{0};
  int clear_count_{0};
  int lost_count_{0};
  uint32_t overtaking_id_{0};
  double stop_start_s_{0.0};
};

}  // namespace adas::planning

#endif  // ADAS_BEHAVIOR_PLANNER__BEHAVIOR_CORE_HPP_
