// adas_sim_vehicle/sim_vehicle_core.hpp
// SIL 仿真车辆核心：运动学自行车模型 + 分段圆弧赛道真值（无 ROS 依赖）。
// 职责：接收归一化执行量 → 推进车辆状态；对外提供 odometry/转角/车道相对状态真值。
#ifndef ADAS_SIM_VEHICLE__SIM_VEHICLE_CORE_HPP_
#define ADAS_SIM_VEHICLE__SIM_VEHICLE_CORE_HPP_

#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "adas_common/types.hpp"

namespace adas::sim {

struct VehicleParams {
  double wheelbase_m{2.7};
  double max_steer_rad{0.6};
  double max_accel_mps2{3.0};
  double max_decel_mps2{8.0};
  double steer_tau_s{0.2};       // 转向执行一阶惯性时间常数
  double drag_accel_mps2{0.1};   // 简化滚阻/风阻（恒定减速项）
};

// 赛道段：长度 + 常曲率（0=直线，左转正）
struct TrackSegment {
  double length_m{0.0};
  double curvature{0.0};
};

// 脚本化前车：沿中心线行驶，按事件表变速（M2 ACC 场景注入）
struct LeadScript {
  bool enabled{false};
  double initial_station_m{60.0};
  double initial_speed_mps{10.0};
  double accel_mps2{2.0};  // 逼近事件目标速度的加减速率（对称）
  std::vector<std::pair<double, double>> events;  // (触发时间 s, 目标速度 m/s)，按时间升序
};

struct LeadState {
  bool present{false};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v_mps{0.0};
  double station_m{0.0};
};

// 脚本化邻道车：spawn 时刻出现在邻道指定弧长处，恒速沿车道行驶（M4 放弃超车场景）
struct AdjacentCarScript {
  bool enabled{false};
  double spawn_time_s{0.0};
  double initial_station_m{0.0};
  double lateral_m{3.5};          // 左邻道中心
  double speed_mps{10.0};
};

// 脚本化横穿行人：自车逼近到触发距离时，从车道一侧沿法向横穿到另一侧（M3 AEB 场景）
struct PedestrianScript {
  bool enabled{false};
  double station_m{150.0};        // 横穿点（中心线弧长）
  double trigger_ego_gap_m{35.0}; // 自车距横穿点小于此值时开始横穿
  double start_lateral_m{-5.0};   // 起始横向（右侧为负）
  double end_lateral_m{5.0};      // 走到此横向后消失
  double speed_mps{1.5};
};

struct PedestrianState {
  bool present{false};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};     // 行走方向（世界系）
  double v_mps{0.0};
  double lateral_m{0.0};
};

// 与 TrackedObject 分类数值保持一致，但 core 本身不依赖 ROS 消息。
enum class ScriptedActorClass : std::uint8_t {
  Unknown = 0,
  Car = 1,
  Truck = 2,
  Pedestrian = 3,
  Bicycle = 4,
};

// 通用固定车道 actor。spawn/disappear 主要供确定性单测与后续 schema 使用；
// schema v1 文件默认从 t=0 出现并一直存在到驶出赛道。
struct ScriptedActor {
  std::uint32_t id{0};
  ScriptedActorClass classification{ScriptedActorClass::Unknown};
  double initial_station_m{0.0};
  double initial_lateral_m{0.0};
  double initial_speed_mps{0.0};
  double accel_limit_mps2{3.0};
  std::vector<std::pair<double, double>> speed_profile;
  double spawn_time_s{0.0};
  double disappear_time_s{-1.0};
  double hard_brake_start_s{-1.0};
  double hard_brake_end_s{-1.0};
  double trigger_ego_gap_m{-1.0};
  double crossing_end_lateral_m{0.0};
  double crossing_speed_mps{0.0};
};

struct ScriptedObjectState {
  std::uint32_t id{0};
  ScriptedActorClass classification{ScriptedActorClass::Unknown};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v_mps{0.0};
  double station_m{0.0};
  double lateral_m{0.0};
};

class SimVehicleCore {
 public:
  static constexpr std::size_t kMaxScriptedActors = 64;

  SimVehicleCore(const VehicleParams& params, const std::vector<TrackSegment>& segments,
                 double lane_width_m, double sample_step_m = 0.5);

  // 以中心线上 station 处、横向偏移 lateral 放置车辆（航向对齐路径）
  void set_initial_state(double station_m, double lateral_m, double speed_mps);

  // 注入脚本化前车（可选）
  void set_lead_script(const LeadScript& script);
  // 注入脚本化横穿行人（可选）
  void set_pedestrian_script(const PedestrianScript& script);
  // 注入脚本化邻道车（可选）
  void set_adjacent_car_script(const AdjacentCarScript& script);

  // 原子替换整个 actor 集合；非法/超限输入抛 invalid_argument，不保留半套场景。
  void set_scripted_actors(const std::vector<ScriptedActor>& actors);
  // 只返回当前可见目标，始终按稳定 ID 升序。
  std::vector<ScriptedObjectState> snapshot_objects() const;

  // 推进一个物理步（自车 + 前车）
  void step(const common::ActuationData& actuation, double dt);

  common::KinematicState state() const;
  double steering_angle_rad() const { return steer_; }
  // 车道相对状态真值；驶出赛道末端附近时 valid=false
  common::LaneStateData lane_state() const;
  // 前车状态（未启用或驶出赛道 → present=false）
  LeadState lead_state() const;
  // 行人状态（未启用/未触发/已走完 → present=false）
  PedestrianState pedestrian_state() const;
  // 邻道车状态（未启用/未到 spawn 时刻/驶出赛道 → present=false）
  LeadState adjacent_car_state() const;
  const common::Trajectory& centerline() const { return centerline_; }

 private:
  VehicleParams params_;
  common::Trajectory centerline_;
  double lane_width_;

  // 车辆状态（odom 系）
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double v_{0.0};
  double steer_{0.0};
  double yaw_rate_{0.0};

  double sim_time_{0.0};
  double track_length_m_{0.0};

  struct ActorRuntime {
    ScriptedActor config;
    double station_m{0.0};
    double lateral_m{0.0};
    double speed_mps{0.0};
    bool crossing_started{false};
    bool done{false};
  };
  std::vector<ActorRuntime> actors_;

  void upsert_legacy_actor(const ScriptedActor& actor, bool enabled);
  double ego_station_m() const;
  common::TrajPoint point_at_station(double station_m) const;
};

}  // namespace adas::sim

#endif  // ADAS_SIM_VEHICLE__SIM_VEHICLE_CORE_HPP_
