// adas_common/types.hpp
// 全栈算法核心层（*_core）共享的纯数据结构。
// 约束：本文件（及 adas_common 全部头文件）禁止 include 任何 rclcpp/rcl/rosidl 头。
// 消息 ↔ 结构体的转换只发生在各包的节点壳（*_node.cpp）里。
#ifndef ADAS_COMMON__TYPES_HPP_
#define ADAS_COMMON__TYPES_HPP_

#include <vector>

namespace adas::common {

// 平面位姿（odom 系），yaw 弧度、逆时针为正
struct Pose2d {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// 轨迹点：位姿 + 速度剖面（对应 adas_msgs/TrajectoryPoint）
struct TrajPoint {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double velocity_mps{0.0};
  double acceleration_mps2{0.0};
  double curvature{0.0};
  double time_from_start_s{0.0};
};

using Trajectory = std::vector<TrajPoint>;

// 自车运动状态（对应 nav_msgs/Odometry 的关注子集）
struct KinematicState {
  Pose2d pose;
  double velocity_mps{0.0};  // 车体纵向速度
  double yaw_rate_rps{0.0};
};

// 控制命令（对应 adas_msgs/Control）
struct LateralCommandData {
  double steering_tire_angle_rad{0.0};
  double rotation_rate_rad_s{0.0};
};

struct LongitudinalCommandData {
  double velocity_mps{0.0};
  double acceleration_mps2{0.0};  // 负 = 制动
};

struct ControlData {
  LateralCommandData lateral;
  LongitudinalCommandData longitudinal;
};

// 归一化执行量（对应 adas_msgs/ActuationCommand）
struct ActuationData {
  double throttle{0.0};  // 0..1
  double brake{0.0};     // 0..1
  double steer{0.0};     // -1..1，左正
};

// 车道相对状态（对应 adas_msgs/LaneState）
struct LaneStateData {
  bool valid{false};
  double lateral_offset{0.0};   // 左正 [m]
  double heading_error{0.0};    // 左正 [rad]
  double curvature{0.0};        // 左转正 [1/m]
  double lane_width{3.5};       // [m]
};

}  // namespace adas::common

#endif  // ADAS_COMMON__TYPES_HPP_
