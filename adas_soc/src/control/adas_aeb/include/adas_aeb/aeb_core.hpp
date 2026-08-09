// adas_aeb/aeb_core.hpp
// 独立 AEB 核心（对标 autoware_autonomous_emergency_braking，无 ROS 依赖）。
// 与主控制链完全并行：只依赖自车运动状态 + 目标列表，不依赖规划轨迹存活。
//
// 判据链：
//   1. 自车路径外推：恒速 + 恒横摆角速度（Autoware IMU-path 同款）
//   2. 目标外推：恒速直线（世界系，由航向+速度重建）
//   3. 逐时刻求距离 → 最早冲突时刻 = TTC
//   4. class-aware 阈值：行人用更长 TTC（继承旧栈 class-aware AEB 实测结论）
//   5. 连帧确认触发 / 连帧清空释放（防单帧噪声误触发——Autoware collision_keeping 同思想）
#ifndef ADAS_AEB__AEB_CORE_HPP_
#define ADAS_AEB__AEB_CORE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "adas_common/types.hpp"

namespace adas::control {

struct AebParams {
  double horizon_s{4.0};
  double step_s{0.1};
  double corridor_half_width_m{1.3};   // 自车半宽 + 侧向裕量
  double rear_filter_m{5.0};           // 车后超过该距离的目标不参与 AEB
  double corridor_width_m{3.5};        // 冲突扫描走廊的横向半宽
  double obj_radius_m{0.5};            // 目标等效半径（点目标膨胀）
  double ttc_emergency_car_s{1.8};
  double ttc_emergency_ped_s{2.5};     // 行人阈值更长
  double min_active_speed_mps{1.0};    // 低速不激活（Autoware 同款）
  int trigger_frames{3};               // 连帧确认触发
  int release_frames{10};              // 连帧清空释放
  double emergency_decel_mps2{8.0};    // 全力制动
};

struct AebObject {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};      // 世界系航向（速度方向）
  double v_mps{0.0};
  uint8_t classification{0};  // 对齐 adas_msgs/TrackedObject CLASS_*
};

enum class AebState : uint8_t {
  kInactive = 0,
  kMonitoring = 1,
  kWarning = 2,
  kEmergency = 3,
};

struct AebResult {
  AebState state{AebState::kInactive};
  double ttc_s{1e9};
  double required_decel_mps2{0.0};   // 在冲突点前停住所需减速度
  std::string reason;
  bool emergency_active{false};
  double brake_accel_mps2{0.0};      // EMERGENCY 时 = -emergency_decel
};

class AebCore {
 public:
  explicit AebCore(const AebParams& params);
  AebResult update(const common::KinematicState& ego, const std::vector<AebObject>& objects);

 private:
  AebParams params_;
  int trigger_count_{0};
  int clear_count_{0};
  bool latched_{false};
};

}  // namespace adas::control

#endif  // ADAS_AEB__AEB_CORE_HPP_
