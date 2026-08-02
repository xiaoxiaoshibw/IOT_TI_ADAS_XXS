// adas_controller_base/controller_base.hpp
// 横/纵向控制器插件虚基类（对标 Autoware trajectory_follower_base）。
// follower 节点按参数选择具体实现构造；换算法只换实现类，节点零改。
#ifndef ADAS_CONTROLLER_BASE__CONTROLLER_BASE_HPP_
#define ADAS_CONTROLLER_BASE__CONTROLLER_BASE_HPP_

#include "adas_common/types.hpp"

namespace adas::control {

// 控制器统一输入（core 层纯结构，无 ROS 依赖）
struct ControlInput {
  const common::Trajectory* trajectory{nullptr};  // 非空且 >=2 点由调用方保证
  common::KinematicState state;
  double steering_angle_rad{0.0};                 // 转角回读
  double dt{0.02};
};

class LateralControllerBase {
 public:
  virtual ~LateralControllerBase() = default;
  virtual common::LateralCommandData run(const ControlInput& input) = 0;
  virtual void reset() {}
};

class LongitudinalControllerBase {
 public:
  virtual ~LongitudinalControllerBase() = default;
  virtual common::LongitudinalCommandData run(const ControlInput& input) = 0;
  virtual void reset() {}
};

}  // namespace adas::control

#endif  // ADAS_CONTROLLER_BASE__CONTROLLER_BASE_HPP_
