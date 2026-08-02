// adas_vehicle_interface/vehicle_interface_base.hpp
// 车辆/底盘接口抽象（对标 Apollo VehicleController + Autoware raw_vehicle_cmd_converter）。
// 换执行器只换实现：SimVehicleInterface（M1）→ Esp32VehicleInterface / CAN（实机阶段）。
#ifndef ADAS_VEHICLE_INTERFACE__VEHICLE_INTERFACE_BASE_HPP_
#define ADAS_VEHICLE_INTERFACE__VEHICLE_INTERFACE_BASE_HPP_

#include "adas_common/types.hpp"

namespace adas::vehicle {

class VehicleInterfaceBase {
 public:
  virtual ~VehicleInterfaceBase() = default;
  // 控制命令 + 当前运动状态 → 归一化执行量
  virtual common::ActuationData apply(const common::ControlData& cmd,
                                      const common::KinematicState& state) = 0;
};

}  // namespace adas::vehicle

#endif  // ADAS_VEHICLE_INTERFACE__VEHICLE_INTERFACE_BASE_HPP_
