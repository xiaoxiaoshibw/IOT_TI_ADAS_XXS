// adas_vehicle_interface/sim_vehicle_interface.hpp
// 仿真车辆实现：加速度 → 油门/制动线性映射（M6 CARLA 阶段可换标定查表），
// 转角 → 归一化转向。标定参数须与 adas_sim_vehicle 的 VehicleParams 一致。
#ifndef ADAS_VEHICLE_INTERFACE__SIM_VEHICLE_INTERFACE_HPP_
#define ADAS_VEHICLE_INTERFACE__SIM_VEHICLE_INTERFACE_HPP_

#include "adas_vehicle_interface/vehicle_interface_base.hpp"

namespace adas::vehicle {

struct SimVehicleInterfaceParams {
  double max_accel_mps2{3.0};   // 油门=1 对应的加速度
  double max_decel_mps2{8.0};   // 制动=1 对应的减速度
  double max_steer_rad{0.6};    // steer=±1 对应的前轮转角
};

class SimVehicleInterface : public VehicleInterfaceBase {
 public:
  explicit SimVehicleInterface(const SimVehicleInterfaceParams& params);
  common::ActuationData apply(const common::ControlData& cmd,
                              const common::KinematicState& state) override;

 private:
  SimVehicleInterfaceParams params_;
};

}  // namespace adas::vehicle

#endif  // ADAS_VEHICLE_INTERFACE__SIM_VEHICLE_INTERFACE_HPP_
