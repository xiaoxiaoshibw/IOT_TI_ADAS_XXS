// 仿真车辆接口实现
#include "adas_vehicle_interface/sim_vehicle_interface.hpp"

#include <algorithm>
#include <cmath>

namespace adas::vehicle {

SimVehicleInterface::SimVehicleInterface(const SimVehicleInterfaceParams& params)
    : params_(params) {}

common::ActuationData SimVehicleInterface::apply(const common::ControlData& cmd,
                                                 const common::KinematicState& state) {
  common::ActuationData act;
  const double a = cmd.longitudinal.acceleration_mps2;
  if (a >= 0.0) {
    const double speed = std::isfinite(state.velocity_mps)
                             ? std::max(0.0, state.velocity_mps)
                             : 0.0;
    const double compensated_accel = a + params_.drag_compensation_per_speed * speed;
    act.throttle = std::clamp(compensated_accel / params_.max_accel_mps2, 0.0, 1.0);
    act.brake = 0.0;
  } else {
    act.throttle = 0.0;
    act.brake = std::clamp(-a / params_.max_decel_mps2, 0.0, 1.0);
  }
  act.steer =
      std::clamp(cmd.lateral.steering_tire_angle_rad / params_.max_steer_rad, -1.0, 1.0);
  return act;
}

}  // namespace adas::vehicle
