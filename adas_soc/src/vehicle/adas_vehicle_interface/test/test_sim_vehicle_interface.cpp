// SimVehicleInterface 单测
#include <gtest/gtest.h>

#include "adas_vehicle_interface/sim_vehicle_interface.hpp"

namespace av = adas::vehicle;
namespace ac = adas::common;

namespace {

ac::ControlData cmd_of(double steer, double accel) {
  ac::ControlData c;
  c.lateral.steering_tire_angle_rad = steer;
  c.longitudinal.acceleration_mps2 = accel;
  return c;
}

}  // namespace

TEST(SimVehicleInterface, PositiveAccelIsThrottleOnly) {
  av::SimVehicleInterface vi((av::SimVehicleInterfaceParams()));  // max_accel=3
  const auto act = vi.apply(cmd_of(0.0, 1.5), {});
  EXPECT_NEAR(act.throttle, 0.5, 1e-9);
  EXPECT_NEAR(act.brake, 0.0, 1e-9);
}

TEST(SimVehicleInterface, NegativeAccelIsBrakeOnly) {
  av::SimVehicleInterface vi((av::SimVehicleInterfaceParams()));  // max_decel=8
  const auto act = vi.apply(cmd_of(0.0, -4.0), {});
  EXPECT_NEAR(act.brake, 0.5, 1e-9);
  EXPECT_NEAR(act.throttle, 0.0, 1e-9);
}

TEST(SimVehicleInterface, DragCompensationMaintainsCruiseThrottle) {
  av::SimVehicleInterfaceParams params;
  params.drag_compensation_per_speed = 0.1;
  av::SimVehicleInterface vi(params);
  ac::KinematicState state;
  state.velocity_mps = 15.0;

  const auto act = vi.apply(cmd_of(0.0, 0.0), state);
  EXPECT_NEAR(act.throttle, 0.5, 1e-9);
  EXPECT_NEAR(act.brake, 0.0, 1e-9);
}

TEST(SimVehicleInterface, DragCompensationDoesNotWeakenBraking) {
  av::SimVehicleInterfaceParams params;
  params.drag_compensation_per_speed = 0.1;
  av::SimVehicleInterface vi(params);
  ac::KinematicState state;
  state.velocity_mps = 15.0;

  const auto act = vi.apply(cmd_of(0.0, -4.0), state);
  EXPECT_NEAR(act.throttle, 0.0, 1e-9);
  EXPECT_NEAR(act.brake, 0.5, 1e-9);
}

TEST(SimVehicleInterface, SteerNormalizedAndClamped) {
  av::SimVehicleInterface vi((av::SimVehicleInterfaceParams()));  // max_steer=0.6
  EXPECT_NEAR(vi.apply(cmd_of(0.3, 0.0), {}).steer, 0.5, 1e-9);
  EXPECT_NEAR(vi.apply(cmd_of(-0.3, 0.0), {}).steer, -0.5, 1e-9);
  EXPECT_NEAR(vi.apply(cmd_of(2.0, 0.0), {}).steer, 1.0, 1e-9);   // 超限钳制
  EXPECT_NEAR(vi.apply(cmd_of(-2.0, 0.0), {}).steer, -1.0, 1e-9);
}

TEST(SimVehicleInterface, ActuationAlwaysInRange) {
  av::SimVehicleInterface vi((av::SimVehicleInterfaceParams()));
  const auto act = vi.apply(cmd_of(0.0, -100.0), {});
  EXPECT_GE(act.brake, 0.0);
  EXPECT_LE(act.brake, 1.0);
  const auto act2 = vi.apply(cmd_of(0.0, 100.0), {});
  EXPECT_LE(act2.throttle, 1.0);
}
