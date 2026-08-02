// SimVehicleCore 单测
#include <gtest/gtest.h>

#include <cmath>

#include "adas_sim_vehicle/sim_vehicle_core.hpp"

namespace as = adas::sim;
namespace ac = adas::common;

namespace {

as::SimVehicleCore make_straight_core() {
  as::VehicleParams vp;
  vp.steer_tau_s = 0.0;  // 测试用：转向零迟滞
  vp.drag_accel_mps2 = 0.0;
  return as::SimVehicleCore(vp, {{500.0, 0.0}}, 3.5);
}

}  // namespace

TEST(SimVehicleCore, StraightDrivingStaysCentered) {
  auto core = make_straight_core();
  core.set_initial_state(0.0, 0.0, 10.0);
  ac::ActuationData act;  // 无油门无制动无转向
  for (int i = 0; i < 250; ++i) {
    core.step(act, 0.02);  // 5s
  }
  const auto ls = core.lane_state();
  ASSERT_TRUE(ls.valid);
  EXPECT_NEAR(ls.lateral_offset, 0.0, 1e-6);
  EXPECT_NEAR(ls.heading_error, 0.0, 1e-6);
  EXPECT_NEAR(core.state().pose.x, 50.0, 0.5);
}

TEST(SimVehicleCore, LateralOffsetSignLeftPositive) {
  auto core = make_straight_core();
  core.set_initial_state(10.0, 0.8, 5.0);  // 左偏 0.8m
  const auto ls = core.lane_state();
  ASSERT_TRUE(ls.valid);
  EXPECT_NEAR(ls.lateral_offset, 0.8, 1e-6);
}

TEST(SimVehicleCore, SteeringLeftTurnsLeft) {
  auto core = make_straight_core();
  core.set_initial_state(0.0, 0.0, 10.0);
  ac::ActuationData act;
  act.steer = 0.5;  // 左打半舵
  for (int i = 0; i < 100; ++i) {
    core.step(act, 0.02);
  }
  EXPECT_GT(core.state().pose.y, 0.1);          // 向左（+y）偏
  EXPECT_GT(core.state().pose.yaw, 0.01);       // 航向左偏
  EXPECT_GT(core.steering_angle_rad(), 0.0);
}

TEST(SimVehicleCore, BrakingStopsAndNoReverse) {
  auto core = make_straight_core();
  core.set_initial_state(0.0, 0.0, 10.0);
  ac::ActuationData act;
  act.brake = 1.0;
  for (int i = 0; i < 500; ++i) {
    core.step(act, 0.02);  // 10s 全力制动
  }
  EXPECT_NEAR(core.state().velocity_mps, 0.0, 1e-9);
  EXPECT_GE(core.state().velocity_mps, 0.0);
}

TEST(SimVehicleCore, ThrottleAccelerates) {
  auto core = make_straight_core();
  core.set_initial_state(0.0, 0.0, 0.0);
  ac::ActuationData act;
  act.throttle = 1.0;
  for (int i = 0; i < 100; ++i) {
    core.step(act, 0.02);  // 2s 全油门（max_accel=3.0）
  }
  EXPECT_NEAR(core.state().velocity_mps, 6.0, 0.1);
}

TEST(SimVehicleCore, OffTrackEndInvalidatesLane) {
  as::VehicleParams vp;
  auto core = as::SimVehicleCore(vp, {{20.0, 0.0}}, 3.5);  // 短赛道
  core.set_initial_state(0.0, 0.0, 20.0);
  ac::ActuationData act;
  for (int i = 0; i < 200; ++i) {
    core.step(act, 0.02);  // 冲出赛道末端
  }
  EXPECT_FALSE(core.lane_state().valid);
}

TEST(SimVehicleCore, LeadScriptFollowsEvents) {
  as::VehicleParams vp;
  as::SimVehicleCore core(vp, {{1000.0, 0.0}}, 3.5);
  as::LeadScript script;
  script.enabled = true;
  script.initial_station_m = 60.0;
  script.initial_speed_mps = 10.0;
  script.accel_mps2 = 5.0;
  script.events = {{2.0, 0.0}};  // t=2s 减速到停
  core.set_lead_script(script);

  ac::ActuationData act;
  // 1s：前车应匀速前进
  for (int i = 0; i < 50; ++i) core.step(act, 0.02);
  auto lead = core.lead_state();
  ASSERT_TRUE(lead.present);
  EXPECT_NEAR(lead.v_mps, 10.0, 1e-9);
  EXPECT_NEAR(lead.station_m, 70.0, 0.1);
  // 到 5s：事件生效（2s 后 5m/s² 减速 2s 停住）
  for (int i = 0; i < 200; ++i) core.step(act, 0.02);
  lead = core.lead_state();
  EXPECT_NEAR(lead.v_mps, 0.0, 1e-9);
}

TEST(SimVehicleCore, LeadDisabledNotPresent) {
  as::VehicleParams vp;
  as::SimVehicleCore core(vp, {{100.0, 0.0}}, 3.5);
  EXPECT_FALSE(core.lead_state().present);
}

TEST(SimVehicleCore, CurvedTrackLaneStateOnCircle) {
  as::VehicleParams vp;
  vp.steer_tau_s = 0.0;
  vp.drag_accel_mps2 = 0.0;
  // 100m 直道 + 半径 50m 左弯
  auto core = as::SimVehicleCore(vp, {{100.0, 0.0}, {150.0, 0.02}}, 3.5);
  core.set_initial_state(120.0, 0.0, 0.0);  // 静置弯道中
  const auto ls = core.lane_state();
  ASSERT_TRUE(ls.valid);
  EXPECT_NEAR(ls.curvature, 0.02, 1e-9);
  EXPECT_NEAR(ls.lateral_offset, 0.0, 0.05);
  EXPECT_NEAR(ls.heading_error, 0.0, 0.05);
}
