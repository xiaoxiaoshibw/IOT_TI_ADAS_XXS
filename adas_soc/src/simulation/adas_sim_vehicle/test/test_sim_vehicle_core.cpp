// SimVehicleCore 单测
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

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

TEST(SimVehicleCore, ScriptedActorsAreSortedAndIdsStayStable) {
  auto core = make_straight_core();
  std::vector<as::ScriptedActor> actors;
  for (const std::uint32_t id : {20U, 3U, 11U}) {
    as::ScriptedActor actor;
    actor.id = id;
    actor.classification = as::ScriptedActorClass::Car;
    actor.initial_station_m = static_cast<double>(id);
    actor.initial_speed_mps = 5.0;
    actor.speed_profile = {{0.0, 5.0}};
    actors.push_back(actor);
  }
  core.set_scripted_actors(actors);
  ASSERT_EQ(core.snapshot_objects().size(), 3U);
  EXPECT_EQ(core.snapshot_objects()[0].id, 3U);
  EXPECT_EQ(core.snapshot_objects()[1].id, 11U);
  EXPECT_EQ(core.snapshot_objects()[2].id, 20U);
  ac::ActuationData act;
  for (int i = 0; i < 100; ++i) core.step(act, 0.02);
  const auto snapshot = core.snapshot_objects();
  ASSERT_EQ(snapshot.size(), 3U);
  EXPECT_EQ(snapshot[0].id, 3U);
  EXPECT_EQ(snapshot[1].id, 11U);
  EXPECT_EQ(snapshot[2].id, 20U);
}

TEST(SimVehicleCore, ActorLimitAndDuplicateIdsAreRejectedAtomically) {
  auto core = make_straight_core();
  as::ScriptedActor actor;
  actor.id = 1;
  actor.classification = as::ScriptedActorClass::Car;
  actor.speed_profile = {{0.0, 0.0}};
  core.set_scripted_actors({actor});

  auto duplicate = actor;
  EXPECT_THROW(core.set_scripted_actors({actor, duplicate}), std::invalid_argument);
  EXPECT_EQ(core.snapshot_objects().size(), 1U);

  std::vector<as::ScriptedActor> too_many(as::SimVehicleCore::kMaxScriptedActors + 1,
                                          actor);
  for (std::size_t i = 0; i < too_many.size(); ++i) too_many[i].id = i + 1;
  EXPECT_THROW(core.set_scripted_actors(too_many), std::invalid_argument);
  EXPECT_EQ(core.snapshot_objects().size(), 1U);
}

TEST(SimVehicleCore, ProfileBoundaryAndAccelerationLimitAreDeterministic) {
  auto core = make_straight_core();
  as::ScriptedActor actor;
  actor.id = 7;
  actor.classification = as::ScriptedActorClass::Car;
  actor.initial_speed_mps = 2.0;
  actor.accel_limit_mps2 = 1.0;
  actor.speed_profile = {{0.0, 2.0}, {1.0, 4.0}};
  core.set_scripted_actors({actor});
  ac::ActuationData act;
  for (int i = 0; i < 50; ++i) core.step(act, 0.02);
  EXPECT_NEAR(core.snapshot_objects().front().v_mps, 2.02, 1e-9);
  for (int i = 0; i < 49; ++i) core.step(act, 0.02);
  EXPECT_NEAR(core.snapshot_objects().front().v_mps, 3.0, 1e-9);
}

TEST(SimVehicleCore, SpawnDisappearAndNegativeStationAreSupported) {
  auto core = make_straight_core();
  as::ScriptedActor actor;
  actor.id = 9;
  actor.classification = as::ScriptedActorClass::Car;
  actor.initial_station_m = -10.0;
  actor.initial_speed_mps = 0.0;
  actor.speed_profile = {{0.0, 0.0}};
  actor.spawn_time_s = 1.0;
  actor.disappear_time_s = 2.0;
  core.set_scripted_actors({actor});
  ac::ActuationData act;
  EXPECT_TRUE(core.snapshot_objects().empty());
  for (int i = 0; i < 50; ++i) core.step(act, 0.02);
  ASSERT_EQ(core.snapshot_objects().size(), 1U);
  EXPECT_NEAR(core.snapshot_objects().front().x, -10.0, 1e-6);
  for (int i = 0; i < 50; ++i) core.step(act, 0.02);
  EXPECT_TRUE(core.snapshot_objects().empty());
}

TEST(SimVehicleCore, PedestrianCrossingUsesEgoGapAndDisappears) {
  auto core = make_straight_core();
  core.set_initial_state(0.0, 0.0, 0.0);
  as::ScriptedActor actor;
  actor.id = 2;
  actor.classification = as::ScriptedActorClass::Pedestrian;
  actor.initial_station_m = 10.0;
  actor.initial_lateral_m = -1.0;
  actor.initial_speed_mps = 1.0;
  actor.accel_limit_mps2 = 1.0;
  actor.speed_profile = {{0.0, 1.0}};
  actor.trigger_ego_gap_m = 15.0;
  actor.crossing_end_lateral_m = 1.0;
  actor.crossing_speed_mps = 1.0;
  core.set_scripted_actors({actor});
  ac::ActuationData act;
  core.step(act, 0.5);
  ASSERT_EQ(core.snapshot_objects().size(), 1U);
  EXPECT_NEAR(core.snapshot_objects().front().lateral_m, -0.5, 1e-9);
  for (int i = 0; i < 3; ++i) core.step(act, 0.5);
  EXPECT_TRUE(core.snapshot_objects().empty());
}

TEST(SimVehicleCore, LegacySettersPopulateUnifiedSnapshots) {
  auto core = make_straight_core();
  as::LeadScript lead;
  lead.enabled = true;
  lead.initial_station_m = 30.0;
  lead.initial_speed_mps = 4.0;
  core.set_lead_script(lead);
  as::AdjacentCarScript adjacent;
  adjacent.enabled = true;
  adjacent.initial_station_m = 20.0;
  adjacent.speed_mps = 3.0;
  core.set_adjacent_car_script(adjacent);
  const auto objects = core.snapshot_objects();
  ASSERT_EQ(objects.size(), 2U);
  EXPECT_EQ(objects[0].id, 1U);
  EXPECT_EQ(objects[1].id, 3U);
  EXPECT_TRUE(core.lead_state().present);
  EXPECT_TRUE(core.adjacent_car_state().present);
}
