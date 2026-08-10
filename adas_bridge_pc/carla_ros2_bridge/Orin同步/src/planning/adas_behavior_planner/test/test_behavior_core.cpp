// BehaviorCore 单测（M2 跟车 + M4 超车）
#include <gtest/gtest.h>

#include "adas_behavior_planner/behavior_core.hpp"

namespace ap = adas::planning;

namespace {

// 前车（id=1）在本车道 gap 处
ap::BehaviorInput lead_at(double gap, double v_lead = 10.0) {
  ap::BehaviorInput in;
  in.primary_lead_id = 1;
  in.lead_gap_m = gap;
  in.lead_speed_mps = v_lead;
  in.ego_speed_mps = 15.0;
  in.objects.push_back(ap::ObjectLite{1, gap, 0.0, v_lead});
  return in;
}

ap::BehaviorParams fast_params() {
  ap::BehaviorParams p;
  p.overtake_wait_frames = 2;
  p.clear_confirm_frames = 2;
  p.exit_hysteresis_frames = 3;
  return p;
}

// 驱动到 OVERTAKE_ACTIVE 状态
void drive_to_active(ap::BehaviorCore& core) {
  auto slow = lead_at(25.0, 5.0);  // 慢车（<0.6*15=9）且已逼近
  core.update(slow);               // LANE_FOLLOW → FOLLOW_LEAD
  core.update(slow);
  core.update(slow);               // slow_count 达到 → WAIT
  core.update(slow);               // clear_count 1
  core.update(slow);               // clear_count 2 → ACTIVE
}

}  // namespace

TEST(BehaviorCore, EntersFollowWhenLeadNear) {
  ap::BehaviorCore core((ap::BehaviorParams()));
  EXPECT_EQ(core.update(ap::BehaviorInput{}).state, ap::BehaviorKind::kLaneFollow);
  EXPECT_EQ(core.update(lead_at(60.0)).state, ap::BehaviorKind::kFollowLead);
}

TEST(BehaviorCore, ExitNeedsHysteresis) {
  ap::BehaviorParams p;
  p.exit_hysteresis_frames = 3;
  ap::BehaviorCore core(p);
  core.update(lead_at(60.0));
  ap::BehaviorInput no_lead;
  no_lead.primary_lead_id = -1;
  EXPECT_EQ(core.update(no_lead).state, ap::BehaviorKind::kFollowLead);
  EXPECT_EQ(core.update(no_lead).state, ap::BehaviorKind::kFollowLead);
  EXPECT_EQ(core.update(no_lead).state, ap::BehaviorKind::kLaneFollow);
}

TEST(BehaviorCore, MrmOverridesAll) {
  ap::BehaviorCore core((ap::BehaviorParams()));
  core.update(lead_at(60.0));
  auto in = lead_at(60.0);
  in.mrm_stop = true;
  const auto out = core.update(in);
  EXPECT_EQ(out.state, ap::BehaviorKind::kStopping);
  EXPECT_NEAR(out.target_speed_mps, 0.0, 1e-9);
}

TEST(BehaviorCore, SlowLeadTriggersOvertakeSequence) {
  ap::BehaviorCore core(fast_params());
  drive_to_active(core);
  EXPECT_EQ(core.state(), ap::BehaviorKind::kOvertakeActive);
  // ACTIVE 输出目标车道 -1
  auto in = lead_at(25.0, 5.0);
  const auto out = core.update(in);
  EXPECT_EQ(out.target_lane, -1);
}

TEST(BehaviorCore, FastLeadNoOvertake) {
  ap::BehaviorCore core(fast_params());
  auto in = lead_at(25.0, 12.0);  // 快车（>9）
  for (int i = 0; i < 20; ++i) {
    core.update(in);
  }
  EXPECT_EQ(core.state(), ap::BehaviorKind::kFollowLead);
}

TEST(BehaviorCore, AdjacentOccupiedBlocksLaneChange) {
  ap::BehaviorCore core(fast_params());
  auto slow = lead_at(25.0, 5.0);
  // 左邻道（lat≈3.5）30m 前方有车 → 邻道不清空，停在 WAIT
  slow.objects.push_back(ap::ObjectLite{7, 30.0, 3.5, 10.0});
  for (int i = 0; i < 20; ++i) {
    core.update(slow);
  }
  EXPECT_EQ(core.state(), ap::BehaviorKind::kOvertakeWait);
}

TEST(BehaviorCore, AbortWhenDangerAppearsEarly) {
  ap::BehaviorCore core(fast_params());
  drive_to_active(core);
  ASSERT_EQ(core.state(), ap::BehaviorKind::kOvertakeActive);
  // 自车仍在本车道内（lat=0.5），邻道突现目标 → 中止回跟车
  auto in = lead_at(25.0, 5.0);
  in.ego_lateral_m = 0.5;
  in.objects.push_back(ap::ObjectLite{7, 20.0, 3.5, 10.0});
  EXPECT_EQ(core.update(in).state, ap::BehaviorKind::kFollowLead);
}

TEST(BehaviorCore, NoAbortWhenCommitted) {
  ap::BehaviorCore core(fast_params());
  drive_to_active(core);
  // 自车已横移过中止界限（lat=2.5）→ 不中止，保持 ACTIVE
  auto in = lead_at(25.0, 5.0);
  in.ego_lateral_m = 2.5;
  in.objects.push_back(ap::ObjectLite{7, 20.0, 3.5, 10.0});
  EXPECT_EQ(core.update(in).state, ap::BehaviorKind::kOvertakeActive);
}

TEST(BehaviorCore, PassedLeadTriggersReturnAndFinish) {
  ap::BehaviorCore core(fast_params());
  drive_to_active(core);
  // 被超车辆落到自车后方 10m（> pass_margin 8）→ RETURN
  ap::BehaviorInput in;
  in.primary_lead_id = -1;  // 自车已在左道，本带无前车
  in.ego_speed_mps = 15.0;
  in.ego_lateral_m = 3.4;
  in.objects.push_back(ap::ObjectLite{1, -10.0, 0.0, 5.0});
  auto out = core.update(in);
  EXPECT_EQ(out.state, ap::BehaviorKind::kOvertakeReturn);
  EXPECT_EQ(out.target_lane, 0);  // 回本车道
  // 回到本车道中心 → 完成
  in.ego_lateral_m = 0.3;
  EXPECT_EQ(core.update(in).state, ap::BehaviorKind::kLaneFollow);
}

ap::MapSignLite stop_sign(double distance_m) {
  return {ap::MapSignType::kStopSign, distance_m, false, 1};
}

ap::MapSignLite traffic_light(double distance_m, bool red) {
  return {ap::MapSignType::kTrafficLight, distance_m, red, 1};
}

ap::MapSignLite junction(double distance_m) {
  return {ap::MapSignType::kJunction, distance_m, false, 1};
}

TEST(BehaviorCoreSigns, StopSignFarAwayDoesNotTrigger) {
  ap::BehaviorCore core((ap::BehaviorParams()));
  ap::BehaviorInput in;
  in.map_signs = {stop_sign(30.0)};
  EXPECT_EQ(core.update(in).state, ap::BehaviorKind::kLaneFollow);
}

TEST(BehaviorCoreSigns, StopSignNearEntersApproach) {
  ap::BehaviorCore core((ap::BehaviorParams()));
  ap::BehaviorInput in;
  in.map_signs = {stop_sign(10.0)};
  EXPECT_EQ(core.update(in).state, ap::BehaviorKind::kApproachingStop);
}

TEST(BehaviorCoreSigns, StopSignWaitsTwoSecondsThenResumes) {
  ap::BehaviorParams params;
  params.stop_sign_stop_duration_s = 2.0;
  ap::BehaviorCore core(params);
  ap::BehaviorInput in;
  in.now_s = 0.0;
  in.map_signs = {stop_sign(10.0)};
  EXPECT_EQ(core.update(in).state, ap::BehaviorKind::kApproachingStop);
  in.map_signs = {stop_sign(0.0)};
  in.now_s = 1.0;
  EXPECT_EQ(core.update(in).state, ap::BehaviorKind::kStoppingAtStop);
  in.now_s = 3.0;
  EXPECT_EQ(core.update(in).state, ap::BehaviorKind::kLaneFollow);
}

TEST(BehaviorCoreSigns, GreenLightExitsWaiting) {
  ap::BehaviorCore core((ap::BehaviorParams()));
  ap::BehaviorInput in;
  in.map_signs = {traffic_light(20.0, true)};
  EXPECT_EQ(core.update(in).state, ap::BehaviorKind::kWaitingAtLight);
  in.map_signs = {traffic_light(20.0, false)};
  EXPECT_EQ(core.update(in).state, ap::BehaviorKind::kLaneFollow);
}

TEST(BehaviorCoreSigns, JunctionTransitionsThroughEnteringState) {
  ap::BehaviorCore core((ap::BehaviorParams()));
  ap::BehaviorInput in;
  in.map_signs = {junction(10.0)};
  EXPECT_EQ(core.update(in).state, ap::BehaviorKind::kEnteringJunction);
  EXPECT_EQ(core.update(in).state, ap::BehaviorKind::kLaneFollow);
}

TEST(BehaviorCoreSigns, TrafficLightFarAwayDoesNotTrigger) {
  ap::BehaviorCore core((ap::BehaviorParams()));
  ap::BehaviorInput in;
  in.map_signs = {traffic_light(40.0, true)};
  EXPECT_EQ(core.update(in).state, ap::BehaviorKind::kLaneFollow);
}
