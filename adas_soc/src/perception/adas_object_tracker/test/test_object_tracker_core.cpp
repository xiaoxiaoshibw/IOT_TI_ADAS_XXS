// ObjectTrackerCore 单测
#include <gtest/gtest.h>

#include <cmath>

#include "adas_object_tracker/object_tracker_core.hpp"

namespace ap = adas::perception;
namespace ac = adas::common;

namespace {

ac::KinematicState ego_at(double x, double y, double yaw, double v) {
  ac::KinematicState s;
  s.pose = ac::Pose2d{x, y, yaw};
  s.velocity_mps = v;
  return s;
}

ac::LaneStateData straight_lane() {
  ac::LaneStateData l;
  l.valid = true;
  return l;
}

ap::RawObject car_at(uint32_t id, double x, double y, double v) {
  ap::RawObject o;
  o.id = id;
  o.classification = 1;  // CAR
  o.x = x;
  o.y = y;
  o.v_mps = v;
  return o;
}

}  // namespace

TEST(ObjectTracker, ElectsNearestInLaneAhead) {
  ap::ObjectTrackerCore core((ap::TrackerParams()));
  const auto out = core.update(
      0.0, {car_at(1, 50.0, 0.0, 10.0), car_at(2, 30.0, 0.2, 8.0), car_at(3, 20.0, 3.5, 5.0)},
      ego_at(0, 0, 0, 15), straight_lane());
  // id=3 在邻道（lat=3.5 > 3.5/2*0.9），id=2 最近同道
  EXPECT_EQ(out.primary_lead_id, 2);
  EXPECT_NEAR(out.primary_lead_gap_m, 30.0, 0.5);
}

TEST(ObjectTracker, IgnoresBehindAndFarObjects) {
  ap::TrackerParams p;
  p.max_lead_range_m = 100.0;
  ap::ObjectTrackerCore core(p);
  const auto out = core.update(0.0, {car_at(1, -20.0, 0.0, 10.0), car_at(2, 150.0, 0.0, 10.0)},
                               ego_at(0, 0, 0, 15), straight_lane());
  EXPECT_EQ(out.primary_lead_id, -1);
}

TEST(ObjectTracker, LeadSwapFlag) {
  ap::ObjectTrackerCore core((ap::TrackerParams()));
  auto out = core.update(0.0, {car_at(1, 50.0, 0.0, 10.0)}, ego_at(0, 0, 0, 15), straight_lane());
  EXPECT_TRUE(out.lead_swapped);  // 无 → 1
  out = core.update(0.05, {car_at(1, 50.0, 0.0, 10.0)}, ego_at(0, 0, 0, 15), straight_lane());
  EXPECT_FALSE(out.lead_swapped);
  // 更近同道目标切入 → 选举切换
  out = core.update(0.1, {car_at(1, 50.0, 0.0, 10.0), car_at(2, 25.0, 0.0, 9.0)},
                    ego_at(0, 0, 0, 15), straight_lane());
  EXPECT_EQ(out.primary_lead_id, 2);
  EXPECT_TRUE(out.lead_swapped);
}

TEST(ObjectTracker, VelocityFilterConverges) {
  ap::ObjectTrackerCore core((ap::TrackerParams()));
  ap::TrackerOutput out;
  for (int i = 0; i < 100; ++i) {
    out = core.update(i * 0.05, {car_at(1, 50.0, 0.0, 10.0)}, ego_at(0, 0, 0, 15),
                      straight_lane());
  }
  EXPECT_NEAR(out.primary_lead_speed_mps, 10.0, 0.1);
  EXPECT_NEAR(out.objects[0].a_est_mps2, 0.0, 0.1);  // 匀速 → 加速度约 0
}

TEST(ObjectTracker, CurvatureCorrectionKeepsCurveLeadInLane) {
  ap::ObjectTrackerCore core((ap::TrackerParams()));
  ac::LaneStateData lane = straight_lane();
  lane.curvature = 0.01;  // R=100 左弯
  // 前车在弯道中心线上：40m 前方，横向 ≈ 0.5*0.01*40² = 8m（自车切线系）
  const auto out = core.update(0.0, {car_at(1, 40.0, 8.0, 10.0)}, ego_at(0, 0, 0, 15), lane);
  EXPECT_EQ(out.primary_lead_id, 1);  // 曲率修正后应判定同道
  // 同样位置无曲率修正（直道）→ 8m 横向 = 邻道之外
  const auto out2 = core.update(0.05, {car_at(1, 40.0, 8.0, 10.0)}, ego_at(0, 0, 0, 15),
                                straight_lane());
  EXPECT_EQ(out2.primary_lead_id, -1);
}

TEST(ObjectTracker, StaleTrackPruned) {
  ap::TrackerParams p;
  p.track_stale_s = 0.5;
  ap::ObjectTrackerCore core(p);
  core.update(0.0, {car_at(1, 50.0, 0.0, 10.0)}, ego_at(0, 0, 0, 15), straight_lane());
  // 1s 后目标消失（空 raw）→ track 清理 + 选举为空 + swap 标志
  const auto out = core.update(1.0, {}, ego_at(0, 0, 0, 15), straight_lane());
  EXPECT_EQ(out.primary_lead_id, -1);
  EXPECT_TRUE(out.lead_swapped);
}
