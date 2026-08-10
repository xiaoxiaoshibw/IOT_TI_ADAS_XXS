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
  l.lane_width = 3.5;  // 修复：原 helper 未设 width，导致 in_ego_lane 始终 false
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
  // Commit 5 — 更近同道目标切入不再立即抢断主车：sticky 选择要等 3 帧确认 +
  // 至少 4 m gap 收益。第一帧仅作候选计数；切换要等到第 3 帧。
  out = core.update(0.10, {car_at(1, 50.0, 0.0, 10.0), car_at(2, 25.0, 0.0, 9.0)},
                    ego_at(0, 0, 0, 15), straight_lane());
  EXPECT_EQ(out.primary_lead_id, 1);  // 仍沿用老目标 (sticky + 3 帧 confirm)
  out = core.update(0.15, {car_at(1, 50.0, 0.0, 10.0), car_at(2, 25.0, 0.0, 9.0)},
                    ego_at(0, 0, 0, 15), straight_lane());
  EXPECT_EQ(out.primary_lead_id, 1);
  out = core.update(0.20, {car_at(1, 50.0, 0.0, 10.0), car_at(2, 25.0, 0.0, 9.0)},
                    ego_at(0, 0, 0, 15), straight_lane());
  EXPECT_EQ(out.primary_lead_id, 2);  // 第 3 帧才发生抢占
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
  // Commit 5 — sticky 主车短暂丢失仍报告原 ID（最多 3 帧）。
  // 切到直道后 lat=8m，超出 in-lane，但 retain 期内的 primary_lead_id 仍为 1。
  const auto out2 = core.update(0.05, {car_at(1, 40.0, 8.0, 10.0)}, ego_at(0, 0, 0, 15),
                                straight_lane());
  EXPECT_EQ(out2.primary_lead_id, 1);
  EXPECT_NEAR(out2.primary_lead_gap_m, 40.0, 0.5);
}

TEST(ObjectTracker, StaleTrackPruned) {
  ap::TrackerParams p;
  p.track_stale_s = 0.5;
  ap::ObjectTrackerCore core(p);
  auto r0 = core.update(0.0, {car_at(1, 50.0, 0.0, 10.0)}, ego_at(0, 0, 0, 15), straight_lane());
  EXPECT_EQ(r0.primary_lead_id, 1);
  // 单次 1 秒间隔后目标消失 → sticky retain 期内仍报告原 ID
  const auto out = core.update(1.0, {}, ego_at(0, 0, 0, 15), straight_lane());
  EXPECT_EQ(out.primary_lead_id, 1);
  EXPECT_FALSE(out.lead_swapped);
  // retain_frames_on_lost=3 → lost 计数到 3 后清空。
  // t=1: lost=1 (retain), t=2: lost=2 (retain), t=3: lost=3 (retain), t=4: lost=3, 3<3=false → clear
  // → 实际可保留 3 帧，第 4 帧清空。下面只循环 2 次以避开临界：
  for (int i = 0; i < 2; ++i) {
    auto retain_out = core.update(2.0 + i * 1.0, {}, ego_at(0, 0, 0, 15), straight_lane());
    EXPECT_EQ(retain_out.primary_lead_id, 1);
    EXPECT_FALSE(retain_out.lead_swapped);
  }
  // 累计 lost 已达 3：第 4 次空帧触发真正清空，swap 标志置位
  const auto cleared = core.update(4.0, {}, ego_at(0, 0, 0, 15), straight_lane());
  EXPECT_EQ(cleared.primary_lead_id, -1);
  EXPECT_TRUE(cleared.lead_swapped);
}

// === Commit 5: sticky 主前车选择 + 圆弧横向预测 + 候选抢占 =======================

TEST(ObjectTracker, TwoNearLeadsNoChatter) {
  // 两辆前车距离相近、不停地切换"瞬时最近"——sticky 选择必须稳定在第一辆。
  ap::ObjectTrackerCore core((ap::TrackerParams()));
  // car 1 = 30m, car 2 = 28m → car 2 略近但若交替 1-2-1-2 出现，sticky 应停在 1
  ap::TrackerOutput out;
  out = core.update(0.0, {car_at(1, 30.0, 0.0, 10.0)}, ego_at(0, 0, 0, 15), straight_lane());
  EXPECT_EQ(out.primary_lead_id, 1);
  for (int i = 1; i <= 6; ++i) {
    out = core.update(i * 0.05,
                      {car_at(1, 30.0, 0.0, 10.0), car_at(2, 28.0, 0.0, 10.0)},
                      ego_at(0, 0, 0, 15), straight_lane());
    EXPECT_EQ(out.primary_lead_id, 1) << "frame " << i;
  }
}

TEST(ObjectTracker, CutInConfirmsAfter3Frames) {
  ap::ObjectTrackerCore core((ap::TrackerParams()));
  ap::TrackerOutput out;
  out = core.update(0.0, {car_at(1, 50.0, 0.0, 10.0)}, ego_at(0, 0, 0, 15), straight_lane());
  EXPECT_EQ(out.primary_lead_id, 1);
  // car 2 在 30m 处 cut-in（gap 收益 20m）
  for (int i = 1; i <= 2; ++i) {
    out = core.update(i * 0.05,
                      {car_at(1, 50.0, 0.0, 10.0), car_at(2, 30.0, 0.0, 10.0)},
                      ego_at(0, 0, 0, 15), straight_lane());
    EXPECT_EQ(out.primary_lead_id, 1) << "frame " << i << " (still confirming)";
  }
  // 第 3 帧应发生抢占
  out = core.update(0.15, {car_at(1, 50.0, 0.0, 10.0), car_at(2, 30.0, 0.0, 10.0)},
                    ego_at(0, 0, 0, 15), straight_lane());
  EXPECT_EQ(out.primary_lead_id, 2);
  EXPECT_TRUE(out.lead_swapped);
}

TEST(ObjectTracker, CutOutRetains3Frames) {
  ap::ObjectTrackerCore core((ap::TrackerParams()));
  ap::TrackerOutput out;
  out = core.update(0.0, {car_at(1, 30.0, 0.0, 10.0)}, ego_at(0, 0, 0, 15), straight_lane());
  EXPECT_EQ(out.primary_lead_id, 1);
  // car 1 消失 → retain 期内应仍报 1
  for (int i = 1; i <= 3; ++i) {
    out = core.update(i * 0.05, {}, ego_at(0, 0, 0, 15), straight_lane());
    EXPECT_EQ(out.primary_lead_id, 1) << "frame " << i;
    EXPECT_NEAR(out.primary_lead_gap_m, 30.0, 0.5);
  }
  // 第 4 帧真正清空
  out = core.update(0.20, {}, ego_at(0, 0, 0, 15), straight_lane());
  EXPECT_EQ(out.primary_lead_id, -1);
}

TEST(ObjectTracker, AdjacentLaneNotSelected) {
  ap::ObjectTrackerCore core((ap::TrackerParams()));
  // 邻道（lat=3.5 m），gap=20 m。同道（lat=0 m），gap=50 m。
  // 即便邻道更近，因 path-projection 测试 in_ego_lane 失败，永远不会被选为 primary。
  ap::TrackerOutput out;
  out = core.update(0.0,
                    {car_at(1, 50.0, 0.0, 10.0), car_at(2, 20.0, 3.5, 10.0)},
                    ego_at(0, 0, 0, 15), straight_lane());
  EXPECT_EQ(out.primary_lead_id, 1);
  // 持续 5 帧仅邻道目标在场：sticky 不会切到 car 2（in_ego_lane=false 永远不合格）
  for (int i = 1; i <= 5; ++i) {
    out = core.update(i * 0.05, {car_at(2, 20.0, 3.5, 10.0)},
                      ego_at(0, 0, 0, 15), straight_lane());
    EXPECT_NE(out.primary_lead_id, 2) << "邻道车永远不会被选 frame=" << i;
  }
}
