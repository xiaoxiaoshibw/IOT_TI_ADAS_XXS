// AebCore 单测
#include <gtest/gtest.h>

#include <cmath>

#include "adas_aeb/aeb_core.hpp"

namespace ct = adas::control;
namespace ac = adas::common;

namespace {

ac::KinematicState ego_straight(double v) {
  ac::KinematicState s;
  s.velocity_mps = v;
  return s;  // 原点朝 +x
}

ct::AebObject stationary_at(double x, double y, uint8_t cls = 1) {
  ct::AebObject o;
  o.x = x;
  o.y = y;
  o.classification = cls;
  return o;
}

// 连续喂 n 帧同一输入
ct::AebResult feed(ct::AebCore& core, const ac::KinematicState& ego,
                   const std::vector<ct::AebObject>& objs, int n) {
  ct::AebResult r;
  for (int i = 0; i < n; ++i) {
    r = core.update(ego, objs);
  }
  return r;
}

}  // namespace

TEST(AebCore, StationaryObstacleInPathTriggersAfterConfirmFrames) {
  ct::AebCore core((ct::AebParams()));
  // 15m/s，前方 20m 静止车：ttc≈1.3s < 1.8 → 危险
  const auto ego = ego_straight(15.0);
  const std::vector<ct::AebObject> objs = {stationary_at(20.0, 0.0)};
  // 前 2 帧确认窗内（trigger_frames=3）
  auto r = feed(core, ego, objs, 2);
  EXPECT_FALSE(r.emergency_active);
  EXPECT_EQ(r.state, ct::AebState::kWarning);
  // 第 3 帧触发
  r = core.update(ego, objs);
  EXPECT_TRUE(r.emergency_active);
  EXPECT_EQ(r.state, ct::AebState::kEmergency);
  EXPECT_LT(r.brake_accel_mps2, -7.0);
  EXPECT_GT(r.required_decel_mps2, 4.0);
}

TEST(AebCore, MatchedSpeedLeadNoTrigger) {
  ct::AebCore core((ct::AebParams()));
  const auto ego = ego_straight(15.0);
  ct::AebObject lead = stationary_at(15.0, 0.0);
  lead.v_mps = 15.0;  // 同速同向 → 距离恒定
  const auto r = feed(core, ego, {lead}, 50);
  EXPECT_FALSE(r.emergency_active);
  EXPECT_EQ(r.state, ct::AebState::kMonitoring);
}

TEST(AebCore, SlowApproachBeyondThresholdNoTrigger) {
  ct::AebCore core((ct::AebParams()));
  const auto ego = ego_straight(15.0);
  ct::AebObject lead = stationary_at(40.0, 0.0);
  lead.v_mps = 10.0;  // 接近速度 5m/s，冲突 ≈(40-1.8)/5≈7.6s > horizon
  const auto r = feed(core, ego, {lead}, 50);
  EXPECT_FALSE(r.emergency_active);
}

TEST(AebCore, CrossingPedestrianPredictedConflict) {
  ct::AebCore core((ct::AebParams()));
  const auto ego = ego_straight(15.0);
  // 行人在 30m 前、右侧 4m，以 2m/s 向左横穿（朝 +y）：
  // 到达走廊边缘 (|y|<1.8) 约 t=1.1s；自车到达 x=30 约 t=2.0s → 冲突在 ~2s
  ct::AebObject ped;
  ped.x = 30.0;
  ped.y = -4.0;
  ped.yaw = 3.14159265 / 2.0;  // 朝 +y
  ped.v_mps = 2.0;
  ped.classification = 3;  // PEDESTRIAN
  const auto r = feed(core, ego, {ped}, 3);
  // 行人阈值 2.5s：冲突 ~2.0s < 2.5 → 触发（若是 car 阈值 1.8 则不会）
  EXPECT_TRUE(r.emergency_active);
  EXPECT_EQ(r.reason, "pedestrian_in_path");
}

TEST(AebCore, AdjacentLaneObjectNoTrigger) {
  ct::AebCore core((ct::AebParams()));
  const auto ego = ego_straight(15.0);
  // 邻道静止车（横向 3.5m > 走廊 1.8m）
  const auto r = feed(core, ego, {stationary_at(20.0, 3.5)}, 50);
  EXPECT_FALSE(r.emergency_active);
}

TEST(AebCore, LowSpeedInactive) {
  ct::AebCore core((ct::AebParams()));
  const auto r = feed(core, ego_straight(0.5), {stationary_at(2.0, 0.0)}, 10);
  EXPECT_EQ(r.state, ct::AebState::kInactive);
  EXPECT_FALSE(r.emergency_active);
}

TEST(AebCore, ReleasesAfterClearFrames) {
  ct::AebParams p;
  p.release_frames = 5;
  ct::AebCore core(p);
  const auto ego = ego_straight(15.0);
  // 触发
  auto r = feed(core, ego, {stationary_at(20.0, 0.0)}, 3);
  ASSERT_TRUE(r.emergency_active);
  // 障碍消失：4 帧内仍保持（release_frames=5）
  r = feed(core, ego, {}, 4);
  EXPECT_TRUE(r.emergency_active);
  // 第 5 帧释放
  r = core.update(ego, {});
  EXPECT_FALSE(r.emergency_active);
}

TEST(AebCore, CurvedPathFollowsYawRate) {
  ct::AebCore core((ct::AebParams()));
  // 左转中（yaw_rate>0）：正前方直线上的目标实际不在弧线走廊内
  ac::KinematicState ego;
  ego.velocity_mps = 15.0;
  ego.yaw_rate_rps = 0.5;  // 半径 30m 急左转
  const auto r = feed(core, ego, {stationary_at(28.0, 0.0)}, 50);
  EXPECT_FALSE(r.emergency_active);
}
