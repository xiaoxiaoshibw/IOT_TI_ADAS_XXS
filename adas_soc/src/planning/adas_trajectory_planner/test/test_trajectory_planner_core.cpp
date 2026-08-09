// TrajectoryPlannerCore 单测
#include <gtest/gtest.h>

#include <cmath>

#include "adas_common/geometry.hpp"
#include "adas_trajectory_planner/trajectory_planner_core.hpp"

namespace ap = adas::planning;
namespace ac = adas::common;

namespace {

ap::PlannerParams default_params() {
  ap::PlannerParams p;
  p.cruise_speed_mps = 15.0;
  return p;
}

ac::KinematicState ego_at(double x, double y, double yaw, double v) {
  ac::KinematicState s;
  s.pose = ac::Pose2d{x, y, yaw};
  s.velocity_mps = v;
  return s;
}

ac::LaneStateData centered_straight_lane() {
  ac::LaneStateData l;
  l.valid = true;
  return l;
}

}  // namespace

TEST(TrajectoryPlannerCore, InvalidLaneGivesEmpty) {
  ap::TrajectoryPlannerCore core(default_params());
  ac::LaneStateData lane;  // valid=false
  EXPECT_TRUE(core.plan(ego_at(0, 0, 0, 10), lane).empty());
}

TEST(TrajectoryPlannerCore, StraightCenteredFollowsXAxis) {
  ap::TrajectoryPlannerCore core(default_params());
  const auto traj = core.plan(ego_at(5.0, 0.0, 0.0, 10.0), centered_straight_lane());
  ASSERT_GE(traj.size(), 2u);
  EXPECT_NEAR(traj.front().x, 5.0, 1e-9);
  EXPECT_NEAR(traj.front().y, 0.0, 1e-9);
  for (const auto& p : traj) {
    EXPECT_NEAR(p.y, 0.0, 1e-9);
    EXPECT_NEAR(p.yaw, 0.0, 1e-9);
  }
  // 速度剖面从当前 10 爬向巡航 15，且不超
  EXPECT_NEAR(traj.front().velocity_mps, 10.0, 1e-9);
  EXPECT_LE(traj.back().velocity_mps, 15.0 + 1e-9);
  EXPECT_GT(traj.back().velocity_mps, 10.0);
}

TEST(TrajectoryPlannerCore, OffsetEgoTrajectoryOnCenterline) {
  ap::TrajectoryPlannerCore core(default_params());
  // 自车在中心线左侧 0.5m（lateral_offset=+0.5），航向平行
  ac::LaneStateData lane = centered_straight_lane();
  lane.lateral_offset = 0.5;
  const auto traj = core.plan(ego_at(0.0, 0.5, 0.0, 10.0), lane);
  ASSERT_FALSE(traj.empty());
  // 轨迹应落在中心线（y=0）而不是自车处
  EXPECT_NEAR(traj.front().y, 0.0, 1e-9);
}

TEST(TrajectoryPlannerCore, CurvatureLimitsSpeed) {
  ap::PlannerParams p = default_params();
  p.max_lat_accel_mps2 = 2.0;
  ap::TrajectoryPlannerCore core(p);
  ac::LaneStateData lane = centered_straight_lane();
  lane.curvature = 0.02;  // R=50m → v_max = sqrt(2/0.02) = 10
  const auto traj = core.plan(ego_at(0, 0, 0, 15.0), lane);
  ASSERT_FALSE(traj.empty());
  // 剖面应向曲率限速收敛（末端不超限速）
  EXPECT_LE(traj.back().velocity_mps, 10.0 + 1e-6);
  // 弧线航向单调左偏
  EXPECT_GT(traj.back().yaw, traj.front().yaw);
}

TEST(TrajectoryPlannerCore, HeadingErrorReconstruction) {
  ap::TrajectoryPlannerCore core(default_params());
  // 自车航向 +0.1rad（相对路径左偏），路径本身沿 +x
  ac::LaneStateData lane = centered_straight_lane();
  lane.heading_error = 0.1;
  const auto traj = core.plan(ego_at(0, 0, 0.1, 10.0), lane);
  ASSERT_FALSE(traj.empty());
  // 重建的参考线航向应为 0（自车航向 - 航向误差）
  EXPECT_NEAR(traj.front().yaw, 0.0, 1e-9);
}

TEST(TrajectoryPlannerCore, StationaryLeadProfileStopsAtStandstillGap) {
  ap::PlannerParams p = default_params();
  p.follow_standstill_m = 4.0;
  ap::TrajectoryPlannerCore core(p);
  ap::LeadInfo lead;
  lead.present = true;
  lead.gap_m = 40.0;
  lead.speed_mps = 0.0;
  const auto traj = core.plan(ego_at(0, 0, 0, 10.0), centered_straight_lane(), lead);
  ASSERT_FALSE(traj.empty());
  // 剖面应在 gap - standstill = 36m 附近降到 0，其后保持 0
  for (const auto& pt : traj) {
    const double s = pt.x;  // 直道沿 +x
    if (s > 36.5) {
      EXPECT_NEAR(pt.velocity_mps, 0.0, 1e-6) << "s=" << s;
    }
  }
  // 起点仍是当前车速
  EXPECT_NEAR(traj.front().velocity_mps, 10.0, 1e-9);
}

TEST(TrajectoryPlannerCore, MovingLeadCapsAtLeadSpeed) {
  ap::TrajectoryPlannerCore core(default_params());
  ap::LeadInfo lead;
  lead.present = true;
  lead.gap_m = 20.0;  // 已接近稳态距离
  lead.speed_mps = 10.0;
  const auto traj = core.plan(ego_at(0, 0, 0, 15.0), centered_straight_lane(), lead);
  ASSERT_FALSE(traj.empty());
  // 剖面末端应收敛到前车速度附近而非巡航 15
  EXPECT_LE(traj.back().velocity_mps, 10.5);
}

TEST(TrajectoryPlannerCore, CruiseOverrideZeroMakesStopProfile) {
  ap::TrajectoryPlannerCore core(default_params());
  const auto traj =
      core.plan(ego_at(0, 0, 0, 10.0), centered_straight_lane(), ap::LeadInfo(), 0.0);
  ASSERT_FALSE(traj.empty());
  EXPECT_NEAR(traj.back().velocity_mps, 0.0, 1e-6);  // 停车剖面
}

TEST(TrajectoryPlannerCore, LaneChangeShiftsToTargetLane) {
  ap::TrajectoryPlannerCore core(default_params());
  // target_lane=-1（左邻道，中心 +3.5）：轨迹起点在当前横向，末端到目标车道中心
  const auto traj =
      core.plan(ego_at(0, 0, 0, 15.0), centered_straight_lane(), ap::LeadInfo(), -1.0, -1);
  ASSERT_FALSE(traj.empty());
  EXPECT_NEAR(traj.front().y, 0.0, 0.05);   // 从自车当前横向出发
  EXPECT_NEAR(traj.back().y, 3.5, 0.05);    // 末端在左车道中心
  // 横移单调且平滑（无回折）
  for (std::size_t i = 1; i < traj.size(); ++i) {
    EXPECT_GE(traj[i].y, traj[i - 1].y - 1e-6);
  }
}

TEST(TrajectoryPlannerCore, SteadyLeftLaneKeepsConstantShift) {
  ap::TrajectoryPlannerCore core(default_params());
  // 自车已在左车道中心（lateral_offset=3.5）继续 target_lane=-1 → 常量偏移参考线
  ac::LaneStateData lane = centered_straight_lane();
  lane.lateral_offset = 3.5;
  const auto traj =
      core.plan(ego_at(0, 3.5, 0, 15.0), lane, ap::LeadInfo(), -1.0, -1);
  ASSERT_FALSE(traj.empty());
  for (const auto& p : traj) {
    EXPECT_NEAR(p.y, 3.5, 0.05);
  }
}

TEST(TrajectoryPlannerCore, LengthRespectsBounds) {
  ap::PlannerParams p = default_params();
  p.min_length_m = 30.0;
  p.max_length_m = 120.0;
  ap::TrajectoryPlannerCore core(p);
  // 静止：仍应有 min_length 轨迹
  const auto t0 = core.plan(ego_at(0, 0, 0, 0.0), centered_straight_lane());
  ASSERT_FALSE(t0.empty());
  const double len0 = ac::distance2d(t0.front().x, t0.front().y, t0.back().x, t0.back().y);
  EXPECT_GE(len0, 29.0);
  // 高速：不超 max_length
  const auto t1 = core.plan(ego_at(0, 0, 0, 40.0), centered_straight_lane());
  const double len1 = ac::distance2d(t1.front().x, t1.front().y, t1.back().x, t1.back().y);
  EXPECT_LE(len1, 121.0);
}

// === Commit 2: lead-cap tests for plan_global_route ============================

namespace {
// 简单直道，全长 120 m，刚好够 max_length_m 截完。
ac::Trajectory straight_route(std::size_t n) {
  ac::Trajectory route;
  route.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    route.push_back({static_cast<double>(i), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  }
  return route;
}
}  // namespace

TEST(TrajectoryPlannerCore, GlobalRouteSlowsForLead) {
  ap::PlannerParams p = default_params();
  ap::TrajectoryPlannerCore core(p);
  ap::LeadInfo lead;
  lead.present = true;
  lead.gap_m = 30.0;
  lead.speed_mps = 4.0;
  const auto traj =
      core.plan_global_route(ego_at(0, 0, 0, 15.0), straight_route(120),
                             /*cruise_speed_mps*/ 15.0,
                             /*goal_stop_distance_m*/ 1.5,
                             /*stop_at_route_end*/ false, lead);
  ASSERT_EQ(traj.size(), 120u);
  // 路线中段（接近跟车收敛区）的速度应不大于 lead 速度 + 少量裕度；
  // 起点保留 15 m/s 起步，最后 30 m 之前已经压到跟车速度附近。
  const std::size_t near_lead = std::min<std::size_t>(traj.size() - 1, 30);
  EXPECT_LE(traj[near_lead].velocity_mps, 5.0);
  for (const auto& pt : traj) {
    EXPECT_TRUE(std::isfinite(pt.velocity_mps));
    EXPECT_LE(pt.velocity_mps, lead.speed_mps + 0.05);
  }
}

TEST(TrajectoryPlannerCore, GlobalRouteLeadAccelerateReleasesToCruise) {
  ap::PlannerParams p = default_params();
  ap::TrajectoryPlannerCore core(p);
  // 仅有 lead 前段：直接用 LeadInfo().present=false 调用应等价于无前车。
  // 本测试验证：当 lead 在 10 m 处"消失"（取下界 + 远端上限），
  // 即我用一个 lead 在前段给限速，预期速度上限相应降低，移除后立刻恢复。
  // 这里用两段比较：lead.cap=12 → 限速；lead.cap=+∞ → 限速=巡航。
  ac::Trajectory route = straight_route(120);
  ap::LeadInfo lead_in;
  lead_in.present = true;
  lead_in.gap_m = 100.0;
  lead_in.speed_mps = 12.0;
  const auto with_lead =
      core.plan_global_route(ego_at(0, 0, 0, 15.0), route, 15.0, 1.5, false, lead_in);
  const auto without_lead =
      core.plan_global_route(ego_at(0, 0, 0, 15.0), route, 15.0, 1.5, false,
                             ap::LeadInfo());
  ASSERT_EQ(with_lead.size(), without_lead.size());
  // 有 lead 的轨迹应在 start 段就低于无 lead 的轨迹（lead cap 生效）。
  // 具体而言：在 s≈[gap_actual - (T_gap * v_lead + standstill)] 之前应有差距。
  EXPECT_LT(with_lead[5].velocity_mps, without_lead[5].velocity_mps);
}

TEST(TrajectoryPlannerCore, GlobalRouteMatchesBaselineWithoutLead) {
  ap::PlannerParams p = default_params();
  ap::TrajectoryPlannerCore core(p);
  const ac::Trajectory route = straight_route(120);
  // 两者路径相同 → 输出逐字段相等。
  const auto traj_a =
      core.plan_global_route(ego_at(0, 0, 0, 12.0), route, 15.0, 1.5, true);
  const auto traj_b =
      core.plan_global_route(ego_at(0, 0, 0, 12.0), route, 15.0, 1.5, true,
                             ap::LeadInfo());
  ASSERT_EQ(traj_a.size(), traj_b.size());
  for (std::size_t i = 0; i < traj_a.size(); ++i) {
    EXPECT_NEAR(traj_a[i].velocity_mps, traj_b[i].velocity_mps, 1e-9)
        << "i=" << i;
    EXPECT_NEAR(traj_a[i].acceleration_mps2, traj_b[i].acceleration_mps2, 1e-9)
        << "i=" << i;
  }
}

TEST(TrajectoryPlannerCore, GlobalRouteTerminalStopBeatsLead) {
  ap::PlannerParams p = default_params();
  ap::TrajectoryPlannerCore core(p);
  ap::LeadInfo lead;
  lead.present = true;
  lead.gap_m = 30.0;
  lead.speed_mps = 4.0;
  // stop_at_route_end = true, route 总长 60 m，目标 stop_distance=1.5：
  // 末端必须为 0（停车 cap 胜出）。
  const auto traj =
      core.plan_global_route(ego_at(0, 0, 0, 15.0), straight_route(60),
                             15.0, 1.5, true, lead);
  ASSERT_FALSE(traj.empty());
  for (const auto& pt : traj) {
    EXPECT_LE(pt.velocity_mps, lead.speed_mps + 0.05);
  }
  // 末 5 个点应处于终端停车状态
  for (std::size_t i = traj.size() - 5; i < traj.size(); ++i) {
    EXPECT_LT(traj[i].velocity_mps, 0.1)
        << "终端停车 cap 应压过 lead cap: i=" << i << " v=" << traj[i].velocity_mps;
  }
}
