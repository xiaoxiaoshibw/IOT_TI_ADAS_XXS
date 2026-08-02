// M3 AEB 全链路离线冒烟（无 ROS）：三个场景串行
//   A 横穿行人：AEB 预测冲突触发 → 停车避让 → 行人通过后恢复巡航
//   B 无误触发回归：完整 ACC 场景（跟车/刹停/重起步）AEB 全程 0 次 EMERGENCY
//   C 前车急刹 6m/s²：超出 ACC 舒适制动权限 → AEB 补位，无碰撞
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "adas_aeb/aeb_core.hpp"
#include "adas_behavior_planner/behavior_core.hpp"
#include "adas_common/geometry.hpp"
#include "adas_command_gate/gate_core.hpp"
#include "adas_object_tracker/object_tracker_core.hpp"
#include "adas_sim_vehicle/sim_vehicle_core.hpp"
#include "adas_trajectory_follower/pid_longitudinal.hpp"
#include "adas_trajectory_follower/pure_pursuit_lateral.hpp"
#include "adas_trajectory_planner/trajectory_planner_core.hpp"
#include "adas_vehicle_interface/sim_vehicle_interface.hpp"

namespace ac = adas::common;
namespace as = adas::sim;
namespace ap = adas::planning;
namespace ape = adas::perception;
namespace ct = adas::control;
namespace av = adas::vehicle;

// 单场景运行器：完整链路（sim → tracker/aeb → behavior → planner → follower → gate → 执行）
struct LoopResult {
  int aeb_emergency_transitions{0};
  double min_obj_dist{1e9};       // 自车与任意目标最小距离
  double final_station{0.0};
  double final_speed{0.0};
  double steady_time_gap{-1.0};   // 场景 B 用
  bool ego_stopped_during_run{false};
};

static LoopResult run_loop(as::SimVehicleCore& sim, double duration_s,
                           double cruise_mps = 15.0) {
  const double dt = 0.02;
  ape::ObjectTrackerCore tracker((ape::TrackerParams()));
  ap::BehaviorParams bp;
  bp.overtake_enabled = false;  // AEB 场景语义：直行遇险制动，不绕行
  ap::BehaviorCore behavior(bp);
  ap::PlannerParams pp;
  pp.cruise_speed_mps = cruise_mps;
  ap::TrajectoryPlannerCore planner(pp);
  ct::PurePursuitLateral lateral((ct::PurePursuitParams()));
  ct::PidLongitudinal longitudinal((ct::PidLongitudinalParams()));
  ct::GateCore gate((ct::GateParams()));
  ct::AebCore aeb((ct::AebParams()));
  av::SimVehicleInterface vehicle_if((av::SimVehicleInterfaceParams()));

  ac::Trajectory traj;
  ape::TrackerOutput tracked;
  ap::BehaviorOutput behav;
  behav.target_speed_mps = cruise_mps;
  ct::AebResult aeb_res;
  bool last_emergency = false;

  LoopResult res;
  double sum_tg = 0.0;
  int tg_n = 0;

  const int ticks = static_cast<int>(duration_s / dt);
  for (int i = 0; i < ticks; ++i) {
    const double t = i * dt;
    const auto ego = sim.state();
    const auto lane = sim.lane_state();
    const auto lead = sim.lead_state();
    const auto ped = sim.pedestrian_state();

    // 原始目标集合
    std::vector<ape::RawObject> raw;
    std::vector<ct::AebObject> aeb_objs;
    if (lead.present) {
      ape::RawObject o;
      o.id = 1;
      o.classification = 1;
      o.x = lead.x;
      o.y = lead.y;
      o.yaw = lead.yaw;
      o.v_mps = lead.v_mps;
      raw.push_back(o);
      aeb_objs.push_back(ct::AebObject{lead.x, lead.y, lead.yaw, lead.v_mps, 1});
      res.min_obj_dist =
          std::min(res.min_obj_dist, ac::distance2d(ego.pose.x, ego.pose.y, lead.x, lead.y));
    }
    if (ped.present) {
      ape::RawObject o;
      o.id = 2;
      o.classification = 3;
      o.x = ped.x;
      o.y = ped.y;
      o.yaw = ped.yaw;
      o.v_mps = ped.v_mps;
      raw.push_back(o);
      aeb_objs.push_back(ct::AebObject{ped.x, ped.y, ped.yaw, ped.v_mps, 3});
      res.min_obj_dist =
          std::min(res.min_obj_dist, ac::distance2d(ego.pose.x, ego.pose.y, ped.x, ped.y));
    }

    // 20Hz：跟踪 + AEB
    if (i % 2 == 0) {
      tracked = tracker.update(t, raw, ego, lane);
      aeb_res = aeb.update(ego, aeb_objs);
      if (aeb_res.emergency_active && !last_emergency) {
        ++res.aeb_emergency_transitions;
        std::printf("  t=%5.2fs AEB 触发 ttc=%.2fs (%s)\n", t, aeb_res.ttc_s,
                    aeb_res.reason.c_str());
      }
      last_emergency = aeb_res.emergency_active;
    }
    // 10Hz：行为 + 规划
    if (i % 5 == 0 && lane.valid) {
      ap::BehaviorInput bin;
      bin.primary_lead_id = tracked.primary_lead_id;
      bin.lead_gap_m = tracked.primary_lead_gap_m;
      bin.lead_speed_mps = tracked.primary_lead_speed_mps;
      bin.ego_speed_mps = ego.velocity_mps;
      bin.ego_lateral_m = lane.lateral_offset;
      behav = behavior.update(bin);
      ap::LeadInfo li;
      if (tracked.primary_lead_id >= 0) {
        li.present = true;
        li.gap_m = tracked.primary_lead_gap_m;
        li.speed_mps = tracked.primary_lead_speed_mps;
      }
      traj = planner.plan(ego, lane, li, behav.target_speed_mps);
    }

    // 50Hz：跟踪控制 + 门控（AEB 紧急源合流）+ 执行
    ct::GateInputs gin;
    gin.now_s = t;
    gin.dt = dt;
    gin.ego_speed_mps = ego.velocity_mps;
    if (traj.size() >= 2) {
      ct::ControlInput cin;
      cin.trajectory = &traj;
      cin.state = ego;
      cin.steering_angle_rad = sim.steering_angle_rad();
      cin.dt = dt;
      gin.follower_received = true;
      gin.follower_stamp_s = t;
      gin.follower_cmd.lateral = lateral.run(cin);
      gin.follower_cmd.longitudinal = longitudinal.run(cin);
    }
    if (aeb_res.emergency_active) {
      gin.aeb_emergency = true;
      gin.aeb_cmd.longitudinal.acceleration_mps2 = aeb_res.brake_accel_mps2;
    }
    const auto d = gate.update(gin);
    sim.step(vehicle_if.apply(d.cmd, ego), dt);

    if (ego.velocity_mps < 0.05 && t > 5.0) {
      res.ego_stopped_during_run = true;
    }
    // 场景 B 稳态时距窗（25~35s）
    if (t >= 25.0 && t < 35.0 && lead.present && ego.velocity_mps > 1.0) {
      sum_tg += (lead.x - ego.pose.x) / ego.velocity_mps;
      ++tg_n;
    }
  }
  res.final_station = sim.state().pose.x;
  res.final_speed = sim.state().velocity_mps;
  if (tg_n > 0) {
    res.steady_time_gap = sum_tg / tg_n;
  }
  return res;
}

int main() {
  // ── 场景 A：横穿行人 ──
  std::puts("场景 A：横穿行人");
  {
    as::VehicleParams vp;
    as::SimVehicleCore sim(vp, {{500.0, 0.0}}, 3.5);
    sim.set_initial_state(0.0, 0.0, 14.0);  // 接近巡航速度起步，尽快到达
    as::PedestrianScript ped;
    ped.enabled = true;
    ped.station_m = 150.0;
    ped.trigger_ego_gap_m = 35.0;
    ped.start_lateral_m = -5.0;
    ped.end_lateral_m = 5.0;
    ped.speed_mps = 1.5;
    sim.set_pedestrian_script(ped);
    const auto r = run_loop(sim, 40.0);
    std::printf("  min_dist=%.2fm 触发次数=%d 终点=%.1fm 停过车=%d\n", r.min_obj_dist,
                r.aeb_emergency_transitions, r.final_station, r.ego_stopped_during_run);
    assert(r.aeb_emergency_transitions >= 1);   // AEB 触发过
    assert(r.min_obj_dist > 2.0);               // 无碰撞（含裕量）
    assert(r.final_station > 160.0);            // 行人通过后恢复巡航驶过横穿点
  }

  // ── 场景 B：完整 ACC 场景 AEB 零误触发 ──
  std::puts("场景 B：ACC 跟车/刹停/重起步（AEB 应全程静默）");
  {
    as::VehicleParams vp;
    as::SimVehicleCore sim(vp, {{1000.0, 0.0}}, 3.5);
    sim.set_initial_state(0.0, 0.0, 0.0);
    as::LeadScript script;
    script.enabled = true;
    script.initial_station_m = 60.0;
    script.initial_speed_mps = 10.0;
    script.accel_mps2 = 3.0;
    script.events = {{35.0, 0.0}, {55.0, 10.0}};
    sim.set_lead_script(script);
    const auto r = run_loop(sim, 80.0);
    std::printf("  稳态时距=%.2fs min_dist=%.2fm AEB触发=%d\n", r.steady_time_gap,
                r.min_obj_dist, r.aeb_emergency_transitions);
    assert(r.aeb_emergency_transitions == 0);   // 零误触发
    assert(r.steady_time_gap >= 1.3 && r.steady_time_gap <= 1.7);  // ACC 行为不受 AEB 影响
    assert(r.min_obj_dist > 2.0);
  }

  // ── 场景 C：前车 6m/s² 急刹（超 ACC 舒适制动权限）──
  std::puts("场景 C：前车急刹 6m/s²（AEB 应补位）");
  {
    as::VehicleParams vp;
    as::SimVehicleCore sim(vp, {{600.0, 0.0}}, 3.5);
    sim.set_initial_state(0.0, 0.0, 10.0);  // 以跟车速度起步
    as::LeadScript script;
    script.enabled = true;
    script.initial_station_m = 20.0;        // 起步即接近稳态间距
    script.initial_speed_mps = 10.0;
    script.accel_mps2 = 6.0;                // 急刹能力
    script.events = {{20.0, 0.0}};          // t=20s 急刹到停
    sim.set_lead_script(script);
    const auto r = run_loop(sim, 40.0);
    std::printf("  min_dist=%.2fm AEB触发=%d 末速=%.2f\n", r.min_obj_dist,
                r.aeb_emergency_transitions, r.final_speed);
    assert(r.min_obj_dist > 1.5);           // 无碰撞
    assert(r.ego_stopped_during_run);       // 自车确实停了下来
  }

  std::puts("M3 AEB closed-loop smoke: ALL PASS");
  return 0;
}
