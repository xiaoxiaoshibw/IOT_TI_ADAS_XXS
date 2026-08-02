// M2 ACC 全链路离线闭环冒烟（无 ROS）：
//   SimVehicleCore(+脚本前车) → ObjectTrackerCore(20Hz 等效)
//   → BehaviorCore(10Hz) → TrajectoryPlannerCore(10Hz, IDM 跟车剖面)
//   → PurePursuit+PID(50Hz) → GateCore → SimVehicleInterface → step
// 场景：1000m 直道；前车 60m 处 10m/s；t=35s 刹停（3m/s²）；t=55s 重新起步到 10。
// 断言即 M2 验收标准：稳态时距 1.5±0.2s / 全程无碰撞 / 停稳距离合理 / 跟随起步。
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "adas_behavior_planner/behavior_core.hpp"
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

int main() {
  const double dt = 0.02;  // 50Hz
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

  ape::ObjectTrackerCore tracker((ape::TrackerParams()));
  ap::BehaviorParams bp;
  bp.overtake_enabled = false;  // ACC 场景语义：纯跟车/刹停/重起步
  ap::BehaviorCore behavior(bp);
  ap::PlannerParams pp;
  pp.cruise_speed_mps = 15.0;
  ap::TrajectoryPlannerCore planner(pp);
  ct::PurePursuitLateral lateral((ct::PurePursuitParams()));
  ct::PidLongitudinal longitudinal((ct::PidLongitudinalParams()));
  ct::GateCore gate((ct::GateParams()));
  av::SimVehicleInterface vehicle_if((av::SimVehicleInterfaceParams()));

  ac::Trajectory traj;
  ape::TrackerOutput tracked;
  ap::BehaviorOutput behav;
  behav.target_speed_mps = pp.cruise_speed_mps;

  // 指标
  double min_gap = 1e9;
  double sum_tg = 0.0;
  int tg_n = 0;
  double v_max_stopped_phase = 0.0;
  double standstill_gap = -1.0;
  double v_at_70 = 0.0;

  const int ticks = static_cast<int>(80.0 / dt);
  for (int i = 0; i < ticks; ++i) {
    const double t = i * dt;
    const auto ego = sim.state();
    const auto lane = sim.lane_state();
    const auto lead = sim.lead_state();

    // 20Hz 目标跟踪（含选举）
    if (i % 2 == 0) {
      std::vector<ape::RawObject> raw;
      if (lead.present) {
        ape::RawObject o;
        o.id = 1;
        o.classification = 1;
        o.x = lead.x;
        o.y = lead.y;
        o.yaw = lead.yaw;
        o.v_mps = lead.v_mps;
        raw.push_back(o);
      }
      tracked = tracker.update(t, raw, ego, lane);
    }
    // 10Hz 行为 + 规划
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

    // 50Hz 跟踪 + 门控 + 执行
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
    const auto d = gate.update(gin);
    sim.step(vehicle_if.apply(d.cmd, ego), dt);

    // ── 指标采集 ──
    if (lead.present) {
      const double gap = lead.station_m > 0.0
                             ? (lead.x - ego.pose.x)  // 直道：沿 +x
                             : 0.0;
      min_gap = std::min(min_gap, gap);
      if (t >= 25.0 && t < 35.0 && ego.velocity_mps > 1.0) {
        sum_tg += gap / ego.velocity_mps;
        ++tg_n;
      }
      if (t >= 45.0 && t < 55.0) {
        v_max_stopped_phase = std::max(v_max_stopped_phase, ego.velocity_mps);
        standstill_gap = gap;
      }
    }
    if (std::fabs(t - 70.0) < dt / 2) {
      v_at_70 = ego.velocity_mps;
    }
  }

  const double avg_tg = tg_n > 0 ? sum_tg / tg_n : -1.0;
  std::printf("指标: 稳态时距=%.2fs  min_gap=%.2fm  停稳gap=%.2fm  停稳期v_max=%.3fm/s  v(70s)=%.2fm/s\n",
              avg_tg, min_gap, standstill_gap, v_max_stopped_phase, v_at_70);

  // M2 验收断言
  assert(avg_tg >= 1.3 && avg_tg <= 1.7);   // 稳态时距 1.5±0.2s
  assert(min_gap > 2.0);                    // 全程无碰撞（含裕量）
  assert(v_max_stopped_phase < 0.1);        // 前车停 → 自车停稳无蠕动
  assert(standstill_gap >= 2.5 && standstill_gap <= 6.5);  // 停在合理静距
  assert(v_at_70 > 5.0);                    // 前车起步 → 自车跟随起步
  std::puts("M2 ACC closed-loop smoke: ALL PASS");
  return 0;
}
