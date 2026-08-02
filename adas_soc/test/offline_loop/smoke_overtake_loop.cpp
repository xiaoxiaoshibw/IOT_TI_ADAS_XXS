// M4 超车全链路离线冒烟（无 ROS）：两个场景串行
//   A 慢车超越：FOLLOW→WAIT→ACTIVE(左变道)→超越→RETURN→回线，全程无碰撞/AEB 静默
//   B 中途危险：早期 ACTIVE 阶段邻道突现车辆 → 中止回跟车；邻道车驶远后重试并完成超越
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "adas_aeb/aeb_core.hpp"
#include "adas_behavior_planner/behavior_core.hpp"
#include "adas_command_gate/gate_core.hpp"
#include "adas_common/geometry.hpp"
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

struct LoopResult {
  double min_obj_dist{1e9};
  double max_abs_lat{0.0};
  double final_lat{0.0};
  int aeb_triggers{0};
  bool reached_active{false};
  bool reached_return{false};
  int abort_count{0};       // ACTIVE → FOLLOW_LEAD 次数
  double ego_final_x{0.0};
  double lead_final_x{0.0};
};

static LoopResult run_loop(as::SimVehicleCore& sim, double duration_s) {
  const double dt = 0.02;
  ape::ObjectTrackerCore tracker((ape::TrackerParams()));
  ap::BehaviorParams bp;  // 缺省超车参数（slow_ratio 0.6 等）
  ap::BehaviorCore behavior(bp);
  ap::PlannerParams pp;
  ap::TrajectoryPlannerCore planner(pp);
  ct::PurePursuitLateral lateral((ct::PurePursuitParams()));
  ct::PidLongitudinal longitudinal((ct::PidLongitudinalParams()));
  ct::GateCore gate((ct::GateParams()));
  ct::AebCore aeb((ct::AebParams()));
  av::SimVehicleInterface vehicle_if((av::SimVehicleInterfaceParams()));

  ac::Trajectory traj;
  ape::TrackerOutput tracked;
  ap::BehaviorOutput behav;
  behav.target_speed_mps = pp.cruise_speed_mps;
  ct::AebResult aeb_res;
  bool last_emergency = false;
  ap::BehaviorKind last_state = ap::BehaviorKind::kLaneFollow;

  LoopResult res;
  const int ticks = static_cast<int>(duration_s / dt);
  for (int i = 0; i < ticks; ++i) {
    const double t = i * dt;
    const auto ego = sim.state();
    const auto lane = sim.lane_state();
    const auto lead = sim.lead_state();
    const auto adj = sim.adjacent_car_state();

    std::vector<ape::RawObject> raw;
    std::vector<ct::AebObject> aeb_objs;
    const auto add_car = [&](uint32_t id, const as::LeadState& c) {
      raw.push_back(ape::RawObject{id, 1, c.x, c.y, c.yaw, c.v_mps});
      aeb_objs.push_back(ct::AebObject{c.x, c.y, c.yaw, c.v_mps, 1});
      res.min_obj_dist =
          std::min(res.min_obj_dist, ac::distance2d(ego.pose.x, ego.pose.y, c.x, c.y));
    };
    if (lead.present) {
      add_car(1, lead);
    }
    if (adj.present) {
      add_car(3, adj);
    }

    if (i % 2 == 0) {
      tracked = tracker.update(t, raw, ego, lane);
      aeb_res = aeb.update(ego, aeb_objs);
      if (aeb_res.emergency_active && !last_emergency) {
        ++res.aeb_triggers;
      }
      last_emergency = aeb_res.emergency_active;
    }
    if (i % 5 == 0 && lane.valid) {
      ap::BehaviorInput bin;
      bin.primary_lead_id = tracked.primary_lead_id;
      bin.lead_gap_m = tracked.primary_lead_gap_m;
      bin.lead_speed_mps = tracked.primary_lead_speed_mps;
      bin.ego_speed_mps = ego.velocity_mps;
      bin.ego_lateral_m = lane.lateral_offset;
      for (const auto& o : tracked.objects) {
        bin.objects.push_back(
            ap::ObjectLite{o.raw.id, o.gap_m, o.lat_m, o.v_filtered_mps});
      }
      behav = behavior.update(bin);
      if (behav.state != last_state) {
        std::printf("  t=%5.2fs 行为 %d→%d lat=%.2f\n", t, static_cast<int>(last_state),
                    static_cast<int>(behav.state), lane.lateral_offset);
        if (behav.state == ap::BehaviorKind::kOvertakeActive) {
          res.reached_active = true;
        }
        if (behav.state == ap::BehaviorKind::kOvertakeReturn) {
          res.reached_return = true;
        }
        if (last_state == ap::BehaviorKind::kOvertakeActive &&
            behav.state == ap::BehaviorKind::kFollowLead) {
          ++res.abort_count;
        }
        last_state = behav.state;
      }
      ap::LeadInfo li;
      if (tracked.primary_lead_id >= 0) {
        li.present = true;
        li.gap_m = tracked.primary_lead_gap_m;
        li.speed_mps = tracked.primary_lead_speed_mps;
      }
      traj = planner.plan(ego, lane, li, behav.target_speed_mps, behav.target_lane);
    }

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

    if (lane.valid) {
      res.max_abs_lat = std::max(res.max_abs_lat, std::fabs(lane.lateral_offset));
      res.final_lat = lane.lateral_offset;
    }
    res.ego_final_x = ego.pose.x;
    if (lead.present) {
      res.lead_final_x = lead.x;
    }
  }
  return res;
}

int main() {
  // ── 场景 A：慢车超越 ──
  std::puts("场景 A：慢车超越（前车 6m/s，巡航 15）");
  {
    as::VehicleParams vp;
    as::SimVehicleCore sim(vp, {{1200.0, 0.0}}, 3.5);
    sim.set_initial_state(0.0, 0.0, 10.0);
    as::LeadScript script;
    script.enabled = true;
    script.initial_station_m = 60.0;
    script.initial_speed_mps = 6.0;  // 慢车（< 0.6×15=9）
    script.accel_mps2 = 2.0;
    sim.set_lead_script(script);
    const auto r = run_loop(sim, 60.0);
    std::printf("  min_dist=%.2fm max|lat|=%.2fm 末lat=%.2fm AEB=%d ego_x=%.0f lead_x=%.0f\n",
                r.min_obj_dist, r.max_abs_lat, r.final_lat, r.aeb_triggers, r.ego_final_x,
                r.lead_final_x);
    assert(r.reached_active && r.reached_return);       // 完整走过超车状态链
    assert(r.min_obj_dist > 2.5);                       // 无碰撞
    assert(r.max_abs_lat < 4.6);                        // 车道有界（≤ 左道中心+1.1）
    assert(std::fabs(r.final_lat) < 0.3);               // 回到本车道中心
    assert(r.ego_final_x > r.lead_final_x + 20.0);      // 确实超越
    assert(r.aeb_triggers == 0);                        // AEB 静默
  }

  // ── 场景 B：早期 ACTIVE 邻道突现车辆 → 中止；邻道车驶远后重试完成 ──
  std::puts("场景 B：中途危险中止 + 重试");
  {
    as::VehicleParams vp;
    as::SimVehicleCore sim(vp, {{1200.0, 0.0}}, 3.5);
    sim.set_initial_state(0.0, 0.0, 10.0);
    as::LeadScript script;
    script.enabled = true;
    script.initial_station_m = 60.0;
    script.initial_speed_mps = 6.0;
    script.accel_mps2 = 2.0;
    sim.set_lead_script(script);
    as::AdjacentCarScript adj;
    adj.enabled = true;
    // 场景 A 实测 ACTIVE≈t_act；spawn 定在其后 0.5s（早期阶段，|lat|<1.4 可中止），
    // 位置在自车前方邻道 25m、速度 10（快于前车，会逐渐驶离清空窗口）
    adj.spawn_time_s = 7.1;       // 按场景 A 实测 ACTIVE≈6.6s 校准：早期阶段（|lat|<1.4）
    adj.initial_station_m = 110.0;
    adj.lateral_m = 3.5;
    adj.speed_mps = 10.0;
    sim.set_adjacent_car_script(adj);
    const auto r = run_loop(sim, 70.0);
    std::printf("  min_dist=%.2fm max|lat|=%.2fm 末lat=%.2fm AEB=%d abort=%d ego_x=%.0f lead_x=%.0f\n",
                r.min_obj_dist, r.max_abs_lat, r.final_lat, r.aeb_triggers, r.abort_count,
                r.ego_final_x, r.lead_final_x);
    assert(r.abort_count >= 1);                         // 发生过中止
    assert(r.min_obj_dist > 2.5);                       // 无碰撞
    assert(r.aeb_triggers == 0);
    assert(r.ego_final_x > r.lead_final_x + 20.0);      // 重试后最终完成超越
    assert(std::fabs(r.final_lat) < 0.3);
  }

  std::puts("M4 overtake closed-loop smoke: ALL PASS");
  return 0;
}
