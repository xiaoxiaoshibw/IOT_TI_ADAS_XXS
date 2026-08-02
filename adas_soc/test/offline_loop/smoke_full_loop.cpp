// M1 全链路离线闭环冒烟（无 ROS）：
//   SimVehicleCore → TrajectoryPlannerCore(10Hz) → PurePursuit+PID(50Hz)
//   → GateCore(50Hz) → SimVehicleInterface → SimVehicleCore.step
// 场景：300m 直道 + R=100m 左弯 200m + 300m 直道；0 起步巡航 15m/s；
//        t=45s 模拟 follower 猝死 → gate builtin_stop 舒适停车。
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "adas_command_gate/gate_core.hpp"
#include "adas_sim_vehicle/sim_vehicle_core.hpp"
#include "adas_trajectory_follower/pid_longitudinal.hpp"
#include "adas_trajectory_follower/pure_pursuit_lateral.hpp"
#include "adas_trajectory_planner/trajectory_planner_core.hpp"
#include "adas_vehicle_interface/sim_vehicle_interface.hpp"

namespace ac = adas::common;
namespace as = adas::sim;
namespace ap = adas::planning;
namespace ct = adas::control;
namespace av = adas::vehicle;

int main() {
  const double dt = 0.02;  // 50Hz
  as::VehicleParams vp;    // 缺省：wheelbase 2.7, max_steer 0.6, accel 3/8, steer_tau 0.2
  as::SimVehicleCore sim(vp, {{300.0, 0.0}, {200.0, 0.01}, {300.0, 0.0}}, 3.5);
  sim.set_initial_state(0.0, 0.3, 0.0);  // 起步带 0.3m 左偏，检验收敛

  ap::PlannerParams pp;
  pp.cruise_speed_mps = 15.0;
  ap::TrajectoryPlannerCore planner(pp);

  ct::PurePursuitLateral lateral((ct::PurePursuitParams()));
  ct::PidLongitudinal longitudinal((ct::PidLongitudinalParams()));
  ct::GateCore gate((ct::GateParams()));
  av::SimVehicleInterface vehicle_if((av::SimVehicleInterfaceParams()));

  ac::Trajectory traj;
  double follower_stamp = -1e9;
  bool follower_alive = true;
  ct::GateSource last_source = ct::GateSource::kBuiltinStop;

  double sum_lat2 = 0.0;
  int lat_n = 0;
  double max_lat_cruise = 0.0, max_lat_curve = 0.0;
  double v_at_20 = 0.0, v_at_58 = 0.0;
  bool ever_builtin_after_kill = false;

  const int ticks = static_cast<int>(60.0 / dt);
  for (int i = 0; i < ticks; ++i) {
    const double t = i * dt;
    const auto ego = sim.state();
    const auto lane = sim.lane_state();

    // 10Hz 规划
    if (i % 5 == 0 && lane.valid) {
      traj = planner.plan(ego, lane);
    }
    // t=45s follower 猝死
    if (t >= 45.0) {
      follower_alive = false;
    }

    ct::GateInputs gin;
    gin.now_s = t;
    gin.dt = dt;
    gin.ego_speed_mps = ego.velocity_mps;
    if (follower_alive && traj.size() >= 2) {
      ct::ControlInput cin;
      cin.trajectory = &traj;
      cin.state = ego;
      cin.steering_angle_rad = sim.steering_angle_rad();
      cin.dt = dt;
      gin.follower_received = true;
      follower_stamp = t;
      gin.follower_stamp_s = follower_stamp;
      gin.follower_cmd.lateral = lateral.run(cin);
      gin.follower_cmd.longitudinal = longitudinal.run(cin);
    } else {
      gin.follower_received = true;   // 曾收到过
      gin.follower_stamp_s = follower_stamp;  // 停更 → gate 将超时
    }

    const auto d = gate.update(gin);
    if (d.source != last_source) {
      std::printf("  t=%5.2fs gate 切源 %d→%d (%s)\n", t, static_cast<int>(last_source),
                  static_cast<int>(d.source), d.reason.c_str());
      last_source = d.source;
    }
    if (t >= 45.5 && d.source == ct::GateSource::kBuiltinStop) {
      ever_builtin_after_kill = true;
    }

    const auto act = vehicle_if.apply(d.cmd, ego);
    sim.step(act, dt);

    // 指标采集
    if (lane.valid && t >= 10.0 && t < 45.0) {
      sum_lat2 += lane.lateral_offset * lane.lateral_offset;
      ++lat_n;
    }
    if (lane.valid && t >= 15.0 && t < 20.0) {
      max_lat_cruise = std::max(max_lat_cruise, std::fabs(lane.lateral_offset));
    }
    if (lane.valid && t >= 30.0 && t < 36.0) {
      max_lat_curve = std::max(max_lat_curve, std::fabs(lane.lateral_offset));
    }
    if (std::fabs(t - 20.0) < dt / 2) {
      v_at_20 = ego.velocity_mps;
    }
    if (std::fabs(t - 58.0) < dt / 2) {
      v_at_58 = ego.velocity_mps;
    }
  }

  const double rms = std::sqrt(sum_lat2 / std::max(lat_n, 1));
  std::printf("指标: RMS(lat,10-45s)=%.3fm  max|lat|直道=%.3fm  max|lat|弯道=%.3fm\n", rms,
              max_lat_cruise, max_lat_curve);
  std::printf("      v(20s)=%.2fm/s  v(58s)=%.3fm/s  kill后builtin=%d\n", v_at_20, v_at_58,
              ever_builtin_after_kill ? 1 : 0);

  // M1 验收断言
  assert(rms <= 0.3);              // 横向 RMS ≤ 0.3m
  assert(max_lat_cruise <= 0.3);   // 直道巡航
  assert(max_lat_curve <= 0.5);    // 弯道（含入弯瞬态）
  assert(v_at_20 >= 14.0);         // 巡航速度建立
  assert(ever_builtin_after_kill); // follower 死亡 → builtin_stop
  assert(v_at_58 < 0.1);           // 平稳停车
  std::puts("M1 full closed-loop smoke: ALL PASS");
  return 0;
}
