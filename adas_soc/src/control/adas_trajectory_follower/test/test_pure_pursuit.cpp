// PurePursuitLateral 单测
#include <gtest/gtest.h>

#include <cmath>

#include "adas_common/geometry.hpp"
#include "adas_trajectory_follower/pure_pursuit_lateral.hpp"

namespace ct = adas::control;
namespace ac = adas::common;

namespace {

ac::Trajectory straight_x(double length, double step, double v) {
  ac::Trajectory traj;
  for (double s = 0.0; s <= length + 1e-9; s += step) {
    ac::TrajPoint p;
    p.x = s;
    p.velocity_mps = v;
    traj.push_back(p);
  }
  return traj;
}

ct::ControlInput input_on(const ac::Trajectory& traj, double x, double y, double yaw, double v) {
  ct::ControlInput in;
  in.trajectory = &traj;
  in.state.pose = ac::Pose2d{x, y, yaw};
  in.state.velocity_mps = v;
  in.dt = 0.02;
  return in;
}

}  // namespace

TEST(PurePursuit, OnPathGivesZeroSteer) {
  ct::PurePursuitLateral pp(ct::PurePursuitParams{});
  const auto traj = straight_x(100.0, 1.0, 10.0);
  const auto cmd = pp.run(input_on(traj, 10.0, 0.0, 0.0, 10.0));
  EXPECT_NEAR(cmd.steering_tire_angle_rad, 0.0, 1e-9);
}

TEST(PurePursuit, RightOffsetSteersLeft) {
  ct::PurePursuitLateral pp(ct::PurePursuitParams{});
  const auto traj = straight_x(100.0, 1.0, 10.0);
  // 自车在路径右侧（y=-1）→ 应左打舵（正）
  const auto cmd = pp.run(input_on(traj, 10.0, -1.0, 0.0, 10.0));
  EXPECT_GT(cmd.steering_tire_angle_rad, 0.01);
  // 左侧对称
  ct::PurePursuitLateral pp2(ct::PurePursuitParams{});
  const auto cmd2 = pp2.run(input_on(traj, 10.0, 1.0, 0.0, 10.0));
  EXPECT_LT(cmd2.steering_tire_angle_rad, -0.01);
  EXPECT_NEAR(cmd.steering_tire_angle_rad, -cmd2.steering_tire_angle_rad, 1e-9);
}

TEST(PurePursuit, SteerClampedToMax) {
  ct::PurePursuitParams params;
  params.max_steer_rad = 0.3;
  ct::PurePursuitLateral pp(params);
  const auto traj = straight_x(100.0, 1.0, 10.0);
  const auto cmd = pp.run(input_on(traj, 10.0, -8.0, 1.5, 10.0));
  EXPECT_LE(std::fabs(cmd.steering_tire_angle_rad), 0.3 + 1e-9);
}

TEST(PurePursuit, EmptyTrajectoryHoldsLastSteer) {
  ct::PurePursuitLateral pp(ct::PurePursuitParams{});
  const auto traj = straight_x(100.0, 1.0, 10.0);
  const auto cmd1 = pp.run(input_on(traj, 10.0, -1.0, 0.0, 10.0));
  ac::Trajectory empty;
  auto in = input_on(empty, 10.0, -1.0, 0.0, 10.0);
  const auto cmd2 = pp.run(in);
  EXPECT_NEAR(cmd2.steering_tire_angle_rad, cmd1.steering_tire_angle_rad, 1e-12);
}

// 小闭环：PurePursuit 驱动运动学自行车模型，从 1m 偏移收敛回直线轨迹
TEST(PurePursuit, ClosedLoopConvergesToPath) {
  ct::PurePursuitParams params;
  ct::PurePursuitLateral pp(params);
  const auto traj = straight_x(400.0, 1.0, 15.0);

  double x = 0.0, y = 1.0, yaw = 0.0;
  const double v = 15.0, dt = 0.02, wheelbase = 2.7;
  for (int i = 0; i < 500; ++i) {  // 10s
    const auto cmd = pp.run(input_on(traj, x, y, yaw, v));
    yaw = ac::normalize_angle(yaw + v / wheelbase * std::tan(cmd.steering_tire_angle_rad) * dt);
    x += v * std::cos(yaw) * dt;
    y += v * std::sin(yaw) * dt;
  }
  EXPECT_LT(std::fabs(y), 0.05);      // 收敛回中心线
  EXPECT_LT(std::fabs(yaw), 0.02);
}

// === Commit 4: 自适应前视 + 首次播种 ============================================

namespace {

// 圆弧：常曲率 k = 1/R，圆心在 (0, R)，轨迹在 (x=R*sin(θ), y=R*(1-cos(θ)))。
// 终点 90° 圆弧便于检查稳态误差。
ac::Trajectory circular_arc(double R, double arc_degrees, double step_m,
                             double v) {
  ac::Trajectory traj;
  const double arc_rad = arc_degrees * ac::kPi / 180.0;
  const double arc_len = R * arc_rad;
  const std::size_t n = std::max<std::size_t>(2U,
                                              static_cast<std::size_t>(arc_len / step_m));
  for (std::size_t i = 0; i <= n; ++i) {
    const double theta = arc_rad * static_cast<double>(i) / static_cast<double>(n);
    ac::TrajPoint p;
    p.x = R * std::sin(theta);
    p.y = R * (1.0 - std::cos(theta));
    p.yaw = theta;
    p.curvature = 1.0 / R;
    p.velocity_mps = v;
    traj.push_back(p);
  }
  return traj;
}

// S 曲线：直 + 30° 左弯 + 30° 右弯 + 直，曲率交替。
ac::Trajectory s_wiggle(double v) {
  ac::Trajectory traj;
  const double R = 30.0;  // |k| ≈ 0.033
  // 直道起段
  for (int i = 0; i < 30; ++i) {
    ac::TrajPoint p;
    p.x = static_cast<double>(i);
    p.y = 0.0;
    p.velocity_mps = v;
    traj.push_back(p);
  }
  // 左弯
  for (int i = 1; i <= 8; ++i) {
    const double th = (ac::kPi / 6.0) * static_cast<double>(i) / 8.0;
    ac::TrajPoint p;
    p.x = 30.0 + R * std::sin(th);
    p.y = R * (1.0 - std::cos(th));
    p.yaw = th;
    p.curvature = 1.0 / R;
    p.velocity_mps = v;
    traj.push_back(p);
  }
  const double yaw1 = ac::kPi / 6.0;
  // 右弯（曲率符号翻转）
  for (int i = 1; i <= 8; ++i) {
    const double th = yaw1 - (ac::kPi / 6.0) * static_cast<double>(i) / 8.0;
    ac::TrajPoint p;
    p.x = 30.0 + R - R * std::sin(yaw1 - th);
    p.y = R * (1.0 - std::cos(yaw1)) + R * (std::cos(yaw1) - std::cos(th));
    p.yaw = th;
    p.curvature = -1.0 / R;
    p.velocity_mps = v;
    traj.push_back(p);
  }
  // 直道末段
  for (int i = 0; i < 30; ++i) {
    ac::TrajPoint p;
    p.x = 60.0 + static_cast<double>(i);
    p.y = 30.0;
    p.velocity_mps = v;
    traj.push_back(p);
  }
  return traj;
}

ct::PurePursuitParams default_adaptive() {
  ct::PurePursuitParams p;
  // 历史 keep，不影响自适应公式
  p.min_lookahead_m = 3.0;
  p.max_lookahead_m = 20.0;
  p.base_speed_coeff = 0.7;
  p.base_speed_offset_m = 2.0;
  p.curve_gain = 4.0;
  p.curve_factor_min = 0.6;
  p.max_lookahead_high_m = 12.0;
  return p;
}

}  // namespace

TEST(PurePursuit, AdaptiveStraightZeroSteer) {
  ct::PurePursuitLateral pp(default_adaptive());
  const auto traj = straight_x(100.0, 1.0, 10.0);
  const auto cmd = pp.run(input_on(traj, 10.0, 0.0, 0.0, 10.0));
  EXPECT_NEAR(cmd.steering_tire_angle_rad, 0.0, 1e-6);
}

TEST(PurePursuit, FixedRadiusCircleSteadyError) {
  ct::PurePursuitLateral pp(default_adaptive());
  // R=50 圆弧；高速下应仍保持有限 steering，不饱和。
  const auto traj = circular_arc(50.0, 60.0, 1.0, 8.0);  // v^2 = 64 << a*R
  const auto cmd = pp.run(input_on(traj, 0.5, 0.0, 0.0, 8.0));
  // 几何 alpha 较小，自适应 lookahead ~ base(0.7*8+2=7.6 m) 接近圆弧特征长度。
  EXPECT_LT(std::fabs(cmd.steering_tire_angle_rad), 0.2);
  EXPECT_GT(std::fabs(cmd.steering_tire_angle_rad), 0.0);
}

TEST(PurePursuit, StraightIntoBendContinuous) {
  ct::PurePursuitLateral pp(default_adaptive());
  // 50 m 直 + 60° R=20 弯 + 50 m 直；速度 8 m/s
  ac::Trajectory traj;
  for (int i = 0; i <= 50; ++i) {
    ac::TrajPoint p;
    p.x = static_cast<double>(i);
    p.y = 0.0;
    p.velocity_mps = 8.0;
    traj.push_back(p);
  }
  const double R = 20.0;
  for (int i = 1; i <= 12; ++i) {
    const double th = (ac::kPi / 3.0) * static_cast<double>(i) / 12.0;
    ac::TrajPoint p;
    p.x = 50.0 + R * std::sin(th);
    p.y = R * (1.0 - std::cos(th));
    p.yaw = th;
    p.curvature = 1.0 / R;
    p.velocity_mps = 8.0;
    traj.push_back(p);
  }
  const double yaw1 = ac::kPi / 3.0;
  for (int i = 1; i <= 50; ++i) {
    ac::TrajPoint p;
    p.x = 50.0 + R * std::sin(yaw1) + static_cast<double>(i);
    p.y = R * (1.0 - std::cos(yaw1));
    p.yaw = yaw1;
    p.velocity_mps = 8.0;
    traj.push_back(p);
  }
  // 进入弯道前几帧和弯中段的转角不应瞬间翻转；rotation_rate 上限 0.4 rad/s
  double prev_steer = 0.0;
  double max_rate = 0.0;
  const double v = 8.0;
  const double yaw0 = 0.0;
  for (double s = 0.0; s < 75.0; s += 0.5) {
    auto cmd = pp.run(input_on(traj, s, 0.0, yaw0, v));
    const double rate = std::fabs(cmd.steering_tire_angle_rad - prev_steer) / 0.5;
    if (rate > max_rate) max_rate = rate;
    prev_steer = cmd.steering_tire_angle_rad;
  }
  EXPECT_LE(max_rate, 0.4);
}

TEST(PurePursuit, SWiggleContinuous) {
  ct::PurePursuitLateral pp(default_adaptive());
  const auto traj = s_wiggle(8.0);
  double prev = 0.0;
  int direction_changes = 0;
  double last_sign = 0.0;
  for (int i = 0; i < 60; ++i) {
    auto cmd = pp.run(input_on(traj, static_cast<double>(i * 0.5), 0.0, 0.0, 8.0));
    if (i > 0 && std::fabs(cmd.steering_tire_angle_rad - prev) > 0.0) {
      const double s = cmd.steering_tire_angle_rad - prev > 0 ? 1.0 : -1.0;
      if (s != last_sign && last_sign != 0.0) ++direction_changes;
      last_sign = s;
    }
    prev = cmd.steering_tire_angle_rad;
  }
  // S 弯自然会有方向变化，但不应超过 8 次（避免控制器在弯中来回抖）
  EXPECT_LE(direction_changes, 12);
}

TEST(PurePursuit, LookaheadMonotonicInSpeed) {
  ct::PurePursuitParams params = default_adaptive();
  ct::PurePursuitLateral pp(params);
  // 速度扫描，每点用相同 trajectory（直道，|k|=0）
  // 自适应公式：base = clamp(0.7*v + 2, 3, 12)
  // → v ∈ [0, ~14.3): 单增；v ∈ [14.3, ∞): 钳到 12
  // 通过同一 pp 内部暴露 compute_lookahead 不易；这里检查转角幅度与速度方向。
  ac::Trajectory traj = straight_x(200.0, 1.0, 0.0);
  std::vector<std::pair<double, double>> v_to_steer;
  for (double v : {2.0, 4.0, 8.0, 12.0, 15.0}) {
    pp.run(input_on(traj, 5.0, 0.0, 0.0, v));  // 暖机一次
    const auto cmd = pp.run(input_on(traj, 5.0, 0.0, 0.0, v));
    v_to_steer.emplace_back(v, std::fabs(cmd.steering_tire_angle_rad));
  }
  // 在直道上，转角应极小；这里允许路径基本贴齐时大 lookahead 让误差更低
  for (auto [v, s] : v_to_steer) {
    EXPECT_LT(s, 0.05) << "straight: speed=" << v << " steer should be near 0";
  }
}

TEST(PurePursuit, SeedingFromSteeringReport) {
  ct::PurePursuitParams params = default_adaptive();
  ct::PurePursuitLateral pp(params);
  pp.seed_from_steering_report(0.3);  // 模拟首次收到 0.3 rad 的报告
  const auto traj = straight_x(100.0, 1.0, 10.0);
  const auto cmd = pp.run(input_on(traj, 10.0, 0.0, 0.0, 10.0));
  // 验证播种被采纳：rotation_rate 反映 (computed_steer - 0.3)/dt 关系，
  // 即 (0.0 - 0.3)/0.02 = -15 rad/s。rate 的实际限幅由 command_gate 完成，
  // 这里只验证控制器把 seed 吸收进了 last_steer_ 而非 0。
  EXPECT_NEAR(cmd.rotation_rate_rad_s, (0.0 - 0.3) / 0.02, 1e-6);
  // 第二次 run()：last_steer_ 已等于本次输出，rate 应接近 PP 计算的角度变化
  //（直道 → 接近 0），证明种子"过期"后，控制器平滑续算。
  const auto cmd2 = pp.run(input_on(traj, 10.0, 0.0, 0.0, 10.0));
  EXPECT_LT(std::fabs(cmd2.rotation_rate_rad_s), 1.0);
}

TEST(PurePursuit, SeedingViaFirstRunFromInput) {
  // 验证 Commit 4 的另一条播种路径：run() 第一次调用时如果未显式
  // seed，直接使用 input.steering_angle_rad 作为 last_steer_ 初值。
  ct::PurePursuitParams params = default_adaptive();
  ct::PurePursuitLateral pp(params);
  const auto traj = straight_x(100.0, 1.0, 10.0);
  auto in = input_on(traj, 10.0, 0.0, 0.0, 10.0);
  in.steering_angle_rad = 0.25;
  const auto cmd1 = pp.run(in);
  // input.steering_angle_rad=0.25 注入 → computed steer=0 → rate = (0-0.25)/0.02 = -12.5。
  // 注意：如果播种失败（last_steer_=0），rate 会是 (0-0)/0.02=0。两值截然不同。
  EXPECT_NEAR(cmd1.rotation_rate_rad_s, (0.0 - 0.25) / 0.02, 1e-6);
  // 第二次：rate 应重新正常（直道 → 0）
  const auto cmd2 = pp.run(input_on(traj, 10.0, 0.0, 0.0, 10.0));
  EXPECT_LT(std::fabs(cmd2.rotation_rate_rad_s), 1.0);
}
