// adas_trajectory_follower/lqr_lateral.hpp
// LQR 横向控制器（对标 Apollo lat_based_lqr_controller，M7 插件）。
// 3 状态模型：[横向误差, 航向误差, 转向执行状态]——转向执行一阶迟滞（τ）
// 显式建模并用 steering_report 回读闭环（M7 实测：不建模迟滞的 2 状态版
// 在急弯与在线延迟下均发散，详见 docs/01 §9 调参实录）。
// 离线 Riccati 速度网格 + 在线 O(1) 插值（旧栈 mpc_longitudinal 同款模式）。
// 无约束 MPC 的等价形式：误差状态运动学模型 + 离散 Riccati 离线迭代，
// 在线 O(1)（沿用旧栈 mpc_longitudinal 的"构造期 Riccati、在线查表"模式）。
//
// 误差状态 x = [e_lat, e_yaw]ᵀ（相对轨迹最近点，左正），控制量 u = 前轮转角：
//   e_lat'  = e_lat + v·sin(e_yaw)·dt ≈ e_lat + v·e_yaw·dt
//   e_yaw'  = e_yaw + (v/L)·tan(δ)·dt − v·κ·dt ≈ e_yaw + (v/L)·δ·dt − v·κ·dt
// 曲率项作前馈：δ_ff = atan(L·κ)，反馈解 u = −K(v)·x。
// K(v) 随速度变化 → 构造时按速度网格逐点 Riccati 迭代，运行时线性插值（增益调度）。
#ifndef ADAS_TRAJECTORY_FOLLOWER__LQR_LATERAL_HPP_
#define ADAS_TRAJECTORY_FOLLOWER__LQR_LATERAL_HPP_

#include <vector>

#include "adas_controller_base/controller_base.hpp"

namespace adas::control {

struct LqrLateralParams {
  double wheelbase_m{2.7};
  double dt_s{0.02};            // 离散步长（= 控制周期）
  double steer_tau_s{0.2};      // 转向执行一阶迟滞时间常数（须与执行器标定一致）
  double q_lat{1.0};            // 横向误差权重
  double q_yaw{1.0};            // 航向误差权重
  double r_steer{30.0};         // 转角代价权重（M7 扫描定稿：r≤8 与转角饱和互激发散）
  double max_steer_rad{0.6};
  double v_grid_min{1.0};       // 增益调度速度网格
  double v_grid_max{30.0};
  double v_grid_step{1.0};
  int riccati_iters{200};       // 每格 Riccati 迭代次数（收敛远早于此）
  double preview_s{0.1};        // 参考点小前视
};

class LqrLateral : public LateralControllerBase {
 public:
  explicit LqrLateral(const LqrLateralParams& params);
  common::LateralCommandData run(const ControlInput& input) override;

 private:
  struct Gain {
    double k_lat{0.0};
    double k_yaw{0.0};
    double k_steer{0.0};  // 执行状态反馈（迟滞阻尼项）
  };
  // 求速度 v 下的反馈增益（3 状态离散 Riccati 定点迭代）
  void solve_gain(double v, Gain& gain) const;

  LqrLateralParams params_;
  std::vector<Gain> gains_;  // 按速度网格
  double last_steer_{0.0};
};

}  // namespace adas::control

#endif  // ADAS_TRAJECTORY_FOLLOWER__LQR_LATERAL_HPP_
