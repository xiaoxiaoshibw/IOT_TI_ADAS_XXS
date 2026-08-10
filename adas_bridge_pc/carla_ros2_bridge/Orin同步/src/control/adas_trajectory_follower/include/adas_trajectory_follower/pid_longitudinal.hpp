// adas_trajectory_follower/pid_longitudinal.hpp
// PID 纵向控制器 + 停车/起步状态机（对标 openpilot longcontrol 状态机 +
// Autoware pid_longitudinal_controller 的前馈结构）。
// 状态机解决的问题（旧栈实测教训）：接近静止目标时纯 PID 会在零速附近
// 蠕动往复；STOPPED 态用恒定制动把车"钉住"，起步统一走 RUNNING 的 PID。
#ifndef ADAS_TRAJECTORY_FOLLOWER__PID_LONGITUDINAL_HPP_
#define ADAS_TRAJECTORY_FOLLOWER__PID_LONGITUDINAL_HPP_

#include "adas_common/pid.hpp"
#include "adas_controller_base/controller_base.hpp"

namespace adas::control {

struct PidLongitudinalParams {
  double kp{1.0};
  double ki{0.3};
  double kd{0.0};
  double integral_limit{2.0};        // 积分限幅（±）
  double max_accel_mps2{3.0};
  double max_decel_mps2{4.0};        // 常规制动上限（AEB 走独立通道，不经此处）
  double stop_speed_mps{0.5};        // v_ref 与 v 同低于此值 → 进入 STOPPED
  double start_speed_mps{0.8};       // v_ref 高于此值 → 退出 STOPPED
  double stop_hold_accel_mps2{-1.5}; // STOPPED 态恒定驻车制动
  double preview_time_s{0.4};        // 速度参考取前视点（减少纯滞后）
  // Commit 6b — 积分冻结带：|error| < 此值时跳过积分累加，避免稳态抖动累积。
  // 0 表示禁用冻结（兼容旧行为）。
  double integrator_freeze_band_mps{0.5};
};

class PidLongitudinal : public LongitudinalControllerBase {
 public:
  enum class State { kRunning, kStopped };

  explicit PidLongitudinal(const PidLongitudinalParams& params);
  common::LongitudinalCommandData run(const ControlInput& input) override;
  void reset() override;
  State state() const { return state_; }

 private:
  PidLongitudinalParams params_;
  common::Pid pid_;
  State state_{State::kRunning};
};

}  // namespace adas::control

#endif  // ADAS_TRAJECTORY_FOLLOWER__PID_LONGITUDINAL_HPP_
