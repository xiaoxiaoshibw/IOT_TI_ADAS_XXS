// adas_trajectory_follower/pure_pursuit_lateral.hpp
// Pure Pursuit 横向控制器（对标 autoware_pure_pursuit，M1 首版；M7 加 MPC 插件）
#ifndef ADAS_TRAJECTORY_FOLLOWER__PURE_PURSUIT_LATERAL_HPP_
#define ADAS_TRAJECTORY_FOLLOWER__PURE_PURSUIT_LATERAL_HPP_

#include "adas_controller_base/controller_base.hpp"

namespace adas::control {

struct PurePursuitParams {
  double wheelbase_m{2.7};
  // 历史保留：早期单一公式 lookahead = gain * v。
  // Commit 4 起默认不再使用；自适应公式见下面的 adaptive.* 与 compute_lookahead()。
  double lookahead_gain_s{0.8};
  double min_lookahead_m{3.0};
  double max_lookahead_m{20.0};
  double max_steer_rad{0.6};
  // Commit 4 — 自适应前视。base_speed_coeff * speed + base_speed_offset_m 给"高速直道"
  // 12 m 量级，curve_factor 把紧弯缩短到 7-9 m，整体避免蛇形和滞后。
  double base_speed_coeff{0.7};
  double base_speed_offset_m{2.0};
  double curve_gain{4.0};          // 1/(1 + curve_gain * |k|) 曲率压缩系数
  double curve_factor_min{0.6};    // 紧弯最压缩比例
  double max_lookahead_high_m{12.0};  // 高速前视距离上限（蛇形保护）
};

class PurePursuitLateral : public LateralControllerBase {
 public:
  explicit PurePursuitLateral(const PurePursuitParams& params);
  common::LateralCommandData run(const ControlInput& input) override;
  // Commit 4 — 由调用方在收到第一条 SteeringReport 时调用，把 last_steer_
  // 设为实际反馈角度；防止 lifecycle 重新激活时控制器从 0 起跳导致
  // rotation_rate_rad_s 第一拍出现尖峰。该方法幂等；reset() 也可重置。
  void seed_from_steering_report(double steering_tire_angle_rad);

 private:
  PurePursuitParams params_;
  double last_steer_{0.0};
  bool seeded_{false};
  // 自适应前视：base × curve_factor，公式详见 .cpp 实现
  double compute_lookahead(double speed_mps, double preview_curvature) const;
};

}  // namespace adas::control

#endif  // ADAS_TRAJECTORY_FOLLOWER__PURE_PURSUIT_LATERAL_HPP_
