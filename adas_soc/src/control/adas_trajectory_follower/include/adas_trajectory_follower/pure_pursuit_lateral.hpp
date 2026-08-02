// adas_trajectory_follower/pure_pursuit_lateral.hpp
// Pure Pursuit 横向控制器（对标 autoware_pure_pursuit，M1 首版；M7 加 MPC 插件）
#ifndef ADAS_TRAJECTORY_FOLLOWER__PURE_PURSUIT_LATERAL_HPP_
#define ADAS_TRAJECTORY_FOLLOWER__PURE_PURSUIT_LATERAL_HPP_

#include "adas_controller_base/controller_base.hpp"

namespace adas::control {

struct PurePursuitParams {
  double wheelbase_m{2.7};
  double lookahead_gain_s{0.8};    // 前视距离 = gain * v
  double min_lookahead_m{3.0};
  double max_lookahead_m{20.0};
  double max_steer_rad{0.6};
};

class PurePursuitLateral : public LateralControllerBase {
 public:
  explicit PurePursuitLateral(const PurePursuitParams& params);
  common::LateralCommandData run(const ControlInput& input) override;

 private:
  PurePursuitParams params_;
  double last_steer_{0.0};
};

}  // namespace adas::control

#endif  // ADAS_TRAJECTORY_FOLLOWER__PURE_PURSUIT_LATERAL_HPP_
