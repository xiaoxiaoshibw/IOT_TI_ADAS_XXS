// adas_command_gate/gate_core.hpp
// 下发前唯一安全裁决点（对标 autoware_control_command_gate）：
//   1. 命令源选择器：follower（常规）/ aeb（紧急纵向覆盖）/ builtin_stop（内建停车，
//      不依赖任何上游——上游全死也能输出减速到停的命令序列）
//   2. 滤波器：速度相关转角/转角速率限幅 + 加速度/jerk 限幅
// 继承旧栈原则：AEB 制动只增不减；接管/降级瞬态出现的任何切换都要平滑。
#ifndef ADAS_COMMAND_GATE__GATE_CORE_HPP_
#define ADAS_COMMAND_GATE__GATE_CORE_HPP_

#include <string>
#include <vector>

#include "adas_common/lookup_table.hpp"
#include "adas_common/types.hpp"

namespace adas::control {

enum class GateSource { kFollower = 0, kAeb = 1, kBuiltinStop = 2 };

struct GateParams {
  double follower_timeout_s{0.2};
  double aeb_stale_timeout_s{0.1};
  double odom_stale_timeout_s{0.15};
  // builtin_stop 行为
  double stop_decel_mps2{2.5};       // 舒适停车减速度（MRM_COMFORT）
  double steer_decay_tau_s{1.0};     // 停车期间转角向 0 衰减时间常数
  // 速度相关限幅表（速度断点严格递增；构造时审计）
  std::vector<double> speed_points_mps{0.0, 10.0, 20.0, 30.0};
  std::vector<double> steer_lim_rad{0.6, 0.35, 0.2, 0.12};
  std::vector<double> steer_rate_lim_rps{0.8, 0.5, 0.35, 0.25};
  // 纵向限幅
  double max_accel_mps2{3.0};
  double max_decel_mps2{8.0};        // 物理制动上限（AEB 可用满）
  double max_jerk_mps3{10.0};        // 常规源双向 jerk 限；紧急源制动方向放开
};

struct GateInputs {
  double now_s{0.0};
  double dt{0.02};
  double ego_speed_mps{0.0};
  // follower 源
  bool follower_received{false};
  double follower_stamp_s{-1e9};
  common::ControlData follower_cmd;
  // aeb 源（M3 接入；未接入时保持 false）
  bool aeb_emergency{false};
  bool aeb_received{false};
  double aeb_stamp_s{-1e9};
  common::ControlData aeb_cmd;
  // odometry freshness gates speed-dependent steering limits
  bool odom_received{false};
  double odom_stamp_s{-1e9};
  // 系统请求
  bool mrm_stop_requested{false};
  bool navigation_planned_stop{false};
  bool force_builtin_stop{false};    // 调试服务强制
};

struct GateDecision {
  common::ControlData cmd;
  GateSource source{GateSource::kBuiltinStop};
  bool limited{false};
  std::string reason;
};

class GateCore {
 public:
  explicit GateCore(const GateParams& params);  // 表非法直接抛异常拒绝启动
  GateDecision update(const GateInputs& in);

 private:
  GateParams params_;
  common::LookupTable1D steer_lim_;
  common::LookupTable1D steer_rate_lim_;
  // 输出侧滤波状态（跨源连续——切源不突跳的关键）
  double last_steer_{0.0};
  double last_accel_{0.0};
  double last_aeb_stamp_{-1e9};

  bool process_aeb_override(const GateInputs& in, GateDecision* decision);
};

}  // namespace adas::control

#endif  // ADAS_COMMAND_GATE__GATE_CORE_HPP_
