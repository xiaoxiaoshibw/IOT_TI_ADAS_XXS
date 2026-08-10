// adas_redundancy/arbiter_core.hpp
// 双栈冗余仲裁核心（无 ROS 依赖）。角色定位 = 旧架构 ESP32 仲裁 + 心跳接管的合并：
//   双判据选源：① gate 命令流存活（静默超时）② gate 健康度（follower 源=正常，
//   builtin_stop=降级）——主栈规划链死亡时主 gate 仍在发 builtin_stop 流，
//   仅凭存活判据会跟着主栈停车；健康度判据让备栈无缝接续巡航（旧栈"无感降级"）。
// 接管平滑（概念继承旧栈实测设计，实现全新）：
//   - 切源瞬态窗内转角/加速度用更紧速率限幅（takeover guard）
//   - 新源处于 AEB 紧急制动时放开"制动加深"方向速率（AEB-seed 特例）
//   - 双源全失联 → invalid（仲裁器停发，车辆接口看门狗全力制动兜底）
#ifndef ADAS_REDUNDANCY__ARBITER_CORE_HPP_
#define ADAS_REDUNDANCY__ARBITER_CORE_HPP_

#include <cstdint>
#include <string>

#include "adas_common/types.hpp"

namespace adas::system {

enum class ActiveRole : uint8_t { kNone = 0, kPrimary = 1, kBackup = 2 };

struct ArbiterParams {
  double cmd_timeout_s{0.06};        // 3 帧@50Hz 静默 → 判死（接管 < 100ms 验收）
  double recover_stable_s{1.0};      // 主栈恢复正常须稳定此时长才回切
  double takeover_guard_s{1.0};      // 接管瞬态窗时长
  double guard_steer_rate_rps{0.25}; // 瞬态窗转角速率（紧）
  double guard_accel_rate_mps3{4.0}; // 瞬态窗加速度速率（紧）
  double aeb_brake_rate_mps3{12.0};  // 新源 AEB 中：制动加深方向放开（旧栈 ~12 实测）
  double normal_steer_rate_rps{0.8}; // 常态转角速率（与 gate 低速档一致）
  double normal_accel_rate_mps3{10.0};
};

// 单栈观测快照
struct StackObservation {
  bool received{false};          // 是否收到过命令
  double stamp_s{-1e18};         // 最近命令到达时刻
  common::ControlData cmd;       // 最近命令
  bool nominal{false};           // gate 状态 = follower 源（true）/降级（false）
  bool aeb_active{false};        // 该栈 AEB EMERGENCY 中
};

struct ArbiterInputs {
  double now_s{0.0};
  double dt{0.01};
  StackObservation primary;
  StackObservation backup;
};

struct ArbiterDecision {
  bool valid{false};             // false = 双源失联，本拍不发（看门狗兜底）
  common::ControlData cmd;
  ActiveRole role{ActiveRole::kNone};
  bool takeover_guard{false};
  std::string reason;
};

class ArbiterCore {
 public:
  explicit ArbiterCore(const ArbiterParams& params);
  ArbiterDecision update(const ArbiterInputs& in);
  ActiveRole role() const { return role_; }

 private:
  ArbiterParams params_;
  ActiveRole role_{ActiveRole::kNone};
  double guard_until_s_{-1e18};
  double primary_nominal_since_s_{-1e18};  // 主栈连续正常的起始时刻（回切判据）
  // 输出侧平滑状态（跨源连续）
  double last_steer_{0.0};
  double last_accel_{0.0};
  bool has_output_{false};
};

}  // namespace adas::system

#endif  // ADAS_REDUNDANCY__ARBITER_CORE_HPP_
