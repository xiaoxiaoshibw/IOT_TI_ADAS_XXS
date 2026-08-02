#ifndef ADAS_CAN_GATEWAY__FEEDBACK_MONITOR_HPP_
#define ADAS_CAN_GATEWAY__FEEDBACK_MONITOR_HPP_

#include <algorithm>
#include <cmath>

namespace adas::can_gateway {

// 执行器反馈闭环监控（DEF-DIAG-003 / P1）。
//
// 只有在 MCU 状态为 ACTIVE 且网关拥有控制权时才判偏差：MRM、AEB、FAILSAFE
// 等状态下 MCU 有意偏离上游命令，那不是执行器故障。偏差必须持续超阈才触发，
// 吸收 MCU 侧的 slew/jerk 限幅暂态；一旦触发即锁存，执行器失配是硬件级
// 故障，禁止自动恢复（需人工复位/进程重启）。反馈超时故障在反馈恢复并
// 稳定保持一段时间后自动清除。
struct FeedbackMonitorConfig {
  double steer_tolerance_deg{8.0};
  double accel_tolerance_mps2{2.5};
  double deviation_hold_s{0.5};
  double feedback_timeout_s{0.2};
  double recovery_hold_s{1.0};
};

class FeedbackMonitor {
 public:
  explicit FeedbackMonitor(const FeedbackMonitorConfig& config) : config_(config) {}

  // link_active：上游控制命令新鲜且 MCU 心跳可见——反馈超时监控的使能门。
  //   不依赖控制权本身，否则超时故障撤权后监控停摆，故障永远无法恢复。
  // deviation_eligible：MCU 处于 ACTIVE 且采信网关命令——偏差监控的使能门。
  void update(double now_s, bool link_active, bool deviation_eligible,
              bool feedback_fresh,
              double commanded_steer_deg, double feedback_steer_deg,
              double commanded_accel_mps2, double feedback_accel_mps2) {
    if (!link_active) {
      // 链路整体不活跃（未在控车/MCU 失联由别的机制处理）：不累计，
      // 也不衰减已锁存的偏差故障。
      deviation_since_s_ = -1.0;
      stale_since_s_ = -1.0;
      fresh_since_s_ = -1.0;
      return;
    }

    if (feedback_fresh) {
      stale_since_s_ = -1.0;
      if (fresh_since_s_ < 0.0) fresh_since_s_ = now_s;
      if (timeout_fault_ && (now_s - fresh_since_s_) >= config_.recovery_hold_s) {
        timeout_fault_ = false;
      }

      if (deviation_eligible) {
        steer_deviation_deg_ = std::abs(commanded_steer_deg - feedback_steer_deg);
        accel_deviation_mps2_ = std::abs(commanded_accel_mps2 - feedback_accel_mps2);
        const bool deviating = steer_deviation_deg_ > config_.steer_tolerance_deg ||
                               accel_deviation_mps2_ > config_.accel_tolerance_mps2;
        if (deviating) {
          if (deviation_since_s_ < 0.0) deviation_since_s_ = now_s;
          if ((now_s - deviation_since_s_) >= config_.deviation_hold_s) {
            deviation_fault_ = true;
          }
        } else {
          deviation_since_s_ = -1.0;
        }
      } else {
        deviation_since_s_ = -1.0;
      }
    } else {
      fresh_since_s_ = -1.0;
      deviation_since_s_ = -1.0;
      if (stale_since_s_ < 0.0) stale_since_s_ = now_s;
      if ((now_s - stale_since_s_) >= config_.feedback_timeout_s) {
        timeout_fault_ = true;
      }
    }
  }

  bool deviation_fault() const { return deviation_fault_; }
  bool timeout_fault() const { return timeout_fault_; }
  bool any_fault() const { return deviation_fault_ || timeout_fault_; }
  double steer_deviation_deg() const { return steer_deviation_deg_; }
  double accel_deviation_mps2() const { return accel_deviation_mps2_; }

  // 显式人工复位（经网关 reset_actuator_fault 服务触发）：清除锁存的偏差
  // 故障与超时故障并复位计时，使监控从干净状态重新评估。执行器失配仍禁止
  // *自动* 恢复——此复位是操作员明确确认已排障后的动作；若失配仍在，偏差
  // 会在 deviation_hold_s 后重新锁存。取代"重启网关进程"这一唯一复位途径。
  void reset() {
    deviation_fault_ = false;
    timeout_fault_ = false;
    deviation_since_s_ = -1.0;
    stale_since_s_ = -1.0;
    fresh_since_s_ = -1.0;
    steer_deviation_deg_ = 0.0;
    accel_deviation_mps2_ = 0.0;
  }

 private:
  FeedbackMonitorConfig config_;
  bool deviation_fault_{false};   // 锁存
  bool timeout_fault_{false};     // 反馈稳定恢复后清除
  double deviation_since_s_{-1.0};
  double stale_since_s_{-1.0};
  double fresh_since_s_{-1.0};
  double steer_deviation_deg_{0.0};
  double accel_deviation_mps2_{0.0};
};

}  // namespace adas::can_gateway

#endif  // ADAS_CAN_GATEWAY__FEEDBACK_MONITOR_HPP_
