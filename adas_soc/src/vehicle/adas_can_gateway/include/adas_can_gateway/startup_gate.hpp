#ifndef ADAS_CAN_GATEWAY__STARTUP_GATE_HPP_
#define ADAS_CAN_GATEWAY__STARTUP_GATE_HPP_

#include <cstdint>

namespace adas::can_gateway {

// 启动/恢复门控（HIL 2026-07-13：primary_timeout=129ms 整改项 1~3）。
//
// 网关不允许一启动就宣告 ACTIVE：必须先连续发送 STANDBY 安全帧，等全部
// 上游输入（gate command / AEB / safety）同时新鲜并稳定保持 inputs_stable_s
// 后，才在一个发送周期边界统一切换 ACTIVE。ACTIVE 期间一旦发送节拍超期
// （线程暂停、USB/总线写失败）或任一输入陈旧，立即降级并重新走完整稳定
// 窗——禁止补发一串历史帧后继续 ACTIVE。降级后 MCU 按自身看门狗进入
// FAILSAFE，这是预期的 fail-closed 行为，不在网关侧掩盖。
struct StartupGateConfig {
  double inputs_stable_s{0.5};
};

class StartupGate {
 public:
  explicit StartupGate(const StartupGateConfig& config) : config_(config) {}

  // 每个发送节拍调用一次。inputs_fresh：全部上游输入新鲜；
  // deadline_ok：本节拍前未检测到任何一类帧的发送间隔超期。
  void update(double now_s, bool inputs_fresh, bool deadline_ok) {
    if (active_) {
      if (deadline_ok && inputs_fresh) return;
      active_ = false;
      ever_active_ = true;
      deadline_rearm_ = !deadline_ok;
      ++rearm_count_;
      fresh_since_s_ = -1.0;
      active_since_s_ = -1.0;
      return;
    }
    if (!deadline_ok || !inputs_fresh) {
      fresh_since_s_ = -1.0;
      return;
    }
    if (fresh_since_s_ < 0.0) fresh_since_s_ = now_s;
    if (now_s - fresh_since_s_ >= config_.inputs_stable_s) {
      active_ = true;
      active_since_s_ = now_s;
    }
  }

  bool active() const { return active_; }
  // 曾经 ACTIVE 过：降级时应持续请求 MRM（任务中 fail-closed），
  // 而不是回到冷启动的干净 STANDBY。
  bool ever_active() const { return ever_active_; }
  bool last_rearm_was_deadline() const { return deadline_rearm_; }
  std::uint64_t rearm_count() const { return rearm_count_; }
  double active_duration_s(double now_s) const {
    return (active_ && active_since_s_ >= 0.0) ? now_s - active_since_s_ : 0.0;
  }

 private:
  StartupGateConfig config_;
  bool active_{false};
  bool ever_active_{false};
  bool deadline_rearm_{false};
  std::uint64_t rearm_count_{0U};
  double fresh_since_s_{-1.0};
  double active_since_s_{-1.0};
};

}  // namespace adas::can_gateway

#endif  // ADAS_CAN_GATEWAY__STARTUP_GATE_HPP_
