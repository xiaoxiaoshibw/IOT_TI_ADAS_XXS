#ifndef ADAS_CAN_GATEWAY__HIL_SESSION_MANAGER_HPP_
#define ADAS_CAN_GATEWAY__HIL_SESSION_MANAGER_HPP_

#include <cstdint>

#include "adas_can_gateway/protocol.hpp"

namespace adas::can_gateway {

// v3 重构（2026-07）：移除 request_arm / request_rearm / arm_requested_。MCU 自驱
// 完成 ANNOUNCE→READY→ARMED→ACTIVE，网关只需按观测到的 0x206 ack 推进要发的
// 0x104 phase（SYNC_STANDBY→COMMIT_ACTIVE→ACTIVE keepalive）。RECOVERY 由 MCU 自
// 愈后重新上报 SESSION_ACCEPTED，网关据此清除本侧 recovery 闩锁——无需 TCP/服务
// 触发 REARM。fault_locked 仍由 MCU 上电自愈（HilSession_init→BOOT_WAIT id=0）解锁。
class HilSessionManager {
 public:
  explicit HilSessionManager(double status_timeout_s = 0.1);

  void begin(std::uint32_t session_id);
  bool observe(const SessionStatus& status, double now_s);
  void update(double now_s);
  Frame next_frame();

  bool control_authorized(double now_s) const;
  bool recovery_required() const { return recovery_required_; }
  bool fault_locked() const { return fault_locked_; }
  std::uint32_t session_id() const { return session_id_; }
  adas::can_protocol::SessionRequest request() const { return request_; }
  adas::can_protocol::SessionAck ack() const { return ack_; }

 private:
  double timeout_s_;
  std::uint32_t session_id_{0U};
  std::uint8_t sequence_{0U};
  adas::can_protocol::SessionRequest request_{
      adas::can_protocol::SessionRequest::kNone};
  adas::can_protocol::SessionAck ack_{
      adas::can_protocol::SessionAck::kBootWait};
  bool status_seen_{false};
  bool recovery_required_{false};
  bool fault_locked_{false};
  double last_status_s_{0.0};
};

}  // namespace adas::can_gateway

#endif  // ADAS_CAN_GATEWAY__HIL_SESSION_MANAGER_HPP_