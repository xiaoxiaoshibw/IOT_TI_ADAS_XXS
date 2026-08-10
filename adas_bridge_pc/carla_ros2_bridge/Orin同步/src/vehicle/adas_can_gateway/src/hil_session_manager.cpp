#include "adas_can_gateway/hil_session_manager.hpp"

#include <stdexcept>

namespace adas::can_gateway {

HilSessionManager::HilSessionManager(double status_timeout_s)
    : timeout_s_(status_timeout_s) {
  if (timeout_s_ <= 0.0) throw std::invalid_argument("status timeout must be positive");
}

void HilSessionManager::begin(std::uint32_t id) {
  if (id == 0U) throw std::invalid_argument("session id must be non-zero");
  session_id_ = id;
  sequence_ = 0U;
  request_ = adas::can_protocol::SessionRequest::kAnnounce;
  ack_ = adas::can_protocol::SessionAck::kBootWait;
  status_seen_ = false;
  recovery_required_ = false;
  fault_locked_ = false;
}

bool HilSessionManager::observe(const SessionStatus& status, double now_s) {
  if (status.session_id != session_id_) {
    if (status.ack == adas::can_protocol::SessionAck::kBootWait &&
        status.session_id == 0U) {
      // A local MCU long-press/reboot clears its session.  Resume ANNOUNCE so
      // both peers can recover without restarting the gateway process.  In the
      // v3 auto-arming model the MCU also self-reinitializes to BOOT_WAIT after
      // its safety FAULT_LOCK heals, so this path is the normal fault recovery.
      fault_locked_ = false;
      recovery_required_ = false;
      sequence_ = 0U;
      request_ = adas::can_protocol::SessionRequest::kAnnounce;
      ack_ = adas::can_protocol::SessionAck::kBootWait;
    } else if (status.ack == adas::can_protocol::SessionAck::kRecoveryRequired) {
      // The MCU continues publishing the stale session's RECOVERY status until
      // it self-heals back to SESSION_ACCEPTED with the same id.  Latch local
      // recovery so control_authorized stays false until that recovery ack arrives.
      recovery_required_ = true;
      request_ = adas::can_protocol::SessionRequest::kNone;
    } else if (status.ack == adas::can_protocol::SessionAck::kFaultLock) {
      fault_locked_ = true;
      recovery_required_ = false;
      request_ = adas::can_protocol::SessionRequest::kNone;
    }
    return false;
  }
  status_seen_ = true;
  last_status_s_ = now_s;
  ack_ = status.ack;
  switch (status.ack) {
    case adas::can_protocol::SessionAck::kSessionAccepted:
      request_ = adas::can_protocol::SessionRequest::kSyncStandby;
      recovery_required_ = false;
      break;
    case adas::can_protocol::SessionAck::kReady:
      // MCU auto-arms without an explicit PREPARE_ARM; keep priming STANDBY
      // until it reports ARMED.  COMMIT follows on the observed ARMED ack.
      request_ = adas::can_protocol::SessionRequest::kSyncStandby;
      break;
    case adas::can_protocol::SessionAck::kArmed:
      request_ = adas::can_protocol::SessionRequest::kCommitActive;
      break;
    case adas::can_protocol::SessionAck::kActive:
      request_ = adas::can_protocol::SessionRequest::kActive;
      break;
    case adas::can_protocol::SessionAck::kRecoveryRequired:
      recovery_required_ = true;
      request_ = adas::can_protocol::SessionRequest::kNone;
      break;
    case adas::can_protocol::SessionAck::kFaultLock:
      fault_locked_ = true;
      recovery_required_ = false;
      request_ = adas::can_protocol::SessionRequest::kNone;
      break;
    case adas::can_protocol::SessionAck::kBootWait:
      request_ = adas::can_protocol::SessionRequest::kAnnounce;
      break;
    case adas::can_protocol::SessionAck::kRejected:
      break;
  }
  return true;
}

void HilSessionManager::update(double now_s) {
  if (status_seen_ && (now_s - last_status_s_) > timeout_s_) {
    status_seen_ = false;
    if (ack_ == adas::can_protocol::SessionAck::kActive) {
      ack_ = adas::can_protocol::SessionAck::kRecoveryRequired;
      recovery_required_ = true;
      request_ = adas::can_protocol::SessionRequest::kNone;
    }
  }
}

Frame HilSessionManager::next_frame() {
  if (session_id_ == 0U || request_ == adas::can_protocol::SessionRequest::kNone) {
    throw std::logic_error("HIL session has no frame to send");
  }
  const auto frame = encode_session_control(request_, session_id_, sequence_);
  sequence_ = static_cast<std::uint8_t>(sequence_ + 1U);
  return frame;
}

bool HilSessionManager::control_authorized(double now_s) const {
  return !fault_locked_ && !recovery_required_ && status_seen_ &&
         ack_ == adas::can_protocol::SessionAck::kActive &&
         (now_s - last_status_s_) <= timeout_s_;
}

}  // namespace adas::can_gateway