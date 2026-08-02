#include "adas_carla_bridge/can_protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace adas::carla_bridge {
namespace {

double steady_now_s() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::int16_t get_i16(const std::array<std::uint8_t, 8>& data,
                     std::size_t offset) {
  const auto raw = static_cast<std::uint16_t>(data[offset]) |
                   static_cast<std::uint16_t>(data[offset + 1U] << 8U);
  return static_cast<std::int16_t>(raw);
}

void set_error(std::string* error, const char* value) {
  if (error != nullptr) *error = value;
}

}  // namespace

std::uint8_t crc8(const std::uint8_t* data, std::size_t length) {
  std::uint8_t value = 0U;
  for (std::size_t index = 0U; index < length; ++index) {
    value ^= data[index];
    for (int bit = 0; bit < 8; ++bit) {
      value = (value & 0x80U) != 0U
                  ? static_cast<std::uint8_t>((value << 1U) ^ 0x31U)
                  : static_cast<std::uint8_t>(value << 1U);
    }
  }
  return value;
}

std::uint8_t frame_crc(std::uint32_t can_id,
                       const std::array<std::uint8_t, 8>& data) {
  std::array<std::uint8_t, 9> protected_data{};
  protected_data[0] = static_cast<std::uint8_t>(can_id & 0xFFU);
  protected_data[1] = static_cast<std::uint8_t>((can_id >> 8U) & 0xFFU);
  std::copy_n(data.begin(), 7U, protected_data.begin() + 2U);
  return crc8(protected_data.data(), protected_data.size());
}

bool sequence_forward(std::uint8_t previous, std::uint8_t current) {
  const auto delta = static_cast<std::uint8_t>(current - previous);
  return delta > 0U && delta <= 127U;
}

std::optional<DecodedFrame> decode_frame(const Frame& frame,
                                         std::string* error) {
  if (frame.id != kMcuControlId && frame.id != kMcuHeartbeatId &&
      frame.id != kMcuDiagId && frame.id != kMcuE2eDiagId) {
    set_error(error, "unsupported_can_id");
    return std::nullopt;
  }
  if (frame.data[7] != frame_crc(frame.id, frame.data)) {
    set_error(error, "crc_or_data_id");
    return std::nullopt;
  }

  DecodedFrame result;
  if (frame.id == kMcuControlId) {
    if (frame.data[4] > 100U || frame.data[5] > 100U ||
        (frame.data[4] != 0U && frame.data[5] != 0U)) {
      set_error(error, "invalid_control_payload");
      return std::nullopt;
    }
    result.kind = FrameKind::kControl;
    result.steer = static_cast<double>(get_i16(frame.data, 0U)) * 0.01 / 30.0;
    result.acceleration_mps2 =
        static_cast<double>(get_i16(frame.data, 2U)) * 0.001;
    result.throttle = static_cast<double>(frame.data[4]) * 0.01;
    result.brake = static_cast<double>(frame.data[5]) * 0.01;
    result.sequence = frame.data[6];
  } else if (frame.id == kMcuHeartbeatId) {
    result.kind = FrameKind::kHeartbeat;
    result.state = frame.data[0];
    result.active_source = frame.data[1];
    result.sequence = frame.data[4];
  } else if (frame.id == kMcuE2eDiagId) {
    result.kind = FrameKind::kE2e;
    result.protocol_version = frame.data[6];
  } else {
    result.kind = FrameKind::kDiag;
  }
  if (error != nullptr) error->clear();
  return result;
}

McuFeedbackGuard::McuFeedbackGuard(double feedback_timeout_s,
                                   double heartbeat_timeout_s,
                                   double e2e_timeout_s, Clock clock)
    : feedback_timeout_s_(feedback_timeout_s),
      heartbeat_timeout_s_(heartbeat_timeout_s),
      e2e_timeout_s_(e2e_timeout_s),
      clock_(clock ? std::move(clock) : Clock(steady_now_s)) {
  if (!std::isfinite(feedback_timeout_s_) || feedback_timeout_s_ <= 0.0 ||
      !std::isfinite(heartbeat_timeout_s_) || heartbeat_timeout_s_ <= 0.0 ||
      !std::isfinite(e2e_timeout_s_) || e2e_timeout_s_ <= 0.0) {
    throw std::invalid_argument("MCU feedback timeouts must be finite and positive");
  }
}

void McuFeedbackGuard::reject() {
  invalid_latched_ = true;
  recovery_frames_ = 0U;
  ++invalid_count_;
}

bool McuFeedbackGuard::feed(const Frame& frame) {
  const double now_s = clock_();
  const auto decoded = decode_frame(frame);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!decoded) {
    reject();
    return false;
  }
  switch (decoded->kind) {
    case FrameKind::kControl:
      if (last_sequence_ &&
          !sequence_forward(*last_sequence_, decoded->sequence)) {
        reject();
        return false;
      }
      last_sequence_ = decoded->sequence;
      control_ = decoded;
      control_rx_s_ = now_s;
      ++valid_count_;
      if (health_valid(now_s)) {
        ++recovery_frames_;
        if (recovery_frames_ >= kRecoveryFrames) invalid_latched_ = false;
      }
      break;
    case FrameKind::kHeartbeat:
      heartbeat_ = decoded;
      heartbeat_rx_s_ = now_s;
      break;
    case FrameKind::kE2e:
      e2e_ = decoded;
      e2e_rx_s_ = now_s;
      if (decoded->protocol_version != kProtocolVersion) reject();
      break;
    case FrameKind::kDiag:
      break;
  }
  return true;
}

bool McuFeedbackGuard::health_valid(double now_s) const {
  if (!heartbeat_ || !e2e_) return false;
  if ((now_s - heartbeat_rx_s_) > heartbeat_timeout_s_ ||
      (now_s - e2e_rx_s_) > e2e_timeout_s_) {
    return false;
  }
  if (e2e_->protocol_version != kProtocolVersion) return false;
  if (heartbeat_->active_source != kSourcePrimary &&
      heartbeat_->active_source != kSourceWatchdog) {
    return false;
  }
  return heartbeat_->state != kSystemInit && heartbeat_->state != kSystemStandby;
}

ActuationSnapshot McuFeedbackGuard::current() {
  const double now_s = clock_();
  std::lock_guard<std::mutex> lock(mutex_);
  const double age_s = control_ ? now_s - control_rx_s_
                                : std::numeric_limits<double>::infinity();
  const bool stale = !control_ || age_s > feedback_timeout_s_;
  const bool healthy = control_ && !stale && health_valid(now_s) &&
                       !invalid_latched_;
  if (!healthy) {
    if (control_ && stale) {
      invalid_latched_ = true;
      recovery_frames_ = 0U;
    }
    return {0.0, 1.0, 0.0, age_s, stale, invalid_latched_, invalid_count_,
            valid_count_};
  }
  return {control_->throttle, control_->brake,
          std::clamp(control_->steer, -1.0, 1.0), age_s, false, false,
          invalid_count_, valid_count_};
}

}  // namespace adas::carla_bridge
