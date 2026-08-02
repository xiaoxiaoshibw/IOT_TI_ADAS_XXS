#include "adas_can_gateway/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace adas::can_gateway {
namespace {

constexpr double kSteerScale = 0.01;
constexpr double kSteerRateScale = 0.1;
constexpr double kAccelScale = 0.001;
constexpr double kSpeedScale = 0.01;

std::int16_t quantize(double value, double scale) {
  const double raw = value / scale;
  const double rounded = raw >= 0.0 ? std::floor(raw + 0.5) : std::ceil(raw - 0.5);
  return static_cast<std::int16_t>(std::clamp(rounded, -32768.0, 32767.0));
}

void put_i16(std::array<std::uint8_t, 8>& data, std::size_t offset,
             std::int16_t value) {
  const auto raw = static_cast<std::uint16_t>(value);
  data[offset] = static_cast<std::uint8_t>(raw & 0xFFU);
  data[offset + 1U] = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
}

std::int16_t get_i16(const std::array<std::uint8_t, 8>& data,
                     std::size_t offset) {
  const auto raw = static_cast<std::uint16_t>(data[offset]) |
                   static_cast<std::uint16_t>(data[offset + 1U] << 8U);
  return static_cast<std::int16_t>(raw);
}

void finish(Frame& frame) { frame.data[7] = frame_crc(frame.id, frame.data); }

}  // namespace

std::uint8_t crc8(const std::uint8_t* data, std::size_t length) {
  std::uint8_t crc = 0U;
  for (std::size_t index = 0U; index < length; ++index) {
    crc ^= data[index];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80U) != 0U
                ? static_cast<std::uint8_t>((crc << 1U) ^ 0x31U)
                : static_cast<std::uint8_t>(crc << 1U);
    }
  }
  return crc;
}

std::uint8_t frame_crc(std::uint32_t can_id,
                       const std::array<std::uint8_t, 8>& data) {
  std::array<std::uint8_t, 9> protected_data{};
  protected_data[0] = static_cast<std::uint8_t>(can_id & 0xFFU);
  protected_data[1] = static_cast<std::uint8_t>((can_id >> 8U) & 0xFFU);
  std::copy_n(data.begin(), 7U, protected_data.begin() + 2);
  return crc8(protected_data.data(), protected_data.size());
}

bool validate_frame(const Frame& frame) {
  return frame.id <= 0x7FFU && frame.data[7] == frame_crc(frame.id, frame.data);
}

EncoderSequences Encoder::sequences() const {
  return EncoderSequences{heartbeat_sequence_, lateral_sequence_,
                          longitudinal_sequence_, status_sequence_, alive_};
}

void Encoder::restore(const EncoderSequences& state) {
  heartbeat_sequence_ = state.heartbeat;
  lateral_sequence_ = state.lateral;
  longitudinal_sequence_ = state.longitudinal;
  status_sequence_ = state.status;
  alive_ = state.alive;
}

std::uint8_t Encoder::next(std::uint8_t& counter) {
  const auto current = counter;
  counter = static_cast<std::uint8_t>(counter + 1U);
  return current;
}

Frame Encoder::heartbeat(const CommandSnapshot& command) {
  Frame frame{kPrimaryBase, {}};
  frame.data[0] = static_cast<std::uint8_t>(command.system_mode);
  frame.data[1] = command.soc_health;
  frame.data[2] = command.fault_level;
  frame.data[3] = static_cast<std::uint8_t>((kProtocolVersion << 4U) |
      (command.control_authority ? 0x01U : 0x00U));
  frame.data[4] = 1U;
  frame.data[5] = next(heartbeat_sequence_);
  alive_ = static_cast<std::uint8_t>(alive_ + 1U);
  frame.data[6] = alive_;
  finish(frame);
  return frame;
}

Frame Encoder::lateral(const CommandSnapshot& command) {
  Frame frame{kPrimaryBase + 1U, {}};
  put_i16(frame.data, 0U, quantize(command.target_steer_deg, kSteerScale));
  put_i16(frame.data, 2U, quantize(command.target_steer_rate_dps, kSteerRateScale));
  frame.data[4] = static_cast<std::uint8_t>(
      (command.lateral_enable ? 0x01U : 0U) |
      (command.lka_active ? 0x02U : 0U));
  frame.data[5] = next(lateral_sequence_);
  finish(frame);
  return frame;
}

Frame Encoder::longitudinal(const CommandSnapshot& command) {
  Frame frame{kPrimaryBase + 2U, {}};
  put_i16(frame.data, 0U, quantize(command.target_accel_mps2, kAccelScale));
  put_i16(frame.data, 2U, quantize(command.target_speed_mps, kSpeedScale));
  frame.data[4] = std::min<std::uint8_t>(command.brake_request_pct, 100U);
  frame.data[5] = static_cast<std::uint8_t>(
      (command.longitudinal_enable ? 0x01U : 0U) |
      (command.acc_active ? 0x02U : 0U) | (1U << 4U));
  frame.data[6] = next(longitudinal_sequence_);
  finish(frame);
  return frame;
}

Frame Encoder::status(const CommandSnapshot& command) {
  Frame frame{kPrimaryBase + 3U, {}};
  frame.data[0] = static_cast<std::uint8_t>(command.status_word & 0xFFU);
  frame.data[1] = static_cast<std::uint8_t>((command.status_word >> 8U) & 0xFFU);
  frame.data[2] = static_cast<std::uint8_t>(command.aeb_risk);
  put_i16(frame.data, 3U,
          quantize(std::abs(command.aeb_required_decel_mps2), kAccelScale));
  frame.data[5] = static_cast<std::uint8_t>(
      (command.emergency_stop ? 0x01U : 0U) |
      (command.mrm_request ? 0x02U : 0U) |
      (command.obstacle_valid ? 0x04U : 0U));
  frame.data[6] = next(status_sequence_);
  finish(frame);
  return frame;
}

std::optional<ControlFeedback> decode_control_feedback(const Frame& frame) {
  if (frame.id != kMcuControlId || !validate_frame(frame) ||
      frame.data[4] > 100U || frame.data[5] > 100U ||
      (frame.data[4] > 0U && frame.data[5] > 0U)) {
    return std::nullopt;
  }
  return ControlFeedback{
      static_cast<double>(get_i16(frame.data, 0U)) * kSteerScale,
      static_cast<double>(get_i16(frame.data, 2U)) * kAccelScale,
      static_cast<double>(frame.data[4]) * 0.01,
      static_cast<double>(frame.data[5]) * 0.01,
      frame.data[6]};
}

std::optional<McuHeartbeat> decode_mcu_heartbeat(const Frame& frame) {
  if (frame.id != kMcuHeartbeatId || !validate_frame(frame)) return std::nullopt;
  return McuHeartbeat{frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                      frame.data[4], frame.data[5], frame.data[6]};
}

std::optional<McuDiag> decode_mcu_diag(const Frame& frame) {
  if (frame.id != kMcuDiagId || !validate_frame(frame)) return std::nullopt;
  return McuDiag{
      static_cast<std::uint16_t>(frame.data[0] |
                                 (static_cast<std::uint16_t>(frame.data[1]) << 8)),
      frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6]};
}

std::optional<McuE2eDiag> decode_mcu_e2e_diag(const Frame& frame) {
  if (frame.id != kMcuE2eDiagId || !validate_frame(frame)) return std::nullopt;
  return McuE2eDiag{frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                    frame.data[4], frame.data[5], frame.data[6]};
}

Frame encode_session_control(adas::can_protocol::SessionRequest request,
                             std::uint32_t session_id, std::uint8_t sequence) {
  Frame frame{adas::can_protocol::kCanIdPrimarySessionControl, {}};
  frame.data[0] = adas::can_protocol::kProtocolVersion;
  frame.data[1] = static_cast<std::uint8_t>(request);
  frame.data[2] = static_cast<std::uint8_t>(session_id & 0xFFU);
  frame.data[3] = static_cast<std::uint8_t>((session_id >> 8U) & 0xFFU);
  frame.data[4] = static_cast<std::uint8_t>((session_id >> 16U) & 0xFFU);
  frame.data[5] = static_cast<std::uint8_t>((session_id >> 24U) & 0xFFU);
  frame.data[6] = sequence;
  finish(frame);
  return frame;
}

std::optional<SessionStatus> decode_session_status(const Frame& frame) {
  if (frame.id != adas::can_protocol::kCanIdMcuSessionStatus ||
      !validate_frame(frame) ||
      frame.data[0] != adas::can_protocol::kProtocolVersion ||
      frame.data[1] > static_cast<std::uint8_t>(
          adas::can_protocol::SessionAck::kFaultLock)) {
    return std::nullopt;
  }
  const auto id = static_cast<std::uint32_t>(frame.data[2]) |
                  (static_cast<std::uint32_t>(frame.data[3]) << 8U) |
                  (static_cast<std::uint32_t>(frame.data[4]) << 16U) |
                  (static_cast<std::uint32_t>(frame.data[5]) << 24U);
  return SessionStatus{
      static_cast<adas::can_protocol::SessionAck>(frame.data[1]), id,
      frame.data[6]};
}

bool sequence_forward(std::uint8_t previous, std::uint8_t current) {
  const auto delta = static_cast<std::uint8_t>(current - previous);
  return delta > 0U && delta <= 127U;
}

}  // namespace adas::can_gateway
