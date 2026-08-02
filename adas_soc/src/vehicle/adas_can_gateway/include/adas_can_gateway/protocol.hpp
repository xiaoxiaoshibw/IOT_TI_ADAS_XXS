#ifndef ADAS_CAN_GATEWAY__PROTOCOL_HPP_
#define ADAS_CAN_GATEWAY__PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "adas_can_gateway/protocol_v3_generated.hpp"

namespace adas::can_gateway {

constexpr std::uint32_t kPrimaryBase = 0x100U;
constexpr std::uint32_t kMcuControlId = 0x201U;
constexpr std::uint32_t kMcuHeartbeatId = 0x202U;
constexpr std::uint32_t kMcuDiagId = 0x203U;
constexpr std::uint32_t kMcuE2eDiagId = 0x204U;
constexpr std::uint8_t kProtocolVersion = adas::can_protocol::kProtocolVersion;

constexpr std::uint16_t kStatusControlEnable = 1U << 0;
constexpr std::uint16_t kStatusLateralEnable = 1U << 1;
constexpr std::uint16_t kStatusLongitudinalEnable = 1U << 2;
constexpr std::uint16_t kStatusLkaActive = 1U << 3;
constexpr std::uint16_t kStatusAccActive = 1U << 4;
constexpr std::uint16_t kStatusAebWarning = 1U << 5;
constexpr std::uint16_t kStatusAebBraking = 1U << 6;
constexpr std::uint16_t kStatusEmergencyStop = 1U << 7;
constexpr std::uint16_t kStatusDegraded = 1U << 8;
constexpr std::uint16_t kStatusMrmRequest = 1U << 10;

enum class SystemMode : std::uint8_t {
  kInit = 0,
  kStandby = 1,
  kActive = 2,
  kDegraded = 3,
  kMrm = 4,
  kEmergencyBrake = 5,
  kFailsafe = 6,
  kFaultLock = 7,
};

enum class AebRisk : std::uint8_t {
  kNone = 0,
  kWarning = 1,
  kPartial = 2,
  kFull = 3,
};

struct Frame {
  std::uint32_t id{0U};
  std::array<std::uint8_t, 8> data{};
};

struct CommandSnapshot {
  double target_steer_deg{0.0};
  double target_steer_rate_dps{0.0};
  double target_accel_mps2{0.0};
  double target_speed_mps{0.0};
  std::uint8_t brake_request_pct{0U};
  bool lateral_enable{false};
  bool longitudinal_enable{false};
  bool lka_active{false};
  bool acc_active{false};
  std::uint16_t status_word{0U};
  AebRisk aeb_risk{AebRisk::kNone};
  double aeb_required_decel_mps2{0.0};
  bool emergency_stop{false};
  bool mrm_request{false};
  bool obstacle_valid{false};
  SystemMode system_mode{SystemMode::kStandby};
  std::uint8_t soc_health{0U};
  std::uint8_t fault_level{0U};
  bool control_authority{false};
};

struct ControlFeedback {
  double steer_deg{0.0};
  double accel_mps2{0.0};
  double throttle{0.0};
  double brake{0.0};
  std::uint8_t sequence{0U};
};

struct McuHeartbeat {
  std::uint8_t state{0U};
  std::uint8_t active_source{0U};
  std::uint8_t safety_flags{0U};
  std::uint8_t fault_level{0U};
  std::uint8_t sequence{0U};
  std::uint8_t alive{0U};
  std::uint8_t loop_load_pct{0U};
};

// 0x202 byte1 active_source（与 MCU adas_can_protocol.h 的 SRC_* 一致）
constexpr std::uint8_t kSourceNone = 0U;
constexpr std::uint8_t kSourcePrimary = 1U;
constexpr std::uint8_t kSourceBackup = 2U;
constexpr std::uint8_t kSourceWatchdog = 9U;

// 0x202 safety_flags 位定义（与 MCU adas_can_protocol.h 的 SF_* 一致）
constexpr std::uint8_t kSafetyPrimaryFresh = 1U << 0;
constexpr std::uint8_t kSafetyBackupFresh = 1U << 1;
constexpr std::uint8_t kSafetyAebFloor = 1U << 2;
constexpr std::uint8_t kSafetyDegraded = 1U << 3;
constexpr std::uint8_t kSafetyManualOverride = 1U << 5;
constexpr std::uint8_t kSafetyEstop = 1U << 6;

struct McuDiag {
  std::uint16_t fault_code{0U};      // FC_* 位图（safety.h）
  std::uint8_t reset_reason{0U};
  std::uint8_t primary_timeouts{0U};
  std::uint8_t backup_timeouts{0U};
  std::uint8_t crc_errors{0U};
  std::uint8_t loop_overruns{0U};
};

struct McuE2eDiag {
  std::uint8_t primary_seq_errors{0U};
  std::uint8_t backup_seq_errors{0U};
  std::uint8_t proto_flags{0U};
  std::uint8_t can_recovery_count{0U};
  std::uint8_t self_test_mask{0U};
  std::uint8_t build_flags{0U};
  std::uint8_t version{0U};
};

struct SessionStatus {
  adas::can_protocol::SessionAck ack{adas::can_protocol::SessionAck::kBootWait};
  std::uint32_t session_id{0U};
  std::uint8_t sequence{0U};
};

std::uint8_t crc8(const std::uint8_t* data, std::size_t length);
std::uint8_t frame_crc(std::uint32_t can_id,
                       const std::array<std::uint8_t, 8>& data);
bool validate_frame(const Frame& frame);

// 四组发送序号 + alive 计数的快照，用于跨进程重启持久化：
// 网关重启不得从 0 静默重置序号，否则 MCU 会把接管首帧判为倒退。
struct EncoderSequences {
  std::uint8_t heartbeat{0U};
  std::uint8_t lateral{0U};
  std::uint8_t longitudinal{0U};
  std::uint8_t status{0U};
  std::uint8_t alive{0U};
};

class Encoder {
 public:
  Frame heartbeat(const CommandSnapshot& command);
  Frame lateral(const CommandSnapshot& command);
  Frame longitudinal(const CommandSnapshot& command);
  Frame status(const CommandSnapshot& command);
  EncoderSequences sequences() const;
  void restore(const EncoderSequences& state);

 private:
  static std::uint8_t next(std::uint8_t& counter);
  std::uint8_t heartbeat_sequence_{0U};
  std::uint8_t lateral_sequence_{0U};
  std::uint8_t longitudinal_sequence_{0U};
  std::uint8_t status_sequence_{0U};
  std::uint8_t alive_{0U};
};

std::optional<ControlFeedback> decode_control_feedback(const Frame& frame);
std::optional<McuHeartbeat> decode_mcu_heartbeat(const Frame& frame);
std::optional<McuDiag> decode_mcu_diag(const Frame& frame);
std::optional<McuE2eDiag> decode_mcu_e2e_diag(const Frame& frame);
Frame encode_session_control(adas::can_protocol::SessionRequest request,
                             std::uint32_t session_id, std::uint8_t sequence);
std::optional<SessionStatus> decode_session_status(const Frame& frame);
bool sequence_forward(std::uint8_t previous, std::uint8_t current);

}  // namespace adas::can_gateway

#endif  // ADAS_CAN_GATEWAY__PROTOCOL_HPP_
