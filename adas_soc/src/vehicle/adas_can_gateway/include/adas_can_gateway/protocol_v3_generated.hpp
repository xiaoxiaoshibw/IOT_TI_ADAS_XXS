// Generated from protocol/adas_can_v3.yaml.
// Do not edit by hand; run tools/generate_protocol.py.
#pragma once

#include <cstdint>

namespace adas::can_protocol {

inline constexpr std::uint8_t kProtocolVersion = 3U;
inline constexpr std::uint32_t kBitrateBps = 500000U;
inline constexpr std::uint32_t kCanIdPrimaryHeartbeat = 0x100U;
inline constexpr std::uint32_t kCanIdPrimaryLateral = 0x101U;
inline constexpr std::uint32_t kCanIdPrimaryLongitudinal = 0x102U;
inline constexpr std::uint32_t kCanIdPrimaryStatus = 0x103U;
inline constexpr std::uint32_t kCanIdPrimarySessionControl = 0x104U;
inline constexpr std::uint32_t kCanIdBackupHeartbeat = 0x110U;
inline constexpr std::uint32_t kCanIdBackupLateral = 0x111U;
inline constexpr std::uint32_t kCanIdBackupLongitudinal = 0x112U;
inline constexpr std::uint32_t kCanIdBackupStatus = 0x113U;
inline constexpr std::uint32_t kCanIdBackupSessionControl = 0x114U;
inline constexpr std::uint32_t kCanIdMcuControl = 0x201U;
inline constexpr std::uint32_t kCanIdMcuHeartbeat = 0x202U;
inline constexpr std::uint32_t kCanIdMcuDiag = 0x203U;
inline constexpr std::uint32_t kCanIdMcuE2eDiag = 0x204U;
inline constexpr std::uint32_t kCanIdMcuLinkStats = 0x205U;
inline constexpr std::uint32_t kCanIdMcuSessionStatus = 0x206U;
inline constexpr std::uint32_t kCanIdFaultInject = 0x301U;
inline constexpr std::uint32_t kCanIdFaultResponse = 0x302U;

enum class SessionRequest : std::uint8_t {
  kNone = 0U,
  kAnnounce = 1U,
  kSyncStandby = 2U,
  kPrepareArm = 3U,
  kCommitActive = 4U,
  kActive = 5U,
  kRearm = 6U,
  kAbort = 7U,
};

enum class SessionAck : std::uint8_t {
  kBootWait = 0U,
  kSessionAccepted = 1U,
  kReady = 2U,
  kArmed = 3U,
  kActive = 4U,
  kRecoveryRequired = 5U,
  kRejected = 6U,
  kFaultLock = 7U,
};

enum class SessionRejectReason : std::uint8_t {
  kNone = 0U,
  kProtocolMismatch = 1U,
  kSessionMismatch = 2U,
  kFramesNotFresh = 3U,
  kUnsafeToArm = 4U,
  kOperatorAuthorizationRequired = 5U,
  kFaultNotRecoverable = 6U,
  kInvalidTransition = 7U,
  kInvalidSequence = 8U,
};

}  // namespace adas::can_protocol
