#ifndef ADAS_CARLA_BRIDGE__CAN_PROTOCOL_HPP_
#define ADAS_CARLA_BRIDGE__CAN_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

namespace adas::carla_bridge {

constexpr std::uint32_t kMcuControlId = 0x201U;
constexpr std::uint32_t kMcuHeartbeatId = 0x202U;
constexpr std::uint32_t kMcuDiagId = 0x203U;
constexpr std::uint32_t kMcuE2eDiagId = 0x204U;
constexpr std::uint8_t kProtocolVersion = 0x02U;
constexpr std::uint8_t kSourcePrimary = 1U;
constexpr std::uint8_t kSourceWatchdog = 9U;
constexpr std::uint8_t kSystemInit = 0U;
constexpr std::uint8_t kSystemStandby = 1U;
constexpr std::size_t kRecoveryFrames = 3U;

struct Frame {
  std::uint32_t id{0U};
  std::array<std::uint8_t, 8> data{};
};

enum class FrameKind { kControl, kHeartbeat, kDiag, kE2e };

struct DecodedFrame {
  FrameKind kind{FrameKind::kDiag};
  double steer{0.0};
  double acceleration_mps2{0.0};
  double throttle{0.0};
  double brake{0.0};
  std::uint8_t sequence{0U};
  std::uint8_t state{0U};
  std::uint8_t active_source{0U};
  std::uint8_t protocol_version{0U};
};

struct ActuationSnapshot {
  double throttle{0.0};
  double brake{1.0};
  double steer{0.0};
  double age_s{0.0};
  bool stale{true};
  bool invalid_latched{true};
  std::uint64_t invalid_count{0U};
  std::uint64_t valid_count{0U};
};

std::uint8_t crc8(const std::uint8_t* data, std::size_t length);
std::uint8_t frame_crc(std::uint32_t can_id,
                       const std::array<std::uint8_t, 8>& data);
bool sequence_forward(std::uint8_t previous, std::uint8_t current);
std::optional<DecodedFrame> decode_frame(const Frame& frame,
                                         std::string* error = nullptr);

class McuFeedbackGuard {
 public:
  using Clock = std::function<double()>;

  explicit McuFeedbackGuard(double feedback_timeout_s = 0.1,
                            double heartbeat_timeout_s = 0.2,
                            double e2e_timeout_s = 0.5,
                            Clock clock = {});

  bool feed(const Frame& frame);
  ActuationSnapshot current();

 private:
  bool health_valid(double now_s) const;
  void reject();

  double feedback_timeout_s_;
  double heartbeat_timeout_s_;
  double e2e_timeout_s_;
  Clock clock_;
  mutable std::mutex mutex_;
  std::optional<DecodedFrame> control_;
  std::optional<DecodedFrame> heartbeat_;
  std::optional<DecodedFrame> e2e_;
  double control_rx_s_{0.0};
  double heartbeat_rx_s_{0.0};
  double e2e_rx_s_{0.0};
  std::optional<std::uint8_t> last_sequence_;
  bool invalid_latched_{true};
  std::size_t recovery_frames_{0U};
  std::uint64_t invalid_count_{0U};
  std::uint64_t valid_count_{0U};
};

}  // namespace adas::carla_bridge

#endif  // ADAS_CARLA_BRIDGE__CAN_PROTOCOL_HPP_
