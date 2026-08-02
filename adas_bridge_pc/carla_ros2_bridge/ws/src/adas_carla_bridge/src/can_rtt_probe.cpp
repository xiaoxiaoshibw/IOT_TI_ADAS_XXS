#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "adas_carla_bridge/can_protocol.hpp"

namespace acb = adas::carla_bridge;

namespace {

constexpr std::uint32_t kPrimaryHeartbeat = 0x100U;
constexpr std::uint32_t kPrimaryLateral = 0x101U;
constexpr std::uint32_t kPrimaryLongitudinal = 0x102U;
constexpr std::uint32_t kPrimaryStatus = 0x103U;

struct Options {
  std::string interface{"can0"};
  std::string csv_path{"can_rtt_samples.csv"};
  int samples{100};
  double step_deg{2.0};
  int settle_ms{80};
  int takeover_gap_ms{20};
  int arm_timeout_s{120};
};

double raw_now_s() {
  timespec stamp{};
  clock_gettime(CLOCK_MONOTONIC_RAW, &stamp);
  return static_cast<double>(stamp.tv_sec) +
         static_cast<double>(stamp.tv_nsec) * 1.0e-9;
}

std::int16_t get_i16(const std::uint8_t* data) {
  return static_cast<std::int16_t>(static_cast<std::uint16_t>(data[0]) |
                                   (static_cast<std::uint16_t>(data[1]) << 8U));
}

void put_i16(std::array<std::uint8_t, 8>& data, std::size_t offset,
             std::int16_t value) {
  const auto raw = static_cast<std::uint16_t>(value);
  data[offset] = static_cast<std::uint8_t>(raw & 0xFFU);
  data[offset + 1U] = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
}

can_frame finish(std::uint32_t id, std::array<std::uint8_t, 8> data) {
  data[7] = acb::frame_crc(id, data);
  can_frame frame{};
  frame.can_id = id;
  frame.can_dlc = 8U;
  std::copy(data.begin(), data.end(), frame.data);
  return frame;
}

Options parse_options(int argc, char** argv) {
  Options result;
  for (int index = 1; index < argc; ++index) {
    const std::string key = argv[index];
    auto value = [&]() -> std::string {
      if (++index >= argc) throw std::invalid_argument("missing value for " + key);
      return argv[index];
    };
    if (key == "--interface") result.interface = value();
    else if (key == "--csv") result.csv_path = value();
    else if (key == "--samples") result.samples = std::stoi(value());
    else if (key == "--step-deg") result.step_deg = std::stod(value());
    else if (key == "--settle-ms") result.settle_ms = std::stoi(value());
    else if (key == "--takeover-gap-ms") result.takeover_gap_ms = std::stoi(value());
    else if (key == "--arm-timeout-s") result.arm_timeout_s = std::stoi(value());
    else if (key == "--help") {
      std::cout << "Usage: can_rtt_probe [--interface can0] [--samples 100] "
                   "[--csv path] [--step-deg 2] [--settle-ms 80]\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown argument: " + key);
    }
  }
  if (result.samples < 0 || !std::isfinite(result.step_deg) ||
      result.step_deg <= 0.1 || result.step_deg > 5.0 || result.settle_ms < 20 ||
      result.takeover_gap_ms < 10 || result.arm_timeout_s <= 0) {
    throw std::invalid_argument("invalid probe options");
  }
  return result;
}

int open_can(const std::string& name) {
  const int fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (fd < 0) throw std::runtime_error(std::strerror(errno));
  const int receive_own = 0;
  setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &receive_own,
             sizeof(receive_own));
  ifreq request{};
  std::strncpy(request.ifr_name, name.c_str(), IFNAMSIZ - 1U);
  if (ioctl(fd, SIOCGIFINDEX, &request) < 0) {
    const std::string error = std::strerror(errno);
    close(fd);
    throw std::runtime_error(error);
  }
  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
    const std::string error = std::strerror(errno);
    close(fd);
    throw std::runtime_error(error);
  }
  return fd;
}

bool write_frame(int fd, const can_frame& frame) {
  const ssize_t size = write(fd, &frame, sizeof(frame));
  if (size == static_cast<ssize_t>(sizeof(frame))) return true;
  if (size < 0 && (errno == EAGAIN || errno == ENOBUFS)) return false;
  throw std::runtime_error("SocketCAN write failed: " +
                           std::string(std::strerror(errno)));
}

class SafePrimarySender {
 public:
  explicit SafePrimarySender(int fd) : fd_(fd) {}

  void observe(const can_frame& frame) {
    const auto id = frame.can_id & CAN_SFF_MASK;
    if (id == kPrimaryHeartbeat) {
      heartbeat_seq_ = static_cast<std::uint8_t>(frame.data[5] + 1U);
      alive_ = static_cast<std::uint8_t>(frame.data[6] + 1U);
    } else if (id == kPrimaryLateral) {
      lateral_seq_ = static_cast<std::uint8_t>(frame.data[5] + 1U);
    } else if (id == kPrimaryLongitudinal) {
      longitudinal_seq_ = static_cast<std::uint8_t>(frame.data[6] + 1U);
    } else if (id == kPrimaryStatus) {
      status_seq_ = static_cast<std::uint8_t>(frame.data[6] + 1U);
    }
  }

  std::optional<double> tick(std::uint64_t tick, double steer_deg, bool active) {
    std::array<std::uint8_t, 8> lateral{};
    put_i16(lateral, 0U, static_cast<std::int16_t>(std::lround(steer_deg * 100.0)));
    lateral[4] = active ? 0x03U : 0U;
    lateral[5] = lateral_seq_++;
    const auto lateral_frame = finish(kPrimaryLateral, lateral);
    const double lateral_tx_s = raw_now_s();
    const bool lateral_sent = write_frame(fd_, lateral_frame);

    std::array<std::uint8_t, 8> longitudinal{};
    longitudinal[4] = 100U;  // Keep CARLA/actuator safely stopped during probe.
    longitudinal[5] = active ? 0x13U : 0x10U;
    longitudinal[6] = longitudinal_seq_++;
    write_frame(fd_, finish(kPrimaryLongitudinal, longitudinal));

    if ((tick & 1U) == 0U) {
      std::array<std::uint8_t, 8> heartbeat{};
      heartbeat[0] = active ? 2U : 1U;
      heartbeat[1] = 1U;     // SoC healthy
      heartbeat[3] = active ? 0x21U : 0x20U;
      heartbeat[4] = 1U;
      heartbeat[5] = heartbeat_seq_++;
      heartbeat[6] = alive_++;
      write_frame(fd_, finish(kPrimaryHeartbeat, heartbeat));

      std::array<std::uint8_t, 8> status{};
      status[0] = active ? 0x1FU : 0U;
      status[6] = status_seq_++;
      write_frame(fd_, finish(kPrimaryStatus, status));
    }
    return lateral_sent ? std::optional<double>(lateral_tx_s) : std::nullopt;
  }

 private:
  int fd_;
  std::uint8_t heartbeat_seq_{0U};
  std::uint8_t lateral_seq_{0U};
  std::uint8_t longitudinal_seq_{0U};
  std::uint8_t status_seq_{0U};
  std::uint8_t alive_{0U};
};

double percentile(const std::vector<double>& sorted, double fraction) {
  if (sorted.empty()) return 0.0;
  const double position = fraction * static_cast<double>(sorted.size() - 1U);
  const auto low = static_cast<std::size_t>(std::floor(position));
  const auto high = static_cast<std::size_t>(std::ceil(position));
  const double alpha = position - static_cast<double>(low);
  return sorted[low] * (1.0 - alpha) + sorted[high] * alpha;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    const int fd = open_can(options.interface);
    SafePrimarySender sender(fd);
    double last_primary_s = raw_now_s();
    const double arm_deadline = last_primary_s + options.arm_timeout_s;
    std::cout << "ARMED interface=" << options.interface
              << " waiting for Orin primary gap > " << options.takeover_gap_ms
              << " ms" << std::endl;

    bool takeover = false;
    while (raw_now_s() < arm_deadline && !takeover) {
      can_frame frame{};
      while (read(fd, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame))) {
        const auto id = frame.can_id & CAN_SFF_MASK;
        if (id >= kPrimaryHeartbeat && id <= kPrimaryStatus) {
          sender.observe(frame);
          last_primary_s = raw_now_s();
        }
      }
      takeover = (raw_now_s() - last_primary_s) * 1000.0 > options.takeover_gap_ms;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (!takeover) throw std::runtime_error("timed out waiting for Orin handoff");
    std::cout << "TAKEOVER safe brake=100%" << std::endl;

    // Force a deterministic, fail-closed startup transition. If the MCU was
    // already in FAILSAFE, an external RESET is intentionally required while
    // these STANDBY frames are present; no software fault-clear is injected.
    std::uint64_t tick = 0U;
    double next_tick_s = raw_now_s();
    auto align_frozen_primary = [&]() {
      std::cout << "ALIGNING_FROZEN_PRIMARY sequence cycle" << std::endl;
      while ((tick & 0x1FFU) != 0U) {
        const double now_s = raw_now_s();
        if (now_s >= next_tick_s) {
          next_tick_s += 0.01;
          sender.tick(tick++, 0.0, false);
        }
        can_frame ignored{};
        while (read(fd, &ignored, sizeof(ignored)) ==
               static_cast<ssize_t>(sizeof(ignored))) {
        }
        std::this_thread::sleep_for(std::chrono::microseconds(200));
      }
      std::cout << "SEQUENCE_ALIGNED resume the frozen Orin gateway now"
                << std::endl;
    };
    std::uint8_t mcu_state = 0xFFU;
    double standby_seen_s = 0.0;
    bool reset_notice_printed = false;
    const double standby_deadline = raw_now_s() + options.arm_timeout_s;
    while (raw_now_s() < standby_deadline) {
      const double now_s = raw_now_s();
      if (now_s >= next_tick_s) {
        next_tick_s += 0.01;
        sender.tick(tick++, 0.0, false);
      }
      can_frame frame{};
      while (read(fd, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame))) {
        if ((frame.can_id & CAN_SFF_MASK) == acb::kMcuHeartbeatId &&
            frame.can_dlc == 8U) {
          mcu_state = frame.data[0];
          if (mcu_state == 1U && standby_seen_s == 0.0) standby_seen_s = raw_now_s();
        }
      }
      if (standby_seen_s > 0.0 && (raw_now_s() - standby_seen_s) >= 0.5) break;
      if (mcu_state == 6U && !reset_notice_printed) {
        std::cout << "WAITING_FOR_MCU_RESET standby frames active" << std::endl;
        reset_notice_printed = true;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    if (standby_seen_s == 0.0) {
      throw std::runtime_error("MCU did not enter STANDBY before timeout");
    }
    if (options.samples == 0) {
      align_frozen_primary();
      std::cout << "RECOVERY_COMPLETE MCU stable in STANDBY; resume/start Orin now"
                << std::endl;
      close(fd);
      return 0;
    }
    std::cout << "MCU_STANDBY_OK switching to ACTIVE" << std::endl;

    std::vector<double> samples_ms;
    std::ofstream csv(options.csv_path);
    if (!csv) throw std::runtime_error("cannot open CSV: " + options.csv_path);
    csv << "sample,target_steer_deg,tx_monotonic_raw_s,rx_monotonic_raw_s,rtt_ms\n";

    double target_deg = 0.0;
    int sample_index = 0;
    bool waiting_crossing = false;
    double step_tx_s = 0.0;
    double next_step_s = raw_now_s() + 1.0;
    bool mcu_active = false;

    while (sample_index < options.samples) {
      const double now_s = raw_now_s();
      if (now_s >= next_tick_s) {
        next_tick_s += 0.01;
        const auto tx = sender.tick(tick++, target_deg, true);
        if (!waiting_crossing && now_s >= next_step_s && tx) {
          target_deg = (sample_index & 1) == 0 ? options.step_deg : -options.step_deg;
          const auto first_step_tx = sender.tick(tick++, target_deg, true);
          if (first_step_tx) {
            step_tx_s = *first_step_tx;
            waiting_crossing = true;
          }
        }
      }

      can_frame frame{};
      while (read(fd, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame))) {
        const auto id = frame.can_id & CAN_SFF_MASK;
        if (id == acb::kMcuHeartbeatId && frame.can_dlc == 8U) {
          mcu_active = frame.data[0] == 2U && frame.data[1] == 1U &&
                       (frame.data[2] & 0x01U) != 0U;
        }
        if (id == acb::kMcuControlId && frame.can_dlc == 8U && waiting_crossing &&
            mcu_active) {
          std::array<std::uint8_t, 8> data{};
          std::copy_n(frame.data, 8U, data.begin());
          if (data[7] != acb::frame_crc(id, data)) continue;
          const double feedback_deg = static_cast<double>(get_i16(frame.data)) * 0.01;
          const bool crossed = target_deg > 0.0 ? feedback_deg >= 0.0
                                                : feedback_deg <= 0.0;
          if (crossed) {
            const double rx_s = raw_now_s();
            const double rtt_ms = (rx_s - step_tx_s) * 1000.0;
            if (rtt_ms >= 0.0 && rtt_ms < 100.0) {
              samples_ms.push_back(rtt_ms);
              csv << sample_index << ',' << target_deg << ',' << step_tx_s << ','
                  << rx_s << ',' << rtt_ms << '\n';
              ++sample_index;
              waiting_crossing = false;
              next_step_s = rx_s + static_cast<double>(options.settle_ms) / 1000.0;
              if (sample_index % 10 == 0 || sample_index == options.samples) {
                std::cout << "SAMPLES " << sample_index << '/' << options.samples
                          << " last_ms=" << rtt_ms << std::endl;
              }
            }
          }
        }
      }
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    std::sort(samples_ms.begin(), samples_ms.end());
    const double mean = std::accumulate(samples_ms.begin(), samples_ms.end(), 0.0) /
                        static_cast<double>(samples_ms.size());
    double squared = 0.0;
    for (const double value : samples_ms) squared += (value - mean) * (value - mean);
    const double stddev = std::sqrt(squared / static_cast<double>(samples_ms.size()));
    std::cout << "RESULT n=" << samples_ms.size() << " min_ms=" << samples_ms.front()
              << " mean_ms=" << mean << " median_ms=" << percentile(samples_ms, 0.5)
              << " p95_ms=" << percentile(samples_ms, 0.95)
              << " p99_ms=" << percentile(samples_ms, 0.99)
              << " max_ms=" << samples_ms.back() << " stddev_ms=" << stddev
              << " csv=" << options.csv_path << std::endl;
    std::cout << "RETURNING_TO_STANDBY keep Orin service stopped" << std::endl;
    standby_seen_s = 0.0;
    reset_notice_printed = false;
    const double return_deadline = raw_now_s() + options.arm_timeout_s;
    while (raw_now_s() < return_deadline) {
      const double now_s = raw_now_s();
      if (now_s >= next_tick_s) {
        next_tick_s += 0.01;
        sender.tick(tick++, 0.0, false);
      }
      can_frame frame{};
      while (read(fd, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame))) {
        const auto id = frame.can_id & CAN_SFF_MASK;
        if (id == acb::kMcuHeartbeatId && frame.can_dlc == 8U) {
          if (frame.data[0] == 1U) {
            if (standby_seen_s == 0.0) standby_seen_s = raw_now_s();
          } else {
            standby_seen_s = 0.0;
          }
          if (frame.data[0] == 6U && !reset_notice_printed) {
            std::cout << "WAITING_FOR_MCU_RESET standby frames active" << std::endl;
            reset_notice_printed = true;
          }
        }
      }
      if (standby_seen_s > 0.0 && raw_now_s() - standby_seen_s >= 0.5) {
        align_frozen_primary();
        std::cout << "HANDOFF_READY MCU stable in STANDBY; resume/start Orin now"
                  << std::endl;
        close(fd);
        return 0;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    throw std::runtime_error("MCU did not return to STANDBY before timeout");
  } catch (const std::exception& error) {
    std::cerr << "can_rtt_probe failed: " << error.what() << std::endl;
    return 1;
  }
}
