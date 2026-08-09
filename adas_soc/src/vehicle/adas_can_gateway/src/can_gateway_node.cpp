#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <linux/can.h>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/random.h>
#include <sys/socket.h>
#include <unistd.h>

#include "adas_can_gateway/feedback_monitor.hpp"
#include "adas_can_gateway/canalystii.hpp"
#include "adas_can_gateway/can_simulator.hpp"
#include "adas_can_gateway/hil_session_manager.hpp"
#include "adas_can_gateway/protocol.hpp"
#include "adas_can_gateway/startup_gate.hpp"
#include "adas_msgs/msg/actuation_command.hpp"
#include "adas_msgs/msg/aeb_status.hpp"
#include "adas_msgs/msg/control.hpp"
#include "adas_msgs/msg/gate_status.hpp"
#include "adas_msgs/msg/mcu_status.hpp"
#include "adas_msgs/msg/safety_status.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace adas::can_gateway {
namespace {

constexpr double kRadToDeg = 57.29577951308232;

double age_seconds(const std::chrono::steady_clock::time_point& stamp, bool present) {
  if (!present) return -1.0;
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - stamp).count();
}

std::uint32_t random_session_id() {
  for (int attempt = 0; attempt < 4; ++attempt) {
    std::uint32_t id = 0U;
    if (getrandom(&id, sizeof(id), 0) != static_cast<ssize_t>(sizeof(id))) {
      throw std::runtime_error("getrandom failed: " + std::string(std::strerror(errno)));
    }
    if (id != 0U) return id;
  }
  throw std::runtime_error("getrandom repeatedly returned zero session id");
}

std::uint32_t load_or_create_session_id(const std::string& path) {
  {
    std::ifstream input(path, std::ios::binary);
    std::uint32_t persisted = 0U;
    if (input.read(reinterpret_cast<char*>(&persisted), sizeof(persisted)) &&
        persisted != 0U) {
      return persisted;
    }
  }
  const std::uint32_t generated = random_session_id();
  const std::string temporary = path + ".tmp";
  {
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    if (!output.write(reinterpret_cast<const char*>(&generated), sizeof(generated))) {
      throw std::runtime_error("failed to persist HIL session id: " + temporary);
    }
    output.flush();
    if (!output) {
      throw std::runtime_error("failed to flush HIL session id: " + temporary);
    }
  }
  if (std::rename(temporary.c_str(), path.c_str()) != 0) {
    throw std::runtime_error("failed to activate HIL session id: " +
                             std::string(std::strerror(errno)));
  }
  return generated;
}

// FC_* 位图（MCU/include/safety.h）到可读降级原因的映射，供 GUI 直接显示。
std::string degrade_reason_text(std::uint16_t fault_code) {
  static constexpr struct {
    std::uint16_t bit;
    const char* name;
  } kReasons[] = {
      {1U << 0, "primary_timeout"},   {1U << 1, "backup_timeout"},
      {1U << 2, "primary_seq_stall"}, {1U << 3, "backup_seq_stall"},
      {1U << 4, "all_sources_lost"},  {1U << 5, "crc_errors"},
      {1U << 6, "soc_fault"},         {1U << 7, "loop_overrun"},
      {1U << 8, "estop_active"},      {1U << 9, "protocol_mismatch"},
      {1U << 10, "fault_lock"},       {1U << 11, "can_bus_off"},
      {1U << 12, "watchdog_reset"},   {1U << 13, "self_test_failed"},
  };
  std::string result;
  for (const auto& reason : kReasons) {
    if ((fault_code & reason.bit) == 0U) continue;
    if (!result.empty()) result += '+';
    result += reason.name;
  }
  if (result.empty()) result = "none";
  return result;
}

bool finite_control(const adas_msgs::msg::Control& command) {
  return std::isfinite(command.lateral.steering_tire_angle_rad) &&
         std::isfinite(command.lateral.steering_tire_rotation_rate_rad_s) &&
         std::isfinite(command.longitudinal.velocity_mps) &&
         std::isfinite(command.longitudinal.acceleration_mps2);
}

}  // namespace

class CanGatewayNode final : public rclcpp::Node {
 public:
  CanGatewayNode() : Node("can_gateway") {
    // 默认主控制链路：Jetson HIL 上为 PEAK PCAN-USB → SocketCAN can1 @ 500k，
    // 由 can_hil.yaml 覆盖 can_interface；此处代码默认值仅裸跑时生效。
    // canalystii（USB CANalyst-II）为调试备源路径，仅显式配置 transport:=canalystii 时启用。
    transport_ = declare_parameter<std::string>("transport", "socketcan");
    interface_name_ = declare_parameter<std::string>("can_interface", "can0");
    canalyst_device_index_ = declare_parameter<int>("canalyst_device_index", 0);
    canalyst_channel_ = declare_parameter<int>("canalyst_channel", 1);
    canalyst_bitrate_ = declare_parameter<int>("canalyst_bitrate", 500000);
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.05);
    status_timeout_s_ = declare_parameter<double>("status_timeout_s", 0.1);
    feedback_timeout_s_ = declare_parameter<double>("feedback_timeout_s", 0.1);
    max_steer_deg_ = declare_parameter<double>("max_steer_deg", 30.0);
    if (command_timeout_s_ <= 0.0 || status_timeout_s_ <= 0.0 ||
        feedback_timeout_s_ <= 0.0 || max_steer_deg_ <= 0.0 ||
        max_steer_deg_ > 30.0) {
      throw std::invalid_argument("CAN gateway timeout/steering parameters are invalid");
    }

    // 启动/恢复门控与发送节拍监控（HIL 2026-07-13 primary_timeout 整改）。
    // 发送 deadline 必须显著低于 MCU 侧超时（CTRL 60ms / HB 100ms / STATUS
    // 120ms），保证网关先于 MCU 察觉自身节拍异常并 fail-closed。
    StartupGateConfig gate_config;
    gate_config.inputs_stable_s = declare_parameter<double>("startup_stable_s", 0.5);
    tx_deadline_control_s_ = declare_parameter<double>("tx_deadline_control_s", 0.03);
    tx_deadline_status_s_ = declare_parameter<double>("tx_deadline_status_s", 0.06);
    deviation_arm_s_ = declare_parameter<double>("feedback_deviation_arm_s", 1.0);
    sequence_persist_path_ = declare_parameter<std::string>(
        "sequence_persist_path", "/var/tmp/adas_can_gateway_seq.bin");
    if (gate_config.inputs_stable_s <= 0.0 || deviation_arm_s_ <= 0.0 ||
        tx_deadline_control_s_ <= 0.01 || tx_deadline_control_s_ >= 0.06 ||
        tx_deadline_status_s_ <= 0.02 || tx_deadline_status_s_ >= 0.1) {
      throw std::invalid_argument("CAN gateway startup-gate/deadline parameters are invalid");
    }
    startup_gate_ = std::make_unique<StartupGate>(gate_config);
    hil_session_ = std::make_unique<HilSessionManager>(
        declare_parameter<double>("hil_session_timeout_s", 0.1));
    const auto session_persist_path = declare_parameter<std::string>(
        "hil_session_persist_path", "/var/log/adas/hil_session_id.bin");
    if (session_persist_path.empty()) {
      throw std::invalid_argument("hil_session_persist_path must not be empty");
    }
    hil_session_->begin(load_or_create_session_id(session_persist_path));
    restore_sequences();

    FeedbackMonitorConfig monitor_config;
    monitor_config.steer_tolerance_deg =
        declare_parameter<double>("feedback_steer_tolerance_deg", 8.0);
    monitor_config.accel_tolerance_mps2 =
        declare_parameter<double>("feedback_accel_tolerance_mps2", 2.5);
    monitor_config.deviation_hold_s =
        declare_parameter<double>("feedback_deviation_hold_s", 0.5);
    monitor_config.feedback_timeout_s = feedback_timeout_s_;
    monitor_config.recovery_hold_s =
        declare_parameter<double>("feedback_recovery_hold_s", 1.0);
    if (monitor_config.steer_tolerance_deg <= 0.0 ||
        monitor_config.accel_tolerance_mps2 <= 0.0 ||
        monitor_config.deviation_hold_s <= 0.0 ||
        monitor_config.recovery_hold_s <= 0.0) {
      throw std::invalid_argument("CAN gateway feedback-monitor parameters are invalid");
    }
    feedback_monitor_ = std::make_unique<FeedbackMonitor>(monitor_config);

    if (transport_ == "socketcan" || transport_ == "sim") {
      sim_can_ = std::make_unique<SimCanInterface>(interface_name_);
    } else if (transport_ == "canalystii") {
      if (canalyst_device_index_ < 0 || canalyst_channel_ < 0 || canalyst_channel_ > 1 ||
          canalyst_bitrate_ <= 0) {
        throw std::invalid_argument("CANalyst-II device/channel/bitrate parameters are invalid");
      }
      canalyst_device_ = std::make_unique<canalystii::Device>();
      canalyst_device_->open(canalyst_device_index_);
      canalyst_device_->init_channel(canalyst_channel_,
                                     static_cast<std::uint32_t>(canalyst_bitrate_));
    } else {
      throw std::invalid_argument("transport must be 'canalystii', 'socketcan', or 'sim'");
    }
    sub_control_ = create_subscription<adas_msgs::msg::Control>(
        "/adas/control/gate/control_cmd", rclcpp::QoS(1).reliable(),
        [this](adas_msgs::msg::Control::ConstSharedPtr message) {
          if (!finite_control(*message)) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            ++invalid_control_count_;
            control_.reset();
            control_received_ = false;
            return;
          }
          std::lock_guard<std::mutex> lock(state_mutex_);
          control_ = message;
          control_rx_time_ = std::chrono::steady_clock::now();
          control_received_ = true;
        });
    sub_aeb_ = create_subscription<adas_msgs::msg::AebStatus>(
        "/adas/control/aeb/status", rclcpp::QoS(1).reliable().transient_local(),
        [this](adas_msgs::msg::AebStatus::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          aeb_ = message;
          aeb_rx_time_ = std::chrono::steady_clock::now();
          aeb_received_ = true;
        });
    sub_safety_ = create_subscription<adas_msgs::msg::SafetyStatus>(
        "/adas/system/safety_status", rclcpp::QoS(1).reliable().transient_local(),
        [this](adas_msgs::msg::SafetyStatus::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          safety_ = message;
          safety_rx_time_ = std::chrono::steady_clock::now();
          safety_received_ = true;
        });
    sub_gate_status_ = create_subscription<adas_msgs::msg::GateStatus>(
        "/adas/control/gate/status", rclcpp::QoS(1).reliable().transient_local(),
        [this](adas_msgs::msg::GateStatus::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          gate_status_ = message;
          gate_status_rx_time_ = std::chrono::steady_clock::now();
          gate_status_received_ = true;
        });
    pub_feedback_ = create_publisher<adas_msgs::msg::ActuationCommand>(
        "/adas/mcu/actuation_feedback", rclcpp::QoS(1).reliable());
    pub_mcu_status_ = create_publisher<adas_msgs::msg::McuStatus>(
        "/adas/mcu/status", rclcpp::QoS(1).reliable());

    // 执行器反馈故障的显式软复位（对齐 MCU 长按/0x301 清故障的"人工确认"
    // 语义）。偏差闩锁本只能靠重启网关进程解除；此服务让操作员在确认已排障
    // 后用一条 `ros2 service call ... std_srvs/srv/Trigger` 清锁，HIL 免重启。
    srv_reset_fault_ = create_service<std_srvs::srv::Trigger>(
        "~/reset_actuator_fault",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          bool had_fault = false;
          {
            std::lock_guard<std::mutex> lock(state_mutex_);
            had_fault = feedback_monitor_ && feedback_monitor_->any_fault();
            if (feedback_monitor_) feedback_monitor_->reset();
          }
          response->success = true;
          response->message = had_fault
                                  ? "actuator feedback fault latch cleared"
                                  : "no actuator feedback fault was latched";
          RCLCPP_WARN(get_logger(), "reset_actuator_fault: %s",
                      response->message.c_str());
        });

    diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
    diagnostics_->setHardwareID(
        transport_ == "canalystii"
            ? "soc-canalystii-usb" + std::to_string(canalyst_device_index_) +
                  "-can" + std::to_string(canalyst_channel_ + 1)
            : "soc-socketcan-" + interface_name_);
    diagnostics_->add("runtime", [this](auto& status) { diagnostics(status); });
    can_thread_running_ = true;
    can_thread_ = std::thread(&CanGatewayNode::can_thread_main, this);
    RCLCPP_INFO(get_logger(),
                "%s gateway up (%s, single primary source); STARTUP_STANDBY until "
                "inputs stable %.0f ms",
                transport_.c_str(), transport_ == "canalystii"
                                      ? ("USB CANalyst-II CAN" +
                                         std::to_string(canalyst_channel_ + 1) + " @ " +
                                         std::to_string(canalyst_bitrate_) + " bps").c_str()
                                      : (interface_name_ + " @ 500000 bps").c_str(),
                gate_config.inputs_stable_s * 1000.0);
  }

  ~CanGatewayNode() override {
    can_thread_running_ = false;
    if (can_thread_.joinable()) can_thread_.join();
    persist_sequences();
    if (canalyst_device_) {
      canalyst_device_->stop(canalyst_channel_);
      canalyst_device_->close();
    }
  }

 private:
  bool command_fresh() const {
    const double command_age = age_seconds(control_rx_time_, control_received_);
    return control_ && command_age >= 0.0 && command_age < command_timeout_s_;
  }
  bool aeb_fresh() const {
    const double aeb_age = age_seconds(aeb_rx_time_, aeb_received_);
    return aeb_ && aeb_age >= 0.0 && aeb_age < status_timeout_s_;
  }
  bool safety_fresh() const {
    const double safety_age = age_seconds(safety_rx_time_, safety_received_);
    return safety_ && safety_age >= 0.0 && safety_age < status_timeout_s_;
  }
  bool planned_stop_fresh() const {
    const double gate_age = age_seconds(gate_status_rx_time_, gate_status_received_);
    return command_fresh() && gate_status_ && gate_age >= 0.0 &&
           gate_age < status_timeout_s_ &&
           gate_status_->reason.rfind("planned_stop_", 0U) == 0U;
  }
  bool gate_control_fresh() const {
    const double gate_age = age_seconds(gate_status_rx_time_, gate_status_received_);
    return gate_status_ && gate_age >= 0.0 && gate_age < status_timeout_s_ &&
           gate_status_->selected_source == adas_msgs::msg::GateStatus::SOURCE_FOLLOWER;
  }
  // 门控放行 ACTIVE 的输入条件：lateral/longitudinal 的来源（gate command）
  // 与 status 帧的来源（AEB、safety）必须同时新鲜。
  bool inputs_fresh() const {
    return command_fresh() && gate_control_fresh() && aeb_fresh() && safety_fresh();
  }

  CommandSnapshot snapshot() const {
    CommandSnapshot result;
    /* The gateway is alive even when its upstream control input is stale.
     * Keep its heartbeat healthy so the MCU can accept the explicit MRM
     * request below; control_authority remains false in that case. */
    result.soc_health = 1U;
    const bool command_fresh = this->command_fresh();
    const bool aeb_fresh = this->aeb_fresh();
    const bool safety_fresh = this->safety_fresh();

    if (command_fresh) {
      result.target_steer_deg = std::clamp(
          static_cast<double>(control_->lateral.steering_tire_angle_rad) * kRadToDeg,
          -max_steer_deg_, max_steer_deg_);
      result.target_steer_rate_dps = std::clamp(
          std::abs(static_cast<double>(
              control_->lateral.steering_tire_rotation_rate_rad_s)) * kRadToDeg,
          0.0, 400.0);
      result.target_accel_mps2 = std::clamp(
          static_cast<double>(control_->longitudinal.acceleration_mps2), -8.0, 3.0);
      result.target_speed_mps = std::clamp(
          static_cast<double>(control_->longitudinal.velocity_mps), 0.0, 327.67);
      result.lateral_enable = true;
      result.longitudinal_enable = true;
      result.lka_active = true;
      result.acc_active = true;
      result.control_authority = true;
      result.status_word = kStatusControlEnable | kStatusLateralEnable |
          kStatusLongitudinalEnable | kStatusLkaActive | kStatusAccActive;
      result.system_mode = SystemMode::kActive;
    } else {
      result.system_mode = SystemMode::kDegraded;
      result.fault_level = 2U;
      result.mrm_request = true;
      result.status_word |= kStatusDegraded | kStatusMrmRequest;
    }

    if (!aeb_fresh) {
      result.mrm_request = true;
      result.fault_level = std::max<std::uint8_t>(result.fault_level, 2U);
      result.status_word |= kStatusDegraded | kStatusMrmRequest;
    } else if (aeb_->state == adas_msgs::msg::AebStatus::STATE_EMERGENCY) {
      result.aeb_risk = AebRisk::kFull;
      result.aeb_required_decel_mps2 = std::clamp(
          static_cast<double>(aeb_->required_decel_mps2), 0.0, 8.0);
      result.obstacle_valid = true;
      result.status_word |= kStatusAebBraking;
    } else if (aeb_->state == adas_msgs::msg::AebStatus::STATE_WARNING) {
      result.aeb_risk = AebRisk::kWarning;
      result.obstacle_valid = true;
      result.status_word |= kStatusAebWarning;
    }

    if (!safety_fresh ||
        safety_->overall >= adas_msgs::msg::SafetyStatus::LEVEL_MRM_COMFORT) {
      result.mrm_request = true;
      result.status_word |= kStatusMrmRequest | kStatusDegraded;
      result.fault_level = std::max<std::uint8_t>(result.fault_level, 1U);
      result.system_mode = SystemMode::kMrm;
    }
    if (safety_fresh &&
        safety_->overall >= adas_msgs::msg::SafetyStatus::LEVEL_MRM_EMERGENCY) {
      result.emergency_stop = true;
      result.status_word |= kStatusEmergencyStop;
      result.fault_level = 2U;
      result.system_mode = SystemMode::kEmergencyBrake;
    }
    // 执行器反馈闭环故障（偏差锁存或反馈超时）：撤销常规控制权并请求 MRM。
    if (feedback_monitor_ && feedback_monitor_->any_fault()) {
      result.control_authority = false;
      result.mrm_request = true;
      result.fault_level = 2U;
      result.status_word |= kStatusDegraded | kStatusMrmRequest;
      if (result.system_mode == SystemMode::kActive) {
        result.system_mode = SystemMode::kMrm;
      }
    }
    return result;
  }

  bool send(const Frame& frame) {
    if (transport_ == "canalystii") {
      try {
        canalyst_device_->send(canalyst_channel_, frame.id, frame.data.data());
        ++tx_frame_count_;
        return true;
      } catch (const std::exception&) {
        ++tx_error_count_;
        return false;
      }
    }
    try {
      sim_can_->send(frame);
    } catch (const std::exception&) {
      ++tx_error_count_;
      return false;
    }
    ++tx_frame_count_;
    return true;
  }

  // ---- 发送节拍健康（整改项 3/6）：每类 primary 帧独立跟踪成功上线时间 ----
  enum TxClass : std::size_t {
    kTxHeartbeat = 0U,
    kTxLateral = 1U,
    kTxLongitudinal = 2U,
    kTxStatus = 3U,
    kTxSession = 4U,
  };

  struct TxClassHealth {
    std::chrono::steady_clock::time_point last_tx{};
    bool sent{false};
    double max_interval_s{0.0};
    std::uint64_t deadline_misses{0U};
  };

  double tx_deadline_s(std::size_t tx_class) const {
    return (tx_class == kTxLateral || tx_class == kTxLongitudinal)
               ? tx_deadline_control_s_
               : tx_deadline_status_s_;
  }

  // 任何一类帧距上次成功上线超过 deadline，即视为本节拍超期。发送失败
  // （USB/队列错误）不会刷新 last_tx，因此也会在这里被捕获。
  bool tx_deadline_ok(const std::chrono::steady_clock::time_point& now) const {
    for (std::size_t tx_class = 0U; tx_class < tx_health_.size(); ++tx_class) {
      // 0x104 is intentionally silent while no handshake/keepalive request is
      // active.  Treating that designed silence as a TX deadline failure
      // makes every recovery attempt immediately demote itself to MRM.
      if (tx_class == kTxSession &&
          hil_session_->request() == adas::can_protocol::SessionRequest::kNone) {
        continue;
      }
      const auto& health = tx_health_[tx_class];
      if (!health.sent) continue;
      const double age = std::chrono::duration<double>(now - health.last_tx).count();
      if (age > tx_deadline_s(tx_class)) return false;
    }
    return true;
  }

  void mark_sent(std::size_t tx_class, const std::chrono::steady_clock::time_point& now) {
    auto& health = tx_health_[tx_class];
    if (health.sent) {
      const double interval =
          std::chrono::duration<double>(now - health.last_tx).count();
      health.max_interval_s = std::max(health.max_interval_s, interval);
      if (interval > tx_deadline_s(tx_class)) ++health.deadline_misses;
    }
    health.last_tx = now;
    health.sent = true;
  }

  void send_frames(const std::chrono::steady_clock::time_point& now,
                   const std::vector<std::pair<std::size_t, Frame>>& frames) {
    if (transport_ != "canalystii") {
      for (const auto& [tx_class, frame] : frames) {
        if (send(frame)) mark_sent(tx_class, now);
      }
      return;
    }
    std::vector<std::pair<std::uint32_t, const std::uint8_t*>> raw_frames;
    raw_frames.reserve(frames.size());
    for (const auto& entry : frames) {
      raw_frames.emplace_back(entry.second.id, entry.second.data.data());
    }
    try {
      canalyst_device_->send_batch(canalyst_channel_, raw_frames);
      tx_frame_count_ += frames.size();
      for (const auto& entry : frames) mark_sent(entry.first, now);
    } catch (const std::exception&) {
      ++tx_error_count_;
    }
  }

  void can_thread_main() {
    ::mlockall(MCL_CURRENT | MCL_FUTURE);
    struct sched_param param = {};
    param.sched_priority = 50;
    ::pthread_setschedparam(::pthread_self(), SCHED_FIFO, &param);
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);
    ::pthread_setaffinity_np(::pthread_self(), sizeof(cpu_set_t), &cpuset);
    struct timespec next;
    ::clock_gettime(CLOCK_MONOTONIC, &next);
    while (can_thread_running_) {
      ::clock_gettime(CLOCK_MONOTONIC, &next);
      next.tv_nsec += 10'000'000;
      if (next.tv_nsec >= 1'000'000'000) {
        next.tv_sec += 1;
        next.tv_nsec -= 1'000'000'000;
      }
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        tick();
      }
      ::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }
  }

  void receive() {
    if (transport_ == "canalystii") {
      std::vector<canalystii::RxFrame> received;
      try {
        canalyst_device_->receive(canalyst_channel_, received, 2);
      } catch (const std::exception&) {
        ++rx_io_error_count_;
        return;
      }
      for (const auto& raw : received) {
        if (raw.can_id > CAN_SFF_MASK || raw.dlc != 8U) {
          ++rx_dlc_error_count_;
          continue;
        }
        Frame frame{raw.can_id, {}};
        std::copy_n(raw.data, 8U, frame.data.begin());
        process_received_frame(frame);
      }
      return;
    }
    try {
      for (const auto& frame : sim_can_->receive()) {
        process_received_frame(frame);
      }
    } catch (const std::exception&) {
      ++rx_io_error_count_;
    }
  }

  void process_received_frame(const Frame& frame) {
      if (!validate_frame(frame)) {
        ++rx_crc_error_count_;
        return;
      }

      if (frame.id == kMcuControlId) {
        const auto feedback = decode_control_feedback(frame);
        if (!feedback) {
          ++rx_payload_error_count_;
          return;
        }
        if (feedback_sequence_seen_ &&
            !sequence_forward(last_feedback_sequence_, feedback->sequence)) {
          ++rx_sequence_error_count_;
          return;
        }
        feedback_sequence_seen_ = true;
        last_feedback_sequence_ = feedback->sequence;
        last_feedback_ = *feedback;
        feedback_rx_time_ = std::chrono::steady_clock::now();
        feedback_received_ = true;
        adas_msgs::msg::ActuationCommand message;
        message.header.stamp = now();
        message.throttle = static_cast<float>(feedback->throttle);
        message.brake = static_cast<float>(feedback->brake);
        message.steer = static_cast<float>(
            std::clamp(feedback->steer_deg / max_steer_deg_, -1.0, 1.0));
        pub_feedback_->publish(message);
        ++rx_valid_control_count_;
      } else if (frame.id == kMcuHeartbeatId) {
        const auto heartbeat = decode_mcu_heartbeat(frame);
        if (heartbeat) {
          mcu_heartbeat_ = *heartbeat;
          last_mcu_state_ = heartbeat->state;
          last_mcu_source_ = heartbeat->active_source;
          mcu_heartbeat_rx_time_ = std::chrono::steady_clock::now();
          mcu_heartbeat_received_ = true;
        }
      } else if (frame.id == kMcuDiagId) {
        const auto diag = decode_mcu_diag(frame);
        if (diag) {
          mcu_diag_ = *diag;
          mcu_diag_rx_time_ = std::chrono::steady_clock::now();
          mcu_diag_received_ = true;
        }
      } else if (frame.id == kMcuE2eDiagId) {
        const auto e2e = decode_mcu_e2e_diag(frame);
        if (e2e) {
          mcu_e2e_ = *e2e;
          last_protocol_version_ = e2e->version;
          protocol_version_ok_ = last_protocol_version_ == kProtocolVersion;
          mcu_e2e_received_ = true;
        }
      } else if (frame.id == adas::can_protocol::kCanIdMcuSessionStatus) {
        const auto session = decode_session_status(frame);
        if (session) {
          (void)hil_session_->observe(
              *session,
              std::chrono::duration<double>(
                  std::chrono::steady_clock::now().time_since_epoch()).count());
        }
      }
  }

  // 执行器反馈闭环：链路活跃时监控 0x201 与下发命令的一致性。
  // 偏差监控布防条件（整改项 4）：MCU 心跳为 ACTIVE 且 active_source 为
  // PRIMARY、命令与反馈都新鲜、且门控 ACTIVE 已满启动稳定窗——启动瞬间
  // MCU 反馈的制动量与尚未建立的命令比较会产生假偏差。
  void update_feedback_monitor(double now_s) {
    const bool upstream_active = command_fresh();
    const double heartbeat_age =
        age_seconds(mcu_heartbeat_rx_time_, mcu_heartbeat_received_);
    const bool heartbeat_fresh = heartbeat_age >= 0.0 && heartbeat_age < 0.2;
    const bool link_active = upstream_active && heartbeat_fresh;
    const double feedback_age = age_seconds(feedback_rx_time_, feedback_received_);
    const bool feedback_fresh = feedback_age >= 0.0 &&
                                feedback_age < feedback_timeout_s_;
    const bool deviation_eligible = link_active && feedback_fresh &&
        mcu_heartbeat_.state == static_cast<std::uint8_t>(SystemMode::kActive) &&
        mcu_heartbeat_.active_source == kSourcePrimary &&
        startup_gate_->active() &&
        startup_gate_->active_duration_s(now_s) >= deviation_arm_s_;
    double commanded_steer_deg = 0.0;
    double commanded_accel_mps2 = 0.0;
    if (control_) {
      commanded_steer_deg = std::clamp(
          static_cast<double>(control_->lateral.steering_tire_angle_rad) * kRadToDeg,
          -max_steer_deg_, max_steer_deg_);
      commanded_accel_mps2 = std::clamp(
          static_cast<double>(control_->longitudinal.acceleration_mps2), -8.0, 3.0);
    }
    feedback_monitor_->update(now_s, link_active, deviation_eligible, feedback_fresh,
                              commanded_steer_deg, last_feedback_.steer_deg,
                              commanded_accel_mps2, last_feedback_.accel_mps2);
  }

  // 门控未放行时的降级帧内容：
  // - 冷启动（从未 ACTIVE）：干净 STANDBY 安全帧，MCU 保持 STANDBY 等待
  //   输入稳定窗满足；保留 AEB/急停信息位不掩盖。
  // - ACTIVE 后再武装（节拍超期或输入陈旧）：fail-closed，撤权并持续请求
  //   MRM，MCU 侧看门狗/FAILSAFE 兜底。
  void demote_for_gate(CommandSnapshot& command) const {
    command.control_authority = false;
    command.lateral_enable = false;
    command.longitudinal_enable = false;
    command.lka_active = false;
    command.acc_active = false;
    command.target_steer_deg = 0.0;
    command.target_steer_rate_dps = 0.0;
    command.target_accel_mps2 = 0.0;
    command.target_speed_mps = 0.0;
    command.status_word &= static_cast<std::uint16_t>(
        ~(kStatusControlEnable | kStatusLateralEnable | kStatusLongitudinalEnable |
          kStatusLkaActive | kStatusAccActive));
    if (startup_gate_->ever_active()) {
      if (planned_stop_fresh()) {
        command.control_authority = true;
        command.mrm_request = false;
        command.fault_level = 0U;
        command.status_word &= static_cast<std::uint16_t>(
            ~(kStatusDegraded | kStatusMrmRequest));
        command.system_mode = SystemMode::kStandby;
      } else {
        command.mrm_request = true;
        command.fault_level = std::max<std::uint8_t>(command.fault_level, 2U);
        command.status_word |= kStatusDegraded | kStatusMrmRequest;
        if (command.system_mode == SystemMode::kStandby ||
            command.system_mode == SystemMode::kActive ||
            command.system_mode == SystemMode::kDegraded) {
          command.system_mode = SystemMode::kMrm;
        }
      }
    } else if (!command.emergency_stop) {
      command.mrm_request = false;
      command.fault_level = 0U;
      command.status_word &= static_cast<std::uint16_t>(
          ~(kStatusDegraded | kStatusMrmRequest));
      command.system_mode = SystemMode::kStandby;
    }
  }

  void tick() {
    const auto now = std::chrono::steady_clock::now();
    const double now_s =
        std::chrono::duration<double>(now.time_since_epoch()).count();
receive();
     hil_session_->update(now_s);
    if (hil_session_->request() == adas::can_protocol::SessionRequest::kNone) {
      // A later REARM starts a new intentional 0x104 transmission epoch; do
      // not measure its first frame against the previous session's last TX.
      tx_health_[kTxSession].sent = false;
    }
    // 门控在帧内容确定前更新：本节拍的四类帧统一反映门控结果，
    // 保证 ACTIVE 切换发生在一个发送周期边界。
    const bool deadline_ok = tx_deadline_ok(now);
    startup_gate_->update(now_s, inputs_fresh(), deadline_ok);
    update_feedback_monitor(now_s);
    auto command = snapshot();
    const auto session_request = hil_session_->request();
    // Prime all four primary mailboxes during PREPARE, one phase before the
    // atomic COMMIT boundary.  CAN mailbox reads are not atomic as a group;
    // waiting until the COMMIT cycle can let the MCU consume 0x104 before it
    // has observed every authority-bearing 0x100..0x103 frame.  The MCU HIL
    // gate still blocks actuator output until COMMIT is accepted, so this
    // removes the mailbox race without enabling control early.
    const bool priming_or_committing =
        session_request == adas::can_protocol::SessionRequest::kPrepareArm ||
        session_request == adas::can_protocol::SessionRequest::kCommitActive;
    if (!startup_gate_->active() ||
        (!hil_session_->control_authorized(now_s) && !priming_or_committing)) {
      demote_for_gate(command);
    }
    const auto mode = static_cast<std::uint8_t>(command.system_mode);
    const unsigned int gate_source = gate_status_
        ? static_cast<unsigned int>(gate_status_->selected_source) : 255U;
    const int safety_level = safety_ ? static_cast<int>(safety_->overall) : -1;
    const char* reason = command.emergency_stop
        ? "HARD_SAFETY"
        : command.mrm_request
            ? "MRM_REQUEST_OR_INPUT_LOSS"
            : gate_control_fresh() ? "READINESS_PASS" : "WAITING_FOR_FOLLOWER";
    if (!safety_transition_logged_ || mode != last_logged_mode_ ||
        command.mrm_request != last_logged_mrm_request_ ||
        startup_gate_->active() != last_logged_gate_active_) {
      RCLCPP_INFO(
          get_logger(),
          "[SAFETY_TRANSITION] from=%u to=%u reason=%s session_id=%u "
          "gate_source=%u gate_control_fresh=%d command_fresh=%d aeb_fresh=%d "
          "safety_fresh=%d inputs_fresh=%d control_authorized=%d mrm_request=%d "
          "safety_level=%d fault_level=%u",
          static_cast<unsigned int>(last_logged_mode_), static_cast<unsigned int>(mode), reason,
          hil_session_->session_id(), gate_source, gate_control_fresh(), command_fresh(),
          aeb_fresh(), safety_fresh(), inputs_fresh(),
          hil_session_->control_authorized(now_s), command.mrm_request, safety_level,
          static_cast<unsigned int>(command.fault_level));
      last_logged_mode_ = mode;
      last_logged_mrm_request_ = command.mrm_request;
      last_logged_gate_active_ = startup_gate_->active();
      safety_transition_logged_ = true;
    }
    std::vector<std::pair<std::size_t, Frame>> frames;
    frames.reserve(5);
    frames.emplace_back(kTxLateral, encoder_.lateral(command));
    frames.emplace_back(kTxLongitudinal, encoder_.longitudinal(command));
    // 慢速帧（0x100/0x103）按绝对时间调度；错过节拍不补发历史帧，
    // 直接以当前时间为新基准。
    if (now >= next_slow_tx_due_) {
      frames.emplace_back(kTxHeartbeat, encoder_.heartbeat(command));
      frames.emplace_back(kTxStatus, encoder_.status(command));
      if (hil_session_->request() != adas::can_protocol::SessionRequest::kNone) {
        frames.emplace_back(kTxSession, hil_session_->next_frame());
      }
      next_slow_tx_due_ = now + std::chrono::milliseconds(20);
    }
    send_frames(now, frames);
    if ((tick_count_ % 10U) == 0U) publish_mcu_status();
    // 序号持久化（整改项 5）：每 0.5s 落盘一次。恢复余量 64 覆盖两次
    // 落盘之间每类最多 50 帧的发送量。
    if ((tick_count_ % 50U) == 0U) persist_sequences();
    ++tick_count_;
  }

  // 序号持久化文件格式：'A''S''Q''1' + 5 字节计数器 + CRC8（poly 0x31）。
  void persist_sequences() {
    if (sequence_persist_path_.empty()) return;
    const auto seq = encoder_.sequences();
    std::array<std::uint8_t, 10> blob{
        'A', 'S', 'Q', '1', seq.heartbeat, seq.lateral, seq.longitudinal,
        seq.status, seq.alive, 0U};
    blob[9] = crc8(blob.data(), 9U);
    const std::string tmp_path = sequence_persist_path_ + ".tmp";
    {
      std::ofstream file(tmp_path, std::ios::binary | std::ios::trunc);
      if (!file.write(reinterpret_cast<const char*>(blob.data()), blob.size())) {
        ++sequence_persist_error_count_;
        return;
      }
    }
    if (std::rename(tmp_path.c_str(), sequence_persist_path_.c_str()) != 0) {
      ++sequence_persist_error_count_;
    }
  }

  void restore_sequences() {
    if (sequence_persist_path_.empty()) return;
    std::ifstream file(sequence_persist_path_, std::ios::binary);
    std::array<std::uint8_t, 10> blob{};
    if (!file.read(reinterpret_cast<char*>(blob.data()), blob.size())) {
      RCLCPP_WARN(get_logger(),
                  "sequence persist file missing/short (%s); sequences start at 0",
                  sequence_persist_path_.c_str());
      return;
    }
    if (blob[0] != 'A' || blob[1] != 'S' || blob[2] != 'Q' || blob[3] != '1' ||
        blob[9] != crc8(blob.data(), 9U)) {
      RCLCPP_WARN(get_logger(),
                  "sequence persist file corrupt (%s); sequences start at 0",
                  sequence_persist_path_.c_str());
      return;
    }
    // +64 余量：落盘周期内每类最多发出 50 帧，MCU 观察到的前向增量落在
    // [14, 64] ⊂ [1, 127]，不会被判倒退或超大跳变。
    constexpr std::uint8_t kMargin = 64U;
    EncoderSequences seq;
    seq.heartbeat = static_cast<std::uint8_t>(blob[4] + kMargin);
    seq.lateral = static_cast<std::uint8_t>(blob[5] + kMargin);
    seq.longitudinal = static_cast<std::uint8_t>(blob[6] + kMargin);
    seq.status = static_cast<std::uint8_t>(blob[7] + kMargin);
    seq.alive = static_cast<std::uint8_t>(blob[8] + kMargin);
    encoder_.restore(seq);
    sequences_restored_ = true;
    RCLCPP_INFO(get_logger(),
                "restored TX sequences from %s (margin +%u)",
                sequence_persist_path_.c_str(), kMargin);
  }

  // 100ms 安全遥测：控制源、安全状态、故障码、命令年龄和降级原因（P0 可视化数据源）
  void publish_mcu_status() {
    adas_msgs::msg::McuStatus message;
    message.header.stamp = now();
    message.system_state = mcu_heartbeat_.state;
    message.active_source = mcu_heartbeat_.active_source;
    message.fault_level = mcu_heartbeat_.fault_level;
    message.loop_load_pct = mcu_heartbeat_.loop_load_pct;
    message.primary_fresh = (mcu_heartbeat_.safety_flags & kSafetyPrimaryFresh) != 0U;
    message.backup_fresh = (mcu_heartbeat_.safety_flags & kSafetyBackupFresh) != 0U;
    message.aeb_floor_active = (mcu_heartbeat_.safety_flags & kSafetyAebFloor) != 0U;
    message.degraded = (mcu_heartbeat_.safety_flags & kSafetyDegraded) != 0U;
    message.manual_override =
        (mcu_heartbeat_.safety_flags & kSafetyManualOverride) != 0U;
    message.estop = (mcu_heartbeat_.safety_flags & kSafetyEstop) != 0U;
    message.fault_code = mcu_diag_.fault_code;
    message.reset_reason = mcu_diag_.reset_reason;
    message.primary_timeout_count = mcu_diag_.primary_timeouts;
    message.backup_timeout_count = mcu_diag_.backup_timeouts;
    message.crc_error_count = mcu_diag_.crc_errors;
    message.loop_overrun_count = mcu_diag_.loop_overruns;
    message.protocol_version = last_protocol_version_;
    message.protocol_version_ok = protocol_version_ok_;
    message.test_build = mcu_e2e_received_ && (mcu_e2e_.build_flags & 0x01U) != 0U;
    message.self_test_mask = mcu_e2e_.self_test_mask;
    message.can_recovery_count = mcu_e2e_.can_recovery_count;
    message.heartbeat_age_s = static_cast<float>(
        age_seconds(mcu_heartbeat_rx_time_, mcu_heartbeat_received_));
    message.diag_age_s =
        static_cast<float>(age_seconds(mcu_diag_rx_time_, mcu_diag_received_));
    message.feedback_age_s =
        static_cast<float>(age_seconds(feedback_rx_time_, feedback_received_));
    message.command_age_s =
        static_cast<float>(age_seconds(control_rx_time_, control_received_));
    message.degrade_reason = degrade_reason_text(mcu_diag_.fault_code);
    pub_mcu_status_->publish(message);
  }

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) {
    const double control_age = age_seconds(control_rx_time_, control_received_);
    const double feedback_age = age_seconds(feedback_rx_time_, feedback_received_);
    const double heartbeat_age =
        age_seconds(mcu_heartbeat_rx_time_, mcu_heartbeat_received_);
    const bool feedback_fresh = feedback_age >= 0.0 && feedback_age < feedback_timeout_s_;
    const bool heartbeat_fresh = heartbeat_age >= 0.0 && heartbeat_age < 0.2;
    std::uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string summary = "CAN gateway healthy";
    if (feedback_monitor_ && feedback_monitor_->deviation_fault()) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      summary = "actuator feedback deviates from command (latched, manual reset)";
    } else if (feedback_monitor_ && feedback_monitor_->timeout_fault()) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      summary = "actuator feedback timeout while commanding";
    } else if (!startup_gate_->active() && startup_gate_->ever_active()) {
      if (startup_gate_->last_rearm_was_deadline()) {
        // A local TX scheduling failure is a gateway fault and must remain an
        // ERROR consumed by the safety monitor.
        level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        summary = "TX deadline missed; ACTIVE demoted, re-gating (fail-closed MRM)";
      } else {
        // Upstream loss is already monitored directly by SafetyMonitor
        // (odometry/objects/trajectory/follower).  Reporting the resulting
        // gateway re-gating state as ERROR creates a circular latch:
        // gateway ERROR -> safety MRM -> gate never selects follower ->
        // gateway can never finish its fresh-input window.  WARN preserves
        // visibility while allowing automatic, fully gated recovery once all
        // directly monitored inputs are fresh again.
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        summary = "upstream inputs recovering; ACTIVE demoted, re-gating (fail-closed MRM)";
      }
    } else if (!feedback_fresh || !heartbeat_fresh || !protocol_version_ok_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      summary = "MCU feedback missing, stale, or protocol mismatch";
    } else if (tx_error_count_ != 0U || rx_crc_error_count_ != 0U ||
               rx_sequence_error_count_ != 0U || tx_deadline_miss_total() != 0U) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      summary = "CAN errors or TX deadline misses observed";
    }
    status.summary(level, summary);
    status.add("transport", transport_);
    const bool socketcan_transport = transport_ == "socketcan" || transport_ == "sim";
    status.add("can_interface", socketcan_transport ? interface_name_ : "USB CANalyst-II");
    status.add("can_link", socketcan_transport
                              ? (transport_ == "sim"
                                     ? "SocketCAN vcan (MCU SIL)"
                                     : "SocketCAN (Orin HIL: PEAK PCAN-USB can1)")
                              : "USB CANalyst-II (debug fallback)");
    status.add("canalyst_channel", transport_ == "canalystii" ? canalyst_channel_ + 1 : 0);
    status.add("single_primary_source", true);
    status.add("control_age_s", control_age);
    status.add("feedback_age_s", feedback_age);
    status.add("mcu_heartbeat_age_s", heartbeat_age);
    status.add("mcu_state", static_cast<int>(last_mcu_state_));
    status.add("mcu_active_source", static_cast<int>(last_mcu_source_));
    status.add("protocol_version", static_cast<int>(last_protocol_version_));
    status.add("protocol_version_ok", protocol_version_ok_);
    status.add("tx_frames", tx_frame_count_);
    status.add("tx_errors", tx_error_count_);
    status.add("rx_valid_control", rx_valid_control_count_);
    status.add("rx_crc_errors", rx_crc_error_count_);
    status.add("rx_dlc_errors", rx_dlc_error_count_);
    status.add("rx_sequence_errors", rx_sequence_error_count_);
    status.add("rx_payload_errors", rx_payload_error_count_);
    status.add("rx_io_errors", rx_io_error_count_);
    status.add("invalid_ros_control", invalid_control_count_);
    status.add("startup_gate_state",
               startup_gate_->active() ? "ACTIVE" : "STARTUP_STANDBY");
    status.add("startup_gate_rearm_count", startup_gate_->rearm_count());
    status.add("hil_session_id", static_cast<std::uint64_t>(hil_session_->session_id()));
    status.add("hil_session_ack", static_cast<int>(hil_session_->ack()));
    status.add("hil_session_request", static_cast<int>(hil_session_->request()));
    status.add("hil_session_recovery_required", hil_session_->recovery_required());
    status.add("hil_session_fault_locked", hil_session_->fault_locked());
    status.add("inputs_fresh", inputs_fresh());
    status.add("gate_control_fresh", gate_control_fresh());
    status.add("gate_source", gate_status_
                                  ? static_cast<int>(gate_status_->selected_source)
                                  : -1);
    status.add("safety_status_received", safety_received_ && safety_ != nullptr);
    status.add("sequences_restored", sequences_restored_);
    status.add("sequence_persist_errors", sequence_persist_error_count_);
    // 整改项 6：每类 primary 帧的发送时效遥测，使下一次 primary_timeout
    // 能直接定位到具体 CAN ID。
    static constexpr const char* kTxClassNames[] = {
        "0x100_heartbeat", "0x101_lateral", "0x102_longitudinal",
        "0x103_status", "0x104_session"};
    const auto tx_now = std::chrono::steady_clock::now();
    for (std::size_t tx_class = 0U; tx_class < tx_health_.size(); ++tx_class) {
      const auto& health = tx_health_[tx_class];
      const std::string prefix = std::string("tx_") + kTxClassNames[tx_class];
      status.add(prefix + "_last_age_s",
                 health.sent
                     ? std::chrono::duration<double>(tx_now - health.last_tx).count()
                     : -1.0);
      status.add(prefix + "_max_interval_s", health.max_interval_s);
      status.add(prefix + "_deadline_misses", health.deadline_misses);
    }
    if (feedback_monitor_) {
      status.add("actuator_deviation_fault", feedback_monitor_->deviation_fault());
      status.add("actuator_timeout_fault", feedback_monitor_->timeout_fault());
      status.add("actuator_steer_deviation_deg", feedback_monitor_->steer_deviation_deg());
      status.add("actuator_accel_deviation_mps2",
                 feedback_monitor_->accel_deviation_mps2());
    }
  }

  std::unique_ptr<SimCanInterface> sim_can_;
  std::unique_ptr<canalystii::Device> canalyst_device_;
  std::string transport_;
  std::string interface_name_;
  int canalyst_device_index_{0};
  int canalyst_channel_{1};
  int canalyst_bitrate_{500000};
  double command_timeout_s_{0.2};
  double status_timeout_s_{0.5};
  double feedback_timeout_s_{0.1};
  double max_steer_deg_{30.0};
  double tx_deadline_control_s_{0.03};
  double tx_deadline_status_s_{0.06};
  double deviation_arm_s_{1.0};
  std::string sequence_persist_path_;
  Encoder encoder_;
  std::uint64_t tick_count_{0U};
  std::unique_ptr<StartupGate> startup_gate_;
  std::unique_ptr<HilSessionManager> hil_session_;
  std::array<TxClassHealth, 5> tx_health_{};
  std::chrono::steady_clock::time_point next_slow_tx_due_{};
  bool safety_transition_logged_{false};
  std::uint8_t last_logged_mode_{static_cast<std::uint8_t>(SystemMode::kInit)};
  bool last_logged_mrm_request_{false};
  bool last_logged_gate_active_{false};
  bool sequences_restored_{false};
  std::uint64_t sequence_persist_error_count_{0U};

  std::uint64_t tx_deadline_miss_total() const {
    std::uint64_t total = 0U;
    // Session traffic is state-dependent rather than periodic.  Its counters
    // remain observable in diagnostics but must not poison runtime health.
    for (std::size_t i = 0U; i < kTxSession; ++i) {
      total += tx_health_[i].deadline_misses;
    }
    return total;
  }

  adas_msgs::msg::Control::ConstSharedPtr control_;
  adas_msgs::msg::AebStatus::ConstSharedPtr aeb_;
  adas_msgs::msg::SafetyStatus::ConstSharedPtr safety_;
  adas_msgs::msg::GateStatus::ConstSharedPtr gate_status_;
  std::chrono::steady_clock::time_point control_rx_time_{};
  std::chrono::steady_clock::time_point aeb_rx_time_{};
  std::chrono::steady_clock::time_point safety_rx_time_{};
  std::chrono::steady_clock::time_point gate_status_rx_time_{};
  std::chrono::steady_clock::time_point feedback_rx_time_{};
  std::chrono::steady_clock::time_point mcu_heartbeat_rx_time_{};
  bool control_received_{false};
  bool aeb_received_{false};
  bool safety_received_{false};
  bool gate_status_received_{false};
  bool feedback_received_{false};
  bool mcu_heartbeat_received_{false};
  bool feedback_sequence_seen_{false};
  std::uint8_t last_feedback_sequence_{0U};
  std::uint8_t last_mcu_state_{0U};
  std::uint8_t last_mcu_source_{0U};
  std::uint8_t last_protocol_version_{0U};
  bool protocol_version_ok_{false};
  McuHeartbeat mcu_heartbeat_{};
  McuDiag mcu_diag_{};
  McuE2eDiag mcu_e2e_{};
  ControlFeedback last_feedback_{};
  std::unique_ptr<FeedbackMonitor> feedback_monitor_;
  std::chrono::steady_clock::time_point mcu_diag_rx_time_{};
  bool mcu_diag_received_{false};
  bool mcu_e2e_received_{false};

  std::uint64_t tx_frame_count_{0U};
  std::uint64_t tx_error_count_{0U};
  std::uint64_t rx_valid_control_count_{0U};
  std::uint64_t rx_crc_error_count_{0U};
  std::uint64_t rx_dlc_error_count_{0U};
  std::uint64_t rx_sequence_error_count_{0U};
  std::uint64_t rx_payload_error_count_{0U};
  std::uint64_t rx_io_error_count_{0U};
  std::uint64_t invalid_control_count_{0U};

  rclcpp::Subscription<adas_msgs::msg::Control>::SharedPtr sub_control_;
  rclcpp::Subscription<adas_msgs::msg::AebStatus>::SharedPtr sub_aeb_;
  rclcpp::Subscription<adas_msgs::msg::SafetyStatus>::SharedPtr sub_safety_;
  rclcpp::Subscription<adas_msgs::msg::GateStatus>::SharedPtr sub_gate_status_;
  rclcpp::Publisher<adas_msgs::msg::ActuationCommand>::SharedPtr pub_feedback_;
  rclcpp::Publisher<adas_msgs::msg::McuStatus>::SharedPtr pub_mcu_status_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_fault_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  std::thread can_thread_;
  std::atomic<bool> can_thread_running_{false};
  std::mutex state_mutex_;
};

}  // namespace adas::can_gateway

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<adas::can_gateway::CanGatewayNode>());
  } catch (const std::exception& error) {
    std::fprintf(stderr, "CAN gateway startup failed: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
