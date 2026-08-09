#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include "adas_msgs/msg/control.hpp"
#include "adas_msgs/msg/gate_status.hpp"
#include "adas_msgs/msg/safety_status.hpp"
#include "adas_msgs/msg/trajectory.hpp"
#include "adas_msgs/msg/tracked_object_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

using namespace std::chrono_literals;

class FaultInjector final : public rclcpp::Node {
 public:
  explicit FaultInjector(pid_t target_pgid)
      : Node("fault_injection_runner"), target_pgid_(target_pgid) {
    const auto sensor_qos = rclcpp::SensorDataQoS();
    pub_trajectory_ = create_publisher<adas_msgs::msg::Trajectory>(
        "/adas/planning/trajectory", rclcpp::QoS(1).reliable());
    pub_objects_ = create_publisher<adas_msgs::msg::TrackedObjectArray>(
        "/adas/perception/objects_raw", sensor_qos);
    sub_safety_ = create_subscription<adas_msgs::msg::SafetyStatus>(
        "/adas/system/safety_status", rclcpp::QoS(1).reliable().transient_local(),
        [this](adas_msgs::msg::SafetyStatus::ConstSharedPtr msg) {
          safety_levels_.push_back(msg->overall);
        });
    sub_gate_ = create_subscription<adas_msgs::msg::GateStatus>(
        "/adas/control/gate/status", rclcpp::QoS(1).reliable().transient_local(),
        [this](adas_msgs::msg::GateStatus::ConstSharedPtr msg) {
          gate_sources_.push_back(msg->selected_source);
        });
    sub_follower_ = create_subscription<adas_msgs::msg::Control>(
        "/adas/control/trajectory_follower/control_cmd", rclcpp::QoS(1).reliable(),
        [this](adas_msgs::msg::Control::ConstSharedPtr msg) {
          ++follower_message_count_;
          last_follower_rx_ = std::chrono::steady_clock::now();
          follower_rx_seen_ = true;
          const double stamp = rclcpp::Time(msg->header.stamp).seconds();
          if (follower_stamp_seen_ && stamp > last_follower_stamp_s_) {
            follower_gap_max_s_ = std::max(follower_gap_max_s_,
                                           stamp - last_follower_stamp_s_);
          }
          last_follower_stamp_s_ = stamp;
          follower_stamp_seen_ = true;
        });
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/adas/localization/kinematic_state", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          const double stamp = rclcpp::Time(msg->header.stamp).seconds();
          if (odom_stamp_seen_ && stamp > last_odom_stamp_s_) {
            odom_gap_max_s_ = std::max(odom_gap_max_s_, stamp - last_odom_stamp_s_);
          }
          last_odom_stamp_s_ = stamp;
          odom_stamp_seen_ = true;
        });
  }

  bool run(const std::string& scenario) {
    if (scenario == "A") return scenario_nan_trajectory("A");
    if (scenario == "B") return scenario_odom_gap();
    if (scenario == "C") return scenario_aeb_crash();
    if (scenario == "D") return scenario_object_jitter();
    if (scenario == "E") return scenario_nan_trajectory("E");
    RCLCPP_ERROR(get_logger(), "unknown fault-injection scenario: %s", scenario.c_str());
    return false;
  }

 private:
  template <typename Predicate>
  bool spin_until(Predicate predicate, double timeout_s) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout_s);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(shared_from_this());
      if (predicate()) return true;
      rclcpp::sleep_for(10ms);
    }
    rclcpp::spin_some(shared_from_this());
    return predicate();
  }

  void spin_for(double duration_s) {
    (void)spin_until([]() { return false; }, duration_s);
  }

  bool wait_for_trajectory_subscriber(double timeout_s) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout_s);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (pub_trajectory_->get_subscription_count() > 0U) return true;
      rclcpp::spin_some(shared_from_this());
      rclcpp::sleep_for(20ms);
    }
    return pub_trajectory_->get_subscription_count() > 0U;
  }

  adas_msgs::msg::Trajectory nan_trajectory() const {
    adas_msgs::msg::Trajectory msg;
    msg.header.stamp = now();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (int i = 0; i < 3; ++i) {
      adas_msgs::msg::TrajectoryPoint point;
      point.pose.position.x = nan;
      point.pose.position.y = nan;
      point.pose.orientation.w = 1.0;
      point.longitudinal_velocity_mps = static_cast<float>(nan);
      point.acceleration_mps2 = static_cast<float>(nan);
      point.curvature = static_cast<float>(nan);
      msg.points.push_back(point);
    }
    return msg;
  }

  adas_msgs::msg::Trajectory valid_trajectory() const {
    adas_msgs::msg::Trajectory msg;
    msg.header.stamp = now();
    for (int i = 0; i < 3; ++i) {
      adas_msgs::msg::TrajectoryPoint point;
      point.pose.position.x = static_cast<double>(i) * 2.0;
      point.pose.position.y = 0.0;
      point.pose.orientation.w = 1.0;
      point.longitudinal_velocity_mps = 1.0F;
      point.acceleration_mps2 = 0.0F;
      point.curvature = 0.0F;
      point.time_from_start.sec = i;
      point.time_from_start.nanosec = 0U;
      msg.points.push_back(point);
    }
    return msg;
  }

  bool scenario_nan_trajectory(const std::string& label) {
    if (!wait_for_trajectory_subscriber(10.0)) {
      RCLCPP_ERROR(get_logger(), "scenario %s: trajectory has no subscribers", label.c_str());
      return false;
    }
    const auto before = safety_levels_.size();
    const auto message = nan_trajectory();
    // Keep the corrupt stream beyond the follower freshness timeout and the
    // monitor's three-frame confirmation window; a short single bad frame is
    // intentionally not expected to trigger MRM.
    const auto end = std::chrono::steady_clock::now() + 3000ms;
    while (std::chrono::steady_clock::now() < end) {
      auto current = message;
      current.header.stamp = now();
      pub_trajectory_->publish(current);
      rclcpp::spin_some(shared_from_this());
      rclcpp::sleep_for(50ms);
    }
    const bool mrm = spin_until(
        [this, before]() {
          for (std::size_t i = before; i < safety_levels_.size(); ++i) {
            if (safety_levels_[i] >= adas_msgs::msg::SafetyStatus::LEVEL_MRM_COMFORT) {
              return true;
            }
          }
          return false;
        },
        4.0);
    RCLCPP_INFO(get_logger(), "scenario %s: invalid trajectory -> MRM=%s",
                label.c_str(), mrm ? "true" : "false");
    return mrm;
  }

  std::vector<pid_t> find_processes(const std::string& marker) const {
    std::vector<pid_t> result;
    for (const auto& entry : std::filesystem::directory_iterator("/proc")) {
      const auto name = entry.path().filename().string();
      if (name.empty() || name.find_first_not_of("0123456789") != std::string::npos) {
        continue;
      }
      const pid_t pid = static_cast<pid_t>(std::strtol(name.c_str(), nullptr, 10));
      std::ifstream command_line(entry.path() / "cmdline", std::ios::binary);
      std::string command((std::istreambuf_iterator<char>(command_line)), {});
      if (pid == static_cast<pid_t>(::getpid()) ||
          (target_pgid_ > 0 && ::getpgid(pid) != target_pgid_) ||
          command.find(marker) == std::string::npos) {
        continue;
      }
      result.push_back(pid);
    }
    return result;
  }

  bool stop_process_temporarily(const std::string& marker, double duration_s) {
    const auto pids = find_processes(marker);
    if (pids.empty()) {
      RCLCPP_ERROR(get_logger(), "cannot find process containing '%s'", marker.c_str());
      return false;
    }
    std::string pid_list;
    for (const auto pid : pids) pid_list += std::to_string(pid) + " ";
    RCLCPP_INFO(get_logger(), "stopping '%s' pids=[%s], follower messages=%zu",
                marker.c_str(), pid_list.c_str(), follower_message_count_);
    for (const auto pid : pids) {
      if (::kill(pid, SIGSTOP) != 0) return false;
    }
    // Keep servicing subscriptions while the target is stopped.  Otherwise
    // the depth-1 odometry/control queues collapse the entire pause into one
    // post-resume sample and the observed gap no longer represents the fault.
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(duration_s);
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(shared_from_this());
      if (fault_window_ && follower_rx_seen_ &&
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        last_follower_rx_).count() > 0.10) {
        // With fresh odometry the follower publishes at 50 Hz.  Once its
        // 150 ms odometry watchdog expires, the deliberate absence of a
        // command is the follower's safe-mode contract.
        follower_safe_seen_during_fault_ = true;
      }
      rclcpp::sleep_for(10ms);
    }
    for (const auto pid : pids) {
      if (::kill(pid, SIGCONT) != 0) return false;
    }
    RCLCPP_INFO(get_logger(), "resumed '%s', follower messages=%zu", marker.c_str(),
                follower_message_count_);
    return true;
  }

  bool kill_process(const std::string& marker) {
    const auto pids = find_processes(marker);
    if (pids.empty()) {
      RCLCPP_ERROR(get_logger(), "cannot find process containing '%s'", marker.c_str());
      return false;
    }
    bool ok = true;
    for (const auto pid : pids) ok = (::kill(pid, SIGKILL) == 0) && ok;
    return ok;
  }

  bool scenario_odom_gap() {
    // Establish a known-good follower output first.  The default SIL launch
    // can legitimately have no route before the planner is initialized; in
    // that state a missing odometry frame cannot be distinguished from an
    // already-idle follower.
    // Give DDS discovery time to match this late-created publisher before
    // evaluating the warmup; the launch may not have produced a trajectory
    // topic at all during its initial activation chain.
    const auto match_end = std::chrono::steady_clock::now() + 10s;
    while (pub_trajectory_->get_subscription_count() == 0U &&
           std::chrono::steady_clock::now() < match_end) {
      rclcpp::spin_some(shared_from_this());
      rclcpp::sleep_for(20ms);
    }
    RCLCPP_INFO(get_logger(), "scenario B: trajectory subscriber count=%zu",
                pub_trajectory_->get_subscription_count());
    const auto warmup_end = std::chrono::steady_clock::now() + 2000ms;
    while (std::chrono::steady_clock::now() < warmup_end) {
      auto trajectory = valid_trajectory();
      trajectory.header.stamp = now();
      pub_trajectory_->publish(trajectory);
      rclcpp::spin_some(shared_from_this());
      rclcpp::sleep_for(20ms);
    }
    rclcpp::spin_some(shared_from_this());
    if (follower_message_count_ == 0U) {
      RCLCPP_ERROR(get_logger(), "scenario B: follower did not produce a warmup command");
      return false;
    }
    const double gap_before = follower_gap_max_s_;
    fault_window_ = true;
    if (!stop_process_temporarily("sim_vehicle_node", 0.25)) {
      fault_window_ = false;
      return false;
    }
    fault_window_ = false;
    const bool recovered = spin_until(
        [this, gap_before]() { return follower_gap_max_s_ > gap_before + 0.02; }, 2.0);
    const bool odom_missing = odom_gap_max_s_ >= 0.20;
    const bool safe_mode = follower_safe_seen_during_fault_;
    RCLCPP_INFO(get_logger(),
                "scenario B: odometry gap %.0f ms, control gap %.0f ms -> follower safe=%s",
                odom_gap_max_s_ * 1000.0, (follower_gap_max_s_ - gap_before) * 1000.0,
                (odom_missing && safe_mode && recovered) ? "true" : "false");
    return odom_missing && safe_mode && recovered;
  }

  bool scenario_aeb_crash() {
    const auto before = gate_sources_.size();
    if (!kill_process("aeb_node")) return false;
    const bool fallback = spin_until(
        [this, before]() {
          for (std::size_t i = before; i < gate_sources_.size(); ++i) {
            const auto source = gate_sources_[i];
            if (source == adas_msgs::msg::GateStatus::SOURCE_BUILTIN_STOP) return true;
          }
          return false;
        },
        4.0);
    RCLCPP_INFO(get_logger(), "scenario C: gate updates before=%zu after=%zu -> builtin_stop=%s",
                before, gate_sources_.size(),
                fallback ? "true" : "false");
    return fallback;
  }

  bool scenario_object_jitter() {
    const auto before = safety_levels_.size();
    std::vector<uint8_t> levels;
    for (int i = 0; i < 100; ++i) {
      adas_msgs::msg::TrackedObjectArray msg;
      msg.header.stamp = now();
      adas_msgs::msg::TrackedObject object;
      object.id = 100U + static_cast<uint32_t>(i % 2);
      object.classification = adas_msgs::msg::TrackedObject::CLASS_CAR;
      object.pose.pose.position.x = (i % 2 == 0) ? 22.0 : 65.0;
      object.pose.pose.position.y = (i % 3 == 0) ? 1.5 : -1.5;
      object.twist.twist.linear.x = (i % 2 == 0) ? 4.0 : 12.0;
      msg.objects.push_back(object);
      pub_objects_->publish(msg);
      rclcpp::spin_some(shared_from_this());
      rclcpp::sleep_for(20ms);
    }
    spin_for(1.0);
    for (std::size_t i = before; i < safety_levels_.size(); ++i) {
      if (levels.empty() || levels.back() != safety_levels_[i]) levels.push_back(safety_levels_[i]);
    }
    int transitions = levels.empty() ? 0 : static_cast<int>(levels.size()) - 1;
    const bool stable = !levels.empty() && transitions <= 3 &&
        std::find(levels.begin(), levels.end(),
                  adas_msgs::msg::SafetyStatus::LEVEL_MRM_EMERGENCY) == levels.end();
    RCLCPP_INFO(get_logger(), "scenario D: 100-frame object jitter -> transitions=%d stable=%s",
                transitions, stable ? "true" : "false");
    return stable;
  }

  rclcpp::Publisher<adas_msgs::msg::Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<adas_msgs::msg::TrackedObjectArray>::SharedPtr pub_objects_;
  rclcpp::Subscription<adas_msgs::msg::SafetyStatus>::SharedPtr sub_safety_;
  rclcpp::Subscription<adas_msgs::msg::GateStatus>::SharedPtr sub_gate_;
  rclcpp::Subscription<adas_msgs::msg::Control>::SharedPtr sub_follower_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  std::vector<uint8_t> safety_levels_;
  std::vector<uint8_t> gate_sources_;
  double last_follower_stamp_s_{0.0};
  bool follower_stamp_seen_{false};
  double follower_gap_max_s_{0.0};
  std::size_t follower_message_count_{0U};
  std::chrono::steady_clock::time_point last_follower_rx_{};
  bool follower_rx_seen_{false};
  double last_odom_stamp_s_{0.0};
  bool odom_stamp_seen_{false};
  double odom_gap_max_s_{0.0};
  bool fault_window_{false};
  bool follower_safe_seen_during_fault_{false};
  pid_t target_pgid_{-1};
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::string scenario = "all";
  pid_t launch_pgid = -1;
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--scenario") scenario = argv[i + 1];
    if (std::string(argv[i]) == "--launch-pgid") {
      launch_pgid = static_cast<pid_t>(std::strtol(argv[i + 1], nullptr, 10));
    }
  }
  auto node = std::make_shared<FaultInjector>(launch_pgid);
  const std::vector<std::string> scenarios = {"A", "B", "C", "D", "E"};
  bool ok = true;
  if (scenario == "all") {
    for (const auto& item : scenarios) {
      const bool result = node->run(item);
      std::printf("SCENARIO_%s %s\n", item.c_str(), result ? "PASS" : "FAIL");
      ok = result && ok;
    }
  } else {
    ok = node->run(scenario);
    std::printf("SCENARIO_%s %s\n", scenario.c_str(), ok ? "PASS" : "FAIL");
  }
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
