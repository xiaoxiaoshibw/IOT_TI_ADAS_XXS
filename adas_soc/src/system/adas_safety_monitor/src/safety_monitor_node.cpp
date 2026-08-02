// adas_safety_monitor 生命周期节点：Topic 存活 + /diagnostics 双通道健康聚合。
#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "adas_common/parameter_validation.hpp"
#include "adas_common/timing_monitor.hpp"
#include "adas_msgs/msg/control.hpp"
#include "adas_msgs/msg/safety_status.hpp"
#include "adas_msgs/msg/tracked_object_array.hpp"
#include "adas_msgs/msg/trajectory.hpp"
#include "adas_safety_monitor/safety_monitor_core.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace adas::system {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

struct DiagnosticHealth {
  uint8_t level{diagnostic_msgs::msg::DiagnosticStatus::STALE};
  std::chrono::steady_clock::time_point stamp{};
};

class SafetyMonitorNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit SafetyMonitorNode(const rclcpp::NodeOptions& options)
      : LifecycleNode("safety_monitor", options) {
    declare_parameter<double>("startup_grace_s", 5.0);
    declare_parameter<double>("odom_timeout_s", 0.3);
    declare_parameter<double>("objects_timeout_s", 0.6);
    declare_parameter<double>("trajectory_timeout_s", 0.6);
    declare_parameter<double>("follower_cmd_timeout_s", 0.4);
    declare_parameter<double>("gate_cmd_timeout_s", 0.4);
    declare_parameter<double>("rate_hz", 10.0);
    declare_parameter<double>("diagnostics_timeout_s", 2.0);
    declare_parameter<double>("diagnostics_startup_grace_s", 5.0);
    declare_parameter<std::vector<std::string>>(
        "required_diagnostic_components",
        {"vehicle_interface", "command_gate", "trajectory_follower",
         "trajectory_planner", "behavior_planner", "object_tracker", "aeb"});
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    try {
      params_.startup_grace_s = get_parameter("startup_grace_s").as_double();
      params_.odom_timeout_s = get_parameter("odom_timeout_s").as_double();
      params_.objects_timeout_s = get_parameter("objects_timeout_s").as_double();
      params_.trajectory_timeout_s = get_parameter("trajectory_timeout_s").as_double();
      params_.follower_cmd_timeout_s =
          get_parameter("follower_cmd_timeout_s").as_double();
      params_.gate_cmd_timeout_s = get_parameter("gate_cmd_timeout_s").as_double();
      rate_hz_ = get_parameter("rate_hz").as_double();
      diagnostics_timeout_s_ = get_parameter("diagnostics_timeout_s").as_double();
      diagnostics_startup_grace_s_ =
          get_parameter("diagnostics_startup_grace_s").as_double();
      monitored_components_ =
          get_parameter("required_diagnostic_components").as_string_array();
      validate_parameters();
      core_ = std::make_unique<SafetyMonitorCore>(params_, now().seconds());

      const auto sensor_qos = rclcpp::SensorDataQoS();
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
          "/adas/localization/kinematic_state", sensor_qos,
          [this](nav_msgs::msg::Odometry::ConstSharedPtr) { stamps_.odom = now().seconds(); });
      sub_objects_ = create_subscription<adas_msgs::msg::TrackedObjectArray>(
          "/adas/perception/objects", sensor_qos,
          [this](adas_msgs::msg::TrackedObjectArray::ConstSharedPtr) {
            stamps_.objects = now().seconds();
          });
      sub_traj_ = create_subscription<adas_msgs::msg::Trajectory>(
          "/adas/planning/trajectory", rclcpp::QoS(1).reliable(),
          [this](adas_msgs::msg::Trajectory::ConstSharedPtr) {
            stamps_.trajectory = now().seconds();
          });
      sub_follower_ = create_subscription<adas_msgs::msg::Control>(
          "/adas/control/trajectory_follower/control_cmd", rclcpp::QoS(1).reliable(),
          [this](adas_msgs::msg::Control::ConstSharedPtr) {
            stamps_.follower_cmd = now().seconds();
          });
      sub_gate_ = create_subscription<adas_msgs::msg::Control>(
          "/adas/control/gate/control_cmd", rclcpp::QoS(1).reliable(),
          [this](adas_msgs::msg::Control::ConstSharedPtr) {
            stamps_.gate_cmd = now().seconds();
          });
      sub_diagnostics_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
          "/diagnostics", rclcpp::QoS(20).reliable(),
          [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
            on_diagnostics(*msg);
          });
      pub_status_ = create_publisher<adas_msgs::msg::SafetyStatus>(
          "/adas/system/safety_status", rclcpp::QoS(1).reliable().transient_local());
      timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_),
                                 [this]() { on_timer(); });
      timer_->cancel();
      timing_.set_budget_ms(1000.0 / rate_hz_);
      diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
      diagnostics_->setHardwareID("soc-safety-monitor");
      diagnostics_->add("runtime", [this](auto& stat) { produce_diagnostics(stat); });
      parameters_valid_ = true;
      last_error_.clear();
      return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
      parameters_valid_ = false;
      last_error_ = e.what();
      last_error_time_ = now();
      RCLCPP_ERROR(get_logger(), "safety_monitor configure 失败: %s", e.what());
      return CallbackReturn::FAILURE;
    }
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    stamps_ = ChannelStamps{};
    diagnostic_health_.clear();
    diagnostics_activation_time_ = std::chrono::steady_clock::now();
    timing_.reset();
    last_level_ = SafetyLevel::kOk;
    last_state_change_time_ = now();
    output_enabled_ = true;
    core_ = std::make_unique<SafetyMonitorCore>(params_, now().seconds());
    pub_status_->on_activate();
    timer_->reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    output_enabled_ = false;
    if (timer_) timer_->cancel();
    if (pub_status_) pub_status_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    release_resources();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
    release_resources();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State&) override {
    last_error_ = "lifecycle error";
    last_error_time_ = now();
    output_enabled_ = false;
    if (timer_) timer_->cancel();
    return CallbackReturn::SUCCESS;
  }

 private:
  void validate_parameters() const {
    common::require_nonnegative("startup_grace_s", params_.startup_grace_s);
    common::require_timeout_exceeds_period("odom_timeout_s", params_.odom_timeout_s,
                                           "rate_hz", rate_hz_);
    common::require_timeout_exceeds_period("objects_timeout_s", params_.objects_timeout_s,
                                           "rate_hz", rate_hz_);
    common::require_timeout_exceeds_period("trajectory_timeout_s",
                                           params_.trajectory_timeout_s, "rate_hz", rate_hz_);
    common::require_timeout_exceeds_period("follower_cmd_timeout_s",
                                           params_.follower_cmd_timeout_s, "rate_hz", rate_hz_);
    common::require_timeout_exceeds_period("gate_cmd_timeout_s", params_.gate_cmd_timeout_s,
                                           "rate_hz", rate_hz_);
    common::require_positive("diagnostics_timeout_s", diagnostics_timeout_s_);
    common::require_nonnegative("diagnostics_startup_grace_s",
                                diagnostics_startup_grace_s_);
    if (monitored_components_.empty()) {
      throw std::invalid_argument("required_diagnostic_components must not be empty");
    }
  }

  std::string component_from_name(const std::string& name) const {
    for (const auto& component : monitored_components_) {
      if (name.find(component) != std::string::npos) return component;
    }
    return {};
  }

  void on_diagnostics(const diagnostic_msgs::msg::DiagnosticArray& msg) {
    for (const auto& status : msg.status) {
      const auto component = component_from_name(status.name);
      if (component.empty()) continue;
      diagnostic_health_[component] =
          DiagnosticHealth{status.level, std::chrono::steady_clock::now()};
    }
  }

  void on_timer() {
    common::ScopedTimingSample sample(timing_);
    auto result = core_->evaluate(now().seconds(), stamps_);
    const auto steady_now = std::chrono::steady_clock::now();
    const double diagnostics_uptime =
        std::chrono::duration<double>(steady_now - diagnostics_activation_time_).count();
    if (diagnostics_uptime >= diagnostics_startup_grace_s_) {
      for (const auto& component : monitored_components_) {
        const auto it = diagnostic_health_.find(component);
        std::string failure;
        if (it == diagnostic_health_.end()) {
          failure = "diag_missing:" + component;
        } else {
          const double age =
              std::chrono::duration<double>(steady_now - it->second.stamp).count();
          if (age > diagnostics_timeout_s_) {
            failure = "diag_stale:" + component;
          } else if (it->second.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
            failure = "diag_error:" + component;
          }
        }
        if (failure.empty()) continue;
        if (std::find(result.failed_components.begin(), result.failed_components.end(), failure) ==
            result.failed_components.end()) {
          result.failed_components.push_back(failure);
        }
        result.overall = std::max(result.overall, SafetyLevel::kMrmComfort);
      }
    }
    if (result.overall != last_level_) {
      std::string failed;
      for (const auto& item : result.failed_components) failed += item + " ";
      RCLCPP_WARN(get_logger(), "安全级别变化: %d → %d（失效: %s）",
                  static_cast<int>(last_level_), static_cast<int>(result.overall),
                  failed.empty() ? "无" : failed.c_str());
      last_level_ = result.overall;
      last_state_change_time_ = now();
    }
    last_failed_components_ = result.failed_components;
    if (!pub_status_ || !pub_status_->is_activated()) return;
    adas_msgs::msg::SafetyStatus msg;
    msg.header.stamp = now();
    msg.overall = static_cast<uint8_t>(result.overall);
    msg.failed_components = result.failed_components;
    pub_status_->publish(msg);
    ++output_count_;
  }

  double age_or_negative(double stamp) const {
    return stamp < -1e17 ? -1.0 : now().seconds() - stamp;
  }

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    const auto timing = timing_.snapshot();
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string summary = "all monitored channels healthy";
    if (!parameters_valid_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      summary = "invalid parameters";
    } else if (!output_enabled_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      summary = "inactive";
    } else if (last_level_ >= SafetyLevel::kMrmComfort) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      summary = "MRM requested";
    } else if (last_level_ == SafetyLevel::kWarn || timing.warning) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      summary = "degraded";
    }
    stat.summary(level, summary);
    stat.add("lifecycle_state", get_current_state().label());
    stat.add("parameters_valid", parameters_valid_);
    stat.add("output_enabled", output_enabled_);
    stat.add("overall_level", static_cast<int>(last_level_));
    stat.add("mrm_requested", last_level_ >= SafetyLevel::kMrmComfort);
    stat.add("odom_age_s", age_or_negative(stamps_.odom));
    stat.add("objects_age_s", age_or_negative(stamps_.objects));
    stat.add("trajectory_age_s", age_or_negative(stamps_.trajectory));
    stat.add("follower_cmd_age_s", age_or_negative(stamps_.follower_cmd));
    stat.add("gate_cmd_age_s", age_or_negative(stamps_.gate_cmd));
    stat.add("diagnostic_components_seen", diagnostic_health_.size());
    stat.add("diagnostics_startup_grace_s", diagnostics_startup_grace_s_);
    stat.add("failed_component_count", last_failed_components_.size());
    stat.add("mrm_reason", last_failed_components_.empty() ? "none"
                                                           : last_failed_components_.front());
    stat.add("last_state_change_time_s", last_state_change_time_.seconds());
    stat.add("output_count", output_count_);
    stat.add("processing_last_ms", timing.last_ms);
    stat.add("processing_average_ms", timing.average_ms);
    stat.add("processing_max_ms", timing.max_ms);
    stat.add("processing_budget_ms", timing.budget_ms);
    stat.add("last_error", last_error_);
    stat.add("last_error_time_s", last_error_time_.seconds());
  }

  void release_resources() {
    output_enabled_ = false;
    if (timer_) timer_->cancel();
    timer_.reset();
    diagnostics_.reset();
    pub_status_.reset();
    sub_odom_.reset();
    sub_objects_.reset();
    sub_traj_.reset();
    sub_follower_.reset();
    sub_gate_.reset();
    sub_diagnostics_.reset();
    core_.reset();
    diagnostic_health_.clear();
  }

  SafetyMonitorParams params_;
  double rate_hz_{10.0};
  double diagnostics_timeout_s_{2.0};
  double diagnostics_startup_grace_s_{5.0};
  std::chrono::steady_clock::time_point diagnostics_activation_time_{};
  std::vector<std::string> monitored_components_;
  std::unique_ptr<SafetyMonitorCore> core_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  common::TimingMonitor timing_;
  ChannelStamps stamps_;
  SafetyLevel last_level_{SafetyLevel::kOk};
  bool output_enabled_{false};
  bool parameters_valid_{false};
  std::size_t output_count_{0U};
  std::string last_error_;
  rclcpp::Time last_error_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_state_change_time_{0, 0, RCL_ROS_TIME};
  std::vector<std::string> last_failed_components_;
  std::map<std::string, DiagnosticHealth> diagnostic_health_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<adas_msgs::msg::TrackedObjectArray>::SharedPtr sub_objects_;
  rclcpp::Subscription<adas_msgs::msg::Trajectory>::SharedPtr sub_traj_;
  rclcpp::Subscription<adas_msgs::msg::Control>::SharedPtr sub_follower_;
  rclcpp::Subscription<adas_msgs::msg::Control>::SharedPtr sub_gate_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_diagnostics_;
  rclcpp_lifecycle::LifecyclePublisher<adas_msgs::msg::SafetyStatus>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace adas::system

RCLCPP_COMPONENTS_REGISTER_NODE(adas::system::SafetyMonitorNode)
