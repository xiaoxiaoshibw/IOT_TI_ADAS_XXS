// adas_command_gate 生命周期节点：唯一控制裁决点 + builtin_stop + 运行诊断。
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "adas_command_gate/gate_core.hpp"
#include "adas_common/cycle_time_monitor.hpp"
#include "adas_common/parameter_validation.hpp"
#include "adas_common/timing_monitor.hpp"
#include "adas_msgs/msg/aeb_status.hpp"
#include "adas_msgs/msg/control.hpp"
#include "adas_msgs/msg/gate_status.hpp"
#include "adas_msgs/msg/navigation_status.hpp"
#include "adas_msgs/msg/safety_status.hpp"
#include "adas_msgs/srv/change_command_source.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace adas::control {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CommandGateNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit CommandGateNode(const rclcpp::NodeOptions& options)
      : LifecycleNode("command_gate", options) {
    declare_parameter<double>("rate_hz", 50.0);
    declare_parameter<double>("follower_timeout_s", 0.2);
    declare_parameter<double>("aeb_stale_timeout_s", 0.1);
    declare_parameter<double>("odom_stale_timeout_s", 0.15);
    declare_parameter<double>("stop_decel_mps2", 2.5);
    declare_parameter<double>("steer_decay_tau_s", 1.0);
    declare_parameter<std::vector<double>>("filter.speed_points_mps",
                                           {0.0, 10.0, 20.0, 30.0});
    declare_parameter<std::vector<double>>("filter.steer_lim_rad", {0.6, 0.35, 0.2, 0.12});
    declare_parameter<std::vector<double>>("filter.steer_rate_lim_rps",
                                           {0.8, 0.5, 0.35, 0.25});
    declare_parameter<double>("filter.max_accel_mps2", 3.0);
    declare_parameter<double>("filter.max_decel_mps2", 8.0);
    declare_parameter<double>("filter.max_jerk_mps3", 10.0);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    try {
      rate_hz_ = get_parameter("rate_hz").as_double();
      params_.follower_timeout_s = get_parameter("follower_timeout_s").as_double();
      params_.aeb_stale_timeout_s = get_parameter("aeb_stale_timeout_s").as_double();
      params_.odom_stale_timeout_s = get_parameter("odom_stale_timeout_s").as_double();
      params_.stop_decel_mps2 = get_parameter("stop_decel_mps2").as_double();
      params_.steer_decay_tau_s = get_parameter("steer_decay_tau_s").as_double();
      params_.speed_points_mps = get_parameter("filter.speed_points_mps").as_double_array();
      params_.steer_lim_rad = get_parameter("filter.steer_lim_rad").as_double_array();
      params_.steer_rate_lim_rps =
          get_parameter("filter.steer_rate_lim_rps").as_double_array();
      params_.max_accel_mps2 = get_parameter("filter.max_accel_mps2").as_double();
      params_.max_decel_mps2 = get_parameter("filter.max_decel_mps2").as_double();
      params_.max_jerk_mps3 = get_parameter("filter.max_jerk_mps3").as_double();
      validate_parameters();
      core_ = std::make_unique<GateCore>(params_);

      const auto sensor_qos = rclcpp::SensorDataQoS();
      sub_follower_ = create_subscription<adas_msgs::msg::Control>(
          "/adas/control/trajectory_follower/control_cmd", rclcpp::QoS(1).reliable(),
          [this](adas_msgs::msg::Control::ConstSharedPtr msg) {
            if (!valid_control(*msg)) {
              ++invalid_command_count_;
              return;
            }
            follower_cmd_ = msg;
            follower_rx_time_ = now();
          });
      sub_aeb_status_ = create_subscription<adas_msgs::msg::AebStatus>(
          "/adas/control/aeb/status", rclcpp::QoS(1).reliable().transient_local(),
          [this](adas_msgs::msg::AebStatus::ConstSharedPtr msg) {
            aeb_status_ = msg;
            aeb_status_rx_time_ = now();
          });
      sub_aeb_cmd_ = create_subscription<adas_msgs::msg::Control>(
          "/adas/control/aeb/emergency_cmd", rclcpp::QoS(1).reliable(),
          [this](adas_msgs::msg::Control::ConstSharedPtr msg) {
            if (!valid_control(*msg)) {
              ++invalid_command_count_;
              return;
            }
            aeb_cmd_ = msg;
            aeb_cmd_rx_time_ = now();
          });
      sub_safety_ = create_subscription<adas_msgs::msg::SafetyStatus>(
          "/adas/system/safety_status", rclcpp::QoS(1).reliable().transient_local(),
          [this](adas_msgs::msg::SafetyStatus::ConstSharedPtr msg) { safety_ = msg; });
      sub_navigation_ = create_subscription<adas_msgs::msg::NavigationStatus>(
          "/adas/navigation/status", rclcpp::QoS(1).reliable().transient_local(),
          [this](adas_msgs::msg::NavigationStatus::ConstSharedPtr msg) {
            if (msg->state == adas_msgs::msg::NavigationStatus::DRIVING) {
              navigation_route_seen_ = true;
              navigation_planned_stop_ = false;
            } else if (msg->state == adas_msgs::msg::NavigationStatus::PLANNING) {
              navigation_planned_stop_ = false;
            } else if (navigation_route_seen_ &&
                       (msg->state == adas_msgs::msg::NavigationStatus::ARRIVED ||
                        msg->state == adas_msgs::msg::NavigationStatus::CANCELED ||
                        msg->state == adas_msgs::msg::NavigationStatus::FAILED)) {
              navigation_planned_stop_ = true;
            }
          });
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
          "/adas/localization/kinematic_state", sensor_qos,
          [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
            odom_ = msg;
            odom_rx_time_ = now();
          });
      pub_cmd_ = create_publisher<adas_msgs::msg::Control>(
          "/adas/control/gate/control_cmd", rclcpp::QoS(1).reliable());
      pub_status_ = create_publisher<adas_msgs::msg::GateStatus>(
          "/adas/control/gate/status", rclcpp::QoS(1).reliable().transient_local());
      srv_source_ = create_service<adas_msgs::srv::ChangeCommandSource>(
          "~/source/change",
          [this](const std::shared_ptr<adas_msgs::srv::ChangeCommandSource::Request> req,
                 std::shared_ptr<adas_msgs::srv::ChangeCommandSource::Response> res) {
            if (req->source == adas_msgs::msg::GateStatus::SOURCE_BUILTIN_STOP) {
              force_builtin_stop_ = true;
              res->success = true;
              res->message = "forced builtin_stop";
            } else if (req->source == adas_msgs::msg::GateStatus::SOURCE_FOLLOWER) {
              force_builtin_stop_ = false;
              res->success = true;
              res->message = "auto selection restored";
            } else {
              res->success = false;
              res->message = "only SOURCE_BUILTIN_STOP / SOURCE_FOLLOWER allowed";
            }
          });
      timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_),
                                 [this]() { on_timer(); });
      timer_->cancel();
      timing_.set_budget_ms(1000.0 / rate_hz_);
      cycle_time_.configure(rate_hz_);
      diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
      diagnostics_->setHardwareID("soc-command-gate");
      diagnostics_->add("runtime", [this](auto& stat) { produce_diagnostics(stat); });
      parameters_valid_ = true;
      last_error_.clear();
      return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
      parameters_valid_ = false;
      last_error_ = e.what();
      last_error_time_ = now();
      RCLCPP_ERROR(get_logger(), "command_gate configure 失败: %s", e.what());
      return CallbackReturn::FAILURE;
    }
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    clear_runtime_inputs();
    timing_.reset();
    cycle_time_.reset();
    output_enabled_ = true;
    force_builtin_stop_ = false;
    navigation_route_seen_ = false;
    navigation_planned_stop_ = false;
    last_source_ = GateSource::kBuiltinStop;
    source_switch_count_ = 0U;
    pub_cmd_->on_activate();
    pub_status_->on_activate();
    timer_->reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    publish_builtin_stop("lifecycle deactivate");
    output_enabled_ = false;
    if (timer_) timer_->cancel();
    if (pub_cmd_) pub_cmd_->on_deactivate();
    if (pub_status_) pub_status_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    release_resources();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
    publish_builtin_stop("lifecycle shutdown");
    release_resources();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State&) override {
    last_error_ = "lifecycle error";
    last_error_time_ = now();
    publish_builtin_stop(last_error_);
    output_enabled_ = false;
    if (timer_) timer_->cancel();
    return CallbackReturn::SUCCESS;
  }

 private:
  static bool valid_control(const adas_msgs::msg::Control& msg) {
    return std::isfinite(msg.lateral.steering_tire_angle_rad) &&
           std::isfinite(msg.lateral.steering_tire_rotation_rate_rad_s) &&
           std::isfinite(msg.longitudinal.velocity_mps) &&
           std::isfinite(msg.longitudinal.acceleration_mps2);
  }

  void validate_parameters() const {
    common::require_timeout_exceeds_period("follower_timeout_s", params_.follower_timeout_s,
                                           "rate_hz", rate_hz_);
    common::require_positive("aeb_stale_timeout_s", params_.aeb_stale_timeout_s);
    common::require_positive("odom_stale_timeout_s", params_.odom_stale_timeout_s);
    common::require_positive("stop_decel_mps2", params_.stop_decel_mps2);
    common::require_nonnegative("steer_decay_tau_s", params_.steer_decay_tau_s);
    common::require_positive("filter.max_accel_mps2", params_.max_accel_mps2);
    common::require_positive("filter.max_decel_mps2", params_.max_decel_mps2);
    common::require_positive("filter.max_jerk_mps3", params_.max_jerk_mps3);
  }

  void on_timer() {
    common::ScopedTimingSample sample(timing_);
    GateInputs in;
    in.now_s = now().seconds();
    in.dt = cycle_time_.tick();
    in.ego_speed_mps = odom_ ? odom_->twist.twist.linear.x : 0.0;
    in.odom_received = static_cast<bool>(odom_);
    in.odom_stamp_s = odom_ ? odom_rx_time_.seconds() : -1e9;
    if (follower_cmd_) {
      in.follower_received = true;
      in.follower_stamp_s = follower_rx_time_.seconds();
      in.follower_cmd.lateral.steering_tire_angle_rad =
          follower_cmd_->lateral.steering_tire_angle_rad;
      in.follower_cmd.lateral.rotation_rate_rad_s =
          follower_cmd_->lateral.steering_tire_rotation_rate_rad_s;
      in.follower_cmd.longitudinal.velocity_mps = follower_cmd_->longitudinal.velocity_mps;
      in.follower_cmd.longitudinal.acceleration_mps2 =
          follower_cmd_->longitudinal.acceleration_mps2;
    }
    if (aeb_status_ && aeb_cmd_ &&
        aeb_status_->state == adas_msgs::msg::AebStatus::STATE_EMERGENCY) {
      in.aeb_emergency = true;
      in.aeb_received = true;
      in.aeb_stamp_s = std::min(aeb_status_rx_time_.seconds(), aeb_cmd_rx_time_.seconds());
      in.aeb_cmd.longitudinal.velocity_mps = aeb_cmd_->longitudinal.velocity_mps;
      in.aeb_cmd.longitudinal.acceleration_mps2 = aeb_cmd_->longitudinal.acceleration_mps2;
    }
    in.mrm_stop_requested =
        safety_ && safety_->overall >= adas_msgs::msg::SafetyStatus::LEVEL_MRM_COMFORT;
    in.navigation_planned_stop = navigation_planned_stop_;
    in.force_builtin_stop = force_builtin_stop_;
    const auto decision = core_->update(in);
    publish_decision(decision);
  }

  void publish_decision(const GateDecision& decision) {
    if (!pub_cmd_ || !pub_cmd_->is_activated()) return;
    adas_msgs::msg::Control cmd;
    cmd.header.stamp = now();
    cmd.lateral.steering_tire_angle_rad =
        static_cast<float>(decision.cmd.lateral.steering_tire_angle_rad);
    cmd.lateral.steering_tire_rotation_rate_rad_s =
        static_cast<float>(decision.cmd.lateral.rotation_rate_rad_s);
    cmd.longitudinal.velocity_mps = static_cast<float>(decision.cmd.longitudinal.velocity_mps);
    cmd.longitudinal.acceleration_mps2 =
        static_cast<float>(decision.cmd.longitudinal.acceleration_mps2);
    pub_cmd_->publish(cmd);
    last_valid_control_time_ = now();
    ++output_count_;

    if (decision.source != last_source_) {
      ++source_switch_count_;
      RCLCPP_WARN(get_logger(), "gate 切源: %d → %d (%s)", static_cast<int>(last_source_),
                  static_cast<int>(decision.source), decision.reason.c_str());
      last_source_ = decision.source;
    }
    last_reason_ = decision.reason;
    last_limited_ = decision.limited;
    fallback_active_ = decision.source == GateSource::kBuiltinStop;
    if (++status_decimator_ >= 5 && pub_status_ && pub_status_->is_activated()) {
      status_decimator_ = 0;
      adas_msgs::msg::GateStatus st;
      st.header.stamp = cmd.header.stamp;
      st.selected_source = static_cast<uint8_t>(decision.source);
      st.limited = decision.limited;
      st.reason = decision.reason;
      pub_status_->publish(st);
    }
  }

  void publish_builtin_stop(const std::string& reason) {
    if (!core_) return;
    GateInputs in;
    in.now_s = now().seconds();
    in.dt = 1.0 / std::max(rate_hz_, 1.0);
    in.ego_speed_mps = odom_ ? odom_->twist.twist.linear.x : 0.0;
    in.odom_received = static_cast<bool>(odom_);
    in.odom_stamp_s = odom_ ? odom_rx_time_.seconds() : -1e9;
    in.force_builtin_stop = true;
    auto decision = core_->update(in);
    decision.reason = reason;
    publish_decision(decision);
  }

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    const auto timing = timing_.snapshot();
    const auto cycle = cycle_time_.snapshot();
    const double follower_age = follower_cmd_ ? (now() - follower_rx_time_).seconds() : -1.0;
    const double aeb_age = aeb_status_ ? (now() - aeb_status_rx_time_).seconds() : -1.0;
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string summary = "command source healthy";
    if (!parameters_valid_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      summary = "invalid parameters";
    } else if (!output_enabled_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      summary = "inactive";
    } else if (fallback_active_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      summary = "builtin_stop active";
    } else if (timing.error) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      summary = "processing budget exceeded";
    } else if (timing.warning) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      summary = "processing near budget";
    }
    stat.summary(level, summary);
    stat.add("lifecycle_state", get_current_state().label());
    stat.add("parameters_valid", parameters_valid_);
    stat.add("output_enabled", output_enabled_);
    stat.add("selected_source", static_cast<int>(last_source_));
    stat.add("selected_source_age_s", follower_age);
    stat.add("source_switch_count", source_switch_count_);
    stat.add("fallback_active", fallback_active_);
    stat.add("builtin_stop_active", last_source_ == GateSource::kBuiltinStop);
    stat.add("command_limited", last_limited_);
    stat.add("selection_reason", last_reason_);
    stat.add("follower_received", static_cast<bool>(follower_cmd_));
    stat.add("follower_age_s", follower_age);
    stat.add("aeb_received", static_cast<bool>(aeb_status_));
    stat.add("aeb_age_s", aeb_age);
    stat.add("mrm_requested", safety_ && safety_->overall >=
                                  adas_msgs::msg::SafetyStatus::LEVEL_MRM_COMFORT);
    stat.add("output_count", output_count_);
    stat.add("invalid_command_count", invalid_command_count_);
    stat.add("last_valid_control_time_s", last_valid_control_time_.seconds());
    stat.add("processing_last_ms", timing.last_ms);
    stat.add("processing_average_ms", timing.average_ms);
    stat.add("processing_max_ms", timing.max_ms);
    stat.add("processing_budget_ms", timing.budget_ms);
    stat.add("cycle_last_ms", cycle.last_raw_s * 1000.0);
    stat.add("cycle_average_ms", cycle.average_raw_s * 1000.0);
    stat.add("cycle_max_jitter_ms", cycle.max_abs_jitter_s * 1000.0);
    stat.add("cycle_clamped_samples", cycle.clamped_samples);
    stat.add("last_error", last_error_);
    stat.add("last_error_time_s", last_error_time_.seconds());
  }

  void clear_runtime_inputs() {
    follower_cmd_.reset();
    aeb_status_.reset();
    aeb_cmd_.reset();
    safety_.reset();
    odom_.reset();
    follower_rx_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    aeb_status_rx_time_ = follower_rx_time_;
    aeb_cmd_rx_time_ = follower_rx_time_;
    odom_rx_time_ = follower_rx_time_;
    output_count_ = 0U;
    status_decimator_ = 0;
    fallback_active_ = true;
  }

  void release_resources() {
    output_enabled_ = false;
    if (timer_) timer_->cancel();
    timer_.reset();
    diagnostics_.reset();
    srv_source_.reset();
    pub_cmd_.reset();
    pub_status_.reset();
    sub_follower_.reset();
    sub_aeb_status_.reset();
    sub_aeb_cmd_.reset();
    sub_safety_.reset();
    sub_navigation_.reset();
    sub_odom_.reset();
    core_.reset();
    clear_runtime_inputs();
  }

  double rate_hz_{50.0};
  GateParams params_;
  std::unique_ptr<GateCore> core_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  common::TimingMonitor timing_;
  common::CycleTimeMonitor cycle_time_;
  bool force_builtin_stop_{false};
  bool navigation_route_seen_{false};
  bool navigation_planned_stop_{false};
  bool output_enabled_{false};
  bool parameters_valid_{false};
  bool fallback_active_{true};
  bool last_limited_{false};
  GateSource last_source_{GateSource::kBuiltinStop};
  int status_decimator_{0};
  std::size_t source_switch_count_{0U};
  std::size_t invalid_command_count_{0U};
  std::size_t output_count_{0U};
  std::string last_reason_{"not active"};
  std::string last_error_;
  rclcpp::Time last_error_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_valid_control_time_{0, 0, RCL_ROS_TIME};

  adas_msgs::msg::Control::ConstSharedPtr follower_cmd_;
  rclcpp::Time follower_rx_time_{0, 0, RCL_ROS_TIME};
  adas_msgs::msg::AebStatus::ConstSharedPtr aeb_status_;
  rclcpp::Time aeb_status_rx_time_{0, 0, RCL_ROS_TIME};
  adas_msgs::msg::Control::ConstSharedPtr aeb_cmd_;
  rclcpp::Time aeb_cmd_rx_time_{0, 0, RCL_ROS_TIME};
  adas_msgs::msg::SafetyStatus::ConstSharedPtr safety_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  rclcpp::Time odom_rx_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<adas_msgs::msg::Control>::SharedPtr sub_follower_;
  rclcpp::Subscription<adas_msgs::msg::AebStatus>::SharedPtr sub_aeb_status_;
  rclcpp::Subscription<adas_msgs::msg::Control>::SharedPtr sub_aeb_cmd_;
  rclcpp::Subscription<adas_msgs::msg::SafetyStatus>::SharedPtr sub_safety_;
  rclcpp::Subscription<adas_msgs::msg::NavigationStatus>::SharedPtr sub_navigation_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp_lifecycle::LifecyclePublisher<adas_msgs::msg::Control>::SharedPtr pub_cmd_;
  rclcpp_lifecycle::LifecyclePublisher<adas_msgs::msg::GateStatus>::SharedPtr pub_status_;
  rclcpp::Service<adas_msgs::srv::ChangeCommandSource>::SharedPtr srv_source_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace adas::control

RCLCPP_COMPONENTS_REGISTER_NODE(adas::control::CommandGateNode)
