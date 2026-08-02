// adas_vehicle_interface 生命周期节点：gate control_cmd → 执行量；内建独立看门狗。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "adas_common/cycle_time_monitor.hpp"
#include "adas_common/parameter_validation.hpp"
#include "adas_common/timing_monitor.hpp"
#include "adas_msgs/msg/actuation_command.hpp"
#include "adas_msgs/msg/control.hpp"
#include "adas_vehicle_interface/sim_vehicle_interface.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace adas::vehicle {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class VehicleInterfaceNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit VehicleInterfaceNode(const rclcpp::NodeOptions& options)
      : LifecycleNode("vehicle_interface", options) {
    declare_parameter<double>("rate_hz", 50.0);
    declare_parameter<double>("watchdog_timeout_s", 0.2);
    declare_parameter<double>("watchdog_brake", 1.0);
    declare_parameter<double>("watchdog_steer_decay_tau_s", 0.5);
    declare_parameter<std::string>("adapter", "sim");
    declare_parameter<double>("sim.max_accel_mps2", 3.0);
    declare_parameter<double>("sim.max_decel_mps2", 8.0);
    declare_parameter<double>("sim.max_steer_rad", 0.6);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    try {
      rate_hz_ = get_parameter("rate_hz").as_double();
      watchdog_timeout_s_ = get_parameter("watchdog_timeout_s").as_double();
      watchdog_brake_ = get_parameter("watchdog_brake").as_double();
      watchdog_steer_decay_tau_s_ =
          get_parameter("watchdog_steer_decay_tau_s").as_double();
      common::require_timeout_exceeds_period("watchdog_timeout_s", watchdog_timeout_s_,
                                             "rate_hz", rate_hz_);
      common::require_range("watchdog_brake", watchdog_brake_, 0.0, 1.0);
      common::require_nonnegative("watchdog_steer_decay_tau_s",
                                  watchdog_steer_decay_tau_s_);

      const auto adapter = get_parameter("adapter").as_string();
      if (adapter != "sim") {
        throw std::invalid_argument("adapter must be 'sim'");
      }
      SimVehicleInterfaceParams p;
      p.max_accel_mps2 = get_parameter("sim.max_accel_mps2").as_double();
      p.max_decel_mps2 = get_parameter("sim.max_decel_mps2").as_double();
      p.max_steer_rad = get_parameter("sim.max_steer_rad").as_double();
      common::require_positive("sim.max_accel_mps2", p.max_accel_mps2);
      common::require_positive("sim.max_decel_mps2", p.max_decel_mps2);
      common::require_positive("sim.max_steer_rad", p.max_steer_rad);
      impl_ = std::make_unique<SimVehicleInterface>(p);
      adapter_ = adapter;

      sub_cmd_ = create_subscription<adas_msgs::msg::Control>(
          "/adas/control/gate/control_cmd", rclcpp::QoS(1).reliable(),
          [this](adas_msgs::msg::Control::ConstSharedPtr msg) {
            cmd_ = msg;
            cmd_rx_time_ = now();
          });
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
          "/adas/localization/kinematic_state", rclcpp::SensorDataQoS(),
          [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_ = msg; });
      pub_act_ = create_publisher<adas_msgs::msg::ActuationCommand>(
          "/adas/vehicle/actuation_cmd", rclcpp::QoS(1).reliable());
      timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_),
                                 [this]() { on_timer(); });
      timer_->cancel();

      timing_.set_budget_ms(1000.0 / rate_hz_);
      cycle_time_.configure(rate_hz_);
      diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
      diagnostics_->setHardwareID("soc-vehicle-interface");
      diagnostics_->add("runtime", [this](auto& stat) { produce_diagnostics(stat); });
      parameters_valid_ = true;
      last_error_.clear();
      return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
      parameters_valid_ = false;
      last_error_ = e.what();
      last_error_time_ = now();
      RCLCPP_ERROR(get_logger(), "vehicle_interface configure 失败: %s", e.what());
      return CallbackReturn::FAILURE;
    }
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    cmd_.reset();
    odom_.reset();
    cmd_rx_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    timing_.reset();
    cycle_time_.reset();
    output_count_ = 0U;
    watchdog_fired_ = true;
    output_enabled_ = true;
    pub_act_->on_activate();
    timer_->reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    publish_fail_safe();
    output_enabled_ = false;
    if (timer_) timer_->cancel();
    if (pub_act_) pub_act_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    release_resources();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
    publish_fail_safe();
    release_resources();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State&) override {
    last_error_ = "lifecycle error";
    last_error_time_ = now();
    publish_fail_safe();
    output_enabled_ = false;
    if (timer_) timer_->cancel();
    return CallbackReturn::SUCCESS;
  }

 private:
  static bool valid_control(const adas_msgs::msg::Control& msg) {
    return std::isfinite(msg.lateral.steering_tire_angle_rad) &&
           std::isfinite(msg.longitudinal.velocity_mps) &&
           std::isfinite(msg.longitudinal.acceleration_mps2);
  }

  void on_timer() {
    common::ScopedTimingSample sample(timing_);
    const double dt = cycle_time_.tick();
    common::ActuationData act;
    const bool cmd_fresh = cmd_ && valid_control(*cmd_) &&
                           (now() - cmd_rx_time_).seconds() < watchdog_timeout_s_;
    if (cmd_ && !valid_control(*cmd_)) ++invalid_command_count_;

    if (cmd_fresh) {
      common::ControlData cd;
      cd.lateral.steering_tire_angle_rad = cmd_->lateral.steering_tire_angle_rad;
      cd.longitudinal.velocity_mps = cmd_->longitudinal.velocity_mps;
      cd.longitudinal.acceleration_mps2 = cmd_->longitudinal.acceleration_mps2;
      common::KinematicState st;
      if (odom_) st.velocity_mps = odom_->twist.twist.linear.x;
      act = impl_->apply(cd, st);
      last_steer_out_ = act.steer;
      watchdog_fired_ = false;
    } else {
      if (!watchdog_fired_) {
        RCLCPP_ERROR(get_logger(), "看门狗触发：上游命令静默或非法，自主制动");
      }
      watchdog_fired_ = true;
      const double alpha = watchdog_steer_decay_tau_s_ <= 0.0
                               ? 1.0
                               : dt / (watchdog_steer_decay_tau_s_ + dt);
      last_steer_out_ *= (1.0 - alpha);
      act = {0.0, watchdog_brake_, last_steer_out_};
    }
    publish_actuation(act);
  }

  void publish_actuation(const common::ActuationData& act) {
    if (!pub_act_ || !pub_act_->is_activated()) return;
    adas_msgs::msg::ActuationCommand msg;
    msg.header.stamp = now();
    msg.throttle = static_cast<float>(std::clamp(act.throttle, 0.0, 1.0));
    msg.brake = static_cast<float>(std::clamp(act.brake, 0.0, 1.0));
    msg.steer = static_cast<float>(std::clamp(act.steer, -1.0, 1.0));
    last_actuation_ = msg;
    pub_act_->publish(msg);
    ++output_count_;
  }

  void publish_fail_safe() {
    watchdog_fired_ = true;
    publish_actuation(common::ActuationData{0.0, watchdog_brake_, last_steer_out_});
  }

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    const double age = cmd_ ? (now() - cmd_rx_time_).seconds() : -1.0;
    const auto timing = timing_.snapshot();
    const auto cycle = cycle_time_.snapshot();
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string summary = "active";
    if (!parameters_valid_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      summary = "invalid parameters";
    } else if (!output_enabled_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      summary = "inactive";
    } else if (watchdog_fired_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      summary = "watchdog fail-safe active";
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
    stat.add("adapter", adapter_);
    stat.add("command_received", static_cast<bool>(cmd_));
    stat.add("command_age_s", age);
    stat.add("command_timeout_s", watchdog_timeout_s_);
    stat.add("watchdog_active", watchdog_fired_);
    stat.add("output_enabled", output_enabled_);
    stat.add("output_count", output_count_);
    stat.add("invalid_command_count", invalid_command_count_);
    stat.add("last_throttle", last_actuation_.throttle);
    stat.add("last_brake", last_actuation_.brake);
    stat.add("last_steer", last_actuation_.steer);
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

  void release_resources() {
    output_enabled_ = false;
    if (timer_) timer_->cancel();
    timer_.reset();
    diagnostics_.reset();
    pub_act_.reset();
    sub_cmd_.reset();
    sub_odom_.reset();
    impl_.reset();
    cmd_.reset();
    odom_.reset();
  }

  double rate_hz_{50.0};
  double watchdog_timeout_s_{0.2};
  double watchdog_brake_{1.0};
  double watchdog_steer_decay_tau_s_{0.5};
  bool watchdog_fired_{true};
  bool output_enabled_{false};
  bool parameters_valid_{false};
  double last_steer_out_{0.0};
  std::string adapter_;
  std::string last_error_;
  rclcpp::Time last_error_time_{0, 0, RCL_ROS_TIME};
  std::size_t output_count_{0U};
  std::size_t invalid_command_count_{0U};
  common::TimingMonitor timing_;
  common::CycleTimeMonitor cycle_time_;
  adas_msgs::msg::ActuationCommand last_actuation_;

  std::unique_ptr<VehicleInterfaceBase> impl_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  adas_msgs::msg::Control::ConstSharedPtr cmd_;
  rclcpp::Time cmd_rx_time_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  rclcpp::Subscription<adas_msgs::msg::Control>::SharedPtr sub_cmd_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp_lifecycle::LifecyclePublisher<adas_msgs::msg::ActuationCommand>::SharedPtr pub_act_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace adas::vehicle

RCLCPP_COMPONENTS_REGISTER_NODE(adas::vehicle::VehicleInterfaceNode)
