// adas_aeb 生命周期节点：独立风险检测、紧急制动请求和标准诊断。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "adas_aeb/aeb_core.hpp"
#include "adas_common/parameter_validation.hpp"
#include "adas_common/timing_monitor.hpp"
#include "adas_msgs/msg/aeb_status.hpp"
#include "adas_msgs/msg/control.hpp"
#include "adas_msgs/msg/tracked_object_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace adas::control {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AebNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit AebNode(const rclcpp::NodeOptions& options) : LifecycleNode("aeb", options) {
    declare_parameter<double>("horizon_s", 4.0);
    declare_parameter<double>("step_s", 0.1);
    declare_parameter<double>("corridor_half_width_m", 1.3);
    declare_parameter<double>("rear_filter_m", 5.0);
    declare_parameter<double>("corridor_width_m", 3.5);
    declare_parameter<double>("obj_radius_m", 0.5);
    declare_parameter<double>("ttc_emergency_car_s", 1.8);
    declare_parameter<double>("ttc_emergency_ped_s", 2.5);
    declare_parameter<double>("min_active_speed_mps", 1.0);
    declare_parameter<int>("trigger_frames", 3);
    declare_parameter<int>("release_frames", 10);
    declare_parameter<double>("emergency_decel_mps2", 8.0);
    declare_parameter<double>("input_timeout_s", 0.5);
    declare_parameter<double>("rate_hz", 20.0);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    try {
      params_.horizon_s = get_parameter("horizon_s").as_double();
      params_.step_s = get_parameter("step_s").as_double();
      params_.corridor_half_width_m = get_parameter("corridor_half_width_m").as_double();
      params_.rear_filter_m = get_parameter("rear_filter_m").as_double();
      params_.corridor_width_m = get_parameter("corridor_width_m").as_double();
      params_.obj_radius_m = get_parameter("obj_radius_m").as_double();
      params_.ttc_emergency_car_s = get_parameter("ttc_emergency_car_s").as_double();
      params_.ttc_emergency_ped_s = get_parameter("ttc_emergency_ped_s").as_double();
      params_.min_active_speed_mps = get_parameter("min_active_speed_mps").as_double();
      params_.trigger_frames = static_cast<int>(get_parameter("trigger_frames").as_int());
      params_.release_frames = static_cast<int>(get_parameter("release_frames").as_int());
      params_.emergency_decel_mps2 = get_parameter("emergency_decel_mps2").as_double();
      input_timeout_s_ = get_parameter("input_timeout_s").as_double();
      rate_hz_ = get_parameter("rate_hz").as_double();
      validate_parameters();
      core_ = std::make_unique<AebCore>(params_);

      const auto sensor_qos = rclcpp::SensorDataQoS();
      sub_objects_ = create_subscription<adas_msgs::msg::TrackedObjectArray>(
          "/adas/perception/objects_raw", sensor_qos,
          [this](adas_msgs::msg::TrackedObjectArray::ConstSharedPtr msg) {
            objects_ = msg;
            objects_rx_time_ = std::chrono::steady_clock::now();
            objects_received_ = true;
          });
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
          "/adas/localization/kinematic_state", sensor_qos,
          [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
            odom_ = msg;
            odom_rx_time_ = std::chrono::steady_clock::now();
            odom_received_ = true;
          });
      pub_status_ = create_publisher<adas_msgs::msg::AebStatus>(
          "/adas/control/aeb/status", rclcpp::QoS(1).reliable().transient_local());
      pub_cmd_ = create_publisher<adas_msgs::msg::Control>(
          "/adas/control/aeb/emergency_cmd", rclcpp::QoS(1).reliable());
      timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_),
                                 [this]() { on_timer(); });
      timer_->cancel();
      timing_.set_budget_ms(1000.0 / rate_hz_);
      diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
      diagnostics_->setHardwareID("soc-aeb");
      diagnostics_->add("runtime", [this](auto& stat) { produce_diagnostics(stat); });
      parameters_valid_ = true;
      last_error_.clear();
      return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
      parameters_valid_ = false;
      last_error_ = e.what();
      last_error_time_ = now();
      RCLCPP_ERROR(get_logger(), "aeb configure 失败: %s", e.what());
      return CallbackReturn::FAILURE;
    }
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    objects_.reset();
    odom_.reset();
    objects_received_ = false;
    odom_received_ = false;
    core_ = std::make_unique<AebCore>(params_);
    timing_.reset();
    last_emergency_ = false;
    trigger_count_ = 0U;
    release_count_ = 0U;
    output_enabled_ = true;
    pub_status_->on_activate();
    pub_cmd_->on_activate();
    timer_->reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    output_enabled_ = false;
    if (timer_) timer_->cancel();
    if (pub_status_) pub_status_->on_deactivate();
    if (pub_cmd_) pub_cmd_->on_deactivate();
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
    common::require_positive("horizon_s", params_.horizon_s);
    common::require_positive("step_s", params_.step_s);
    if (params_.step_s > params_.horizon_s) {
      throw std::invalid_argument("step_s must not exceed horizon_s");
    }
    common::require_positive("corridor_half_width_m", params_.corridor_half_width_m);
    common::require_nonnegative("rear_filter_m", params_.rear_filter_m);
    common::require_positive("corridor_width_m", params_.corridor_width_m);
    common::require_nonnegative("obj_radius_m", params_.obj_radius_m);
    common::require_positive("ttc_emergency_car_s", params_.ttc_emergency_car_s);
    common::require_positive("ttc_emergency_ped_s", params_.ttc_emergency_ped_s);
    if (params_.ttc_emergency_ped_s < params_.ttc_emergency_car_s) {
      throw std::invalid_argument("ttc_emergency_ped_s must be >= ttc_emergency_car_s");
    }
    common::require_nonnegative("min_active_speed_mps", params_.min_active_speed_mps);
    common::require_positive("emergency_decel_mps2", params_.emergency_decel_mps2);
    if (params_.trigger_frames <= 0 || params_.release_frames <= 0) {
      throw std::invalid_argument("trigger_frames and release_frames must be > 0");
    }
    common::require_timeout_exceeds_period("input_timeout_s", input_timeout_s_, "rate_hz",
                                           rate_hz_);
  }

  double receive_age(const std::chrono::steady_clock::time_point& received,
                     bool present) const {
    if (!present) return -1.0;
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - received).count();
  }

  bool fresh(const std::chrono::steady_clock::time_point& received, bool present) const {
    const double age = receive_age(received, present);
    return age >= 0.0 && age < input_timeout_s_;
  }

  void on_timer() {
    common::ScopedTimingSample sample(timing_);
    adas_msgs::msg::AebStatus status;
    status.header.stamp = now();
    if (!odom_ || !objects_ || !fresh(odom_rx_time_, odom_received_) ||
        !fresh(objects_rx_time_, objects_received_)) {
      status.state = adas_msgs::msg::AebStatus::STATE_INACTIVE;
      status.ttc_s = 1e9f;
      status.reason = "inputs_missing_or_stale";
      last_state_ = status.state;
      last_reason_ = status.reason;
      publish_status(status);
      return;
    }

    common::KinematicState ego;
    ego.pose.x = odom_->pose.pose.position.x;
    ego.pose.y = odom_->pose.pose.position.y;
    ego.pose.yaw =
        2.0 * std::atan2(odom_->pose.pose.orientation.z, odom_->pose.pose.orientation.w);
    ego.velocity_mps = odom_->twist.twist.linear.x;
    ego.yaw_rate_rps = odom_->twist.twist.angular.z;
    std::vector<AebObject> objects;
    objects.reserve(objects_->objects.size());
    for (const auto& object : objects_->objects) {
      AebObject converted;
      converted.x = object.pose.pose.position.x;
      converted.y = object.pose.pose.position.y;
      converted.yaw = 2.0 * std::atan2(object.pose.pose.orientation.z,
                                      object.pose.pose.orientation.w);
      converted.v_mps = object.twist.twist.linear.x;
      converted.classification = object.classification;
      objects.push_back(converted);
    }

    const auto result = core_->update(ego, objects);
    if (result.emergency_active != last_emergency_) {
      if (result.emergency_active) {
        ++trigger_count_;
      } else {
        ++release_count_;
      }
      RCLCPP_WARN(get_logger(), "AEB %s：ttc=%.2fs reason=%s",
                  result.emergency_active ? "触发" : "释放", result.ttc_s,
                  result.reason.c_str());
      last_emergency_ = result.emergency_active;
    }
    last_state_ = static_cast<uint8_t>(result.state);
    last_ttc_s_ = result.ttc_s;
    last_reason_ = result.reason;
    status.state = last_state_;
    status.ttc_s = static_cast<float>(std::min(result.ttc_s, 1e9));
    status.required_decel_mps2 = static_cast<float>(result.required_decel_mps2);
    status.reason = result.reason;
    publish_status(status);

    if (result.emergency_active && pub_cmd_ && pub_cmd_->is_activated()) {
      adas_msgs::msg::Control cmd;
      cmd.header.stamp = status.header.stamp;
      cmd.longitudinal.velocity_mps = 0.0f;
      cmd.longitudinal.acceleration_mps2 = static_cast<float>(result.brake_accel_mps2);
      pub_cmd_->publish(cmd);
      last_brake_request_ = result.brake_accel_mps2;
    }
  }

  void publish_status(const adas_msgs::msg::AebStatus& status) {
    if (pub_status_ && pub_status_->is_activated()) {
      pub_status_->publish(status);
      ++output_count_;
    }
  }

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    const auto timing = timing_.snapshot();
    const double objects_age = receive_age(objects_rx_time_, objects_received_);
    const double odom_age = receive_age(odom_rx_time_, odom_received_);
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string summary = "monitoring";
    if (!parameters_valid_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      summary = "invalid parameters";
    } else if (!output_enabled_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      summary = "inactive";
    } else if (!objects_ || !odom_ || objects_age >= input_timeout_s_ ||
               odom_age >= input_timeout_s_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      summary = "inputs missing or stale";
    } else if (last_emergency_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      summary = "emergency braking active";
    } else if (timing.warning) {
      level = timing.error ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                           : diagnostic_msgs::msg::DiagnosticStatus::WARN;
      summary = "processing budget warning";
    }
    stat.summary(level, summary);
    stat.add("lifecycle_state", get_current_state().label());
    stat.add("parameters_valid", parameters_valid_);
    stat.add("output_enabled", output_enabled_);
    // diagnostic_updater streams uint8_t as a character. STATE_INACTIVE (0)
    // would therefore insert '\0' into KeyValue.value and Fast-CDR rejects the
    // entire DiagnosticArray string during serialization.
    stat.add("aeb_state", static_cast<int>(last_state_));
    stat.add("ttc_s", last_ttc_s_);
    stat.add("car_trigger_threshold_s", params_.ttc_emergency_car_s);
    stat.add("pedestrian_trigger_threshold_s", params_.ttc_emergency_ped_s);
    stat.add("trigger_count", trigger_count_);
    stat.add("release_count", release_count_);
    stat.add("objects_age_s", objects_age);
    stat.add("odometry_age_s", odom_age);
    stat.add("tracked_object_count", objects_ ? objects_->objects.size() : 0U);
    stat.add("brake_request_mps2", last_brake_request_);
    stat.add("reason", last_reason_);
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
    pub_cmd_.reset();
    sub_objects_.reset();
    sub_odom_.reset();
    core_.reset();
    objects_.reset();
    odom_.reset();
  }

  AebParams params_;
  std::unique_ptr<AebCore> core_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  common::TimingMonitor timing_;
  double input_timeout_s_{0.5};
  double rate_hz_{20.0};
  bool last_emergency_{false};
  bool output_enabled_{false};
  bool parameters_valid_{false};
  uint8_t last_state_{adas_msgs::msg::AebStatus::STATE_INACTIVE};
  double last_ttc_s_{1e9};
  double last_brake_request_{0.0};
  std::size_t trigger_count_{0U};
  std::size_t release_count_{0U};
  std::size_t output_count_{0U};
  std::string last_reason_{"not active"};
  std::string last_error_;
  rclcpp::Time last_error_time_{0, 0, RCL_ROS_TIME};
  adas_msgs::msg::TrackedObjectArray::ConstSharedPtr objects_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  std::chrono::steady_clock::time_point objects_rx_time_{};
  std::chrono::steady_clock::time_point odom_rx_time_{};
  bool objects_received_{false};
  bool odom_received_{false};
  rclcpp::Subscription<adas_msgs::msg::TrackedObjectArray>::SharedPtr sub_objects_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp_lifecycle::LifecyclePublisher<adas_msgs::msg::AebStatus>::SharedPtr pub_status_;
  rclcpp_lifecycle::LifecyclePublisher<adas_msgs::msg::Control>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace adas::control

RCLCPP_COMPONENTS_REGISTER_NODE(adas::control::AebNode)
