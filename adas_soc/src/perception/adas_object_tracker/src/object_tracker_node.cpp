// adas_object_tracker 生命周期节点：事件驱动目标跟踪与运行诊断。
#include <cmath>
#include <memory>
#include <vector>

#include "adas_common/parameter_validation.hpp"
#include "adas_common/timing_monitor.hpp"
#include "adas_msgs/msg/lane_state.hpp"
#include "adas_msgs/msg/tracked_object_array.hpp"
#include "adas_object_tracker/object_tracker_core.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace adas::perception {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ObjectTrackerNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit ObjectTrackerNode(const rclcpp::NodeOptions& options)
      : LifecycleNode("object_tracker", options) {
    declare_parameter<double>("lane_margin_factor", 0.9);
    declare_parameter<double>("max_lead_range_m", 120.0);
    declare_parameter<double>("v_filter_tau_s", 0.3);
    declare_parameter<double>("a_filter_tau_s", 0.5);
    declare_parameter<double>("track_stale_s", 0.5);
    declare_parameter<double>("expected_rate_hz", 20.0);
    declare_parameter<double>("input_timeout_s", 0.6);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    try {
      params_.lane_margin_factor = get_parameter("lane_margin_factor").as_double();
      params_.max_lead_range_m = get_parameter("max_lead_range_m").as_double();
      params_.v_filter_tau_s = get_parameter("v_filter_tau_s").as_double();
      params_.a_filter_tau_s = get_parameter("a_filter_tau_s").as_double();
      params_.track_stale_s = get_parameter("track_stale_s").as_double();
      expected_rate_hz_ = get_parameter("expected_rate_hz").as_double();
      input_timeout_s_ = get_parameter("input_timeout_s").as_double();
      validate_parameters();
      core_ = std::make_unique<ObjectTrackerCore>(params_);
      const auto sensor_qos = rclcpp::SensorDataQoS();
      sub_raw_ = create_subscription<adas_msgs::msg::TrackedObjectArray>(
          "/adas/perception/objects_raw", sensor_qos,
          [this](adas_msgs::msg::TrackedObjectArray::ConstSharedPtr msg) { on_raw(msg); });
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
          "/adas/localization/kinematic_state", sensor_qos,
          [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
            odom_ = msg;
            odom_rx_time_ = now();
          });
      sub_lane_ = create_subscription<adas_msgs::msg::LaneState>(
          "/adas/perception/lane_state", sensor_qos,
          [this](adas_msgs::msg::LaneState::ConstSharedPtr msg) {
            lane_ = msg;
            lane_rx_time_ = now();
          });
      pub_objects_ = create_publisher<adas_msgs::msg::TrackedObjectArray>(
          "/adas/perception/objects", sensor_qos);
      timing_.set_budget_ms(1000.0 / expected_rate_hz_);
      diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
      diagnostics_->setHardwareID("soc-object-tracker");
      diagnostics_->add("runtime", [this](auto& stat) { produce_diagnostics(stat); });
      parameters_valid_ = true;
      last_error_.clear();
      return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
      parameters_valid_ = false;
      last_error_ = e.what();
      last_error_time_ = now();
      RCLCPP_ERROR(get_logger(), "object_tracker configure 失败: %s", e.what());
      return CallbackReturn::FAILURE;
    }
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    core_ = std::make_unique<ObjectTrackerCore>(params_);
    odom_.reset();
    lane_.reset();
    timing_.reset();
    raw_received_ = false;
    output_count_ = 0U;
    output_enabled_ = true;
    pub_objects_->on_activate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    output_enabled_ = false;
    if (pub_objects_) pub_objects_->on_deactivate();
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
    return CallbackReturn::SUCCESS;
  }

 private:
  void validate_parameters() const {
    common::require_range("lane_margin_factor", params_.lane_margin_factor, 0.0, 1.0);
    common::require_positive("max_lead_range_m", params_.max_lead_range_m);
    common::require_nonnegative("v_filter_tau_s", params_.v_filter_tau_s);
    common::require_nonnegative("a_filter_tau_s", params_.a_filter_tau_s);
    common::require_positive("track_stale_s", params_.track_stale_s);
    common::require_timeout_exceeds_period("input_timeout_s", input_timeout_s_,
                                           "expected_rate_hz", expected_rate_hz_);
  }

  void on_raw(adas_msgs::msg::TrackedObjectArray::ConstSharedPtr msg) {
    raw_received_ = true;
    raw_rx_time_ = now();
    last_raw_count_ = msg->objects.size();
    if (!output_enabled_ || !odom_ || !lane_) return;
    common::ScopedTimingSample sample(timing_);
    common::KinematicState ego;
    ego.pose.x = odom_->pose.pose.position.x;
    ego.pose.y = odom_->pose.pose.position.y;
    ego.pose.yaw =
        2.0 * std::atan2(odom_->pose.pose.orientation.z, odom_->pose.pose.orientation.w);
    ego.velocity_mps = odom_->twist.twist.linear.x;
    common::LaneStateData lane;
    lane.valid = lane_->valid;
    lane.lateral_offset = lane_->lateral_offset;
    lane.heading_error = lane_->heading_error;
    lane.curvature = lane_->curvature;
    lane.lane_width = lane_->lane_width;
    std::vector<RawObject> raw;
    raw.reserve(msg->objects.size());
    for (const auto& object : msg->objects) {
      RawObject converted;
      converted.id = object.id;
      converted.classification = object.classification;
      converted.x = object.pose.pose.position.x;
      converted.y = object.pose.pose.position.y;
      converted.yaw = 2.0 * std::atan2(object.pose.pose.orientation.z,
                                      object.pose.pose.orientation.w);
      converted.v_mps = object.twist.twist.linear.x;
      raw.push_back(converted);
    }
    const auto output = core_->update(now().seconds(), raw, ego, lane);
    if (output.lead_swapped) {
      RCLCPP_INFO(get_logger(), "主前车切换 → id=%d gap=%.1fm", output.primary_lead_id,
                  output.primary_lead_gap_m);
    }
    last_primary_lead_id_ = output.primary_lead_id;
    last_output_count_ = output.objects.size();
    adas_msgs::msg::TrackedObjectArray result;
    result.header = msg->header;
    result.primary_lead_id = output.primary_lead_id;
    result.primary_lead_gap_m = static_cast<float>(output.primary_lead_gap_m);
    result.primary_lead_speed_mps = static_cast<float>(output.primary_lead_speed_mps);
    result.objects.reserve(output.objects.size());
    for (const auto& tracked : output.objects) {
      adas_msgs::msg::TrackedObject object;
      object.id = tracked.raw.id;
      object.classification = tracked.raw.classification;
      object.pose.pose.position.x = tracked.raw.x;
      object.pose.pose.position.y = tracked.raw.y;
      object.pose.pose.orientation.z = std::sin(tracked.raw.yaw / 2.0);
      object.pose.pose.orientation.w = std::cos(tracked.raw.yaw / 2.0);
      object.twist.twist.linear.x = tracked.v_filtered_mps;
      object.path_longitudinal_m = static_cast<float>(tracked.gap_m);
      object.path_lateral_m = static_cast<float>(tracked.lat_m);
      result.objects.push_back(object);
    }
    if (pub_objects_ && pub_objects_->is_activated()) {
      pub_objects_->publish(result);
      ++output_count_;
    }
  }

  double age(const rclcpp::Time& received, bool present) const {
    return present ? (now() - received).seconds() : -1.0;
  }

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    const auto timing = timing_.snapshot();
    const double raw_age = age(raw_rx_time_, raw_received_);
    const double odom_age = age(odom_rx_time_, static_cast<bool>(odom_));
    const double lane_age = age(lane_rx_time_, static_cast<bool>(lane_));
    const bool inputs_fresh = raw_received_ && odom_ && lane_ && raw_age < input_timeout_s_ &&
                              odom_age < input_timeout_s_ && lane_age < input_timeout_s_;
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string summary = "tracking";
    if (!parameters_valid_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      summary = "invalid parameters";
    } else if (!output_enabled_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      summary = "inactive";
    } else if (!inputs_fresh) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      summary = "inputs missing or stale";
    } else if (timing.warning) {
      level = timing.error ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                           : diagnostic_msgs::msg::DiagnosticStatus::WARN;
      summary = "processing budget warning";
    }
    stat.summary(level, summary);
    stat.add("lifecycle_state", get_current_state().label());
    stat.add("parameters_valid", parameters_valid_);
    stat.add("output_enabled", output_enabled_);
    stat.add("raw_objects_age_s", raw_age);
    stat.add("odometry_age_s", odom_age);
    stat.add("lane_age_s", lane_age);
    stat.add("input_timeout_s", input_timeout_s_);
    stat.add("raw_object_count", last_raw_count_);
    stat.add("tracked_object_count", last_output_count_);
    stat.add("primary_lead_id", last_primary_lead_id_);
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
    diagnostics_.reset();
    pub_objects_.reset();
    sub_raw_.reset();
    sub_odom_.reset();
    sub_lane_.reset();
    core_.reset();
    odom_.reset();
    lane_.reset();
  }

  TrackerParams params_;
  double expected_rate_hz_{20.0};
  double input_timeout_s_{0.6};
  std::unique_ptr<ObjectTrackerCore> core_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  common::TimingMonitor timing_;
  bool output_enabled_{false};
  bool parameters_valid_{false};
  bool raw_received_{false};
  std::size_t output_count_{0U};
  std::size_t last_raw_count_{0U};
  std::size_t last_output_count_{0U};
  int last_primary_lead_id_{-1};
  std::string last_error_;
  rclcpp::Time last_error_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time raw_rx_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_rx_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time lane_rx_time_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  adas_msgs::msg::LaneState::ConstSharedPtr lane_;
  rclcpp::Subscription<adas_msgs::msg::TrackedObjectArray>::SharedPtr sub_raw_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<adas_msgs::msg::LaneState>::SharedPtr sub_lane_;
  rclcpp_lifecycle::LifecyclePublisher<adas_msgs::msg::TrackedObjectArray>::SharedPtr
      pub_objects_;
};

}  // namespace adas::perception

RCLCPP_COMPONENTS_REGISTER_NODE(adas::perception::ObjectTrackerNode)
