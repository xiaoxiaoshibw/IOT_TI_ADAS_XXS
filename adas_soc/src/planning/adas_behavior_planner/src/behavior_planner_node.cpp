// adas_behavior_planner 生命周期节点：既有行为状态机适配与运行诊断。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "adas_behavior_planner/behavior_core.hpp"
#include "adas_common/parameter_validation.hpp"
#include "adas_common/timing_monitor.hpp"
#include "adas_msgs/msg/behavior_state.hpp"
#include "adas_msgs/msg/lane_state.hpp"
#include "adas_msgs/msg/map_sign.hpp"
#include "adas_msgs/msg/safety_status.hpp"
#include "adas_msgs/msg/tracked_object_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace adas::planning {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class BehaviorPlannerNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit BehaviorPlannerNode(const rclcpp::NodeOptions& options)
      : LifecycleNode("behavior_planner", options) {
    declare_parameter<double>("cruise_speed_mps", 15.0);
    declare_parameter<double>("follow_enter_range_m", 80.0);
    declare_parameter<double>("follow_exit_range_m", 95.0);
    declare_parameter<int>("exit_hysteresis_frames", 5);
    declare_parameter<bool>("overtake.enabled", true);
    declare_parameter<double>("overtake.slow_ratio", 0.6);
    declare_parameter<double>("overtake.trigger_gap_m", 30.0);
    declare_parameter<int>("overtake.wait_frames", 10);
    declare_parameter<int>("overtake.clear_confirm_frames", 5);
    declare_parameter<double>("overtake.clear_front_m", 60.0);
    declare_parameter<double>("overtake.clear_rear_m", 15.0);
    declare_parameter<double>("overtake.pass_margin_m", 8.0);
    declare_parameter<double>("overtake.lane_attain_tol_m", 0.6);
    declare_parameter<double>("overtake.return_done_lat_m", 0.5);
    declare_parameter<double>("overtake.abort_lat_limit_m", 1.4);
    declare_parameter<int>("overtake.target_lane", -1);
    declare_parameter<double>("overtake.lane_width_m", 3.5);
    declare_parameter<double>("stop_sign_approach_m", 15.0);
    declare_parameter<double>("stop_sign_stop_duration_s", 2.0);
    declare_parameter<double>("traffic_light_approach_m", 30.0);
    declare_parameter<double>("junction_approach_m", 30.0);
    declare_parameter<double>("rate_hz", 10.0);
    declare_parameter<double>("input_timeout_s", 0.6);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    try {
      load_parameters();
      validate_parameters();
      core_ = std::make_unique<BehaviorCore>(params_);
      const auto sensor_qos = rclcpp::SensorDataQoS();
      sub_objects_ = create_subscription<adas_msgs::msg::TrackedObjectArray>(
          "/adas/perception/objects", sensor_qos,
          [this](adas_msgs::msg::TrackedObjectArray::ConstSharedPtr msg) {
            objects_ = msg;
            objects_rx_time_ = now();
          });
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
      sub_map_sign_ = create_subscription<adas_msgs::msg::MapSign>(
          "/adas/map/sign", rclcpp::QoS(1).reliable().transient_local(),
          [this](adas_msgs::msg::MapSign::ConstSharedPtr msg) {
            const auto same_sign = [&msg](const auto& existing) {
              return existing.lane_id == msg->lane_id && existing.type == msg->type &&
                     std::hypot(existing.position.x - msg->position.x,
                                existing.position.y - msg->position.y) < 0.5;
            };
            const auto it = std::find_if(map_signs_.begin(), map_signs_.end(), same_sign);
            if (it == map_signs_.end()) {
              map_signs_.push_back(*msg);
            } else {
              *it = *msg;
            }
          });
      sub_safety_ = create_subscription<adas_msgs::msg::SafetyStatus>(
          "/adas/system/safety_status", rclcpp::QoS(1).reliable().transient_local(),
          [this](adas_msgs::msg::SafetyStatus::ConstSharedPtr msg) { safety_ = msg; });
      pub_behavior_ = create_publisher<adas_msgs::msg::BehaviorState>(
          "/adas/planning/behavior", rclcpp::QoS(1).reliable());
      timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_),
                                 [this]() { on_timer(); });
      timer_->cancel();
      timing_.set_budget_ms(1000.0 / rate_hz_);
      diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
      diagnostics_->setHardwareID("soc-behavior-planner");
      diagnostics_->add("runtime", [this](auto& stat) { produce_diagnostics(stat); });
      parameters_valid_ = true;
      last_error_.clear();
      return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
      parameters_valid_ = false;
      last_error_ = e.what();
      last_error_time_ = now();
      RCLCPP_ERROR(get_logger(), "behavior_planner configure 失败: %s", e.what());
      return CallbackReturn::FAILURE;
    }
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    core_ = std::make_unique<BehaviorCore>(params_);
    objects_.reset();
    odom_.reset();
    lane_.reset();
    map_signs_.clear();
    safety_.reset();
    timing_.reset();
    output_count_ = 0U;
    output_enabled_ = true;
    pub_behavior_->on_activate();
    timer_->reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    output_enabled_ = false;
    if (timer_) timer_->cancel();
    if (pub_behavior_) pub_behavior_->on_deactivate();
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
  void load_parameters() {
    params_.cruise_speed_mps = get_parameter("cruise_speed_mps").as_double();
    params_.follow_enter_range_m = get_parameter("follow_enter_range_m").as_double();
    params_.follow_exit_range_m = get_parameter("follow_exit_range_m").as_double();
    params_.exit_hysteresis_frames =
        static_cast<int>(get_parameter("exit_hysteresis_frames").as_int());
    params_.overtake_enabled = get_parameter("overtake.enabled").as_bool();
    params_.overtake_slow_ratio = get_parameter("overtake.slow_ratio").as_double();
    params_.overtake_trigger_gap_m = get_parameter("overtake.trigger_gap_m").as_double();
    params_.overtake_wait_frames =
        static_cast<int>(get_parameter("overtake.wait_frames").as_int());
    params_.clear_confirm_frames =
        static_cast<int>(get_parameter("overtake.clear_confirm_frames").as_int());
    params_.clear_front_m = get_parameter("overtake.clear_front_m").as_double();
    params_.clear_rear_m = get_parameter("overtake.clear_rear_m").as_double();
    params_.pass_margin_m = get_parameter("overtake.pass_margin_m").as_double();
    params_.lane_attain_tol_m = get_parameter("overtake.lane_attain_tol_m").as_double();
    params_.return_done_lat_m = get_parameter("overtake.return_done_lat_m").as_double();
    params_.abort_lat_limit_m = get_parameter("overtake.abort_lat_limit_m").as_double();
    params_.target_lane = static_cast<int>(get_parameter("overtake.target_lane").as_int());
    params_.lane_width_m = get_parameter("overtake.lane_width_m").as_double();
    params_.stop_sign_approach_m = get_parameter("stop_sign_approach_m").as_double();
    params_.stop_sign_stop_duration_s =
        get_parameter("stop_sign_stop_duration_s").as_double();
    params_.traffic_light_approach_m =
        get_parameter("traffic_light_approach_m").as_double();
    params_.junction_approach_m = get_parameter("junction_approach_m").as_double();
    rate_hz_ = get_parameter("rate_hz").as_double();
    input_timeout_s_ = get_parameter("input_timeout_s").as_double();
  }

  void validate_parameters() const {
    common::require_nonnegative("cruise_speed_mps", params_.cruise_speed_mps);
    common::require_positive("follow_enter_range_m", params_.follow_enter_range_m);
    common::require_positive("follow_exit_range_m", params_.follow_exit_range_m);
    if (params_.follow_exit_range_m <= params_.follow_enter_range_m) {
      throw std::invalid_argument("follow_exit_range_m must exceed follow_enter_range_m");
    }
    if (params_.exit_hysteresis_frames <= 0) {
      throw std::invalid_argument("exit_hysteresis_frames must be > 0");
    }
    common::require_range("overtake.slow_ratio", params_.overtake_slow_ratio, 0.0, 1.0);
    common::require_positive("overtake.lane_width_m", params_.lane_width_m);
    common::require_positive("stop_sign_approach_m", params_.stop_sign_approach_m);
    common::require_positive("stop_sign_stop_duration_s", params_.stop_sign_stop_duration_s);
    common::require_positive("traffic_light_approach_m", params_.traffic_light_approach_m);
    common::require_positive("junction_approach_m", params_.junction_approach_m);
    common::require_timeout_exceeds_period("input_timeout_s", input_timeout_s_, "rate_hz",
                                           rate_hz_);
  }

  void on_timer() {
    common::ScopedTimingSample sample(timing_);
    BehaviorInput input;
    if (objects_) {
      input.primary_lead_id = objects_->primary_lead_id;
      input.lead_gap_m = objects_->primary_lead_gap_m;
      input.lead_speed_mps = objects_->primary_lead_speed_mps;
      input.objects.reserve(objects_->objects.size());
      for (const auto& object : objects_->objects) {
        ObjectLite converted;
        converted.id = object.id;
        converted.lon_m = object.path_longitudinal_m;
        converted.lat_m = object.path_lateral_m;
        converted.v_mps = object.twist.twist.linear.x;
        input.objects.push_back(converted);
      }
    }
    if (odom_) input.ego_speed_mps = odom_->twist.twist.linear.x;
    if (lane_) {
      input.ego_lateral_m = lane_->lateral_offset;
      input.target_lane_available =
          params_.target_lane < 0 ? lane_->left_lane_available
                                  : (params_.target_lane > 0
                                         ? lane_->right_lane_available
                                         : true);
    }
    input.now_s = now().seconds();
    if (odom_) {
      input.map_signs.reserve(map_signs_.size());
      for (const auto& sign : map_signs_) {
        MapSignLite converted;
        converted.type = static_cast<MapSignType>(sign.type);
        converted.distance_m = std::hypot(
            sign.position.x - odom_->pose.pose.position.x,
            sign.position.y - odom_->pose.pose.position.y);
        converted.traffic_light_red =
            sign.traffic_light_state == adas_msgs::msg::MapSign::LIGHT_RED;
        converted.lane_id = sign.lane_id;
        input.map_signs.push_back(converted);
      }
    }
    input.mrm_stop =
        safety_ && safety_->overall >= adas_msgs::msg::SafetyStatus::LEVEL_MRM_COMFORT;
    const auto previous = core_->state();
    const auto output = core_->update(input);
    if (output.state != previous) {
      RCLCPP_INFO(get_logger(), "行为切换: %d → %d (target_lane=%d)",
                  static_cast<int>(previous), static_cast<int>(output.state),
                  output.target_lane);
    }
    last_state_ = output.state;
    last_object_count_ = input.objects.size();
    adas_msgs::msg::BehaviorState msg;
    msg.header.stamp = now();
    msg.state = static_cast<uint8_t>(output.state);
    msg.target_speed_mps = static_cast<float>(output.target_speed_mps);
    msg.target_lane = static_cast<int8_t>(output.target_lane);
    if (pub_behavior_ && pub_behavior_->is_activated()) {
      pub_behavior_->publish(msg);
      ++output_count_;
    }
  }

  double age(const rclcpp::Time& received, bool present) const {
    return present ? (now() - received).seconds() : -1.0;
  }

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    const auto timing = timing_.snapshot();
    const double object_age = age(objects_rx_time_, static_cast<bool>(objects_));
    const double odom_age = age(odom_rx_time_, static_cast<bool>(odom_));
    const double lane_age = age(lane_rx_time_, static_cast<bool>(lane_));
    const bool inputs_fresh = objects_ && odom_ && lane_ && object_age < input_timeout_s_ &&
                              odom_age < input_timeout_s_ && lane_age < input_timeout_s_;
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string summary = "behavior state valid";
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
    stat.add("behavior_state", static_cast<int>(last_state_));
    stat.add("objects_age_s", object_age);
    stat.add("odometry_age_s", odom_age);
    stat.add("lane_age_s", lane_age);
    stat.add("input_timeout_s", input_timeout_s_);
    stat.add("object_count", last_object_count_);
    stat.add("mrm_stop", safety_ && safety_->overall >=
                                  adas_msgs::msg::SafetyStatus::LEVEL_MRM_COMFORT);
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
    pub_behavior_.reset();
    sub_objects_.reset();
    sub_odom_.reset();
    sub_lane_.reset();
    sub_map_sign_.reset();
    sub_safety_.reset();
    core_.reset();
  }

  BehaviorParams params_;
  double rate_hz_{10.0};
  double input_timeout_s_{0.6};
  std::unique_ptr<BehaviorCore> core_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  common::TimingMonitor timing_;
  bool output_enabled_{false};
  bool parameters_valid_{false};
  std::size_t output_count_{0U};
  std::size_t last_object_count_{0U};
  BehaviorKind last_state_{BehaviorKind::kLaneFollow};
  std::string last_error_;
  rclcpp::Time last_error_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time objects_rx_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_rx_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time lane_rx_time_{0, 0, RCL_ROS_TIME};
  adas_msgs::msg::TrackedObjectArray::ConstSharedPtr objects_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  adas_msgs::msg::LaneState::ConstSharedPtr lane_;
  adas_msgs::msg::SafetyStatus::ConstSharedPtr safety_;
  std::vector<adas_msgs::msg::MapSign> map_signs_;
  rclcpp::Subscription<adas_msgs::msg::TrackedObjectArray>::SharedPtr sub_objects_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<adas_msgs::msg::LaneState>::SharedPtr sub_lane_;
  rclcpp::Subscription<adas_msgs::msg::MapSign>::SharedPtr sub_map_sign_;
  rclcpp::Subscription<adas_msgs::msg::SafetyStatus>::SharedPtr sub_safety_;
  rclcpp_lifecycle::LifecyclePublisher<adas_msgs::msg::BehaviorState>::SharedPtr pub_behavior_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace adas::planning

RCLCPP_COMPONENTS_REGISTER_NODE(adas::planning::BehaviorPlannerNode)
