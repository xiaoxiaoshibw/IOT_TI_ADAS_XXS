// adas_trajectory_planner 生命周期节点：局部轨迹规划与输入/耗时诊断。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

#include "adas_common/parameter_validation.hpp"
#include "adas_common/timing_monitor.hpp"
#include "adas_msgs/msg/behavior_state.hpp"
#include "adas_msgs/msg/lane_state.hpp"
#include "adas_msgs/msg/tracked_object_array.hpp"
#include "adas_msgs/msg/trajectory.hpp"
#include "adas_trajectory_planner/trajectory_planner_core.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace adas::planning {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TrajectoryPlannerNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit TrajectoryPlannerNode(const rclcpp::NodeOptions& options)
      : LifecycleNode("trajectory_planner", options) {
    declare_parameter<double>("horizon_s", 8.0);
    declare_parameter<double>("min_length_m", 30.0);
    declare_parameter<double>("max_length_m", 120.0);
    declare_parameter<double>("step_m", 1.0);
    declare_parameter<double>("cruise_speed_mps", 15.0);
    declare_parameter<double>("max_lat_accel_mps2", 2.5);
    declare_parameter<double>("max_accel_mps2", 1.5);
    declare_parameter<double>("max_decel_mps2", 2.0);
    declare_parameter<double>("follow_time_gap_s", 1.1);
    declare_parameter<double>("follow_standstill_m", 4.0);
    declare_parameter<double>("lane_change_time_s", 3.0);
    declare_parameter<double>("lane_change_min_len_m", 25.0);
    declare_parameter<double>("lane_width_m", 3.5);
    declare_parameter<double>("input_timeout_s", 0.3);
    // GUI 点选目标后，global_planner 发布的 CARLA 地图全局路径直接作为局部轨迹
    // 参考线；没有导航路径时仍使用原有 lane_state 重建路径，保证 ACC/AEB/SIL 不变。
    declare_parameter<bool>("global_route.enabled", true);
    declare_parameter<double>("global_route.goal_stop_distance_m", 1.5);
    // Commit 2 — global-route-specific ACC params. Distinct from the
    // lane-state plan() defaults (follow_time_gap_s/standstill_m) because the
    // route's horizon is longer and the curve-aware profile allows earlier,
    // smoother lead-vehicle deceleration. The lane-state plan() path keeps
    // its 1.1 s/4 m defaults untouched.
    declare_parameter<double>("global_route.lead_time_gap_s", 1.4);
    declare_parameter<double>("global_route.lead_standstill_m", 4.0);
    declare_parameter<bool>("lateral_avoidance_enabled", false);
    // 20 Hz bounds the behavior->trajectory scheduler wait to 50 ms while
    // behavior decisions remain at 10 Hz. The core is O(horizon/step) and is
    // intentionally much lighter than perception or CARLA processing.
    declare_parameter<double>("rate_hz", 20.0);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    try {
      load_parameters();
      validate_parameters();
      core_ = std::make_unique<TrajectoryPlannerCore>(params_);
      const auto sensor_qos = rclcpp::SensorDataQoS();
      sub_lane_ = create_subscription<adas_msgs::msg::LaneState>(
          "/adas/perception/lane_state", sensor_qos,
          [this](adas_msgs::msg::LaneState::ConstSharedPtr msg) {
            lane_ = msg;
            lane_rx_time_ = std::chrono::steady_clock::now();
            lane_received_ = true;
          });
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
          "/adas/localization/kinematic_state", sensor_qos,
          [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
            odom_ = msg;
            odom_rx_time_ = std::chrono::steady_clock::now();
            odom_received_ = true;
          });
      sub_objects_ = create_subscription<adas_msgs::msg::TrackedObjectArray>(
          "/adas/perception/objects", sensor_qos,
          [this](adas_msgs::msg::TrackedObjectArray::ConstSharedPtr msg) {
            objects_ = msg;
            objects_rx_time_ = std::chrono::steady_clock::now();
            objects_received_ = true;
          });
      sub_behavior_ = create_subscription<adas_msgs::msg::BehaviorState>(
          "/adas/planning/behavior", rclcpp::QoS(1).reliable(),
          [this](adas_msgs::msg::BehaviorState::ConstSharedPtr msg) {
            behavior_ = msg;
            behavior_rx_time_ = std::chrono::steady_clock::now();
            behavior_received_ = true;
          });
      {
        auto route_qos = rclcpp::QoS(1).reliable().transient_local();
        sub_global_route_ = create_subscription<nav_msgs::msg::Path>(
            "/adas/planning/global_route", route_qos,
            [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
              global_route_ = msg;
              global_route_received_ = !msg->poses.empty();
            });
      }
      pub_traj_ = create_publisher<adas_msgs::msg::Trajectory>(
          "/adas/planning/trajectory", rclcpp::QoS(1).reliable());
      timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_),
                                 [this]() { on_timer(); });
      timer_->cancel();
      timing_.set_budget_ms(1000.0 / rate_hz_);
      diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
      diagnostics_->setHardwareID("soc-trajectory-planner");
      diagnostics_->add("runtime", [this](auto& stat) { produce_diagnostics(stat); });
      parameters_valid_ = true;
      last_error_.clear();
      return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
      parameters_valid_ = false;
      last_error_ = e.what();
      last_error_time_ = now();
      RCLCPP_ERROR(get_logger(), "trajectory_planner configure 失败: %s", e.what());
      return CallbackReturn::FAILURE;
    }
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    lane_.reset();
    odom_.reset();
    objects_.reset();
    behavior_.reset();
    global_route_.reset();
    lane_received_ = odom_received_ = objects_received_ = behavior_received_ = false;
    global_route_received_ = false;
    timing_.reset();
    output_count_ = 0U;
    output_enabled_ = true;
    pub_traj_->on_activate();
    timer_->reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    output_enabled_ = false;
    if (timer_) timer_->cancel();
    if (pub_traj_) pub_traj_->on_deactivate();
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
    params_.horizon_s = get_parameter("horizon_s").as_double();
    params_.min_length_m = get_parameter("min_length_m").as_double();
    params_.max_length_m = get_parameter("max_length_m").as_double();
    params_.step_m = get_parameter("step_m").as_double();
    params_.cruise_speed_mps = get_parameter("cruise_speed_mps").as_double();
    params_.max_lat_accel_mps2 = get_parameter("max_lat_accel_mps2").as_double();
    params_.max_accel_mps2 = get_parameter("max_accel_mps2").as_double();
    params_.max_decel_mps2 = get_parameter("max_decel_mps2").as_double();
    params_.follow_time_gap_s = get_parameter("follow_time_gap_s").as_double();
    params_.follow_standstill_m = get_parameter("follow_standstill_m").as_double();
    params_.global_route_follow_time_gap_s =
        get_parameter("global_route.lead_time_gap_s").as_double();
    params_.global_route_follow_standstill_m =
        get_parameter("global_route.lead_standstill_m").as_double();
    params_.lane_change_time_s = get_parameter("lane_change_time_s").as_double();
    params_.lane_change_min_len_m = get_parameter("lane_change_min_len_m").as_double();
    params_.lane_width_m = get_parameter("lane_width_m").as_double();
    params_.lateral_avoidance_enabled =
        get_parameter("lateral_avoidance_enabled").as_bool();
    input_timeout_s_ = get_parameter("input_timeout_s").as_double();
    global_route_enabled_ = get_parameter("global_route.enabled").as_bool();
    global_route_goal_stop_distance_m_ =
        get_parameter("global_route.goal_stop_distance_m").as_double();
    rate_hz_ = get_parameter("rate_hz").as_double();
  }

  void validate_parameters() const {
    common::require_positive("horizon_s", params_.horizon_s);
    common::require_positive("min_length_m", params_.min_length_m);
    common::require_positive("max_length_m", params_.max_length_m);
    if (params_.max_length_m < params_.min_length_m) {
      throw std::invalid_argument("max_length_m must be >= min_length_m");
    }
    common::require_positive("step_m", params_.step_m);
    common::require_nonnegative("cruise_speed_mps", params_.cruise_speed_mps);
    common::require_positive("max_lat_accel_mps2", params_.max_lat_accel_mps2);
    common::require_positive("max_accel_mps2", params_.max_accel_mps2);
    common::require_positive("max_decel_mps2", params_.max_decel_mps2);
    common::require_positive("follow_time_gap_s", params_.follow_time_gap_s);
    common::require_nonnegative("follow_standstill_m", params_.follow_standstill_m);
    common::require_positive("global_route.lead_time_gap_s",
                             params_.global_route_follow_time_gap_s);
    common::require_nonnegative("global_route.lead_standstill_m",
                                params_.global_route_follow_standstill_m);
    common::require_positive("lane_width_m", params_.lane_width_m);
    common::require_timeout_exceeds_period("input_timeout_s", input_timeout_s_, "rate_hz",
                                           rate_hz_);
    common::require_nonnegative("global_route.goal_stop_distance_m",
                                global_route_goal_stop_distance_m_);
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
    if (!lane_ || !odom_ || !fresh(lane_rx_time_, lane_received_) ||
        !fresh(odom_rx_time_, odom_received_)) {
      inputs_valid_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "输入缺失/过期，本拍不发轨迹");
      return;
    }
    inputs_valid_ = true;
    common::KinematicState ego;
    ego.pose.x = odom_->pose.pose.position.x;
    ego.pose.y = odom_->pose.pose.position.y;
    ego.pose.yaw =
        2.0 * std::atan2(odom_->pose.pose.orientation.z, odom_->pose.pose.orientation.w);
    ego.velocity_mps = odom_->twist.twist.linear.x;
    ego.yaw_rate_rps = odom_->twist.twist.angular.z;
    common::LaneStateData lane;
    lane.valid = lane_->valid;
    lane.lateral_offset = lane_->lateral_offset;
    lane.heading_error = lane_->heading_error;
    lane.curvature = lane_->curvature;
    lane.lane_width = lane_->lane_width;
    LeadInfo lead;
    std::vector<StaticObstacle> obstacles;
    if (objects_ && fresh(objects_rx_time_, objects_received_)) {
      obstacles.reserve(objects_->objects.size());
      for (const auto& object : objects_->objects) {
        // The current message contract has no STATIC class. Unknown-class
        // detections are the conservative static-obstacle input; classified
        // vehicles/pedestrians remain on the normal dynamic-object path.
        obstacles.push_back({object.path_longitudinal_m, object.path_lateral_m,
                             object.classification ==
                                 adas_msgs::msg::TrackedObject::CLASS_UNKNOWN});
      }
    }
    if (objects_ && objects_->primary_lead_id >= 0 &&
        fresh(objects_rx_time_, objects_received_)) {
      lead.present = true;
      lead.gap_m = objects_->primary_lead_gap_m;
      lead.speed_mps = objects_->primary_lead_speed_mps;
    }
    double cruise_override = -1.0;
    int target_lane = 0;
    if (behavior_ && fresh(behavior_rx_time_, behavior_received_)) {
      cruise_override = behavior_->target_speed_mps;
      target_lane = behavior_->target_lane;
    }
    const auto trajectory = global_route_enabled_ && global_route_ &&
                                    global_route_->poses.size() >= 2U
                                ? plan_global_route(ego, lane, cruise_override, lead, obstacles)
                                : core_->plan(ego, lane, lead, cruise_override, target_lane);
    if (trajectory.empty()) {
      output_valid_ = false;
      return;
    }
    output_valid_ = true;
    last_trajectory_points_ = trajectory.size();
    adas_msgs::msg::Trajectory msg;
    msg.header.stamp = now();
    msg.header.frame_id = "odom";
    msg.points.reserve(trajectory.size());
    for (const auto& point : trajectory) {
      adas_msgs::msg::TrajectoryPoint converted;
      converted.time_from_start = rclcpp::Duration::from_seconds(point.time_from_start_s);
      converted.pose.position.x = point.x;
      converted.pose.position.y = point.y;
      converted.pose.orientation.z = std::sin(point.yaw / 2.0);
      converted.pose.orientation.w = std::cos(point.yaw / 2.0);
      converted.longitudinal_velocity_mps = static_cast<float>(point.velocity_mps);
      converted.acceleration_mps2 = static_cast<float>(point.acceleration_mps2);
      converted.curvature = static_cast<float>(point.curvature);
      msg.points.push_back(converted);
    }
    if (pub_traj_ && pub_traj_->is_activated()) {
      pub_traj_->publish(msg);
      ++output_count_;
    }
  }

  common::Trajectory plan_global_route(const common::KinematicState& ego,
                                       const common::LaneStateData& lane,
                                       double cruise_override_mps,
                                       const LeadInfo& lead,
                                       const std::vector<StaticObstacle>& obstacles) const {
    // Commit 2 — 把 node 内的内联实现委托给 core::plan_global_route（lead 沿
    // 路由做跟车共移 cap，详见 core 实现）。node 仅负责从 nav_msgs/Path 截取
    // 滚动视界内的 pose 序列 + 转换 yaw。core 负责 cruise ∩ curve ∩ stop
    // ∩ lead 四重 cap、backward pass、forward pass。
    const auto& poses = global_route_->poses;
    std::size_t nearest = 0U;
    double nearest_d2 = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < poses.size(); ++i) {
      const double dx = poses[i].pose.position.x - ego.pose.x;
      const double dy = poses[i].pose.position.y - ego.pose.y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < nearest_d2) {
        nearest_d2 = d2;
        nearest = i;
      }
    }

    common::Trajectory route;
    route.reserve(poses.size() - nearest);
    double covered_m = 0.0;
    for (std::size_t i = nearest; i < poses.size(); ++i) {
      if (i > nearest) {
        const auto& before = poses[i - 1U].pose.position;
        const auto& current = poses[i].pose.position;
        covered_m += std::hypot(current.x - before.x, current.y - before.y);
        if (covered_m > params_.max_length_m) break;
      }
      common::TrajPoint point;
      point.x = poses[i].pose.position.x;
      point.y = poses[i].pose.position.y;
      point.yaw = 2.0 * std::atan2(poses[i].pose.orientation.z,
                                   poses[i].pose.orientation.w);
      route.push_back(point);
    }
    if (route.size() < 2U) return {};
    // The route is normally a rolling horizon. Only apply the terminal stop
    // profile when this window actually reaches the user-selected goal.
    const bool reaches_goal = nearest + route.size() == poses.size();
    (void)lane;  // 当前 core::plan_global_route 暂不依赖 lane；保留接口以备
                  // 后续把 error_curvature 等车道信号也接入路线 cap。
    const double cruise = std::max(
        0.0, cruise_override_mps >= 0.0 ? cruise_override_mps : params_.cruise_speed_mps);
    return core_->plan_global_route(ego, route, cruise,
                                    global_route_goal_stop_distance_m_, reaches_goal, lead,
                                    obstacles);
  }

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    const auto timing = timing_.snapshot();
    const double lane_age = receive_age(lane_rx_time_, lane_received_);
    const double odom_age = receive_age(odom_rx_time_, odom_received_);
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string summary = "planning";
    if (!parameters_valid_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      summary = "invalid parameters";
    } else if (!output_enabled_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      summary = "inactive";
    } else if (!inputs_valid_ || !output_valid_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      summary = "input or trajectory invalid";
    } else if (timing.warning) {
      level = timing.error ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                           : diagnostic_msgs::msg::DiagnosticStatus::WARN;
      summary = "processing budget warning";
    }
    stat.summary(level, summary);
    stat.add("lifecycle_state", get_current_state().label());
    stat.add("parameters_valid", parameters_valid_);
    stat.add("output_enabled", output_enabled_);
    stat.add("inputs_valid", inputs_valid_);
    stat.add("trajectory_valid", output_valid_);
    stat.add("lane_age_s", lane_age);
    stat.add("odometry_age_s", odom_age);
    stat.add("objects_received", static_cast<bool>(objects_));
    stat.add("behavior_received", static_cast<bool>(behavior_));
    stat.add("global_route_enabled", global_route_enabled_);
    stat.add("global_route_received", global_route_received_);
    stat.add("trajectory_point_count", last_trajectory_points_);
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
    pub_traj_.reset();
    sub_lane_.reset();
    sub_odom_.reset();
    sub_objects_.reset();
    sub_behavior_.reset();
    sub_global_route_.reset();
    core_.reset();
  }

  PlannerParams params_;
  std::unique_ptr<TrajectoryPlannerCore> core_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  common::TimingMonitor timing_;
  double input_timeout_s_{0.3};
  double rate_hz_{20.0};
  bool global_route_enabled_{true};
  double global_route_goal_stop_distance_m_{1.5};
  bool output_enabled_{false};
  bool parameters_valid_{false};
  bool inputs_valid_{false};
  bool output_valid_{true};
  std::size_t output_count_{0U};
  std::size_t last_trajectory_points_{0U};
  std::string last_error_;
  rclcpp::Time last_error_time_{0, 0, RCL_ROS_TIME};
  adas_msgs::msg::LaneState::ConstSharedPtr lane_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  adas_msgs::msg::TrackedObjectArray::ConstSharedPtr objects_;
  adas_msgs::msg::BehaviorState::ConstSharedPtr behavior_;
  nav_msgs::msg::Path::ConstSharedPtr global_route_;
  std::chrono::steady_clock::time_point lane_rx_time_{};
  std::chrono::steady_clock::time_point odom_rx_time_{};
  std::chrono::steady_clock::time_point objects_rx_time_{};
  std::chrono::steady_clock::time_point behavior_rx_time_{};
  bool lane_received_{false};
  bool odom_received_{false};
  bool objects_received_{false};
  bool behavior_received_{false};
  bool global_route_received_{false};
  rclcpp::Subscription<adas_msgs::msg::LaneState>::SharedPtr sub_lane_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<adas_msgs::msg::TrackedObjectArray>::SharedPtr sub_objects_;
  rclcpp::Subscription<adas_msgs::msg::BehaviorState>::SharedPtr sub_behavior_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_route_;
  rclcpp_lifecycle::LifecyclePublisher<adas_msgs::msg::Trajectory>::SharedPtr pub_traj_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace adas::planning

RCLCPP_COMPONENTS_REGISTER_NODE(adas::planning::TrajectoryPlannerNode)
