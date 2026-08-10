#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "adas_global_planner/global_planner_core.hpp"
#include "adas_global_planner/route_adapter.hpp"
#include "adas_global_planner/route_validator.hpp"
#include "adas_global_planner/semantic_route.hpp"
#include "adas_msgs/msg/global_route.hpp"
#include "adas_msgs/msg/lane_graph.hpp"
#include "adas_msgs/msg/navigation_status.hpp"
#include "adas_msgs/msg/route_point.hpp"
#include "adas_msgs/srv/cancel_navigation.hpp"
#include "adas_msgs/srv/set_navigation_goal.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace adas::planning {
namespace {

Maneuver maneuver_from_msg(std::uint8_t value) {
  switch (value) {
    case adas_msgs::msg::LaneConnection::LEFT: return Maneuver::kLeft;
    case adas_msgs::msg::LaneConnection::RIGHT: return Maneuver::kRight;
    case adas_msgs::msg::LaneConnection::LANE_CHANGE_LEFT: return Maneuver::kLaneChangeLeft;
    case adas_msgs::msg::LaneConnection::LANE_CHANGE_RIGHT: return Maneuver::kLaneChangeRight;
    default: return Maneuver::kStraight;
  }
}

std::uint8_t maneuver_to_msg(const SemanticRoutePoint& point) {
  if (point.stop) return adas_msgs::msg::RoutePoint::MANEUVER_STOP;
  switch (point.maneuver) {
    case Maneuver::kLeft: return adas_msgs::msg::RoutePoint::MANEUVER_LEFT;
    case Maneuver::kRight: return adas_msgs::msg::RoutePoint::MANEUVER_RIGHT;
    case Maneuver::kLaneChangeLeft:
      return adas_msgs::msg::RoutePoint::MANEUVER_LANE_CHANGE_LEFT;
    case Maneuver::kLaneChangeRight:
      return adas_msgs::msg::RoutePoint::MANEUVER_LANE_CHANGE_RIGHT;
    case Maneuver::kStraight: return adas_msgs::msg::RoutePoint::MANEUVER_STRAIGHT;
  }
  return adas_msgs::msg::RoutePoint::MANEUVER_UNKNOWN;
}

double yaw_from_pose(const geometry_msgs::msg::Pose& pose) {
  return 2.0 * std::atan2(pose.orientation.z, pose.orientation.w);
}

}  // namespace

class GlobalPlannerNode : public rclcpp::Node {
 public:
  GlobalPlannerNode() : Node("global_planner") {
    RCLCPP_INFO(get_logger(), "initializing semantic global planner");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    RCLCPP_INFO(get_logger(), "TF listener ready");
    PlannerCost cost;
    cost.lane_change_penalty_m = declare_parameter("lane_change_penalty_m", 8.0);
    cost.junction_penalty_m = declare_parameter("junction_penalty_m", 2.0);
    cost.turn_penalty_m = declare_parameter("turn_penalty_m", 1.0);
    cost.snap_max_distance_m = declare_parameter("snap_max_distance_m", 8.0);
    cost.lane_change_min_length_m = declare_parameter(
        "route_geometry.lane_change_min_length_m", 10.0);
    cost.lane_change_max_lateral_m = declare_parameter(
        "route_geometry.lane_change_max_lateral_m", 6.0);
    route_geometry_config_.sample_spacing_m = declare_parameter(
        "route_geometry.sample_spacing_m", 2.0);
    route_geometry_config_.min_point_spacing_m = declare_parameter(
        "route_geometry.min_point_spacing_m", 0.05);
    route_geometry_config_.max_connection_distance_m = declare_parameter(
        "route_geometry.max_connection_distance_m", 3.0);
    route_geometry_config_.max_connection_heading_jump_rad = declare_parameter(
        "route_geometry.max_connection_heading_jump_rad", 1.2);
    route_geometry_config_.lane_change_min_length_m =
        cost.lane_change_min_length_m;
    route_geometry_config_.lane_change_min_lateral_m = declare_parameter(
        "route_geometry.lane_change_min_lateral_m", 1.0);
    route_geometry_config_.lane_change_max_lateral_m =
        cost.lane_change_max_lateral_m;
    route_geometry_config_.lane_change_max_heading_difference_rad =
        declare_parameter(
            "route_geometry.lane_change_max_heading_difference_rad", 0.35);
    route_validation_config_.max_adjacent_gap_m = declare_parameter(
        "route_validation.max_adjacent_gap_m", 3.0);
    route_validation_config_.min_point_spacing_m = declare_parameter(
        "route_validation.min_point_spacing_m", 0.05);
    route_validation_config_.max_heading_jump_rad = declare_parameter(
        "route_validation.max_heading_jump_rad", 1.2);
    route_validation_config_.max_reverse_progress_m = declare_parameter(
        "route_validation.max_reverse_progress_m", 0.5);
    route_validation_config_.maximum_duplicate_ratio = declare_parameter(
        "route_validation.maximum_duplicate_ratio", 0.1);
    route_validation_config_.minimum_route_length_m = declare_parameter(
        "route_validation.minimum_route_length_m", 0.1);
    route_validation_config_.min_route_points = static_cast<std::size_t>(
        declare_parameter("route_validation.min_route_points", 2));
    arrival_tolerance_m_ = declare_parameter("arrival_tolerance_m", 1.5);
    transform_timeout_s_ = declare_parameter("transform_timeout_s", 0.1);
    route_heartbeat_hz_ = declare_parameter("route_heartbeat_hz", 2.0);
    if (route_heartbeat_hz_ <= 0.0) {
      throw std::invalid_argument("route_heartbeat_hz must be positive");
    }
    planner_ = std::make_unique<GlobalPlannerCore>(cost);
    RCLCPP_INFO(get_logger(), "planner core ready");

    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    sub_map_ = create_subscription<adas_msgs::msg::LaneGraph>(
        "/adas/map/lane_graph", latched_qos,
        std::bind(&GlobalPlannerNode::on_map, this, std::placeholders::_1));
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/adas/localization/kinematic_state", rclcpp::SensorDataQoS(),
        std::bind(&GlobalPlannerNode::on_odom, this, std::placeholders::_1));
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/adas/navigation/goal_pose", rclcpp::QoS(1).reliable(),
        std::bind(&GlobalPlannerNode::on_goal, this, std::placeholders::_1));
    // Deprecated compatibility input for the existing Qt6 panel.  The canonical
    // contract is goal_pose; deployments can remove this after the UI is migrated.
    sub_goal_legacy_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/adas/navigation/goal", rclcpp::QoS(1).reliable(),
        std::bind(&GlobalPlannerNode::on_goal, this, std::placeholders::_1));
    sub_cancel_ = create_subscription<std_msgs::msg::Empty>(
        "/adas/navigation/cancel", rclcpp::QoS(1).reliable(),
        std::bind(&GlobalPlannerNode::on_cancel, this, std::placeholders::_1));
    srv_goal_ = create_service<adas_msgs::srv::SetNavigationGoal>(
        "/adas/navigation/set_goal",
        [this](const std::shared_ptr<adas_msgs::srv::SetNavigationGoal::Request> request,
               std::shared_ptr<adas_msgs::srv::SetNavigationGoal::Response> response) {
          response->request_id = request->request_id;
          if (request->request_id.empty() || request->goal.header.frame_id.empty() ||
              !std::isfinite(request->goal.pose.position.x) ||
              !std::isfinite(request->goal.pose.position.y)) {
            response->accepted = false;
            response->message = "invalid request_id, frame, or goal coordinates";
            return;
          }
          begin_new_route_version();
          goal_ = std::make_shared<geometry_msgs::msg::PoseStamped>(request->goal);
          goal_map_.reset();
          goal_id_ = request->request_id;
          active_route_ = GlobalRoute{};
          planning_pending_ = true;
          publish_route_state(adas_msgs::msg::GlobalRoute::STATUS_PLANNING);
          response->accepted = true;
          response->goal_id = goal_id_;
          response->message = "goal accepted";
          plan_route();
        });
    srv_cancel_ = create_service<adas_msgs::srv::CancelNavigation>(
        "/adas/navigation/cancel_goal",
        [this](const std::shared_ptr<adas_msgs::srv::CancelNavigation::Request> request,
               std::shared_ptr<adas_msgs::srv::CancelNavigation::Response> response) {
          response->request_id = request->request_id;
          response->goal_id = goal_id_;
          if (request->request_id.empty() ||
              (!request->goal_id.empty() && request->goal_id != goal_id_)) {
            response->accepted = false;
            response->message = "invalid request_id or goal_id mismatch";
            return;
          }
          on_cancel(std::make_shared<std_msgs::msg::Empty>());
          response->accepted = true;
          response->message = "cancel accepted";
        });
    RCLCPP_INFO(get_logger(), "navigation subscriptions ready");

    pub_route_ = create_publisher<adas_msgs::msg::GlobalRoute>(
        "/adas/navigation/global_route", latched_qos);
    pub_status_ = create_publisher<adas_msgs::msg::NavigationStatus>(
        "/adas/navigation/status", latched_qos);
    route_heartbeat_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / route_heartbeat_hz_), [this]() {
          if (!last_valid_route_) return;
          last_valid_route_->header.stamp = now();
          pub_route_->publish(*last_valid_route_);
        });
    RCLCPP_INFO(get_logger(), "navigation publishers ready");
    publish_route_state(adas_msgs::msg::GlobalRoute::STATUS_INVALID);
    publish_status(adas_msgs::msg::NavigationStatus::WAITING_FOR_MAP,
                   "waiting for lane graph");
  }

 private:
  void on_map(const adas_msgs::msg::LaneGraph::ConstSharedPtr msg) {
    auto graph = std::make_unique<LaneGraph>();
    std::size_t skipped_lanes = 0U;
    std::size_t skipped_connections = 0U;
    for (const auto& lane_msg : msg->lanes) {
      LaneSegment lane;
      lane.id = lane_msg.id;
      lane.speed_limit_mps = lane_msg.speed_limit_mps;
      lane.junction = lane_msg.junction;
      lane.centerline.reserve(lane_msg.centerline.size());
      for (const auto& pose : lane_msg.centerline) {
        lane.centerline.push_back({pose.position.x, pose.position.y, yaw_from_pose(pose)});
      }
      if (!graph->add_lane(lane)) ++skipped_lanes;
    }
    for (const auto& lane_msg : msg->lanes) {
      for (const auto& edge : lane_msg.outgoing) {
        if (!graph->add_connection({lane_msg.id, edge.to_lane_id,
                                    maneuver_from_msg(edge.maneuver), edge.extra_cost_m})) {
          ++skipped_connections;
        }
      }
    }

    const std::string frame = msg->header.frame_id;
    if (msg->map_id.empty() || msg->map_hash.empty() || frame.empty() ||
        graph->lane_count() == 0U) {
      graph_.reset();
      fail_route("lane graph identity, frame, or topology is invalid");
      return;
    }
    if (skipped_lanes != 0U || skipped_connections != 0U) {
      RCLCPP_WARN(get_logger(),
                  "map %s: skipped %zu invalid lane(s) and %zu invalid connection(s)",
                  msg->map_id.c_str(), skipped_lanes, skipped_connections);
    }

    const bool map_changed = map_hash_ != msg->map_hash;
    graph_ = std::move(graph);
    map_id_ = msg->map_id;
    map_hash_ = msg->map_hash;
    map_frame_ = frame;
    RCLCPP_INFO(get_logger(), "loaded map %s with %zu lanes", map_id_.c_str(),
                graph_->lane_count());
    publish_status(adas_msgs::msg::NavigationStatus::IDLE, "map ready");
    if (goal_) {
      if (map_changed && active_route_.valid) begin_new_route_version();
      planning_pending_ = true;
      publish_route_state(adas_msgs::msg::GlobalRoute::STATUS_PLANNING);
      plan_route();
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    odom_ = msg;
    if (planning_pending_ && goal_) {
      plan_route();
      return;
    }
    if (!goal_map_ || !active_route_.valid) return;
    geometry_msgs::msg::PoseStamped start;
    std::string error;
    if (!current_pose_in_map(start, error)) {
      fail_route(error);
      return;
    }
    const double distance = std::hypot(
        goal_map_->pose.position.x - start.pose.position.x,
        goal_map_->pose.position.y - start.pose.position.y);
    remaining_distance_m_ = distance;
    if (distance <= arrival_tolerance_m_ && std::abs(msg->twist.twist.linear.x) < 0.2) {
      publish_status(adas_msgs::msg::NavigationStatus::ARRIVED, "goal reached");
    }
  }

  void on_goal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    begin_new_route_version();
    goal_ = msg;
    goal_map_.reset();
    goal_id_ = std::to_string(route_sequence_);
    active_route_ = GlobalRoute{};
    planning_pending_ = true;
    publish_route_state(adas_msgs::msg::GlobalRoute::STATUS_PLANNING);
    plan_route();
  }

  void on_cancel(const std_msgs::msg::Empty::ConstSharedPtr) {
    begin_new_route_version();
    goal_.reset();
    goal_map_.reset();
    active_route_ = GlobalRoute{};
    planning_pending_ = false;
    remaining_distance_m_ = 0.0;
    publish_route_state(adas_msgs::msg::GlobalRoute::STATUS_CANCELLED);
    publish_status(adas_msgs::msg::NavigationStatus::CANCELED,
                   "navigation canceled; safe stop requested");
  }

  void begin_new_route_version() {
    if (route_sequence_ == std::numeric_limits<std::uint32_t>::max()) {
      route_sequence_ = 1U;
    } else {
      ++route_sequence_;
      if (route_sequence_ == 0U) route_sequence_ = 1U;
    }
    active_route_id_ = std::to_string(route_sequence_);
  }

  bool transform_to_map(const geometry_msgs::msg::PoseStamped& input,
                        geometry_msgs::msg::PoseStamped& output,
                        std::string& error) {
    if (input.header.frame_id.empty()) {
      error = "pose frame_id is empty";
      return false;
    }
    if (input.header.frame_id == map_frame_) {
      output = input;
      return true;
    }
    try {
      output = tf_buffer_->transform(
          input, map_frame_, tf2::durationFromSec(transform_timeout_s_));
      return true;
    } catch (const tf2::TransformException& exception) {
      error = "cannot transform " + input.header.frame_id + " to " + map_frame_ +
              ": " + exception.what();
      return false;
    }
  }

  bool current_pose_in_map(geometry_msgs::msg::PoseStamped& output,
                           std::string& error) {
    if (!odom_) {
      error = "localization is unavailable";
      return false;
    }
    geometry_msgs::msg::PoseStamped input;
    input.header = odom_->header;
    input.pose = odom_->pose.pose;
    return transform_to_map(input, output, error);
  }

  void plan_route() {
    if (!goal_) return;
    if (!graph_) {
      publish_status(adas_msgs::msg::NavigationStatus::WAITING_FOR_MAP,
                     "goal stored; waiting for map");
      return;
    }
    if (!odom_) {
      publish_status(adas_msgs::msg::NavigationStatus::PLANNING,
                     "goal stored; waiting for localization");
      return;
    }

    geometry_msgs::msg::PoseStamped start_map;
    geometry_msgs::msg::PoseStamped transformed_goal;
    std::string error;
    if (!current_pose_in_map(start_map, error) ||
        !transform_to_map(*goal_, transformed_goal, error)) {
      fail_route(error);
      return;
    }
    goal_map_ = std::make_shared<geometry_msgs::msg::PoseStamped>(transformed_goal);
    planning_pending_ = false;
    publish_status(adas_msgs::msg::NavigationStatus::PLANNING, "planning route");
    active_route_ = planner_->plan(
        *graph_, start_map.pose.position.x, start_map.pose.position.y,
        transformed_goal.pose.position.x, transformed_goal.pose.position.y);
    if (!active_route_.valid) {
      fail_route(active_route_.failure_reason);
      return;
    }

    const auto semantic = build_semantic_route(
        active_route_, start_map.pose.position.x, start_map.pose.position.y,
        transformed_goal.pose.position.x, transformed_goal.pose.position.y,
        route_geometry_config_);
    if (!semantic.valid) {
      active_route_ = GlobalRoute{};
      fail_route(semantic.failure_reason);
      return;
    }

    const auto validation = validate_route(
        semantic.points, route_validation_config_);
    if (!validation.valid) {
      RCLCPP_ERROR(
          get_logger(),
          "route validation failed: %s; index=%zu max_gap=%.3f "
          "max_heading_jump=%.3f max_reverse=%.3f length=%.3f",
          validation.reason.c_str(), validation.offending_index,
          validation.maximum_adjacent_gap_m,
          validation.maximum_heading_jump_rad,
          validation.maximum_reverse_progress_m, validation.route_length_m);
      active_route_ = GlobalRoute{};
      fail_route("route validation failed: " + validation.reason +
                 "; offending_index=" +
                 std::to_string(validation.offending_index) +
                 "; maximum_gap_m=" +
                 std::to_string(validation.maximum_adjacent_gap_m));
      return;
    }

    adas_msgs::msg::GlobalRoute message = make_route_message(
        adas_msgs::msg::GlobalRoute::STATUS_VALID);
    message.length = static_cast<float>(semantic.length_m);
    message.points.reserve(semantic.points.size());
    for (const auto& point : semantic.points) {
      adas_msgs::msg::RoutePoint converted;
      converted.x = point.pose.x;
      converted.y = point.pose.y;
      converted.yaw = point.pose.yaw;
      converted.lane_id = point.lane_id;
      converted.road_id = point.road_id;
      converted.speed_limit = static_cast<float>(point.speed_limit_mps);
      converted.maneuver = maneuver_to_msg(point);
      message.points.push_back(converted);
    }
    remaining_distance_m_ = semantic.length_m;
    pub_route_->publish(message);
    last_valid_route_ = message;
    publish_status(adas_msgs::msg::NavigationStatus::DRIVING, "route ready");
  }

  adas_msgs::msg::GlobalRoute make_route_message(std::uint8_t status) {
    adas_msgs::msg::GlobalRoute message;
    message.header.stamp = now();
    message.header.frame_id = map_frame_;
    message.route_id = route_sequence_;
    message.frame_id = map_frame_;
    message.map_id = map_id_;
    message.map_hash = map_hash_;
    message.goal_id = goal_id_;
    message.length = 0.0F;
    message.status = status;
    return message;
  }

  void publish_route_state(std::uint8_t status) {
    if (status != adas_msgs::msg::GlobalRoute::STATUS_VALID) last_valid_route_.reset();
    if (pub_route_) pub_route_->publish(make_route_message(status));
  }

  void fail_route(const std::string& detail) {
    planning_pending_ = false;
    active_route_ = GlobalRoute{};
    remaining_distance_m_ = 0.0;
    publish_route_state(adas_msgs::msg::GlobalRoute::STATUS_FAILED);
    publish_status(adas_msgs::msg::NavigationStatus::FAILED, detail);
  }

  void publish_status(std::uint8_t state, const std::string& detail) {
    adas_msgs::msg::NavigationStatus status;
    status.header.stamp = now();
    status.header.frame_id = map_frame_;
    status.state = state;
    status.map_id = map_id_;
    status.goal_id = goal_id_;
    status.route_id = active_route_id_;
    status.detail = detail;
    status.remaining_distance_m = static_cast<float>(remaining_distance_m_);
    if (pub_status_) pub_status_->publish(status);
  }

  double arrival_tolerance_m_{1.5};
  double transform_timeout_s_{0.1};
  double route_heartbeat_hz_{2.0};
  double remaining_distance_m_{0.0};
  RouteGeometryConfig route_geometry_config_;
  RouteValidationConfig route_validation_config_;
  bool planning_pending_{false};
  std::uint32_t route_sequence_{0U};
  std::string map_id_;
  std::string map_hash_;
  std::string map_frame_{"map"};
  std::string goal_id_;
  std::string active_route_id_;
  std::unique_ptr<LaneGraph> graph_;
  std::unique_ptr<GlobalPlannerCore> planner_;
  GlobalRoute active_route_;
  std::optional<adas_msgs::msg::GlobalRoute> last_valid_route_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_map_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<adas_msgs::msg::LaneGraph>::SharedPtr sub_map_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_legacy_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_cancel_;
  rclcpp::Service<adas_msgs::srv::SetNavigationGoal>::SharedPtr srv_goal_;
  rclcpp::Service<adas_msgs::srv::CancelNavigation>::SharedPtr srv_cancel_;
  rclcpp::Publisher<adas_msgs::msg::GlobalRoute>::SharedPtr pub_route_;
  rclcpp::Publisher<adas_msgs::msg::NavigationStatus>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr route_heartbeat_timer_;
};

}  // namespace adas::planning

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto planner = std::make_shared<adas::planning::GlobalPlannerNode>();
  auto adapter = std::make_shared<adas::planning::RouteAdapterNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  // Keep owning shared pointers alive: executors retain weak node references.
  executor.add_node(planner);
  executor.add_node(adapter);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
