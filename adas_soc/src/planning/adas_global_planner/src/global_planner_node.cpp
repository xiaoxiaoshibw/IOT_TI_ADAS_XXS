#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "adas_global_planner/global_planner_core.hpp"
#include "adas_global_planner/route_validator.hpp"
#include "adas_global_planner/semantic_route.hpp"
#include "adas_msgs/msg/lane_graph.hpp"
#include "adas_msgs/msg/navigation_status.hpp"
#include "adas_msgs/srv/cancel_navigation.hpp"
#include "adas_msgs/srv/set_navigation_goal.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

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

double yaw_from_pose(const geometry_msgs::msg::Pose& pose) {
  return 2.0 * std::atan2(pose.orientation.z, pose.orientation.w);
}

std::string route_id(const std::string& map_hash, const rclcpp::Time& stamp) {
  std::ostringstream value;
  value << map_hash.substr(0, 8) << '-' << stamp.nanoseconds();
  return value.str();
}

double point_segment_distance(double px, double py, const MapPoint& a, const MapPoint& b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length2 = dx * dx + dy * dy;
  if (length2 <= 1e-12) return std::hypot(px - a.x, py - a.y);
  const double t = std::clamp(((px - a.x) * dx + (py - a.y) * dy) / length2, 0.0, 1.0);
  return std::hypot(px - (a.x + t * dx), py - (a.y + t * dy));
}

}  // namespace

class GlobalPlannerNode : public rclcpp::Node {
 public:
  GlobalPlannerNode() : Node("global_planner") {
    PlannerCost cost;
    cost.lane_change_penalty_m = declare_parameter("lane_change_penalty_m", 8.0);
    cost.junction_penalty_m = declare_parameter("junction_penalty_m", 2.0);
    cost.turn_penalty_m = declare_parameter("turn_penalty_m", 1.0);
    cost.snap_max_distance_m = declare_parameter("snap_max_distance_m", 8.0);
    arrival_tolerance_m_ = declare_parameter("arrival_tolerance_m", 1.5);
    lane_width_m_ = declare_parameter("lane_width_m", 3.5);
    replan_threshold_m_ =
        declare_parameter("replan_threshold_m", 1.5 * lane_width_m_);
    deviation_check_rate_hz_ = declare_parameter("deviation_check_rate_hz", 5.0);
    replan_cooldown_s_ = declare_parameter("replan_cooldown_s", 10.0);
    if (!std::isfinite(lane_width_m_) || lane_width_m_ <= 0.0 ||
        !std::isfinite(replan_threshold_m_) || replan_threshold_m_ <= 0.0 ||
        !std::isfinite(deviation_check_rate_hz_) || deviation_check_rate_hz_ <= 0.0 ||
        !std::isfinite(replan_cooldown_s_) || replan_cooldown_s_ < 0.0) {
        throw std::invalid_argument("invalid global planner replan parameters");
    }
    route_geometry_.sample_spacing_m =
        declare_parameter("route_geometry.sample_spacing_m", 2.0);
    route_geometry_.min_point_spacing_m =
        declare_parameter("route_geometry.min_point_spacing_m", 0.05);
    route_geometry_.max_connection_distance_m =
        declare_parameter("route_geometry.max_connection_distance_m", 3.0);
    route_geometry_.max_connection_heading_jump_rad =
        declare_parameter("route_geometry.max_connection_heading_jump_rad", 1.2);
    route_geometry_.lane_change_min_length_m =
        declare_parameter("route_geometry.lane_change_min_length_m", 10.0);
    route_geometry_.lane_change_min_lateral_m =
        declare_parameter("route_geometry.lane_change_min_lateral_m", 1.0);
    route_geometry_.lane_change_max_lateral_m =
        declare_parameter("route_geometry.lane_change_max_lateral_m", 6.0);
    route_geometry_.lane_change_max_heading_difference_rad =
        declare_parameter("route_geometry.lane_change_max_heading_difference_rad", 0.35);
    route_validation_.max_adjacent_gap_m =
        declare_parameter("route_validation.max_adjacent_gap_m", 3.0);
    route_validation_.min_point_spacing_m =
        declare_parameter("route_validation.min_point_spacing_m", 0.05);
    route_validation_.max_heading_jump_rad =
        declare_parameter("route_validation.max_heading_jump_rad", 1.2);
    route_validation_.max_reverse_progress_m =
        declare_parameter("route_validation.max_reverse_progress_m", 0.5);
    route_validation_.maximum_duplicate_ratio =
        declare_parameter("route_validation.maximum_duplicate_ratio", 0.1);
    route_validation_.minimum_route_length_m =
        declare_parameter("route_validation.minimum_route_length_m", 0.1);
    const auto min_route_points =
        declare_parameter<int>("route_validation.min_route_points", 2);
    if (!std::isfinite(route_geometry_.sample_spacing_m) ||
        route_geometry_.sample_spacing_m <= 0.0 ||
        !std::isfinite(route_geometry_.min_point_spacing_m) ||
        route_geometry_.min_point_spacing_m <= 0.0 ||
        !std::isfinite(route_geometry_.max_connection_distance_m) ||
        route_geometry_.max_connection_distance_m <= 0.0 ||
        !std::isfinite(route_geometry_.max_connection_heading_jump_rad) ||
        route_geometry_.max_connection_heading_jump_rad <= 0.0 ||
        !std::isfinite(route_geometry_.lane_change_min_length_m) ||
        route_geometry_.lane_change_min_length_m <= 0.0 ||
        !std::isfinite(route_geometry_.lane_change_min_lateral_m) ||
        route_geometry_.lane_change_min_lateral_m < 0.0 ||
        !std::isfinite(route_geometry_.lane_change_max_lateral_m) ||
        route_geometry_.lane_change_max_lateral_m <
            route_geometry_.lane_change_min_lateral_m ||
        !std::isfinite(route_geometry_.lane_change_max_heading_difference_rad) ||
        route_geometry_.lane_change_max_heading_difference_rad <= 0.0 ||
        !std::isfinite(route_validation_.max_adjacent_gap_m) ||
        route_validation_.max_adjacent_gap_m <= 0.0 ||
        !std::isfinite(route_validation_.min_point_spacing_m) ||
        route_validation_.min_point_spacing_m <= 0.0 ||
        !std::isfinite(route_validation_.max_heading_jump_rad) ||
        route_validation_.max_heading_jump_rad <= 0.0 ||
        !std::isfinite(route_validation_.max_reverse_progress_m) ||
        route_validation_.max_reverse_progress_m < 0.0 ||
        !std::isfinite(route_validation_.maximum_duplicate_ratio) ||
        route_validation_.maximum_duplicate_ratio < 0.0 ||
        route_validation_.maximum_duplicate_ratio > 1.0 ||
        !std::isfinite(route_validation_.minimum_route_length_m) ||
        route_validation_.minimum_route_length_m <= 0.0 || min_route_points < 2) {
      throw std::invalid_argument("invalid route geometry/validation parameters");
    }
    route_validation_.min_route_points = static_cast<std::size_t>(min_route_points);
    replan_policy_ = std::make_unique<ReplanPolicy>(replan_threshold_m_, replan_cooldown_s_);
    planner_ = std::make_unique<GlobalPlannerCore>(cost);

    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    sub_map_ = create_subscription<adas_msgs::msg::LaneGraph>(
        "/adas/map/lane_graph", map_qos,
        std::bind(&GlobalPlannerNode::on_map, this, std::placeholders::_1));
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/adas/localization/kinematic_state", rclcpp::SensorDataQoS(),
        std::bind(&GlobalPlannerNode::on_odom, this, std::placeholders::_1));
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
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
          goal_ = std::make_shared<geometry_msgs::msg::PoseStamped>(request->goal);
          goal_id_ = request->request_id;
          clear_active_route();
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
          goal_.reset();
          clear_active_route();
          publish_status(adas_msgs::msg::NavigationStatus::CANCELED,
                         "navigation canceled");
          response->accepted = true;
          response->message = "cancel accepted";
        });

    pub_route_ = create_publisher<nav_msgs::msg::Path>(
        "/adas/planning/global_route", rclcpp::QoS(1).reliable().transient_local());
    pub_status_ = create_publisher<adas_msgs::msg::NavigationStatus>(
        "/adas/navigation/status", rclcpp::QoS(1).reliable().transient_local());
    deviation_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / deviation_check_rate_hz_),
        std::bind(&GlobalPlannerNode::check_route_deviation, this));
    publish_status(adas_msgs::msg::NavigationStatus::WAITING_FOR_MAP, "waiting for lane graph");
  }

 private:
  void on_map(const adas_msgs::msg::LaneGraph::ConstSharedPtr msg) {
    auto graph = std::make_unique<LaneGraph>();
    std::size_t skipped_lanes = 0;
    std::size_t skipped_connections = 0;
    // 单条坏车道/坏连接（真实地图里会有退化件：单点路口连接线、自环等）不该
    // 拖垮整张地图——逐条容错，只累计跳过数，M6.0 验收要"全部可行驶车道可遍历"。
    for (const auto& lane_msg : msg->lanes) {
      LaneSegment lane;
      lane.id = lane_msg.id;
      lane.speed_limit_mps = lane_msg.speed_limit_mps;
      lane.junction = lane_msg.junction;
      lane.centerline.reserve(lane_msg.centerline.size());
      for (const auto& pose : lane_msg.centerline) {
        lane.centerline.push_back(
            {pose.position.x, pose.position.y, yaw_from_pose(pose)});
      }
      if (!graph->add_lane(lane)) ++skipped_lanes;
    }
    for (const auto& lane_msg : msg->lanes) {
      for (const auto& edge : lane_msg.outgoing) {
        if (!graph->add_connection(
                {lane_msg.id, edge.to_lane_id, maneuver_from_msg(edge.maneuver),
                 edge.extra_cost_m})) {
          ++skipped_connections;
        }
      }
    }
    if (msg->map_id.empty() || graph->lane_count() == 0U) {
      publish_status(adas_msgs::msg::NavigationStatus::FAILED,
                     "lane graph is empty or has no valid lanes");
      return;
    }
    if (skipped_lanes != 0U || skipped_connections != 0U) {
      RCLCPP_WARN(get_logger(),
                  "map %s: skipped %zu invalid lane(s) and %zu invalid connection(s)",
                  msg->map_id.c_str(), skipped_lanes, skipped_connections);
    }
    graph_ = std::move(graph);
    map_id_ = msg->map_id;
    map_hash_ = msg->map_hash;
    RCLCPP_INFO(get_logger(), "loaded map %s with %zu lanes", map_id_.c_str(),
                graph_->lane_count());
    publish_status(adas_msgs::msg::NavigationStatus::IDLE, "map ready");
    if (goal_) plan_route();
  }

  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    odom_ = msg;
    if (plan_when_localized_) {
      plan_when_localized_ = false;
      plan_route();
    }
    if (!goal_ || !active_route_.valid || !arrival_point_valid_ || arrived_) return;
    const double distance = std::hypot(
        arrival_point_.x - msg->pose.pose.position.x,
        arrival_point_.y - msg->pose.pose.position.y);
    remaining_distance_m_ = distance;
    if (distance <= arrival_tolerance_m_ && std::abs(msg->twist.twist.linear.x) < 0.2) {
      arrived_ = true;
      remaining_distance_m_ = 0.0;
      publish_status(adas_msgs::msg::NavigationStatus::ARRIVED,
                     "route endpoint reached");
    }
  }

  void on_goal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    goal_ = msg;
    goal_id_ = std::to_string(now().nanoseconds());
    clear_active_route();
    plan_route();
  }

  void on_cancel(const std_msgs::msg::Empty::ConstSharedPtr) {
    goal_.reset();
    clear_active_route();
    publish_status(adas_msgs::msg::NavigationStatus::CANCELED, "navigation canceled");
  }

  void clear_active_route() {
    active_route_ = GlobalRoute{};
    arrival_point_ = MapPoint{};
    arrival_point_valid_ = false;
    arrived_ = false;
    plan_when_localized_ = false;
    active_route_id_.clear();
    remaining_distance_m_ = 0.0;
    if (replan_policy_) replan_policy_->reset();
    nav_msgs::msg::Path empty;
    empty.header.stamp = now();
    empty.header.frame_id = "map";
    pub_route_->publish(empty);
  }

  void check_route_deviation() {
    if (!goal_ || !graph_ || !active_route_.valid || !odom_ || arrived_) return;

    const double x = odom_->pose.pose.position.x;
    const double y = odom_->pose.pose.position.y;
    double deviation = std::numeric_limits<double>::infinity();
    for (const auto& segment : active_route_.segments) {
      for (std::size_t i = 1; i < segment.centerline.size(); ++i) {
        deviation = std::min(
            deviation, point_segment_distance(x, y, segment.centerline[i - 1],
                                              segment.centerline[i]));
      }
    }
    if (!std::isfinite(deviation) || deviation <= replan_threshold_m_) return;

    const auto now = std::chrono::steady_clock::now();
    const double now_s = std::chrono::duration<double>(now.time_since_epoch()).count();
    if (!replan_policy_ || !replan_policy_->request(deviation, now_s)) return;
    RCLCPP_WARN(get_logger(), "route deviation %.2fm exceeds %.2fm; replanning",
                deviation, replan_threshold_m_);
    plan_route(true);
  }

  void plan_route(bool is_replan = false) {
    if (!graph_) {
      publish_status(adas_msgs::msg::NavigationStatus::WAITING_FOR_MAP, "goal stored; waiting for map");
      return;
    }
    if (!odom_) {
      plan_when_localized_ = true;
      publish_status(adas_msgs::msg::NavigationStatus::PLANNING, "goal stored; waiting for localization");
      return;
    }
    plan_when_localized_ = false;
    publish_status(adas_msgs::msg::NavigationStatus::PLANNING, "planning route");
    const GlobalRoute previous_route = active_route_;
    GlobalRoute candidate;
    const Pose current{odom_->pose.pose.position.x, odom_->pose.pose.position.y,
                       yaw_from_pose(odom_->pose.pose)};
    const Pose target{goal_->pose.position.x, goal_->pose.position.y,
                      yaw_from_pose(goal_->pose)};
    const bool planned = is_replan
                             ? planner_->replan(*graph_, current, target, candidate)
                             : (candidate = planner_->plan(*graph_, current.x, current.y,
                                                           target.x, target.y),
                                candidate.valid);
    if (!planned) {
      if (is_replan && previous_route.valid) {
        active_route_ = previous_route;
        RCLCPP_WARN(get_logger(), "replan failed (%s); retaining previous route",
                    candidate.failure_reason.c_str());
        publish_status(adas_msgs::msg::NavigationStatus::DRIVING,
                       "replan failed; retaining previous route");
        return;
      }
      active_route_ = candidate;
      publish_status(adas_msgs::msg::NavigationStatus::FAILED, active_route_.failure_reason);
      return;
    }

    // 不直接把各车道中心线首尾拼接。连接车道在路口/变道处通常有重叠或
    // 横向偏移，直接拼接会给跟踪器制造几米的几何跳变，车辆会在真正进入
    // 弯道前突然打满方向。先裁剪起终点、按前向投影连接 successor，再做
    // 路线完整性校验；校验失败时保持安全停车，不发布一条可能撞墙的路线。
    const auto semantic = build_semantic_route(
        candidate, current.x, current.y, target.x, target.y, route_geometry_);
    if (!semantic.valid) {
      if (is_replan && previous_route.valid) {
        active_route_ = previous_route;
        RCLCPP_WARN(get_logger(), "replanned route geometry invalid (%s); retaining previous route",
                    semantic.failure_reason.c_str());
        publish_status(adas_msgs::msg::NavigationStatus::DRIVING,
                       "replan geometry invalid; retaining previous route");
        return;
      }
      active_route_ = candidate;
      publish_status(adas_msgs::msg::NavigationStatus::FAILED,
                     "route geometry invalid: " + semantic.failure_reason);
      return;
    }
    const auto validation = validate_route(semantic.points, route_validation_);
    if (!validation.valid) {
      if (is_replan && previous_route.valid) {
        active_route_ = previous_route;
        RCLCPP_WARN(get_logger(), "replanned route validation failed (%s); retaining previous route",
                    validation.reason.c_str());
        publish_status(adas_msgs::msg::NavigationStatus::DRIVING,
                       "replan validation failed; retaining previous route");
        return;
      }
      active_route_ = candidate;
      publish_status(adas_msgs::msg::NavigationStatus::FAILED,
                     "route validation failed: " + validation.reason);
      return;
    }
    active_route_ = std::move(candidate);

    // The clicked goal may be off the lane centerline. The route endpoint is
    // the safe projection onto the final lane, so stopping and ARRIVED must
    // use this same canonical point instead of the raw click position.
    arrival_point_ = semantic.points.back().pose;
    arrival_point_valid_ = true;
    arrived_ = false;

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "map";
    path.poses.reserve(semantic.points.size());
    for (const auto& point : semantic.points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.pose.x;
      pose.pose.position.y = point.pose.y;
      pose.pose.orientation.z = std::sin(point.pose.yaw * 0.5);
      pose.pose.orientation.w = std::cos(point.pose.yaw * 0.5);
      path.poses.push_back(std::move(pose));
    }
    active_route_id_ = route_id(map_hash_, path.header.stamp);
    remaining_distance_m_ = semantic.length_m;
    pub_route_->publish(path);
    publish_status(adas_msgs::msg::NavigationStatus::DRIVING, "route ready");
  }

  void publish_status(std::uint8_t state, const std::string& detail) {
    adas_msgs::msg::NavigationStatus status;
    status.header.stamp = now();
    status.header.frame_id = "map";
    status.state = state;
    status.map_id = map_id_;
    status.goal_id = goal_id_;
    status.route_id = active_route_id_;
    status.detail = detail;
    status.remaining_distance_m = static_cast<float>(remaining_distance_m_);
    if (pub_status_) pub_status_->publish(status);
  }

  double arrival_tolerance_m_{1.5};
  double lane_width_m_{3.5};
  double replan_threshold_m_{5.25};
  double deviation_check_rate_hz_{5.0};
  double replan_cooldown_s_{10.0};
  double remaining_distance_m_{0.0};
  MapPoint arrival_point_{};
  bool arrival_point_valid_{false};
  bool arrived_{false};
  bool plan_when_localized_{false};
  std::unique_ptr<ReplanPolicy> replan_policy_;
  RouteGeometryConfig route_geometry_;
  RouteValidationConfig route_validation_;
  std::string map_id_;
  std::string map_hash_;
  std::string goal_id_;
  std::string active_route_id_;
  std::unique_ptr<LaneGraph> graph_;
  std::unique_ptr<GlobalPlannerCore> planner_;
  GlobalRoute active_route_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_;
  rclcpp::Subscription<adas_msgs::msg::LaneGraph>::SharedPtr sub_map_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_cancel_;
  rclcpp::Service<adas_msgs::srv::SetNavigationGoal>::SharedPtr srv_goal_;
  rclcpp::Service<adas_msgs::srv::CancelNavigation>::SharedPtr srv_cancel_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_route_;
  rclcpp::Publisher<adas_msgs::msg::NavigationStatus>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr deviation_timer_;
};

}  // namespace adas::planning

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<adas::planning::GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
