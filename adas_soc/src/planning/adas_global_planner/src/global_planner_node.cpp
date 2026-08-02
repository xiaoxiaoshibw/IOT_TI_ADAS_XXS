#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "adas_global_planner/global_planner_core.hpp"
#include "adas_msgs/msg/lane_graph.hpp"
#include "adas_msgs/msg/navigation_status.hpp"
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

    pub_route_ = create_publisher<nav_msgs::msg::Path>(
        "/adas/planning/global_route", rclcpp::QoS(1).reliable().transient_local());
    pub_status_ = create_publisher<adas_msgs::msg::NavigationStatus>(
        "/adas/navigation/status", rclcpp::QoS(1).reliable().transient_local());
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
    if (!goal_ || !active_route_.valid) return;
    const double distance = std::hypot(
        goal_->pose.position.x - msg->pose.pose.position.x,
        goal_->pose.position.y - msg->pose.pose.position.y);
    remaining_distance_m_ = distance;
    if (distance <= arrival_tolerance_m_ && std::abs(msg->twist.twist.linear.x) < 0.2) {
      publish_status(adas_msgs::msg::NavigationStatus::ARRIVED, "goal reached");
    }
  }

  void on_goal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    goal_ = msg;
    goal_id_ = std::to_string(now().nanoseconds());
    active_route_ = GlobalRoute{};
    plan_route();
  }

  void on_cancel(const std_msgs::msg::Empty::ConstSharedPtr) {
    goal_.reset();
    active_route_ = GlobalRoute{};
    nav_msgs::msg::Path empty;
    empty.header.stamp = now();
    empty.header.frame_id = "map";
    pub_route_->publish(empty);
    publish_status(adas_msgs::msg::NavigationStatus::CANCELED, "navigation canceled");
  }

  void plan_route() {
    if (!graph_) {
      publish_status(adas_msgs::msg::NavigationStatus::WAITING_FOR_MAP, "goal stored; waiting for map");
      return;
    }
    if (!odom_) {
      publish_status(adas_msgs::msg::NavigationStatus::PLANNING, "goal stored; waiting for localization");
      return;
    }
    publish_status(adas_msgs::msg::NavigationStatus::PLANNING, "planning route");
    active_route_ = planner_->plan(
        *graph_, odom_->pose.pose.position.x, odom_->pose.pose.position.y,
        goal_->pose.position.x, goal_->pose.position.y);
    if (!active_route_.valid) {
      publish_status(adas_msgs::msg::NavigationStatus::FAILED, active_route_.failure_reason);
      return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "map";
    for (const auto& segment : active_route_.segments) {
      for (std::size_t i = 0; i < segment.centerline.size(); ++i) {
        if (!path.poses.empty() && i == 0U) continue;
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = segment.centerline[i].x;
        pose.pose.position.y = segment.centerline[i].y;
        pose.pose.orientation.z = std::sin(segment.centerline[i].yaw * 0.5);
        pose.pose.orientation.w = std::cos(segment.centerline[i].yaw * 0.5);
        path.poses.push_back(std::move(pose));
      }
    }
    active_route_id_ = route_id(map_hash_, path.header.stamp);
    remaining_distance_m_ = active_route_.total_cost_m;
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
  double remaining_distance_m_{0.0};
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
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_route_;
  rclcpp::Publisher<adas_msgs::msg::NavigationStatus>::SharedPtr pub_status_;
};

}  // namespace adas::planning

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<adas::planning::GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
