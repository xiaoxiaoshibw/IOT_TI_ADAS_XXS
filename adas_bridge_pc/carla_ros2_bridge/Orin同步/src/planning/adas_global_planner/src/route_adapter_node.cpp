#include <cmath>
#include <memory>

#include "adas_global_planner/route_adapter.hpp"

namespace adas::planning {

RouteAdapterNode::RouteAdapterNode() : Node("route_adapter") {
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/adas/planning/global_route", qos);
  subscription_ = create_subscription<adas_msgs::msg::GlobalRoute>(
      "/adas/navigation/global_route", qos,
      [this](adas_msgs::msg::GlobalRoute::ConstSharedPtr message) {
        adapt_and_publish(*message);
      });
}

void RouteAdapterNode::adapt_and_publish(const adas_msgs::msg::GlobalRoute& route) {
  if (published_once_ && route.route_id == last_route_id_ &&
      route.status == last_status_) {
    return;  // GlobalRoute heartbeat only refreshes TTL; legacy Path stays latched.
  }
  published_once_ = true;
  last_route_id_ = route.route_id;
  last_status_ = route.status;
  nav_msgs::msg::Path path;
  path.header = route.header;
  const bool valid = route.status == adas_msgs::msg::GlobalRoute::STATUS_VALID &&
                     global_route_frame_valid(route.header.frame_id, route.frame_id) &&
                     route.points.size() >= 2U;
  if (valid) {
    path.poses.reserve(route.points.size());
    for (const auto& point : route.points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.orientation.z = std::sin(point.yaw * 0.5);
      pose.pose.orientation.w = std::cos(point.yaw * 0.5);
      path.poses.push_back(std::move(pose));
    }
  }
  publisher_->publish(path);
}

}  // namespace adas::planning
