#include <cmath>
#include <memory>
#include <string>

#include "adas_global_planner/route_adapter.hpp"

namespace adas::planning {

namespace {

nav_msgs::msg::Path make_path(const adas_msgs::msg::GlobalRoute& route,
                              const std::string& frame_id) {
  nav_msgs::msg::Path path;
  path.header = route.header;
  if (path.header.frame_id.empty()) {
    path.header.frame_id = frame_id;
  }
  // VALID 路径走完整 poses；clear 状态保留 frame/stamp 但 poses 为空。
  if (route.status == adas_msgs::msg::GlobalRoute::STATUS_VALID &&
      route.points.size() >= 2U) {
    path.poses.reserve(route.points.size());
    for (const auto& point : route.points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = static_cast<double>(point.x);
      pose.pose.position.y = static_cast<double>(point.y);
      pose.pose.orientation.z = std::sin(static_cast<double>(point.yaw) * 0.5);
      pose.pose.orientation.w = std::cos(static_cast<double>(point.yaw) * 0.5);
      path.poses.push_back(std::move(pose));
    }
  }
  return path;
}

}  // namespace

RouteAdapterNode::RouteAdapterNode() : Node("route_adapter") {
  // P0.C: 适配器必须有显式会话 ID；split-topology 不在 launch 中传
  // --run-id 时直接 fail-closed。声明为带 default 的字符串参数,launch
  // 可覆盖,但 build 阶段不强制有值（adapter 不该静默生成 UUID,
  // 该责任属于 GUI/Orchestrator/launch 上层）。
  bound_run_id_ = declare_parameter<std::string>("run_id", std::string());
  frame_id_ = declare_parameter<std::string>("frame_id", std::string("map"));
  if (frame_id_.empty()) frame_id_ = "map";

  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/adas/planning/global_route", qos);
  subscription_ = create_subscription<adas_msgs::msg::GlobalRoute>(
      "/adas/navigation/global_route", qos,
      [this](adas_msgs::msg::GlobalRoute::ConstSharedPtr message) {
        adapt_and_publish(*message);
      });

  // 启动时立即发布一次空 Path,把 transient_local 旧会话路线清掉。
  publish_cleared_path();
  startup_clear_published_ = true;
}

void RouteAdapterNode::publish_cleared_path() {
  nav_msgs::msg::Path empty;
  empty.header.stamp = now();
  empty.header.frame_id = frame_id_;
  publisher_->publish(empty);
}

void RouteAdapterNode::adapt_and_publish(
    const adas_msgs::msg::GlobalRoute& route) {
  const auto decision = evaluate_route_for_adapter(bound_run_id_, route);
  if (!decision.accept) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "route_adapter 拒绝 GlobalRoute：%s",
                         decision.reason.c_str());
    return;
  }
  // P0.3: 重复心跳（同 route_id + status）不重复发布。
  if (published_once_ && route.route_id == last_route_id_ &&
      route.status == last_status_) {
    return;
  }
  const auto path = make_path(route, frame_id_);
  publisher_->publish(path);
  published_once_ = true;
  last_route_id_ = route.route_id;
  last_status_ = route.status;
}

}  // namespace adas::planning
