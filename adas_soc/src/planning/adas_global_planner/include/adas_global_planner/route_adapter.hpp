#ifndef ADAS_GLOBAL_PLANNER__ROUTE_ADAPTER_HPP_
#define ADAS_GLOBAL_PLANNER__ROUTE_ADAPTER_HPP_

#include <cstdint>
#include <string>

#include "adas_msgs/msg/global_route.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace adas::planning {

inline bool global_route_frame_valid(const std::string& header_frame,
                                     const std::string& declared_frame) {
  return !header_frame.empty() && header_frame == declared_frame;
}

class RouteAdapterNode : public rclcpp::Node {
 public:
  RouteAdapterNode();

 private:
  void adapt_and_publish(const adas_msgs::msg::GlobalRoute& route);

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::Subscription<adas_msgs::msg::GlobalRoute>::SharedPtr subscription_;
  std::uint32_t last_route_id_{0U};
  std::uint8_t last_status_{adas_msgs::msg::GlobalRoute::STATUS_INVALID};
  bool published_once_{false};
};

}  // namespace adas::planning

#endif  // ADAS_GLOBAL_PLANNER__ROUTE_ADAPTER_HPP_
