#include <memory>

#include "adas_global_planner/route_adapter.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<adas::planning::RouteAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
