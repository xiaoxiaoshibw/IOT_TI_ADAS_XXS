cmake_minimum_required(VERSION 3.16)
project(adas_lidar_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(lidar_perception_node src/lidar_perception_node.cpp)
ament_target_dependencies(lidar_perception_node
  rclcpp sensor_msgs diagnostic_msgs)

install(TARGETS lidar_perception_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
