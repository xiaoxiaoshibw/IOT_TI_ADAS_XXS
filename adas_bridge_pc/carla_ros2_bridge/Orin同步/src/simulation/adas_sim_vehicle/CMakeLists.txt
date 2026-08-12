cmake_minimum_required(VERSION 3.16)
project(adas_sim_vehicle)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(adas_msgs REQUIRED)
find_package(adas_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 纯算法核心（无 ROS 依赖，可独立单测）
add_library(${PROJECT_NAME}_core src/sim_vehicle_core.cpp)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(${PROJECT_NAME}_core adas_common)

# 节点组件
add_library(${PROJECT_NAME}_node SHARED src/sim_vehicle_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp rclcpp_components adas_msgs adas_common nav_msgs
  geometry_msgs tf2_ros)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "adas::sim::SimVehicleNode"
  EXECUTABLE sim_vehicle_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}_core test/test_sim_vehicle_core.cpp)
  target_link_libraries(test_${PROJECT_NAME}_core ${PROJECT_NAME}_core)
endif()

ament_package()
