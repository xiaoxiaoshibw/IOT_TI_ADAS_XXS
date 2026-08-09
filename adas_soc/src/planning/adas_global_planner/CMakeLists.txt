cmake_minimum_required(VERSION 3.16)
project(adas_global_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(adas_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_core
  src/global_planner_core.cpp
  src/semantic_route.cpp
  src/route_validator.cpp)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

add_executable(global_planner_node src/global_planner_node.cpp)
target_link_libraries(global_planner_node ${PROJECT_NAME}_core)
ament_target_dependencies(global_planner_node
  rclcpp adas_msgs nav_msgs geometry_msgs std_msgs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
# 节点可执行文件必须装到 lib/${PROJECT_NAME}，ros2 run / launch 才找得到（装到 bin 会
# 报 "No executable found"）。
install(TARGETS global_planner_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}_core test/test_global_planner_core.cpp)
  target_link_libraries(test_${PROJECT_NAME}_core ${PROJECT_NAME}_core)
endif()

ament_package()
