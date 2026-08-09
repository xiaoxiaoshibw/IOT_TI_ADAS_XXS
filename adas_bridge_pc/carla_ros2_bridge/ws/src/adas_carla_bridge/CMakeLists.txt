cmake_minimum_required(VERSION 3.16)
project(adas_carla_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(adas_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(adas_carla_can_protocol src/can_protocol.cpp)
target_include_directories(adas_carla_can_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(can_feedback_node src/can_feedback_node.cpp)
target_link_libraries(can_feedback_node adas_carla_can_protocol)
ament_target_dependencies(can_feedback_node rclcpp adas_msgs)

add_executable(can_rtt_probe src/can_rtt_probe.cpp)
target_link_libraries(can_rtt_probe adas_carla_can_protocol)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS
  adas_carla_can_protocol
  can_feedback_node
  can_rtt_probe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS adas_carla_bridge/bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bridge_node)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_can_protocol_cpp test/test_can_protocol_cpp.cpp)
  target_link_libraries(test_can_protocol_cpp adas_carla_can_protocol)
  ament_add_pytest_test(test_can_protocol_py test/test_can_protocol.py)
  ament_add_pytest_test(test_bridge_safety test/test_bridge_safety.py)
  ament_add_pytest_test(test_carla_world_reference test/test_carla_world_reference.py)
  ament_add_pytest_test(test_map_export test/test_map_export.py)
endif()

ament_export_include_directories(include)
ament_export_libraries(adas_carla_can_protocol)
ament_package()
