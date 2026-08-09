cmake_minimum_required(VERSION 3.16)
project(adas_can_gateway)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(adas_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)

add_library(${PROJECT_NAME}_protocol
  src/protocol.cpp
  src/hil_session_manager.cpp
  src/can_simulator.cpp)
target_include_directories(${PROJECT_NAME}_protocol PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

add_executable(can_gateway_node src/can_gateway_node.cpp)
target_link_libraries(can_gateway_node ${PROJECT_NAME}_protocol PkgConfig::LIBUSB)
ament_target_dependencies(can_gateway_node rclcpp adas_msgs diagnostic_updater std_srvs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_protocol
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS can_gateway_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol ${PROJECT_NAME}_protocol)
  ament_add_gtest(test_feedback_monitor test/test_feedback_monitor.cpp)
  target_link_libraries(test_feedback_monitor ${PROJECT_NAME}_protocol)
  ament_add_gtest(test_startup_gate test/test_startup_gate.cpp)
  target_link_libraries(test_startup_gate ${PROJECT_NAME}_protocol)
  ament_add_gtest(test_hil_session_manager test/test_hil_session_manager.cpp)
  target_link_libraries(test_hil_session_manager ${PROJECT_NAME}_protocol)

  ament_add_gtest(test_${PROJECT_NAME}_node test/test_can_gateway_node.cpp)
  target_link_libraries(test_${PROJECT_NAME}_node ${PROJECT_NAME}_protocol PkgConfig::LIBUSB)
  target_compile_definitions(test_${PROJECT_NAME}_node PRIVATE ADAS_CAN_GATEWAY_NO_MAIN)
  ament_target_dependencies(test_${PROJECT_NAME}_node
    rclcpp adas_msgs diagnostic_updater std_srvs)
endif()

ament_package()
