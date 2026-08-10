cmake_minimum_required(VERSION 3.16)
project(adas_redundancy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(adas_msgs REQUIRED)
find_package(adas_common REQUIRED)

add_library(${PROJECT_NAME}_core src/arbiter_core.cpp)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(${PROJECT_NAME}_core adas_common)

add_library(${PROJECT_NAME}_node SHARED src/redundancy_arbiter_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp rclcpp_components rclcpp_lifecycle diagnostic_msgs diagnostic_updater
  adas_msgs adas_common)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "adas::system::RedundancyArbiterNode"
  EXECUTABLE redundancy_arbiter_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}_core test/test_arbiter_core.cpp)
  target_link_libraries(test_${PROJECT_NAME}_core ${PROJECT_NAME}_core)
endif()

ament_package()
