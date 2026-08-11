cmake_minimum_required(VERSION 3.16)
project(adas_launch)

find_package(ament_cmake REQUIRED)
find_package(adas_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(fault_injection_runner test/fault_injection_runner.cpp)
ament_target_dependencies(fault_injection_runner adas_msgs nav_msgs rclcpp)
install(TARGETS fault_injection_runner DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../../adas_mcu/tools/mcu_sil_runner.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_scenario_overlay test/test_scenario_overlay.py)
  add_launch_test(test/test_invalid_parameters.py TIMEOUT 30)
  add_launch_test(test/test_sil_lifecycle_mrm.py TIMEOUT 90)
  add_launch_test(test/test_sil_aeb.py TIMEOUT 80)
  add_launch_test(test/test_sil_acc.py TIMEOUT 110)
  add_launch_test(test/test_carla_launch_smoke.py TIMEOUT 50)
endif()

ament_package()
