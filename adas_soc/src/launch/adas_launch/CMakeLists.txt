cmake_minimum_required(VERSION 3.16)
project(adas_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_invalid_parameters.py TIMEOUT 30)
  add_launch_test(test/test_sil_lifecycle_mrm.py TIMEOUT 90)
  add_launch_test(test/test_sil_aeb.py TIMEOUT 80)
  add_launch_test(test/test_sil_acc.py TIMEOUT 110)
  add_launch_test(test/test_carla_launch_smoke.py TIMEOUT 50)
endif()

ament_package()
