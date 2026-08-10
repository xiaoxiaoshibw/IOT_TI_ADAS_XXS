cmake_minimum_required(VERSION 3.16)
project(adas_scenario_loader)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scenario_loader test/test_scenario_loader.cpp)
  target_include_directories(test_scenario_loader PRIVATE include)
endif()

ament_export_include_directories(include)
ament_package()
