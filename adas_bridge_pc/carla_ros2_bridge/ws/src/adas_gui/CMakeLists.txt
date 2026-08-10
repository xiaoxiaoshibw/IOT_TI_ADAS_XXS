cmake_minimum_required(VERSION 3.16)
project(adas_gui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(adas_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt6 QUIET COMPONENTS Widgets Network Svg)

if(NOT Qt6_FOUND)
  message(WARNING "Qt6 Widgets not found - adas_gui executable will be skipped")
else()
  set(CMAKE_AUTOMOC ON)
  set(CMAKE_AUTORCC ON)
  add_executable(adas_gui
    src/main.cpp
    src/main_window.cpp
    src/map_view.cpp
    src/scenario_run_panel.cpp
    src/launch_panel.cpp
    src/process_manager.cpp
    src/ros_bridge.cpp
    src/safety_panel.cpp
    src/log_drawer.cpp
    src/realtime_plot.cpp
    src/fault_inject_panel.cpp
    src/widgets.cpp
    src/request_tracker.cpp
    src/icons.cpp
    src/telemetry_freshness.cpp
    src/secure_settings.cpp
    src/orin_stack_manager.cpp
    src/adas_gui.qrc)
  target_include_directories(adas_gui PRIVATE src)
  target_link_libraries(adas_gui Qt6::Widgets Qt6::Network Qt6::Svg)
  ament_target_dependencies(adas_gui rclcpp adas_msgs std_msgs nav_msgs geometry_msgs)
  # 与 adas_global_planner 相同的普通可执行文件安装模式（ros2 run 发现路径）
  install(TARGETS adas_gui DESTINATION lib/${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  if(Qt6_FOUND)
    # 全部测试统一在 if(Qt6_FOUND) 里配置；format / dds_env 当前只用
    # std::string，碰巧不需要 Qt 也能编过，但保留 Qt6::Core 链接以便未来
    # 给 format / dds_env 加 QString 化接口时不会突然缺链接。
    ament_add_gtest(test_format test/test_format.cpp)
    target_include_directories(test_format PRIVATE src)
    target_link_libraries(test_format Qt6::Core)
    ament_add_gtest(test_map_camera test/test_map_camera.cpp)
    target_include_directories(test_map_camera PRIVATE src)
    target_link_libraries(test_map_camera Qt6::Core)
    ament_add_gtest(test_map_objects test/test_map_objects.cpp src/map_view.cpp
      APPEND_ENV LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0)
    target_include_directories(test_map_objects PRIVATE src)
    target_link_libraries(test_map_objects Qt6::Widgets)
    # snap 自带的 libpthread.so.0 缺失 __libc_pthread_init，会在 Qt6 Widgets
    # 测试启动时崩。强制使用系统 libpthread 让 colcon test 可重复运行。
    ament_add_gtest(test_lane_hit test/test_lane_hit.cpp)
    target_include_directories(test_lane_hit PRIVATE src)
    target_link_libraries(test_lane_hit Qt6::Core)
    ament_add_gtest(test_health_model test/test_health_model.cpp)
    target_include_directories(test_health_model PRIVATE src)
    target_link_libraries(test_health_model Qt6::Core)
    ament_add_gtest(test_telemetry_freshness
      test/test_telemetry_freshness.cpp
      src/telemetry_freshness.cpp)
    target_include_directories(test_telemetry_freshness PRIVATE src)
    target_link_libraries(test_telemetry_freshness Qt6::Core)
    ament_add_gtest(test_safety_panel
      test/test_safety_panel.cpp
      src/safety_panel.cpp
      src/widgets.cpp
      src/icons.cpp
      src/telemetry_freshness.cpp)
    target_include_directories(test_safety_panel PRIVATE src)
    target_link_libraries(test_safety_panel Qt6::Widgets Qt6::Svg)
    ament_target_dependencies(test_safety_panel
      rclcpp adas_msgs std_msgs nav_msgs geometry_msgs)
    ament_add_gtest(test_request_tracker
      test/test_request_tracker.cpp
      src/request_tracker.cpp)
    target_include_directories(test_request_tracker PRIVATE src)
    target_link_libraries(test_request_tracker Qt6::Core)
    ament_add_gtest(test_dds_env test/test_dds_env.cpp)
    target_include_directories(test_dds_env PRIVATE src)
    target_link_libraries(test_dds_env Qt6::Core)
    # launch_config 依赖 QString，测试需要 Qt6::Core
    ament_add_gtest(test_launch_config test/test_launch_config.cpp)
    target_include_directories(test_launch_config PRIVATE src)
    target_link_libraries(test_launch_config Qt6::Core)
    # preflight_check 的 classify_* 是纯逻辑，但头文件里的 probe_* 用了 QDir/
    # QFile/QProcess，仍需 Qt6::Core 链接才能过编译。
    ament_add_gtest(test_preflight_check test/test_preflight_check.cpp)
    target_include_directories(test_preflight_check PRIVATE src)
    target_link_libraries(test_preflight_check Qt6::Core)
    # demo_presets 依赖 QString，测试需要 Qt6::Core
    ament_add_gtest(test_demo_presets test/test_demo_presets.cpp)
    target_include_directories(test_demo_presets PRIVATE src)
    target_link_libraries(test_demo_presets Qt6::Core)
    ament_add_gtest(test_scenario_workflow test/test_scenario_workflow.cpp)
    target_include_directories(test_scenario_workflow PRIVATE src)
    target_link_libraries(test_scenario_workflow Qt6::Core)
    ament_add_gtest(test_process_manager
      test/test_process_manager.cpp
      src/process_manager.cpp
      src/orin_stack_manager.cpp
      src/secure_settings.cpp)
    target_include_directories(test_process_manager PRIVATE src)
    target_link_libraries(test_process_manager Qt6::Core Qt6::Network)
    # single_instance 是真实 flock(2)，用 QTemporaryDir 隔离临时文件，需要 Qt6::Core。
    ament_add_gtest(test_single_instance test/test_single_instance.cpp)
    target_include_directories(test_single_instance PRIVATE src)
    target_link_libraries(test_single_instance Qt6::Core)
    # orin_stack_manager 命令行拼装的纯函数测试（不真 ssh）
    ament_add_gtest(test_orin_stack_manager
      test/test_orin_stack_manager.cpp
      src/orin_stack_manager.cpp)
    target_include_directories(test_orin_stack_manager PRIVATE src)
    target_link_libraries(test_orin_stack_manager Qt6::Core)
    # secure_settings 用 QTemporaryDir，需要 Qt6::Core
    ament_add_gtest(test_secure_settings test/test_secure_settings.cpp
      src/secure_settings.cpp)
    target_include_directories(test_secure_settings PRIVATE src)
    target_link_libraries(test_secure_settings Qt6::Core)
    # 端到端烟雾测试：实例化 ProcessManager + 跑 preflight，验证
    # start_full_stack=false 时 Orin 不被体检、startAll 链路能走到 startCarla
    # 并优雅报 Failed（CARLA 二进制缺失时）。
    ament_add_gtest(test_smoke_one_click_start
      test/test_smoke_one_click_start.cpp
      src/process_manager.cpp
      src/orin_stack_manager.cpp
      src/secure_settings.cpp)
    target_include_directories(test_smoke_one_click_start PRIVATE src)
    target_link_libraries(test_smoke_one_click_start Qt6::Core Qt6::Network)
  endif()
endif()

ament_package()
