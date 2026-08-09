cmake_minimum_required(VERSION 3.8)
project(adas_mqtt_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Python 包
ament_python_install_package(${PROJECT_NAME})

# 安装 Python 模块
install(PROGRAMS
  ${PROJECT_NAME}/mqtt_bridge.py
  ${PROJECT_NAME}/mqtt_bridge_node.py
  ${PROJECT_NAME}/__init__.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装 config 文件（该可选 IoT 配置不影响 PC SIL）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY
    config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

# 安装测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
