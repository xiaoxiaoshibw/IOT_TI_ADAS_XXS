#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /bridge_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
exec ros2 run adas_carla_bridge bridge_node "$@"
