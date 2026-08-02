#!/usr/bin/env bash
set -eo pipefail

: "${NAV_WS:=/tmp/adas_nav_ws}"
source /opt/ros/jazzy/setup.bash
source "$NAV_WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

"$NAV_WS/install/adas_global_planner/bin/global_planner_node" \
  > /tmp/global_planner_node.log 2>&1 &
planner_pid=$!
cleanup() {
  kill "$planner_pid" 2>/dev/null || true
  wait "$planner_pid" 2>/dev/null || true
}
trap cleanup EXIT

python3 "$NAV_WS/navigation_topic_smoke.py"
cat /tmp/global_planner_node.log
