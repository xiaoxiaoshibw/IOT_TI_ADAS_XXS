#!/usr/bin/env bash
# Start the CARLA ROS 2 bridge. Arguments override/add bridge-node options.

set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/scripts/common.sh"

source_workspace
exec ros2 run adas_carla_bridge bridge_node --scenario "${SCENARIO:-free}" --town "${TOWN:-Town04}" "$@"
