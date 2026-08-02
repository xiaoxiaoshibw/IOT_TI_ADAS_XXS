#!/usr/bin/env bash
# Build the PC-side ROS 2 workspace. Extra arguments are passed to colcon.

set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/scripts/common.sh"

source_ros
cd "${WORKSPACE}"
colcon build --symlink-install "$@"
