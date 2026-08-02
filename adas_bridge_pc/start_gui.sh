#!/usr/bin/env bash
# Start the Qt6 ADAS safety dashboard.

set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/scripts/common.sh"

source_workspace
exec ros2 run adas_gui adas_gui "$@"
