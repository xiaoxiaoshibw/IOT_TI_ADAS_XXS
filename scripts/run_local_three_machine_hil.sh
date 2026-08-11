#!/usr/bin/env bash
# Legacy filename; runs the local MIL simulated-hardware loop on vcan0 only.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
BUILD=0
for arg in "$@"; do
  [[ "$arg" == "--build" ]] && BUILD=1
done

[[ -f "$ROS_SETUP" ]] || { echo "Missing ROS setup: $ROS_SETUP" >&2; exit 1; }
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u

if (( BUILD )) || [[ ! -f "$ROOT/adas_soc/install/setup.bash" ]]; then
  # ament_cmake_python cannot replace an old real package directory with the
  # symlink required by --symlink-install. This path is generated build output
  # only; remove exactly that stale artifact, never a source/install tree.
  stale_python_link="$ROOT/adas_soc/build/adas_msgs/ament_cmake_python/adas_msgs/adas_msgs"
  if [[ -d "$stale_python_link" && ! -L "$stale_python_link" ]]; then
    cmake -E remove_directory "$stale_python_link"
  fi
  build_run="$ROOT/logs/local_three_machine/$(date +%Y%m%d_%H%M%S)_build_$$"
  mkdir -p "$build_run"
  set +e
  (cd "$ROOT/adas_soc" && colcon build --symlink-install \
    --event-handlers console_direct+) 2>&1 | tee "$build_run/build.log"
  build_rc=${PIPESTATUS[0]}
  set -e
  if (( build_rc != 0 )); then
    BUILD_RUN="$build_run" BUILD_RC="$build_rc" python3 - <<'PY'
import json
import os
from pathlib import Path
run = Path(os.environ['BUILD_RUN'])
report = {
    'schema_version': 1, 'overall': 'FAIL', 'failed_stage': 'build',
    'checks': [{'name': 'Build', 'status': 'FAIL',
                'detail': 'colcon exited %s; see build.log' % os.environ['BUILD_RC']}],
    'log_dir': str(run), 'ros_domain_id': '145', 'can_interface': 'vcan0',
}
(run / 'report.json').write_text(
    json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
PY
    echo "Build failed; report=$build_run/report.json" >&2
    exit "$build_rc"
  fi
fi
set +u
# shellcheck disable=SC1091
source "$ROOT/adas_soc/install/setup.bash"
set -u

if (( BUILD )) && [[ " $* " == *" --gui"* || " $* " == *" --gui-offscreen"* || " $* " == *" --carla"* ]]; then
  pc_stale_python_link="$ROOT/adas_bridge_pc/carla_ros2_bridge/ws/build/adas_msgs/ament_cmake_python/adas_msgs/adas_msgs"
  if [[ -d "$pc_stale_python_link" && ! -L "$pc_stale_python_link" ]]; then
    cmake -E remove_directory "$pc_stale_python_link"
  fi
  "$ROOT/adas_bridge_pc/build.sh"
fi

if [[ " $* " == *" --carla"* ]]; then
  pc_setup="$ROOT/adas_bridge_pc/carla_ros2_bridge/ws/install/setup.bash"
  [[ -f "$pc_setup" ]] || {
    echo "PC CARLA bridge is not built; rerun with --build --carla" >&2
    exit 1
  }
  set +u
  # shellcheck disable=SC1090
  source "$pc_setup"
  set -u
fi

filtered=()
for arg in "$@"; do
  [[ "$arg" == "--build" ]] || filtered+=("$arg")
done

# This entry is intentionally isolated from normal SIL/HIL discovery.
export ROS_DOMAIN_ID=145
exec python3 "$ROOT/scripts/local_three_machine/orchestrator.py" "${filtered[@]}"
