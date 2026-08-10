#!/usr/bin/env bash
# Local PC + Orin stack + F280025C host-runner HIL on vcan0 only.
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
  (cd "$ROOT/adas_soc" && colcon build --symlink-install --event-handlers console_direct+)
fi
set +u
# shellcheck disable=SC1091
source "$ROOT/adas_soc/install/setup.bash"
set -u

if (( BUILD )) && [[ " $* " == *" --gui"* || " $* " == *" --gui-offscreen"* ]]; then
  "$ROOT/adas_bridge_pc/build.sh"
fi

filtered=()
for arg in "$@"; do
  [[ "$arg" == "--build" ]] || filtered+=("$arg")
done

# This entry is intentionally isolated from normal SIL/HIL discovery.
export ROS_DOMAIN_ID=145
exec python3 "$ROOT/scripts/local_three_machine/orchestrator.py" "${filtered[@]}"
