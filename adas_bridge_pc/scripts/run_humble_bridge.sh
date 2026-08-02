#!/usr/bin/env bash
# Run only the ROS-facing CARLA bridge in Humble, matching the Orin runtime.
set -euo pipefail

# Make DDS routing self-contained.  The GUI normally inherits this from
# start_gui.sh, but the bridge must not silently fall back to multicast when
# launched by QProcess or a service with a reduced environment.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

IMAGE="${ADAS_HUMBLE_BRIDGE_IMAGE:-adas-humble-carla-bridge:0.0.2}"
PC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "${PC_ROOT}/logs"
sudo -n docker image inspect "${IMAGE}" >/dev/null 2>&1 || {
  echo "Humble Bridge image missing: ${IMAGE}; build with humble_bridge.Dockerfile" >&2
  exit 69
}
exec sudo -n docker run --rm \
  --name adas-humble-carla-bridge \
  --network host \
  --init \
  --stop-signal SIGINT \
  --stop-timeout 5 \
  --device /dev/bus/usb:/dev/bus/usb \
  --volume "${PC_ROOT}/logs:/bridge_ws/logs" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI="${CYCLONEDDS_URI:-}" \
  "${IMAGE}" "$@"
