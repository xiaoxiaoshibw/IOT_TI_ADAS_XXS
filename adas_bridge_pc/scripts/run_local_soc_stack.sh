#!/usr/bin/env bash
# GUI-owned CARLA + local SoC control stack. CARLA and its bridge are owned by
# ProcessManager; this script owns only the SoC lifecycle nodes.
set -Eeuo pipefail

scenario_id="free"
while (($#)); do
  case "$1" in
    --scenario)
      [[ $# -ge 2 && -n "$2" ]] || {
        echo "--scenario requires a non-empty catalog id" >&2
        exit 2
      }
      scenario_id="$2"
      shift 2
      ;;
    --help|-h)
      echo "usage: $0 [--scenario CATALOG_ID]"
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
SOC_SETUP="${ROOT}/adas_soc/install/setup.bash"

[[ -f "${ROS_SETUP}" ]] || { echo "Missing ROS setup: ${ROS_SETUP}" >&2; exit 1; }
[[ -f "${SOC_SETUP}" ]] || {
  echo "Missing SoC workspace: ${SOC_SETUP}; build adas_soc first" >&2
  exit 1
}

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${SOC_SETUP}"
set -u

exec ros2 launch adas_launch carla.launch.py "scenario_id:=${scenario_id}"
