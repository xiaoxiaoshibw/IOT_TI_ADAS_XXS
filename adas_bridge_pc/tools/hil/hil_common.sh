#!/usr/bin/env bash
set -Eeuo pipefail

HIL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ADAS_PC_DIR="$(cd -- "${HIL_DIR}/../.." && pwd)"
PROJECT_ROOT="$(cd -- "${ADAS_PC_DIR}/.." && pwd)"
JETSON_HOST="${JETSON_HOST:-jetson@192.168.100.32}"

pass() { printf 'PASS  %s\n' "$*"; }
warn() { printf 'WARN  %s\n' "$*"; }
fail() { printf 'FAIL  %s\n' "$*" >&2; return 1; }

load_pc_ros() {
  # shellcheck source=/dev/null
  source "${ADAS_PC_DIR}/scripts/common.sh"
  source_workspace
}

new_artifact_root() {
  local out
  out="${PROJECT_ROOT}/evidence/artifacts/hil_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "${out}"/{environment,carla,ros2_pc,ros2_jetson,can,mcu,vehicle,screenshots}
  printf '%s\n' "${out}"
}
