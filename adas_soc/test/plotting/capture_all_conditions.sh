#!/usr/bin/env bash
# Capture all implemented SIL conditions as CSV for MATLAB plotting.
# Run from a sourced colcon workspace root, for example:
#   bash src/test/plotting/capture_all_conditions.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RECORDER="${ROOT}/test/plotting/record_sil.py"
DATA_DIR="${ROOT}/test/plotting/data"
mkdir -p "${DATA_DIR}"

run_case() {
  local name="$1"
  local launch_file="$2"
  local duration="$3"
  local log="/tmp/adas_plot_${name}.log"
  echo "[plot] ${name}: ${launch_file}, ${duration}s"
  ros2 launch adas_launch "${launch_file}" >"${log}" 2>&1 &
  local launch_pid=$!
  python3 "${RECORDER}" --output "${DATA_DIR}/${name}.csv" --duration-s "${duration}"
  kill "${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
}

run_case sil_baseline sil.launch.py 60
run_case sil_acc sil_acc.launch.py 75
run_case sil_aeb sil_aeb.launch.py 50
run_case sil_overtake sil_overtake.launch.py 35
run_case sil_lqr sil_lqr.launch.py 30

echo "[plot] sil_redundant: inject primary trajectory_planner failure at t≈30s"
ros2 launch adas_launch sil_redundant.launch.py >/tmp/adas_plot_sil_redundant.log 2>&1 &
REDUNDANT_PID=$!
python3 "${RECORDER}" --output "${DATA_DIR}/sil_redundant.csv" --duration-s 50 &
RECORDER_PID=$!
sleep 30
pkill -f "trajectory_planner_node.*__ns:=/primary" || true
wait "${RECORDER_PID}"
kill "${REDUNDANT_PID}" 2>/dev/null || true
wait "${REDUNDANT_PID}" 2>/dev/null || true

echo "[plot] CSV capture complete: ${DATA_DIR}"
