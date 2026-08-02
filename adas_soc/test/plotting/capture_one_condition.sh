#!/usr/bin/env bash
# Capture exactly one SIL scenario.  Run each invocation from a fresh WSL
# session so DDS participants from an earlier condition cannot pollute CSVs.
# The ROS/colcon environment must already be sourced by the caller.
set -euo pipefail

if [[ $# -lt 3 || $# -gt 4 ]]; then
  echo "Usage: $0 <case-name> <launch-file> <duration-s> [inject-primary-failure-at-s]" >&2
  exit 2
fi

NAME="$1"
LAUNCH_FILE="$2"
DURATION="$3"
INJECT_AT="${4:-}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RECORDER="${ROOT}/test/plotting/record_sil.py"
DATA_DIR="${ROOT}/test/plotting/data"
LOG="/tmp/adas_plot_${NAME}.log"
mkdir -p "${DATA_DIR}"

LAUNCH_PID=""
INJECT_PID=""
cleanup() {
  [[ -n "${INJECT_PID}" ]] && kill "${INJECT_PID}" 2>/dev/null || true
  if [[ -n "${LAUNCH_PID}" ]]; then
    # ros2 launch may leave its child nodes alive after an interrupt.  Bound
    # shutdown so the next cold-start capture cannot inherit DDS publishers.
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    pkill -INT -P "${LAUNCH_PID}" 2>/dev/null || true
    for _ in 1 2 3 4 5; do
      kill -0 "${LAUNCH_PID}" 2>/dev/null || break
      sleep 1
    done
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    pkill -TERM -P "${LAUNCH_PID}" 2>/dev/null || true
    sleep 1
    kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
    pkill -KILL -P "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[plot] ${NAME}: ${LAUNCH_FILE}, recording ${DURATION}s after 5s warm-up"
ros2 launch adas_launch "${LAUNCH_FILE}" >"${LOG}" 2>&1 &
LAUNCH_PID=$!
sleep 5

if [[ -n "${INJECT_AT}" ]]; then
  (
    sleep "${INJECT_AT}"
    pkill -f 'trajectory_planner_node.*__ns:=/primary' || true
  ) &
  INJECT_PID=$!
fi

python3 "${RECORDER}" --output "${DATA_DIR}/${NAME}.csv" --duration-s "${DURATION}"
echo "[plot] wrote ${DATA_DIR}/${NAME}.csv"
