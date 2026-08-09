#!/usr/bin/env bash
# M3 故障注入回归：每个场景使用全新 SIL 进程组，避免一个故障污染下一个判定。
set -Eeuo pipefail
IFS=$'\n\t'

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOC_WS="${ROOT}/adas_soc"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
LOG_ROOT="${FAULT_LOG_ROOT:-${ROOT}/logs/fault_injection}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-146}"

set +u
source "${ROS_SETUP}"
set -u
if [[ ! -f "${SOC_WS}/install/setup.bash" ]]; then
  (cd "${SOC_WS}" && colcon build --symlink-install --packages-select adas_launch)
fi
set +u
source "${SOC_WS}/install/setup.bash"
set -u
mkdir -p "${LOG_ROOT}"
RUNNER="$(ros2 pkg prefix adas_launch)/lib/adas_launch/fault_injection_runner"
[[ -x "${RUNNER}" ]] || {
  echo "fault_injection_runner 未安装：${RUNNER}" >&2
  exit 1
}

stop_launch() {
  local pid="${1:-}"
  [[ -n "${pid}" ]] || return 0
  if kill -0 "${pid}" 2>/dev/null; then
    kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    for _ in {1..30}; do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL -- "-${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
}

launch_pid=""
cleanup_on_exit() {
  stop_launch "${launch_pid}"
}
trap cleanup_on_exit EXIT
trap 'exit 130' INT TERM

wait_stack_ready() {
  local deadline=$((SECONDS + 45))
  while (( SECONDS < deadline )); do
    local topics
    topics="$(ros2 topic list 2>/dev/null || true)"
    if grep -qx /adas/system/safety_status <<<"${topics}" &&
       grep -qx /adas/control/gate/status <<<"${topics}"; then
      # SafetyMonitor diagnostics startup grace is 5 s; wait beyond it so a
      # deliberately killed component is distinguished from normal startup.
      sleep 7
      return 0
    fi
    sleep 1
  done
  return 1
}

for scenario in A B C D E; do
  run_id="$(date +%Y%m%d_%H%M%S)_${scenario}_$$"
  log="${LOG_ROOT}/${run_id}.log"
  echo "=== fault injection scenario ${scenario} (ROS_DOMAIN_ID=${ROS_DOMAIN_ID}) ==="
  setsid ros2 launch adas_launch sil.launch.py >"${log}" 2>&1 &
  launch_pid=$!
  failed=0
  if ! wait_stack_ready || ! "${RUNNER}" --scenario "${scenario}" \
      --launch-pgid "${launch_pid}"; then
    failed=1
  fi
  stop_launch "${launch_pid}"
  if (( failed )); then
    echo "SCENARIO_${scenario} FAIL; launch log tail:" >&2
    tail -100 "${log}" >&2 || true
    exit 1
  fi
  echo "SCENARIO_${scenario} PASS"
done

echo "FAULT INJECTION PASS (A-E)"
