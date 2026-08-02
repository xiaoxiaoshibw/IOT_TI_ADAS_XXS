#!/usr/bin/env bash
set -Eeuo pipefail

HIL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${HIL_DIR}/hil_common.sh"

duration_s=1800
artifact_dir=""
manager_pid=""
session_dir=""
manager_log=""
cleanup_done=0

usage() {
  cat <<'EOF'
Usage: run_long_duration_test.sh [--time SECONDS] [--output DIR]

Starts the existing gated HIL stack, collects read-only stability metrics,
stops only that managed session, and writes a CSV-derived Phase 1 summary.
EOF
}

while (($#)); do
  case "$1" in
    --time)
      (($# >= 2)) || { fail "--time 缺少参数"; exit 2; }
      duration_s="$2"
      shift 2
      ;;
    --output)
      (($# >= 2)) || { fail "--output 缺少参数"; exit 2; }
      artifact_dir="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "未知参数: $1"
      usage >&2
      exit 2
      ;;
  esac
done

if ! python3 - "${duration_s}" <<'PY'
import math
import sys
try:
    value = float(sys.argv[1])
except ValueError:
    raise SystemExit(2)
if not math.isfinite(value) or value <= 0:
    raise SystemExit(2)
PY
then
  fail "--time 必须是有限正数"
  exit 2
fi

if [[ -z "${artifact_dir}" ]]; then
  artifact_dir="${PROJECT_ROOT}/evidence/artifacts/hil_long_$(date +%Y%m%d_%H%M%S)"
elif [[ "${artifact_dir}" != /* ]]; then
  artifact_dir="${PROJECT_ROOT}/${artifact_dir}"
fi
mkdir -p "${artifact_dir}"/{environment,logs,can,mcu,vehicle,runtime}
manager_log="${artifact_dir}/logs/hil_manager.log"
printf '%s\n' "${duration_s}" >"${artifact_dir}/environment/requested_duration_s.txt"
{
  date --iso-8601=ns
  uname -a
  git -C "${PROJECT_ROOT}" branch --show-current
  git -C "${PROJECT_ROOT}" rev-parse HEAD
  git -C "${PROJECT_ROOT}" status --short
} >"${artifact_dir}/environment/host_and_git.txt"

stop_managed_session() {
  local wait_status=0
  (( cleanup_done == 0 )) || return 0
  cleanup_done=1
  if [[ -n "${manager_pid}" ]] && kill -0 "${manager_pid}" 2>/dev/null; then
    if [[ -n "${session_dir}" && -r "${session_dir}/manager.pid" ]]; then
      "${HIL_DIR}/stop_hil_pc.sh" "${session_dir}" \
        >>"${artifact_dir}/logs/stop.log" 2>&1 || true
    else
      kill -TERM "${manager_pid}" 2>/dev/null || true
    fi
    set +e
    wait "${manager_pid}" 2>/dev/null
    wait_status=$?
    set -e
    printf '%s\n' "${wait_status}" >"${artifact_dir}/logs/manager_wait_status.txt"
  fi
}

on_exit() {
  local status=$?
  trap - EXIT INT TERM
  stop_managed_session
  exit "${status}"
}
trap on_exit EXIT INT TERM

pass "长测证据目录: ${artifact_dir}"
"${HIL_DIR}/check_hil_preflight.sh" | tee "${artifact_dir}/logs/preflight.log"

load_pc_ros
printenv | grep -E '^(ROS_|RMW_|CYCLONEDDS|FASTDDS|DOMAIN)' \
  >"${artifact_dir}/environment/ros_environment.txt" || true
setsid "${HIL_DIR}/start_hil_e2e.sh" >"${manager_log}" 2>&1 &
manager_pid=$!
printf '%s\n' "${manager_pid}" >"${artifact_dir}/logs/launcher_pid.txt"

start_deadline=$((SECONDS + 180))
while (( SECONDS < start_deadline )); do
  if ! kill -0 "${manager_pid}" 2>/dev/null; then
    tail -n 160 "${manager_log}" >&2 || true
    fail "HIL manager 在 ready 前退出"
    exit 3
  fi
  session_dir=$(find "${ADAS_PC_DIR}/logs" -maxdepth 1 -type d -name 'hil_run_*' \
    -exec sh -c 'test -r "$1/manager.pid" && test "$(cat "$1/manager.pid")" = "$2"' _ {} "${manager_pid}" \; \
    -print | head -n 1)
  if [[ -n "${session_dir}" ]] && grep -q ' event=STACK_RUNNING ' "${manager_log}"; then
    break
  fi
  sleep 1
done
[[ -n "${session_dir}" ]] || { fail "180 秒内未获得本次 HIL session_dir"; exit 4; }
grep -q ' event=STACK_RUNNING ' "${manager_log}" \
  || { tail -n 160 "${manager_log}" >&2 || true; fail "180 秒内 HIL 未通过十阶段状态门"; exit 4; }
printf '%s\n' "${session_dir}" >"${artifact_dir}/logs/hil_session_dir.txt"
pass "HIL ready: ${session_dir}"

"${HIL_DIR}/check_hil_runtime.sh" --session-dir "${session_dir}" \
  >"${artifact_dir}/runtime/ready_gate.log" 2>&1
pass "长测前 runtime gate PASS"

python3 "${HIL_DIR}/hil_metrics_collector.py" \
  --duration "${duration_s}" \
  --output-dir "${artifact_dir}" \
  --jetson-host "${JETSON_HOST}" \
  --manager-pid "${manager_pid}" \
  | tee "${artifact_dir}/logs/collector.log"

"${HIL_DIR}/check_hil_runtime.sh" --session-dir "${session_dir}" \
  >"${artifact_dir}/runtime/final_gate.log" 2>&1
pass "长测后 runtime gate PASS"

cp -- "${session_dir}/processes.tsv" "${artifact_dir}/runtime/processes.tsv"
cp -- "${session_dir}/exit_status.tsv" "${artifact_dir}/runtime/exit_status_before_stop.tsv" 2>/dev/null || true
cp -- "${session_dir}/health_gate.log" "${artifact_dir}/runtime/startup_health_gate.log"
cp -- "${session_dir}/stack.log" "${artifact_dir}/logs/stack.log"
cp -- "${session_dir}/bridge.log" "${artifact_dir}/logs/bridge.log"
cp -- "${session_dir}/carla.log" "${artifact_dir}/logs/carla.log"
cp -- "${session_dir}/gui.log" "${artifact_dir}/logs/gui.log"

stop_managed_session
ssh -o BatchMode=yes -o ConnectTimeout=8 "${JETSON_HOST}" '
  date --iso-8601=ns
  ip -details -statistics link show can1
  timeout 2 candump -L can1,201:7FF,202:7FF,203:7FF,204:7FF,206:7FF || true
' >"${artifact_dir}/can/post_stop_can_and_mcu.txt" 2>&1 || true
trap - EXIT INT TERM
phase_pass=$(python3 - "${artifact_dir}/metrics.json" <<'PY'
import json
import sys
print("true" if json.load(open(sys.argv[1], encoding="utf-8")).get("phase1_pass") else "false")
PY
)
if [[ "${phase_pass}" != "true" ]]; then
  fail "Phase 1 稳定性指标未通过；真实 CSV 与 FAIL 摘要已保留: ${artifact_dir}"
  exit 5
fi
pass "Phase 1 长时间稳定性测试完成: ${artifact_dir}"
