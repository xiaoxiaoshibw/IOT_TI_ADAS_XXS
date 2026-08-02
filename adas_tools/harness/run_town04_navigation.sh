#!/usr/bin/env bash
# Phase 3 evidence runner.  It wraps the existing gated HIL startup and never
# bypasses Gate/CAN/MCU or changes an established launch file.
set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
HIL_DIR="${ROOT}/adas_pc/tools/hil"
VALIDATION_DIR="${ROOT}/ADAS0.0.2/SoC/tests/navigation_validation"
# shellcheck disable=SC1091
source "${HIL_DIR}/hil_common.sh"

scenario=""
duration=""
output=""
takeover_mode="observe"
manager_pid=""
bootstrap_pid=""
session_dir=""
cleanup_done=0

usage() {
  cat <<'EOF'
Usage: tools/harness/run_town04_navigation.sh --scenario CASE [options]

CASE: straight | semantic | junction | cancel | takeover
Options:
  --duration SEC               evidence collection duration
  --output DIR                 new result directory (must not exist)
  --test-build-matrix          takeover: run guarded matrix on this host/can1

The test-build matrix refuses production firmware using MCU CAN 0x204 proof.
Use takeover observation mode when an approved external fault injector is used.
EOF
}

while (($#)); do
  case "$1" in
    --scenario) scenario="${2:?missing scenario}"; shift 2 ;;
    --duration) duration="${2:?missing duration}"; shift 2 ;;
    --output) output="${2:?missing output}"; shift 2 ;;
    --test-build-matrix) takeover_mode="test-build-matrix"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) fail "未知参数: $1"; usage >&2; exit 2 ;;
  esac
done
case "${scenario}" in
  straight|semantic|junction|cancel|takeover) ;;
  *) fail "--scenario 必须是 straight|semantic|junction|cancel|takeover"; exit 2 ;;
esac
if [[ -z "${duration}" ]]; then
  case "${scenario}" in
    straight) duration=180 ;;
    semantic) duration=60 ;;
    junction|cancel) duration=120 ;;
    takeover) duration=45 ;;
  esac
fi
python3 - "${duration}" <<'PY'
import math, sys
value = float(sys.argv[1])
raise SystemExit(0 if math.isfinite(value) and value > 0 else 2)
PY

if [[ -z "${output}" ]]; then
  output="${ROOT}/evidence/navigation/town04_${scenario}_$(date +%Y%m%d_%H%M%S)"
elif [[ "${output}" != /* ]]; then
  output="${ROOT}/${output}"
fi
[[ ! -e "${output}" ]] || { fail "拒绝覆盖已有目录: ${output}"; exit 2; }
mkdir -p "${output}"/{csv,figures,logs,report,raw_hil,runtime}

stop_managed_session() {
  (( cleanup_done == 0 )) || return 0
  cleanup_done=1
  if [[ -n "${manager_pid}" ]] && kill -0 "${manager_pid}" 2>/dev/null; then
    if [[ -n "${session_dir}" && -r "${session_dir}/manager.pid" ]]; then
      "${HIL_DIR}/stop_hil_pc.sh" "${session_dir}" >>"${output}/logs/stop.log" 2>&1 || true
    else
      kill -TERM "${manager_pid}" 2>/dev/null || true
    fi
    wait "${manager_pid}" 2>/dev/null || true
  fi
  if [[ -n "${bootstrap_pid}" ]] && kill -0 "${bootstrap_pid}" 2>/dev/null; then
    kill -TERM "${bootstrap_pid}" 2>/dev/null || true
    wait "${bootstrap_pid}" 2>/dev/null || true
  fi
}
trap 'status=$?; trap - EXIT INT TERM; stop_managed_session; exit "$status"' EXIT INT TERM

export TOWN=Town04
export SCENARIO=free
"${HIL_DIR}/check_hil_preflight.sh" | tee "${output}/logs/preflight.log"
load_pc_ros
setsid "${HIL_DIR}/start_hil_e2e.sh" >"${output}/logs/hil_manager.log" 2>&1 &
manager_pid=$!
# Phase 2.2 deliberately holds a safe stop while no valid route exists.  Start
# the topology-selected bootstrap probe alongside the manager so its unchanged
# readiness gate can prove actual CARLA motion through the full HIL chain.
python3 "${VALIDATION_DIR}/route_generation_test.py" \
  --output "${output}/runtime/bootstrap_route.json" --distance 100 --timeout 30 \
  >"${output}/logs/bootstrap_route.log" 2>&1 &
bootstrap_pid=$!

deadline=$((SECONDS + 180))
while (( SECONDS < deadline )); do
  kill -0 "${manager_pid}" 2>/dev/null || { tail -n 100 "${output}/logs/hil_manager.log" >&2; exit 3; }
  session_dir=$(find "${ADAS_PC_DIR}/logs" -maxdepth 1 -type d -name 'hil_run_*' \
    -exec sh -c 'test -r "$1/manager.pid" && test "$(cat "$1/manager.pid")" = "$2"' _ {} "${manager_pid}" \; \
    -print | head -n 1)
  [[ -n "${session_dir}" ]] && grep -q ' event=STACK_RUNNING ' "${output}/logs/hil_manager.log" && break
  sleep 1
done
[[ -n "${session_dir}" ]] && grep -q ' event=STACK_RUNNING ' "${output}/logs/hil_manager.log" \
  || { fail "180 秒内 HIL 未 ready"; exit 4; }
printf '%s\n' "${session_dir}" >"${output}/runtime/hil_session_dir.txt"
wait "${bootstrap_pid}"
bootstrap_pid=""
"${HIL_DIR}/check_hil_runtime.sh" --session-dir "${session_dir}" >"${output}/runtime/ready_gate.log"

# Phase 3 must not silently fall back to the Phase 1 nav_msgs/Path deployment.
topic_snapshot="${output}/runtime/topic_list.txt"
ros2 topic list >"${topic_snapshot}"
for required_topic in \
  /adas/navigation/goal_pose \
  /adas/navigation/global_route \
  /adas/navigation/status \
  /adas/map/lane_graph \
  /adas/planning/trajectory \
  /adas/control/gate/status \
  /adas/mcu/status; do
  grep -Fxq "${required_topic}" "${topic_snapshot}" || {
    fail "Phase 3 Topic 缺失: ${required_topic}；拒绝使用旧 Path 接口继续实验"
    exit 4
  }
done
route_type=$(ros2 topic type /adas/navigation/global_route)
[[ "${route_type}" == "adas_msgs/msg/GlobalRoute" ]] || {
  fail "GlobalRoute 类型错误: ${route_type}"
  exit 4
}

contract_snapshot="${output}/runtime/navigation_contract.txt"
{
  printf 'goal_pose_type=%s\n' "$(ros2 topic type /adas/navigation/goal_pose)"
  printf 'global_route_type=%s\n' "${route_type}"
  ros2 interface show adas_msgs/msg/GlobalRoute
  ros2 interface show adas_msgs/msg/RoutePoint
  ros2 topic info -v /adas/navigation/global_route
} >"${contract_snapshot}"

python3 "${VALIDATION_DIR}/tf_chain_test.py" \
  --output "${output}/runtime/tf_chain.json" --timeout 10

csv="${output}/csv/hil_town04_${scenario}.csv"
python3 "${ROOT}/tools/hil_logger/hil_logger.py" --output "${csv}" --duration "${duration}" \
  >"${output}/logs/ros_logger.log" 2>&1 &
logger_pid=$!
python3 "${HIL_DIR}/hil_metrics_collector.py" --duration "${duration}" \
  --output-dir "${output}/raw_hil" --jetson-host "${JETSON_HOST}" --manager-pid "${manager_pid}" \
  >"${output}/logs/raw_hil_collector.log" 2>&1 &
collector_pid=$!

result="${output}/report/${scenario}_result.json"
set +e
case "${scenario}" in
  straight)
    python3 "${VALIDATION_DIR}/hil_navigation_test.py" --output "${result}" --timeout "${duration}" ; test_status=$? ;;
  semantic)
    python3 "${VALIDATION_DIR}/route_generation_test.py" --output "${output}/report/route_generation.json" --timeout 30 && \
    python3 "${VALIDATION_DIR}/semantic_route_test.py" --output "${result}" \
      --topology-csv "${output}/csv/lane_topology.csv" --timeout 30; test_status=$? ;;
  junction)
    python3 "${VALIDATION_DIR}/junction_test.py" --output "${result}" \
      --route-csv "${output}/csv/junction_routes.csv" --timeout 30; test_status=$? ;;
  cancel)
    python3 "${VALIDATION_DIR}/cancel_route_test.py" --output "${result}" --timeout "${duration}"; test_status=$? ;;
  takeover)
    if [[ "${takeover_mode}" == "test-build-matrix" ]]; then
      if [[ -z "${JETSON_SOC_DIR:-}" ]]; then
        fail "--test-build-matrix 要求显式设置 Jetson 上的 JETSON_SOC_DIR"
        test_status=2
      else
        python3 "${VALIDATION_DIR}/mcu_takeover_test.py" --output "${result}" \
          --mode observe --timeout "${duration}" & observer_pid=$!
        sleep 2
        remote_output="/tmp/phase3_mcu_fault_matrix_$$.json"
        ssh -o BatchMode=yes -o ConnectTimeout=8 "${JETSON_HOST}" \
          "python3 '${JETSON_SOC_DIR}/test/hil/run_mcu_fault_matrix.py' --interface can1 --output '${remote_output}'" \
          >"${output}/logs/mcu_fault_matrix.log" 2>&1
        remote_status=$?
        if (( remote_status == 0 )); then
          scp -q "${JETSON_HOST}:${remote_output}" "${output}/report/mcu_fault_matrix_raw.json"
        fi
        wait "${observer_pid}"; observer_status=$?
        test_status=$(( remote_status != 0 ? remote_status : observer_status ))
      fi
    else
      python3 "${VALIDATION_DIR}/mcu_takeover_test.py" --output "${result}" \
        --mode observe --timeout "${duration}"; test_status=$?
    fi ;;
esac
wait "${logger_pid}"; logger_status=$?
wait "${collector_pid}"; collector_status=$?
set -e

python3 "${ROOT}/tools/hil_logger/analyze_navigation.py" \
  --csv "${csv}" --can-metrics "${output}/raw_hil/can_metrics.csv" \
  --output "${output}/report/metrics.json" --markdown "${output}/report/metrics.md"
python3 "${ROOT}/tools/performance_matlab/navigation/plot_navigation.py" \
  --csv "${csv}" --can-metrics "${output}/raw_hil/can_metrics.csv" --output-dir "${output}/figures"

cp -- "${session_dir}/stack.log" "${output}/logs/stack.log"
cp -- "${session_dir}/bridge.log" "${output}/logs/bridge.log"
stop_managed_session
trap - EXIT INT TERM
if (( test_status != 0 || logger_status != 0 || collector_status != 0 )); then
  fail "验证未通过；真实证据已保留: ${output} (test=${test_status}, logger=${logger_status}, collector=${collector_status})"
  exit 5
fi
pass "Town04 ${scenario} 验证完成: ${output}"
