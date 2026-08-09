#!/usr/bin/env bash
# 比赛日 PC SIL 兜底入口：不依赖 CARLA、Orin、F280025C 或 CAN 设备。
#
# 例：
#   ./scripts/run_sil_fallback.sh --build --check
#   ./scripts/run_sil_fallback.sh --scenario aeb
#
# 默认启动基准 SIL 并保持运行，Ctrl-C 正常收尾。--check 会启动、检查
# 关键话题心跳和执行输出，再自动退出，适合赛前验收/现场快速确认。
set -Eeuo pipefail
IFS=$'\n\t'

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOC_WS="${ROOT}/adas_soc"
LOG_ROOT="${SIL_LOG_ROOT:-${ROOT}/logs/sil}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

BUILD=0
CHECK=0
HOST_TESTS=0
SCENARIO="baseline"
DURATION=0
READY_TIMEOUT="${SIL_READY_TIMEOUT:-45}"

usage() {
  sed -n '2,15p' "${BASH_SOURCE[0]}"
  cat <<'EOF'

选项：
  --build                 重新构建 adas_soc（首次运行会自动构建）
  --host-tests            先运行 adas_mcu 的 GCC 主机回归测试
  --check                 启动后检查关键话题/执行流，完成后自动退出
  --scenario NAME         baseline|acc|aeb|overtake|redundant|lqr
  --duration SEC          运行指定秒数后退出；默认 0 表示持续运行
  -h, --help              显示帮助

SIL 默认使用 ROS_DOMAIN_ID=145 与真实 HIL/GUI 隔离；需要让其它工具接入
这个 SIL 时，请显式使用同一个 ROS_DOMAIN_ID。
EOF
}

while (($#)); do
  case "$1" in
    --build) BUILD=1 ;;
    --host-tests) HOST_TESTS=1 ;;
    --check) CHECK=1 ;;
    --scenario)
      (($# >= 2)) || { echo "--scenario 缺少参数" >&2; exit 2; }
      SCENARIO="$2"
      shift
      ;;
    --duration)
      (($# >= 2)) || { echo "--duration 缺少参数" >&2; exit 2; }
      DURATION="$2"
      shift
      ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

[[ -d "${SOC_WS}/src" ]] || { echo "找不到 SoC 工作区：${SOC_WS}" >&2; exit 1; }
[[ "${DURATION}" =~ ^[0-9]+$ ]] || { echo "--duration 必须是非负整数" >&2; exit 2; }
[[ "${READY_TIMEOUT}" =~ ^[0-9]+$ ]] || { echo "SIL_READY_TIMEOUT 必须是非负整数" >&2; exit 2; }

case "${SCENARIO}" in
  baseline)  LAUNCH_FILE="sil.launch.py" ;;
  acc)       LAUNCH_FILE="sil_acc.launch.py" ;;
  aeb)       LAUNCH_FILE="sil_aeb.launch.py" ;;
  overtake)  LAUNCH_FILE="sil_overtake.launch.py" ;;
  redundant) LAUNCH_FILE="sil_redundant.launch.py" ;;
  lqr)       LAUNCH_FILE="sil_lqr.launch.py" ;;
  *) echo "不支持的场景：${SCENARIO}" >&2; exit 2 ;;
esac

if (( CHECK && DURATION == 0 )); then
  DURATION=8
fi

# 使用独立 Domain，防止比赛现场残留的 HIL/GUI 节点污染 SIL 观测。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-145}"

source_ros() {
  [[ -f "${ROS_SETUP}" ]] || {
    echo "找不到 ROS 2 setup：${ROS_SETUP}" >&2
    return 1
  }
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  set -u
}

source_workspace() {
  source_ros
  [[ -f "${SOC_WS}/install/setup.bash" ]] || return 1
  set +u
  # shellcheck disable=SC1091
  source "${SOC_WS}/install/setup.bash"
  set -u
}

if (( HOST_TESTS )); then
  echo "=== MCU host SIL tests ==="
  bash "${ROOT}/adas_mcu/tests/run_host_tests.sh"
fi

if (( BUILD )) || [[ ! -f "${SOC_WS}/install/setup.bash" ]]; then
  source_ros
  echo "=== build adas_soc ==="
  (cd "${SOC_WS}" && colcon build --symlink-install --event-handlers console_direct+)
fi

source_workspace
mkdir -p "${LOG_ROOT}"
RUN_ID="$(date +%Y%m%d_%H%M%S)_$$"
LAUNCH_LOG="${LOG_ROOT}/${RUN_ID}_${SCENARIO}.log"
LAUNCH_PID=""

stop_launch() {
  local pid="${LAUNCH_PID}"
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

trap 'exit 130' INT TERM
trap 'status=$?; stop_launch; exit "${status}"' EXIT

echo "=== start SIL ==="
echo "scenario=${SCENARIO} launch=${LAUNCH_FILE} ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "log=${LAUNCH_LOG}"
setsid ros2 launch adas_launch "${LAUNCH_FILE}" >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

required_topics=(
  /adas/localization/kinematic_state
  /adas/perception/lane_state
  /adas/vehicle/actuation_cmd
)
if [[ "${SCENARIO}" == redundant ]]; then
  required_topics+=(
    /primary/adas/control/gate/status
    /backup/adas/control/gate/status
  )
else
  required_topics+=(/adas/control/gate/status)
fi

echo "=== wait SIL topics (${READY_TIMEOUT}s) ==="
deadline=$((SECONDS + READY_TIMEOUT))
ready=0
while (( SECONDS < deadline )); do
  kill -0 "${LAUNCH_PID}" 2>/dev/null || {
    echo "SIL launch 提前退出，日志尾部：" >&2
    tail -40 "${LAUNCH_LOG}" >&2 || true
    exit 1
  }
  topic_list="$(ros2 topic list 2>/dev/null || true)"
  ready=1
  for topic in "${required_topics[@]}"; do
    grep -qx "${topic}" <<<"${topic_list}" || ready=0
  done
  if (( ready )); then
    break
  fi
  sleep 1
done

if (( ! ready )); then
  echo "SIL 关键话题未在 ${READY_TIMEOUT}s 内就绪，日志尾部：" >&2
  tail -60 "${LAUNCH_LOG}" >&2 || true
  exit 1
fi

echo "SIL topics ready"
if (( CHECK )); then
  echo "=== sample closed-loop topics ==="
  for topic in "${required_topics[@]}"; do
    echo "--- ${topic}"
    timeout 8 ros2 topic echo "${topic}" --once || {
      echo "话题无有效消息：${topic}" >&2
      exit 1
    }
  done

  hz_output="$(timeout 6 ros2 topic hz /adas/vehicle/actuation_cmd 2>&1 || true)"
  grep -q "average rate" <<<"${hz_output}" || {
    echo "执行输出没有形成稳定心跳" >&2
    echo "${hz_output}" >&2
    exit 1
  }
  echo "actuation_cmd heartbeat: PASS"
fi

if (( DURATION > 0 )); then
  echo "SIL running for ${DURATION}s..."
  sleep "${DURATION}"
  echo "SIL duration complete"
else
  echo "SIL running; press Ctrl-C to stop"
  wait "${LAUNCH_PID}"
fi

echo "=== SIL PASS ==="
