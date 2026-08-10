#!/usr/bin/env bash
# Start CARLA. Configure CARLA_ROOT; override CARLA_ARGS when needed.
#
# Phase 1 hardening：所有 CARLA 进程调用统一收敛到 carla_invoker.py。
# invoker 自己持 flock + RPC readiness 探测 + 进程组守护。
# 这里只是 invoker 的薄包装,保留原 start_carla.sh 的命令行兼容性
# （CARLA_ARGS / TOWN / CARLA_PORT 环境变量继续生效）。
#
# 用法：
#   start_carla.sh                       # 用默认 quality=Epic + TOWN 启动
#   CARLA_ARGS="-quality-level=Low" start_carla.sh
#
# 回退开关：环境变量 ADAS_LEGACY_CARLA=1 走旧的 CarlaUE4.sh 直接路径
# （用于 invoker 自身出问题时临时绕开）。

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INVOKER="${REPO_ROOT}/scripts/carla_invoker.py"

# 解析 CARLA_ARGS 为 invoker 的 --quality-level
QUALITY_LEVEL="Epic"
if [[ -n "${CARLA_ARGS:-}" ]]; then
  read -r -a _carla_args <<< "${CARLA_ARGS}"
  for ((i = 0; i < ${#_carla_args[@]}; i++)); do
    case "${_carla_args[$i]}" in
      -quality-level=*)  QUALITY_LEVEL="${_carla_args[$i]#*=}" ;;
      -quality-level)
        if [[ $((i + 1)) -lt ${#_carla_args[@]} ]]; then
          QUALITY_LEVEL="${_carla_args[$((i + 1))]}"
        fi ;;
    esac
  done
fi

if [[ "${ADAS_LEGACY_CARLA:-0}" == "1" ]]; then
  # 回退到旧的直接路径（仅 dev / debug 使用）
  source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/common.sh"
  exec "$(carla_executable)" ${CARLA_ARGS:-} "$@"
fi

if [[ ! -f "${INVOKER}" ]]; then
  echo "Error: ${INVOKER} 不存在" >&2
  exit 1
fi

exec python3 "${INVOKER}" start \
  --quality-level "${QUALITY_LEVEL}" \
  --town "${TOWN:-Town04}" \
  --port "${CARLA_PORT:-2000}" \
  --host "${CARLA_HOST:-127.0.0.1}" \
  --timeout "${CARLA_STARTUP_TIMEOUT:-60}" \
  "$@"
