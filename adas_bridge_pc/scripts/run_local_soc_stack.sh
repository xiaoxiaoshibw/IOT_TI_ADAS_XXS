#!/usr/bin/env bash
# GUI-owned CARLA + local SoC control stack. CARLA and its bridge are owned by
# ProcessManager; this script owns only the SoC lifecycle nodes.
#
# P0.C: 协议 run_id 必须在 GUI 启动一次,这里只做透传,不能自动生成。
# 缺少 --run-id 或不是规范 UUID v4（小写、version 4、variant 8/9/a/b）
# 时直接 fail-closed,避免分机拓扑里两端各自生成 ID。
set -Eeuo pipefail

scenario_id="free"
run_id=""
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
    --run-id)
      [[ $# -ge 2 && -n "$2" ]] || {
        echo "--run-id requires a non-empty session id" >&2
        exit 2
      }
      run_id="$2"
      shift 2
      ;;
    --help|-h)
      echo "usage: $0 --run-id UUID_V4 [--scenario CATALOG_ID]"
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

# P0.C: 严格校验 UUID v4 小写形式。空 / 大写 / 非法版本一律 fail-closed。
UUID_V4_RE='^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$'
if [[ -z "${run_id}" ]]; then
  echo "--run-id 是必填项;必须由 GUI 注入规范 UUID v4,本脚本不再自动生成" >&2
  exit 2
fi
if [[ ! "${run_id}" =~ ${UUID_V4_RE} ]]; then
  echo "--run-id 必须是规范 UUID v4（小写、version 4、variant 8/9/a/b），收到: ${run_id}" >&2
  exit 2
fi

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${SOC_SETUP}"
set -u

exec ros2 launch adas_launch carla.launch.py \
  "scenario_id:=${scenario_id}" "run_id:=${run_id}"
