#!/usr/bin/env bash
# Shared environment setup for the root-level helper scripts.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${REPO_ROOT}/carla_ros2_bridge/ws"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
# HIL 默认地图：单一事实源。start_carla.sh 据此让 CARLA 直接以该图启动，
# start_bridge.sh 的 --town 也取它，避免 CARLA 默认 Town10HD 起来后桥再做
# 一次 Town10HD→TOWN 的世界重载（carla_world.py 已带"已是目标图则跳过"守卫）。
export TOWN="${TOWN:-Town04}"
# HIL 主机是多网卡环境，单靠 DDS 组播会出现 PC 可见 Orin、Orin 却看不到
# PC 发布端的非对称发现。固定直连网卡并加入 Orin 单播 peer。
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"enx00e04c176a70\"/></Interfaces></General><Discovery><Peers><Peer Address=\"192.168.100.32\"/></Peers></Discovery></Domain></CycloneDDS>}"

fail() {
  printf 'Error: %s\n' "$*" >&2
  exit 1
}

source_ros() {
  [[ -f "${ROS_SETUP}" ]] || fail "ROS 2 ${ROS_DISTRO} was not found at ${ROS_SETUP}."
  # ROS 的 setup.bash 内部会引用未定义变量（如 AMENT_TRACE_SETUP_FILES），
  # 与本脚本的 set -u 冲突，source 期间临时关闭 nounset。
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  set -u
}

source_workspace() {
  source_ros
  [[ -f "${WORKSPACE}/install/setup.bash" ]] || fail "Workspace is not built. Run ./build.sh first."
  set +u
  # shellcheck disable=SC1091
  source "${WORKSPACE}/install/setup.bash"
  set -u
}

carla_executable() {
  local root="${CARLA_ROOT:-${HOME}/CARLA_0.9.16}"
  [[ -x "${root}/CarlaUE4.sh" ]] || fail "Set CARLA_ROOT to the CARLA 0.9.16 directory (missing ${root}/CarlaUE4.sh)."
  printf '%s\n' "${root}/CarlaUE4.sh"
}
