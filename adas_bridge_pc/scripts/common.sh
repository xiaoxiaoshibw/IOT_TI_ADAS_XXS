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
# HIL 主机是多网卡环境，单靠 DDS 组播会出现非对称发现。旧实现硬编码
# enx00e04c176a70；USB 网卡重枚举/拔出后 CycloneDDS 会在创建首个节点时直接
# 失败，连本机 CARLA GUI 都无法启动。仅当当前确有 192.168.100.x 网卡时绑定；
# ADAS_DDS_IFACE 仍可显式指定，但同样要求该接口当前存在。
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
  dds_iface="${ADAS_DDS_IFACE:-}"
  if [[ -n "${dds_iface}" ]] && ! ip link show dev "${dds_iface}" >/dev/null 2>&1; then
    dds_iface=""
  fi
  if [[ -z "${dds_iface}" ]]; then
    dds_iface="$(ip -o -4 addr show 2>/dev/null | awk '$4 ~ /^192\.168\.100\./ {print $2; exit}')"
  fi
  if [[ -n "${dds_iface}" ]]; then
    dds_peer="${ADAS_DDS_PEER:-192.168.100.32}"
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${dds_iface}\"/></Interfaces></General><Discovery><Peers><Peer Address=\"${dds_peer}\"/></Peers></Discovery></Domain></CycloneDDS>"
  fi
  unset dds_iface dds_peer
fi

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
  local root="${CARLA_ROOT:-}"
  if [[ -z "${root}" ]]; then
    local candidate
    for candidate in "${HOME}/CARLA_0.9.16" "${HOME}/程序/CARLA_0.9.16"; do
      if [[ -x "${candidate}/CarlaUE4.sh" ]]; then
        root="${candidate}"
        break
      fi
    done
  fi
  root="${root:-${HOME}/CARLA_0.9.16}"
  [[ -x "${root}/CarlaUE4.sh" ]] || fail "Set CARLA_ROOT to the CARLA 0.9.16 directory (missing ${root}/CarlaUE4.sh)."
  printf '%s\n' "${root}/CarlaUE4.sh"
}
