#!/usr/bin/env bash
set -Eeuo pipefail
HIL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=adas_pc/tools/hil/hil_common.sh
source "${HIL_DIR}/hil_common.sh"

failures=0
check_file() {
  if [[ -e "$1" ]]; then pass "$2: $1"; else fail "$2 缺失: $1" || true; failures=$((failures + 1)); fi
}
check_cmd() {
  if command -v "$1" >/dev/null 2>&1; then pass "命令 $1"; else fail "命令缺失: $1" || true; failures=$((failures + 1)); fi
}

check_file "/opt/ros/jazzy/setup.bash" "ROS2 Jazzy"
check_file "${ADAS_PC_DIR}/carla_ros2_bridge/ws/install/setup.bash" "Bridge 工作区"
check_file "${HOME}/CARLA_0.9.16/CarlaUE4.sh" "CARLA 0.9.16"
for command_name in python3 ros2 ssh ss candump; do check_cmd "${command_name}"; done

load_pc_ros
if [[ "${ROS_DOMAIN_ID}" == "43" ]]; then pass "ROS_DOMAIN_ID=43"; else fail "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" || true; failures=$((failures + 1)); fi
if [[ "${RMW_IMPLEMENTATION}" == "rmw_cyclonedds_cpp" ]]; then pass "RMW=${RMW_IMPLEMENTATION}"; else warn "RMW=${RMW_IMPLEMENTATION}"; fi
if [[ "${CYCLONEDDS_URI}" == *'enx00e04c176a70'* && "${CYCLONEDDS_URI}" == *'192.168.100.32'* ]]; then
  pass "DDS 固定直连网卡与 Jetson peer"
else
  fail "CYCLONEDDS_URI 未绑定直连链路" || true
  failures=$((failures + 1))
fi
python3 -c 'import carla; print("PASS  CARLA Python API", carla.__file__)' \
  || { fail "CARLA Python API 不可导入" || true; failures=$((failures + 1)); }

if ssh -o BatchMode=yes -o ConnectTimeout=8 "${JETSON_HOST}" true; then
  pass "Jetson SSH"
else
  fail "Jetson SSH 不可达" || true
  failures=$((failures + 1))
fi

remote=$(ssh -o BatchMode=yes -o ConnectTimeout=8 "${JETSON_HOST}" '
  printf "hil=%s can_service=%s\\n" "$(systemctl is-active adas-hil)" "$(systemctl is-active adas-can)"
  ip -details -statistics link show can1
  timeout 2 candump -L can1,201:7FF | head -n 1
' 2>&1 || true)
printf '%s\n' "${remote}"
if [[ "${remote}" == *'hil=active can_service=active'* ]]; then pass "Jetson HIL/CAN services active"; else fail "Jetson 服务未全部 active" || true; failures=$((failures + 1)); fi
if [[ "${remote}" == *'state ERROR-ACTIVE'* && "${remote}" == *'bitrate 500000'* ]]; then pass "can1 ERROR-ACTIVE @500k"; else fail "can1 状态/速率异常" || true; failures=$((failures + 1)); fi
if [[ "${remote}" == *'can1 201#'* ]]; then pass "MCU 0x201 心跳存在"; else fail "2 秒内未见 MCU 0x201" || true; failures=$((failures + 1)); fi

space_kb=$(df -Pk "${PROJECT_ROOT}" | awk 'NR==2 {print $4}')
if (( space_kb >= 1048576 )); then pass "磁盘可用 $((space_kb / 1024)) MiB"; else warn "磁盘不足 1 GiB"; fi
probe="${PROJECT_ROOT}/evidence/artifacts/.hil_write_probe.$$"
if : >"${probe}" && rm -f -- "${probe}"; then
  pass "evidence/artifacts 可写"
else
  fail "evidence/artifacts 不可写" || true
  failures=$((failures + 1))
fi

(( failures == 0 )) || exit 1
pass "HIL preflight 完成"
