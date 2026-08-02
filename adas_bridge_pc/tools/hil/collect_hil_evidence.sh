#!/usr/bin/env bash
set -Eeuo pipefail
HIL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=adas_pc/tools/hil/hil_common.sh
source "${HIL_DIR}/hil_common.sh"
out="${1:-$(new_artifact_root)}"
mkdir -p "${out}"/{environment,carla,ros2_pc,ros2_jetson,can,mcu,vehicle,screenshots}

git -C "${PROJECT_ROOT}" rev-parse HEAD >"${out}/environment/git_commit.txt"
git -C "${PROJECT_ROOT}" status --short >"${out}/environment/git_status.txt"
git -C "${PROJECT_ROOT}" diff --stat >"${out}/environment/git_diff_stat.txt"
uname -a >"${out}/environment/uname.txt"
printenv | sort | grep -E '^(ROS_|RMW_|CYCLONEDDS|FASTDDS|DOMAIN)' >"${out}/environment/ros_environment.txt" || true
pgrep -af 'CarlaUE4|bridge_node|adas_gui' >"${out}/environment/processes.txt" || true
ss -lntp >"${out}/environment/listening_ports.txt"

load_pc_ros
ros2 node list >"${out}/ros2_pc/nodes.txt" 2>&1 || true
ros2 topic list -t >"${out}/ros2_pc/topics.txt" 2>&1 || true
timeout 5 ros2 topic hz /adas/localization/kinematic_state >"${out}/ros2_pc/odom_hz.txt" 2>&1 || true

ssh -o BatchMode=yes -o ConnectTimeout=8 "${JETSON_HOST}" '
  set -a; source /etc/adas/adas-hil.env; set +a
  source "/opt/ros/${ADAS_ROS_DISTRO}/setup.bash"
  source "${ADAS_INSTALL_PREFIX}/setup.bash"
  export ROS_DOMAIN_ID=43
  systemctl is-active adas-hil adas-can
  ros2 node list
  ros2 topic list -t
' >"${out}/ros2_jetson/topology.txt" 2>&1 || true
ssh -o BatchMode=yes -o ConnectTimeout=8 "${JETSON_HOST}" 'ip -details -statistics link show can1' \
  >"${out}/can/can1_stats.txt" 2>&1 || true
ssh -o BatchMode=yes -o ConnectTimeout=8 "${JETSON_HOST}" \
  'timeout 5 candump -L can1,100:7FF,101:7FF,102:7FF,103:7FF,104:7FF,201:7FF,202:7FF,203:7FF,204:7FF,206:7FF' \
  >"${out}/can/candump.log" 2>&1 || true

cat >"${out}/summary.md" <<EOF
# HIL evidence

- Collected: $(date --iso-8601=seconds)
- Project: ${PROJECT_ROOT}
- Runtime gate: run \`adas_pc/tools/hil/check_hil_runtime.sh --session-dir ${out}\`
- Raw CAN: \`can/candump.log\`
- PC/Jetson topology: \`ros2_pc/\`, \`ros2_jetson/\`
EOF
printf 'Evidence: %s\n' "${out}"
