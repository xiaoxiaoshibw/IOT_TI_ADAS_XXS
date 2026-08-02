#!/usr/bin/env bash
# WSL 演示：合成地图 + 全局规划器 + Qt6 GUI（WSLg 窗口）。
# 用法: bash Ubutu上位机/carla_ros2_bridge/ws/src/adas_gui/demo/run_demo.sh
set -e
source /opt/ros/jazzy/setup.bash
DEMO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 默认用同仓库的 PC 侧 SoC 工作区（SIL/Jazzy），可用 ADAS_SOC_WS 覆盖
ADAS_SOC_WS="${ADAS_SOC_WS:-$(cd "${DEMO_DIR}/../../../../../.." && pwd)/ADAS0.0.2/SoC}"
if [[ ! -f "${ADAS_SOC_WS}/install/setup.bash" || ! -f "${ADAS_SOC_WS}/src/launch/adas_launch/config/global_planner.yaml" ]]; then
  echo "需要已构建的 SoC 工作区；请设置 ADAS_SOC_WS，例如：export ADAS_SOC_WS=/path/to/ADAS0.0.2/SoC" >&2
  exit 1
fi
source "${ADAS_SOC_WS}/install/setup.bash"

ros2 run adas_global_planner global_planner_node --ros-args \
  --params-file "${ADAS_SOC_WS}/src/launch/adas_launch/config/global_planner.yaml" &
PLANNER=$!
python3 "${DEMO_DIR}/demo_map_publisher.py" &
PUBLISHER=$!
trap 'kill ${PLANNER} ${PUBLISHER} 2>/dev/null || true' EXIT

# 前台跑 GUI；关窗即退出并清理后台进程
ros2 run adas_gui adas_gui
