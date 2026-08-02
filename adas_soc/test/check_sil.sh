#!/usr/bin/env bash
# SIL 在线闭环快速体检：起全栈 → 等收敛 → 采样关键 topic → 收尾
# 用法（WSL/Ubuntu，已 colcon build）：
#   source /opt/ros/jazzy/setup.bash && source <ws>/install/setup.bash
#   bash test/check_sil.sh
set -u

echo "=== 启动 SIL 全栈 ==="
ros2 launch adas_launch sil.launch.py > /tmp/sil_launch.log 2>&1 &
LAUNCH_PID=$!
trap 'kill ${LAUNCH_PID} 2>/dev/null; wait ${LAUNCH_PID} 2>/dev/null' EXIT

echo "等待 25s（起步 + 收敛）..."
sleep 25

echo "=== topic 清单 ==="
timeout 5 ros2 topic list

echo "=== actuation_cmd 频率（应 ~50Hz）==="
timeout 6 ros2 topic hz /adas/vehicle/actuation_cmd 2>&1 | tail -3

echo "=== 车道状态（lateral_offset 应接近 0）==="
timeout 5 ros2 topic echo /adas/perception/lane_state --once

echo "=== 车速（应接近巡航 15m/s）==="
timeout 5 ros2 topic echo /adas/localization/kinematic_state --once --field twist.twist.linear.x

echo "=== gate 状态（selected_source 应为 0=follower）==="
timeout 5 ros2 topic echo /adas/control/gate/status --once

echo "=== launch 日志尾部 ==="
tail -20 /tmp/sil_launch.log
echo "=== 体检结束 ==="
