#!/usr/bin/env bash
# M5 在线体检：双栈冗余——kill 主栈规划器 → 仲裁器无缝切备栈，车辆持续巡航
# 用法（WSL/Ubuntu，已 colcon build 并 source）：bash test/check_redundant.sh
set -u

echo "=== 启动双栈冗余（17 进程）==="
ros2 launch adas_launch sil_redundant.launch.py > /tmp/sil_red_launch.log 2>&1 &
LAUNCH_PID=$!
trap 'kill ${LAUNCH_PID} 2>/dev/null; wait ${LAUNCH_PID} 2>/dev/null' EXIT

echo "等待 30s（16 节点激活链 + 巡航建立）..."
sleep 30
echo "--- 激活链尾部"
grep "\[lifecycle\]" /tmp/sil_red_launch.log | tail -2
echo "--- kill 前速度（应 ≈15）"
timeout 5 ros2 topic echo /adas/localization/kinematic_state --once --field twist.twist.linear.x 2>/dev/null | head -1
echo "--- 全局下发流频率（仲裁器输出，应 ~100Hz）"
timeout 4 ros2 topic hz /adas/control/gate/control_cmd 2>&1 | tail -1

echo ">>> kill 主栈 trajectory_planner"
pkill -f "trajectory_planner_node.*__ns:=/primary"
sleep 5
echo "--- kill+5s 速度（应仍 ≈15：备栈无缝接管）"
timeout 5 ros2 topic echo /adas/localization/kinematic_state --once --field twist.twist.linear.x 2>/dev/null | head -1
echo "--- 主栈 gate 状态（应=2 builtin_stop 降级）/ 备栈 gate 状态（应=0 正常）"
timeout 5 ros2 topic echo /primary/adas/control/gate/status --once --field selected_source 2>/dev/null | head -1
timeout 5 ros2 topic echo /backup/adas/control/gate/status --once --field selected_source 2>/dev/null | head -1

echo "等待 10s 再确认持续行驶..."
sleep 10
echo "--- kill+15s 速度（应仍 ≈15）与位置"
timeout 5 ros2 topic echo /adas/localization/kinematic_state --once --field twist.twist.linear.x 2>/dev/null | head -1
timeout 5 ros2 topic echo /adas/localization/kinematic_state --once --field pose.pose.position.x 2>/dev/null | head -1
echo "--- 冗余切换日志"
grep -E "冗余切换" /tmp/sil_red_launch.log | head -4
echo "=== 体检结束 ==="
