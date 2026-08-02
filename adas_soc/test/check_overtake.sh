#!/usr/bin/env bash
# M4 在线体检：慢车超越场景——变道中/完成后两窗采样 + 行为切换日志
# 用法（WSL/Ubuntu，已 colcon build 并 source）：bash test/check_overtake.sh
set -u

echo "=== 启动 SIL 超越场景 ==="
ros2 launch adas_launch sil_overtake.launch.py > /tmp/sil_overtake_launch.log 2>&1 &
LAUNCH_PID=$!
trap 'kill ${LAUNCH_PID} 2>/dev/null; wait ${LAUNCH_PID} 2>/dev/null' EXIT

echo "等待 12s（应处于变道/超越中）..."
sleep 12
echo "--- t≈12s 行为状态（期望 3=ACTIVE 或 4=RETURN）/ 车道横向（应明显偏左 >1m）"
timeout 5 ros2 topic echo /adas/planning/behavior --once --field state 2>/dev/null | head -1
timeout 5 ros2 topic echo /adas/perception/lane_state --once --field lateral_offset 2>/dev/null | head -1

echo "等待到 30s（应已超越回线巡航）..."
sleep 18
echo "--- t≈30s 行为状态（期望 0=LANE_FOLLOW）/ 横向（≈0）/ 车速（≈15）/ 位置"
timeout 5 ros2 topic echo /adas/planning/behavior --once --field state 2>/dev/null | head -1
timeout 5 ros2 topic echo /adas/perception/lane_state --once --field lateral_offset 2>/dev/null | head -1
timeout 5 ros2 topic echo /adas/localization/kinematic_state --once --field twist.twist.linear.x 2>/dev/null | head -1
timeout 5 ros2 topic echo /adas/localization/kinematic_state --once --field pose.pose.position.x 2>/dev/null | head -1

echo "--- 行为切换日志（期望 0→1→2→3→4→0）"
grep -E "行为切换" /tmp/sil_overtake_launch.log | head -8
echo "--- AEB 事件（期望为空）"
grep -E "AEB 触发" /tmp/sil_overtake_launch.log | head -3
echo "=== 体检结束 ==="
