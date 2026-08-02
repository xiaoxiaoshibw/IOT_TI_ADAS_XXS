#!/usr/bin/env bash
# M3 在线体检两部分：
#   1) AEB 横穿行人场景：采样触发前/后状态，确认触发-避让-恢复链路
#   2) MRM 降级：基准场景运行中 kill object_tracker → safety_monitor MRM_COMFORT
#      → gate 切 builtin_stop → 车停
# 用法（WSL/Ubuntu，已 colcon build 并 source）：bash test/check_aeb_mrm.sh
set -u

echo "===== 第 1 部分：AEB 横穿行人场景 ====="
ros2 launch adas_launch sil_aeb.launch.py > /tmp/sil_aeb_launch.log 2>&1 &
PID1=$!
echo "等待 20s（起步→触发点，AEB 应已触发）..."
sleep 20
echo "--- t≈20s 自车速度（应已明显减速或在恢复中）"
timeout 5 ros2 topic echo /adas/localization/kinematic_state --once --field twist.twist.linear.x 2>/dev/null | head -1
echo "--- AEB 状态"
timeout 5 ros2 topic echo /adas/control/aeb/status --once 2>/dev/null | grep -E "state|ttc_s|reason" | head -4
echo "等待到 35s（行人已通过，应恢复巡航驶过横穿点）..."
sleep 15
echo "--- t≈35s 自车位置 x（应 > 160）与速度"
timeout 5 ros2 topic echo /adas/localization/kinematic_state --once --field pose.pose.position.x 2>/dev/null | head -1
timeout 5 ros2 topic echo /adas/localization/kinematic_state --once --field twist.twist.linear.x 2>/dev/null | head -1
echo "--- AEB 事件日志"
grep -E "AEB (触发|释放)" /tmp/sil_aeb_launch.log | head -6

# 彻底清场：launch 正常退出 + 兜底强杀 + 确认无残留节点（残留 sim 会污染第 2 部分观测）
kill $PID1 2>/dev/null; wait $PID1 2>/dev/null
sleep 5
# 字符类打断 pkill -f 对本脚本自身命令行的自匹配
pkill -9 -f "install/adas[_]" 2>/dev/null
sleep 3
echo "--- 清场后剩余节点（应为空）"
timeout 8 ros2 node list 2>/dev/null

echo ""
echo "===== 第 2 部分：kill object_tracker → MRM 舒适停车 ====="
ros2 launch adas_launch sil.launch.py > /tmp/sil_mrm_launch.log 2>&1 &
PID2=$!
trap 'kill ${PID2} 2>/dev/null; wait ${PID2} 2>/dev/null' EXIT
echo "等待 15s（正常巡航建立）..."
sleep 15
echo "--- kill 前速度"
timeout 5 ros2 topic echo /adas/localization/kinematic_state --once --field twist.twist.linear.x 2>/dev/null | head -1
echo ">>> kill object_tracker_node"
pkill -f object_tracker_node
sleep 3
echo "--- kill+3s: safety_status（overall 应=2 MRM_COMFORT，failed=objects）"
timeout 5 ros2 topic echo /adas/system/safety_status --once 2>/dev/null | grep -E "overall|failed" -A2 | head -5
echo "--- gate 状态（selected_source 应=2 builtin_stop）"
timeout 5 ros2 topic echo /adas/control/gate/status --once --field selected_source 2>/dev/null | head -1
echo "等待 10s（舒适停车完成）..."
sleep 10
echo "--- 最终速度（应≈0）"
timeout 5 ros2 topic echo /adas/localization/kinematic_state --once --field twist.twist.linear.x 2>/dev/null | head -1
echo "--- 安全事件日志"
grep -E "安全级别|切源" /tmp/sil_mrm_launch.log | head -6
echo "===== 体检结束 ====="
