#!/usr/bin/env bash
# ACC 在线闭环体检：起 sil_acc 场景 → 三个时间窗采样（稳态跟车 / 前车刹停 / 重起步）
# 用法（WSL/Ubuntu，已 colcon build）：
#   source /opt/ros/jazzy/setup.bash && source <ws>/install/setup.bash
#   bash test/check_acc.sh
set -u

echo "=== 启动 SIL ACC 场景 ==="
ros2 launch adas_launch sil_acc.launch.py > /tmp/sil_acc_launch.log 2>&1 &
LAUNCH_PID=$!
trap 'kill ${LAUNCH_PID} 2>/dev/null; wait ${LAUNCH_PID} 2>/dev/null' EXIT

sample() {
  echo "--- t≈$1s: 自车速度 / 主前车 gap / 行为状态"
  timeout 5 ros2 topic echo /adas/localization/kinematic_state --once --field twist.twist.linear.x 2>/dev/null | head -1
  timeout 5 ros2 topic echo /adas/perception/objects --once --field primary_lead_gap_m 2>/dev/null | head -1
  timeout 5 ros2 topic echo /adas/planning/behavior --once --field state 2>/dev/null | head -1
}

echo "等待 30s（起步 + 追上前车进入稳态跟车）..."
sleep 30
sample 30

echo "等待到 47s（前车已刹停，自车应停稳）..."
sleep 17
sample 47

echo "等待到 68s（前车重起步，自车应跟随）..."
sleep 21
sample 68

echo "=== launch 日志尾部（行为切换/主前车事件）==="
grep -E "行为切换|主前车|切源" /tmp/sil_acc_launch.log | tail -10
echo "=== 体检结束 ==="
