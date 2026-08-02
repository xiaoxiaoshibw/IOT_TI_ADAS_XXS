#!/usr/bin/env bash
# Start CARLA, the Qt6 GUI, and the PC bridge together, then gate on
# check_hil_ready.py before declaring the HIL stack usable for testing.
# Ctrl-C stops everything (CARLA/GUI/bridge + the watchdog/monitor pair).

set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/scripts/common.sh"
# check_hil_ready.py/topic_monitor.py 用 rclpy 订阅 adas_msgs，本 shell 也要
# 直接跑 `ros2 topic echo`，所以这里要把工作区环境 source 进当前 shell
# （不仅仅是子脚本各自 source 一份）。
source_workspace

SESSION_DIR="${REPO_ROOT}/logs/hil_run_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${SESSION_DIR}"
printf 'HIL session log dir: %s\n' "${SESSION_DIR}"

# 按进程组终止：CARLA/桥接/GUI 都是 `cmd &` 拉起的，$! 拿到的是该 job 的进程
# 组组长 pid，但 CarlaUE4.sh/ros2 run 内部还会派生真正干活的子进程——只
# kill 组长有时候杀不干净（组长已退出、子进程残留占用端口 2000），必须对整
# 个进程组发信号。SIGTERM 给 2.5s 走优雅退出，超时再 SIGKILL。这里和
# carla_watchdog.py::stop_process_group() 的两段式退出保持一致。
stop_pgroup() {
  local pid="$1" label="$2"
  [[ -z "${pid}" ]] && return 0
  local pgid
  pgid=$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')
  [[ -z "${pgid}" ]] && return 0
  kill -TERM -"${pgid}" 2>/dev/null || true
  for _ in $(seq 1 25); do
    kill -0 -"${pgid}" 2>/dev/null || return 0
    sleep 0.1
  done
  printf '%s: 2.5s 内未退出，SIGKILL 进程组 %s\n' "${label}" "${pgid}" >&2
  kill -KILL -"${pgid}" 2>/dev/null || true
}

cleanup() {
  trap - EXIT INT TERM
  # 先停应用层（watchdog/monitor/echo/bridge/gui），最后才停 CARLA——避免
  # 桥接在 CARLA 已经没了的情况下反复重连报错刷屏。
  stop_pgroup "${watchdog_pid:-}" "carla_watchdog"
  stop_pgroup "${monitor_pid:-}" "topic_monitor"
  stop_pgroup "${safety_echo_pid:-}" "safety_echo"
  stop_pgroup "${bridge_pid:-}" "bridge"
  stop_pgroup "${gui_pid:-}" "gui"
  stop_pgroup "${carla_pid:-}" "carla"
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

carla_port="${CARLA_PORT:-2000}"
timeout_s="${CARLA_STARTUP_TIMEOUT:-60}"

# 先探测端口是不是已经被"别人的"CARLA占着——避免重复拉起第二份实例去抢
# 同一个仿真世界的 synchronous_mode 控制权（历史事故：两个独立客户端各自
# 认为自己是唯一的 tick 发起者，互相踩，其中一个的 cleanup 还会把对方连带
# 杀掉）。检测到已有实例就直接复用，不写 carla.pid——cleanup 不会碰它，
# 它的生死由启动它的那个会话负责，不归这次 start_pc_stack.sh 管。
if (exec 3<>"/dev/tcp/127.0.0.1/${carla_port}") 2>/dev/null; then
  exec 3>&- 3<&-
  printf 'CARLA 端口 %s 已被占用，视为外部已运行的实例，不重复拉起、也不会去停它。\n' \
    "${carla_port}"
  printf '如果这不是你期望的（比如上一次没退干净），先手动确认/清理再重跑。\n'
  carla_pid=""
else
  "${REPO_ROOT}/start_carla.sh" > "${SESSION_DIR}/carla.log" 2>&1 &
  carla_pid=$!
  echo "${carla_pid}" > "${SESSION_DIR}/carla.pid"
  # 等 RPC 端口(2000)可连接才算就绪，替代固定 sleep；超时按失败退出。
  # 注意：端口可连接只是必要条件，不是充分条件——carla_watchdog.py/
  # check_hil_ready.py 会用真正的 RPC 调用（get_world()）复核。
  printf 'CARLA started (pid %s). Waiting for port %s (timeout %ss)...\n' \
    "${carla_pid}" "${carla_port}" "${timeout_s}"
  for ((i = 0; i < timeout_s; i++)); do
    if (exec 3<>"/dev/tcp/127.0.0.1/${carla_port}") 2>/dev/null; then
      exec 3>&- 3<&-
      printf 'CARLA port %s ready after %ss.\n' "${carla_port}" "${i}"
      break
    fi
    kill -0 "${carla_pid}" 2>/dev/null || fail "CARLA exited during startup."
    sleep 1
  done
  (exec 3<>"/dev/tcp/127.0.0.1/${carla_port}") 2>/dev/null \
    || fail "CARLA port ${carla_port} not ready after ${timeout_s}s."
  exec 3>&- 3<&- 2>/dev/null || true
fi

"${REPO_ROOT}/start_gui.sh" > "${SESSION_DIR}/gui.log" 2>&1 &
gui_pid=$!
echo "${gui_pid}" > "${SESSION_DIR}/gui.pid"
printf 'Qt6 GUI started (pid %s). ROS_DOMAIN_ID=%s\n' "${gui_pid}" "${ROS_DOMAIN_ID}"

"${REPO_ROOT}/start_bridge.sh" "$@" > "${SESSION_DIR}/bridge.log" 2>&1 &
bridge_pid=$!
echo "${bridge_pid}" > "${SESSION_DIR}/bridge.pid"
printf 'Bridge started (pid %s).\n' "${bridge_pid}"

printf '\nRunning HIL health gate (check_hil_ready.py)...\n'
if ! python3 "${REPO_ROOT}/scripts/check_hil_ready.py" --session-dir "${SESSION_DIR}"; then
  printf '\nHIL health gate FAILED — 禁止测试。上面列出的故障码需要先处理。\n'
  printf 'Logs kept at: %s\n' "${SESSION_DIR}"
  exit 1
fi

# 门禁通过后才拉起常驻的看门狗 + 话题监控，采集本次会话的 ros2 topic echo 记录。
python3 "${REPO_ROOT}/scripts/carla_watchdog.py" --session-dir "${SESSION_DIR}" \
  > "${SESSION_DIR}/carla_watchdog_stdout.log" 2>&1 &
watchdog_pid=$!
echo "${watchdog_pid}" > "${SESSION_DIR}/carla_watchdog.pid"

python3 "${REPO_ROOT}/scripts/topic_monitor.py" --session-dir "${SESSION_DIR}" \
  > "${SESSION_DIR}/ros2_topic.log" 2>&1 &
monitor_pid=$!
echo "${monitor_pid}" > "${SESSION_DIR}/topic_monitor.pid"

ros2 topic echo /adas/system/safety_status > "${SESSION_DIR}/safety_monitor.log" 2>&1 &
safety_echo_pid=$!
echo "${safety_echo_pid}" > "${SESSION_DIR}/safety_echo.pid"

printf '\nHIL READY (watchdog pid %s, topic monitor pid %s). Ctrl-C to stop everything.\n' \
  "${watchdog_pid}" "${monitor_pid}"

wait "${bridge_pid}"
