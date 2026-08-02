#!/usr/bin/env bash
# start_pc_stack_clean.sh — 改进版 PC 栈启动器（阶段一 · 任务 3）
#
# 目标（相对 start_pc_stack.sh 的修复点）：
#   · 每个组件用 setsid 建【独立会话/进程组】，cleanup 只对各自组发信号，
#     绝不误伤管理脚本自身或兄弟进程（根治历史 exit 143 连锁自杀）。
#   · 拆分 INT/TERM/EXIT trap：信号只记录“请求关闭”，EXIT 保存并使用原始退出码，
#     cleanup 幂等且不覆盖首个失败进程的真实状态。
#   · wait -n -p 精确识别“第一个退出的子进程”，记录 FIRST_FAILED_*。
#   · 启动前对 2000/2001/2002 做 TCP+UDP 端口预检；冲突则 REFUSE_START、退出 20，
#     绝不自动杀占用进程（含外部/他人的 CARLA）。
#   · --preflight-only 只做只读体检，不启动、不终止任何进程。
#
# 本版【不实现】CARLA 自动重启：优先把进程组/端口预检/首退追踪/cleanup 打磨稳，
# 自动重启推迟到连续启停测试通过后、由 systemd 的 Restart=on-failure +
# StartLimitBurst 实现，避免脚本内重启与服务重启叠加成风暴。
#
# 原文件 start_pc_stack.sh 保持不变，作为回滚版本。

# 本脚本大量函数经 trap 与命令替换间接调用，shellcheck 的可达性分析会误报
# SC2317（unreachable）；这些函数确有调用，故文件级豁免该项。
# shellcheck disable=SC2317
set -Eeuo pipefail
IFS=$'\n\t'

# ── 路径（不依赖 CWD）──────────────────────────────────────────────
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

# ── 集中定义退出码（杜绝散落魔法数）────────────────────────────────
readonly EXIT_OK=0
readonly EXIT_GENERAL=1
readonly EXIT_PORT_CONFLICT=20
readonly EXIT_DEP_MISSING=21

# ── 可调参数（尽量沿用原脚本口径，不擅自降低）──────────────────────
# 端口清单可经 STACK_REQUIRED_PORTS 覆盖（空格分隔；自动化测试指向空闲端口，
# 从而与占用 2000/2001/2002 的外部 CARLA 隔离）。用显式 IFS=' ' 拆分，
# 避免全局 IFS=$'\n\t' 把空格分隔串并成单元素。
IFS=' ' read -r -a REQUIRED_PORTS <<< "${STACK_REQUIRED_PORTS-2000 2001 2002}"
readonly REQUIRED_PORTS
readonly CARLA_READY_TIMEOUT_SEC="${CARLA_STARTUP_TIMEOUT:-60}"
readonly CARLA_READY_POLL_SEC=1
readonly CLEANUP_TERM_WAIT_SEC=3
readonly CARLA_RPC_PORT="${CARLA_PORT:-2000}"

# 组件启动器路径可经环境覆盖（仅供测试注入 stub；生产默认=真实启动脚本）。
# CARLA 就绪判据 STACK_CARLA_READY_MODE：rpc（默认，要求 RPC/world 持续稳定）|
# pid（仅确认存活，确定性进程管理测试专用）。TCP listen 不是 CARLA world ready：
# 初始化期立即 load_world 可使 CARLA 0.9.16 Shipping/RHIThread SIGSEGV。
CARLA_LAUNCHER="${STACK_CARLA_LAUNCHER:-${SCRIPT_DIR}/start_carla.sh}"
GUI_LAUNCHER="${STACK_GUI_LAUNCHER:-${SCRIPT_DIR}/start_gui.sh}"
BRIDGE_LAUNCHER="${STACK_BRIDGE_LAUNCHER:-${SCRIPT_DIR}/start_bridge.sh}"
CARLA_READY_MODE="${STACK_CARLA_READY_MODE:-rpc}"
CARLA_STABILIZATION_SEC="${CARLA_STABILIZATION_SEC:-10}"

# CARLA 安装根（与 common.sh 口径一致；只读，用于识别“本项目 CARLA”）
CARLA_ROOT="${CARLA_ROOT:-${HOME}/CARLA_0.9.16}"

# ── 运行期状态（不得硬编码为某次运行结果）──────────────────────────
CLEANUP_STARTED=0
SHUTDOWN_REQUESTED=0
SHUTDOWN_SIGNAL=""
MAIN_EXIT_STATUS=0
FIRST_FAILED_PROCESS=""
FIRST_FAILED_PID=""
FIRST_FAILED_PGID=""
FIRST_FAILED_STATUS=""
FIRST_FAILED_SIGNAL=""
FIRST_FAILED_AT=""

PREFLIGHT_ONLY=0
SKIP_GUI=0
declare -a BRIDGE_ARGS=(
  --control-source can
  --can-transport canalystii
  --can-device-index 0
  --can-channel 1
  --can-bitrate 500000
  --can-feedback-timeout-s 0.1
)

# 直接子进程映射
declare -A PROC_PID=()
declare -A PROC_PGID=()
declare -A PROC_STATE=()
declare -A PID_TO_ROLE=()
declare -a MONITORED_PIDS=()

SESSION_DIR=""
STACK_LOG=""
MANAGER_PGID=""

# ── 结构化日志 ─────────────────────────────────────────────────────
_ts() { date +%Y-%m-%dT%H:%M:%S%:z; }

log() {
  # 用法：log LEVEL EVENT [key=value ...]
  local level="$1" event="$2"
  shift 2
  local line
  line="$(_ts) level=${level} event=${event}"
  local kv
  for kv in "$@"; do
    line+=" ${kv}"
  done
  if [[ -n "${STACK_LOG}" ]]; then
    printf '%s\n' "${line}" | tee -a "${STACK_LOG}" >&2
  else
    printf '%s\n' "${line}" >&2
  fi
}
log_info()  { log INFO  "$@"; }
log_warn()  { log WARN  "$@"; }
log_error() { log ERROR "$@"; }

# 把可能含空格的值安全地封成 key="value"
kv_q() {
  local key="$1" val="$2"
  printf '%s="%s"' "${key}" "${val//\"/\\\"}"
}

# 用空格连接数组元素（IFS=$'\n\t' 下 "${arr[*]}" 会用换行连接，日志观感差）
join_sp() { local IFS=' '; printf '%s' "$*"; }

# ── 信号编号/名称互转（通用，不硬编码 130/137/139/143）─────────────
signal_number() {
  # 名称→编号，如 INT→2；失败返回 0
  local name="$1" n
  n="$(kill -l "${name}" 2>/dev/null || true)"
  [[ "${n}" =~ ^[0-9]+$ ]] && printf '%s' "${n}" || printf '0'
}

status_to_signal() {
  # 退出码→信号名（SIGxxx）；<128 视为非信号退出
  local status="$1"
  if (( status < 128 )); then
    printf 'NONE'
    return 0
  fi
  local n=$(( status - 128 )) name
  name="$(kill -l "${n}" 2>/dev/null || true)"
  if [[ -n "${name}" ]]; then
    printf 'SIG%s' "${name}"
  else
    printf 'UNKNOWN_SIGNAL_%s' "${n}"
  fi
}

# ── 日志目录 ───────────────────────────────────────────────────────
init_logging() {
  SESSION_DIR="${ADAS_PC_LOG_ROOT:-${REPO_ROOT}/logs}/hil_run_$(date +%Y%m%d_%H%M%S)_$$"
  mkdir -p "${SESSION_DIR}"
  STACK_LOG="${SESSION_DIR}/stack.log"
  : > "${STACK_LOG}"
  # 组件独立日志占位，避免混写
  : > "${SESSION_DIR}/carla.log"
  : > "${SESSION_DIR}/gui.log"
  : > "${SESSION_DIR}/bridge.log"
  printf 'role\tpid\tppid\tpgid\tsid\tstat\tlstart\tcmd\n' > "${SESSION_DIR}/processes.tsv"
  printf '%s\n' "$$" > "${SESSION_DIR}/manager.pid"
  log_info SESSION_DIR_READY "$(kv_q dir "${SESSION_DIR}")"
}

# ── 依赖检查 ───────────────────────────────────────────────────────
check_dependencies() {
  local missing=0 c
  for c in ss setsid ps kill awk sed date tee flock pgrep; do
    if ! command -v "${c}" >/dev/null 2>&1; then
      log_error DEP_MISSING "$(kv_q command "${c}")"
      missing=1
    fi
  done
  local f
  for f in start_carla.sh start_bridge.sh scripts/common.sh; do
    if [[ ! -f "${SCRIPT_DIR}/${f}" ]]; then
      log_error DEP_MISSING "$(kv_q file "${SCRIPT_DIR}/${f}")"
      missing=1
    fi
  done
  if (( ! SKIP_GUI )) && [[ ! -f "${GUI_LAUNCHER}" ]]; then
    log_error DEP_MISSING "$(kv_q file "${GUI_LAUNCHER}")"
    missing=1
  fi
  if [[ ! -x "${CARLA_ROOT}/CarlaUE4.sh" ]]; then
    log_warn CARLA_ROOT_SUSPECT \
      "$(kv_q carla_root "${CARLA_ROOT}")" reason=missing_CarlaUE4.sh
  fi
  return "${missing}"
}

# ── 进程/端口只读辅助 ──────────────────────────────────────────────
proc_cmdline() {
  local pid="$1" c
  if c="$(tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null)"; then
    printf '%s' "${c% }"
  else
    printf ''
    return 1
  fi
}
proc_owner() { stat -c '%U' "/proc/$1" 2>/dev/null || printf 'unknown'; }
proc_exe()   { readlink -f "/proc/$1/exe" 2>/dev/null || printf '' ; }

# 只读识别“本项目 CARLA”——组合多信号，不仅凭进程名含 Carla
is_project_carla_pid() {
  local pid="$1" exe cmd base
  exe="$(proc_exe "${pid}")"
  cmd="$(proc_cmdline "${pid}" || true)"
  base="$(basename -- "${exe:-}")"
  # 信号 1：可执行文件就是 UE4 shipping 二进制
  local is_ue4=0
  [[ "${base}" == "CarlaUE4-Linux-Shipping" || "${base}" == CarlaUE4-Linux* ]] && is_ue4=1
  # 信号 2：exe 或 cmdline 落在 CARLA_ROOT 下
  local in_root=0
  [[ -n "${exe}" && "${exe}" == "${CARLA_ROOT}"* ]] && in_root=1
  [[ -n "${cmd}" && "${cmd}" == *"${CARLA_ROOT}"* ]] && in_root=1
  # 二者兼具才判为“本项目 CARLA”（保守；本轮仅用于报告，不做终止）
  [[ "${is_ue4}" -eq 1 && "${in_root}" -eq 1 ]]
}

# 给定端口+协议，回显监听该端口的 PID（可能多个，逐行）
port_listener_pids() {
  local port="$1" proto="$2"
  # 用数组传参：IFS=$'\n\t' 下字符串不按空格拆分，必须用数组避免畸形单参
  local -a flags
  case "${proto}" in
    tcp) flags=(-H -ltnp) ;;
    udp) flags=(-H -lunp) ;;
    *)   return 0 ;;
  esac
  # 第 4 列是本地 地址:端口；末段等于 port 即命中
  ss "${flags[@]}" 2>/dev/null | awk -v p="${port}" '
    {
      n = split($4, a, ":");
      if (a[n] == p) print $0;
    }' | grep -oE 'pid=[0-9]+' | cut -d= -f2 | sort -u
}

report_port_owner() {
  local port="$1" proto="$2" pid="$3"
  local owner cmd exe classify
  if [[ -z "${pid}" ]]; then
    log_error PORT_CONFLICT \
      "$(kv_q port "${port}")" "$(kv_q protocol "${proto}")" \
      pid=unknown reason=insufficient_permissions
    return 0
  fi
  owner="$(proc_owner "${pid}")"
  exe="$(proc_exe "${pid}")"
  if ! cmd="$(proc_cmdline "${pid}")"; then
    cmd=""
  fi
  if [[ -z "${cmd}" && -z "${exe}" ]]; then
    log_error PORT_CONFLICT \
      "$(kv_q port "${port}")" "$(kv_q protocol "${proto}")" \
      "$(kv_q pid "${pid}")" pid_detail=unknown \
      reason=insufficient_permissions
    return 0
  fi
  if is_project_carla_pid "${pid}"; then classify="project_carla"; else classify="foreign"; fi
  log_error PORT_CONFLICT \
    "$(kv_q port "${port}")" "$(kv_q protocol "${proto}")" \
    "$(kv_q pid "${pid}")" "$(kv_q owner "${owner}")" \
    "$(kv_q classify "${classify}")" \
    "$(kv_q exe "${exe}")" "$(kv_q command "${cmd}")"
}

# 端口预检：有冲突返回非零（并逐个报告 owner），全空闲返回 0
check_required_ports() {
  local conflict=0 port proto pid
  for port in "${REQUIRED_PORTS[@]}"; do
    for proto in tcp udp; do
      while IFS= read -r pid; do
        [[ -z "${pid}" ]] && continue
        report_port_owner "${port}" "${proto}" "${pid}"
        conflict=1
      done < <(port_listener_pids "${port}" "${proto}")
    done
  done
  return "${conflict}"
}

# 只读报告：当前相关进程（CARLA/bridge/gui），不做任何终止
report_related_processes() {
  local pids
  # 精确匹配 UE4 二进制与本项目 ROS 节点，避免宽泛匹配
  pids="$(pgrep -f 'CarlaUE4-Linux-Shipping|CarlaUE4.sh|bridge_node|adas_gui' 2>/dev/null || true)"
  if [[ -z "${pids}" ]]; then
    log_info RELATED_PROCESSES none=true
    return 0
  fi
  local pid line
  while IFS= read -r pid; do
    [[ -z "${pid}" ]] && continue
    line="$(ps -o pid=,ppid=,pgid=,sid=,stat=,cmd= -p "${pid}" 2>/dev/null || true)"
    [[ -z "${line}" ]] && continue
    log_info RELATED_PROCESS "$(kv_q ps "$(printf '%s' "${line}" | sed 's/^ *//')")"
  done <<< "${pids}"
}

env_summary() {
  log_info ENV_SUMMARY \
    "$(kv_q ros_domain_id "${ROS_DOMAIN_ID:-unset}")" \
    "$(kv_q rmw "${RMW_IMPLEMENTATION:-unset}")" \
    "$(kv_q town "${TOWN:-unset}")" \
    "$(kv_q carla_root "${CARLA_ROOT}")" \
    "$(kv_q required_ports "$(join_sp "${REQUIRED_PORTS[@]}")")" \
    "$(kv_q bash "${BASH_VERSION}")" \
    "$(kv_q preflight_only "${PREFLIGHT_ONLY}")" \
    "$(kv_q skip_gui "${SKIP_GUI}")"
}

# ── 组件启动（供 V2–V4；本轮 preflight 不会走到）──────────────────
register_process() {
  local role="$1" pid="$2" pgid="$3"
  PROC_PID["${role}"]="${pid}"
  PROC_PGID["${role}"]="${pgid}"
  PROC_STATE["${role}"]="running"
  PID_TO_ROLE["${pid}"]="${role}"
  MONITORED_PIDS+=("${pid}")
  local row
  row="$(ps -o pid=,ppid=,pgid=,sid=,stat=,lstart=,cmd= -p "${pid}" 2>/dev/null | sed 's/^ *//' || true)"
  printf '%s\t%s\n' "${role}" "${row}" >> "${SESSION_DIR}/processes.tsv"
}

start_component() {
  # start_component ROLE LOGFILE CMD...
  local role="$1" logfile="$2"
  shift 2
  log_info PROCESS_STARTING "$(kv_q role "${role}")"
  setsid "$@" >"${logfile}" 2>&1 &
  local pid=$!
  # 读取真实 PGID（不假定 PID==PGID）
  local pgid
  pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ' || true)"
  if [[ -z "${pgid}" ]]; then
    log_error PROCESS_START_FAILED "$(kv_q role "${role}")" "$(kv_q pid "${pid}")" reason=no_pgid
    return 1
  fi
  if [[ "${pgid}" == "${MANAGER_PGID}" ]]; then
    log_error PROCESS_PGID_NOT_ISOLATED \
      "$(kv_q role "${role}")" "$(kv_q pid "${pid}")" "$(kv_q pgid "${pgid}")" \
      "$(kv_q manager_pgid "${MANAGER_PGID}")"
    return 1
  fi
  register_process "${role}" "${pid}" "${pgid}"
  echo "${pid}" > "${SESSION_DIR}/${role}.pid"
  log_info PROCESS_STARTED "$(kv_q role "${role}")" "$(kv_q pid "${pid}")" "$(kv_q pgid "${pgid}")"
  return 0
}

wait_carla_ready() {
  # 测试就绪判据：只确认 CARLA 进程存活（stub 不开 RPC 端口）
  if [[ "${CARLA_READY_MODE}" == "pid" ]]; then
    if kill -0 "${PROC_PID[carla]:-0}" 2>/dev/null; then
      log_info CARLA_READY_PID_MODE "$(kv_q pid "${PROC_PID[carla]:-}")"
      return 0
    fi
    log_error CARLA_EXITED_DURING_STARTUP "$(kv_q pid "${PROC_PID[carla]:-}")" mode=pid
    return 1
  fi
  local waited=0
  while (( waited < CARLA_READY_TIMEOUT_SEC )); do
    if (exec 3<>"/dev/tcp/127.0.0.1/${CARLA_RPC_PORT}") 2>/dev/null; then
      exec 3>&- 3<&-
      log_info CARLA_PORT_READY "$(kv_q port "${CARLA_RPC_PORT}")" "$(kv_q waited_s "${waited}")"
      break
    fi
    # CARLA 提前退出则立即停止等待
    if ! kill -0 "${PROC_PID[carla]}" 2>/dev/null; then
      log_error CARLA_EXITED_DURING_STARTUP "$(kv_q pid "${PROC_PID[carla]:-}")"
      tail -n 20 "${SESSION_DIR}/carla.log" 2>/dev/null | sed 's/^/carla.log| /' >&2 || true
      return 1
    fi
    sleep "${CARLA_READY_POLL_SEC}"
    waited=$(( waited + CARLA_READY_POLL_SEC ))
  done
  if (( waited >= CARLA_READY_TIMEOUT_SEC )); then
    log_error CARLA_PORT_TIMEOUT "$(kv_q port "${CARLA_RPC_PORT}")" "$(kv_q timeout_s "${CARLA_READY_TIMEOUT_SEC}")"
    tail -n 20 "${SESSION_DIR}/carla.log" 2>/dev/null | sed 's/^/carla.log| /' >&2 || true
    return 1
  fi
  local readiness_json="${SESSION_DIR}/carla_readiness.json"
  if ! python3 "${SCRIPT_DIR}/scripts/carla_readiness.py" \
      --host 127.0.0.1 --port "${CARLA_RPC_PORT}" --expected-town "${TOWN:-Town04}" \
      --timeout "${CARLA_READY_TIMEOUT_SEC}" --stabilization "${CARLA_STABILIZATION_SEC}" \
      --output "${readiness_json}" >"${SESSION_DIR}/carla_readiness.log" 2>&1; then
    log_error CARLA_RPC_NOT_READY "$(kv_q report "${readiness_json}")"
    tail -n 40 "${SESSION_DIR}/carla_readiness.log" >&2 || true
    return 1
  fi
  if ! kill -0 "${PROC_PID[carla]}" 2>/dev/null; then
    log_error CARLA_EXITED_AFTER_RPC_GATE "$(kv_q pid "${PROC_PID[carla]}")"
    return 1
  fi
  log_info CARLA_RPC_READY "$(kv_q report "${readiness_json}")" \
    "$(kv_q stabilization_s "${CARLA_STABILIZATION_SEC}")"
  return 0
}

record_first_exit() {
  local pid="$1" status="$2"
  [[ -n "${FIRST_FAILED_PID}" ]] && return 0   # 只记录第一个
  local role="${PID_TO_ROLE[${pid}]:-unknown}"
  local pgid="${PROC_PGID[${role}]:-}"
  local sig; sig="$(status_to_signal "${status}")"
  FIRST_FAILED_PROCESS="${role}"
  FIRST_FAILED_PID="${pid}"
  FIRST_FAILED_PGID="${pgid}"
  FIRST_FAILED_STATUS="${status}"
  FIRST_FAILED_SIGNAL="${sig}"
  FIRST_FAILED_AT="$(_ts)"
  PROC_STATE["${role}"]="exited"
  if (( status == 0 )); then
    log_info FIRST_PROCESS_EXITED \
      "$(kv_q role "${role}")" "$(kv_q pid "${pid}")" "$(kv_q pgid "${pgid}")" \
      "$(kv_q status "${status}")"
  else
    log_error FIRST_PROCESS_FAILED \
      "$(kv_q role "${role}")" "$(kv_q pid "${pid}")" "$(kv_q pgid "${pgid}")" \
      "$(kv_q status "${status}")" "$(kv_q signal "${sig}")" "$(kv_q at "${FIRST_FAILED_AT}")"
  fi
}

monitor_processes() {
  # 阻塞直到第一个受监控子进程退出（单次），随后返回其退出码
  local exited_pid="" status=0
  set +e
  wait -n -p exited_pid "${MONITORED_PIDS[@]}"
  status=$?
  set -e
  if (( SHUTDOWN_REQUESTED )); then
    log_warn REQUESTED_SHUTDOWN "$(kv_q signal "SIG${SHUTDOWN_SIGNAL}")"
    return 0
  fi
  if [[ -z "${exited_pid}" ]]; then
    # 无法定位具体 PID（如无子进程/被信号中断）——保守记录
    log_warn WAIT_RETURNED_NO_PID "$(kv_q status "${status}")"
    return "${status}"
  fi
  record_first_exit "${exited_pid}" "${status}"
  return "${status}"
}

# ── cleanup（幂等；只碰已注册且独立进程组；绝不打到管理脚本组）─────
# 回显某进程组内仍存活的成员 PID（按 pgid 精确匹配；不是无范围 pkill）
pgroup_live_pids() { pgrep -g "$1" 2>/dev/null || true; }

kill_process_group() {
  # kill_process_group ROLE —— 以【进程组存活成员】为判据回收，先 SIGTERM 限时等待再 SIGKILL
  local role="$1"
  local pid="${PROC_PID[${role}]:-}"
  local pgid="${PROC_PGID[${role}]:-}"
  [[ -z "${pid}" ]] && { log_info CLEANUP_SKIP "$(kv_q role "${role}")" reason=not_started; return 0; }

  # 根因进程：其真实退出状态已记录，绝不改写；但仍需回收它可能遗留的子进程
  # （真实场景：CarlaUE4.sh 死了但 UE4 二进制仍占 2000）。
  local is_root=0
  if [[ "${role}" == "${FIRST_FAILED_PROCESS}" ]]; then
    is_root=1
    log_info ROOT_CAUSE_TERMINATION "$(kv_q role "${role}")" "$(kv_q pid "${pid}")" \
      "$(kv_q status "${FIRST_FAILED_STATUS}")" "$(kv_q signal "${FIRST_FAILED_SIGNAL}")"
  fi

  # PGID 安全校验：空/0/1/等于管理脚本组，一律拒绝发组信号（护住自身）
  if [[ -z "${pgid}" || "${pgid}" == "0" || "${pgid}" == "1" || "${pgid}" == "${MANAGER_PGID}" ]]; then
    log_error CLEANUP_REFUSE_UNSAFE_PGID "$(kv_q role "${role}")" \
      "$(kv_q pgid "${pgid}")" "$(kv_q manager_pgid "${MANAGER_PGID}")"
    return 0
  fi

  # 组内是否还有存活成员（leader 可能已死但派生子进程仍在）
  local live
  live="$(pgroup_live_pids "${pgid}")"
  if [[ -z "${live}" ]]; then
    PROC_STATE["${role}"]="${PROC_STATE[${role}]:-exited}"
    log_info CLEANUP_SKIP "$(kv_q role "${role}")" "$(kv_q pgid "${pgid}")" reason=group_empty
    return 0
  fi

  if (( is_root )); then
    log_info CLEANUP_REAP_ORPHANS "$(kv_q role "${role}")" "$(kv_q pgid "${pgid}")" \
      "$(kv_q live_pids "${live//$'\n'/ }")"
  else
    log_info CLEANUP_SIGTERM "$(kv_q role "${role}")" "$(kv_q pgid "${pgid}")" \
      "$(kv_q live_pids "${live//$'\n'/ }")"
  fi
  kill -TERM -- "-${pgid}" 2>/dev/null || true
  local waited=0
  while (( waited < CLEANUP_TERM_WAIT_SEC * 10 )); do
    if [[ -z "$(pgroup_live_pids "${pgid}")" ]]; then
      PROC_STATE["${role}"]="terminated"
      log_info EXPECTED_CLEANUP_TERMINATION "$(kv_q role "${role}")" "$(kv_q pgid "${pgid}")"
      return 0
    fi
    sleep 0.1
    waited=$(( waited + 1 ))
  done
  log_warn CLEANUP_SIGKILL "$(kv_q role "${role}")" "$(kv_q pgid "${pgid}")"
  kill -KILL -- "-${pgid}" 2>/dev/null || true
  PROC_STATE["${role}"]="killed"
  log_warn FORCED_CLEANUP_TERMINATION "$(kv_q role "${role}")" "$(kv_q pgid "${pgid}")"
  return 0
}

run_cleanup() {
  (( CLEANUP_STARTED )) && return 0
  CLEANUP_STARTED=1
  log_info CLEANUP_BEGIN \
    "$(kv_q first_failed "${FIRST_FAILED_PROCESS:-none}")" \
    "$(kv_q shutdown_signal "${SHUTDOWN_SIGNAL:-none}")"
  # 依赖顺序：先停消费/派生端（bridge、gui），最后停 CARLA，
  # 避免 bridge 在 CARLA 已亡时刷重连错误。每步失败不打断后续。
  local role
  for role in bridge gui carla; do
    kill_process_group "${role}" || true
  done
  # 回收僵尸
  wait 2>/dev/null || true
  # 终态汇总（读取 PROC_STATE，便于复盘各组件最终去向）
  local r
  for r in "${!PROC_STATE[@]}"; do
    log_info PROC_FINAL_STATE "$(kv_q role "${r}")" "$(kv_q state "${PROC_STATE[${r}]}")"
  done
  log_info CLEANUP_END
}

# ── trap 处理 ──────────────────────────────────────────────────────
handle_signal() {
  # 记录“请求关闭”，随即 exit，由幂等的 EXIT trap(handle_exit)统一收尾。
  # 不在信号 trap 内直接做多进程清理（避免重入）；直接 exit 是最可靠的触发方式，
  # 不依赖 `wait -n` 在收到 trap 后一定返回（带显式 PID 列表时该行为不稳）。
  local sig="$1"
  if (( SHUTDOWN_REQUESTED )); then return 0; fi
  SHUTDOWN_REQUESTED=1
  SHUTDOWN_SIGNAL="${sig}"
  log_warn REQUESTED_SHUTDOWN "$(kv_q signal "SIG${sig}")"
  exit $(( 128 + $(signal_number "${sig}") ))
}
# 注：前台(交互)运行时 Ctrl+C 的 SIGINT 可正常捕获；但若本脚本被非交互 shell 以
# `cmd &` 异步后台启动，POSIX 规定其 SIGINT/SIGQUIT 被置为 SIG_IGN 且不可再 trap，
# 此时应改用 SIGTERM 请求关闭（TERM 不受该限制，handle_signal 一样处理）。

handle_exit() {
  local captured="$1"
  trap - EXIT INT TERM
  MANAGER_PGID="${MANAGER_PGID:-$(ps -o pgid= -p "$$" | tr -d ' ')}"
  run_cleanup
  # 决定最终退出码，保留根因/请求信号语义，绝不用 cleanup 的 SIGTERM 覆盖
  local final="${captured}"
  if (( SHUTDOWN_REQUESTED )); then
    final=$(( 128 + $(signal_number "${SHUTDOWN_SIGNAL}") ))
  elif [[ -n "${FIRST_FAILED_STATUS}" ]]; then
    final="${FIRST_FAILED_STATUS}"
  fi
  log_info STACK_EXIT \
    "$(kv_q final_status "${final}")" \
    "$(kv_q captured "${captured}")" \
    "$(kv_q first_failed_process "${FIRST_FAILED_PROCESS:-none}")" \
    "$(kv_q first_failed_pid "${FIRST_FAILED_PID:-none}")" \
    "$(kv_q first_failed_pgid "${FIRST_FAILED_PGID:-none}")" \
    "$(kv_q first_failed_status "${FIRST_FAILED_STATUS:-none}")" \
    "$(kv_q first_failed_signal "${FIRST_FAILED_SIGNAL:-none}")" \
    "$(kv_q shutdown_signal "${SHUTDOWN_SIGNAL:-none}")"
  exit "${final}"
}

# ── 参数解析 ───────────────────────────────────────────────────────
usage() {
  cat <<'EOF'
用法: start_pc_stack_clean.sh [选项] [-- 传给 bridge 的参数...]

选项:
  --preflight-only   只做只读体检（依赖/端口/相关进程/环境），不启动也不终止任何进程
  --skip-gui         不启动 GUI，供已运行的 GUI 统一编排后台栈
  --help             显示本帮助

说明:
  · 端口 2000/2001/2002 被占用时默认 REFUSE_START，返回码 20，绝不自动杀占用进程。
  · 各组件用 setsid 独立进程组启动；cleanup 只对各自组发信号，不会误伤本脚本或兄弟进程。
  · 本版不做 CARLA 自动重启（推迟到 systemd 阶段）。
EOF
}

parse_args() {
  while (( $# )); do
    case "$1" in
      --preflight-only) PREFLIGHT_ONLY=1; shift ;;
      --skip-gui) SKIP_GUI=1; shift ;;
      --help|-h) usage; exit "${EXIT_OK}" ;;
      --) shift; BRIDGE_ARGS+=("$@"); break ;;
      *) BRIDGE_ARGS+=("$1"); shift ;;
    esac
  done
}

# ── 主流程 ─────────────────────────────────────────────────────────
main() {
  parse_args "$@"

  # 复用 common.sh 的环境（ROS_DOMAIN_ID/RMW/CYCLONEDDS_URI/TOWN/CARLA_ROOT 口径）
  # 注意：common.sh 会 set -euo pipefail（不含 -E），source 后重新固化本脚本的选项。
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/scripts/common.sh"
  set -Eeuo pipefail
  IFS=$'\n\t'
  CARLA_ROOT="${CARLA_ROOT:-${HOME}/CARLA_0.9.16}"

  MANAGER_PGID="$(ps -o pgid= -p "$$" | tr -d ' ')"

  # 拆分 trap：INT/TERM 只记录请求；EXIT 保存原始退出码并做幂等清理
  trap 'handle_signal INT'  INT
  trap 'handle_signal TERM' TERM
  trap 'handle_exit $?'     EXIT

  init_logging
  env_summary
  log_info MANAGER_CONTEXT "$(kv_q pid "$$")" "$(kv_q pgid "${MANAGER_PGID}")"

  if ! check_dependencies; then
    log_error PREFLIGHT_FAILED reason=dependency_missing
    MAIN_EXIT_STATUS="${EXIT_DEP_MISSING}"
    exit "${MAIN_EXIT_STATUS}"
  fi

  # ── 端口预检（两种模式共用）──
  report_related_processes
  if ! check_required_ports; then
    log_error REFUSE_START reason=port_conflict "$(kv_q ports "$(join_sp "${REQUIRED_PORTS[@]}")")"
    MAIN_EXIT_STATUS="${EXIT_PORT_CONFLICT}"
    exit "${MAIN_EXIT_STATUS}"
  fi
  log_info PORTS_CLEAR "$(kv_q ports "$(join_sp "${REQUIRED_PORTS[@]}")")"

  if (( PREFLIGHT_ONLY )); then
    log_info PREFLIGHT_OK
    MAIN_EXIT_STATUS="${EXIT_OK}"
    exit "${MAIN_EXIT_STATUS}"
  fi

  # ── 完整启动（V2–V4 使用；本轮 preflight 不会到达）──
  log_info PHASE_BEGIN phase=1 name=carla_process
  start_component carla "${SESSION_DIR}/carla.log" "${CARLA_LAUNCHER}"
  log_info PHASE_PASS phase=1 name=carla_process
  log_info PHASE_BEGIN phase=2 name=carla_rpc
  if ! wait_carla_ready; then
    MAIN_EXIT_STATUS="${EXIT_GENERAL}"
    exit "${MAIN_EXIT_STATUS}"
  fi
  log_info PHASE_PASS phase=2 name=carla_rpc
  log_info PHASE_BEGIN phase=3 name=bridge
  start_component bridge "${SESSION_DIR}/bridge.log" "${BRIDGE_LAUNCHER}" "${BRIDGE_ARGS[@]}"
  log_info PHASE_PASS phase=3 name=bridge

  if [[ "${STACK_SKIP_HEALTH_GATE:-0}" != "1" ]]; then
    log_info PHASE_BEGIN phase=4-9 name=hil_health_gate
    source_workspace
    if ! python3 "${SCRIPT_DIR}/scripts/check_hil_ready.py" \
        --session-dir "${SESSION_DIR}" >"${SESSION_DIR}/health_gate.log" 2>&1; then
      log_error PHASE_FAIL phase=4-9 name=hil_health_gate
      tail -n 120 "${SESSION_DIR}/health_gate.log" >&2 || true
      MAIN_EXIT_STATUS="${EXIT_GENERAL}"
      exit "${MAIN_EXIT_STATUS}"
    fi
    log_info PHASE_PASS phase=4-9 name=hil_health_gate
  else
    log_warn PHASE_SKIP phase=4-9 name=hil_health_gate reason=test_injection
  fi

  if (( SKIP_GUI )); then
    log_info PHASE_SKIP phase=10 name=gui reason=external_gui
  else
    log_info PHASE_BEGIN phase=10 name=gui
    start_component gui "${SESSION_DIR}/gui.log" "${GUI_LAUNCHER}"
    log_info PHASE_PASS phase=10 name=gui
  fi

  log_info STACK_RUNNING \
    "$(kv_q carla_pid "${PROC_PID[carla]:-}")" \
    "$(kv_q gui_pid "${PROC_PID[gui]:-external}")" \
    "$(kv_q bridge_pid "${PROC_PID[bridge]:-}")"

  monitor_processes || true
  MAIN_EXIT_STATUS="${FIRST_FAILED_STATUS:-0}"
  exit "${MAIN_EXIT_STATUS}"
}

main "$@"
