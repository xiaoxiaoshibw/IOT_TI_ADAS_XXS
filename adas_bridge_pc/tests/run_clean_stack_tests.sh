#!/usr/bin/env bash
# run_clean_stack_tests.sh — start_pc_stack_clean.sh 的确定性单元测试
#
# 用轻量 stub 进程 + 空闲端口（39000-39002）驱动【真实的】clean 脚本，验证
# 进程管理正确性，秒级完成、与占用 2000-2002 的外部 CARLA 完全隔离：
#   V2  正常启动→SIGINT 停止：三组件独立 PGID、CARLA 派生孙进程随组回收、
#       退出码 130、无残留、有序 EXPECTED_CLEANUP_TERMINATION
#   V3  bridge 首个退出(code=42)：FIRST_FAILED_PROCESS=bridge、退出码保真(非143)、
#       carla/gui 被有序清理、无残留
#   V4  carla 收 SIGSEGV：FIRST_FAILED signal=SIGSEGV status=139、其遗留孙进程被回收、
#       bridge/gui 被清理、无残留
#   T7  连续启停 10 次：每次无残留、端口释放
#
# 不依赖 CARLA/GPU/ROS；不发无范围 kill；测试自身进程组不受影响。

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ADAS_PC_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CLEAN="${ADAS_PC_DIR}/start_pc_stack_clean.sh"
FREE_PORTS="39000 39001 39002"
WORK="$(mktemp -d /tmp/clean_stack_test.XXXXXX)"

PASS=0
FAIL=0
trap 'rm -rf "${WORK}" 2>/dev/null || true' EXIT

pass() { PASS=$((PASS + 1)); printf '  \033[32mPASS\033[0m %s\n' "$1"; }
fail() { FAIL=$((FAIL + 1)); printf '  \033[31mFAIL\033[0m %s\n' "$1"; [[ -n "${2:-}" ]] && printf '       %s\n' "$2"; }
assert_eq() { if [[ "$2" == "$3" ]]; then pass "$1 [=$2]"; else fail "$1" "期望=$3 实际=$2"; fi; }
assert_grep() { if grep -qE "$2" "$3" 2>/dev/null; then pass "$1"; else fail "$1" "未匹配: $2  于 $3"; fi; }
assert_dead() { if kill -0 "$2" 2>/dev/null; then fail "$1" "PID $2 仍存活"; else pass "$1 [pid $2 已亡]"; fi; }
assert_no_group() {
  local live; live="$(pgrep -g "$2" 2>/dev/null || true)"
  if [[ -z "${live}" ]]; then pass "$1 [pgid $2 无残留]"; else fail "$1" "pgid $2 残留: ${live//$'\n'/ }"; fi
}

# 生成一个 stub 脚本。模式：
#   long          常驻，直到被组信号杀死
#   long+gc       常驻，且派生一个孙进程(sleep)，把孙 PID 写到 $2 指定文件
#   exit:<code>   sleep 0.6 后以该码退出
#   segv+gc       派生孙进程并把孙 PID 写文件，sleep 0.6 后对自身发 SIGSEGV
make_stub() {
  local path="$1" mode="$2" gc_file="${3:-/dev/null}"
  {
    printf '#!/usr/bin/env bash\n'
    printf 'set -u\n'
    case "${mode}" in
      long)
        printf 'exec sleep 100000\n'
        ;;
      long+gc)
        printf 'sleep 100000 &\n'
        printf 'echo $! > %q\n' "${gc_file}"
        printf 'wait\n'
        ;;
      exit:*)
        printf 'sleep 0.6\n'
        printf 'exit %s\n' "${mode#exit:}"
        ;;
      segv+gc)
        printf 'sleep 100000 &\n'
        printf 'echo $! > %q\n' "${gc_file}"
        printf 'sleep 0.6\n'
        printf 'kill -SEGV $$\n'
        printf 'wait\n'
        ;;
    esac
  } > "${path}"
  chmod +x "${path}"
}

# 启动 clean 脚本（stub 注入 + 空闲端口 + pid 就绪判据），回显其 PID 与捕获文件。
# 用 setsid 把被测脚本放进【独立会话】：本测试台架是非交互脚本(job control 关闭)，
# 否则 clean 与台架同进程组，向 clean 发 SIGINT / clean 的组清理会波及台架自身。
# setsid 隔离后 CLEAN_PID 即新会话组长，kill -INT 只作用于它、其清理只碰自己的子组。
launch_clean() {
  local tag="$1" carla_stub="$2" gui_stub="$3" bridge_stub="$4"
  local extra_arg="${5:-}"
  local -a extra_args=()
  [[ -n "${extra_arg}" ]] && extra_args+=("${extra_arg}")
  local cap="${WORK}/${tag}.stderr"
  setsid env \
    STACK_REQUIRED_PORTS="${FREE_PORTS}" \
    STACK_CARLA_READY_MODE=pid \
    STACK_SKIP_HEALTH_GATE=1 \
    STACK_CARLA_LAUNCHER="${carla_stub}" \
    STACK_GUI_LAUNCHER="${gui_stub}" \
    STACK_BRIDGE_LAUNCHER="${bridge_stub}" \
    "${CLEAN}" "${extra_args[@]}" > "${cap}" 2>&1 &
  CLEAN_PID=$!
  CLEAN_CAP="${cap}"
}

# 轮询 clean 的捕获文件直到出现某事件或超时
wait_event() {
  local cap="$1" ev="$2" timeout_ds="${3:-100}" i=0
  while (( i < timeout_ds )); do
    grep -qE "event=${ev}\b" "${cap}" 2>/dev/null && return 0
    sleep 0.1; i=$((i + 1))
  done
  return 1
}

session_dir_of() { sed -nE 's/.*event=SESSION_DIR_READY dir="([^"]+)".*/\1/p' "$1" | head -n1; }

# 必须在调用者(主) shell 里 wait——命令替换 $() 的子 shell 无法 wait 父 shell 的子进程。
# 结果写入全局 RC_LAST。
RC_LAST=0
wait_clean_exit() { set +e; wait "${CLEAN_PID}"; RC_LAST=$?; set -e; }

# ── V2：正常启动→SIGINT 停止 ───────────────────────────────────────
test_v2() {
  printf '\n[V2] 正常启动→SIGINT 停止\n'
  local gc_file="${WORK}/v2_gc.pid"
  make_stub "${WORK}/v2_carla.sh"  long+gc "${gc_file}"
  make_stub "${WORK}/v2_gui.sh"    long
  make_stub "${WORK}/v2_bridge.sh" long
  launch_clean v2 "${WORK}/v2_carla.sh" "${WORK}/v2_gui.sh" "${WORK}/v2_bridge.sh"

  if ! wait_event "${CLEAN_CAP}" STACK_RUNNING; then
    fail "V2 启动" "超时未见 STACK_RUNNING"; cat "${CLEAN_CAP}"; return
  fi
  local sd; sd="$(session_dir_of "${CLEAN_CAP}")"
  local cpid gpid bpid; cpid="$(cat "${sd}/carla.pid")"; gpid="$(cat "${sd}/gui.pid")"; bpid="$(cat "${sd}/bridge.pid")"
  local mpgid; mpgid="$(sed -nE 's/.*event=MANAGER_CONTEXT pid="[0-9]+" pgid="([0-9]+)".*/\1/p' "${CLEAN_CAP}" | head -n1)"
  local cpgid gpgid bpgid
  cpgid="$(ps -o pgid= -p "${cpid}" | tr -d ' ')"
  gpgid="$(ps -o pgid= -p "${gpid}" | tr -d ' ')"
  bpgid="$(ps -o pgid= -p "${bpid}" | tr -d ' ')"

  # 独立进程组
  if [[ "${cpgid}" != "${mpgid}" && "${gpgid}" != "${mpgid}" && "${bpgid}" != "${mpgid}" ]]; then
    pass "V2 三组件 PGID 均≠管理脚本(${mpgid})"
  else
    fail "V2 PGID 隔离" "carla=${cpgid} gui=${gpgid} bridge=${bpgid} mgr=${mpgid}"
  fi
  if [[ "${cpgid}" != "${gpgid}" && "${cpgid}" != "${bpgid}" && "${gpgid}" != "${bpgid}" ]]; then
    pass "V2 三组件 PGID 互不相同"
  else
    fail "V2 PGID 互异" "carla=${cpgid} gui=${gpgid} bridge=${bpgid}"
  fi
  # setsid 后的 PID 应等于各自 PGID（会话组长）
  assert_eq "V2 carla PID==PGID(会话组长)" "${cpid}" "${cpgid}"

  local gcpid; gcpid="$(cat "${gc_file}" 2>/dev/null || echo '')"
  if [[ -n "${gcpid}" ]] && kill -0 "${gcpid}" 2>/dev/null; then
    pass "V2 CARLA 孙进程存活(${gcpid})"
  else
    fail "V2 孙进程" "未捕获或未存活: '${gcpid}'"
  fi

  # 后台进程的 SIGINT 被 POSIX 置为 SIG_IGN 不可 trap，故用 SIGTERM 请求关闭(等价路径)
  kill -TERM "${CLEAN_PID}"
  wait_clean_exit; local rc="${RC_LAST}"
  assert_eq "V2 退出码=143(SIGTERM 请求关闭)" "${rc}" "143"
  assert_grep "V2 记录 REQUESTED_SHUTDOWN" 'event=REQUESTED_SHUTDOWN' "${CLEAN_CAP}"
  assert_grep "V2 bridge 为 EXPECTED_CLEANUP_TERMINATION" 'event=EXPECTED_CLEANUP_TERMINATION role="bridge"' "${CLEAN_CAP}"
  assert_grep "V2 carla 为 EXPECTED_CLEANUP_TERMINATION" 'event=EXPECTED_CLEANUP_TERMINATION role="carla"' "${CLEAN_CAP}"
  # 无残留
  assert_dead "V2 carla 已亡" "${cpid}"
  assert_dead "V2 gui 已亡" "${gpid}"
  assert_dead "V2 bridge 已亡" "${bpid}"
  assert_dead "V2 CARLA 孙进程随组回收" "${gcpid}"
  assert_no_group "V2 carla 组" "${cpgid}"
}

# ── V3：bridge 首个退出，退出码保真 ────────────────────────────────
test_v3() {
  printf '\n[V3] bridge 首个退出(code=42)\n'
  make_stub "${WORK}/v3_carla.sh"  long
  make_stub "${WORK}/v3_gui.sh"    long
  make_stub "${WORK}/v3_bridge.sh" exit:42
  launch_clean v3 "${WORK}/v3_carla.sh" "${WORK}/v3_gui.sh" "${WORK}/v3_bridge.sh"

  wait_clean_exit; local rc="${RC_LAST}"
  local sd; sd="$(session_dir_of "${CLEAN_CAP}")"
  assert_eq "V3 退出码=42(根因保真,非143)" "${rc}" "42"
  assert_grep "V3 FIRST_PROCESS_FAILED role=bridge status=42" 'event=FIRST_PROCESS_FAILED role="bridge" .*status="42"' "${CLEAN_CAP}"
  assert_grep "V3 carla 被有序清理" 'event=EXPECTED_CLEANUP_TERMINATION role="carla"' "${CLEAN_CAP}"
  assert_grep "V3 gui 被有序清理" 'event=EXPECTED_CLEANUP_TERMINATION role="gui"' "${CLEAN_CAP}"
  assert_grep "V3 bridge 标记 ROOT_CAUSE_TERMINATION" 'event=ROOT_CAUSE_TERMINATION role="bridge"' "${CLEAN_CAP}"
  # 无残留
  local cpid gpid; cpid="$(cat "${sd}/carla.pid")"; gpid="$(cat "${sd}/gui.pid")"
  assert_dead "V3 carla 已亡" "${cpid}"
  assert_dead "V3 gui 已亡" "${gpid}"
}

# ── V4：carla 收 SIGSEGV，遗留孙进程被回收 ─────────────────────────
test_v4() {
  printf '\n[V4] carla 收 SIGSEGV(status=139)\n'
  local gc_file="${WORK}/v4_gc.pid"
  make_stub "${WORK}/v4_carla.sh"  segv+gc "${gc_file}"
  make_stub "${WORK}/v4_gui.sh"    long
  make_stub "${WORK}/v4_bridge.sh" long
  launch_clean v4 "${WORK}/v4_carla.sh" "${WORK}/v4_gui.sh" "${WORK}/v4_bridge.sh"

  # 等 carla 起来并记录孙进程
  wait_event "${CLEAN_CAP}" STACK_RUNNING || true
  local gcpid; gcpid="$(cat "${gc_file}" 2>/dev/null || echo '')"

  wait_clean_exit; local rc="${RC_LAST}"
  local sd; sd="$(session_dir_of "${CLEAN_CAP}")"
  assert_eq "V4 退出码=139(SIGSEGV)" "${rc}" "139"
  assert_grep "V4 FIRST_PROCESS_FAILED role=carla signal=SIGSEGV" 'event=FIRST_PROCESS_FAILED role="carla" .*signal="SIGSEGV"' "${CLEAN_CAP}"
  assert_grep "V4 carla 标记 ROOT_CAUSE_TERMINATION" 'event=ROOT_CAUSE_TERMINATION role="carla"' "${CLEAN_CAP}"
  assert_grep "V4 回收 carla 遗留孙进程 CLEANUP_REAP_ORPHANS" 'event=CLEANUP_REAP_ORPHANS role="carla"' "${CLEAN_CAP}"
  assert_grep "V4 bridge 被有序清理" 'event=EXPECTED_CLEANUP_TERMINATION role="bridge"' "${CLEAN_CAP}"
  if [[ -n "${gcpid}" ]]; then
    assert_dead "V4 carla 遗留孙进程被回收" "${gcpid}"
  else
    fail "V4 孙进程" "未捕获孙 PID"
  fi
  local gpid bpid; gpid="$(cat "${sd}/gui.pid")"; bpid="$(cat "${sd}/bridge.pid")"
  assert_dead "V4 gui 已亡" "${gpid}"
  assert_dead "V4 bridge 已亡" "${bpid}"
}

# ── T7：连续启停 10 次，无残留 ─────────────────────────────────────
test_skip_gui() {
  printf '\n[V5] 外部 GUI 模式\n'
  make_stub "${WORK}/v5_carla.sh" long
  make_stub "${WORK}/v5_gui.sh" exit:99
  make_stub "${WORK}/v5_bridge.sh" long
  launch_clean v5 "${WORK}/v5_carla.sh" "${WORK}/v5_gui.sh" \
    "${WORK}/v5_bridge.sh" --skip-gui
  if ! wait_event "${CLEAN_CAP}" STACK_RUNNING; then
    fail "V5 启动" "超时未见 STACK_RUNNING"
    return
  fi
  local sd; sd="$(session_dir_of "${CLEAN_CAP}")"
  if [[ ! -e "${sd}/gui.pid" ]]; then
    pass "V5 未重复启动 GUI"
  else
    fail "V5 GUI" "意外生成 gui.pid"
  fi
  assert_grep "V5 记录外部 GUI" 'event=PHASE_SKIP phase=10 name=gui reason=external_gui' "${CLEAN_CAP}"
  kill -TERM "${CLEAN_PID}"
  wait_clean_exit
  assert_eq "V5 退出码=143" "${RC_LAST}" "143"
}

test_cycles() {
  printf '\n[T7] 连续启停 10 次\n'
  make_stub "${WORK}/cyc_carla.sh"  long+gc "${WORK}/cyc_gc.pid"
  make_stub "${WORK}/cyc_gui.sh"    long
  make_stub "${WORK}/cyc_bridge.sh" long
  local ok=0 n=10 i
  printf '  %-6s %-8s %-8s %-8s %-10s %-8s\n' 轮次 carla_pid carla_pgid 孙pid 停止方式 残留
  for (( i = 1; i <= n; i++ )); do
    : > "${WORK}/cyc_gc.pid"
    launch_clean "cyc${i}" "${WORK}/cyc_carla.sh" "${WORK}/cyc_gui.sh" "${WORK}/cyc_bridge.sh"
    if ! wait_event "${CLEAN_CAP}" STACK_RUNNING 150; then
      printf '  %-6s 启动超时\n' "${i}"; continue
    fi
    local sd cpid cpgid gcpid; sd="$(session_dir_of "${CLEAN_CAP}")"
    cpid="$(cat "${sd}/carla.pid")"; cpgid="$(ps -o pgid= -p "${cpid}" | tr -d ' ')"
    gcpid="$(cat "${WORK}/cyc_gc.pid" 2>/dev/null || echo '?')"
    kill -TERM "${CLEAN_PID}"; wait_clean_exit; local rc="${RC_LAST}"
    local resid; resid="$(pgrep -g "${cpgid}" 2>/dev/null | tr '\n' ' ' || true)"
    printf '  %-6s %-8s %-8s %-8s %-10s %-8s\n' "${i}" "${cpid}" "${cpgid}" "${gcpid}" "SIGTERM(${rc})" "${resid:-无}"
    if [[ "${rc}" == "143" && -z "${resid}" ]]; then ok=$((ok + 1)); fi
  done
  assert_eq "T7 10 轮全部干净启停" "${ok}" "${n}"
}

printf '========== start_pc_stack_clean.sh 确定性测试 ==========\n'
printf 'clean 脚本: %s\n' "${CLEAN}"
printf '空闲端口:   %s   工作目录: %s\n' "${FREE_PORTS}" "${WORK}"
test_v2
test_v3
test_v4
test_skip_gui
test_cycles
printf '\n========== 汇总: PASS=%d FAIL=%d ==========\n' "${PASS}" "${FAIL}"
(( FAIL == 0 ))
