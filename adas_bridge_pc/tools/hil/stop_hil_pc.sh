#!/usr/bin/env bash
set -Eeuo pipefail
HIL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=adas_pc/tools/hil/hil_common.sh
source "${HIL_DIR}/hil_common.sh"
session_dir="${1:-$(find "${ADAS_PC_DIR}/logs" -maxdepth 1 -type d -name 'hil_run_*' -printf '%p\n' | sort | tail -n 1)}"
[[ -n "${session_dir}" && -r "${session_dir}/manager.pid" ]] || fail "找不到 manager.pid；拒绝宽泛清理"
manager_pid=$(<"${session_dir}/manager.pid")
[[ "${manager_pid}" =~ ^[0-9]+$ ]] || fail "manager.pid 非法"
cmdline=$(tr '\0' ' ' <"/proc/${manager_pid}/cmdline" 2>/dev/null || true)
[[ "${cmdline}" == *'start_pc_stack_clean.sh'* ]] || fail "PID ${manager_pid} 不是 HIL manager；拒绝发送信号"
kill -TERM "${manager_pid}"
pass "已向 HIL manager PID ${manager_pid} 请求统一退出；仅其登记的独立进程组会被清理"
