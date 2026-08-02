#!/usr/bin/env bash
# wait_for_mcu_safe.sh — 操作侧一键等"现在可以按复位"
#
# 流程：
#   Phase 1: 跑 hil_preflight_before_mcu.sh（条件门，不放过）
#   Phase 2: 输出"SAFE TO PRESS RESET"提示 + 可选终端/LED 信号
#   Phase 3: 监控 0x202 心跳，等待 MCU 状态机迁移
#            - 看到 STANDBY/INIT    → 复位已生效，等启动
#            - 看到 ACTIVE         → 成功，退出 0
#            - 看到 DEGRADED       → 提示但继续等（可能在 RECOVERY）
#            - 看到 FAILSAFE/EMERG → 失败，退出非 0
#            - timeout             → 失败，退出非 0
#
# 这是 prompt §B.4 现场操作流的"按一个按钮"版：
#   1. PC 跑 start_pc_stack.sh
#   2. Orin 跑 can_hil.launch.py
#   3. PC 跑 wait_for_mcu_safe.sh
#   4. 看到 "SAFE TO PRESS RESET" 后，操作员按 LAUNCHXL-F280025C 的 S1
#   5. 脚本自动验证 MCU 进入 ACTIVE
#
# 注意：本脚本**不**驱动 MCU 电源/复位线（需要物理隔离评估，默认不做）。
# 它只判定"何时该按" + "按了之后是否成功"。
#
# 退出码：
#   0 = MCU 已进入 ACTIVE（成功）
#   1 = preflight 未通过
#   2 = MCU 上电后未在窗口内进入 ACTIVE
#   3 = MCU 进入 FAILSAFE / EMERGENCY_BRAKE / FAULT_LOCK（异常）
#   4 = CAN 监控期出错
#   5 = 用户中断（Ctrl-C）
#   7 = 缺工具

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PREFLIGHT="${SCRIPT_DIR}/hil_preflight_before_mcu.sh"

# 0x202 byte0 系统状态值（与 adas_can_protocol.h 对齐）
ST_INIT=0
ST_STANDBY=1
ST_ACTIVE=2
ST_DEGRADED=3
ST_MRM=4
ST_EMERGENCY_BRAKE=5
ST_FAILSAFE=6
ST_FAULT_LOCK=7

# 默认参数
CAN_IF="can1"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
MONITOR_TIMEOUT=30          # MCU 上电后等 ACTIVE 的秒数
PREFLIGHT_TIMEOUT=60        # preflight 阶段总超时
PREFLIGHT_STABLE=5          # 传给 preflight 的稳定窗
MONITOR_POLL_MS=200         # 状态轮询间隔

# OLED/LED 通知（可选）：有这些工具时才用
NOTIFY_METHOD="auto"         # auto | none | bell

log() {
    printf '[%s] %s\n' "$(date '+%H:%M:%S' 2>/dev/null || date '+%H:%M:%S')" "$*" >&2
}

usage() {
    cat <<EOF
用法: ${0##*/} [选项]

  --can-if <name>          CAN 接口（默认 can1）
  --ros-domain <id>        ROS_DOMAIN_ID（默认 43）
  --preflight-timeout <N>  preflight 总超时秒数（默认 60）
  --stable-seconds <N>     preflight 稳定窗（默认 5）
  --monitor-timeout <N>    按复位后等 ACTIVE 的秒数（默认 30）
  --notify <method>        通知方式: auto | bell | none（默认 auto）
  -h, --help               显示本帮助

退出码：0=ACTIVE；1=preflight 失败；2=MCU 未进 ACTIVE；3=FAILSAFE/EMERG；
        4=CAN 错误；5=用户中断；7=缺工具。

典型用法（PC 侧）:
  # 1) 启动 PC + Orin
  cd adas_pc && ./start_pc_stack.sh       # PC
  ssh orin 'sudo systemctl start adas-hil'  # Orin
# 2) 跑 preflight + 监控（一个命令搞定）
   ./tools/harness/wait_for_mcu_safe.sh
  # 3) 看到 "SAFE TO PRESS RESET NOW" 后按 LAUNCHXL S1 复位
  # 4) 看到 "MCU is ACTIVE — ready for HIL" 即完成
EOF
}

# 解析参数
while [[ $# -gt 0 ]]; do
    case "$1" in
        --can-if)              CAN_IF="$2"; shift 2;;
        --ros-domain)          ROS_DOMAIN_ID="$2"; shift 2;;
        --preflight-timeout)   PREFLIGHT_TIMEOUT="$2"; shift 2;;
        --stable-seconds)      PREFLIGHT_STABLE="$2"; shift 2;;
        --monitor-timeout)     MONITOR_TIMEOUT="$2"; shift 2;;
        --notify)              NOTIFY_METHOD="$2"; shift 2;;
        -h|--help)             usage; exit 0;;
        *)                     printf '未知参数: %s\n' "$1" >&2; usage; exit 7;;
    esac
done

# 工具检查
for tool in awk grep date; do
    command -v "${tool}" >/dev/null 2>&1 || { printf '缺工具: %s\n' "${tool}" >&2; exit 7; }
done
if ! command -v candump >/dev/null 2>&1; then
    printf '缺工具: candump（apt install can-utils）\n' >&2; exit 7
fi
[[ -x "${PREFLIGHT}" ]] || { printf '找不到 preflight: %s\n' "${PREFLIGHT}" >&2; exit 7; }

# 用户中断
trap 'printf "\n用户中断 — 不动 MCU。\n" >&2; exit 5' INT TERM

# 通知函数
notify_user() {
    case "${NOTIFY_METHOD}" in
        none) ;;
        bell|auto)
            # 终端响铃
            printf '\a' >&2
            ;;
    esac
}

# -------------------- Phase 1: preflight --------------------
log "Phase 1/3: preflight gate（can=${CAN_IF} stable=${PREFLIGHT_STABLE}s timeout=${PREFLIGHT_TIMEOUT}s）"
if ! "${PREFLIGHT}" \
        --can-if "${CAN_IF}" \
        --ros-domain "${ROS_DOMAIN_ID}" \
        --stable-seconds "${PREFLIGHT_STABLE}" \
        --sample-seconds "$((PREFLIGHT_STABLE < 8 ? PREFLIGHT_STABLE + 1 : 8))" \
        --timeout-seconds "${PREFLIGHT_TIMEOUT}"; then
    rc=$?
    log "preflight FAILED (exit ${rc})"
    log "→ 不要按 MCU 复位。先排查: preflight 退出码 1-6 对应 ROS2/CAN/timeout"
    exit 1
fi

# -------------------- Phase 2: 提示可按复位 --------------------
log ""
log "================================================================"
log "  SAFE TO PRESS MCU RESET NOW"
log "  按 LAUNCHXL-F280025C 上的 S1 按钮（或重新上电）"
log "  等待 MCU 启动并进入 ACTIVE（最久 ${MONITOR_TIMEOUT}s）"
log "================================================================"
log ""
notify_user

# -------------------- Phase 3: 监控 0x202 --------------------
log "Phase 3/3: 监控 0x202 心跳..."

# 启 candump 后台写到临时文件，限时 MONITOR_TIMEOUT
TMP_LOG="/tmp/wait_mcu_safe_$$_$(date +%s).log"
timeout "${MONITOR_TIMEOUT}"s candump -t A -L "${CAN_IF}" >"${TMP_LOG}" 2>/dev/null &
CANDUMP_PID=$!

# trap 扩展：用户中断时也要清掉 candump
trap 'kill '"${CANDUMP_PID}"' 2>/dev/null || true; rm -f "${TMP_LOG}"; printf "\n用户中断 — 不动 MCU。\n" >&2; exit 5' INT TERM

# 等 candump 启动
sleep 0.5

DEADLINE=$(($(date +%s) + MONITOR_TIMEOUT))
LAST_STATE_NAME="?"
SAW_RESET=0                  # 是否观察到 ST_INIT/ST_STANDBY（证明 MCU 复位生效）
SAW_ACTIVE=0
SAW_FAILSAFE=0
SAW_DEGRADED=0
SAW_EMERG=0
SAW_FAULT=0
FRAMES_202_TOTAL=0

state_name() {
    case "$1" in
        "${ST_INIT}")             echo "INIT";;
        "${ST_STANDBY}")          echo "STBY";;
        "${ST_ACTIVE}")           echo "ACTIVE";;
        "${ST_DEGRADED}")         echo "DEG";;
        "${ST_MRM}")              echo "MRM";;
        "${ST_EMERGENCY_BRAKE}")  echo "EMERG";;
        "${ST_FAILSAFE}")         echo "FAIL";;
        "${ST_FAULT_LOCK}")       echo "LOCK";;
        *)                        echo "UNK($1)";;
    esac
}

# 主轮询循环
while [[ $(date +%s) -lt ${DEADLINE} ]]; do
    # 取最后一帧 0x202
    last_line=$(grep -E '\([0-9]+\.[0-9]+\)\s+'"${CAN_IF}"'\s+202#' "${TMP_LOG}" 2>/dev/null | tail -1)
    if [[ -n "${last_line}" ]]; then
        # 解析 byte0（在 # 后的第一个 hex 字节）
        # candump -L: "(123.456) can1 202#0A 1A ..."
        hex_first=$(awk -F'#' '{print $2}' <<<"${last_line}" | awk '{print $1}')
        if [[ -n "${hex_first}" ]]; then
            state_dec=$((16#${hex_first}))
            state_dec=$((state_dec & 0xFF))  # 截断到 0-255
            FRAMES_202_TOTAL=$((FRAMES_202_TOTAL + 1))
            name=$(state_name "${state_dec}")
            # 只在状态变化时打印
            if [[ "${name}" != "${LAST_STATE_NAME}" ]]; then
                log "  0x202 byte0=0x${hex_first} state=${name} (frame #${FRAMES_202_TOTAL})"
                LAST_STATE_NAME="${name}"
            fi
            # 状态机事件
            case "${state_dec}" in
                "${ST_INIT}"|"${ST_STANDBY}")
                    [[ "${SAW_RESET}" == "0" ]] && { log "  → MCU 已启动（reset 生效）"; SAW_RESET=1; }
                    ;;
                "${ST_ACTIVE}")
                    [[ "${SAW_ACTIVE}" == "0" ]] && { log "  → ACTIVE 达成"; SAW_ACTIVE=1; }
                    ;;
                "${ST_DEGRADED}")
                    [[ "${SAW_DEGRADED}" == "0" ]] && { log "  → DEGRADED（降级，不一定是错）"; SAW_DEGRADED=1; }
                    ;;
                "${ST_EMERGENCY_BRAKE}")
                    SAW_EMERG=1
                    ;;
                "${ST_FAILSAFE}")
                    SAW_FAILSAFE=1
                    ;;
                "${ST_FAULT_LOCK}")
                    SAW_FAULT=1
                    ;;
            esac
            # 致命状态：立即退出
            if [[ "${SAW_EMERG}" == "1" || "${SAW_FAILSAFE}" == "1" || "${SAW_FAULT}" == "1" ]]; then
                kill "${CANDUMP_PID}" 2>/dev/null || true
                rm -f "${TMP_LOG}"
                log ""
                log "✗ MCU 进入异常态 ${name}，HIL 不可继续"
                log "  请检查: 链路/soC 侧故障注入? 复位原因 (0x203)?"
                exit 3
            fi
            # 成功：ACTIVE
            if [[ "${SAW_ACTIVE}" == "1" ]]; then
                kill "${CANDUMP_PID}" 2>/dev/null || true
                rm -f "${TMP_LOG}"
                log ""
                log "============================================================"
                log "  ✓ MCU is ACTIVE — ready for HIL"
                log "  Saw 0x202 frame: state=${name}, total=${FRAMES_202_TOTAL}"
                log "============================================================"
                notify_user
                exit 0
            fi
        fi
    fi
    sleep "$(awk "BEGIN { printf \"%.3f\", ${MONITOR_POLL_MS}/1000 }")"
done

# timeout
kill "${CANDUMP_PID}" 2>/dev/null || true
rm -f "${TMP_LOG}"
log ""
log "✗ timeout: ${MONITOR_TIMEOUT}s 内 MCU 未进入 ACTIVE"
log "  最后观察到状态: ${LAST_STATE_NAME}"
log "  0x202 总帧数: ${FRAMES_202_TOTAL}"
log "  复位是否生效: $([[ ${SAW_RESET} == 1 ]] && echo yes || echo no)"
log "  请检查: 复位是否真的按下? CAN 链路? MCU 是否在跑?"
exit 2
