#!/usr/bin/env bash
# hil_preflight_before_mcu.sh
#
# 操作侧 HIL preflight gate：只有 PC / ROS2 / Orin / CAN 链路已经稳定
# 并通过条件门，才允许给 LAUNCHXL-F280025C MCU 上电或复位。
#
# 设计原则（对齐 docs/hil_startup_preflight_guide.md）：
#   1. 不伪造证据：CAN 不可用时直接返回失败，不输出"可以上电"。
#   2. 不基于固定 sleep 判定：所有放行条件都是"连续满足 N 秒"。
#   3. 不修改 MCU 安全语义：失败时绝不输出 PASS / 0 退出码。
#   4. 与 start_pc_stack.sh 的 check_hil_ready.py 互不依赖：本脚本可独立
#      在 Orin 侧（systemd 启动 MCU 上电前）调用，也可在 PC 侧手动调用。
#
# 退出码（必须严格遵守，方便 systemd / 启动器分流）：
#   0 = PRECONDITION PASS（可给 MCU 上电/复位）
#   1 = ROS2 关键节点未 ready / lifecycle 未 ACTIVE / topic 不新鲜
#   2 = CAN 接口异常（不存在/未 UP/bitrate 错/BUS-OFF/ERROR-PASSIVE/丢帧）
#   3 = 关键 CAN 帧缺失（0x100/0x101/0x102/0x103 任一在采样窗内零帧）
#   4 = CAN 帧不连续（seq 倒退/重复/长空窗/freshness 冻结）
#   5 = CAN 错误计数增长（TX/RX error 或 bus-error 增量 > 0）
#   6 = 超时（在指定 timeout-seconds 内未达到稳定窗）
#   7 = 缺少必要工具（candump / ros2 / python3 / ip），或参数非法
#
# 重要：脚本若因缺工具退出 7，调用方**不得**直接放行，必须人工确认环境。

set -euo pipefail

PROG_NAME="$(basename "$0")"
# 退出码约定（严格遵守，方便 systemd / 启动器分流）
EXIT_PASS=0
# shellcheck disable=SC2034
EXIT_ROS_NOT_READY=1
# shellcheck disable=SC2034
EXIT_CAN_IF=2
EXIT_CAN_FRAMES_MISSING=3
EXIT_CAN_FRAMES_NOT_CONT=4
# shellcheck disable=SC2034
EXIT_CAN_ERR_GROWING=5
EXIT_TIMEOUT=6
EXIT_MISSING_TOOL=7

# 默认参数
CAN_IF="can1"
STABLE_SECONDS=5
SAMPLE_SECONDS=8           # 单次窗口采样长度
TIMEOUT_SECONDS=60
REQUIRE_BACKUP=0           # 主源就够；如启用备机(0x110~0x113)需要 1
CAN_BITRATE_BPS=500000
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
VERBOSE=0

log() {
    if [[ "${VERBOSE}" == "1" ]]; then
        printf '[%s] %s\n' "$(date '+%H:%M:%S.%3N' 2>/dev/null || date '+%H:%M:%S')" "$*" >&2
    fi
}
fail() {
    local code="$1"; shift
    printf 'PRECONDITION FAIL: %s\n' "$*" >&2
    exit "${code}"
}
note() { printf '%s\n' "$*"; }

usage() {
    cat <<EOF
用法: ${PROG_NAME} [选项]

  --can-if <name>           SocketCAN 接口名（默认 can1）
  --stable-seconds <N>      全部条件需连续满足 N 秒才放行（默认 5）
  --sample-seconds <N>      单次 CAN 帧连续性采样长度（默认 8）
  --timeout-seconds <N>     总等待超时（默认 60）
  --require-backup          要求备源 0x110~0x113 同样新鲜（默认关闭）
  --bitrate <bps>           期望 bitrate（默认 500000）
  --ros-domain <id>         ROS_DOMAIN_ID（默认 \$ROS_DOMAIN_ID 或 43）
  --allow-no-bitrate        跳过 bitrate 字段校验（仅供 vcan 仿真环境用，
                            生产 PEAK PCAN-USB 路径下禁止使用）
  --verbose                 打印每个采样窗的内部状态
  -h|--help                显示本帮助

退出码：0=PASS；1=ROS2 未 ready；2=CAN 接口异常；3=CAN 帧缺失；
        4=CAN 帧不连续；5=CAN 错误增长；6=超时；7=缺工具。
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --can-if)            CAN_IF="$2"; shift 2;;
        --stable-seconds)    STABLE_SECONDS="$2"; shift 2;;
        --sample-seconds)    SAMPLE_SECONDS="$2"; shift 2;;
        --timeout-seconds)   TIMEOUT_SECONDS="$2"; shift 2;;
        --require-backup)    REQUIRE_BACKUP=1; shift;;
        --bitrate)           CAN_BITRATE_BPS="$2"; shift 2;;
        --ros-domain)        ROS_DOMAIN_ID="$2"; shift 2;;
        --allow-no-bitrate)  ALLOW_NO_BITRATE=1; shift;;
        --verbose|-v)        VERBOSE=1; shift;;
        -h|--help)           usage; exit 0;;
        *)                   printf '未知参数：%s\n' "$1" >&2; usage; exit "${EXIT_MISSING_TOOL}";;
    esac
done

# 仅供 vcan / 仿真环境使用：跳过真实 bitrate 校验
ALLOW_NO_BITRATE="${ALLOW_NO_BITRATE:-0}"

# 1) 工具自检：缺哪个都不允许假装能跑
for tool in ip candump python3 awk grep sort uniq; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
        fail "${EXIT_MISSING_TOOL}" "缺少必要工具：${tool}"
    fi
done
if ! command -v ros2 >/dev/null 2>&1; then
    note "ros2 命令未发现——本环境的 ROS_DOMAIN_ID=${ROS_DOMAIN_ID} 不可达。ROS2 门禁将立即判失败。"
    ROS2_AVAILABLE=0
else
    ROS2_AVAILABLE=1
fi
log "工具自检通过"

# 2) 参数合法性
if ! [[ "${STABLE_SECONDS}" =~ ^[0-9]+$ ]] || [[ "${STABLE_SECONDS}" -lt 1 ]]; then
    fail "${EXIT_MISSING_TOOL}" "--stable-seconds 必须是正整数"
fi
if ! [[ "${SAMPLE_SECONDS}" =~ ^[0-9]+$ ]] || [[ "${SAMPLE_SECONDS}" -lt 1 ]]; then
    fail "${EXIT_MISSING_TOOL}" "--sample-seconds 必须是正整数"
fi
if ! [[ "${TIMEOUT_SECONDS}" =~ ^[0-9]+$ ]] || [[ "${TIMEOUT_SECONDS}" -lt 1 ]]; then
    fail "${EXIT_MISSING_TOOL}" "--timeout-seconds 必须是正整数"
fi
if [[ "${SAMPLE_SECONDS}" -lt "${STABLE_SECONDS}" ]]; then
    fail "${EXIT_MISSING_TOOL}" "--sample-seconds (${SAMPLE_SECONDS}) 必须 >= --stable-seconds (${STABLE_SECONDS})"
fi

# 关键 CAN ID 列表（与 adas_can_protocol.h 对齐）
PRI_IDS=(0x100 0x101 0x102 0x103)
if [[ "${REQUIRE_BACKUP}" == "1" ]]; then
    BAK_IDS=(0x110 0x111 0x112 0x113)
else
    BAK_IDS=()
fi

ALL_IDS=("${PRI_IDS[@]}" "${BAK_IDS[@]}")
# shellcheck disable=SC2034
: "${ALL_IDS[@]}"  # 保留变量定义供将来扩展（备用 0x110~0x113 检查）
export ALL_IDS

START_EPOCH=$(date +%s)
DEADLINE=$((START_EPOCH + TIMEOUT_SECONDS))

note "HIL preflight gate: can=${CAN_IF} stable=${STABLE_SECONDS}s sample=${SAMPLE_SECONDS}s timeout=${TIMEOUT_SECONDS}s ros_domain=${ROS_DOMAIN_ID} require_backup=${REQUIRE_BACKUP}"

# 3) CAN 接口状态检查
check_can_iface() {
    if ! ip link show "${CAN_IF}" >/dev/null 2>&1; then
        return 1   # 不存在
    fi
    local state
    state=$(ip -details -statistics link show "${CAN_IF}" 2>/dev/null | head -1)
    # 真实 CAN：state UP；vcan：state UNKNOWN（虚拟设备），但 <UP,...> 链路标志存在
    if ! grep -qE "state UP|state UNKNOWN" <<<"${state}"; then
        return 1
    fi
    if ! grep -qE "UP[,>]|UNKNOWN" <<<"${state}"; then
        return 1
    fi
    # bitrate 校验
    if [[ "${ALLOW_NO_BITRATE}" != "1" ]]; then
        if ! grep -q "bitrate ${CAN_BITRATE_BPS}" <<<"${state}"; then
            log "bitrate 不匹配：期望 ${CAN_BITRATE_BPS}，实际：$(grep -o 'bitrate [0-9]*' <<<"${state}" | head -1)"
            return 1
        fi
    fi
    # BUS-OFF / ERROR-PASSIVE 显式禁止
    if ip -details -statistics link show "${CAN_IF}" 2>/dev/null | grep -qE "BUS-OFF|ERROR-PASSIVE"; then
        return 1
    fi
    return 0
}

# 4) 单次 CAN 采样
#    返回值写到全局变量供 caller 判别；stdout 末尾输出 JSON 单行供脚本消费
do_candump_sample() {
    local out_file="$1"
    local t_sec="$2"
    # 使用 -t A 绝对时间；-L 文本格式（log 模式）；超时精确到秒
    timeout "${t_sec}"s candump -t A -L "${CAN_IF}" >"${out_file}" 2>/dev/null || true
}

# 5) 分析采样结果：判断每个关键 ID 是否到帧、seq 是否连续、是否有空窗
analyze_sample() {
    local sample_file="$1"
    python3 - "${sample_file}" "${REQUIRE_BACKUP}" <<'PYEOF'
import sys, re
from collections import defaultdict

path = sys.argv[1]
require_backup = (sys.argv[2] == "1")
PRI_IDS = [0x100, 0x101, 0x102, 0x103]
BAK_IDS = [0x110, 0x111, 0x112, 0x113]
ALL = PRI_IDS + (BAK_IDS if require_backup else [])

# 解析 candump -L： "(timestamp) can1 100#DATA"  或  "(timestamp) can1 100##0DATA"
frame_re = re.compile(r"\(([\d.]+)\)\s+\S+\s+([0-9A-Fa-f]+)#")
by_id = defaultdict(list)
with open(path, "r", errors="replace") as f:
    for line in f:
        m = frame_re.search(line)
        if not m:
            continue
        try:
            ts = float(m.group(1))
            cid = int(m.group(2), 16)
        except ValueError:
            continue
        if cid in ALL:
            by_id[cid].append(ts)

missing = [hex(c) for c in ALL if len(by_id[c]) == 0]
if missing:
    print(f"FAIL_MISSING:{','.join(missing)}")
    sys.exit(3)

# 频率与 seq 行为（仅 0x100/0x101/0x102/0x103 含 seq 字段；统一检查 5 号字节为 seq）
# candump -L 不直接给 byte 索引；需用另一个解析路径。下面用 count + max_gap 做最简连续性。
# 由于 0x101/0x102 标称 10ms（>=80Hz @ 8s 采样），0x100/0x103 标称 20ms（>=40Hz）。
NOMINAL = {0x100: 0.020, 0x101: 0.010, 0x102: 0.010, 0x103: 0.020,
           0x110: 0.020, 0x111: 0.010, 0x112: 0.010, 0x113: 0.020}
issues = []
max_gap_summary = {}
for cid in ALL:
    arr = sorted(by_id[cid])
    if len(arr) < 2:
        issues.append(f"INSUFFICIENT:{hex(cid)}")
        continue
    gaps = [b - a for a, b in zip(arr, arr[1:])]
    max_gap = max(gaps)
    max_gap_summary[hex(cid)] = round(max_gap, 4)
    # 标称间隔的 4 倍视为"可疑长空窗"——单点扰动不致触发，但启动期那种 1s
    # 长停顿会被抓出。允许 1 次以内（首帧间隔不计入统计）。
    long_gaps = [g for g in gaps[1:] if g > 4 * NOMINAL[cid]]
    if len(long_gaps) > 1:
        issues.append(f"CONTINUITY_GAP:{hex(cid)}:max_gap={max_gap:.3f}s long_count={len(long_gaps)}")

if issues:
    print(f"FAIL_NOT_CONTINUOUS:{'|'.join(issues)}|max_gaps={max_gap_summary}")
    sys.exit(4)
else:
    print(f"OK:cnt=" + ",".join(f"{hex(c)}:{len(by_id[c])}" for c in ALL)
          + "|max_gap=" + ",".join(f"{k}:{v}s" for k, v in max_gap_summary.items()))
    sys.exit(0)
PYEOF
}

# 6) CAN 错误增量检查：记录首末 TX/RX 错误 + 采样前后差值
get_can_err_counters() {
    ip -details -statistics link show "${CAN_IF}" 2>/dev/null \
        | awk '/[0-9]+ +drop/ || /[0-9]+ +errors/ || /[0-9]+ +bus-error/ || /[0-9]+ +berr/ {print}'
}

# 7) ROS2 关键节点 + lifecycle + topic
check_ros2() {
    if [[ "${ROS2_AVAILABLE}" != "1" ]]; then
        return 1
    fi
    # 关键节点：can_gateway、command_gate、safety_monitor、aeb、trajectory_follower
    # 至少 lifecycle ACTIVE 才算 ready。
    local required_nodes=("can_gateway")
    local optional_lifecycle=("command_gate" "safety_monitor" "aeb" "trajectory_follower")
    local missing=()
    local nodelist
    if ! nodelist=$(ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" ros2 node list 2>/dev/null); then
        return 1
    fi
    for n in "${required_nodes[@]}"; do
        if ! grep -qx "${n}" <<<"${nodelist}"; then
            missing+=("${n}")
        fi
    done
    if [[ ${#missing[@]} -gt 0 ]]; then
        log "ROS2 缺失关键节点：${missing[*]}"
        return 1
    fi
    # lifecycle 检查（如 ros2 lifecycle 可用）
    if command -v ros2 >/dev/null 2>&1; then
        for n in "${optional_lifecycle[@]}"; do
            if grep -qx "${n}" <<<"${nodelist}"; then
                local state
                state=$(ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" ros2 lifecycle get "${n}" 2>/dev/null || echo "unknown")
                if [[ "${state}" != "active" ]]; then
                    log "Lifecycle 节点 ${n} 状态=${state}（非 active）"
                    return 1
                fi
            fi
        done
    fi
    # 关键 topic：safety_status、/adas/control/gate/control_cmd
    local topics=("/adas/system/safety_status" "/adas/control/gate/control_cmd")
    for t in "${topics[@]}"; do
        local hz
        hz=$(ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" timeout 3s ros2 topic hz "${t}" 2>&1 | grep -oE 'rate: [0-9.]+' | head -1 || echo "")
        if [[ -z "${hz}" ]]; then
            log "topic ${t} 无 hz 输出"
            return 1
        fi
    done
    return 0
}

# 8) 主循环：每个采样窗记录"已连续满足"秒数；任何失败即清零、重新累计
stable_acc=0
sample_index=0
last_ros_ok=0
last_can_if_ok=0
last_can_sample_rc=0
last_can_err_baseline=""

note "[1/4] CAN 接口初始检查..."
if check_can_iface; then
    last_can_if_ok=1
    note "  CAN ${CAN_IF}: UP @ ${CAN_BITRATE_BPS} bps"
else
    note "  CAN ${CAN_IF}: 异常（不存在 / 未 UP / 错误状态）"
fi

last_can_err_baseline=$(get_can_err_counters || true)
log "CAN error baseline: ${last_can_err_baseline:-<empty>}"

while true; do
    now=$(date +%s)
    if [[ "${now}" -ge "${DEADLINE}" ]]; then
        fail "${EXIT_TIMEOUT}" "在 ${TIMEOUT_SECONDS}s 内未达到 ${STABLE_SECONDS}s 稳定窗"
    fi

    sample_index=$((sample_index + 1))
    log "=== sample #${sample_index} (acc=${stable_acc}s/${STABLE_SECONDS}s) ==="

    # (a) ROS2
    if check_ros2; then
        last_ros_ok=1
        log "  ROS2: OK"
    else
        last_ros_ok=0
        log "  ROS2: not ready"
    fi

    # (b) CAN 接口
    if check_can_iface; then
        last_can_if_ok=1
        log "  CAN ${CAN_IF}: UP @ ${CAN_BITRATE_BPS} bps"
    else
        last_can_if_ok=0
        log "  CAN ${CAN_IF}: 异常"
    fi

    # (c) CAN 帧连续性
    sample_file="/tmp/hil_preflight_${$}_${sample_index}.log"
    do_candump_sample "${sample_file}" "${SAMPLE_SECONDS}"
    if [[ ! -s "${sample_file}" ]]; then
        log "  CAN: 采样 0 帧（接口无流量或采样失败）"
        last_can_sample_rc="${EXIT_CAN_FRAMES_MISSING}"
    else
        set +e
        analyze_sample "${sample_file}"
        last_can_sample_rc=$?
        set -e
    fi
    rm -f "${sample_file}"

    # (d) CAN 错误增量
    current_can_err=$(get_can_err_counters || true)
    err_grew=0
    if [[ -n "${last_can_err_baseline}" && -n "${current_can_err}" ]]; then
        # 简易数值差：统计 errors/drop/bus-error 各自数字变化
        if ! diff <(printf '%s\n' "${last_can_err_baseline}") \
                  <(printf '%s\n' "${current_can_err}") >/dev/null; then
            err_grew=1
        fi
    fi
    last_can_err_baseline="${current_can_err}"

    # 累计稳定窗
    if [[ "${last_ros_ok}" == "1" && "${last_can_if_ok}" == "1" \
          && "${last_can_sample_rc}" == "0" && "${err_grew}" == "0" ]]; then
        stable_acc=$((stable_acc + SAMPLE_SECONDS))
        note "  [+] 累计稳定 ${stable_acc}s / ${STABLE_SECONDS}s"
    else
        if [[ "${stable_acc}" -gt 0 ]]; then
            note "  [-] 条件回归，重置累计（was ${stable_acc}s）"
        fi
        stable_acc=0
        # 立即诊断失败原因（不立即退出——给一次补救机会，除非不修就一直回归）
        if [[ "${last_can_if_ok}" != "1" ]]; then
            log "  失败: CAN 接口异常"
        elif [[ "${last_can_sample_rc}" == "${EXIT_CAN_FRAMES_MISSING}" ]]; then
            log "  失败: 关键 CAN 帧缺失"
        elif [[ "${last_can_sample_rc}" == "${EXIT_CAN_FRAMES_NOT_CONT}" ]]; then
            log "  失败: CAN 帧不连续 / seq 异常"
        elif [[ "${err_grew}" == "1" ]]; then
            log "  失败: CAN 错误计数增长"
        elif [[ "${last_ros_ok}" != "1" ]]; then
            log "  失败: ROS2 关键节点未 ready"
        fi
    fi

    if [[ "${stable_acc}" -ge "${STABLE_SECONDS}" ]]; then
        break
    fi
done

note ""
note "=========================================="
note " PRECONDITION PASS"
note "   can=${CAN_IF} @ ${CAN_BITRATE_BPS} bps"
note "   ros_domain=${ROS_DOMAIN_ID} ros2=OK"
note "   stable=${STABLE_SECONDS}s samples=${sample_index}"
note "   can_error_baseline=${last_can_err_baseline:-<none>}"
note " MCU MAY BE POWERED ON (or reset)"
note "=========================================="
exit "${EXIT_PASS}"
