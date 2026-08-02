/*
 * main.c — ADAS 控制器 MCU 主程序（TI LAUNCHXL-F280025C / C28x @100MHz）
 *
 * 角色：接收 Orin Nano 经 CAN 下发的控制目标(0x100~0x103，主/备两路)，做
 *   报文校验 → 主备仲裁 → 安全状态机 → 实时控制映射 → 驱动舵机/闪光灯/蜂鸣器，
 *   并把最终控制量回传 PC(0x201/0x202/0x203)。链路失联/急停/AEB 时自主安全制动。
 *
 * 时序：CPU Timer0 产生 1kHz 中断，ISR 仅做"计 tick + 喂看门狗 + 置运行标志"，
 *   完整控制管线在主循环里跑（背景任务模式），保证 ISR 极短、抖动可控。
 *   控制环卡死 → 片上看门狗超时硬复位（软件 TWDT 等价）。
 *
 * 详见同目录 README.md 与 include/adas_can_protocol.h。
 */
#include "driverlib.h"
#include "device.h"

#include "board.h"
#include "can_comm.h"
#include "control.h"
#include "safety.h"
#include "actuators.h"
#include "hmi.h"
#include "oled2.h"
#include "dgus_screen.h"
#include "adas_config.h"
#include "adas_can_protocol.h"
#include "adas_types.h"
#include "self_test.h"
#include "hil_session.h"
#include "asr_pro.h"

/* ---- ISR 共享（volatile：跨 ISR/主循环可见） ---- */
static volatile uint32_t g_tick_ms = 0U;      /* 自启动累计毫秒 */
static volatile bool     g_run     = false;   /* 有新 tick 待处理 */
static volatile uint16_t g_overrun = 0U;      /* 上个 tick 未及处理的次数 */

/* ---- 主循环持有的状态 ---- */
static McuStatus_t     g_status;
static ControlOutput_t g_out;
static HilSession_t     g_session;
static bool             g_hil_inputs_ready = false;

/* ---- ASR-PRO 语音指令状态 ---- */
static bool     g_asr_stop_requested = false;   /* 靠边停车请求（保持直到停车完成） */
static uint32_t g_asr_stop_start_ms  = 0U;      /* 停车请求起始时刻 */
static uint32_t g_asr_last_status_ms = 0U;      /* 最近一次状态语音播报时刻 */
static uint32_t g_asr_last_speed_ms  = 0U;      /* 最近一次车速语音播报时刻 */
#define ASR_STOP_TIMEOUT_MS     10000U          /* 停车请求 10s 超时自动取消 */
#define ASR_COOLDOWN_MS         3000U           /* 语音播报冷却（防重复触发） */

/* ---- 控制 tick 共享计数 ----
 * 这些变量只允许在 run_tick（控制路径）和 main_handleAsrCommand（背景路径
 * 的"车速查询"分支）里被访问。s_mcu_seq 在 run_tick 内 ++，ASR 路径只读
 * 不增（避免读改写竞争），且读取用 DINT 快照保证原子性。s_alive 与
 * s_last_*_ms 仅在 run_tick 内变更，无需 DINT 保护。 */
static volatile uint16_t s_mcu_seq   = 0U;
static uint16_t s_alive     = 0U;
static uint32_t s_last_control_tx_ms = 0U;
static uint32_t s_last_heartbeat_tx_ms = 0U;
static uint32_t s_last_diag_tx_ms = 0U;

#define LOOP_PERIOD_COUNTS  (DEVICE_SYSCLK_HZ / CTRL_LOOP_HZ)   /* 100000 */

/* -------------------- 1kHz 控制中断 -------------------- */
__interrupt void cpuTimer0ISR(void)
{
    g_tick_ms++;
    if (g_run)                                 /* 上一 tick 尚未被主循环消费 → 超时 */
    {
        if (g_overrun < 0xFFFFU) { g_overrun++; }
    }
    g_run = true;
    /* 注意：不在 ISR 内喂狗。喂狗放在主循环 run_tick 末尾——只有控制管线
     * 真正跑完一圈才喂，从而能捕获"ISR 仍在跑但控制环卡死"的情形（TWDT 语义）。*/

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

/* -------------------- ASR-PRO 语音指令处理 -------------------- */
static void main_handleAsrCommand(uint32_t now)
{
    uint16_t cmd, arg;
    if (!AsrPro_pollCommand(&cmd, &arg)) { return; }

    switch (cmd)
    {
    case ASR_CMD_STOP:
        /* 靠边停车：触发 MRM（最小风险策略），控制层以 MRM_DECEL_MS2 减速
         * 并保持方向居中，直到车辆停稳或超时。 */
        if (!g_asr_stop_requested)
        {
            g_asr_stop_requested = true;
            g_asr_stop_start_ms  = now;
            Safety_applyInject(INJ_CMD_FORCE_MRM, 0U);
            Oled2_setAsrCommand(cmd, now);
        }
        break;

    case ASR_CMD_QUERY_STATUS:
        /* 状态查询：蜂鸣器短促鸣响提示，OLED2 已在后台持续刷新系统状态。
         * 通过 CAN 回传一次诊断帧，SoC/PC 侧可读取。 */
        if ((now - g_asr_last_status_ms) >= ASR_COOLDOWN_MS)
        {
            g_asr_last_status_ms = now;
            Actuators_chirp(now);
            CanComm_sendDiag(&g_status);
            Oled2_setAsrCommand(cmd, now);
        }
        break;

    case ASR_CMD_QUERY_SPEED:
        /* 车速查询：蜂鸣器提示 + 回传一次控制帧（含当前车速信息）。
         * s_mcu_seq 在控制 tick 内每 10 ms 增一次，ASR background 路径
         * 只读不增（避免与 run_tick 的 ++ 读改写竞争）；DINT 包裹保证
         * 拿到一致快照。 */
        if ((now - g_asr_last_speed_ms) >= ASR_COOLDOWN_MS)
        {
            uint16_t seq_snapshot;
            DINT;
            seq_snapshot = s_mcu_seq;
            EINT;
            g_asr_last_speed_ms = now;
            Actuators_chirp(now);
            CanComm_sendControl(&g_out, &g_status, seq_snapshot);
            Oled2_setAsrCommand(cmd, now);
        }
        break;

    case ASR_CMD_FAULT_INJECT:
        /* 故障注入（仅测试构建生效）：注入一次降级故障，触发安全降级流程。
         * 生产构建中此命令被忽略，防止误触发。 */
#if ADAS_TEST_BUILD
        Safety_applyInject(INJ_CMD_FORCE_DEGRADE, arg);
        Oled2_setAsrCommand(cmd, now);
#else
        (void)arg;
#endif
        break;

    default:
        break;
    }
}

/* -------------------- 单个控制 tick -------------------- */

static void run_tick(uint32_t now, bool overrun)
{
    SafetyDecision_t dec;
    const SocCommand_t *cmd;
    uint16_t crc_err = 0U;
    bool buzz;
    uint32_t c_start = CPUTimer_getTimerCount(CPUTIMER0_BASE);

    /* 1) 收 + 校验 + 解码（主/备两路 + 故障注入） */
    CanComm_service(now);
    (void)CanComm_pollRx(now, &crc_err);

    /* 2) 故障注入命令（测试用）。
     * 收到 0x301 → 应用 → 立刻回 0x302。回送里附 system_state/fault_level
     * 是上一 tick 的 g_status（最多 1 ms 滞后）；测试路径非实时控制，
     * 1 ms 滞后不影响判定。seq 自增由 CanComm 内部维护，测试脚本据此
     * 确认回送时间。 */
    {
        uint16_t icmd, iparam;
        if (CanComm_getInjectCommand(&icmd, &iparam))
        {
            Safety_applyInject(icmd, iparam);
            CanComm_sendInjectResponse(icmd, iparam, &g_status);
        }
    }

    /* 3) 按钮（一键主备热冗余） */
    Hmi_poll(now);

    /* 3b) 操作员清故障请求 → 强制 CAN 重新入网（长按 / 0x301 CLEAR 抬起）。
     *     放在采信 CAN 状态之前，使刷新后的总线健康能被本 tick 的新鲜度评估
     *     与延后解闩采信，做到"一次长按即恢复"、无需整机复位。 */
    if (Safety_consumeRecoveryRequest())
{
        CanComm_forceRecovery(now);
        /* 长按是本地恢复入口：复位 HIL session 与 Safety 闩锁，强制 CAN 重新
         * 入网。v3 重构后故障条件消除会由状态机自动解闩，长按只是为了在该刻
         * 立即触发一次 force-recovery，不必靠它进入运行模式。 */
        HilSession_init(&g_session);
    }
    Safety_setCanState(CanComm_isBusHealthy(), CanComm_recoveryExhausted());

    /* 4) 链路新鲜度评估 + 5) 仲裁选源 + 状态机 */
    Safety_evaluateLinks(now, crc_err);
    Safety_step(now, overrun, &g_status, &dec);

/* 5b) Cold-start session gate (v3 自驱授权）。会话门是“授权闩锁”：MCU 自己
     *     在 fresh+safe 的链路条件下自动从 READY→ARMED→ACTIVE，无需按钮/TCP；
     *     控制停权仍由 safety.c 在 FTTI 内独立裁决，因此这里不再要求 0x104 会话
     *     keepalive 保鲜（网关在其 recovery 期会停止发 0x104，MCU 必须能自愈）。 */
    {
        uint16_t request;
        uint16_t sequence;
        uint32_t session_id;
        HilSessionInputs_t in;
        LinkMonitor_t *primary = CanComm_getLink(SRC_IDX_PRIMARY);
        bool has_request = CanComm_getSessionRequest(&request, &session_id,
                                                      &sequence);
        in.control_fresh = primary->protocol_ok && primary->lat_valid &&
            primary->lon_valid && primary->status_valid && primary->seq_seen &&
            primary->lat_seen && primary->lon_seen && primary->status_seen &&
            ((now - primary->last_hb_ms) <= HB_TIMEOUT_MS) &&
            ((now - primary->last_lat_ms) <= CTRL_TIMEOUT_MS) &&
            ((now - primary->last_lon_ms) <= CTRL_TIMEOUT_MS) &&
            ((now - primary->last_status_ms) <= STATUS_TIMEOUT_MS);
        in.safe_to_arm = !g_status.estop &&
                         (g_status.system_state != SYS_MODE_FAULT_LOCK) &&
                         (g_status.fault_level < FAULT_LEVEL_FATAL);
        /* estop 不再视为会话致命故障：EMERGENCY_BRAKE 是瞬时安全态，由 safety.c
         * 在 500 ms 内自行跌落；会话保持 ACTIVE，松开即可平滑重新跟随。会话级
         * 致命只认 FAULT_LOCK（bus-off 耗尽/自检失败）或 SoC 自报 FATAL。 */
        in.fatal_fault = (g_status.system_state == SYS_MODE_FAULT_LOCK) ||
                         (g_status.fault_level >= FAULT_LEVEL_FATAL);
        g_hil_inputs_ready = in.control_fresh && in.safe_to_arm;
        /* 故障消除后自动复位会话：从 session FAULT_LOCK 回到 BOOT_WAIT，0x206
         * 因此报告 BOOT_WAIT/id=0，网关观测到后自动恢复 ANNOUNCE 并重新握手。 */
        if (g_session.ack == HIL_ACK_FAULT_LOCK && !in.fatal_fault)
        {
            HilSession_init(&g_session);
        }
        if (has_request)
        {
            (void)HilSession_handle(&g_session, request, session_id,
                                    sequence, &in);
        }
        else
        {
            HilSession_tick(&g_session, &in);
        }
    }

    /* 6) 取采信源命令 → 7) 实时控制映射（限幅/slew/映射/转向灯） */
    cmd = (dec.cmd_valid && HilSession_controlEnabled(&g_session))
          ? CanComm_getCommand(dec.active_idx) : 0;
    if (!HilSession_controlEnabled(&g_session) &&
        (dec.system_state == SYS_MODE_ACTIVE ||
         dec.system_state == SYS_MODE_DEGRADED))
    {
        dec.system_state = HilSession_requiresFailsafe(&g_session)
                           ? SYS_MODE_FAILSAFE : SYS_MODE_STANDBY;
        g_status.system_state = dec.system_state;
        g_status.active_source = HilSession_requiresFailsafe(&g_session)
                                 ? SRC_WATCHDOG : SRC_NONE;
        if (HilSession_requiresFailsafe(&g_session))
        {
            g_status.degraded = true;
            g_status.fault_level = FAULT_LEVEL_SEVERE;
        }
    }
    Control_step(dec.system_state, cmd,
                 dec.cmd_valid && HilSession_controlEnabled(&g_session), &g_out);

    /* 7b) ASR 靠边停车看门狗：MRM 超时未停稳 → 强制 FAILSAFE；
     *     MRM 提前退出 → 重新注入。不直接覆盖控制输出，全部由
     *     Safety/Control 层经 slew 限幅处理，避免双路径冲突。 */
    if (g_asr_stop_requested)
    {
        if ((now - g_asr_stop_start_ms) >= ASR_STOP_TIMEOUT_MS)
        {
            g_asr_stop_requested = false;
            Safety_applyInject(INJ_CMD_FORCE_DEGRADE, 0U);
        }
        else if (dec.system_state != SYS_MODE_MRM)
        {
            Safety_applyInject(INJ_CMD_FORCE_MRM, 0U);
        }
    }

    /* 8) 驱动外设：舵机 / 闪光灯 / 蜂鸣器 / LED */
    buzz = Actuators_update(now, &g_out, &g_status);

    /* 9) 环路负载估算（本 tick 消耗计数 / 周期） */
    {
        uint32_t c_end = CPUTimer_getTimerCount(CPUTIMER0_BASE);
        uint32_t used  = (c_start >= c_end) ? (c_start - c_end)
                                            : (LOOP_PERIOD_COUNTS);  /* 跨界=满 */
        uint32_t pct   = (used * 100U) / LOOP_PERIOD_COUNTS;
        if (pct > 100U) { pct = 100U; }
        g_status.loop_load_pct = (uint16_t)pct;
    }

    /* 10) 子速率回传（周期对齐到 tick） */
    if ((now - s_last_control_tx_ms) >= TX_CONTROL_PERIOD_MS)
    {
        s_last_control_tx_ms = now;
        CanComm_sendControl(&g_out, &g_status, s_mcu_seq++);
    }
    if ((now - s_last_heartbeat_tx_ms) >= TX_HEARTBEAT_PERIOD_MS)
    {
        s_last_heartbeat_tx_ms = now;
        CanComm_sendHeartbeat(&g_status, s_mcu_seq, s_alive++, buzz);
        CanComm_sendSessionStatus(g_session.ack, g_session.session_id,
                                  g_session.last_sequence);
    }
    if ((now - s_last_diag_tx_ms) >= TX_DIAG_PERIOD_MS)
    {
        s_last_diag_tx_ms = now;
        CanComm_sendDiag(&g_status);
        CanComm_sendE2eDiag(&g_status);
        CanComm_sendLinkStats();
    }

    /* 11) 控制环跑完一整圈 → 喂片上看门狗（卡死则 ~0.8s 后硬复位） */
    Board_serviceWatchdog();
}

/* -------------------- 入口 -------------------- */
void main(void)
{
    uint16_t self_test_mask = 0U;
    bool self_test_ok;
    /* 设备时钟/外设默认 + 中断系统 */
    Device_init();
    Device_initGPIO();
    Interrupt_initModule();
    Interrupt_initVectorTable();

    /* 板级 + 各功能模块初始化 */
    Board_init();
    CanComm_init();
    Control_init();
    self_test_ok = SelfTest_run(&self_test_mask);
    Safety_init(Board_getResetReason(), Board_wasWatchdogReset(),
                self_test_ok, self_test_mask);
    Actuators_init();
    Hmi_init();
    HilSession_init(&g_session);
    Oled2_init();
#if DGUS_ENABLE
    Dgus_init();
#endif

    /* 安全缺省输出：不驱动、方向居中 */
    g_out.final_steer_deg = 0.0f;
    g_out.final_accel_ms2 = 0.0f;
    g_out.final_throttle  = 0.0f;
    g_out.final_brake     = 0.0f;
    g_status.system_state = SYS_MODE_INIT;
    g_status.active_source = SRC_NONE;

    /* 注册 1kHz 定时中断并放行 */
    Interrupt_register(INT_TIMER0, &cpuTimer0ISR);
    Interrupt_enable(INT_TIMER0);
    Board_startControlTimer();

    EINT;   /* 使能全局中断 */
    ERTM;   /* 使能实时调试中断 */

    /* 背景任务：等待每个 1kHz tick 到来后跑完整控制管线 */
    for (;;)
    {
        uint32_t now;
        bool overrun;

        if (!g_run)
        {
            /* OLED2/DGUS 都是尽力而为的后台外设：把软件 I2C 与 DGUS 串口屏的
             * 收发进度都放在 1kHz 控制路径之外，缺失或迟缓的显示不占用控制
             * tick 预算。 */
            DINT;
            now = g_tick_ms;
            overrun = g_run;
            EINT;
            if (!overrun)
            {
                Oled2_update(now, &g_out,
                             CanComm_getCommand(SRC_IDX_PRIMARY), &g_status,
                             CanComm_isBusHealthy());
#if DGUS_ENABLE
                Dgus_service(now);
#endif
                AsrPro_service();
                main_handleAsrCommand(now);
            }
            continue;
        }

        /* 消费本 tick（读→清标志顺序，容忍与 ISR 的极小竞态） */
        DINT;
        now = g_tick_ms;
        overrun = (g_overrun > 0U);
        g_overrun = 0U;
        g_run = false;
        EINT;

        run_tick(now, overrun);
    }
}
