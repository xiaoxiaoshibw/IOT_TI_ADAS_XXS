/*
 * safety.h — 安全状态机 + 主备仲裁 + 链路新鲜度裁决
 *
 * 这是 MCU 的"安全监督器 / Command Gate 等价物"：无源可信时决定降级/失效，
 * 双源之间做仲裁与一键热冗余锁定，并把 AEB/急停映射为系统态。
 * 控制数值不在这里算（那是 control.c）；这里只产出"用哪个源、进哪个态"。
 */
#ifndef SAFETY_H_
#define SAFETY_H_

#include <stdint.h>
#include <stdbool.h>
#include "adas_types.h"

/* 故障码位图（McuStatus_t.fault_code / 0x203 上报） */
#define FC_PRIMARY_TIMEOUT   (1U << 0)
#define FC_BACKUP_TIMEOUT    (1U << 1)
#define FC_PRIMARY_SEQ_STALL (1U << 2)   /* 报文仍到但 seq 停滞：上游控制环挂起 */
#define FC_BACKUP_SEQ_STALL  (1U << 3)
#define FC_ALL_SOURCES_LOST  (1U << 4)
#define FC_CRC_ERRORS        (1U << 5)
#define FC_SOC_FAULT         (1U << 6)   /* SoC 自报 FATAL */
#define FC_LOOP_OVERRUN      (1U << 7)
#define FC_ESTOP_ACTIVE      (1U << 8)
#define FC_PROTO_MISMATCH    (1U << 9)
#define FC_FAULT_LOCK        (1U << 10)
#define FC_INJECT_LOCK       FC_FAULT_LOCK
#define FC_CAN_BUS_OFF       (1U << 11)
#define FC_WATCHDOG_RESET    (1U << 12)
#define FC_SELF_TEST         (1U << 13)

/* 仲裁 + 状态机每 tick 的裁决结果 */
typedef struct
{
    uint16_t system_state;   /* SYS_MODE_* */
    uint16_t active_source;  /* SRC_*（1 主 / 2 备 / 9 看门狗 / 0 无） */
    uint16_t active_idx;     /* SRC_IDX_*，无源时 = SRC_IDX_INVALID */
    bool     cmd_valid;      /* active_idx 是否指向一个可信命令 */
} SafetyDecision_t;

#define SRC_IDX_INVALID  0xFFFFU

/* HIL 会话授权的只读快照。由 main.c 每 tick 在 Safety_step 前注入。
 * safety.c 不拥有、不驱动 HIL 会话状态机（唯一实例是 main.c 的 g_session）——
 * 这里只读一份最小信息，用来给"外部软安全请求 MRM"加会话门：只有会话已授权
 * 控制（HilSession ack==ACTIVE）、且当前会话已建立 mrm=0 零基线后，来自 SoC
 * 状态帧的 mrm_request=1 才被采信为合法 MRM。硬安全请求（急停 / AEB 全制动 /
 * 本地强制 MRM）不受此门限制，FTTI 路径不增加任何会话依赖。 */
typedef struct
{
    bool     control_authorized;  /* 会话已授权控制（HilSession ack==ACTIVE） */
    bool     ever_active;         /* 会话是否曾进入 ACTIVE（掉线后判 FAILSAFE 用） */
    bool     normal_shutdown;     /* 合法 DISARM/停机后受控停车（当前协议无此帧，恒 false） */
    uint32_t session_epoch;       /* 会话世代（由 HilSession.session_id 承担）；变化即重建零基线 */
} SafetySessionContext_t;

void Safety_init(uint16_t reset_reason, bool watchdog_reset,
                 bool self_test_ok, uint16_t self_test_mask);
void Safety_setCanState(bool healthy, bool recovery_exhausted);

/* 注入 HIL 会话授权上下文（只读快照，每 tick 在 Safety_step 前调用）。
 * 传 NULL 忽略。从不调用时默认 control_authorized=false —— fail-closed：
 * 外部 MRM 静默，直到会话进入 ACTIVE 并建立零基线。 */
void Safety_setSessionContext(const SafetySessionContext_t *ctx);
bool Safety_isPlannedShutdown(const SocCommand_t *cmd);

/* 每 1ms：结合各源时间戳/seq 停滞刷新 LinkMonitor.fresh/control_fresh。
 * crc_err_this_tick 为本 tick CanComm 报告的 CRC 失败次数（累加进诊断）。*/
void Safety_evaluateLinks(uint32_t now_ms, uint16_t crc_err_this_tick);

/* 每 1ms：仲裁选源 + 推进状态机；写 McuStatus，返回裁决。
 * loop_overrun 为本 tick 是否发生控制环超时（用于诊断/看门狗关联）。*/
void Safety_step(uint32_t now_ms, bool loop_overrun,
                 McuStatus_t *st, SafetyDecision_t *dec);

/* ---- HMI：一键主备热冗余（v3 后仅长按清故障保留语义；短按由 hmi.c 自处理） ---- */
void Safety_requestSwitchSource(void);  /* 仅 REDUNDANCY_ENABLE 时由短按触发 */
/* 长按：清故障 + 解除手动锁定 + 回 AUTO。同时抬起一次"强制 CAN 恢复"请求，
 * 供 main 消费去驱动 CanComm_forceRecovery；FAULT_LOCK 的最终解闩延后到本
 * tick 的 Safety_step，用当次刷新后的 CAN 状态判定（保证一次长按即恢复）。*/
void Safety_requestClear(void);
/* main 每 tick 消费一次：若上一次清故障抬起了强制恢复请求则返回 true 并清标志。*/
bool Safety_consumeRecoveryRequest(void);
bool Safety_isManualOverride(void);
uint16_t Safety_overrideSource(void);   /* SRC_PRIMARY / SRC_BACKUP */

/* ---- 故障注入（0x301）---- */
void Safety_applyInject(uint16_t cmd, uint16_t param);

#endif /* SAFETY_H_ */
