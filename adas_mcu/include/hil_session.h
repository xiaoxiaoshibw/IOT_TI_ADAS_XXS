/* Cold-start HIL session gate. Pure C: target firmware and host tests share it.
 *
 * v3 会话语义重构（2026-07）：移除物理按钮/TCP 授权门控，改为 MCU 自驱动的
 * 状态机自动会把会话推进到 ACTIVE。 uphealve 安全仍由 safety.c 的 control_fresh
 * 在 FTTI 预算内保证；会话门只做"上电授权闩锁 + 灾难性故障闩锁"。
 *   BOOT_WAIT → SESSION_ACCEPTED → READY →(自动ARM浸泡)→ ARMED →(自动)→ ACTIVE
 *   链路长时间断 → RECOVERY_REQUIRED →(自动恢复浸泡)→ SESSION_ACCEPTED(同 session)
 *   灾难性故障 → FAULT_LOCK →(故障消除自动 HilSession_init)→ BOOT_WAIT → 重新握手
 * 0x104/0x206 线格式不变；上行 (SoC→MCU) PREPARE_ARM/REARM 帧不再产生效果（拒绝为
 * 非法迁移），ABORT 仍可用作测试重置。 */
#ifndef HIL_SESSION_H_
#define HIL_SESSION_H_

#include <stdbool.h>
#include <stdint.h>

#include "adas_can_protocol_v3_generated.h"

typedef struct
{
    uint16_t ack;
    uint16_t reject_reason;
    uint32_t session_id;
    uint16_t last_sequence;
    bool sequence_seen;
    bool ever_active;
    uint16_t control_stale_ticks;   /* ACTIVE 持续不新鲜计数（tick） */
    uint16_t arm_ticks;             /* READY 自动 ARM 浸泡计数（tick = ms） */
    uint16_t recover_ticks;        /* RECOVERY 自动恢复浸泡计数（tick = ms） */
} HilSession_t;

typedef struct
{
    bool control_fresh;
    bool safe_to_arm;
    bool fatal_fault;
} HilSessionInputs_t;

void HilSession_init(HilSession_t *session);

/* Process one CRC-validated 0x104 request. Returns true when accepted.
 * 重复 sequence 仅在“与当前 ack 相符的幂等重传”时按 keepalive 接受。 */
bool HilSession_handle(HilSession_t *session, uint16_t request,
                       uint32_t session_id, uint16_t sequence,
                       const HilSessionInputs_t *inputs);

/* 每 tick 自驱推进状态机（即使没有 0x104 帧到达）。 */
void HilSession_tick(HilSession_t *session,
                     const HilSessionInputs_t *inputs);

bool HilSession_controlEnabled(const HilSession_t *session);
bool HilSession_requiresFailsafe(const HilSession_t *session);

#endif /* HIL_SESSION_H_ */