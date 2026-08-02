/*
 * hil_session.c — HIL 会话门状态机实现
 *
 * 设计（v3 重构 2026-07）：会话门是“上电授权闩锁”。无需任何物理按钮或上位机
 * 业务授权——MCU 自己在收到新鲜、安全的 CAN 控制流后自动把会话推进到 ACTIVE；
 * 链路长时间断开或灾难性故障时回落，条件消除后自动恢复与重新握手。
 * 执行器停权仍由 safety.c 的 control_fresh 在 FTTI 预算内独立裁决（本模块不参与
 * 控制停权时序），所以拉长会话“断”门限不会削弱失效反应速度。
 */
#include "hil_session.h"

#include "adas_config.h"

static void reject(HilSession_t *s, uint16_t reason)
{
    s->reject_reason = reason;
}

static void reset_progress(HilSession_t *s)
{
    s->control_stale_ticks = 0U;
    s->arm_ticks = 0U;
    s->recover_ticks = 0U;
}

static bool seq_newer(const HilSession_t *s, uint16_t seq)
{
    uint16_t delta;
    if (!s->sequence_seen) { return true; }
    delta = (uint16_t)((seq - s->last_sequence) & 0x00FFU);
    return (delta > 0U && delta <= 127U);
}

static bool request_matches_ack(uint16_t request, uint16_t ack)
{
    return ((request == HIL_REQ_ANNOUNCE && ack == HIL_ACK_SESSION_ACCEPTED) ||
            (request == HIL_REQ_SYNC_STANDBY && ack == HIL_ACK_READY) ||
            (request == HIL_REQ_COMMIT_ACTIVE && ack == HIL_ACK_ARMED) ||
            ((request == HIL_REQ_COMMIT_ACTIVE || request == HIL_REQ_ACTIVE) &&
              ack == HIL_ACK_ACTIVE));
}

void HilSession_init(HilSession_t *s)
{
    s->ack = HIL_ACK_BOOT_WAIT;
    s->reject_reason = HIL_REJECT_NONE;
    s->session_id = 0UL;
    s->last_sequence = 0U;
    s->sequence_seen = false;
    s->ever_active = false;
    reset_progress(s);
}

void HilSession_tick(HilSession_t *s, const HilSessionInputs_t *in)
{
    if (in->fatal_fault)
    {
        s->ack = HIL_ACK_FAULT_LOCK;
        reject(s, HIL_REJECT_FAULT_NOT_RECOVERABLE);
        reset_progress(s);
        return;
    }

    switch (s->ack)
    {
    case HIL_ACK_READY:
        if (in->control_fresh && in->safe_to_arm)
        {
            if (s->arm_ticks < HIL_AUTO_ARM_DELAY_MS) { s->arm_ticks++; }
            if (s->arm_ticks >= HIL_AUTO_ARM_DELAY_MS)
            {
                s->ack = HIL_ACK_ARMED;
                s->arm_ticks = 0U;
            }
        }
        else { s->arm_ticks = 0U; }
        s->control_stale_ticks = 0U;
        s->recover_ticks = 0U;
        break;

    case HIL_ACK_ARMED:
        if (in->control_fresh && in->safe_to_arm)
        {
            s->ack = HIL_ACK_ACTIVE;
            s->ever_active = true;
            reset_progress(s);
        }
        break;

    case HIL_ACK_ACTIVE:
        if (in->control_fresh)
        {
            s->control_stale_ticks = 0U;
        }
        else
        {
            if (s->control_stale_ticks < HIL_SESSION_DROP_MS) { s->control_stale_ticks++; }
            if (s->control_stale_ticks >= HIL_SESSION_DROP_MS)
            {
                s->ack = HIL_ACK_RECOVERY_REQUIRED;
                reject(s, HIL_REJECT_FRAMES_NOT_FRESH);
                reset_progress(s);
            }
        }
        s->arm_ticks = 0U;
        s->recover_ticks = 0U;
        break;

    case HIL_ACK_RECOVERY_REQUIRED:
        if (in->control_fresh && in->safe_to_arm)
        {
            if (s->recover_ticks < HIL_AUTO_RECOVER_DELAY_MS) { s->recover_ticks++; }
            if (s->recover_ticks >= HIL_AUTO_RECOVER_DELAY_MS)
            {
                /* 自动恢复：回到 SESSION_ACCEPTED，保留 session_id 与序列连续性，
                 * 上行 0x206 因此对外报告重新可用的会话；网关观察到后清除其
                 * recovery 闩锁并恢复 SYNC_STANDBY，无需 REARM/新 session。 */
                s->ack = HIL_ACK_SESSION_ACCEPTED;
                reject(s, HIL_REJECT_NONE);
                reset_progress(s);
            }
        }
        else { s->recover_ticks = 0U; }
        break;

    default:
        reset_progress(s);
        break;
    }
}

bool HilSession_handle(HilSession_t *s, uint16_t request,
                       uint32_t id, uint16_t sequence,
                       const HilSessionInputs_t *in)
{
    HilSession_tick(s, in);
    if (s->ack == HIL_ACK_FAULT_LOCK) { return false; }

    if (request == HIL_REQ_ABORT)
    {
        HilSession_init(s);
        return true;
    }
    if (id == 0UL)
    {
        reject(s, HIL_REJECT_SESSION_MISMATCH);
        return false;
    }

    /* ANNOUNCE = 冷启动或对端重启的“我要新会话”帧：在任何非 FAULT_LOCK 状态都
     * 重新建立 session（含 BOOT_WAIT 的冷启动路径与网关重启路径）。幂等重传按
     * 下方的 seq 接受逻辑处理。 */
    if (request == HIL_REQ_ANNOUNCE)
    {
        if (s->sequence_seen && id == s->session_id &&
            sequence == s->last_sequence && s->ack == HIL_ACK_SESSION_ACCEPTED)
        {
            return true;
        }
        s->session_id = id;
        s->last_sequence = sequence;
        s->sequence_seen = true;
        s->ever_active = false;
        reset_progress(s);
        s->ack = HIL_ACK_SESSION_ACCEPTED;
        reject(s, HIL_REJECT_NONE);
        return true;
    }

    if (s->sequence_seen && id == s->session_id &&
        sequence == s->last_sequence && request_matches_ack(request, s->ack))
    {
        return true;
    }

    if (id != s->session_id)
    {
        reject(s, HIL_REJECT_SESSION_MISMATCH);
        return false;
    }
    if (!seq_newer(s, sequence))
    {
        reject(s, HIL_REJECT_INVALID_SEQUENCE);
        return false;
    }
    if (request_matches_ack(request, s->ack))
    {
        s->last_sequence = sequence;
        s->sequence_seen = true;
        reject(s, HIL_REJECT_NONE);
        return true;
    }

    if (s->ack == HIL_ACK_SESSION_ACCEPTED && request == HIL_REQ_SYNC_STANDBY)
    {
        if (!in->control_fresh)
        {
            reject(s, HIL_REJECT_FRAMES_NOT_FRESH);
            return false;
        }
        s->ack = HIL_ACK_READY;
    }
    else if ((s->ack == HIL_ACK_ARMED || s->ack == HIL_ACK_ACTIVE) &&
             request == HIL_REQ_COMMIT_ACTIVE)
    {
        if (!in->control_fresh || !in->safe_to_arm)
        {
            reject(s, !in->control_fresh ? HIL_REJECT_FRAMES_NOT_FRESH :
                   HIL_REJECT_UNSAFE_TO_ARM);
            return false;
        }
        s->ack = HIL_ACK_ACTIVE;
        s->ever_active = true;
        reset_progress(s);
    }
    else if (s->ack == HIL_ACK_ACTIVE && request == HIL_REQ_ACTIVE)
    {
        if (!in->control_fresh)
        {
            s->ack = HIL_ACK_RECOVERY_REQUIRED;
            reject(s, HIL_REJECT_FRAMES_NOT_FRESH);
            return false;
        }
    }
    else
    {
        /* PREPARE_ARM / REARM 等旧授权帧在新模型中无语义 → 非法迁移。
         * 旧 sequence 不前进（避免消耗序列号预算）。 */
        reject(s, HIL_REJECT_INVALID_TRANSITION);
        return false;
    }

    s->last_sequence = sequence;
    s->sequence_seen = true;
    reject(s, HIL_REJECT_NONE);
    return true;
}

bool HilSession_controlEnabled(const HilSession_t *s)
{
    return (s->ack == HIL_ACK_ACTIVE);
}

bool HilSession_requiresFailsafe(const HilSession_t *s)
{
    return (s->ever_active && s->ack != HIL_ACK_ACTIVE);
}