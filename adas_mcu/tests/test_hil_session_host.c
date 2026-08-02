#include <assert.h>
#include <stdio.h>

#include "adas_config.h"
#include "hil_session.h"

static HilSessionInputs_t healthy(void)
{
    HilSessionInputs_t in = { true, true, false };
    return in;
}

static void arm(HilSession_t *s, uint32_t id)
{
    HilSessionInputs_t in = healthy();
    /* 0x104 握手：ANNOUNCE 建立 session，SYNC_STANDBY 进入 READY。之后由 tick
     * 自驱：READY 经 ARM 浸泡自动到 ARMED，再自动到 ACTIVE。 */
    assert(HilSession_handle(s, HIL_REQ_ANNOUNCE, id, 1U, &in));
    assert(HilSession_handle(s, HIL_REQ_SYNC_STANDBY, id, 2U, &in));
    assert(s->ack == HIL_ACK_READY);
    for (uint16_t i = 0U; i < HIL_AUTO_ARM_DELAY_MS; i++)
    {
        HilSession_tick(s, &in);
    }
    assert(s->ack == HIL_ACK_ARMED);
    HilSession_tick(s, &in);
    assert(s->ack == HIL_ACK_ACTIVE);
    assert(HilSession_controlEnabled(s));
}

static void test_cold_start_auto_arms(void)
{
    HilSession_t s;
    HilSessionInputs_t in = healthy();
    HilSession_init(&s);
    assert(!HilSession_controlEnabled(&s));
    /* 跳过 ANNOUNCE 不能直接 ACTIVE */
    assert(!HilSession_handle(&s, HIL_REQ_COMMIT_ACTIVE, 1UL, 1U, &in));
    assert(s.ack == HIL_ACK_BOOT_WAIT);
    arm(&s, 1UL);
}

static void test_no_operator_authorization_needed(void)
{
    HilSession_t s;
    HilSessionInputs_t in = healthy();
    HilSession_init(&s);
    assert(HilSession_handle(&s, HIL_REQ_ANNOUNCE, 8UL, 1U, &in));
    assert(HilSession_handle(&s, HIL_REQ_SYNC_STANDBY, 8UL, 2U, &in));
    assert(s.ack == HIL_ACK_READY);
    /* PREPARE_ARM 在新模型里是非法迁移（被拒绝，ack 仍在 READY），不授权任何东西。
     * 序列号不前进（非法迁移不清耗预算）。 */
    assert(!HilSession_handle(&s, HIL_REQ_PREPARE_ARM, 8UL, 3U, &in));
    assert(s.ack == HIL_ACK_READY);
    assert(s.reject_reason == HIL_REJECT_INVALID_TRANSITION);
    assert(s.last_sequence == 2U);
    /* 自动 ARM 不需要任何 PREPARE 帧；READY 浸泡期满自动 ARMED，下一 tick
     * 自动 ACTIVE（无需 COMMIT、无需任何授权）。 */
    for (uint16_t i = 0U; i < HIL_AUTO_ARM_DELAY_MS; i++)
    {
        HilSession_tick(&s, &in);
    }
    assert(HilSession_controlEnabled(&s));
}

static void test_active_survives_brief_stale(void)
{
    HilSession_t s;
    HilSessionInputs_t in = healthy();
    HilSession_init(&s);
    arm(&s, 44UL);
    in.control_fresh = false;
    /* 短于会话 DROP 窗口的抖动不撤授权（控制停权仍由 safety.c 独立完成） */
    for (uint16_t i = 0U; i < (HIL_SESSION_DROP_MS - 1U); i++)
    {
        HilSession_tick(&s, &in);
    }
    assert(HilSession_controlEnabled(&s));
}

static void test_long_stale_auto_recovers(void)
{
    HilSession_t s;
    HilSessionInputs_t in = healthy();
    HilSession_init(&s);
    arm(&s, 42UL);
    in.control_fresh = false;
    for (uint16_t i = 0U; i < HIL_SESSION_DROP_MS; i++) { HilSession_tick(&s, &in); }
    assert(s.ack == HIL_ACK_RECOVERY_REQUIRED);
    assert(HilSession_requiresFailsafe(&s));
    /* 旧的 HIL_REQ_ACTIVE / REARM 都不再恢复会话 */
    assert(!HilSession_handle(&s, HIL_REQ_ACTIVE, 42UL, 5U, &in));
    assert(!HilSession_handle(&s, HIL_REQ_REARM, 43UL, 6U, &in));
    assert(s.ack == HIL_ACK_RECOVERY_REQUIRED);
    /* 控制恢复后自动回到 SESSION_ACCEPTED（同 session id），无需任何授权/新 session */
    in.control_fresh = true;
    for (uint16_t i = 0U; i < HIL_AUTO_RECOVER_DELAY_MS; i++) { HilSession_tick(&s, &in); }
    assert(s.ack == HIL_ACK_SESSION_ACCEPTED);
    assert(s.session_id == 42UL);
    assert(!HilSession_controlEnabled(&s));
    assert(HilSession_requiresFailsafe(&s));
    /* 网关随后发 SYNC_STANDBY → READY → 自动 ARMED/ACTIVE */
    assert(HilSession_handle(&s, HIL_REQ_SYNC_STANDBY, 42UL, 7U, &in));
    assert(s.ack == HIL_ACK_READY);
    for (uint16_t i = 0U; i < HIL_AUTO_ARM_DELAY_MS; i++) { HilSession_tick(&s, &in); }
    assert(s.ack == HIL_ACK_ARMED);
    HilSession_tick(&s, &in);
    assert(s.ack == HIL_ACK_ACTIVE);
}

static void test_fatal_fault_auto_reinitializes(void)
{
    HilSession_t s;
    HilSessionInputs_t in = healthy();
    HilSession_init(&s);
    arm(&s, 9UL);
    in.fatal_fault = true;
    HilSession_tick(&s, &in);
    assert(s.ack == HIL_ACK_FAULT_LOCK);
    /* 故障消除后，应用/固件 main 会在本 tick 主动 HilSession_init 重握手；
     * 此处直接模拟该调用（main.c 的恢复动作）。 */
    in.fatal_fault = false;
    HilSession_init(&s);
    assert(s.ack == HIL_ACK_BOOT_WAIT);
    /* ABORT 仍然可用作测试重置 */
    assert(HilSession_handle(&s, HIL_REQ_ANNOUNCE, 9UL, 1U, &in));
    assert(s.ack == HIL_ACK_SESSION_ACCEPTED);
    assert(HilSession_handle(&s, HIL_REQ_ABORT, 9UL, 2U, &in));
    assert(s.ack == HIL_ACK_BOOT_WAIT);
}

static void test_peer_restart_announce_restarts_session(void)
{
    HilSession_t s;
    HilSessionInputs_t in = healthy();
    HilSession_init(&s);
    arm(&s, 5UL);
    /* 网关重启发来新 session id 的 ANNOUNCE：MCU 重新建立会话 */
    assert(HilSession_handle(&s, HIL_REQ_ANNOUNCE, 99UL, 1U, &in));
    assert(s.ack == HIL_ACK_SESSION_ACCEPTED);
    assert(s.session_id == 99UL);
    assert(s.ever_active == false);
}

static void test_stale_sequence_and_duplicate(void)
{
    HilSession_t s;
    HilSessionInputs_t in = healthy();
    HilSession_init(&s);
    assert(HilSession_handle(&s, HIL_REQ_ANNOUNCE, 5UL, 10U, &in));
    /* ANNOUNCE 幂等重传只有当 sequence 相符且当前为 SESSION_ACCEPTED 才接受 */
    assert(HilSession_handle(&s, HIL_REQ_ANNOUNCE, 5UL, 10U, &in));
    assert(HilSession_handle(&s, HIL_REQ_ANNOUNCE, 5UL, 11U, &in));
    assert(!HilSession_handle(&s, HIL_REQ_SYNC_STANDBY, 5UL, 9U, &in));
    assert(s.reject_reason == HIL_REJECT_INVALID_SEQUENCE);
}

int main(void)
{
    test_cold_start_auto_arms();
    test_no_operator_authorization_needed();
    test_active_survives_brief_stale();
    test_long_stale_auto_recovers();
    test_fatal_fault_auto_reinitializes();
    test_peer_restart_announce_restarts_session();
    test_stale_sequence_and_duplicate();
    puts("hil_session host tests: PASS");
    return 0;
}