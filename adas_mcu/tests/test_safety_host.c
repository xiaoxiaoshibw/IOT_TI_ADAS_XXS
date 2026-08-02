#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "safety.h"
#include "can_comm.h"
#include "adas_can_protocol.h"
#include "adas_config.h"

static SocCommand_t commands[SRC_IDX_COUNT];
static LinkMonitor_t links[SRC_IDX_COUNT];

const SocCommand_t *CanComm_getCommand(uint16_t idx)
{
    return &commands[idx];
}

void CanComm_clearSafetyRequests(uint16_t idx)
{
    SocCommand_t *c = &commands[idx];
    c->emergency_stop = false;
    c->aeb_risk = AEB_RISK_NONE;
    c->aeb_required_decel = 0.0f;
    c->mrm_request = false;
    c->obstacle_valid = false;
}

LinkMonitor_t *CanComm_getLink(uint16_t idx)
{
    return &links[idx];
}

static void reset_fixture(void)
{
    memset(commands, 0, sizeof(commands));
    memset(links, 0, sizeof(links));
    Safety_init(0U, false, true, 0U);
}

static void make_fresh(uint16_t idx, uint32_t now)
{
    SocCommand_t *c = &commands[idx];
    LinkMonitor_t *l = &links[idx];
    l->seq_seen = l->lat_seen = l->lon_seen = l->status_seen = true;
    l->protocol_ok = true;
    l->lat_valid = l->lon_valid = l->status_valid = true;
    l->last_hb_ms = l->last_lat_ms = l->last_lon_ms = l->last_status_ms = now;
    c->soc_health = 1U;
    c->control_authority = true;
    c->status_word = ST_CONTROL_ENABLE;
}

static McuStatus_t step(uint32_t now)
{
    McuStatus_t status;
    SafetyDecision_t decision;
    memset(&status, 0, sizeof(status));
    Safety_evaluateLinks(now, 0U);
    Safety_step(now, false, &status, &decision);
    return status;
}

/* 让某源仅"安全新鲜"（心跳+状态可信）而不"控制新鲜"：不设横/纵向帧，
 * 模拟 HIL 未起/已停时 SoC 仍发心跳+状态（可承载 AEB/急停/MRM 软请求），
 * 但正常控制源仲裁返回 INVALID。 */
static void make_safety_fresh(uint16_t idx, uint32_t now)
{
    LinkMonitor_t *l = &links[idx];
    SocCommand_t *c = &commands[idx];
    l->seq_seen = true;
    l->status_seen = true;
    l->protocol_ok = true;
    l->status_valid = true;
    l->last_hb_ms = now;
    l->last_status_ms = now;
    c->soc_health = 1U;
}

/* 注入 HIL 会话授权只读快照（等价 main.c 每 tick 在 Safety_step 前所做）。 */
static void set_session(bool authorized, bool ever_active,
                        bool normal_shutdown, uint32_t epoch)
{
    SafetySessionContext_t ctx;
    ctx.control_authorized = authorized;
    ctx.ever_active        = ever_active;
    ctx.normal_shutdown    = normal_shutdown;
    ctx.session_epoch      = epoch;
    Safety_setSessionContext(&ctx);
}

static void test_cold_start_never_brakes(void)
{
    reset_fixture();
    assert(step(0U).system_state == SYS_MODE_INIT);
    assert(step(INIT_SETTLE_MS + 1U).system_state == SYS_MODE_STANDBY);
}

static void test_heartbeat_alone_is_not_a_complete_source(void)
{
    reset_fixture();
    links[0].seq_seen = true;
    links[0].last_hb_ms = 10U;
    commands[0].soc_health = 1U;
    commands[0].control_authority = true;
    assert(step(10U).system_state == SYS_MODE_INIT);
    assert(!links[0].control_fresh);
}

static void test_protocol_mismatch_is_rejected(void)
{
    reset_fixture();
    make_fresh(0U, 10U);
    links[0].protocol_ok = false;
    assert(step(10U).system_state == SYS_MODE_INIT);
    assert(!links[0].control_fresh);
}

static void test_status_timeout_clears_safety_bits(void)
{
    reset_fixture();
    make_fresh(0U, 100U);
    commands[0].emergency_stop = true;
    commands[0].aeb_risk = AEB_RISK_FULL;
    commands[0].obstacle_valid = true;
    (void)step(100U);
    links[0].last_status_ms = 100U;
    links[0].last_hb_ms = links[0].last_lat_ms = links[0].last_lon_ms = 230U;
    Safety_evaluateLinks(230U, 0U);
    assert(!commands[0].emergency_stop);
    assert(commands[0].aeb_risk == AEB_RISK_NONE);
    assert(!commands[0].obstacle_valid);
}

#if REDUNDANCY_ENABLE
static void test_backup_estop_cannot_be_suppressed_by_primary(void)
{
    reset_fixture();
    make_fresh(0U, 100U);
    make_fresh(1U, 100U);
    commands[1].aeb_risk = AEB_RISK_FULL;
    commands[1].obstacle_valid = true;
    assert(step(100U).system_state == SYS_MODE_EMERGENCY_BRAKE);
}

static void test_backup_estop_survives_partial_control_loss(void)
{
    reset_fixture();
    make_fresh(0U, 200U);
    make_fresh(1U, 200U);
    commands[1].aeb_risk = AEB_RISK_FULL;
    commands[1].obstacle_valid = true;
    /* 备源横向帧过期，但心跳和安全状态仍可信。 */
    links[1].last_lat_ms = 0U;
    links[1].lat_seq_err_cnt = SEQ_STALL_LIMIT;
    assert(step(200U).system_state == SYS_MODE_EMERGENCY_BRAKE);
    assert(!links[1].control_fresh);
    assert(links[1].safety_fresh);
}

static void test_failback_requires_stable_primary(void)
{
    McuStatus_t status;
    reset_fixture();
    make_fresh(1U, 100U);
    status = step(100U);
    assert(status.active_source == SRC_BACKUP);

    make_fresh(0U, 200U);
    make_fresh(1U, 200U);
    status = step(200U);
    assert(status.active_source == SRC_BACKUP);

    make_fresh(0U, 200U + FAILBACK_HOLD_MS + 1U);
    make_fresh(1U, 200U + FAILBACK_HOLD_MS + 1U);
    /* Preserve the start of the continuous-primary freshness interval. */
    links[0].fresh_since_ms = 200U;
    links[0].fresh_since_valid = true;
    status = step(200U + FAILBACK_HOLD_MS + 1U);
    assert(status.active_source == SRC_PRIMARY);
}

static void test_freshness_started_at_zero_is_not_reinitialized(void)
{
    McuStatus_t status;
    reset_fixture();
    make_fresh(1U, 0U);
    status = step(0U);
    assert(status.active_source == SRC_BACKUP);
    assert(links[1].fresh_since_valid);
    assert(links[1].fresh_since_ms == 0U);

    make_fresh(0U, 1U);
    make_fresh(1U, 1U);
    status = step(1U);
    assert(status.active_source == SRC_BACKUP);

    make_fresh(0U, FAILBACK_HOLD_MS + 2U);
    make_fresh(1U, FAILBACK_HOLD_MS + 2U);
    status = step(FAILBACK_HOLD_MS + 2U);
    assert(status.active_source == SRC_PRIMARY);
}
#endif

static void test_single_primary_does_not_degrade_when_backup_is_disabled(void)
{
    McuStatus_t status;
    reset_fixture();
    make_fresh(SRC_IDX_PRIMARY, INIT_SETTLE_MS + 10U);
    status = step(INIT_SETTLE_MS + 10U);
#if REDUNDANCY_ENABLE
    assert(status.system_state == SYS_MODE_DEGRADED);
#else
    assert(status.system_state == SYS_MODE_ACTIVE);
    assert((status.fault_code & FC_BACKUP_TIMEOUT) == 0U);
#endif
}

static void test_bus_off_lock_requires_recovery_and_explicit_clear(void)
{
    McuStatus_t status;
    reset_fixture();
    make_fresh(0U, 10U);
    Safety_setCanState(false, true);
    status = step(10U);
    assert(status.system_state == SYS_MODE_FAULT_LOCK);
    assert((status.fault_code & FC_CAN_BUS_OFF) != 0U);

    Safety_requestClear();
    status = step(11U);
    assert(status.system_state == SYS_MODE_FAULT_LOCK);

    Safety_setCanState(true, false);
    Safety_requestClear();
    make_fresh(0U, 12U);
    status = step(12U);
    assert(status.system_state != SYS_MODE_FAULT_LOCK);
}

static void test_clear_defers_unlock_and_requests_recovery(void)
{
    McuStatus_t status;
    reset_fixture();
    make_fresh(0U, 10U);
    Safety_setCanState(false, true);
    status = step(10U);
    assert(status.system_state == SYS_MODE_FAULT_LOCK);

    /* 清故障抬起一次性强制恢复请求，供 main 驱动 CanComm_forceRecovery。 */
    Safety_requestClear();
    assert(Safety_consumeRecoveryRequest());
    assert(!Safety_consumeRecoveryRequest());   /* 一次性 */

    /* 本 tick 总线仍不健康 → 解闩延后裁决，保持 FAULT_LOCK（消费恢复请求
     * 不影响待定的清故障意图）。 */
    status = step(11U);
    assert(status.system_state == SYS_MODE_FAULT_LOCK);

    /* main 已强制恢复 → 总线健康；同一次清故障意图在本 tick 解闩，无需第二次按键。*/
    Safety_setCanState(true, false);
    Safety_requestClear();
    make_fresh(0U, 12U);
    status = step(12U);
    assert(status.system_state != SYS_MODE_FAULT_LOCK);
}

static void test_ftti_budget_constants_are_consistent(void)
{
    assert(CTRL_TIMEOUT_MS + SAFETY_ARBITRATION_BUDGET_MS + SAFETY_OUTPUT_BUDGET_MS <= FTTI_CONTROL_LOSS_MS);
    assert(HB_TIMEOUT_MS + SAFETY_ARBITRATION_BUDGET_MS + SAFETY_OUTPUT_BUDGET_MS <= FTTI_HEARTBEAT_LOSS_MS);
    assert(SAFETY_ARBITRATION_BUDGET_MS + SAFETY_OUTPUT_BUDGET_MS <= FTTI_E2E_REJECT_MS);
    assert(SAFETY_ARBITRATION_BUDGET_MS + SAFETY_OUTPUT_BUDGET_MS <= FTTI_AEB_MS);
    assert(SAFETY_ARBITRATION_BUDGET_MS + SAFETY_OUTPUT_BUDGET_MS <= FTTI_MRM_REQUEST_MS);
}

static void test_mixed_control_cycles_are_not_eligible_for_arbitration(void)
{
    McuStatus_t status;
    reset_fixture();
    make_fresh(0U, 10U);
    links[0].last_lat_seq = 21U;
    links[0].last_lon_seq = 20U;
    status = step(10U);
    assert(!links[0].control_fresh);
    assert(status.active_source == SRC_NONE);

    links[0].last_lon_seq = 21U;
    status = step(11U);
    assert(links[0].control_fresh);
    assert(status.active_source == SRC_PRIMARY);
}

static void test_pair_skew_is_tolerated_for_two_continuous_ticks(void)
{
    McuStatus_t status;
    reset_fixture();
    make_fresh(0U, 10U);
    links[0].last_lat_seq = 20U;
    links[0].last_lon_seq = 20U;
    status = step(10U);
    assert(links[0].control_fresh);
    assert(status.active_source == SRC_PRIMARY);

    /* 下一周期横向帧刚好在仲裁前到达，纵向帧仍在同一 CAN 轮询批次中。 */
    links[0].last_lat_seq = 21U;
    links[0].last_lat_ms = 11U;
    status = step(11U);
    assert(links[0].control_fresh);
    assert(status.active_source == SRC_PRIMARY);
    assert(status.primary_timeout_cnt == 0U);

    status = step(12U);
    assert(links[0].control_fresh);
    assert(status.active_source == SRC_PRIMARY);
    assert(status.primary_timeout_cnt == 0U);

    /* 第三个连续 tick 仍未成对，属于真实失配，必须退出仲裁。 */
    status = step(13U);
    assert(!links[0].control_fresh);
    assert(status.active_source == SRC_NONE);
    assert(status.primary_timeout_cnt == 1U);

    links[0].last_lon_seq = 21U;
    links[0].last_lon_ms = 14U;
    status = step(14U);
    assert(links[0].control_fresh);
    assert(status.active_source == SRC_PRIMARY);
}

/* §B3 补的回归：超过 grace 的连续 tick 不能"偷偷再 grace"，
 * 必须等两源同 seq 出现重置对账状态后才能再 grace。 */
static void test_pair_skew_does_not_re_grace_within_window(void)
{
    McuStatus_t status;
    reset_fixture();
    make_fresh(0U, 10U);

    /* 先做一次同步对账（让 s_pair_sync_seen[idx]=true，进入 grace 路径）。 */
    links[0].last_lat_seq = 20U;
    links[0].last_lon_seq = 20U;
    status = step(10U);
    assert(links[0].control_fresh);

    /* 第 1-2-3 个连续 tick 持续失配：grace 结束、退出仲裁。 */
    links[0].last_lat_seq = 21U;
    links[0].last_lat_ms = 11U;
    (void)step(11U);
    (void)step(12U);
    status = step(13U);
    assert(!links[0].control_fresh);

    /* 后面 4-5 两个 tick 仍然失配（且失配 delta > 1），必须保持不可信。
     * 注意：primary_timeout_cnt 是"丢失边缘"边沿触发（只在 fresh→
     * not-fresh 那一刻 +1），连续多个 not-fresh tick 不再递增。 */
    links[0].last_lat_seq = 23U;
    links[0].last_lat_ms = 14U;
    status = step(14U);
    assert(!links[0].control_fresh);
    status = step(15U);
    assert(!links[0].control_fresh);
    assert(status.primary_timeout_cnt == 1U);

    /* 同 seq 对齐后重新开口：seq_delta==0 路径立即恢复 fresh，
     * 不强制走新的 grace 窗口。 */
    links[0].last_lat_seq = 30U;
    links[0].last_lon_seq = 30U;
    links[0].last_lat_ms = 16U; links[0].last_lon_ms = 16U;
    status = step(16U);
    assert(links[0].control_fresh);
    assert(status.active_source == SRC_PRIMARY);
}

static void test_pair_skew_grace_does_not_mask_timeout_or_real_mismatch(void)
{
    reset_fixture();
    make_fresh(0U, 10U);
    links[0].last_lat_seq = links[0].last_lon_seq = 20U;
    (void)step(10U);

    links[0].last_lat_seq = 22U;
    assert(step(11U).active_source == SRC_NONE);
    assert(!links[0].control_fresh);

    /* 重新建立相干性后，即使 seq 仅相邻，物理超时仍立即生效。 */
    links[0].last_lat_seq = links[0].last_lon_seq = 23U;
    make_fresh(0U, 20U);
    links[0].last_lat_seq = links[0].last_lon_seq = 23U;
    (void)step(20U);
    links[0].last_lat_seq = 24U;
    links[0].last_lat_ms = CTRL_TIMEOUT_MS + 21U;
    links[0].last_lon_ms = 20U;
    assert(step(CTRL_TIMEOUT_MS + 21U).active_source == SRC_NONE);
    assert(!links[0].control_fresh);
}

static void test_watchdog_reset_requires_explicit_clear(void)
{
    McuStatus_t status;
    memset(commands, 0, sizeof(commands));
    memset(links, 0, sizeof(links));
    Safety_init(1U, true, true, 0U);
    make_fresh(0U, 10U);
    status = step(10U);
    assert(status.system_state == SYS_MODE_FAULT_LOCK);
    assert((status.fault_code & FC_WATCHDOG_RESET) != 0U);
    Safety_requestClear();
    status = step(11U);
    assert(status.system_state != SYS_MODE_FAULT_LOCK);
}

static void test_self_test_failure_cannot_be_cleared(void)
{
    McuStatus_t status;
    memset(commands, 0, sizeof(commands));
    memset(links, 0, sizeof(links));
    Safety_init(0U, false, false, 1U);
    make_fresh(0U, 10U);
    status = step(10U);
    assert(status.system_state == SYS_MODE_FAULT_LOCK);
    assert((status.fault_code & FC_SELF_TEST) != 0U);
    assert(status.self_test_mask == 1U);
    Safety_requestClear();
    status = step(11U);
    assert(status.system_state == SYS_MODE_FAULT_LOCK);
}

static void test_inject_drop_all_enters_failsafe(void)
{
    McuStatus_t status;
    /* INIT 稳定窗从首个 tick 起算，双源丢失判定要在窗后验证 */
    const uint32_t t_drop = 2U * INIT_SETTLE_MS + 20U;
    reset_fixture();
    make_fresh(0U, 10U);
    make_fresh(1U, 10U);
    status = step(10U);
    assert(status.system_state == SYS_MODE_ACTIVE);

    Safety_applyInject(INJ_CMD_DROP_ALL, 0U);
    make_fresh(0U, t_drop);
    make_fresh(1U, t_drop);
    status = step(t_drop);
    assert(status.system_state == SYS_MODE_FAILSAFE);
    assert(status.active_source == SRC_WATCHDOG);
    assert((status.fault_code & FC_ALL_SOURCES_LOST) != 0U);

    Safety_applyInject(INJ_CMD_CLEAR, 0U);
    make_fresh(0U, t_drop + 1U);
    make_fresh(1U, t_drop + 1U);
    status = step(t_drop + 1U);
    assert(status.system_state == SYS_MODE_ACTIVE);
}

static void test_inject_force_mrm_and_clear(void)
{
    McuStatus_t status;
    reset_fixture();
    make_fresh(0U, INIT_SETTLE_MS + 10U);
    status = step(INIT_SETTLE_MS + 10U);
    assert(status.system_state != SYS_MODE_MRM);

    Safety_applyInject(INJ_CMD_FORCE_MRM, 0U);
    make_fresh(0U, INIT_SETTLE_MS + 11U);
    status = step(INIT_SETTLE_MS + 11U);
    assert(status.system_state == SYS_MODE_MRM);
    assert(status.degraded);

    Safety_applyInject(INJ_CMD_CLEAR, 0U);
    make_fresh(0U, INIT_SETTLE_MS + 12U);
    status = step(INIT_SETTLE_MS + 12U);
    assert(status.system_state != SYS_MODE_MRM);
}

static void test_inject_can_exhausted_locks_and_clears_as_simulation(void)
{
    McuStatus_t status;
    reset_fixture();
    make_fresh(0U, 10U);
    Safety_applyInject(INJ_CMD_CAN_EXHAUSTED, 0U);
    make_fresh(0U, 11U);
    status = step(11U);
    assert(status.system_state == SYS_MODE_FAULT_LOCK);
    assert((status.fault_code & FC_CAN_BUS_OFF) != 0U);
    assert(!links[0].control_fresh);    /* 模拟耗尽期间任何源都不可信 */

    /* 模拟故障可以被注入 CLEAR 撤销；真实耗尽由
     * test_bus_off_lock_requires_recovery_and_explicit_clear 保证不可撤销 */
    Safety_applyInject(INJ_CMD_CLEAR, 0U);
    make_fresh(0U, 12U);
    status = step(12U);
    assert(status.system_state != SYS_MODE_FAULT_LOCK);
}

static void test_inject_self_test_fail_locks_and_clears_as_simulation(void)
{
    McuStatus_t status;
    reset_fixture();
    make_fresh(0U, 10U);
    Safety_applyInject(INJ_CMD_SELF_TEST_FAIL, 0U);
    make_fresh(0U, 11U);
    status = step(11U);
    assert(status.system_state == SYS_MODE_FAULT_LOCK);
    assert((status.fault_code & FC_SELF_TEST) != 0U);

    Safety_applyInject(INJ_CMD_CLEAR, 0U);
    make_fresh(0U, 12U);
    status = step(12U);
    assert(status.system_state != SYS_MODE_FAULT_LOCK);
}

static void test_real_can_exhaustion_survives_inject_clear(void)
{
    McuStatus_t status;
    reset_fixture();
    make_fresh(0U, 10U);
    Safety_setCanState(false, true);
    Safety_applyInject(INJ_CMD_CLEAR, 0U);
    status = step(11U);
    assert(status.system_state == SYS_MODE_FAULT_LOCK);
}

/* ================= HIL 会话门 / 外部软 MRM 授权 回归矩阵 =================
 * 见 safety.c Safety_step 的软/硬安全请求分离 + Safety_setSessionContext。
 * 核心不变式：会话未 ACTIVE（control_authorized=false）或当前会话未建立 mrm=0
 * 零基线时，来自 SoC 状态帧的 mrm_request=1 绝不触发 SYS_MODE_MRM；而急停 /
 * AEB / 本地强制 MRM 不受该门限制，运行中掉线仍进 FAILSAFE 而非退回 STANDBY。 */

/* 测试1：会话未 ACTIVE（BOOT_WAIT/READY）时，控制新鲜源上的残留 mrm=1 不触发 MRM。
 * 旧代码：src_mrm=1 → MRM；新代码：会话门 + 无零基线双重拦截 → 名义控制。 */
static void test_mrm_blocked_before_session_active(void)
{
    reset_fixture();
    set_session(false, false, false, 1U);   /* 未授权控制 */
    make_fresh(0U, INIT_SETTLE_MS + 10U);
    commands[0].mrm_request = true;          /* SoC 残留高电平 MRM */
    assert(step(INIT_SETTLE_MS + 10U).system_state != SYS_MODE_MRM);
}

/* 测试2/3：ARMED——即便当前会话已见 mrm=0 零基线，只要会话仍未授权控制，
 * 后续 mrm=1 依然不得触发 MRM（隔离出 control_authorized 这道门本身）。 */
static void test_mrm_blocked_unauthorized_despite_zero_baseline(void)
{
    reset_fixture();
    set_session(false, false, false, 1U);
    make_fresh(0U, INIT_SETTLE_MS + 10U);
    commands[0].mrm_request = false;         /* 建立零基线 */
    assert(step(INIT_SETTLE_MS + 10U).system_state != SYS_MODE_MRM);
    make_fresh(0U, INIT_SETTLE_MS + 11U);
    commands[0].mrm_request = true;          /* 基线已满足，但仍未授权 */
    assert(step(INIT_SETTLE_MS + 11U).system_state != SYS_MODE_MRM);
}

/* 测试4：新会话已 ACTIVE，但从未看到 mrm=0（mrm 始终为 1）→ 不触发 MRM。 */
static void test_mrm_blocked_without_zero_baseline(void)
{
    reset_fixture();
    set_session(true, false, false, 2U);     /* 已授权，新会话 epoch=2 */
    make_fresh(0U, INIT_SETTLE_MS + 10U);
    commands[0].mrm_request = true;          /* 上一会话残留，一直为 1 */
    assert(step(INIT_SETTLE_MS + 10U).system_state != SYS_MODE_MRM);
    make_fresh(0U, INIT_SETTLE_MS + 11U);
    commands[0].mrm_request = true;
    assert(step(INIT_SETTLE_MS + 11U).system_state != SYS_MODE_MRM);
}

/* 测试5：当前 ACTIVE 会话，合法 0→1 → 进入 MRM。 */
static void test_mrm_allowed_after_zero_to_one(void)
{
    reset_fixture();
    set_session(true, false, false, 3U);
    make_fresh(0U, INIT_SETTLE_MS + 10U);
    commands[0].mrm_request = false;         /* 先建立零基线 */
    assert(step(INIT_SETTLE_MS + 10U).system_state != SYS_MODE_MRM);
    make_fresh(0U, INIT_SETTLE_MS + 11U);
    commands[0].mrm_request = true;          /* 合法请求 */
    assert(step(INIT_SETTLE_MS + 11U).system_state == SYS_MODE_MRM);
}

/* 测试6：会话未授权时 AEB 全制动仍立即生效（绕过会话门）。 */
static void test_aeb_bypasses_session_gate(void)
{
    reset_fixture();
    set_session(false, false, false, 1U);
    make_safety_fresh(0U, INIT_SETTLE_MS + 10U);
    commands[0].aeb_risk = AEB_RISK_FULL;
    commands[0].obstacle_valid = true;
    assert(step(INIT_SETTLE_MS + 10U).system_state == SYS_MODE_EMERGENCY_BRAKE);
}

/* 测试7：会话未授权时 emergency_stop 仍立即生效（绕过会话门）。 */
static void test_estop_bypasses_session_gate(void)
{
    reset_fixture();
    set_session(false, false, false, 1U);
    make_safety_fresh(0U, INIT_SETTLE_MS + 10U);
    commands[0].emergency_stop = true;
    assert(step(INIT_SETTLE_MS + 10U).system_state == SYS_MODE_EMERGENCY_BRAKE);
}

/* 测试8：曾进入 ACTIVE 后异常掉线（控制帧超时、会话失授权、无正常 DISARM）
 * → FAILSAFE，绝不退回无保护 STANDBY。 */
static void test_active_session_loss_enters_failsafe(void)
{
    McuStatus_t s1;
    reset_fixture();
    set_session(true, true, false, 1U);
    make_fresh(0U, INIT_SETTLE_MS + 10U);
    s1 = step(INIT_SETTLE_MS + 10U);         /* 先 engage */
    assert(s1.system_state == SYS_MODE_ACTIVE || s1.system_state == SYS_MODE_DEGRADED);
    /* 掉线：不再授权，控制帧陈旧 → idx=INVALID（不再 make_fresh 刷新时间戳） */
    set_session(false, true, false, 1U);
    assert(step(INIT_SETTLE_MS + 610U).system_state == SYS_MODE_FAILSAFE);
}

/* 测试9：合法正常停机（normal_shutdown）后允许回 STANDBY。当前 0x104 协议无
 * DISARM/STOP 帧，main.c 恒传 normal_shutdown=false，此为钩子级验证。 */
static void test_normal_shutdown_allows_standby(void)
{
    reset_fixture();
    set_session(true, true, false, 1U);
    make_fresh(0U, INIT_SETTLE_MS + 10U);
    (void)step(INIT_SETTLE_MS + 10U);        /* engage */
    set_session(false, true, true, 1U);      /* normal_shutdown=true */
    assert(step(INIT_SETTLE_MS + 610U).system_state == SYS_MODE_STANDBY);
}

/* 测试10：测试/本地强制 MRM 注入即使会话未授权也生效（不受会话门限制）。 */
static void test_force_mrm_injection_bypasses_gate(void)
{
    reset_fixture();
    set_session(false, false, false, 1U);    /* 未授权 */
    Safety_applyInject(INJ_CMD_FORCE_MRM, 0U);
    make_fresh(0U, INIT_SETTLE_MS + 10U);
    assert(step(INIT_SETTLE_MS + 10U).system_state == SYS_MODE_MRM);
}

/* 测试12：会话 epoch 变化清除零基线——新会话必须重新先看到 mrm=0。 */
static void test_session_epoch_change_clears_baseline(void)
{
    reset_fixture();
    set_session(true, false, false, 4U);     /* epoch=4 */
    make_fresh(0U, INIT_SETTLE_MS + 10U);
    commands[0].mrm_request = false;         /* epoch4 建立零基线 */
    assert(step(INIT_SETTLE_MS + 10U).system_state != SYS_MODE_MRM);
    make_fresh(0U, INIT_SETTLE_MS + 11U);
    commands[0].mrm_request = true;
    assert(step(INIT_SETTLE_MS + 11U).system_state == SYS_MODE_MRM);   /* epoch4 合法 */
    set_session(true, false, false, 5U);     /* 世代切换 epoch=5 */
    make_fresh(0U, INIT_SETTLE_MS + 12U);
    commands[0].mrm_request = true;          /* 新会话首帧即 1，无基线 */
    assert(step(INIT_SETTLE_MS + 12U).system_state != SYS_MODE_MRM);
}

/* 测试13：跨会话世代不得继承旧会话的"曾运行"标记。s_ever_engaged 是开机级
 * 闩锁，历史上只在 Safety_init（MCU 复位）时清零；真实场景中 hil_session.c
 * 在收到新 ANNOUNCE 时会把 ever_active 清 false（新会话/GUI 重启 CARLA、
 * bridge 触发），但若 s_ever_engaged 不随会话世代同步清除，旧会话曾 ACTIVE
 * 过时，新会话启动瞬间的双源未就绪（完全正常的握手前窗口）会被误判为
 * "运行中失效"而直接进 FAILSAFE，而不是期望的 STANDBY。
 * 对应 PROJECT_STATUS.md AUD-P1-MCU-001 / CLAUDE.md 会话门约定。 */
static void test_new_session_epoch_clears_stale_ever_engaged(void)
{
    McuStatus_t s1;
    reset_fixture();
    set_session(true, true, false, 1U);
    make_fresh(0U, INIT_SETTLE_MS + 10U);
    s1 = step(INIT_SETTLE_MS + 10U);          /* 旧会话 engage */
    assert(s1.system_state == SYS_MODE_ACTIVE || s1.system_state == SYS_MODE_DEGRADED);

    /* 新会话（epoch 切换）：ever_active 已被 ANNOUNCE 语义重置为 false；
     * 控制帧尚未到达是新会话的正常瞬态，不是运行中丢失。 */
    set_session(true, false, false, 2U);
    assert(step(INIT_SETTLE_MS + 620U).system_state == SYS_MODE_STANDBY);
}

/* 测试14：HilSession 达到 ACTIVE（会话 ever_active=true）本身不得触发
 * FAILSAFE。HilSession 的自驱 ACTIVE 只要求 CAN 协议/时序合法（main.c
 * in.control_fresh），不检查 SocCommand.control_authority；command_gate 未选中
 * follower 时（例如尚无导航路线，仍在 builtin_stop），会话可以在 SoC 侧
 * control_authority 恒 false、Safety 自身仲裁从未选出过有效源的情况下自驱到
 * ACTIVE。若把这个会话级信号当成"车辆曾被真实接管"，车辆会在从未参与过控制
 * 时就被判 FAILSAFE——这正是"GUI 启动 CARLA 后、导航路线设置前"立即 MRM/
 * FAILSAFE 的实测根因（真实 HIL 上直接观测：session ack=ACTIVE 但
 * primary_timeout_count 全程为 0，即 control_fresh 从未真正为 true 过）。
 * 本测试从不调用 make_fresh，模拟 Safety 自身仲裁从未验证过任何源；只有
 * s_ever_engaged（由 Safety 自身仲裁置位，要求 control_authority）才是正确
 * 判据，此时必须停在 STANDBY。 */
static void test_session_active_alone_does_not_force_failsafe(void)
{
    reset_fixture();
    set_session(true, true, false, 1U);   /* 会话已 ACTIVE，但从未 make_fresh */
    assert(step(0U).system_state == SYS_MODE_INIT);   /* 盖章 s_init_ms，先过 INIT */
    assert(step(INIT_SETTLE_MS + 1U).system_state == SYS_MODE_STANDBY);
}

#if REDUNDANCY_ENABLE
/* 测试11：主源已见 mrm=0 不得授权备源遗留的 mrm=1（每源基线互不污染）。
 * REDUNDANCY_ENABLE=0 时不编译（当前配置为单源，非污染语义无意义）。 */
static void test_backup_residual_mrm_not_polluted_by_primary_zero(void)
{
    reset_fixture();
    set_session(true, false, false, 6U);
    make_fresh(SRC_IDX_PRIMARY, INIT_SETTLE_MS + 10U);
    make_fresh(SRC_IDX_BACKUP,  INIT_SETTLE_MS + 10U);
    commands[SRC_IDX_PRIMARY].mrm_request = false;   /* 主源零基线 */
    commands[SRC_IDX_BACKUP].mrm_request  = true;    /* 备源遗留 1，从未见 0 */
    assert(step(INIT_SETTLE_MS + 10U).system_state != SYS_MODE_MRM);
}
#endif

static void test_planned_shutdown_signal(void)
{
    SocCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.soc_health = 1U;
    cmd.control_authority = true;
    cmd.soc_mode = SYS_MODE_STANDBY;
    assert(Safety_isPlannedShutdown(&cmd));
    cmd.control_authority = false;
    assert(!Safety_isPlannedShutdown(&cmd));
    cmd.control_authority = true;
    cmd.mrm_request = true;
    assert(!Safety_isPlannedShutdown(&cmd));
    cmd.mrm_request = false;
    cmd.soc_mode = SYS_MODE_ACTIVE;
    assert(!Safety_isPlannedShutdown(&cmd));
}

int main(void)
{
    test_cold_start_never_brakes();
    test_heartbeat_alone_is_not_a_complete_source();
    test_protocol_mismatch_is_rejected();
    test_status_timeout_clears_safety_bits();
#if REDUNDANCY_ENABLE
    test_backup_estop_cannot_be_suppressed_by_primary();
    test_backup_estop_survives_partial_control_loss();
    test_failback_requires_stable_primary();
    test_freshness_started_at_zero_is_not_reinitialized();
#endif
    test_single_primary_does_not_degrade_when_backup_is_disabled();
    test_pair_skew_does_not_re_grace_within_window();
    test_bus_off_lock_requires_recovery_and_explicit_clear();
    test_clear_defers_unlock_and_requests_recovery();
    test_ftti_budget_constants_are_consistent();
    test_mixed_control_cycles_are_not_eligible_for_arbitration();
    test_pair_skew_is_tolerated_for_two_continuous_ticks();
    test_pair_skew_grace_does_not_mask_timeout_or_real_mismatch();
    test_watchdog_reset_requires_explicit_clear();
    test_self_test_failure_cannot_be_cleared();
    test_inject_drop_all_enters_failsafe();
    test_inject_force_mrm_and_clear();
    test_inject_can_exhausted_locks_and_clears_as_simulation();
    test_inject_self_test_fail_locks_and_clears_as_simulation();
    test_real_can_exhaustion_survives_inject_clear();
    /* HIL 会话门 / 外部软 MRM 授权 回归矩阵 */
    test_mrm_blocked_before_session_active();
    test_mrm_blocked_unauthorized_despite_zero_baseline();
    test_mrm_blocked_without_zero_baseline();
    test_mrm_allowed_after_zero_to_one();
    test_aeb_bypasses_session_gate();
    test_estop_bypasses_session_gate();
    test_active_session_loss_enters_failsafe();
    test_normal_shutdown_allows_standby();
    test_force_mrm_injection_bypasses_gate();
    test_session_epoch_change_clears_baseline();
    test_new_session_epoch_clears_stale_ever_engaged();
    test_session_active_alone_does_not_force_failsafe();
    test_planned_shutdown_signal();
#if REDUNDANCY_ENABLE
    test_backup_residual_mrm_not_polluted_by_primary_zero();
#endif
    puts("safety host tests: PASS");
    return 0;
}
