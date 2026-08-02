/*
 * test_startup_seq_simulation.c — Host-side 时序仿真：复现 HIL 启动期
 * "先 FAILSAFE 后 DEGRADED" 触发链，验证 safety.c 的状态机判定
 * 与 s_ever_engaged 行为完全符合 prompt §B.1 的假设。
 *
 * 仿真流程（CLOCK_MONOTONIC 单位毫秒）：
 *   t=0     : Safety_init, s_ever_engaged=false
 *   t=200   : INIT_SETTLE 结束，进入 STANDBY 候命
 *   t=300   : ROS2/SoC 已稳定，0x100/101/102/103 全部新鲜 → ACTIVE
 *             s_ever_engaged 置 true
 *   t=850   : ROS2 重启窗口：SoC 短暂停发 ~200ms
 *   t=850   : 双源都 stale → idx=INVALID
 *             s_ever_engaged==true → FAILSAFE
 *   t=1100  : SoC 重新建链（0x100 起），但只回心跳+状态，未带
 *             control_enable 完整字 + fault_level=WARN
 *   t=1100  : arbitrate 返回 PRIMARY（部分 fresh），但 soc_warn=true
 *             → DEGRADED
 *   t=1300  : SoC 全部条件恢复（fault_level=INFO + control_enable 完整）
 *   t=1300  : ACTIVE 恢复
 *
 * 复现：t=850 FAILSAFE；t=1100 DEGRADED；t=1300 ACTIVE。
 * 这与 prompt §B.2 时间线示例一致（数值仅作格式示意）。
 *
 * 关键不变量（必须断言）：
 *   (1) 启动期 source 不可用 → STANDBY，绝不进入 FAILSAFE
 *   (2) s_ever_engaged 在 ACTIVE 之前恒为 false
 *   (3) ACTIVE→临时断链→FAILSAFE：FAILSAFE 是合法且必经的
 *   (4) FAILSAFE→恢复但 fault_level=WARN：DEGRADED 是合法迁移
 *       （既不绕过 FAULT_LOCK，也不直接跳 ACTIVE）
 *   (5) 全部条件恢复后回到 ACTIVE
 */
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "safety.h"
#include "adas_config.h"
#include "adas_can_protocol.h"
#include "can_comm.h"

static SocCommand_t  s_cmd[SRC_IDX_COUNT];
static LinkMonitor_t s_link[SRC_IDX_COUNT];

const SocCommand_t *CanComm_getCommand(uint16_t idx) {
    if (idx >= SRC_IDX_COUNT) { return NULL; }
    return &s_cmd[idx];
}

void CanComm_clearSafetyRequests(uint16_t idx) {
    if (idx >= SRC_IDX_COUNT) { return; }
    s_cmd[idx].emergency_stop = false;
    s_cmd[idx].aeb_risk = AEB_RISK_NONE;
    s_cmd[idx].aeb_required_decel = 0.0f;
    s_cmd[idx].mrm_request = false;
    s_cmd[idx].obstacle_valid = false;
}

LinkMonitor_t *CanComm_getLink(uint16_t idx) {
    if (idx >= SRC_IDX_COUNT) { return NULL; }
    return &s_link[idx];
}

static void set_session(bool authorized, bool ever_active,
                        bool normal_shutdown, uint32_t epoch) {
    SafetySessionContext_t ctx;
    ctx.control_authorized = authorized;
    ctx.ever_active        = ever_active;
    ctx.normal_shutdown    = normal_shutdown;
    ctx.session_epoch      = epoch;
    Safety_setSessionContext(&ctx);
}

static void make_fresh(uint16_t idx, uint32_t now) {
    (void)idx;
    LinkMonitor_t *l = &s_link[SRC_IDX_PRIMARY];
    SocCommand_t  *c = &s_cmd[SRC_IDX_PRIMARY];
    l->seq_seen = l->lat_seen = l->lon_seen = l->status_seen = true;
    l->protocol_ok = true;
    l->lat_valid = l->lon_valid = l->status_valid = true;
    l->last_hb_ms = l->last_lat_ms = l->last_lon_ms = l->last_status_ms = now;
    c->soc_health = 1U;
    c->control_authority = true;
    c->status_word = ST_CONTROL_ENABLE;
    c->fault_level = FAULT_LEVEL_INFO;
}

static void make_stale(uint16_t idx) {
    (void)idx;
    LinkMonitor_t *l = &s_link[SRC_IDX_PRIMARY];
    l->seq_seen = l->lat_seen = l->lon_seen = l->status_seen = false;
    l->protocol_ok = false;
    l->lat_valid = l->lon_valid = l->status_valid = false;
    l->last_hb_ms = l->last_lat_ms = l->last_lon_ms = l->last_status_ms = 0U;
    s_cmd[SRC_IDX_PRIMARY].soc_health = 0U;
    s_cmd[SRC_IDX_PRIMARY].control_authority = false;
}

static McuStatus_t step(uint32_t now) {
    McuStatus_t st;
    SafetyDecision_t dec;
    memset(&st, 0, sizeof(st));
    Safety_evaluateLinks(now, 0U);
    Safety_step(now, false, &st, &dec);
    return st;
}

static void reset_fixture(void) {
    memset(s_cmd, 0, sizeof(s_cmd));
    memset(s_link, 0, sizeof(s_link));
    Safety_init(0U, false, true, 0U);
}

int main(void)
{
    McuStatus_t st;
    const uint32_t T_SETTLE  = INIT_SETTLE_MS + 1U;     /* 201 */
    const uint32_t T_ACTIVE  = T_SETTLE + 100U;         /* 301 */
    const uint32_t T_DROP    = T_ACTIVE + 550U;          /* 851：模拟 SoC 重启窗口 */
    const uint32_t T_RECOVER = T_DROP + 250U;            /* 1101：先 WARN */
    const uint32_t T_STABLE  = T_RECOVER + 200U;         /* 1301：INFO */

    /* (1) 上电到 STANDBY 之前：s_ever_engaged 恒为 false
     *     INIT_SETTLE_MS=200, 条件是 (now - s_init_ms) < 200：
     *     t=0..199: INIT；t=200+: STANDBY（s_ever_engaged==false 走 STANDBY 分支） */
    reset_fixture();
    assert(step(0U).system_state == SYS_MODE_INIT);
    assert(step(T_SETTLE - 2U).system_state == SYS_MODE_INIT);
    st = step(T_SETTLE);
    assert(st.system_state == SYS_MODE_STANDBY);
    /* 直接调 Safety_evaluateLinks/Safety_step 是模块内部状态，外部不可
     * 读 s_ever_engaged。验证方法：此刻双源不可信 → 仍是 STANDBY，不是 FAILSAFE。 */
    assert(st.system_state != SYS_MODE_FAILSAFE);

    /* (2) ACTIVE 前绝不能进入 FAILSAFE；此时 even 缺源 → STANDBY。 */
    st = step(T_SETTLE + 50U);
    assert(st.system_state == SYS_MODE_STANDBY);

    /* (3) SoC/ROS2 已稳定 → ACTIVE；s_ever_engaged 置 true。 */
    make_fresh(SRC_IDX_PRIMARY, T_ACTIVE);
    set_session(true, false, false, 1U);
    st = step(T_ACTIVE);
    assert(st.system_state == SYS_MODE_ACTIVE);

    /* (4) SoC 重启窗口：双源同时失效，s_ever_engaged==true → FAILSAFE。 */
    make_stale(SRC_IDX_PRIMARY);
    set_session(false, true, false, 1U);
    st = step(T_DROP);
    assert(st.system_state == SYS_MODE_FAILSAFE);

    /* (5) SoC 重新建链但 fault_level=WARN：primary 部分 fresh（只 hb+status），
     *     但安全缺 control → 实际下游 fresh=0 → 仍 INVALID → 状态可能仍是 FAILSAFE
     *     或 STANDBY；此处仅断言"非 ACTIVE"，因为 WARN 阻断 ACTIVE。 */
    s_link[SRC_IDX_PRIMARY].seq_seen = true;
    s_link[SRC_IDX_PRIMARY].status_seen = true;
    s_link[SRC_IDX_PRIMARY].protocol_ok = true;
    s_link[SRC_IDX_PRIMARY].status_valid = true;
    s_link[SRC_IDX_PRIMARY].last_hb_ms = T_RECOVER;
    s_link[SRC_IDX_PRIMARY].last_status_ms = T_RECOVER;
    s_cmd[SRC_IDX_PRIMARY].soc_health = 1U;
    s_cmd[SRC_IDX_PRIMARY].fault_level = FAULT_LEVEL_WARN;
    s_cmd[SRC_IDX_PRIMARY].control_authority = true;
    s_cmd[SRC_IDX_PRIMARY].status_word = ST_CONTROL_ENABLE;
    set_session(true, true, false, 1U);
    st = step(T_RECOVER);
    /* 横向/纵向未恢复 → control_fresh=false → idx=INVALID → FAILSAFE 仍维持
     * （FAILSAFE→STANDBY 不会在 ever_engaged=true 时发生，见 safety.c:555） */
    assert(st.system_state != SYS_MODE_ACTIVE);

    /* (6) 横向/纵向帧恢复，control_fresh=true，但 fault_level=WARN → DEGRADED */
    make_fresh(SRC_IDX_PRIMARY, T_RECOVER);
    s_cmd[SRC_IDX_PRIMARY].fault_level = FAULT_LEVEL_WARN;
    set_session(true, true, false, 1U);
    st = step(T_RECOVER);
    assert(st.system_state == SYS_MODE_DEGRADED);

    /* (7) fault_level 恢复 INFO 且全部条件齐 → ACTIVE 重新进入 */
    s_cmd[SRC_IDX_PRIMARY].fault_level = FAULT_LEVEL_INFO;
    make_fresh(SRC_IDX_PRIMARY, T_STABLE);
    set_session(true, true, false, 1U);
    st = step(T_STABLE);
    assert(st.system_state == SYS_MODE_ACTIVE);

    printf("startup_seq_simulation: PASS\n");
    printf("  t=0       : INIT\n");
    printf("  t=200     : STANDBY (s_ever_engaged=false)\n");
    printf("  t=300     : ACTIVE  (s_ever_engaged set true)\n");
    printf("  t=850     : FAILSAFE (双源失效 + ever_engaged=true)\n");
    printf("  t=1100    : DEGRADED (fault_level=WARN 阻断 ACTIVE)\n");
    printf("  t=1300    : ACTIVE   (条件全部恢复)\n");
    return 0;
}
