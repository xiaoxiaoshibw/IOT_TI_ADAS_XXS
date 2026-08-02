/*
 * safety.c — 安全状态机 + 主备仲裁实现
 *
 * 新鲜度（control_fresh）判据（任一不满足即该源不可作控制源）：
 *   - 横向帧、纵向帧、心跳帧都在各自超时窗内到达（物理未断链）；
 *   - 心跳 seq 未停滞（seq_stall_cnt < SEQ_STALL_LIMIT，防"报文还在发但控制环挂死"）；
 *   - SoC 自报 soc_health>0 且 control_authority=1（允许自动）。
 *
 * 状态机（优先级由高到低，每 tick 重算目标态）：
 *   FAULT_LOCK(闩锁) > EMERGENCY_BRAKE(急停/AEB全制动) > FAILSAFE(双源全失)
 *   > MRM(本地/测试强制，或"会话已授权+零基线"后的外部软请求)
 *   > DEGRADED(仅备源/单源/SoC告警/seq停滞) > ACTIVE > STANDBY > INIT
 *
 * 硬安全 vs 软安全：急停 / AEB 全制动 / 本地强制 MRM 为硬请求，任何会话态下立即
 * 生效；来自 SoC 状态帧的 mrm_request 为软请求，绑定 HIL 会话授权——只有会话
 * ACTIVE 且当前会话已建立 mrm=0 零基线后才采信，见 Safety_setSessionContext。
 *
 * 一键热冗余（v3 重构后）：短按在手势层无语义（会话已自驱授权，不再由按钮 ARM）；
 *   长按 = 清故障 + 请求 CAN 重新入网（本地诊断辅助），并复位会话闩锁。FAULT_LOCK
 *   在故障条件消除后由 Safety_step 每滴答自动解闩，无需长按/断电。
 */
#include "safety.h"
#include "can_comm.h"
#include "adas_config.h"
#include "adas_can_protocol.h"

/* 内部状态 */
static uint16_t s_state          = SYS_MODE_INIT;
static uint32_t s_init_ms        = 0U;
static bool     s_init_stamped   = false;

static bool     s_manual         = false;         /* 手动锁定源 */
static uint16_t s_override_idx   = SRC_IDX_PRIMARY;

static bool     s_fault_lock     = false;         /* 闩锁态（v3 后：条件消除自动解闩，注入锁除外） */
static bool     s_inj_lock        = false;        /* 测试注入的显式闩锁，需 INJ_CMD_CLEAR/长按解除 */
static bool     s_clear_pending  = false;         /* 已请求清故障，待 Safety_step 用当次 CAN 状态解闩 */
static bool     s_recovery_req   = false;         /* 请求 main 驱动 CanComm_forceRecovery（一次性） */
static uint16_t s_fault_code     = 0U;
static bool     s_ever_engaged   = false;
#if REDUNDANCY_ENABLE
static uint16_t s_active_idx     = SRC_IDX_INVALID;
#endif
static uint32_t s_emergency_since_ms = 0U;
static bool     s_can_healthy     = true;
static bool     s_can_recovery_exhausted = false;
static bool     s_watchdog_reset  = false;
static bool     s_self_test_ok    = true;
static uint16_t s_self_test_mask  = 0U;

/* 诊断计数（饱和 255） */
static uint16_t s_pri_to_cnt     = 0U;
static uint16_t s_bak_to_cnt     = 0U;
static uint16_t s_crc_err_cnt    = 0U;
static uint16_t s_loop_ovr_cnt   = 0U;
static uint16_t s_reset_reason   = 0U;

/* 上一 tick 各源 control_fresh，用于捕捉"丢失边缘"累加超时计数 */
static bool     s_prev_pri_fresh = false;
#if REDUNDANCY_ENABLE
static bool     s_prev_bak_fresh = false;
#endif

/* HIL 会话授权只读快照 + 外部软安全请求（MRM）会话门状态。
 *   s_session_ctx        : 由 Safety_setSessionContext 每 tick 更新；默认 fail-closed
 *                          （control_authorized=false → 外部 MRM 静默）。
 *   s_last_session_epoch : 已建立零基线所属的会话世代（HilSession.session_id）。
 *   s_mrm_zero_seen[i]   : 源 i 是否已在当前会话内看到过一次"新鲜 mrm=0"（零基线）。
 * 会话世代变化、或某源不再 safety_fresh 时，清除对应零基线：新会话/重连后必须
 * 先重新观测到 mrm=0，之后的 mrm=1 才算合法软请求（防旧会话残留高电平污染）。 */
static SafetySessionContext_t s_session_ctx = { false, false, false, 0U };
static uint32_t s_last_session_epoch  = 0U;
static bool     s_session_epoch_valid = false;
static bool     s_mrm_zero_seen[SRC_IDX_COUNT];

/* 横/纵向帧在 CAN 轮询与 1 ms 仲裁边界两侧到达时，最新 seq 会短暂相差 1。
 * 只有先建立过同 seq 相干性，才允许相邻 seq 使用短窗口；窗口按连续 tick
 * 和绝对时间双重限制，避免真正的帧丢失被反复当作边界竞态。 */
static bool     s_pair_sync_seen[SRC_IDX_COUNT];
static bool     s_pair_skew_active[SRC_IDX_COUNT];
static uint16_t s_pair_skew_ticks[SRC_IDX_COUNT];
static uint32_t s_pair_skew_since_ms[SRC_IDX_COUNT];
static uint32_t s_pair_skew_last_eval_ms[SRC_IDX_COUNT];

/* 注入的强制态 */
static bool     s_force_degrade  = false;
static bool     s_force_estop    = false;
static bool     s_force_mrm      = false;         /* 测试：模拟可信源 MRM 请求 */
static bool     s_drop_primary   = false;         /* 测试：忽略主源 */
static bool     s_drop_backup    = false;

/* 注入的模拟硬件故障。与真实状态分开保存：INJ_CMD_CLEAR 只能撤销模拟故障，
 * 真实的 Bus-Off 恢复耗尽或自检失败仍必须满足各自的解除条件。 */
static bool     s_inj_can_exhausted  = false;
static bool     s_inj_self_test_fail = false;

static bool can_healthy_now(void)   { return s_can_healthy && !s_inj_can_exhausted; }
static bool self_test_ok_now(void)  { return s_self_test_ok && !s_inj_self_test_fail; }

static void sat_inc(uint16_t *c) { if (*c < 0xFFU) { (*c)++; } }

void Safety_init(uint16_t reset_reason, bool watchdog_reset,
                 bool self_test_ok, uint16_t self_test_mask)
{
    uint16_t i;
    s_state = SYS_MODE_INIT;
    s_init_stamped = false;
    s_manual = false;
    s_override_idx = SRC_IDX_PRIMARY;
    /* v3：FAULT_LOCK 改为每 tick 按“当前是否真的存在故障条件”自动裁决，不再
     * 在 init 时直接置位（看门狗复位/自检失败改为 boot-hold 计时后在 step 解除）。
     * 唯一需要显式清除的是测试注入的闩锁。 */
    s_fault_lock = false;
    s_inj_lock = false;
    s_clear_pending = false;
    s_recovery_req = false;
    s_fault_code = 0U;
    s_ever_engaged = false;
#if REDUNDANCY_ENABLE
    s_active_idx = SRC_IDX_INVALID;
#endif
    s_emergency_since_ms = 0U;
    s_can_healthy = true;
    s_can_recovery_exhausted = false;
    s_watchdog_reset = watchdog_reset;
    s_self_test_ok = self_test_ok;
    s_self_test_mask = self_test_mask;
    s_pri_to_cnt = s_bak_to_cnt = s_crc_err_cnt = s_loop_ovr_cnt = 0U;
    s_prev_pri_fresh = false;
#if REDUNDANCY_ENABLE
    s_prev_bak_fresh = false;
#endif
    for (i = 0U; i < SRC_IDX_COUNT; i++)
    {
        s_pair_sync_seen[i] = false;
        s_pair_skew_active[i] = false;
        s_pair_skew_ticks[i] = 0U;
        s_pair_skew_since_ms[i] = 0U;
        s_pair_skew_last_eval_ms[i] = 0U;
        s_mrm_zero_seen[i] = false;
    }
    s_force_degrade = s_force_estop = s_force_mrm = false;
    s_drop_primary = s_drop_backup = false;
    s_inj_can_exhausted = s_inj_self_test_fail = false;
    s_reset_reason = reset_reason;
    /* 会话门复位：默认无授权、无零基线；世代待首个 Safety_step 采样。 */
    s_session_ctx.control_authorized = false;
    s_session_ctx.ever_active        = false;
    s_session_ctx.normal_shutdown    = false;
    s_session_ctx.session_epoch      = 0U;
    s_last_session_epoch  = 0U;
    s_session_epoch_valid = false;
}

void Safety_setSessionContext(const SafetySessionContext_t *ctx)
{
    if (ctx != 0) { s_session_ctx = *ctx; }
}

bool Safety_isPlannedShutdown(const SocCommand_t *cmd)
{
    return cmd != 0 && cmd->soc_health > 0U && cmd->control_authority &&
           cmd->soc_mode == SYS_MODE_STANDBY && !cmd->mrm_request &&
           !cmd->emergency_stop;
}

void Safety_setCanState(bool healthy, bool recovery_exhausted)
{
    s_can_healthy = healthy;
    s_can_recovery_exhausted = recovery_exhausted;
    /* v3：bus-off 耗尽会立即触发本 tick 的 FAULT_LOCK（由 step 评估）；总线恢复
     * 健康后 recovery_exhausted 转假，step 中的自动解闩随之生效。 */
    if (recovery_exhausted) { s_fault_lock = true; }
}

/* -------- 链路新鲜度评估 -------- */
static bool control_pair_coherent(uint16_t idx, const LinkMonitor_t *l,
                                  uint32_t now, bool pair_inputs_ok)
{
    uint16_t seq_delta;

    if (!pair_inputs_ok)
    {
        s_pair_sync_seen[idx] = false;
        s_pair_skew_active[idx] = false;
        return false;
    }

    seq_delta = (uint16_t)((l->last_lat_seq - l->last_lon_seq) & 0x00FFU);
    if (seq_delta == 0U)
    {
        s_pair_sync_seen[idx] = true;
        s_pair_skew_active[idx] = false;
        s_pair_skew_ticks[idx] = 0U;
        return true;
    }

    if (!s_pair_sync_seen[idx] || (seq_delta != 1U && seq_delta != 0x00FFU))
    {
        s_pair_sync_seen[idx] = false;
        s_pair_skew_active[idx] = false;
        return false;
    }

    if (!s_pair_skew_active[idx])
    {
        s_pair_skew_active[idx] = true;
        s_pair_skew_ticks[idx] = 1U;
        s_pair_skew_since_ms[idx] = now;
    }
    else if ((now - s_pair_skew_last_eval_ms[idx]) == 1U)
    {
        if (s_pair_skew_ticks[idx] < 0xFFFFU) { s_pair_skew_ticks[idx]++; }
    }
    else if (now != s_pair_skew_last_eval_ms[idx])
    {
        /* 评估并非连续 1 ms tick，不能证明错位只存在于边界窗口。 */
        s_pair_sync_seen[idx] = false;
        return false;
    }
    s_pair_skew_last_eval_ms[idx] = now;

    if (s_pair_skew_ticks[idx] <= CONTROL_PAIR_SKEW_GRACE_TICKS &&
        (now - s_pair_skew_since_ms[idx]) < CONTROL_PAIR_SKEW_GRACE_MS)
    {
        return true;
    }

    s_pair_sync_seen[idx] = false;
    return false;
}

static void eval_one(uint16_t idx, LinkMonitor_t *l, const SocCommand_t *c,
                     uint32_t now, bool dropped)
{
    bool lat_ok = l->lat_seen && ((now - l->last_lat_ms) <= CTRL_TIMEOUT_MS);
    bool lon_ok = l->lon_seen && ((now - l->last_lon_ms) <= CTRL_TIMEOUT_MS);
    bool hb_ok  = l->seq_seen && ((now - l->last_hb_ms)  <= HB_TIMEOUT_MS);
    bool status_ok = l->status_seen && ((now - l->last_status_ms) <= STATUS_TIMEOUT_MS);
    bool seq_ok = (l->hb_seq_err_cnt < SEQ_STALL_LIMIT) &&
                  (l->lat_seq_err_cnt < SEQ_STALL_LIMIT) &&
                  (l->lon_seq_err_cnt < SEQ_STALL_LIMIT) &&
                  (l->status_seq_err_cnt < SEQ_STALL_LIMIT);
    bool pair_inputs_ok = lat_ok && lon_ok && l->lat_valid && l->lon_valid &&
                          (l->lat_seq_err_cnt < SEQ_STALL_LIMIT) &&
                          (l->lon_seq_err_cnt < SEQ_STALL_LIMIT) &&
                          l->protocol_ok && !dropped;
    bool coherent = control_pair_coherent(idx, l, now, pair_inputs_ok);
    bool safety_seq_ok = (l->hb_seq_err_cnt < SEQ_STALL_LIMIT) &&
                         (l->status_seq_err_cnt < SEQ_STALL_LIMIT);
    bool soc_ok = (c->soc_health > 0U) && c->control_authority;

    l->fresh = (hb_ok && seq_ok && !dropped);
    /* 安全请求与名义控制解耦：横向或纵向帧丢失时，只要固定源 ID、协议、
     * 心跳和状态帧仍可信，就不能屏蔽该源的 AEB/急停/MRM 请求。 */
    l->safety_fresh = (hb_ok && status_ok && l->status_valid && safety_seq_ok &&
                       l->protocol_ok && (c->soc_health > 0U) && !dropped);
    l->control_fresh = (lat_ok && lon_ok && hb_ok && status_ok && coherent &&
                         l->lat_valid && l->lon_valid && l->status_valid && seq_ok &&
                         l->protocol_ok && soc_ok && !dropped);

    if (!status_ok || dropped)
    {
        CanComm_clearSafetyRequests(idx);
    }

    if (l->control_fresh)
    {
        if (!l->fresh_since_valid)
        {
            l->fresh_since_ms = now;
            l->fresh_since_valid = true;
        }
    }
    else
    {
        l->fresh_since_ms = 0U;
        l->fresh_since_valid = false;
    }
}

void Safety_evaluateLinks(uint32_t now_ms, uint16_t crc_err_this_tick)
{
    LinkMonitor_t *lp = CanComm_getLink(SRC_IDX_PRIMARY);
    LinkMonitor_t *lb = CanComm_getLink(SRC_IDX_BACKUP);
    const SocCommand_t *cp = CanComm_getCommand(SRC_IDX_PRIMARY);
    const SocCommand_t *cb = CanComm_getCommand(SRC_IDX_BACKUP);

    eval_one(SRC_IDX_PRIMARY, lp, cp, now_ms,
             s_drop_primary || !can_healthy_now());
    eval_one(SRC_IDX_BACKUP, lb, cb, now_ms,
             s_drop_backup || !can_healthy_now());

    if (crc_err_this_tick > 0U)
    {
        uint16_t k;
        for (k = 0U; k < crc_err_this_tick && s_crc_err_cnt < 0xFFU; k++)
        {
            s_crc_err_cnt++;
        }
    }

    /* 丢失边缘：control_fresh 由真变假 → 累加对应源超时事件 */
    if (s_prev_pri_fresh && !lp->control_fresh) { sat_inc(&s_pri_to_cnt); }
#if REDUNDANCY_ENABLE
    if (s_prev_bak_fresh && !lb->control_fresh) { sat_inc(&s_bak_to_cnt); }
#endif
    s_prev_pri_fresh = lp->control_fresh;
#if REDUNDANCY_ENABLE
    s_prev_bak_fresh = lb->control_fresh;
#endif
}

/* -------- 选源仲裁 --------
 * 返回采信源 idx（SRC_IDX_INVALID 表示无可信源）。
 * 规则：手动锁定优先用锁定源；锁定源失新鲜则回退另一可信源；
 *       AUTO 模式优先主源、其次备源。 */
static uint16_t arbitrate(uint32_t now_ms, bool *is_backup)
{
    LinkMonitor_t *lp = CanComm_getLink(SRC_IDX_PRIMARY);
#if REDUNDANCY_ENABLE
    LinkMonitor_t *lb = CanComm_getLink(SRC_IDX_BACKUP);
    bool bak = lb->control_fresh;
#endif
    bool pri = lp->control_fresh;

    *is_backup = false;

#if REDUNDANCY_ENABLE
    if (s_manual)
    {
        if (s_override_idx == SRC_IDX_PRIMARY)
        {
            if (pri) { return SRC_IDX_PRIMARY; }
            if (bak) { *is_backup = true; return SRC_IDX_BACKUP; }
        }
        else
        {
            if (bak) { *is_backup = true; return SRC_IDX_BACKUP; }
            if (pri) { return SRC_IDX_PRIMARY; }
        }
        return SRC_IDX_INVALID;
    }

    /* AUTO：主优先 */
    if (s_active_idx == SRC_IDX_BACKUP && bak &&
        (!pri || !lp->fresh_since_valid ||
         (now_ms - lp->fresh_since_ms) < FAILBACK_HOLD_MS))
    {
        *is_backup = true;
        return SRC_IDX_BACKUP;
    }
    if (pri) { return SRC_IDX_PRIMARY; }
    if (bak) { *is_backup = true; return SRC_IDX_BACKUP; }
    return SRC_IDX_INVALID;
#else
    (void)now_ms;
    return pri ? SRC_IDX_PRIMARY : SRC_IDX_INVALID;
#endif
}

/* -------- 汇总故障码 -------- */
static void build_fault_code(uint16_t active_idx)
{
    LinkMonitor_t *lp = CanComm_getLink(SRC_IDX_PRIMARY);
#if REDUNDANCY_ENABLE
    LinkMonitor_t *lb = CanComm_getLink(SRC_IDX_BACKUP);
#endif
    const SocCommand_t *cp = CanComm_getCommand(SRC_IDX_PRIMARY);
#if REDUNDANCY_ENABLE
    const SocCommand_t *cb = CanComm_getCommand(SRC_IDX_BACKUP);
#endif
    uint16_t fc = 0U;

    if (!lp->control_fresh) { fc |= FC_PRIMARY_TIMEOUT; }
#if REDUNDANCY_ENABLE
    if (!lb->control_fresh) { fc |= FC_BACKUP_TIMEOUT; }
#endif
    if (lp->seq_stall_cnt >= SEQ_STALL_LIMIT) { fc |= FC_PRIMARY_SEQ_STALL; }
#if REDUNDANCY_ENABLE
    if (lb->seq_stall_cnt >= SEQ_STALL_LIMIT) { fc |= FC_BACKUP_SEQ_STALL; }
#endif
    if ((lp->seq_seen && !lp->protocol_ok)
#if REDUNDANCY_ENABLE
        || (lb->seq_seen && !lb->protocol_ok)
#endif
       )
    {
        fc |= FC_PROTO_MISMATCH;
    }
    if (!lp->control_fresh
#if REDUNDANCY_ENABLE
        && !lb->control_fresh
#endif
       ) { fc |= FC_ALL_SOURCES_LOST; }
    if (s_crc_err_cnt > 0U) { fc |= FC_CRC_ERRORS; }
    if (cp->fault_level >= FAULT_LEVEL_FATAL
#if REDUNDANCY_ENABLE
        || cb->fault_level >= FAULT_LEVEL_FATAL
#endif
       ) { fc |= FC_SOC_FAULT; }
    if (s_loop_ovr_cnt > 0U) { fc |= FC_LOOP_OVERRUN; }
    if (s_force_estop) { fc |= FC_ESTOP_ACTIVE; }
    if (s_fault_lock)  { fc |= FC_FAULT_LOCK; }
    if (!can_healthy_now()) { fc |= FC_CAN_BUS_OFF; }
    if (s_watchdog_reset) { fc |= FC_WATCHDOG_RESET; }
    if (!self_test_ok_now()) { fc |= FC_SELF_TEST; }
    (void)active_idx;
    s_fault_code = fc;
}

void Safety_step(uint32_t now_ms, bool loop_overrun,
                 McuStatus_t *st, SafetyDecision_t *dec)
{
    bool is_backup = false;
    uint16_t idx;
    uint16_t target;
    const SocCommand_t *cmd;

    if (loop_overrun) { sat_inc(&s_loop_ovr_cnt); }

    if (!s_init_stamped) { s_init_ms = now_ms; s_init_stamped = true; }

    /* 延后/自动解闩：FAULT_LOCK 在以下任一条件持续时维持，条件消除即自动解除，
     * 无需再靠物理长按/断电复位（v3 自驱授权重构）。
     *   - 测试注入的显式闩锁 s_inj_lock（仅 INJ_CMD_CLEAR/长按可解）
     *   - CAN bus-off 快速恢复耗尽（s_can_recovery_exhausted：总线恢复健康即假）
     *   - CAN 当前不健康
     *   - 自检失败（真实或注入）
     *   - 看门狗/自检 boot-hold：因看门狗复位或上电自检失败首次进入 LOCK 时，
     *     保留 FAULT_LOCK_BOOT_HOLD_MS 让操作员通过 CAN 0x202 心跳可见，超时即解除。
     * 长按 Safety_requestClear 只撤销注入闩锁并强制 CAN 重新入网；真实条件仍按
     * 上面这条每 tick 重算，故障未排除则下一 tick 重新闩锁。 */
    if (s_clear_pending)
    {
        s_inj_lock = false;
        s_clear_pending = false;
    }
    {
        bool watchdog_hold = s_watchdog_reset &&
                             ((now_ms - s_init_ms) < FAULT_LOCK_BOOT_HOLD_MS);
        bool persists = s_inj_lock ||
                        s_can_recovery_exhausted ||
                        !s_can_healthy ||
                        !self_test_ok_now() ||
                        watchdog_hold;
        if (s_fault_lock && !persists)
        {
            s_fault_lock = false;
        }
        else if (persists)
        {
            s_fault_lock = true;
        }
    }

    idx = arbitrate(now_ms, &is_backup);
    cmd = (idx != SRC_IDX_INVALID) ? CanComm_getCommand(idx) : 0;

    /* -------- 安全请求汇总（硬安全 vs 软安全 分离）--------
     * 硬安全请求：急停 / AEB 全制动 / 本地强制 MRM（语音急停、故障注入）——永远
     *             绕过 HIL 会话门，FTTI 路径不增加任何会话依赖。
     * 软安全请求：来自 SoC 状态帧的 mrm_request（bit AD_F_MRM）——必须先经过
     *             "会话已授权控制 + 当前会话已建立 mrm=0 零基线"两道校验，避免
     *             HIL 未 ACTIVE 或旧会话残留的 mrm=1 在会话未授权时误触发 MRM。 */

    /* 会话世代切换 → 清空所有源的 MRM 零基线（新会话须先重新看到 mrm=0），
     * 并清除 s_ever_engaged。s_ever_engaged 是开机级"曾运行"闩锁（只在
     * Safety_init/MCU 复位时清零），若不随会话世代同步清除，旧会话曾
     * ACTIVE 过时，新会话（如 GUI 重启 CARLA/bridge 触发的新 ANNOUNCE）
     * 启动瞬间的双源未就绪——完全正常的握手前窗口——会被误判为"运行中
     * 失效"而直接进 FAILSAFE，而不是期望的 STANDBY。s_session_ctx.ever_active
     * 已经是会话世代正确重置（见 hil_session.c ANNOUNCE 分支），这里让
     * s_ever_engaged 同步跟随，同一会话内曾运行后失控仍正确落 FAILSAFE
     * （见 test_active_session_loss_enters_failsafe），只是不再跨会话污染。
     * 世代由 HilSession.session_id 承担；首次进入（epoch_valid=false）也清一次。 */
    if (!s_session_epoch_valid ||
        (s_session_ctx.session_epoch != s_last_session_epoch))
    {
        uint16_t k;
        for (k = 0U; k < SRC_IDX_COUNT; k++) { s_mrm_zero_seen[k] = false; }
        s_ever_engaged         = false;
        s_last_session_epoch   = s_session_ctx.session_epoch;
        s_session_epoch_valid  = true;
    }

    bool src_estop   = false;   /* 硬：急停 + AEB 全制动 */
    bool src_mrm_raw = false;   /* 软：已建立零基线的源当前提出的 mrm=1 */
    if (cmd != 0)
    {
        src_estop = cmd->emergency_stop ||
                    (cmd->aeb_risk >= AEB_RISK_FULL && cmd->obstacle_valid);
    }

    /* Safety requests are ORed across all fresh sources. Arbitration chooses
     * nominal control, but must never suppress a trustworthy brake request.
     * MRM 额外按源维护零基线：某源新鲜地看到一次 mrm=0 才解锁其后续 mrm=1，
     * 主/备各自独立，一源看到 0 不授权另一源遗留的 1（互不污染）。 */
    {
        uint16_t source_idx;
        for (source_idx = 0U; source_idx < SRC_IDX_COUNT; source_idx++)
        {
#if !REDUNDANCY_ENABLE
            if (source_idx == SRC_IDX_BACKUP) { continue; }
#endif
            const LinkMonitor_t *link = CanComm_getLink(source_idx);
            const SocCommand_t *source_cmd = CanComm_getCommand(source_idx);
            if (link->safety_fresh)
            {
                src_estop = src_estop || source_cmd->emergency_stop ||
                    ((source_cmd->aeb_risk >= AEB_RISK_FULL) && source_cmd->obstacle_valid);
                if (!source_cmd->mrm_request)
                {
                    s_mrm_zero_seen[source_idx] = true;   /* 建立/保持零基线 */
                }
                else if (s_mrm_zero_seen[source_idx])
                {
                    src_mrm_raw = true;                   /* 已过基线的合法软请求 */
                }
            }
            else
            {
                s_mrm_zero_seen[source_idx] = false;      /* 断链/陈旧 → 重连须重建 */
            }
        }
    }

    /* 外部软 MRM 会话门：仅当会话已授权控制时采信已过零基线的 mrm=1。
     * forced_mrm（本地语音急停 / 测试注入）不受会话门限制。 */
    bool external_mrm = s_session_ctx.control_authorized && src_mrm_raw;
    bool forced_mrm   = s_force_mrm;

    if (src_estop || s_force_estop)
    {
        if (s_emergency_since_ms == 0U) { s_emergency_since_ms = now_ms; }
    }
    else if (s_state == SYS_MODE_EMERGENCY_BRAKE && s_emergency_since_ms != 0U &&
             (now_ms - s_emergency_since_ms) < EMERGENCY_HOLD_MS)
    {
        src_estop = true;
    }
    else
    {
        s_emergency_since_ms = 0U;
    }

    /* -------- 目标态裁决（优先级） -------- */
    if (s_fault_lock)
    {
        target = SYS_MODE_FAULT_LOCK;
    }
    else if (s_force_estop || src_estop)
    {
        target = SYS_MODE_EMERGENCY_BRAKE;
    }
    else if (idx == SRC_IDX_INVALID)
    {
        /* 双源全失：先过 INIT 稳定窗；曾运行（s_ever_engaged，即 Safety 自身仲裁
         * 曾经选出过有效、有真实控制权的源）且非"合法正常停机" → FAILSAFE 受控
         * 停车；从未运行 → STANDBY。心跳/控制帧突然消失绝不等于正常退出，必须
         * 落到 FAILSAFE 而非退回无保护 STANDBY。
         *
         * 判据只看 s_ever_engaged，不再额外 OR s_session_ctx.ever_active：
         * HilSession 的 ACTIVE 只要求 CAN 帧协议/时序合法（main.c in.control_fresh），
         * 不检查 SocCommand.control_authority；command_gate 未选中 follower（例如
         * 尚无导航路线，仍在 builtin_stop）时，会话可以在 SoC 侧 startup_gate 仍是
         * STARTUP_STANDBY、control_authority 恒 false 的情况下自驱到 ACTIVE 并把
         * ever_active 闩死为 true。若仍 OR 上这个信号，等同于把"CAN 会话握手正常"
         * 误当成"车辆曾被真实接管"，导致车辆从未真正参与控制就被判定 FAILSAFE——
         * 这正是"GUI 启动 CARLA 后立即 MRM/FAILSAFE"在无导航路线时的实测根因。
         * s_ever_engaged 由 Safety 自身仲裁在 idx 有效且 ctrl_enabled 时置位（见
         * 本函数末尾），天然要求 soc_ok（control_authority && soc_health>0），
         * 是"曾经真实接管"的正确判据，且已按会话世代重置（见上方 epoch 分支）。 */
        if (!s_init_stamped || (now_ms - s_init_ms) < INIT_SETTLE_MS)
        {
            target = SYS_MODE_INIT;
        }
        else if (s_session_ctx.normal_shutdown)
        {
            /* 合法 DISARM/停机后受控停机。当前 0x104 协议无 DISARM/STOP 帧，
             * main.c 恒传 normal_shutdown=false，故此分支实际不触发（保留钩子）。 */
            target = SYS_MODE_STANDBY;
        }
        else if (s_ever_engaged)
        {
            target = SYS_MODE_FAILSAFE;
        }
        else
        {
            target = SYS_MODE_STANDBY;
        }
    }
    else if (forced_mrm)
    {
        target = SYS_MODE_MRM;           /* 本地/测试强制，绕过会话门 */
    }
    else if (external_mrm)
    {
        target = SYS_MODE_MRM;           /* 外部软请求，已过会话门 + 零基线 */
    }
    else
    {
        /* 有可信源。判断是否降级：用的是备源 / 另一源已失 / SoC 告警 / seq 停滞 */
        LinkMonitor_t *lp = CanComm_getLink(SRC_IDX_PRIMARY);
        bool both_fresh = lp->control_fresh;
#if REDUNDANCY_ENABLE
        LinkMonitor_t *lb = CanComm_getLink(SRC_IDX_BACKUP);
        both_fresh = both_fresh && lb->control_fresh;
#endif
        bool soc_warn = (cmd->fault_level >= FAULT_LEVEL_WARN);
        /* REDUNDANCY_ENABLE=0 时仲裁从不切到 BACKUP（arbitrate() 不返回
         * BACKUP），is_backup 恒为 false；§B2 把它直接剔除掉避免与
         * "is_backup 必有意义"的误解。 */
        bool backup_active = false;
#if REDUNDANCY_ENABLE
        backup_active = is_backup;
#endif
        bool degraded = backup_active || !both_fresh || soc_warn || s_force_degrade;

        /* control_enable 未置位 → 只做待机(不下发驱动) */
        bool ctrl_enabled = ((cmd->status_word & ST_CONTROL_ENABLE) != 0U) ||
                            cmd->lateral_enable || cmd->longitudinal_enable;

        if (!ctrl_enabled)
        {
            target = SYS_MODE_STANDBY;
        }
        else
        {
            target = degraded ? SYS_MODE_DEGRADED : SYS_MODE_ACTIVE;
        }
    }

    s_state = target;
    if (s_state == SYS_MODE_ACTIVE || s_state == SYS_MODE_DEGRADED ||
        s_state == SYS_MODE_MRM)
    {
        s_ever_engaged = true;
    }
#if REDUNDANCY_ENABLE
    s_active_idx = idx;
#endif

    /* -------- 写 McuStatus + 裁决 -------- */
    build_fault_code(idx);

    st->system_state = s_state;
    if (s_state == SYS_MODE_FAILSAFE || s_state == SYS_MODE_EMERGENCY_BRAKE ||
        s_state == SYS_MODE_FAULT_LOCK)
    {
        st->active_source = SRC_WATCHDOG;   /* MCU 自主安全接管 */
    }
    else if (idx == SRC_IDX_PRIMARY) { st->active_source = SRC_PRIMARY; }
    else if (idx == SRC_IDX_BACKUP)  { st->active_source = SRC_BACKUP; }
    else                             { st->active_source = SRC_NONE; }

    st->fault_code = s_fault_code;
    /* 故障等级：严重态→FATAL/SEVERE；降级→WARN；否则跟随源或 INFO */
    if (s_state == SYS_MODE_EMERGENCY_BRAKE || s_state == SYS_MODE_FAILSAFE ||
        s_state == SYS_MODE_FAULT_LOCK)
    {
        st->fault_level = FAULT_LEVEL_SEVERE;
    }
    else if (s_state == SYS_MODE_DEGRADED || s_state == SYS_MODE_MRM)
    {
        st->fault_level = FAULT_LEVEL_WARN;
    }
    else
    {
        st->fault_level = (cmd != 0) ? cmd->fault_level : FAULT_LEVEL_INFO;
    }

    st->aeb_floor_active = (s_state == SYS_MODE_EMERGENCY_BRAKE ||
                            s_state == SYS_MODE_FAILSAFE ||
                            s_state == SYS_MODE_FAULT_LOCK);
    st->degraded = (s_state == SYS_MODE_DEGRADED || s_state == SYS_MODE_MRM);
    st->estop    = (s_state == SYS_MODE_EMERGENCY_BRAKE);
    st->manual_override = s_manual;
    st->override_source = (s_override_idx == SRC_IDX_BACKUP) ? SRC_BACKUP : SRC_PRIMARY;

    st->primary_timeout_cnt = s_pri_to_cnt;
    st->backup_timeout_cnt  = s_bak_to_cnt;
    st->crc_err_cnt         = s_crc_err_cnt;
    st->loop_overrun_cnt    = s_loop_ovr_cnt;
    st->reset_reason        = s_reset_reason;
    st->self_test_mask      = s_self_test_mask;

    dec->system_state  = s_state;
    dec->active_source = st->active_source;
    dec->active_idx    = idx;
    dec->cmd_valid     = (idx != SRC_IDX_INVALID);
}

/* -------- HMI 接口 -------- */
void Safety_requestSwitchSource(void)
{
#if REDUNDANCY_ENABLE
    /* 短按：手动切到另一台并锁定。若当前 AUTO，先按当前采信源推断起点。 */
    if (!s_manual)
    {
        LinkMonitor_t *lp = CanComm_getLink(SRC_IDX_PRIMARY);
        /* 起点：当前若在用主则切备，否则切主 */
        s_override_idx = lp->control_fresh ? SRC_IDX_BACKUP : SRC_IDX_PRIMARY;
        s_manual = true;
    }
    else
    {
        s_override_idx = (s_override_idx == SRC_IDX_PRIMARY)
                         ? SRC_IDX_BACKUP : SRC_IDX_PRIMARY;
    }
#else
    s_manual = false;
    s_override_idx = SRC_IDX_PRIMARY;
#endif
}

void Safety_requestClear(void)
{
    s_manual = false;
    s_override_idx = SRC_IDX_PRIMARY;
    /* 先撤销模拟故障与 watchdog 复位标志。真实的 Bus-Off 恢复耗尽 / 自检失败
     * 能否解闩，延后到 Safety_step 用本 tick 刷新后的 CAN/自检状态裁决——这样
     * 操作员一次长按触发的 CanComm_forceRecovery 可在同一 tick 生效并解闩，
     * 而故障未真正排除时下一 tick 仍会重新闩锁。 */
    s_inj_can_exhausted = false;
    s_inj_self_test_fail = false;
    s_inj_lock = false;
    s_watchdog_reset = false;
    s_force_degrade = false;
    s_force_estop = false;
    s_force_mrm = false;
    s_drop_primary = false;
    s_drop_backup = false;
    s_fault_code = 0U;
    s_clear_pending = true;   /* 解闩判定交给下一次 Safety_step */
    s_recovery_req = true;    /* 请求 main 强制 CAN 重新入网 */
    /* 计数保留供诊断；不复位状态机(下一 tick 自然重算) */
}

bool Safety_consumeRecoveryRequest(void)
{
    bool req = s_recovery_req;
    s_recovery_req = false;
    return req;
}

bool Safety_isManualOverride(void) { return s_manual; }

uint16_t Safety_overrideSource(void)
{
    return (s_override_idx == SRC_IDX_BACKUP) ? SRC_BACKUP : SRC_PRIMARY;
}

/* -------- 故障注入 -------- */
void Safety_applyInject(uint16_t cmd, uint16_t param)
{
    (void)param;
    switch (cmd)
    {
        case INJ_CMD_CLEAR:         Safety_requestClear(); break;
        case INJ_CMD_FORCE_DEGRADE: s_force_degrade = true; break;
        case INJ_CMD_FORCE_ESTOP:   s_force_estop = true; break;
        case INJ_CMD_DROP_PRIMARY:  s_drop_primary = true; break;
        case INJ_CMD_DROP_BACKUP:   s_drop_backup = true; break;
        case INJ_CMD_FORCE_LOCK:    s_inj_lock = true; s_fault_lock = true; break;
        case INJ_CMD_DROP_ALL:      s_drop_primary = true;
                                    s_drop_backup = true; break;
        case INJ_CMD_FORCE_MRM:     s_force_mrm = true; break;
        case INJ_CMD_CAN_EXHAUSTED: s_inj_can_exhausted = true;
                                    s_inj_lock = true; s_fault_lock = true; break;
        case INJ_CMD_SELF_TEST_FAIL: s_inj_self_test_fail = true;
                                    s_inj_lock = true; s_fault_lock = true; break;
        default: break;
    }
}
