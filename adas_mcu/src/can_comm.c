/*
 * can_comm.c — CANA 收发实现
 *
 * 邮箱分配（CANA 共 32 个 message object）：
 *   RX  1=0x100(主 HB) 2=0x101(主横) 3=0x102(主纵) 4=0x103(主 ADAS)
 *       5=0x110(备 HB) 6=0x111(备横) 7=0x112(备纵) 8=0x113(备 ADAS)
 *       9=0x301(故障注入)
 *   TX 16=0x201(控制) 17=0x202(心跳) 18=0x203(诊断) 19=0x302(注入响应)
 *
 * C28x：CAN 数据以 uint16_t[8] 承载，每元素低 8 位为一个字节。
 */
#include "can_comm.h"
#include "board.h"
#include "adas_config.h"
#include "adas_can_protocol.h"
#include "adas_can_protocol_v3_generated.h"
#include "crc8.h"

#include "driverlib.h"
#include "device.h"
#include <string.h>

/* Production builds must never silently compile out CAN bus-off handling.
 * Host tests may use a reduced driverlib stub, so they opt in explicitly. */
#ifndef ADAS_HOST_TEST
#define ADAS_HOST_TEST 0
#endif
#if (ADAS_HOST_TEST == 0) && !defined(CAN_STATUS_BUS_OFF)
#error "Target driverlib must define CAN_STATUS_BUS_OFF; bus-off recovery cannot be disabled"
#endif

/* 邮箱号 */
#define MOBJ_PRI_HB     1U
#define MOBJ_PRI_LAT    2U
#define MOBJ_PRI_LON    3U
#define MOBJ_PRI_AD     4U
#define MOBJ_BAK_HB     5U
#define MOBJ_BAK_LAT    6U
#define MOBJ_BAK_LON    7U
#define MOBJ_BAK_AD     8U
#define MOBJ_INJECT     9U
#define MOBJ_PRI_SESSION 10U
#define MOBJ_TX_CONTROL 16U
#define MOBJ_TX_HB      17U
#define MOBJ_TX_DIAG    18U
#define MOBJ_TX_INJRESP 19U
#define MOBJ_TX_E2E_DIAG 20U
#define MOBJ_TX_LINKSTATS 21U
#define MOBJ_TX_SESSION 22U

static SocCommand_t  s_cmd[SRC_IDX_COUNT];
static LinkMonitor_t s_link[SRC_IDX_COUNT];

/* 链路间隔统计（0x205）：按 源×帧类 记有效帧到达间隔，MCU 作为接收端
 * 独立度量网关发送连续性（不信任发送端自报）。 */
typedef struct
{
    uint32_t last_ms;
    uint16_t min_gap;      /* 初值 0xFFFF = 尚无数据 */
    uint16_t max_gap;
    uint16_t miss;         /* 间隔超阈值次数，饱和 255 */
    bool     seen;
} GapStat_t;

static uint16_t s_injCmd   = INJ_CMD_CLEAR;
static uint16_t s_injParam = 0U;
static bool     s_injNew   = false;
/* 0x302 响应序列号：每个回送帧自增，给测试脚本可验证的"MCU 收到并处理"证据。 */
static uint16_t s_injRespSeq = 0U;
static bool     s_busHealthy = true;
static uint16_t s_recoveryCount = 0U;   /* 当前 bus-off 突发内的重试次数(驱动耗尽闩锁) */
static uint16_t s_recoveryTotal = 0U;   /* 累计硬件重启次数，饱和 0xFFFF，仅诊断用 */
static uint32_t s_lastRecoveryMs = 0U;
static uint16_t s_sessionRequest = HIL_REQ_NONE;
static uint32_t s_sessionId = 0UL;
static uint16_t s_sessionSequence = 0U;
static bool s_sessionNew = false;

/* ---------------- 小端整型打包/解包 ---------------- */
static int16_t get_i16(const uint16_t *d, int i)
{
    uint16_t lo = d[i]     & 0x00FFU;
    uint16_t hi = d[i + 1] & 0x00FFU;
    return (int16_t)(lo | (uint16_t)(hi << 8));
}

static uint32_t get_u32(const uint16_t *d, int i)
{
    return ((uint32_t)(d[i] & 0x00FFU)) |
           ((uint32_t)(d[i + 1] & 0x00FFU) << 8) |
           ((uint32_t)(d[i + 2] & 0x00FFU) << 16) |
           ((uint32_t)(d[i + 3] & 0x00FFU) << 24);
}

static void put_u32(uint16_t *d, int i, uint32_t v)
{
    d[i] = (uint16_t)(v & 0xFFUL);
    d[i + 1] = (uint16_t)((v >> 8) & 0xFFUL);
    d[i + 2] = (uint16_t)((v >> 16) & 0xFFUL);
    d[i + 3] = (uint16_t)((v >> 24) & 0xFFUL);
}

static void put_i16(uint16_t *d, int i, int16_t v)
{
    d[i]     = (uint16_t)((uint16_t)v & 0x00FFU);
    d[i + 1] = (uint16_t)(((uint16_t)v >> 8) & 0x00FFU);
}

static int16_t f2i16(float v, float scale)
{
    float q = v / scale;
    q = (q >= 0.0f) ? (q + 0.5f) : (q - 0.5f);
    if (q >  32767.0f) { q =  32767.0f; }
    if (q < -32768.0f) { q = -32768.0f; }
    return (int16_t)q;
}

/* CRC 校验最后一字节 */
static bool crc_ok(uint32_t can_id, const uint16_t *d)
{
    return (crc8_compute_frame(can_id, d, CAN_FRAME_DLC - 1U) ==
            (d[CAN_FRAME_DLC - 1U] & 0x00FFU));
}

typedef enum { FRAME_HB, FRAME_LAT, FRAME_LON, FRAME_STATUS } FrameKind_t;
#define FRAME_KIND_COUNT 4U

static GapStat_t s_gap[SRC_IDX_COUNT][FRAME_KIND_COUNT];

static void gap_reset(void)
{
    uint16_t i, k;
    for (i = 0U; i < SRC_IDX_COUNT; i++)
    {
        for (k = 0U; k < FRAME_KIND_COUNT; k++)
        {
            s_gap[i][k].last_ms = 0U;
            s_gap[i][k].min_gap = 0xFFFFU;
            s_gap[i][k].max_gap = 0U;
            s_gap[i][k].miss    = 0U;
            s_gap[i][k].seen    = false;
        }
    }
}

/* 仅对 CRC+seq 通过的帧调用。间隔超 GAP_LINK_RESTART_MS 视为链路重启：
 * 只重置基线，不计入统计（区分运行中断续与允许的重启停顿）。 */
static void gap_update(uint16_t idx, FrameKind_t kind, uint32_t now)
{
    GapStat_t *g = &s_gap[idx][kind];
    if (g->seen)
    {
        uint32_t gap = now - g->last_ms;
        if (gap <= GAP_LINK_RESTART_MS)
        {
            uint16_t thresh = (kind == FRAME_LAT || kind == FRAME_LON)
                              ? GAP_THRESH_CTRL_MS : GAP_THRESH_SLOW_MS;
            uint16_t g16 = (uint16_t)gap;
            if (g16 > g->max_gap) { g->max_gap = g16; }
            if (g16 < g->min_gap) { g->min_gap = g16; }
            if (g16 > thresh && g->miss < 255U) { g->miss++; }
        }
    }
    g->seen = true;
    g->last_ms = now;
}

static uint16_t sat255(uint32_t v) { return (v > 255UL) ? 255U : (uint16_t)v; }

static bool seq_accept(LinkMonitor_t *l, FrameKind_t kind, uint16_t seq)
{
    uint16_t *last;
    uint16_t *errors;
    bool *seen;
    uint16_t delta;
    switch (kind)
    {
        case FRAME_HB:     last = &l->last_hb_seq; errors = &l->hb_seq_err_cnt; seen = &l->seq_seen; break;
        case FRAME_LAT:    last = &l->last_lat_seq; errors = &l->lat_seq_err_cnt; seen = &l->lat_seen; break;
        case FRAME_LON:    last = &l->last_lon_seq; errors = &l->lon_seq_err_cnt; seen = &l->lon_seen; break;
        default:           last = &l->last_status_seq; errors = &l->status_seq_err_cnt; seen = &l->status_seen; break;
    }
    if (!*seen)
    {
        *seen = true;
        *last = seq;
        return true;
    }
    delta = (uint16_t)((seq - *last) & 0x00FFU);
    if (delta == 0U || delta > 127U)
    {
        if (*errors < 0xFFFFU) { (*errors)++; }
        if (l->seq_stall_cnt < 0xFFFFU) { l->seq_stall_cnt++; }
        return false;
    }
    *last = seq;
    *errors = 0U;
    l->seq_stall_cnt = 0U;
    return true;
}

/* ---------------- 初始化 ---------------- */
static void setupRx(uint16_t obj, uint32_t id)
{
    CAN_setupMessageObject(CANA_BASE, obj, id, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_RX, 0U,
                           CAN_MSG_OBJ_NO_FLAGS, CAN_FRAME_DLC);
}

static void setupTx(uint16_t obj, uint32_t id)
{
    CAN_setupMessageObject(CANA_BASE, obj, id, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_TX, 0U,
                           CAN_MSG_OBJ_NO_FLAGS, CAN_FRAME_DLC);
}

#if defined(CAN_STATUS_BUS_OFF)
static void restartCanHardware(void)
{
    CAN_initModule(CANA_BASE);
    CAN_setBitRate(CANA_BASE, DEVICE_SYSCLK_HZ, CAN_BITRATE_BPS, 20U);
    CAN_enableRetry(CANA_BASE);
    setupRx(MOBJ_PRI_HB, CANID_PRIMARY_BASE + CANID_OFFS_HEARTBEAT);
    setupRx(MOBJ_PRI_LAT, CANID_PRIMARY_BASE + CANID_OFFS_LATERAL);
    setupRx(MOBJ_PRI_LON, CANID_PRIMARY_BASE + CANID_OFFS_LONGITUDINAL);
    setupRx(MOBJ_PRI_AD, CANID_PRIMARY_BASE + CANID_OFFS_ADAS_STATUS);
    setupRx(MOBJ_PRI_SESSION, CANID_PRIMARY_SESSION_CONTROL);
#if REDUNDANCY_ENABLE
    setupRx(MOBJ_BAK_HB, CANID_BACKUP_BASE + CANID_OFFS_HEARTBEAT);
    setupRx(MOBJ_BAK_LAT, CANID_BACKUP_BASE + CANID_OFFS_LATERAL);
    setupRx(MOBJ_BAK_LON, CANID_BACKUP_BASE + CANID_OFFS_LONGITUDINAL);
    setupRx(MOBJ_BAK_AD, CANID_BACKUP_BASE + CANID_OFFS_ADAS_STATUS);
#endif
#if ADAS_TEST_BUILD
    setupRx(MOBJ_INJECT, CANID_FAULT_INJECT);
#endif
    setupTx(MOBJ_TX_CONTROL, CANID_MCU_CONTROL);
    setupTx(MOBJ_TX_HB, CANID_MCU_HEARTBEAT);
    setupTx(MOBJ_TX_DIAG, CANID_MCU_DIAG);
    setupTx(MOBJ_TX_E2E_DIAG, CANID_MCU_E2E_DIAG);
    setupTx(MOBJ_TX_LINKSTATS, CANID_MCU_LINKSTATS);
    setupTx(MOBJ_TX_SESSION, CANID_MCU_SESSION_STATUS);
#if ADAS_TEST_BUILD
    setupTx(MOBJ_TX_INJRESP, CANID_FAULT_RESPONSE);
#endif
    CAN_startModule(CANA_BASE);
}
#endif

void CanComm_init(void)
{
    uint16_t i;
    for (i = 0U; i < SRC_IDX_COUNT; i++)
    {
        SocCommand_t *c = &s_cmd[i];
        LinkMonitor_t *l = &s_link[i];
        /* 清零 + 安全缺省：无使能、无目标 */
        memset(c, 0, sizeof(*c));
        memset(l, 0, sizeof(*l));
        c->target_steer_rate_dps = MCU_MAX_STEER_RATE_DPS;
        c->drive_dir = DIR_NEUTRAL;
        c->soc_mode  = SYS_MODE_INIT;
    }

    CAN_initModule(CANA_BASE);
    /* 500 kbps；时间量子 20（依 DEVICE 例程惯例，实板可用 SysConfig 复核） */
    CAN_setBitRate(CANA_BASE, DEVICE_SYSCLK_HZ, CAN_BITRATE_BPS, 20U);
    CAN_enableRetry(CANA_BASE);

    /* 主路 */
    setupRx(MOBJ_PRI_HB,  CANID_PRIMARY_BASE + CANID_OFFS_HEARTBEAT);
    setupRx(MOBJ_PRI_LAT, CANID_PRIMARY_BASE + CANID_OFFS_LATERAL);
    setupRx(MOBJ_PRI_LON, CANID_PRIMARY_BASE + CANID_OFFS_LONGITUDINAL);
    setupRx(MOBJ_PRI_AD,  CANID_PRIMARY_BASE + CANID_OFFS_ADAS_STATUS);
    setupRx(MOBJ_PRI_SESSION, CANID_PRIMARY_SESSION_CONTROL);
#if REDUNDANCY_ENABLE
    setupRx(MOBJ_BAK_HB,  CANID_BACKUP_BASE + CANID_OFFS_HEARTBEAT);
    setupRx(MOBJ_BAK_LAT, CANID_BACKUP_BASE + CANID_OFFS_LATERAL);
    setupRx(MOBJ_BAK_LON, CANID_BACKUP_BASE + CANID_OFFS_LONGITUDINAL);
    setupRx(MOBJ_BAK_AD,  CANID_BACKUP_BASE + CANID_OFFS_ADAS_STATUS);
#endif
#if ADAS_TEST_BUILD
    setupRx(MOBJ_INJECT,  CANID_FAULT_INJECT);
#endif

    setupTx(MOBJ_TX_CONTROL, CANID_MCU_CONTROL);
    setupTx(MOBJ_TX_HB,      CANID_MCU_HEARTBEAT);
    setupTx(MOBJ_TX_DIAG,    CANID_MCU_DIAG);
    setupTx(MOBJ_TX_E2E_DIAG, CANID_MCU_E2E_DIAG);
    setupTx(MOBJ_TX_LINKSTATS, CANID_MCU_LINKSTATS);
    setupTx(MOBJ_TX_SESSION, CANID_MCU_SESSION_STATUS);
#if ADAS_TEST_BUILD
    setupTx(MOBJ_TX_INJRESP, CANID_FAULT_RESPONSE);
#endif

    CAN_startModule(CANA_BASE);
    gap_reset();
    s_busHealthy = true;
    s_recoveryCount = 0U;
    s_recoveryTotal = 0U;
    s_lastRecoveryMs = 0U;
    s_sessionRequest = HIL_REQ_NONE;
    s_sessionId = 0UL;
    s_sessionSequence = 0U;
    s_sessionNew = false;
}

void CanComm_service(uint32_t now_ms)
{
#if defined(CAN_STATUS_BUS_OFF)
    uint32_t status = CAN_getStatus(CANA_BASE);
    if ((status & CAN_STATUS_BUS_OFF) != 0U)
    {
        s_busHealthy = false;
        if (s_recoveryCount < CAN_RECOVERY_MAX_TRIES)
        {
            /* 快速恢复阶段：满足 FTTI 的即时重试预算(MAX_TRIES 次) */
            if ((now_ms - s_lastRecoveryMs) >= CAN_RECOVERY_DELAY_MS)
            {
                s_lastRecoveryMs = now_ms;
                s_recoveryCount++;
                if (s_recoveryTotal < 0xFFFFU) { s_recoveryTotal++; }
                restartCanHardware();
            }
        }
        else if ((now_ms - s_lastRecoveryMs) >= CAN_RECOVERY_REARM_MS)
        {
            /* 快速预算耗尽后不永久放弃：以更慢节奏持续尝试重新入网，使物理
             * 故障排除后无需整机复位即可自动恢复通信。此阶段 s_recoveryCount
             * 保持在 MAX，recoveryExhausted() 仍为真 → FAULT_LOCK 仍闩锁。 */
            s_lastRecoveryMs = now_ms;
            if (s_recoveryTotal < 0xFFFFU) { s_recoveryTotal++; }
            restartCanHardware();
        }
    }
    else
    {
        if (!s_busHealthy)
        {
            /* 总线重新健康：清零快速重试预算，为后续独立故障恢复即时恢复能力，
             * 并令 recoveryExhausted() 转假，使操作员的清故障得以解闩。 */
            s_recoveryCount = 0U;
            s_lastRecoveryMs = now_ms;
        }
        s_busHealthy = true;
    }
#else
    (void)now_ms;
    s_busHealthy = true;
#endif
}

void CanComm_forceRecovery(uint32_t now_ms)
{
#if defined(CAN_STATUS_BUS_OFF)
    s_recoveryCount = 0U;
    s_lastRecoveryMs = now_ms;
    if (s_recoveryTotal < 0xFFFFU) { s_recoveryTotal++; }
    restartCanHardware();
    /* 重新初始化后立即复读总线状态：物理故障已排除则本 tick 即恢复健康，
     * 配合 safety 的延后解闩逻辑做到"一次长按即恢复"。 */
    s_busHealthy = ((CAN_getStatus(CANA_BASE) & CAN_STATUS_BUS_OFF) == 0U);
#else
    (void)now_ms;
    s_busHealthy = true;
    s_recoveryCount = 0U;
#endif
}

bool CanComm_isBusHealthy(void) { return s_busHealthy; }
uint16_t CanComm_recoveryCount(void) { return s_recoveryCount; }
bool CanComm_recoveryExhausted(void)
{
    return (!s_busHealthy && s_recoveryCount >= CAN_RECOVERY_MAX_TRIES);
}

/* ---------------- 解码单帧 ----------------
 * 序列停滞判据集中在心跳(0x100)上：报文仍到达但 seq 停滞 → 上游控制环挂起。
 * 每类帧都刷新各自的物理到达时间戳，供 safety.c 结合超时/停滞综合判新鲜。
 */
static void decodeHeartbeat(uint16_t idx, const uint16_t *d, uint32_t now)
{
    SocCommand_t *c = &s_cmd[idx];
    LinkMonitor_t *l = &s_link[idx];

    c->soc_mode          = d[HB_B_SYS_MODE]  & 0x00FFU;
    c->soc_health        = d[HB_B_SOC_HEALTH] & 0x00FFU;
    c->fault_level       = d[HB_B_FAULT_LEVEL] & 0x00FFU;
    c->control_authority = ((d[HB_B_AUTHORITY] & HB_F_AUTHORITY) != 0U);
    c->source_id         = d[HB_B_SOURCE_ID] & 0x00FFU;
    l->protocol_ok = (((d[HB_B_AUTHORITY] >> HB_VERSION_SHIFT) & HB_VERSION_MASK) ==
                      ADAS_PROTOCOL_VERSION) &&
                     (c->source_id == ((idx == SRC_IDX_PRIMARY) ? SRC_PRIMARY : SRC_BACKUP));
    l->last_hb_ms = now;
}

static void decodeLateral(uint16_t idx, const uint16_t *d, uint32_t now)
{
    SocCommand_t *c = &s_cmd[idx];
    LinkMonitor_t *l = &s_link[idx];
    uint16_t flags = d[LAT_B_FLAGS] & 0x00FFU;

    c->target_steer_deg      = (float)get_i16(d, LAT_B_ANGLE_LO) * SCALE_STEER_DEG;
    c->target_steer_rate_dps = (float)get_i16(d, LAT_B_RATE_LO) * SCALE_STEER_RATE_DPS;
    if (c->target_steer_rate_dps < 0.0f) { c->target_steer_rate_dps = -c->target_steer_rate_dps; }
    c->lateral_enable = ((flags & LAT_F_ENABLE) != 0U);
    c->lka_active     = ((flags & LAT_F_LKA_ACTIVE) != 0U);
    l->lat_valid = (c->target_steer_deg >= -MAX_STEER_DEG) &&
                   (c->target_steer_deg <= MAX_STEER_DEG) &&
                   (c->target_steer_rate_dps <= MCU_MAX_STEER_RATE_DPS) &&
                   ((flags & ~(LAT_F_ENABLE | LAT_F_LKA_ACTIVE)) == 0U);
    if (!l->lat_valid) { c->lateral_enable = false; }

    l->last_lat_seq = d[LAT_B_SEQ] & 0x00FFU;
    l->last_lat_ms  = now;
    l->lat_seen = true;
}

static void decodeLongitudinal(uint16_t idx, const uint16_t *d, uint32_t now)
{
    SocCommand_t *c = &s_cmd[idx];
    LinkMonitor_t *l = &s_link[idx];
    uint16_t flags = d[LON_B_FLAGS] & 0x00FFU;

    c->target_accel_ms2  = (float)get_i16(d, LON_B_ACC_LO) * SCALE_ACCEL_MS2;
    c->target_speed_ms   = (float)get_i16(d, LON_B_SPD_LO) * SCALE_SPEED_MS;
    c->brake_request_pct = d[LON_B_BRAKE] & 0x00FFU;
    c->longitudinal_enable = ((flags & LON_F_ENABLE) != 0U);
    c->acc_active          = ((flags & LON_F_ACC_ACTIVE) != 0U);
    c->parking_brake       = ((flags & LON_F_PARKING) != 0U);
    c->drive_dir           = (flags >> LON_F_DIR_SHIFT) & LON_F_DIR_MASK;
    /* Accept the representable boundary codes.  For example int16 3000
     * multiplied by 0.001f can round a few ULP above 3.0f on C28x; comparing
     * that decoded float to the engineering limit without a half-LSB margin
     * incorrectly rejects the legal maximum command during HIL COMMIT. */
    l->lon_valid =
                   (c->target_accel_ms2 >=
                    (-MAX_DECEL_MS2 - (0.5f * SCALE_ACCEL_MS2))) &&
                   (c->target_accel_ms2 <=
                    (MAX_ACCEL_MS2 + (0.5f * SCALE_ACCEL_MS2))) &&
                   (c->target_speed_ms >= 0.0f) &&
                   (c->brake_request_pct <= 100U) &&
                   (c->drive_dir <= DIR_REVERSE) &&
                   ((flags & ~(LON_F_ENABLE | LON_F_ACC_ACTIVE | LON_F_PARKING |
                               (LON_F_DIR_MASK << LON_F_DIR_SHIFT))) == 0U);
    if (!l->lon_valid)
    {
        c->longitudinal_enable = false;
        c->brake_request_pct = 0U;
        c->drive_dir = DIR_NEUTRAL;
    }

    l->last_lon_seq = d[LON_B_SEQ] & 0x00FFU;
    l->last_lon_ms  = now;
    l->lon_seen = true;
}

static void decodeAdasStatus(uint16_t idx, const uint16_t *d, uint32_t now)
{
    SocCommand_t *c = &s_cmd[idx];
    LinkMonitor_t *l = &s_link[idx];
    uint16_t flags = d[AD_B_FLAGS] & 0x00FFU;

    c->status_word = (uint16_t)((d[AD_B_STATUS_LO] & 0x00FFU) |
                                ((d[AD_B_STATUS_HI] & 0x00FFU) << 8));
    c->aeb_risk = d[AD_B_AEB_RISK] & 0x00FFU;
    c->aeb_required_decel = (float)get_i16(d, AD_B_DECEL_LO) * SCALE_ACCEL_MS2;
    if (c->aeb_required_decel < 0.0f) { c->aeb_required_decel = -c->aeb_required_decel; }
    c->emergency_stop = ((flags & AD_F_ESTOP) != 0U);
    c->mrm_request    = ((flags & AD_F_MRM) != 0U);
    c->obstacle_valid = ((flags & AD_F_OBSTACLE_VALID) != 0U);
    l->status_valid = (c->aeb_risk <= AEB_RISK_FULL) &&
                      (c->aeb_required_decel <= MAX_DECEL_MS2) &&
                      ((flags & ~(AD_F_ESTOP | AD_F_MRM | AD_F_OBSTACLE_VALID)) == 0U);
    if (!l->status_valid)
    {
        c->emergency_stop = false;
        c->aeb_risk = AEB_RISK_NONE;
        c->mrm_request = false;
        c->obstacle_valid = false;
    }

    l->last_status_seq = d[AD_B_SEQ] & 0x00FFU;
    l->last_status_ms  = now;
    l->status_seen = true;
}

/* 读一个收邮箱；若有新帧且 CRC 通过，交 decoder 处理。返回是否有有效新帧 */
static bool pollObj(uint16_t obj, uint32_t can_id, uint16_t idx, FrameKind_t kind, uint32_t now,
                    void (*dec)(uint16_t, const uint16_t *, uint32_t),
                    uint16_t *crc_err)
{
    uint16_t data[CAN_FRAME_DLC];
    if (!CAN_readMessage(CANA_BASE, obj, data))
    {
        return false;
    }
    if (!crc_ok(can_id, data))
    {
        if (*crc_err < 0xFFFFU) { (*crc_err)++; }
        return false;
    }
    {
        uint16_t seq_offset = (kind == FRAME_HB || kind == FRAME_LAT) ? 5U : 6U;
        if (!seq_accept(&s_link[idx], kind, data[seq_offset] & 0x00FFU)) { return false; }
    }
    gap_update(idx, kind, now);
    dec(idx, data, now);
    return true;
}

bool CanComm_pollRx(uint32_t now_ms, uint16_t *crc_err_out)
{
    bool any = false;
    uint16_t crc_err = 0U;

    any |= pollObj(MOBJ_PRI_HB, CANID_PRIMARY_BASE + CANID_OFFS_HEARTBEAT, SRC_IDX_PRIMARY, FRAME_HB, now_ms, decodeHeartbeat, &crc_err);
    any |= pollObj(MOBJ_PRI_LAT, CANID_PRIMARY_BASE + CANID_OFFS_LATERAL, SRC_IDX_PRIMARY, FRAME_LAT, now_ms, decodeLateral, &crc_err);
    any |= pollObj(MOBJ_PRI_LON, CANID_PRIMARY_BASE + CANID_OFFS_LONGITUDINAL, SRC_IDX_PRIMARY, FRAME_LON, now_ms, decodeLongitudinal, &crc_err);
    any |= pollObj(MOBJ_PRI_AD, CANID_PRIMARY_BASE + CANID_OFFS_ADAS_STATUS, SRC_IDX_PRIMARY, FRAME_STATUS, now_ms, decodeAdasStatus, &crc_err);
#if REDUNDANCY_ENABLE
    any |= pollObj(MOBJ_BAK_HB, CANID_BACKUP_BASE + CANID_OFFS_HEARTBEAT, SRC_IDX_BACKUP, FRAME_HB, now_ms, decodeHeartbeat, &crc_err);
    any |= pollObj(MOBJ_BAK_LAT, CANID_BACKUP_BASE + CANID_OFFS_LATERAL, SRC_IDX_BACKUP, FRAME_LAT, now_ms, decodeLateral, &crc_err);
    any |= pollObj(MOBJ_BAK_LON, CANID_BACKUP_BASE + CANID_OFFS_LONGITUDINAL, SRC_IDX_BACKUP, FRAME_LON, now_ms, decodeLongitudinal, &crc_err);
    any |= pollObj(MOBJ_BAK_AD, CANID_BACKUP_BASE + CANID_OFFS_ADAS_STATUS, SRC_IDX_BACKUP, FRAME_STATUS, now_ms, decodeAdasStatus, &crc_err);
#endif

    /* Session control uses a dedicated protocol byte and its sequence is
     * validated by hil_session.c, where idempotent retransmits are visible. */
    {
        uint16_t data[CAN_FRAME_DLC];
        if (CAN_readMessage(CANA_BASE, MOBJ_PRI_SESSION, data))
        {
            if (!crc_ok(CANID_PRIMARY_SESSION_CONTROL, data))
            {
                if (crc_err < 0xFFFFU) { crc_err++; }
            }
            else if ((data[0] & 0x00FFU) == ADAS_PROTOCOL_VERSION_V3)
            {
                s_sessionRequest = data[1] & 0x00FFU;
                s_sessionId = get_u32(data, 2);
                s_sessionSequence = data[6] & 0x00FFU;
                s_sessionNew = true;
                any = true;
            }
        }
    }

    /* 故障注入 0x301 */
#if ADAS_TEST_BUILD
    {
        uint16_t data[CAN_FRAME_DLC];
        if (CAN_readMessage(CANA_BASE, MOBJ_INJECT, data) && crc_ok(CANID_FAULT_INJECT, data))
        {
            s_injCmd   = data[INJ_B_CMD] & 0x00FFU;
            s_injParam = data[INJ_B_PARAM] & 0x00FFU;
            s_injNew   = true;
        }
    }
#endif

    if (crc_err_out != 0) { *crc_err_out = crc_err; }
    return any;
}

/* 取解码结果（idx: SRC_IDX_PRIMARY/BACKUP）。
 * 越界或 SRC_IDX_INVALID 一律返回 NULL，调用方必须显式检查。之前的实现
 * 静默回退到 &s_cmd[0]，会让"无源"判定出错位地拿到主源命令。 */
const SocCommand_t *CanComm_getCommand(uint16_t idx)
{
    if (idx >= SRC_IDX_COUNT) { return 0; }
    return &s_cmd[idx];
}

void CanComm_clearSafetyRequests(uint16_t idx)
{
    SocCommand_t *c;
    if (idx >= SRC_IDX_COUNT) { return; }

    c = &s_cmd[idx];
    c->emergency_stop = false;
    c->aeb_risk = AEB_RISK_NONE;
    c->aeb_required_decel = 0.0f;
    c->mrm_request = false;
    c->obstacle_valid = false;
}

LinkMonitor_t *CanComm_getLink(uint16_t idx)
{
    if (idx >= SRC_IDX_COUNT) { return 0; }
    return &s_link[idx];
}

bool CanComm_getInjectCommand(uint16_t *cmd_out, uint16_t *param_out)
{
    if (!s_injNew) { return false; }
    if (cmd_out   != 0) { *cmd_out = s_injCmd; }
    if (param_out != 0) { *param_out = s_injParam; }
    s_injNew = false;
    return true;
}

bool CanComm_getSessionRequest(uint16_t *request_out, uint32_t *session_id_out,
                               uint16_t *sequence_out)
{
    if (!s_sessionNew) { return false; }
    if (request_out != 0) { *request_out = s_sessionRequest; }
    if (session_id_out != 0) { *session_id_out = s_sessionId; }
    if (sequence_out != 0) { *sequence_out = s_sessionSequence; }
    s_sessionNew = false;
    return true;
}

/* ---------------- 回传编码 ---------------- */
void CanComm_sendControl(const ControlOutput_t *out, const McuStatus_t *st,
                         uint16_t mcu_seq)
{
    uint16_t d[CAN_FRAME_DLC];
    (void)st;
    put_i16(d, OUT_B_STEER_LO, f2i16(out->final_steer_deg, SCALE_STEER_DEG));
    put_i16(d, OUT_B_ACC_LO,   f2i16(out->final_accel_ms2, SCALE_ACCEL_MS2));
    d[OUT_B_THROTTLE] = (uint16_t)(out->final_throttle * 100.0f + 0.5f) & 0x00FFU;
    d[OUT_B_BRAKE]    = (uint16_t)(out->final_brake    * 100.0f + 0.5f) & 0x00FFU;
    d[OUT_B_SEQ] = mcu_seq & 0x00FFU;
    d[OUT_B_CRC] = crc8_compute_frame(CANID_MCU_CONTROL, d, CAN_FRAME_DLC - 1U);

    CAN_sendMessage(CANA_BASE, MOBJ_TX_CONTROL, CAN_FRAME_DLC, d);
}

void CanComm_sendHeartbeat(const McuStatus_t *st, uint16_t mcu_seq,
                           uint16_t alive, bool buzzer_on)
{
    uint16_t d[CAN_FRAME_DLC];
    uint16_t sf = 0U;
    if (s_link[SRC_IDX_PRIMARY].control_fresh) { sf |= SF_PRIMARY_FRESH; }
    if (s_link[SRC_IDX_BACKUP].control_fresh)  { sf |= SF_BACKUP_FRESH; }
    if (st->aeb_floor_active) { sf |= SF_AEB_FLOOR; }
    if (st->degraded)         { sf |= SF_DEGRADED; }
    if (buzzer_on)            { sf |= SF_BUZZER_ON; }
    if (st->manual_override)  { sf |= SF_MANUAL_OVERRIDE; }
    if (st->estop)            { sf |= SF_ESTOP; }

    d[MHB_B_STATE]       = st->system_state & 0x00FFU;
    d[MHB_B_SOURCE]      = st->active_source & 0x00FFU;
    d[MHB_B_SAFEFLAGS]   = sf & 0x00FFU;
    d[MHB_B_FAULT_LEVEL] = st->fault_level & 0x00FFU;
    d[MHB_B_SEQ]         = mcu_seq & 0x00FFU;
    d[MHB_B_ALIVE]       = alive & 0x00FFU;
    d[MHB_B_LOAD]        = st->loop_load_pct & 0x00FFU;
    d[MHB_B_CRC]         = crc8_compute_frame(CANID_MCU_HEARTBEAT, d, CAN_FRAME_DLC - 1U);

    CAN_sendMessage(CANA_BASE, MOBJ_TX_HB, CAN_FRAME_DLC, d);
}

void CanComm_sendDiag(const McuStatus_t *st)
{
    uint16_t d[CAN_FRAME_DLC];
    d[DIAG_B_FAULT_LO] = st->fault_code & 0x00FFU;
    d[DIAG_B_FAULT_HI] = (st->fault_code >> 8) & 0x00FFU;
    d[DIAG_B_RESET]    = st->reset_reason & 0x00FFU;
    d[DIAG_B_PRI_TO]   = st->primary_timeout_cnt & 0x00FFU;
    d[DIAG_B_BAK_TO]   = st->backup_timeout_cnt & 0x00FFU;
    d[DIAG_B_CRC_ERR]  = st->crc_err_cnt & 0x00FFU;
    d[DIAG_B_LOOP_OVR] = st->loop_overrun_cnt & 0x00FFU;
    d[DIAG_B_CRC]      = crc8_compute_frame(CANID_MCU_DIAG, d, CAN_FRAME_DLC - 1U);

    CAN_sendMessage(CANA_BASE, MOBJ_TX_DIAG, CAN_FRAME_DLC, d);
}

static uint16_t seqErrorSum(const LinkMonitor_t *link)
{
    uint32_t sum = (uint32_t)link->hb_seq_err_cnt + link->lat_seq_err_cnt +
                   link->lon_seq_err_cnt + link->status_seq_err_cnt;
    return (uint16_t)((sum > 255U) ? 255U : sum);
}

void CanComm_sendE2eDiag(const McuStatus_t *st)
{
    uint16_t d[CAN_FRAME_DLC];
    uint16_t flags = 0U;
    if (s_link[SRC_IDX_PRIMARY].protocol_ok) { flags |= E2E_PF_PRI_OK; }
    if (s_link[SRC_IDX_BACKUP].protocol_ok) { flags |= E2E_PF_BAK_OK; }
    if (s_busHealthy) { flags |= E2E_PF_CAN_OK; }
    if (CanComm_recoveryExhausted()) { flags |= E2E_PF_RECOVERY_EXHAUSTED; }
    d[E2E_B_PRI_SEQ_ERR] = seqErrorSum(&s_link[SRC_IDX_PRIMARY]);
    d[E2E_B_BAK_SEQ_ERR] = seqErrorSum(&s_link[SRC_IDX_BACKUP]);
    d[E2E_B_PROTO_FLAGS] = flags;
    /* 累计重启次数(而非当前突发计数)：突发计数在重新入网后清零，累计值保留
     * "MCU 曾多次重启 CAN"这一关键 HIL 诊断线索。耗尽状态另由 EXHAUSTED 标志报告。*/
    d[E2E_B_CAN_RECOVERY] = s_recoveryTotal & 0x00FFU;
    d[E2E_B_SELF_TEST] = st->self_test_mask & 0x00FFU;
d[E2E_B_BUILD_FLAGS] = (ADAS_TEST_BUILD ? E2E_BF_TEST_BUILD : 0U);
    /* E2E_BF_TCP_AUTH_HIL 位保留为线格式常量（不再置位）：v3 后会话自驱，
     * 不存在“TCP/物理授权”构建模式。不改协议头以维持位级兼容。 */
    d[E2E_B_VERSION] = ADAS_PROTOCOL_VERSION & 0x00FFU;
    d[E2E_B_CRC] = crc8_compute_frame(CANID_MCU_E2E_DIAG, d, CAN_FRAME_DLC - 1U);
    CAN_sendMessage(CANA_BASE, MOBJ_TX_E2E_DIAG, CAN_FRAME_DLC, d);
}

void CanComm_sendSessionStatus(uint16_t ack, uint32_t session_id,
                               uint16_t sequence)
{
    uint16_t d[CAN_FRAME_DLC];
    d[0] = ADAS_PROTOCOL_VERSION_V3;
    d[1] = ack & 0x00FFU;
    put_u32(d, 2, session_id);
    d[6] = sequence & 0x00FFU;
    d[7] = crc8_compute_frame(CANID_MCU_SESSION_STATUS, d,
                              CAN_FRAME_DLC - 1U);
    CAN_sendMessage(CANA_BASE, MOBJ_TX_SESSION, CAN_FRAME_DLC, d);
}

/* 0x302 注入响应帧布局（与 0x301 的 INJ_* 字段对齐）：
 *   [0] cmd 回显                 [1] param 回显
 *   [2] system_state             [3] fault_level
 *   [4] alive（连续自增，证明不是缓存）
 *   [5] seq 自增（响应序列号）
 *   [6] fault_code 低 8 位
 *   [7] crc8
 * 测试脚本读 seq 即可断言"MCU 确实回了响应"。 */
void CanComm_sendInjectResponse(uint16_t cmd, uint16_t param,
                                const McuStatus_t *st)
{
    uint16_t d[CAN_FRAME_DLC];
    static uint16_t s_inj_alive = 0U;   /* 模块局部，所有响应帧共享 */
    if (s_injRespSeq < 0xFFFFU) { s_injRespSeq++; }
    if (s_inj_alive  < 0xFFFFU) { s_inj_alive++; }

    d[0] = cmd & 0x00FFU;
    d[1] = param & 0x00FFU;
    d[2] = (st != 0) ? (st->system_state & 0x00FFU) : 0U;
    d[3] = (st != 0) ? (st->fault_level  & 0x00FFU) : 0U;
    d[4] = s_inj_alive & 0x00FFU;
    d[5] = s_injRespSeq & 0x00FFU;
    d[6] = (st != 0) ? (st->fault_code & 0x00FFU) : 0U;
    d[7] = crc8_compute_frame(CANID_FAULT_RESPONSE, d,
                              CAN_FRAME_DLC - 1U);
    CAN_sendMessage(CANA_BASE, MOBJ_TX_INJRESP, CAN_FRAME_DLC, d);
}

void CanComm_sendLinkStats(void)
{
    uint16_t d[CAN_FRAME_DLC];
    const GapStat_t *pl = &s_gap[SRC_IDX_PRIMARY][FRAME_LAT];
    const GapStat_t *pn = &s_gap[SRC_IDX_PRIMARY][FRAME_LON];
    const GapStat_t *ph = &s_gap[SRC_IDX_PRIMARY][FRAME_HB];
    const GapStat_t *ps = &s_gap[SRC_IDX_PRIMARY][FRAME_STATUS];
    const GapStat_t *bl = &s_gap[SRC_IDX_BACKUP][FRAME_LAT];
    const GapStat_t *bn = &s_gap[SRC_IDX_BACKUP][FRAME_LON];
    d[LS_B_PRI_CTRL_MAX]  = sat255((pl->max_gap > pn->max_gap) ? pl->max_gap : pn->max_gap);
    d[LS_B_PRI_CTRL_MISS] = sat255((uint32_t)pl->miss + pn->miss);
    d[LS_B_PRI_SLOW_MAX]  = sat255((ph->max_gap > ps->max_gap) ? ph->max_gap : ps->max_gap);
    d[LS_B_PRI_SLOW_MISS] = sat255((uint32_t)ph->miss + ps->miss);
    d[LS_B_PRI_CTRL_MIN]  = sat255((pl->min_gap < pn->min_gap) ? pl->min_gap : pn->min_gap);
    d[LS_B_BAK_CTRL_MAX]  = sat255((bl->max_gap > bn->max_gap) ? bl->max_gap : bn->max_gap);
    d[LS_B_BAK_CTRL_MISS] = sat255((uint32_t)bl->miss + bn->miss);
    d[LS_B_CRC] = crc8_compute_frame(CANID_MCU_LINKSTATS, d, CAN_FRAME_DLC - 1U);
    CAN_sendMessage(CANA_BASE, MOBJ_TX_LINKSTATS, CAN_FRAME_DLC, d);
}

uint16_t CanComm_priCtrlMaxGap(void)
{
    const GapStat_t *pl = &s_gap[SRC_IDX_PRIMARY][FRAME_LAT];
    const GapStat_t *pn = &s_gap[SRC_IDX_PRIMARY][FRAME_LON];
    return sat255((pl->max_gap > pn->max_gap) ? pl->max_gap : pn->max_gap);
}
