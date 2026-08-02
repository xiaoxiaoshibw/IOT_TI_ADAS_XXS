/*
 * adas_safety_stack.c — F280025C 安全栈 ESP32 移植版（完全复刻）
 *
 * 目标：在 ESP32 上实现与 F280025C 主固件功能等价的 ADAS 安全栈，
 *       用于公平对比裸机 vs FreeRTOS 下的实时确定性。
 *
 * 核心差异：ESP32 FreeRTOS 多任务 vs F280025C 裸机轮询 @1kHz
 *   1kHz pipeline 运行在专用核心 + 最高优先级定时器任务
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "adas";

/* ======================== CAN 协议常量 ======================== */
#define CAN_BITRATE_BPS        500000UL
#define CAN_FRAME_DLC          8U

#define CANID_PRIMARY_BASE     0x100U
#define CANID_OFFS_HEARTBEAT   0x0U
#define CANID_OFFS_LATERAL     0x1U
#define CANID_OFFS_LONGITUDINAL 0x2U
#define CANID_OFFS_ADAS_STATUS 0x3U
#define CANID_OFFS_SESSION     0x4U

#define CANID_MCU_CONTROL      0x201U
#define CANID_MCU_HEARTBEAT    0x202U
#define CANID_MCU_DIAG         0x203U
#define CANID_MCU_E2E_DIAG     0x204U
#define CANID_MCU_LINKSTATS    0x205U
#define CANID_MCU_SESSION      0x206U

#define CANID_FAULT_INJECT     0x301U
#define CANID_FAULT_RESPONSE   0x302U

/* ======================== 系统常量 ======================== */
#define CTRL_LOOP_HZ          1000U
#define CTRL_LOOP_PERIOD_US   1000U
#define TX_CONTROL_PERIOD_MS  10U
#define TX_HEARTBEAT_PERIOD_MS 20U
#define TX_DIAG_PERIOD_MS     100U

#define HB_TIMEOUT_MS         100U
#define CTRL_TIMEOUT_MS       60U
#define STATUS_TIMEOUT_MS     120U
#define SEQ_STALL_LIMIT       5U
#define INIT_SETTLE_MS        200U
#define FAILBACK_HOLD_MS      300U
#define EMERGENCY_HOLD_MS     500U

#define CAN_RECOVERY_DELAY_MS 100U
#define CAN_RECOVERY_MAX_TRIES 3U

#define MAX_STEER_DEG         30.0f
#define MCU_MAX_STEER_RATE_DPS 400.0f
#define MAX_ACCEL_MS2         3.0f
#define MAX_DECEL_MS2         8.0f
#define ACCEL_SLEW_MS3        8.0f
#define STEER_RETURN_DPS      60.0f
#define FAILSAFE_DECEL_MS2    8.0f
#define MRM_DECEL_MS2         4.8f

#define ADAS_PROTOCOL_VERSION 3U

/* ======================== 枚举 ======================== */
#define SYS_MODE_INIT          0U
#define SYS_MODE_STANDBY       1U
#define SYS_MODE_ACTIVE        2U
#define SYS_MODE_DEGRADED      3U
#define SYS_MODE_MRM           4U
#define SYS_MODE_EMERGENCY_BRAKE 5U
#define SYS_MODE_FAILSAFE      6U
#define SYS_MODE_FAULT_LOCK    7U

#define FAULT_LEVEL_INFO       0U
#define FAULT_LEVEL_WARN       1U
#define FAULT_LEVEL_SEVERE     2U
#define FAULT_LEVEL_FATAL      3U

#define SRC_NONE               0U
#define SRC_PRIMARY            1U
#define SRC_BACKUP             2U
#define SRC_WATCHDOG           9U

#define AEB_RISK_NONE          0U
#define AEB_RISK_WARNING       1U
#define AEB_RISK_PARTIAL       2U
#define AEB_RISK_FULL          3U

/* ======================== 安全标志 ======================== */
#define ST_CONTROL_ENABLE      (1U << 0)
#define ST_LATERAL_ENABLE      (1U << 1)
#define ST_LONGITUDINAL_ENABLE (1U << 2)
#define ST_LKA_ACTIVE          (1U << 3)
#define ST_ACC_ACTIVE          (1U << 4)
#define ST_AEB_WARNING         (1U << 5)
#define ST_AEB_BRAKING         (1U << 6)
#define ST_EMERGENCY_STOP      (1U << 7)
#define ST_DEGRADED_MODE       (1U << 8)
#define ST_TAKEOVER_REQUEST    (1U << 9)
#define ST_MRM_REQUEST         (1U << 10)
#define ST_PERCEPTION_VALID    (1U << 11)
#define ST_LOCALIZATION_VALID  (1U << 12)
#define ST_PLANNING_VALID      (1U << 13)

#define SF_PRIMARY_FRESH       (1U << 0)
#define SF_BACKUP_FRESH        (1U << 1)
#define SF_AEB_FLOOR           (1U << 2)
#define SF_DEGRADED            (1U << 3)
#define SF_BUZZER_ON           (1U << 4)
#define SF_MANUAL_OVERRIDE     (1U << 5)
#define SF_ESTOP               (1U << 6)

#define HB_F_AUTHORITY         (1U << 0)
#define HB_VERSION_SHIFT       4U
#define HB_VERSION_MASK        0x0FU

/* 故障码 */
#define FC_PRIMARY_TIMEOUT     (1U << 0)
#define FC_BACKUP_TIMEOUT      (1U << 1)
#define FC_ALL_SOURCES_LOST    (1U << 4)
#define FC_CRC_ERRORS          (1U << 5)
#define FC_LOOP_OVERRUN        (1U << 7)
#define FC_ESTOP_ACTIVE        (1U << 8)
#define FC_FAULT_LOCK          (1U << 10)
#define FC_CAN_BUS_OFF         (1U << 11)

/* ======================== 帧字节偏移 ======================== */
#define HB_B_SYS_MODE      0
#define HB_B_SOC_HEALTH    1
#define HB_B_FAULT_LEVEL   2
#define HB_B_AUTHORITY     3
#define HB_B_SOURCE_ID     4
#define HB_B_SEQ           5
#define HB_B_ALIVE         6
#define HB_B_CRC           7

#define LAT_B_ANGLE_LO     0
#define LAT_B_ANGLE_HI     1
#define LAT_B_RATE_LO      2
#define LAT_B_RATE_HI      3
#define LAT_B_FLAGS        4
#define LAT_B_SEQ          5
#define LAT_B_CRC          7

#define LON_B_ACC_LO       0
#define LON_B_ACC_HI       1
#define LON_B_SPD_LO       2
#define LON_B_SPD_HI       3
#define LON_B_BRAKE        4
#define LON_B_FLAGS        5
#define LON_B_SEQ          6
#define LON_B_CRC          7

#define AD_B_STATUS_LO     0
#define AD_B_STATUS_HI     1
#define AD_B_AEB_RISK      2
#define AD_B_DECEL_LO      3
#define AD_B_DECEL_HI      4
#define AD_B_FLAGS         5
#define AD_B_SEQ           6
#define AD_B_CRC           7

#define SESS_B_VER         0
#define SESS_B_REQ         1
#define SESS_B_ID_0        2
#define SESS_B_ID_1        3
#define SESS_B_ID_2        4
#define SESS_B_ID_3        5
#define SESS_B_SEQ         6
#define SESS_B_CRC         7

#define OUT_B_STEER_LO     0
#define OUT_B_STEER_HI     1
#define OUT_B_ACC_LO       2
#define OUT_B_ACC_HI       3
#define OUT_B_THROTTLE     4
#define OUT_B_BRAKE        5
#define OUT_B_SEQ          6
#define OUT_B_CRC          7

#define MHB_B_STATE        0
#define MHB_B_SOURCE       1
#define MHB_B_SAFEFLAGS    2
#define MHB_B_FAULT_LEVEL  3
#define MHB_B_SEQ          4
#define MHB_B_ALIVE        5
#define MHB_B_LOAD         6
#define MHB_B_CRC          7

#define DIAG_B_FAULT_LO    0
#define DIAG_B_FAULT_HI    1
#define DIAG_B_RESET       2
#define DIAG_B_PRI_TO      3
#define DIAG_B_BAK_TO      4
#define DIAG_B_CRC_ERR     5
#define DIAG_B_LOOP_OVR    6
#define DIAG_B_CRC         7

#define E2E_B_PRI_SEQ_ERR   0
#define E2E_B_BAK_SEQ_ERR   1
#define E2E_B_PROTO_FLAGS   2
#define E2E_B_CAN_RECOVERY  3
#define E2E_B_SELF_TEST     4
#define E2E_B_BUILD_FLAGS   5
#define E2E_B_VERSION       6
#define E2E_B_CRC           7

#define LS_B_PRI_CTRL_MAX   0
#define LS_B_PRI_CTRL_MISS  1
#define LS_B_PRI_SLOW_MAX   2
#define LS_B_PRI_SLOW_MISS  3
#define LS_B_PRI_CTRL_MIN   4
#define LS_B_BAK_CTRL_MAX   5
#define LS_B_BAK_CTRL_MISS  6
#define LS_B_CRC            7

#define INJ_B_CMD           0
#define INJ_B_PARAM         1
#define INJ_B_SEQ           5
#define INJ_B_CRC           7

#define INJ_CMD_CLEAR         0U
#define INJ_CMD_FORCE_DEGRADE 1U
#define INJ_CMD_FORCE_ESTOP   2U
#define INJ_CMD_DROP_PRIMARY  3U
#define INJ_CMD_FORCE_LOCK    5U
#define INJ_CMD_FORCE_MRM     7U

/* ======================== 数据结构 ======================== */
typedef struct {
    float target_steer_deg;
    float target_steer_rate_dps;
    bool  lateral_enable;
    bool  lka_active;
    float target_accel_ms2;
    float target_speed_ms;
    uint16_t brake_request_pct;
    bool  longitudinal_enable;
    bool  acc_active;
    bool  emergency_stop;
    bool  mrm_request;
    uint16_t aeb_risk;
    float aeb_required_decel;
    bool  obstacle_valid;
    uint16_t soc_mode;
    uint16_t soc_health;
    uint16_t fault_level;
    bool  control_authority;
    uint16_t source_id;
    uint16_t status_word;
} SocCommand_t;

typedef struct {
    uint32_t last_hb_ms;
    uint32_t last_lat_ms;
    uint32_t last_lon_ms;
    uint32_t last_status_ms;
    uint16_t last_hb_seq, last_lat_seq, last_lon_seq, last_status_seq;
    uint16_t seq_stall_cnt;
    bool     fresh;
    bool     control_fresh;
    bool     safety_fresh;
    uint32_t fresh_since_ms;
    bool     fresh_since_valid;
} LinkMonitor_t;

typedef struct {
    float final_steer_deg;
    float final_accel_ms2;
    float final_throttle;
    float final_brake;
} ControlOutput_t;

typedef struct {
    uint16_t system_state;
    uint16_t active_source;
    uint16_t fault_level;
    uint16_t fault_code;
    bool     aeb_floor_active;
    bool     degraded;
    bool     estop;
    bool     manual_override;
    uint16_t override_source;
    uint16_t primary_timeout_cnt;
    uint16_t backup_timeout_cnt;
    uint16_t crc_err_cnt;
    uint16_t loop_overrun_cnt;
    uint16_t reset_reason;
    uint16_t loop_load_pct;
} McuStatus_t;

/* ======================== 全局状态 ======================== */
static McuStatus_t   g_status;
static ControlOutput_t g_out;
static SocCommand_t  g_cmd[2];
static LinkMonitor_t g_link[2];
static bool g_run = false;
static uint32_t g_tick_ms = 0;
static uint32_t g_overrun = 0;
static uint32_t g_mcu_seq = 0;
static uint32_t g_alive = 0;
static uint16_t g_inj_cmd = 0;
static bool g_inj_new = false;
static bool g_recovery_req = false;
static bool g_can_healthy = true;
static bool g_can_exhausted = false;
static uint32_t g_last_ctrl_tx = 0;
static uint32_t g_last_hb_tx = 0;
static uint32_t g_last_diag_tx = 0;
static float g_current_steer = 0.0f;
static float g_current_accel = 0.0f;
static uint32_t g_emergency_hold_start = 0;
static bool g_emergency_holding = false;

/* ======================== CRC-8 ======================== */
static uint8_t crc8(const uint8_t *d, int len)
{
    uint8_t crc = 0x00;
    for (int i = 0; i < len; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return crc;
}

static uint8_t crc8_frame(uint32_t id, const uint8_t *data7)
{
    uint8_t buf[9];
    buf[0] = (uint8_t)(id & 0xFF);
    buf[1] = (uint8_t)((id >> 8) & 0xFF);
    memcpy(buf + 2, data7, 7);
    return crc8(buf, 9);
}

static bool crc_ok(uint32_t id, const uint8_t *d)
{
    return d[7] == crc8_frame(id, d);
}

/* ======================== 定点缩放 ======================== */
static int16_t f2i16(float v, float scale)
{
    float s = v / scale;
    if (s > 32767.0f) s = 32767.0f;
    if (s < -32768.0f) s = -32768.0f;
    return (int16_t)(s);
}

static float i162f(int16_t raw, float scale)
{
    return (float)raw * scale;
}

static int16_t get_i16(const uint8_t *d, int i)
{
    return (int16_t)(d[i] | (d[i+1] << 8));
}

/* ======================== 帧解析 ======================== */
static bool decode_hb(SocCommand_t *cmd, const uint8_t *d)
{
    cmd->soc_mode       = d[HB_B_SYS_MODE];
    cmd->soc_health     = d[HB_B_SOC_HEALTH];
    cmd->fault_level    = d[HB_B_FAULT_LEVEL];
    cmd->control_authority = (d[HB_B_AUTHORITY] & HB_F_AUTHORITY) != 0;
    cmd->source_id      = d[HB_B_SOURCE_ID];
    return true;
}

static bool decode_lat(SocCommand_t *cmd, const uint8_t *d)
{
    cmd->target_steer_deg = i162f(get_i16(d, LAT_B_ANGLE_LO), 0.01f);
    cmd->target_steer_rate_dps = i162f(get_i16(d, LAT_B_RATE_LO), 0.1f);
    cmd->lateral_enable  = (d[LAT_B_FLAGS] & (1U << 0)) != 0;
    cmd->lka_active      = (d[LAT_B_FLAGS] & (1U << 1)) != 0;
    return true;
}

static bool decode_lon(SocCommand_t *cmd, const uint8_t *d)
{
    cmd->target_accel_ms2 = i162f(get_i16(d, LON_B_ACC_LO), 0.001f);
    cmd->target_speed_ms  = i162f(get_i16(d, LON_B_SPD_LO), 0.01f);
    cmd->brake_request_pct = d[LON_B_BRAKE];
    cmd->longitudinal_enable = (d[LON_B_FLAGS] & (1U << 0)) != 0;
    cmd->acc_active      = (d[LON_B_FLAGS] & (1U << 1)) != 0;
    return true;
}

static bool decode_status(SocCommand_t *cmd, const uint8_t *d)
{
    cmd->status_word    = d[AD_B_STATUS_LO] | (d[AD_B_STATUS_HI] << 8);
    cmd->aeb_risk       = d[AD_B_AEB_RISK];
    cmd->aeb_required_decel = i162f(get_i16(d, AD_B_DECEL_LO), 0.001f);
    cmd->emergency_stop = (d[AD_B_FLAGS] & (1U << 0)) != 0;
    cmd->mrm_request    = (d[AD_B_FLAGS] & (1U << 1)) != 0;
    cmd->obstacle_valid = (d[AD_B_FLAGS] & (1U << 2)) != 0;
    return true;
}

/* ======================== 帧编码 ======================== */
static void encode_control(uint8_t *d, const ControlOutput_t *out, uint16_t seq)
{
    memset(d, 0, 8);
    int16_t steer = f2i16(out->final_steer_deg, 0.01f);
    int16_t accel = f2i16(out->final_accel_ms2, 0.001f);
    d[OUT_B_STEER_LO] = (uint8_t)(steer & 0xFF);
    d[OUT_B_STEER_HI] = (uint8_t)((steer >> 8) & 0xFF);
    d[OUT_B_ACC_LO]   = (uint8_t)(accel & 0xFF);
    d[OUT_B_ACC_HI]   = (uint8_t)((accel >> 8) & 0xFF);
    d[OUT_B_THROTTLE] = (uint8_t)(out->final_throttle * 100.0f);
    d[OUT_B_BRAKE]    = (uint8_t)(out->final_brake * 100.0f);
    d[OUT_B_SEQ]      = (uint8_t)(seq & 0xFF);
    d[OUT_B_CRC]      = crc8_frame(CANID_MCU_CONTROL, d);
}

static void encode_heartbeat(uint8_t *d, const McuStatus_t *st, uint16_t seq, uint16_t alive)
{
    memset(d, 0, 8);
    uint16_t sf = 0;
    if (g_link[0].control_fresh) sf |= SF_PRIMARY_FRESH;
    if (g_link[1].control_fresh) sf |= SF_BACKUP_FRESH;
    if (st->aeb_floor_active)    sf |= SF_AEB_FLOOR;
    if (st->degraded)            sf |= SF_DEGRADED;
    if (st->estop)               sf |= SF_ESTOP;

    d[MHB_B_STATE]       = (uint8_t)(st->system_state & 0xFF);
    d[MHB_B_SOURCE]      = (uint8_t)(st->active_source & 0xFF);
    d[MHB_B_SAFEFLAGS]   = (uint8_t)(sf & 0xFF);
    d[MHB_B_FAULT_LEVEL] = (uint8_t)(st->fault_level & 0xFF);
    d[MHB_B_SEQ]         = (uint8_t)(seq & 0xFF);
    d[MHB_B_ALIVE]       = (uint8_t)(alive & 0xFF);
    d[MHB_B_LOAD]        = (uint8_t)(st->loop_load_pct & 0xFF);
    d[MHB_B_CRC]         = crc8_frame(CANID_MCU_HEARTBEAT, d);
}

static void encode_diag(uint8_t *d, const McuStatus_t *st)
{
    memset(d, 0, 8);
    d[DIAG_B_FAULT_LO]   = (uint8_t)(st->fault_code & 0xFF);
    d[DIAG_B_FAULT_HI]   = (uint8_t)((st->fault_code >> 8) & 0xFF);
    d[DIAG_B_RESET]      = (uint8_t)(st->reset_reason & 0xFF);
    d[DIAG_B_PRI_TO]     = (uint8_t)(st->primary_timeout_cnt & 0xFF);
    d[DIAG_B_BAK_TO]     = (uint8_t)(st->backup_timeout_cnt & 0xFF);
    d[DIAG_B_CRC_ERR]    = (uint8_t)(st->crc_err_cnt & 0xFF);
    d[DIAG_B_LOOP_OVR]   = (uint8_t)(st->loop_overrun_cnt & 0xFF);
    d[DIAG_B_CRC]        = crc8_frame(CANID_MCU_DIAG, d);
}

static void encode_e2e(uint8_t *d, uint16_t proto_flags)
{
    memset(d, 0, 8);
    d[E2E_B_PROTO_FLAGS] = (uint8_t)(proto_flags & 0xFF);
    d[E2E_B_BUILD_FLAGS] = 0;
    d[E2E_B_VERSION]     = ADAS_PROTOCOL_VERSION;
    d[E2E_B_CRC]         = crc8_frame(CANID_MCU_E2E_DIAG, d);
}

static void encode_linkstats(uint8_t *d)
{
    memset(d, 0, 8);
    d[LS_B_CRC] = crc8_frame(CANID_MCU_LINKSTATS, d);
}

/* ======================== 控制计算负载 ======================== */
static volatile float g_sink;
static uint16_t bench_compute(void)
{
    float acc = 0.0f;
    float x = 0.123f;
    for (int i = 0; i < 256; i++) {
        acc += sinf(x) * cosf(x) + x * x;
        x += 0.0009765625f;
    }
    g_sink = acc;
    return (uint16_t)((int32_t)(acc * 16.0f)) & 0xFFFF;
}

/* ======================== 控制执行 ======================== */
static float clampf(float v, float lo, float hi)
{
    if (isnan(v)) return 0.0f;
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void control_step(uint32_t now)
{
    uint16_t state = g_status.system_state;
    SocCommand_t *cmd = &g_cmd[g_status.active_source > 0 ? g_status.active_source - 1 : 0];
    bool lateral_en = cmd->lateral_enable && (g_status.active_source != SRC_NONE);
    bool long_en = cmd->longitudinal_enable && (g_status.active_source != SRC_NONE);

    /* 确定目标 */
    float target_steer = 0.0f;
    float target_accel = 0.0f;
    bool immediate = false;

    switch (state) {
    case SYS_MODE_FAULT_LOCK:
    case SYS_MODE_EMERGENCY_BRAKE:
        target_steer = g_current_steer;
        target_accel = -MAX_DECEL_MS2;
        immediate = true;
        break;
    case SYS_MODE_FAILSAFE:
        target_steer = g_current_steer > 0
            ? fmaxf(0.0f, g_current_steer - STEER_RETURN_DPS / 1000.0f)
            : fminf(0.0f, g_current_steer + STEER_RETURN_DPS / 1000.0f);
        if (fabsf(target_steer) < 0.1f) target_steer = 0.0f;
        target_accel = -FAILSAFE_DECEL_MS2;
        break;
    case SYS_MODE_MRM:
        target_steer = lateral_en && g_status.active_source == SRC_PRIMARY ? cmd->target_steer_deg : 0.0f;
        target_accel = -MRM_DECEL_MS2;
        break;
    case SYS_MODE_ACTIVE:
    case SYS_MODE_DEGRADED:
        target_steer = lateral_en ? cmd->target_steer_deg : 0.0f;
        if (long_en) {
            if (cmd->aeb_risk == AEB_RISK_FULL && cmd->obstacle_valid) {
                target_accel = -fmaxf(MAX_DECEL_MS2, cmd->aeb_required_decel);
                immediate = true;
            } else if (cmd->emergency_stop) {
                target_accel = -MAX_DECEL_MS2;
                immediate = true;
            } else {
                target_accel = cmd->target_accel_ms2;
            }
        } else {
            target_accel = 0.0f;
        }
        break;
    default:
        target_steer = 0.0f;
        target_accel = 0.0f;
        break;
    }

    /* 限幅 */
    target_steer = clampf(target_steer, -MAX_STEER_DEG, MAX_STEER_DEG);

    /* 斜率限制 */
    if (immediate) {
        g_current_accel = target_accel;
    } else {
        float max_delta = ACCEL_SLEW_MS3 / 1000.0f;
        if (target_accel > g_current_accel + max_delta)
            g_current_accel += max_delta;
        else if (target_accel < g_current_accel - max_delta)
            g_current_accel -= max_delta;
        else
            g_current_accel = target_accel;
    }

    float steer_delta = MCU_MAX_STEER_RATE_DPS / 1000.0f;
    float rate_limit = cmd->target_steer_rate_dps > 0 ? cmd->target_steer_rate_dps / 1000.0f : steer_delta;
    if (target_steer > g_current_steer + rate_limit)
        g_current_steer += rate_limit;
    else if (target_steer < g_current_steer - rate_limit)
        g_current_steer -= rate_limit;
    else
        g_current_steer = target_steer;

    g_current_steer = clampf(g_current_steer, -MAX_STEER_DEG, MAX_STEER_DEG);
    g_current_accel = clampf(g_current_accel, -MAX_DECEL_MS2, MAX_ACCEL_MS2);

    /* 踏板映射 */
    g_out.final_steer_deg = g_current_steer;
    g_out.final_accel_ms2 = g_current_accel;
    if (g_current_accel >= 0) {
        g_out.final_throttle = g_current_accel / MAX_ACCEL_MS2;
        g_out.final_brake = 0.0f;
    } else {
        g_out.final_throttle = 0.0f;
        g_out.final_brake = -g_current_accel / MAX_DECEL_MS2;
    }
    g_out.final_throttle = clampf(g_out.final_throttle, 0.0f, 1.0f);
    g_out.final_brake = clampf(g_out.final_brake, 0.0f, 1.0f);
}

/* ======================== 安全状态机 ======================== */
static void safety_step(uint32_t now)
{
    SocCommand_t *pcmd = &g_cmd[0];

    /* 链路新鲜度评估 */
    for (int i = 0; i < 2; i++) {
        LinkMonitor_t *lk = &g_link[i];
        lk->fresh = (now - lk->last_hb_ms < HB_TIMEOUT_MS);
        lk->safety_fresh = lk->fresh && (now - lk->last_status_ms < STATUS_TIMEOUT_MS);
        lk->control_fresh = lk->safety_fresh &&
            (now - lk->last_lat_ms < CTRL_TIMEOUT_MS) &&
            (now - lk->last_lon_ms < CTRL_TIMEOUT_MS);
        if (lk->control_fresh && !lk->fresh_since_valid) {
            lk->fresh_since_ms = now;
            lk->fresh_since_valid = true;
        }
        if (!lk->control_fresh) lk->fresh_since_valid = false;
    }

    bool pri_fresh = g_link[0].control_fresh;
    bool bak_fresh = g_link[1].control_fresh;

    uint16_t new_state = g_status.system_state;
    uint16_t new_source = g_status.active_source;

    /* 故障锁存 */
    if (g_can_exhausted || (g_status.fault_code & FC_FAULT_LOCK)) {
        new_state = SYS_MODE_FAULT_LOCK;
    }
    /* AUTOSAR 风格优先级仲裁 */
    else if (pcmd->emergency_stop || (pcmd->aeb_risk == AEB_RISK_FULL && pcmd->obstacle_valid)) {
        new_state = SYS_MODE_EMERGENCY_BRAKE;
        if (!g_emergency_holding) {
            g_emergency_hold_start = now;
            g_emergency_holding = true;
        }
    }
    else if (g_emergency_holding && (now - g_emergency_hold_start < EMERGENCY_HOLD_MS)) {
        new_state = SYS_MODE_EMERGENCY_BRAKE;
    }
    else {
        g_emergency_holding = false;
        if (!pri_fresh && !bak_fresh) {
            if (g_status.system_state >= SYS_MODE_ACTIVE)
                new_state = SYS_MODE_FAILSAFE;
            else
                new_state = SYS_MODE_STANDBY;
        }
        else if (pcmd->mrm_request && pri_fresh) {
            new_state = SYS_MODE_MRM;
        }
        else if (!pri_fresh && bak_fresh) {
            new_state = SYS_MODE_DEGRADED;
            new_source = SRC_BACKUP;
        }
        else if (pri_fresh && pcmd->control_authority &&
                 (pcmd->status_word & ST_CONTROL_ENABLE)) {
            new_state = SYS_MODE_ACTIVE;
            new_source = SRC_PRIMARY;
        }
        else if (new_state < SYS_MODE_STANDBY) {
            new_state = SYS_MODE_STANDBY;
            new_source = SRC_NONE;
        }
        else if (new_state == SYS_MODE_MRM && !pcmd->mrm_request) {
            new_state = SYS_MODE_ACTIVE;
        }
    }

    if (!g_can_healthy) {
        g_status.fault_code |= FC_CAN_BUS_OFF;
    }

    g_status.system_state = new_state;
    g_status.active_source = new_source;
    g_status.degraded = (new_state == SYS_MODE_DEGRADED);
    g_status.estop = (new_state == SYS_MODE_EMERGENCY_BRAKE);
    g_status.fault_level = (new_state >= SYS_MODE_FAILSAFE) ? FAULT_LEVEL_SEVERE :
                           (new_state >= SYS_MODE_DEGRADED) ? FAULT_LEVEL_WARN :
                           FAULT_LEVEL_INFO;
}

/* ======================== 帧接收 ======================== */
static void process_rx_frame(const twai_message_t *msg, uint32_t now)
{
    uint8_t d[8];
    for (int i = 0; i < 8; i++) d[i] = msg->data[i];

    if (!crc_ok(msg->identifier, d)) {
        g_status.crc_err_cnt++;
        return;
    }

    int src_idx = 0;
    uint32_t id = msg->identifier;
    if (id >= CANID_PRIMARY_BASE && id <= CANID_PRIMARY_BASE + 4) {
        src_idx = 0;
    } else if (id >= 0x110 && id <= 0x113) {
        src_idx = 1;
        id -= 0x10;
    } else if (id == CANID_FAULT_INJECT) {
        g_inj_cmd = d[INJ_B_CMD];
        g_inj_new = true;
        return;
    } else {
        return;
    }

    SocCommand_t *cmd = &g_cmd[src_idx];
    LinkMonitor_t *lk = &g_link[src_idx];

    switch (id) {
    case CANID_PRIMARY_BASE + CANID_OFFS_HEARTBEAT:
        decode_hb(cmd, d);
        lk->last_hb_ms = now;
        lk->last_hb_seq = d[HB_B_SEQ];
        break;
    case CANID_PRIMARY_BASE + CANID_OFFS_LATERAL:
        decode_lat(cmd, d);
        lk->last_lat_ms = now;
        lk->last_lat_seq = d[LAT_B_SEQ];
        break;
    case CANID_PRIMARY_BASE + CANID_OFFS_LONGITUDINAL:
        decode_lon(cmd, d);
        lk->last_lon_ms = now;
        lk->last_lon_seq = d[LON_B_SEQ];
        break;
    case CANID_PRIMARY_BASE + CANID_OFFS_ADAS_STATUS:
        decode_status(cmd, d);
        lk->last_status_ms = now;
        lk->last_status_seq = d[AD_B_SEQ];
        break;
    case CANID_PRIMARY_BASE + CANID_OFFS_SESSION:
        break;
    }
}

/* ======================== 帧发送 ======================== */
static bool send_frame(uint32_t id, const uint8_t *data)
{
    twai_message_t tx = {
        .identifier = id,
        .data_length_code = 8,
        .extd = 0, .rtr = 0,
    };
    memcpy(tx.data, data, 8);
    return twai_transmit(&tx, pdMS_TO_TICKS(5)) == ESP_OK;
}

/* ======================== 1kHz 控制管道 ======================== */
static void run_tick(uint32_t now)
{
    int64_t t0 = esp_timer_get_time();

    /* 1. 收 CAN 帧 */
    twai_message_t rx;
    while (twai_receive(&rx, 0) == ESP_OK) {
        process_rx_frame(&rx, now);
    }

    /* 2. 故障注入 */
    if (g_inj_new) {
        g_inj_new = false;
        switch (g_inj_cmd) {
        case INJ_CMD_CLEAR:
            g_status.fault_code = 0;
            g_recovery_req = true;
            break;
        case INJ_CMD_FORCE_DEGRADE:
            g_status.degraded = true;
            break;
        case INJ_CMD_FORCE_ESTOP:
            g_cmd[0].emergency_stop = true;
            break;
        case INJ_CMD_DROP_PRIMARY:
            g_link[0].control_fresh = false;
            break;
        case INJ_CMD_FORCE_LOCK:
            g_status.fault_code |= FC_FAULT_LOCK;
            break;
        case INJ_CMD_FORCE_MRM:
            g_cmd[0].mrm_request = true;
            break;
        }
    }

    /* 3. 链路评估 + 状态机 */
    LinkMonitor_t *lk = &g_link[0];
    if (now - lk->last_hb_ms >= HB_TIMEOUT_MS) {
        g_status.primary_timeout_cnt++;
        lk->fresh = false;
    }
    safety_step(now);

    /* 4. 控制计算（同 F280025C 控制负载） */
    bench_compute();
    control_step(now);

    /* 5. 负载测量 */
    int64_t t1 = esp_timer_get_time();
    int32_t elapsed = (int32_t)(t1 - t0);
    g_status.loop_load_pct = (uint16_t)((elapsed * 100) / 1000);
    if (g_status.loop_load_pct > 99) g_status.loop_load_pct = 99;
    if (elapsed > 1000) g_status.loop_overrun_cnt++;

    /* 6. 子速率发送 */
    if (now - g_last_ctrl_tx >= TX_CONTROL_PERIOD_MS) {
        g_last_ctrl_tx = now;
        uint8_t d[8];
        encode_control(d, &g_out, g_mcu_seq++);
        send_frame(CANID_MCU_CONTROL, d);
    }
    if (now - g_last_hb_tx >= TX_HEARTBEAT_PERIOD_MS) {
        g_last_hb_tx = now;
        uint8_t d[8];
        encode_heartbeat(d, &g_status, g_mcu_seq, g_alive++);
        send_frame(CANID_MCU_HEARTBEAT, d);
    }
    if (now - g_last_diag_tx >= TX_DIAG_PERIOD_MS) {
        g_last_diag_tx = now;
        uint8_t d[8];
        encode_diag(d, &g_status);
        send_frame(CANID_MCU_DIAG, d);
        encode_e2e(d, 0);
        send_frame(CANID_MCU_E2E_DIAG, d);
        encode_linkstats(d);
        send_frame(CANID_MCU_LINKSTATS, d);
    }
}

/* ======================== 定时器回调 ======================== */
static void timer_callback(void *arg)
{
    if (g_run) {
        g_overrun++;
        return;
    }
    g_run = true;
}

/* ======================== 主任务 ======================== */
static void adas_task(void *arg)
{
    ESP_LOGI(TAG, "ADAS safety stack started (1kHz)");

    while (1) {
        if (g_run) {
            g_run = false;
            g_tick_ms++;
            run_tick(g_tick_ms);
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

/* ======================== 初始化 ======================== */
void app_main(void)
{
    ESP_LOGI(TAG, "Initializing ADAS safety stack (ESP32 port)");

    /* 初始化状态 */
    memset(&g_status, 0, sizeof(g_status));
    memset(&g_out, 0, sizeof(g_out));
    memset(&g_cmd, 0, sizeof(g_cmd));
    memset(&g_link, 0, sizeof(g_link));
    g_status.system_state = SYS_MODE_INIT;
    g_status.reset_reason = 1;

    /* 初始化 TWAI */
    twai_general_config_t g_cfg =
        TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_17, GPIO_NUM_16, TWAI_MODE_NORMAL);
    g_cfg.rx_queue_len = 32;
    g_cfg.tx_queue_len = 32;
    twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_cfg, &t_cfg, &f_cfg));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI 500k up. TX=GPIO17 RX=GPIO16");

    /* 等待 CAN 稳定 */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 进入 STANDBY */
    g_status.system_state = SYS_MODE_STANDBY;

    /* 启动 1kHz 定时器 */
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .name = "adas_1khz"
    };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_periodic_start(timer, 1000));
    ESP_LOGI(TAG, "1kHz timer started");

    /* 创建主任务（运行在 core 0，最高优先级） */
    xTaskCreatePinnedToCore(adas_task, "adas", 8192, NULL, configMAX_PRIORITIES - 1, NULL, 0);

    ESP_LOGI(TAG, "Ready. Waiting for SoC on 0x100/0x101/0x102/0x103");
}
