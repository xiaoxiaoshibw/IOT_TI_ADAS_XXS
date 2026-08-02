/*
 * adas_types.h — MCU 内部数据模型（解码后的控制目标 / 链路健康 / 输出）
 *
 * 分层思路（对齐 ADAS0.0.2/SoC 的控制与状态分层）：
 *   SocCommand_t   —— 从某一个 SoC 源解码出的最新控制目标+状态（每源一份）
 *   LinkMonitor_t  —— 该源的链路新鲜度/序列停滞跟踪
 *   ControlOutput_t—— 控制+安全仲裁后的最终执行量
 *   McuStatus_t    —— MCU 自身运行/安全/诊断状态（用于回传）
 */
#ifndef ADAS_TYPES_H_
#define ADAS_TYPES_H_

#include <stdint.h>
#include <stdbool.h>

/* 一路 SoC 发来的、经 CRC/范围校验后的最新控制目标与状态 */
typedef struct
{
    /* --- 横向（0x101） --- */
    float    target_steer_deg;      /* 目标前轮转角 (°) */
    float    target_steer_rate_dps; /* 目标转角速率幅值上限 (°/s) */
    bool     lateral_enable;
    bool     lka_active;

    /* --- 纵向（0x102） --- */
    float    target_accel_ms2;      /* 目标纵向加速度 (m/s^2)，+加/-减 */
    float    target_speed_ms;       /* 目标车速 (m/s) */
    uint16_t brake_request_pct;     /* 制动请求 0..100 % */
    bool     longitudinal_enable;
    bool     acc_active;
    bool     parking_brake;
    uint16_t drive_dir;             /* DIR_* */

    /* --- ADAS 模式与安全（0x103） --- */
    uint16_t status_word;           /* 16 位 ST_* 位图 */
    uint16_t aeb_risk;              /* AEB_RISK_* */
    float    aeb_required_decel;    /* AEB 请求减速度幅值 (m/s^2, >=0) */
    bool     emergency_stop;
    bool     mrm_request;
    bool     obstacle_valid;

    /* --- SoC 心跳（0x100） --- */
    uint16_t soc_mode;              /* SYS_MODE_* */
    uint16_t soc_health;            /* >0 视为存活 */
    uint16_t fault_level;           /* FAULT_LEVEL_* */
    bool     control_authority;     /* 是否允许自动控制 */
    uint16_t source_id;             /* 心跳自报的源 ID */
} SocCommand_t;

/* 一路源的链路健康跟踪 */
typedef struct
{
    uint32_t last_hb_ms;            /* 最近一次有效心跳时刻 */
    uint32_t last_lat_ms;           /* 最近一次有效横向帧 */
    uint32_t last_lon_ms;           /* 最近一次有效纵向帧 */
    uint32_t last_status_ms;        /* 最近一次有效 ADAS 状态帧 */

    uint16_t last_hb_seq;
    uint16_t last_lat_seq;
    uint16_t last_lon_seq;
    uint16_t last_status_seq;

    uint16_t seq_stall_cnt;         /* 报文仍到达但 seq 停滞的连续计数 */
    uint16_t hb_seq_err_cnt;
    uint16_t lat_seq_err_cnt;
    uint16_t lon_seq_err_cnt;
    uint16_t status_seq_err_cnt;
    bool     seq_seen;              /* 是否已收到过至少一帧(用于首帧免判停滞) */
    bool     lat_seen;
    bool     lon_seen;
    bool     status_seen;
    bool     protocol_ok;
    bool     lat_valid;
    bool     lon_valid;
    bool     status_valid;

    uint16_t timeout_events;        /* 累计超时事件(诊断，饱和 255) */
    bool     fresh;                 /* 本 tick 结论：该源是否新鲜可用 */
    bool     control_fresh;         /* 横+纵向都新鲜(可作为控制源) */
    bool     safety_fresh;          /* 心跳+安全状态可信(可触发 AEB/MRM) */
    uint32_t fresh_since_ms;        /* 连续 fresh 起始时刻(回切迟滞用) */
    bool     fresh_since_valid;     /* 起始时刻有效；0 ms 也是合法时间点 */
} LinkMonitor_t;

/* 控制+安全仲裁后的最终执行量 */
typedef struct
{
    float    final_steer_deg;       /* 最终前轮转角 (°) */
    float    final_accel_ms2;       /* 最终纵向加速度 (m/s^2) */
    float    final_throttle;        /* 0..1 */
    float    final_brake;           /* 0..1 */
    uint16_t drive_dir;             /* DIR_* */
    bool     turn_left;             /* 转向灯请求 */
    bool     turn_right;
    bool     hazard;                /* 双闪(危险报警) */
} ControlOutput_t;

/* MCU 自身状态（回传 + HMI） */
typedef struct
{
    uint16_t system_state;          /* SYS_MODE_*（MCU 状态机当前态） */
    uint16_t active_source;         /* SRC_*（当前采信的控制源） */
    uint16_t fault_level;           /* FAULT_LEVEL_* */
    uint16_t fault_code;            /* 位图故障码，见 safety.h FC_* */
    bool     aeb_floor_active;      /* 安全地板全力制动生效中 */
    bool     degraded;              /* 处于降级 */
    bool     estop;                 /* 紧急停车中 */
    bool     manual_override;       /* 一键热冗余：人工锁定源 */
    uint16_t override_source;       /* 人工锁定到的源(SRC_PRIMARY/SRC_BACKUP) */

    /* 诊断计数（饱和 255） */
    uint16_t primary_timeout_cnt;
    uint16_t backup_timeout_cnt;
    uint16_t crc_err_cnt;
    uint16_t loop_overrun_cnt;
    uint16_t reset_reason;
    uint16_t loop_load_pct;
    uint16_t self_test_mask;
} McuStatus_t;

#endif /* ADAS_TYPES_H_ */
