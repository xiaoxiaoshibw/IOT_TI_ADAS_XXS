/*
 * adas_can_protocol.h — Orin Nano <-> F280025C CAN 帧协议契约
 *
 * ★ 这是 MCU 与 Orin 两侧共享的唯一线格式定义。任何字段/缩放/字节序改动
 *   都必须同步 Orin 侧编码器（参考实现见 tools/orin_can_reference.py），
 *   否则数据将被 CRC 或范围校验拒绝。
 *
 * 总线：经典 CAN 2.0A（11 位标准帧），500 kbps，8 字节数据场。
 * 字节序：多字节整型一律小端（LSB 在低地址），便于 x86/Python 与 C28x 对齐。
 * 校验：每帧末字节(byte7)为 CRC-8，覆盖 CAN Data ID(低字节、高字节)+byte0..byte6。
 *       CRC-8：多项式 0x31(MSB-first)、初值 0x00、无反转、无异或输出，
 *       与 ADAS0.0.2/SoC 通信侧及 tools/orin_can_reference.* 保持一致。
 * 计数：sequence 为逐帧 0..255 滚动；MCU 据此检测乱序/重复/停滞。
 *
 * 注意 C28x 平台：char = 16 bit。CAN 驱动以 uint16_t[8] 承载 8 个字节，
 *   每个元素仅低 8 位有效。本文件所有"字节"均指该数组元素的低 8 位。
 */
#ifndef ADAS_CAN_PROTOCOL_H_
#define ADAS_CAN_PROTOCOL_H_

#include <stdint.h>
#include "adas_can_protocol_v3_generated.h"

/* 协议版本：不兼容改动时递增，MCU 拒收不匹配版本(经 0x100/心跳携带) */
#define ADAS_PROTOCOL_VERSION   ADAS_PROTOCOL_VERSION_V3

#define CAN_BITRATE_BPS         500000UL
#define CAN_FRAME_DLC           8U

/* ------------------------------------------------------------------ */
/* CAN ID 分配（对齐 IOT_TI/文档/项目整体架构说明.md 第八节）          */
/* ------------------------------------------------------------------ */
/* Orin(主) → MCU：0x100 基址；Orin(备) → MCU：0x110 基址（冗余扩展）  */
#define CANID_PRIMARY_BASE      0x100U
#define CANID_BACKUP_BASE       0x110U
#define CANID_OFFS_HEARTBEAT    0x0U          /* base+0：SoC 心跳与系统状态 20ms */
#define CANID_OFFS_LATERAL      0x1U          /* base+1：横向控制目标      10ms */
#define CANID_OFFS_LONGITUDINAL 0x2U          /* base+2：纵向控制目标      10ms */
#define CANID_OFFS_ADAS_STATUS  0x3U          /* base+3：ADAS 模式与安全    20ms */

/* MCU → PC/Orin */
#define CANID_MCU_CONTROL       0x201U        /* 最终控制输出  10ms */
#define CANID_MCU_HEARTBEAT     0x202U        /* MCU 心跳       20ms */
#define CANID_MCU_DIAG          0x203U        /* 故障与诊断    100ms */
#define CANID_MCU_E2E_DIAG      0x204U        /* E2E/CAN/自检扩展诊断 100ms */
#define CANID_MCU_LINKSTATS     0x205U        /* 链路间隔诊断  100ms */

/* PC → MCU / MCU → PC：故障注入（非周期，测试用） */
#define CANID_FAULT_INJECT      0x301U
#define CANID_FAULT_RESPONSE    0x302U

/* ------------------------------------------------------------------ */
/* 定点缩放（工程量 = 原始整型 * SCALE）                               */
/* ------------------------------------------------------------------ */
#define SCALE_STEER_DEG         0.01f         /* int16：0.01°/LSB，±327.67° */
#define SCALE_STEER_RATE_DPS    0.1f          /* int16：0.1 (°/s)/LSB */
#define SCALE_ACCEL_MS2         0.001f        /* int16：0.001 (m/s^2)/LSB，±32.7 */
#define SCALE_SPEED_MS          0.01f         /* int16：0.01 (m/s)/LSB */

/* ------------------------------------------------------------------ */
/* 枚举（与 adas_types.h 保持一致）                                    */
/* ------------------------------------------------------------------ */
/* 系统运行模式（0x100 byte0 / 0x202 byte0） */
#define SYS_MODE_INIT           0U
#define SYS_MODE_STANDBY        1U
#define SYS_MODE_ACTIVE         2U
#define SYS_MODE_DEGRADED       3U
#define SYS_MODE_MRM            4U            /* 最小风险策略 */
#define SYS_MODE_EMERGENCY_BRAKE 5U
#define SYS_MODE_FAILSAFE       6U
#define SYS_MODE_FAULT_LOCK     7U

/* 故障等级 */
#define FAULT_LEVEL_INFO        0U
#define FAULT_LEVEL_WARN        1U
#define FAULT_LEVEL_SEVERE      2U
#define FAULT_LEVEL_FATAL       3U

/* AEB 风险等级（0x103 byte2） */
#define AEB_RISK_NONE           0U
#define AEB_RISK_WARNING        1U
#define AEB_RISK_PARTIAL        2U
#define AEB_RISK_FULL           3U

/* 控制来源 / active_source（0x202 byte1，0x201 byte6 高半字节） */
#define SRC_NONE                0U
#define SRC_PRIMARY             1U
#define SRC_BACKUP              2U
#define SRC_WATCHDOG            9U            /* 看门狗安全地板接管 */

/* 驱动方向（0x102 byte5 bit4-5） */
#define DIR_NEUTRAL             0U
#define DIR_DRIVE               1U
#define DIR_REVERSE            2U

/* ------------------------------------------------------------------ */
/* 统一 ADAS 状态字（0x103 byte0=低 8 位, byte1=高 8 位）              */
/* ------------------------------------------------------------------ */
#define ST_CONTROL_ENABLE       (1U << 0)
#define ST_LATERAL_ENABLE       (1U << 1)
#define ST_LONGITUDINAL_ENABLE  (1U << 2)
#define ST_LKA_ACTIVE           (1U << 3)
#define ST_ACC_ACTIVE           (1U << 4)
#define ST_AEB_WARNING          (1U << 5)
#define ST_AEB_BRAKING          (1U << 6)
#define ST_EMERGENCY_STOP       (1U << 7)
#define ST_DEGRADED_MODE        (1U << 8)
#define ST_TAKEOVER_REQUEST     (1U << 9)
#define ST_MRM_REQUEST          (1U << 10)
#define ST_PERCEPTION_VALID     (1U << 11)
#define ST_LOCALIZATION_VALID   (1U << 12)
#define ST_PLANNING_VALID       (1U << 13)
/* bit14/15 预留 */

/* ------------------------------------------------------------------ */
/* 报文字节布局（uint16_t data[8]，每元素低 8 位）                     */
/* ------------------------------------------------------------------ */
/* 0x100/0x110 SoC 心跳（20ms）
 *   [0] system_mode      [1] soc_health(>0=活)  [2] fault_level
 *   [3] bit0=control_authority, bit4..7=protocol_version  [4] source_id(1主/2备)
 *   [5] sequence  [6] alive_counter  [7] crc8
 */
#define HB_B_SYS_MODE      0
#define HB_B_SOC_HEALTH    1
#define HB_B_FAULT_LEVEL   2
#define HB_B_AUTHORITY     3
#define HB_B_SOURCE_ID     4
#define HB_B_SEQ           5
#define HB_B_ALIVE         6
#define HB_B_CRC           7
#define HB_F_AUTHORITY      (1U << 0)
#define HB_VERSION_SHIFT    4U
#define HB_VERSION_MASK     0x0FU

/* 0x101/0x111 横向控制目标（10ms）
 *   [0..1] int16 target_steering_angle (0.01°)
 *   [2..3] int16 target_steering_rate  (0.1 °/s, 幅值上限)
 *   [4] flags: bit0 lateral_enable, bit1 lka_active
 *   [5] sequence  [6] reserved  [7] crc8
 */
#define LAT_B_ANGLE_LO     0
#define LAT_B_ANGLE_HI     1
#define LAT_B_RATE_LO      2
#define LAT_B_RATE_HI      3
#define LAT_B_FLAGS        4
#define LAT_B_SEQ          5
#define LAT_B_CRC          7
#define LAT_F_ENABLE       (1U << 0)
#define LAT_F_LKA_ACTIVE   (1U << 1)

/* 0x102/0x112 纵向控制目标（10ms）
 *   [0..1] int16 target_acceleration (0.001 m/s^2, 有符号)
 *   [2..3] int16 target_speed        (0.01 m/s, >=0)
 *   [4] brake_request 0..100 %
 *   [5] flags: bit0 long_enable, bit1 acc_active, bit2 parking, bit4-5 drive_dir
 *   [6] sequence  [7] crc8
 */
#define LON_B_ACC_LO       0
#define LON_B_ACC_HI       1
#define LON_B_SPD_LO       2
#define LON_B_SPD_HI       3
#define LON_B_BRAKE        4
#define LON_B_FLAGS        5
#define LON_B_SEQ          6
#define LON_B_CRC          7
#define LON_F_ENABLE       (1U << 0)
#define LON_F_ACC_ACTIVE   (1U << 1)
#define LON_F_PARKING      (1U << 2)
#define LON_F_DIR_SHIFT    4                  /* bit4-5 = drive_direction */
#define LON_F_DIR_MASK     0x3U

/* 0x103/0x113 ADAS 模式与安全（20ms）
 *   [0] status_lo  [1] status_hi   (16 位 ADAS 状态字)
 *   [2] aeb_risk_level 0..3
 *   [3..4] int16 aeb_required_decel (0.001 m/s^2, 幅值 >=0)
 *   [5] flags: bit0 emergency_stop, bit1 mrm_request, bit2 obstacle_valid
 *   [6] sequence  [7] crc8
 */
#define AD_B_STATUS_LO     0
#define AD_B_STATUS_HI     1
#define AD_B_AEB_RISK      2
#define AD_B_DECEL_LO      3
#define AD_B_DECEL_HI      4
#define AD_B_FLAGS         5
#define AD_B_SEQ           6
#define AD_B_CRC           7
#define AD_F_ESTOP         (1U << 0)
#define AD_F_MRM           (1U << 1)
#define AD_F_OBSTACLE_VALID (1U << 2)

/* 0x201 MCU 最终控制输出（10ms）
 *   [0..1] int16 final_steering_angle (0.01°)
 *   [2..3] int16 final_acceleration   (0.001 m/s^2)
 *   [4] final_throttle 0..100 %   [5] final_brake 0..100 %
 *   [6] MCU output sequence
 *   [7] crc8
 */
#define OUT_B_STEER_LO     0
#define OUT_B_STEER_HI     1
#define OUT_B_ACC_LO       2
#define OUT_B_ACC_HI       3
#define OUT_B_THROTTLE     4
#define OUT_B_BRAKE        5
#define OUT_B_SEQ          6
#define OUT_B_CRC          7

/* 0x202 MCU 心跳（20ms）
 *   [0] mcu_state  [1] active_source  [2] safety_flags  [3] fault_level
 *   [4] mcu_seq    [5] alive_counter  [6] loop_load_pct  [7] crc8
 */
#define MHB_B_STATE        0
#define MHB_B_SOURCE       1
#define MHB_B_SAFEFLAGS    2
#define MHB_B_FAULT_LEVEL  3
#define MHB_B_SEQ          4
#define MHB_B_ALIVE        5
#define MHB_B_LOAD         6
#define MHB_B_CRC          7
#define SF_PRIMARY_FRESH   (1U << 0)
#define SF_BACKUP_FRESH    (1U << 1)
#define SF_AEB_FLOOR       (1U << 2)
#define SF_DEGRADED        (1U << 3)
#define SF_BUZZER_ON       (1U << 4)
#define SF_MANUAL_OVERRIDE (1U << 5)
#define SF_ESTOP           (1U << 6)

/* 0x203 MCU 诊断（100ms）
 *   [0..1] fault_code(u16)  [2] reset_reason  [3] primary_timeout_cnt
 *   [4] backup_timeout_cnt  [5] crc_err_cnt   [6] loop_overrun_cnt  [7] crc8
 */
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
#define E2E_PF_PRI_OK       (1U << 0)
#define E2E_PF_BAK_OK       (1U << 1)
#define E2E_PF_CAN_OK       (1U << 2)
#define E2E_PF_RECOVERY_EXHAUSTED (1U << 3)
#define E2E_BF_TEST_BUILD   (1U << 0)
#define E2E_BF_TCP_AUTH_HIL (1U << 1)

/* 0x205 MCU 链路间隔诊断（100ms）——MCU 视角独立测量网关发送连续性。
 * 仅统计"链路存活期间"的有效帧到达间隔（CRC+seq 通过才计），间隔超
 * GAP_LINK_RESTART_MS 视为链路重启，重置基线不计入。数值饱和 255。
 * ctrl = 0x101/0x102 合并（max 取劣、miss 求和），slow = 0x100/0x103 合并。
 * min_gap 用于识别"冻结后补发历史帧"的突发（接近 0ms 的连发）。
 *   [0] pri_ctrl_max_gap_ms  [1] pri_ctrl_miss_cnt
 *   [2] pri_slow_max_gap_ms  [3] pri_slow_miss_cnt
 *   [4] pri_ctrl_min_gap_ms（255=尚无数据）
 *   [5] bak_ctrl_max_gap_ms  [6] bak_ctrl_miss_cnt
 *   [7] crc8
 */
#define LS_B_PRI_CTRL_MAX   0
#define LS_B_PRI_CTRL_MISS  1
#define LS_B_PRI_SLOW_MAX   2
#define LS_B_PRI_SLOW_MISS  3
#define LS_B_PRI_CTRL_MIN   4
#define LS_B_BAK_CTRL_MAX   5
#define LS_B_BAK_CTRL_MISS  6
#define LS_B_CRC            7

/* 0x301 故障注入命令（PC→MCU） */
#define INJ_CMD_CLEAR         0U
#define INJ_CMD_FORCE_DEGRADE 1U
#define INJ_CMD_FORCE_ESTOP   2U
#define INJ_CMD_DROP_PRIMARY  3U
#define INJ_CMD_DROP_BACKUP   4U
#define INJ_CMD_FORCE_LOCK    5U
#define INJ_CMD_DROP_ALL      6U           /* 双源同时失效 → FAILSAFE */
#define INJ_CMD_FORCE_MRM     7U           /* 模拟可信源 MRM 请求 → MRM */
#define INJ_CMD_CAN_EXHAUSTED 8U           /* 模拟 Bus-Off 恢复耗尽 → FAULT_LOCK */
#define INJ_CMD_SELF_TEST_FAIL 9U          /* 模拟自检失败 → FAULT_LOCK */
#define INJ_B_CMD          0
#define INJ_B_PARAM        1
#define INJ_B_SEQ          5
#define INJ_B_CRC          7

/* 0x302 注入响应（MCU→PC）。MCU 在 Safety_applyInject 后回送一帧。
 *   [0] cmd 回显                 [1] param 回显
 *   [2] system_state             [3] fault_level
 *   [4] alive（自增，标识 MCU 仍存活）
 *   [5] resp_seq（响应序号，每次回送 +1）
 *   [6] fault_code 低 8 位
 *   [7] crc8
 * 测试脚本读 [5] seq 即可断言响应已回送（或监测 seq 停滞/超时判定 MCU 死锁）。 */
#define INJ_RESP_B_CMD         0
#define INJ_RESP_B_PARAM       1
#define INJ_RESP_B_STATE       2
#define INJ_RESP_B_FAULT_LEVEL 3
#define INJ_RESP_B_ALIVE       4
#define INJ_RESP_B_SEQ         5
#define INJ_RESP_B_FAULT_LO    6
#define INJ_RESP_B_CRC         7

#endif /* ADAS_CAN_PROTOCOL_H_ */
