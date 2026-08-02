/*
 * adas_config.h — ADAS MCU 集中配置（所有可调参数）
 *
 * 目标板：TI LAUNCHXL-F280025C（TMS320F280025C，C28x @ 100 MHz）
 * 约定：所有可调参数集中于此，其它模块通过 include 引用，禁止在算法文件散布字面量。
 *
 * 引脚映射见 board.h；CAN 帧协议见 adas_can_protocol.h。
 */
#ifndef ADAS_CONFIG_H_
#define ADAS_CONFIG_H_

/* ------------------------------------------------------------------ */
/* 时钟 / 控制环                                                       */
/* ------------------------------------------------------------------ */
#define DEVICE_SYSCLK_HZ        100000000UL   /* 100 MHz，与 Device_init() 一致 */
#define DEVICE_EPWMCLK_HZ       (DEVICE_SYSCLK_HZ / 2UL)

#define CTRL_LOOP_HZ            1000U         /* 1 kHz 硬实时控制环 */
#define CTRL_TICK_US            (1000000UL / CTRL_LOOP_HZ)   /* 1000 us */
#define CTRL_DT_S               (1.0f / (float)CTRL_LOOP_HZ) /* 0.001 s */

/* 子速率分频（以 1 ms tick 为单位）：TX 报文周期 */
#define TX_CONTROL_PERIOD_MS    10U           /* 0x201 最终控制输出 10 ms */
#define TX_HEARTBEAT_PERIOD_MS  20U           /* 0x202 MCU 心跳 20 ms */
#define TX_DIAG_PERIOD_MS       100U          /* 0x203 故障诊断 100 ms */

/* ------------------------------------------------------------------ */
/* 通信超时 / 新鲜度（毫秒）                                           */
/* ------------------------------------------------------------------ */
/* Orin→MCU 报文周期：心跳 20 ms、横/纵向 10 ms、ADAS 状态 20 ms       */
#define HB_TIMEOUT_MS           100U          /* 心跳静默 100 ms(5 帧) 视为丢失 */
#define CTRL_TIMEOUT_MS         60U           /* 横/纵向静默 60 ms(6 帧) 视为丢失 */
#define STATUS_TIMEOUT_MS       120U          /* ADAS 状态静默 120 ms 视为陈旧 */
#define SEQ_STALL_LIMIT         5U            /* seq 停滞 5 帧(报文仍到但计数不变)判挂起 */
/* 横/纵向成对控制帧可能跨过 1 ms 仲裁边界。仅在此前已经收到同 seq 对的前提下，
 * 容忍相邻 seq 最多连续两个 tick；持续错位仍按不相干处理。 */
#define CONTROL_PAIR_SKEW_GRACE_MS     2U
#define CONTROL_PAIR_SKEW_GRACE_TICKS  2U

/* 备机自动接管迟滞：主控恢复后需连续 fresh 该时长才回切，抑制抖动 */
/* HIL safety timing budgets, in milliseconds. These compile-time guards
 * prevent a configuration change from silently exceeding the agreed budget;
 * they do not replace timestamped measurements on the target hardware. */
#define SAFETY_ARBITRATION_BUDGET_MS  1U
#define SAFETY_OUTPUT_BUDGET_MS       15U
#define FTTI_CONTROL_LOSS_MS          100U
#define FTTI_HEARTBEAT_LOSS_MS        140U
#define FTTI_E2E_REJECT_MS            16U
#define FTTI_AEB_MS                   16U
#define FTTI_MRM_REQUEST_MS           16U
#define FTTI_ALL_SOURCES_LOST_MS      100U

/* v3 会话语义已重构为自驱授权门：会话 ACTIVE→RECOVERY 的回落是“启用闩锁”级别的
 * 事件，不是“控制停权”路径。控制停权仍完全由 safety.c 的 control_fresh（CTRL/HB/
 * STATUS 超时窗 + 仲裁 + 输出预算）在 FTTI 内保证。因此 FTTI 守卫不再计入会话陈旧
 * 确认窗口；下面的预算只覆盖实际的停权时序。 */
#if ((CTRL_TIMEOUT_MS + SAFETY_ARBITRATION_BUDGET_MS + SAFETY_OUTPUT_BUDGET_MS) > FTTI_CONTROL_LOSS_MS)
#error "Control-loss FTTI budget exceeded"
#endif
#if ((HB_TIMEOUT_MS + SAFETY_ARBITRATION_BUDGET_MS + SAFETY_OUTPUT_BUDGET_MS) > FTTI_HEARTBEAT_LOSS_MS)
#error "Heartbeat-loss FTTI budget exceeded"
#endif
#if ((SAFETY_ARBITRATION_BUDGET_MS + SAFETY_OUTPUT_BUDGET_MS) > FTTI_E2E_REJECT_MS)
#error "E2E rejection FTTI budget exceeded"
#endif
#if ((SAFETY_ARBITRATION_BUDGET_MS + SAFETY_OUTPUT_BUDGET_MS) > FTTI_AEB_MS)
#error "AEB FTTI budget exceeded"
#endif
#if ((SAFETY_ARBITRATION_BUDGET_MS + SAFETY_OUTPUT_BUDGET_MS) > FTTI_MRM_REQUEST_MS)
#error "MRM-request FTTI budget exceeded"
#endif
#if ((CTRL_TIMEOUT_MS + SAFETY_ARBITRATION_BUDGET_MS + SAFETY_OUTPUT_BUDGET_MS) > FTTI_ALL_SOURCES_LOST_MS)
#error "All-sources-lost FTTI budget exceeded"
#endif

#define FAILBACK_HOLD_MS        300U
#define EMERGENCY_HOLD_MS       500U

/* Fault injection is compiled out of normal firmware images. */
#ifndef ADAS_TEST_BUILD
#define ADAS_TEST_BUILD         0
#endif
#define CAN_RECOVERY_DELAY_MS   100U
#define CAN_RECOVERY_MAX_TRIES  3U
/* 快速重试预算(MAX_TRIES)耗尽后，不再永久放弃：以此更慢的节奏持续尝试
 * 重新入网。这样 HIL 中上位机/收发器故障排除后，MCU 无需整机复位即可自动
 * 恢复 CAN 通信（总线一旦重新健康，快速重试预算立即清零复位）。 */
#define CAN_RECOVERY_REARM_MS   1000U

/* 链路间隔统计（0x205）：MCU 作为接收端独立测量网关发送连续性。
 * 到达间隔超阈值记 miss；超 GAP_LINK_RESTART_MS 视为链路重启，
 * 仅重置基线不计入统计（区分"运行中断续"与"整改允许的重启停顿"）。 */
#define GAP_THRESH_CTRL_MS      30U           /* 0x101/0x102，标称 10ms（=网关 TX deadline） */
#define GAP_THRESH_SLOW_MS      60U           /* 0x100/0x103，标称 20ms（=网关 TX deadline） */
#define GAP_LINK_RESTART_MS     500U

/* ------------------------------------------------------------------ */
/* 片上看门狗 / 安全地板                                               */
/* ------------------------------------------------------------------ */
/* 控制环卡死超过该时长 → 片上 WD 硬复位（在 ISR 内喂狗）              */
#define WATCHDOG_ENABLE         1
#define WATCHDOG_TIMEOUT_MS     52U
/* 双源全部失联 / 紧急停车 → 安全地板全力制动减速度 */
#define SAFETY_FLOOR_DECEL_MS2  8.0f

/* ------------------------------------------------------------------ */
/* 控制量限幅 / 变化率限制（最终执行前的最后一道执行器保护层）         */
/* ------------------------------------------------------------------ */
#define MAX_STEER_DEG           30.0f         /* 前轮转角上限 ±30° */
#define MCU_MAX_STEER_RATE_DPS  400.0f        /* MCU 侧转角变化率硬上限 °/s */
#define MAX_ACCEL_MS2           3.0f          /* 目标加速上限 */
#define MAX_DECEL_MS2           8.0f          /* 目标/制动减速上限(全力制动) */

/* 加速度→油门、减速度→制动 的简化线性映射（仿真执行器；实车需标定）  */
#define ACCEL_TO_THROTTLE_GAIN  (1.0f / MAX_ACCEL_MS2)   /* accel(m/s^2)→[0,1] */
#define DECEL_TO_BRAKE_GAIN     (1.0f / MAX_DECEL_MS2)   /* decel(m/s^2)→[0,1] */

/* 极低速判定（用于停车保持 / 蠕行抑制） */
#define STANDSTILL_SPEED_MS     0.15f

/* ------------------------------------------------------------------ */
/* 控制映射 / 平顺 / 降级 ramp                                         */
/* ------------------------------------------------------------------ */
/* 加速度变化率(jerk)上限：把目标加速度朝目标逐 tick 逼近，消除源切换/
 * 接管时的纵向冲击（对应主项目"备机热待机消除全力制动冲击"的思路）。 */
#define ACCEL_SLEW_MS3          8.0f          /* m/s^3 */
/* 失效态方向回中速率：通信全丢时方向盘以此速率平滑回正 */
#define STEER_RETURN_DPS        60.0f         /* °/s */
/* 通信全丢(FAILSAFE)：受控减速到安全地板（走 jerk 限幅，非急停） */
#define FAILSAFE_DECEL_MS2      SAFETY_FLOOR_DECEL_MS2
/* Minimum-risk manoeuvre uses a gentler, explicitly calibrated deceleration. */
#define MRM_DECEL_MS2           4.8f
/* 紧急制动(EMERGENCY/AEB 全制动/急停)：1=立即全力制动，跳过 jerk 限幅 */
#define ESTOP_IMMEDIATE         1
/* 上电到 STANDBY 的稳定等待（等 CAN/外设就绪） */
#define INIT_SETTLE_MS          200U

/* ------------------------------------------------------------------ */
/* 执行器：舵机 / 闪光灯 / 蜂鸣器（外设行为）                          */
/* ------------------------------------------------------------------ */
/* 舵机：标准 50 Hz PWM，1000~2000 us 脉宽，1500 us 居中 */
#define SERVO_PERIOD_US         20000U
#define SERVO_MIN_US            1000U
#define SERVO_MID_US            1500U
#define SERVO_MAX_US            2000U
/* 转角→脉宽方向：+MAX_STEER_DEG 映射到 SERVO_MAX_US（左打满），可翻转 */
#define SERVO_INVERT            0

/* 转向灯：|最终转角| 超阈值点亮对应侧，失效态双闪(危险报警) */
#define TURN_SIGNAL_ON_DEG      5.0f          /* 转角门限 */
#define TURN_SIGNAL_OFF_DEG     2.5f          /* 回中滞回门限 */
#define TURN_BLINK_HALF_MS      320U          /* ~1.56 Hz 闪烁半周期 */

/* 蜂鸣器：安全降级/失效鸣叫；源切换短促提示 */
#define BUZZER_DEGRADED_ON_MS   400U          /* 降级：长鸣占空 */
#define BUZZER_DEGRADED_OFF_MS  400U
#define BUZZER_CRITICAL_ON_MS   120U          /* 严重(失效/急停)：急促 */
#define BUZZER_CRITICAL_OFF_MS  120U
#define BUZZER_CHIRP_MS         150U          /* 源切换一次性提示时长 */
/* 有源蜂鸣器=GPIO 通断即可(BUZZER_ACTIVE=1)；无源蜂鸣器需 PWM 载频 */
#define BUZZER_ACTIVE           1
#define BUZZER_TONE_HZ          2700U         /* 无源蜂鸣器载频(仅 BUZZER_ACTIVE=0) */

/* ------------------------------------------------------------------ */
/* HIL 会话门时序（v3 自驱授权：无按钮/无需 TCP 授权）                   */
/* ------------------------------------------------------------------ */
/* READY 持续 fresh+safe 满该时长后自动 ARM。给链路一个稳定性浸泡，避免在
 * 抖动链路上莽撞授权。不属于 FTTI 反应时序（执行器停权由 safety.c 独立保证）。 */
#define HIL_AUTO_ARM_DELAY_MS        300U
/* RECOVERY_REQUIRED 持续 fresh+safe 满该时长后自动回到 SESSION_ACCEPTED（同
 * session），网关观察到 0x206 后清除其 recovery 闩锁并恢复 SYNC_STANDBY。 */
#define HIL_AUTO_RECOVER_DELAY_MS    500U
/* ACTIVE 持续不新鲜满该时长才回落 RECOVERY（启用级事件，非停权）。短于该窗口的
 * 抖动不撤授权；safety.c 仍会在 60 ms 内独立进入 FAILSAFE/DEGRADED 停权。 */
#define HIL_SESSION_DROP_MS          1000U
/* FAULT_LOCK 在上电因"看门狗复位 / 自检失败"闩锁时，保留该 boot hold 让操作员
 * 通过 CAN 0x202 心跳可见（system_state 字段），超时后条件清洁即自动解闩。 */
#define FAULT_LOCK_BOOT_HOLD_MS       1000U

/* ------------------------------------------------------------------ */
/* HMI：一键主备热冗余按钮（v3 重构后仅保留长按清故障）               */
/* ------------------------------------------------------------------ */
#define BUTTON_DEBOUNCE_MS      30U           /* 去抖窗口 */
#define BUTTON_LONGPRESS_MS     1200U         /* 长按：清故障并回到 AUTO */
/* 短按在新模型中无语义（不再授权 ARM）；保留手势识别以便将来扩展。 */

/* ------------------------------------------------------------------ */
/* 冗余：主/备源与仲裁策略                                             */
/* ------------------------------------------------------------------ */
/* 1 = 启用备机流(0x110~0x113)与一键热冗余；0 = 仅单 Orin(优先级 1 最简) */
#define REDUNDANCY_ENABLE       0
/* 备机 CAN ID 基址(=主机基址+0x10)，见 adas_can_protocol.h */

/* ------------------------------------------------------------------ */
/* 迪文 DGUS 触摸屏（DMG80480C043_02WTC）—— §5 死代码门控            */
/* ------------------------------------------------------------------ */
/* 1 = 启用 SCIA 上的 DGUS 触摸串口屏（包含 init/service + DGUS 状态
 *      上屏与触屏回调）；0 = 完全不编译相关代码与 main 路径上的调用。
 * 默认 0：阶段 1 HIL 台架尚未配 DGUS 屏，连线未接时启用会拖慢启动。 */
#define DGUS_ENABLE             0

#endif /* ADAS_CONFIG_H_ */
