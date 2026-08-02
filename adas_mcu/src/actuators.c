/*
 * actuators.c — 外设驱动实现
 *
 * 舵机：final_steer_deg(±MAX_STEER_DEG) 线性映射到 [SERVO_MIN_US, SERVO_MAX_US]，
 *       居中 SERVO_MID_US；SERVO_INVERT 可翻转左右。
 * 转向灯：out->turn_left/right/hazard 决定哪侧闪，TURN_BLINK_HALF_MS 控制节奏。
 * 蜂鸣器：优先级 急停/失效(急促) > 降级(长鸣) > 一次性切源短鸣。
 *         有源蜂鸣器直接 GPIO 通断；无源用 EPWM2A 载频（BUZZER_ACTIVE=0）。
 * 状态 LED：RUN 常规心跳呼吸(1Hz 方波)，FAULT 跟随 degraded/estop 常亮/快闪。
 */
#include "actuators.h"
#include "board.h"
#include "adas_config.h"
#include "adas_can_protocol.h"

#include "driverlib.h"
#include "device.h"

/* 转向灯闪烁相位 */
static uint32_t s_blink_ref_ms = 0U;
static bool     s_blink_on     = false;

/* 蜂鸣器一次性短鸣 */
static uint32_t s_chirp_until  = 0U;
static bool     s_chirp_active = false;

/* 切源检测（源变化自动短鸣提示） */
static uint16_t s_last_source  = SRC_NONE;
static bool     s_source_seen  = false;

#if !BUZZER_ACTIVE
static bool s_buzzerPwmInit = false;
static void buzzer_pwm_init(void)
{
    /* 无源蜂鸣器：EPWM2A 产生 BUZZER_TONE_HZ 方波 */
    GPIO_setPinConfig(BOARD_BUZZER_PINCFG);
    GPIO_setPadConfig(BOARD_BUZZER_PWM_GPIO, GPIO_PIN_TYPE_STD);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    EPWM_setClockPrescaler(BOARD_BUZZER_EPWM_BASE,
                           EPWM_CLOCK_DIVIDER_4, EPWM_HSCLOCK_DIVIDER_2);
    {
        /* TBCLK = 100MHz/8 = 12.5MHz；PRD = TBCLK/(2*f)-1（up-down 近似用 up） */
        uint32_t prd = (DEVICE_EPWMCLK_HZ / 8UL / BUZZER_TONE_HZ) - 1UL;
        EPWM_setTimeBasePeriod(BOARD_BUZZER_EPWM_BASE, (uint16_t)prd);
        EPWM_setCounterCompareValue(BOARD_BUZZER_EPWM_BASE,
                                    EPWM_COUNTER_COMPARE_A, (uint16_t)(prd / 2U));
    }
    EPWM_setTimeBaseCounterMode(BOARD_BUZZER_EPWM_BASE, EPWM_COUNTER_MODE_UP);
    EPWM_setActionQualifierAction(BOARD_BUZZER_EPWM_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(BOARD_BUZZER_EPWM_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    s_buzzerPwmInit = true;
}
#endif

static void buzzer_set(bool on)
{
#if BUZZER_ACTIVE
    Board_writeGpio(BOARD_BUZZER_GPIO, on);
#else
    /* 无源：发声=放行 AQ 由 CMPA 产生载频；静音=SW 强制输出低 */
    if (on)
    {
        EPWM_setActionQualifierContSWForceAction(BOARD_BUZZER_EPWM_BASE,
                                                 EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
    }
    else
    {
        EPWM_setActionQualifierContSWForceAction(BOARD_BUZZER_EPWM_BASE,
                                                 EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_OUTPUT_LOW);
    }
#endif
}

void Actuators_init(void)
{
    s_blink_ref_ms = 0U;
    s_blink_on = false;
    s_chirp_active = false;
    s_last_source = SRC_NONE;
    s_source_seen = false;

    Board_setServoMicros(SERVO_MID_US);
    Board_setServo2Micros(SERVO_MID_US);  /* 舵机2 与舵机1 同步转向 */
    Board_writeGpio(BOARD_LIGHT_LEFT_GPIO, false);
    Board_writeGpio(BOARD_LIGHT_RIGHT_GPIO, false);
#if !BUZZER_ACTIVE
    if (!s_buzzerPwmInit) { buzzer_pwm_init(); }
#endif
    buzzer_set(false);
    Board_writeGpio(BOARD_LED_RUN_GPIO, false);
    Board_writeGpio(BOARD_LED_FAULT_GPIO, false);
}

/* 转角 → 舵机脉宽（同时驱动舵机1和舵机2，消除编译器差异） */
static void drive_servo(float steer_deg)
{
    float s = steer_deg;
    float span;
    float us;
    uint16_t micros;
    if (s >  MAX_STEER_DEG) { s =  MAX_STEER_DEG; }
    if (s < -MAX_STEER_DEG) { s = -MAX_STEER_DEG; }
#if SERVO_INVERT
    s = -s;
#endif
    span = (float)(SERVO_MAX_US - SERVO_MID_US);      /* 半行程 us */
    us = (float)SERVO_MID_US + (s / MAX_STEER_DEG) * span;
    micros = (uint16_t)(us + 0.5f);
    Board_setServoMicros(micros);
    Board_setServo2Micros(micros);
}

/* 闪烁相位推进：以 TURN_BLINK_HALF_MS 翻转 */
static void advance_blink(uint32_t now)
{
    if ((now - s_blink_ref_ms) >= TURN_BLINK_HALF_MS)
    {
        s_blink_ref_ms = now;
        s_blink_on = !s_blink_on;
    }
}

void Actuators_chirp(uint32_t now_ms)
{
    s_chirp_active = true;
    s_chirp_until = now_ms + BUZZER_CHIRP_MS;
}

bool Actuators_update(uint32_t now_ms, const ControlOutput_t *out,
                      const McuStatus_t *st)
{
    bool buzz = false;

    /* -------- 舵机 -------- */
    drive_servo(out->final_steer_deg);              /* 舵机1+2：同步转向 */

    /* -------- 转向灯 / 危险报警 -------- */
    advance_blink(now_ms);
    if (out->hazard)
    {
        /* 双闪：两侧同步 */
        Board_writeGpio(BOARD_LIGHT_LEFT_GPIO,  s_blink_on);
        Board_writeGpio(BOARD_LIGHT_RIGHT_GPIO, s_blink_on);
    }
    else
    {
        Board_writeGpio(BOARD_LIGHT_LEFT_GPIO,  out->turn_left  && s_blink_on);
        Board_writeGpio(BOARD_LIGHT_RIGHT_GPIO, out->turn_right && s_blink_on);
    }

    /* -------- 切源自动短鸣 -------- */
    if (!s_source_seen)
    {
        s_last_source = st->active_source;
        s_source_seen = true;
    }
    else if (st->active_source != s_last_source)
    {
        /* 采信源变化(主↔备/接管) → 一次性提示 */
        Actuators_chirp(now_ms);
        s_last_source = st->active_source;
    }

    /* -------- 蜂鸣器优先级 -------- */
    if (st->estop || st->aeb_floor_active ||
        st->system_state == SYS_MODE_EMERGENCY_BRAKE ||
        st->system_state == SYS_MODE_FAILSAFE ||
        st->system_state == SYS_MODE_FAULT_LOCK)
    {
        /* 严重：急促 on/off */
        uint32_t period = BUZZER_CRITICAL_ON_MS + BUZZER_CRITICAL_OFF_MS;
        uint32_t ph = (now_ms) % period;
        buzz = (ph < BUZZER_CRITICAL_ON_MS);
    }
    else if (st->degraded || st->system_state == SYS_MODE_DEGRADED ||
             st->system_state == SYS_MODE_MRM)
    {
        /* 降级：长鸣 */
        uint32_t period = BUZZER_DEGRADED_ON_MS + BUZZER_DEGRADED_OFF_MS;
        uint32_t ph = (now_ms) % period;
        buzz = (ph < BUZZER_DEGRADED_ON_MS);
    }
    else if (s_chirp_active)
    {
        /* 一次性切源提示 */
        if ((int32_t)(now_ms - s_chirp_until) >= 0) { s_chirp_active = false; }
        buzz = s_chirp_active;
    }
    buzzer_set(buzz);

    /* -------- 状态 LED -------- */
    /* RUN：1Hz 方波(存活心跳) */
    Board_writeGpio(BOARD_LED_RUN_GPIO, ((now_ms % 1000U) < 500U));
    /* FAULT：正常灭；降级慢闪；严重常亮 */
    if (st->estop || st->aeb_floor_active)
    {
        Board_writeGpio(BOARD_LED_FAULT_GPIO, true);
    }
    else if (st->degraded)
    {
        Board_writeGpio(BOARD_LED_FAULT_GPIO, ((now_ms % 500U) < 250U));
    }
    else
    {
        Board_writeGpio(BOARD_LED_FAULT_GPIO, false);
    }

    return buzz;
}
