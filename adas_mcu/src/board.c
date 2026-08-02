/*
 * board.c — 板级初始化实现（driverlib）
 *
 * 覆盖：系统时钟、GPIO 方向/上拉、CANA pinmux、舵机 EPWM(50Hz)、
 *       1 kHz CPU Timer0、片上看门狗、复位原因。
 *
 * 注：Device_init()/Device_initGPIO() 由 C2000Ware device 支持包提供
 *     （device.c/device.h），将 SYSCLK 配到 100 MHz。
 */
#include "board.h"
#include "adas_config.h"
#include "adas_can_protocol.h"
#include "asr_pro.h"

#include "driverlib.h"
#include "device.h"

/* 舵机 TBCLK = 100MHz / (16*2) = 3.125 MHz → 3.125 counts/us = 25/8 */
#define SERVO_TB_DIV        32UL
#define SERVO_TB_PERIOD     ((uint16_t)((DEVICE_EPWMCLK_HZ / SERVO_TB_DIV / 50UL) - 1UL))
#define SERVO_US_TO_CMP(us) ((uint16_t)(((uint32_t)(us) * (DEVICE_EPWMCLK_HZ / 1000000UL)) / SERVO_TB_DIV))

static uint16_t s_resetReason = 0U;

/* ------------------------------------------------------------------ */
static void board_initGpioOut(uint16_t gpio, bool initialHigh)
{
    /* 这些引脚复位后默认即为 GPIO 功能(mux 选项 0)，无需再设 pinmux，
     * 仅配置方向/驱动/初值即可，避免误改其它引脚复用。 */
    GPIO_setDirectionMode(gpio, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(gpio, GPIO_PIN_TYPE_STD);
    GPIO_writePin(gpio, initialHigh ? 1U : 0U);
}

static void board_initGpioIn(uint16_t gpio)
{
    GPIO_setDirectionMode(gpio, GPIO_DIR_MODE_IN);
    /* 三针按钮模块由 3.3 V 供电并主动驱动 OUT：松开低、按下高。
     * 不启用内部上拉，避免与模块的低电平输出对拉。 */
    GPIO_setPadConfig(gpio, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(gpio, GPIO_QUAL_6SAMPLE);
}

/* ------------------------------------------------------------------ */
static void board_initServoPwm(void)
{
    /* GPIO0 复用为 EPWM1A（舵机1），GPIO1 复用为 EPWM1B（舵机2） */
    GPIO_setPinConfig(BOARD_SERVO_PINCFG);
    GPIO_setPadConfig(BOARD_SERVO_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(BOARD_SERVO2_PINCFG);
    GPIO_setPadConfig(BOARD_SERVO2_GPIO, GPIO_PIN_TYPE_STD);

    /* 配置期间停 TBCLK 同步 */
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    EPWM_setClockPrescaler(BOARD_SERVO_EPWM_BASE,
                           EPWM_CLOCK_DIVIDER_16,
                           EPWM_HSCLOCK_DIVIDER_2);   /* /32 → 3.125MHz */
    EPWM_setTimeBasePeriod(BOARD_SERVO_EPWM_BASE, SERVO_TB_PERIOD);
    EPWM_setTimeBaseCounterMode(BOARD_SERVO_EPWM_BASE, EPWM_COUNTER_MODE_UP);
    EPWM_setPhaseShift(BOARD_SERVO_EPWM_BASE, 0U);
    EPWM_setTimeBaseCounter(BOARD_SERVO_EPWM_BASE, 0U);
    EPWM_disablePhaseShiftLoad(BOARD_SERVO_EPWM_BASE);

    /* CMPA/CMPB 初值 = 居中脉宽（A=舵机1，B=舵机2） */
    EPWM_setCounterCompareValue(BOARD_SERVO_EPWM_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                SERVO_US_TO_CMP(SERVO_MID_US));
    EPWM_setCounterCompareValue(BOARD_SERVO_EPWM_BASE,
                                EPWM_COUNTER_COMPARE_B,
                                SERVO_US_TO_CMP(SERVO_MID_US));

    /* TBZERO 置高、CMPA/CMPB 置低 → 高电平脉宽 = CMPx/TBCLK */
    EPWM_setActionQualifierAction(BOARD_SERVO_EPWM_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(BOARD_SERVO_EPWM_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(BOARD_SERVO_EPWM_BASE, EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(BOARD_SERVO_EPWM_BASE, EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

/* ------------------------------------------------------------------ */
static void board_initCan(void)
{
    GPIO_setPinConfig(BOARD_CANTX_PINCFG);
    GPIO_setPinConfig(BOARD_CANRX_PINCFG);
    /* CANRX 建议异步限定，避免采样误差 */
    GPIO_setQualificationMode(BOARD_CANRX_GPIO, GPIO_QUAL_ASYNC);
    GPIO_setPadConfig(BOARD_CANRX_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(BOARD_CANTX_GPIO, GPIO_PIN_TYPE_STD);
    /* CAN 控制器初始化在 can_comm.c 完成 */
}

/* ------------------------------------------------------------------ */
void Board_init(void)
{
    /* 记录并清除复位原因 */
    s_resetReason = (uint16_t)SysCtl_getResetCause();
    SysCtl_clearResetCause(SYSCTL_CAUSE_POR | SYSCTL_CAUSE_XRS |
                           SYSCTL_CAUSE_WDRS | SYSCTL_CAUSE_NMIWDRS);

    /* EPWM 时钟分频 1:1，使 EPWMCLK = SYSCLK = 100MHz（舵机数学的前提） */

    /* 数字输出：闪光灯 / 蜂鸣器(有源) / 状态 LED，初始全灭 */
    board_initGpioOut(BOARD_LIGHT_LEFT_GPIO, false);
    board_initGpioOut(BOARD_LIGHT_RIGHT_GPIO, false);
#if BUZZER_ACTIVE
    board_initGpioOut(BOARD_BUZZER_GPIO, false);
#endif
    board_initGpioOut(BOARD_LED_RUN_GPIO, false);
    board_initGpioOut(BOARD_LED_FAULT_GPIO, false);

    /* 数字输入：一键热冗余按钮 */
    board_initGpioIn(BOARD_BUTTON_GPIO);

    /* 舵机 PWM */
    board_initServoPwm();

    /* CAN 引脚 */
    board_initCan();

    /* ASR-PRO 语音模块（LINA SCI 模式，GPIO22/23） */
    AsrPro_init();

    /* CPU Timer0（1 kHz），周期在此设定，启动在 Board_startControlTimer */
    CPUTimer_setEmulationMode(CPUTIMER0_BASE,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0U);
    CPUTimer_setPeriod(CPUTIMER0_BASE,
                       (DEVICE_SYSCLK_HZ / CTRL_LOOP_HZ) - 1U);  /* 100000-1 */
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);

#if WATCHDOG_ENABLE
    /* 片上看门狗只在完整控制 tick 执行完毕后喂狗；ISR 不喂狗，
     * 因此控制管线卡死会触发硬复位。 */
    /* INTOSC1≈10MHz, predivide 32, prescale 64, 8-bit counter:
     * 256*32*64/10MHz ≈ 52.4ms. */
    SysCtl_setWatchdogPredivider(SYSCTL_WD_PREDIV_32);
    SysCtl_setWatchdogPrescaler(SYSCTL_WD_PRESCALE_64);
    SysCtl_setWatchdogMode(SYSCTL_WD_MODE_RESET);
    SysCtl_serviceWatchdog();
    SysCtl_enableWatchdog();
#endif
}

/* ------------------------------------------------------------------ */
void Board_startControlTimer(void)
{
    CPUTimer_setEmulationMode(CPUTIMER0_BASE, CPUTIMER_EMULATIONMODE_RUNFREE);
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    CPUTimer_startTimer(CPUTIMER0_BASE);
}

void Board_serviceWatchdog(void)
{
#if WATCHDOG_ENABLE
    SysCtl_serviceWatchdog();
#endif
}

uint16_t Board_getResetReason(void)
{
    return s_resetReason;
}

bool Board_wasWatchdogReset(void)
{
    const uint16_t watchdog_causes =
        (uint16_t)(SYSCTL_CAUSE_WDRS | SYSCTL_CAUSE_NMIWDRS);
    const uint16_t external_causes =
        (uint16_t)(SYSCTL_CAUSE_POR | SYSCTL_CAUSE_XRS);

    /* XDS/UniFlash system reset reports XRS and WDRS together (0x0006).
     * That is an intentional external reset, not evidence that the runtime
     * watchdog expired.  A genuine watchdog reset has WDRS/NMIWDRS without
     * POR/XRS and must remain fatal-latched. */
    return ((s_resetReason & watchdog_causes) != 0U) &&
           ((s_resetReason & external_causes) == 0U);
}

/* ------------------------------------------------------------------ */
void Board_setServoMicros(uint16_t micros)
{
    if (micros < SERVO_MIN_US) { micros = SERVO_MIN_US; }
    if (micros > SERVO_MAX_US) { micros = SERVO_MAX_US; }
    EPWM_setCounterCompareValue(BOARD_SERVO_EPWM_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                SERVO_US_TO_CMP(micros));
}

void Board_setServo2Micros(uint16_t micros)
{
    if (micros < SERVO_MIN_US) { micros = SERVO_MIN_US; }
    if (micros > SERVO_MAX_US) { micros = SERVO_MAX_US; }
    EPWM_setCounterCompareValue(BOARD_SERVO_EPWM_BASE,
                                EPWM_COUNTER_COMPARE_B,
                                SERVO_US_TO_CMP(micros));
}

void Board_writeGpio(uint16_t gpio, bool high)
{
    GPIO_writePin(gpio, high ? 1U : 0U);
}

bool Board_readGpio(uint16_t gpio)
{
    return (GPIO_readPin(gpio) != 0U);
}
