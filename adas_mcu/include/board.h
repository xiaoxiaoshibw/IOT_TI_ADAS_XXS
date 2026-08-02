/*
 * board.h — LAUNCHXL-F280025C 板级抽象：引脚映射、外设初始化、时基
 *
 * ★ 引脚分配已于 2026-07 实板最终确认：BoosterPack 排针丝印 "IO.n" 即 GPIOn。
 *   CAN 用板载收发器 + J14；状态灯用板载 LED4/LED5；舵机/转向灯/蜂鸣器/按钮
 *   集中从 J2 引出（pin35~pin40，共地 J2 pin22）。
 *   串口屏走 SCIA（IO.28/29）、ASR-PRO 走 LINA 的 SCI 兼容模式（IO.22/23）。
 *   OLED2 软件 I2C 走 IO.43/IO.26（详见 oled2.c，地址/线序自适应）。
 *   换封装/走线时只改本文件的宏。
 *
 * 依赖 C2000Ware driverlib（driverlib.h / device.h）。建议从
 * C2000Ware 的 "empty_driverlib_project"(F280025C) 派生本工程。
 */
#ifndef BOARD_H_
#define BOARD_H_

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/* CAN（CANA：GPIO32/33）                                              */
/*   LAUNCHXL-F280025C 板载 CAN 收发器 → 接口 J14(GND / LO=CANL / HI=CANH)， */
/*   须把 S4(CAN ROUTE) 拨到 XCVR(板载收发器)，即可直接上 CAN 差分总线。 */
/*   若改用外部 TCAN332/SN65HVD230，把 S4 拨到 BP，从 BoosterPack 取 TX/RX。*/
/* ------------------------------------------------------------------ */
#define BOARD_CANTX_GPIO        32U           /* GPIO32 → CANA_TX */
#define BOARD_CANRX_GPIO        33U           /* GPIO33 → CANA_RX */
#define BOARD_CANTX_PINCFG      GPIO_32_CANA_TX
#define BOARD_CANRX_PINCFG      GPIO_33_CANA_RX

/* ------------------------------------------------------------------ */
/* 舵机 PWM：共用 EPWM1 时基，50 Hz                                     */
/*   舵机1 = EPWM1A：GPIO0 = 丝印 IO.0，J2 pin40                        */
/*   舵机2 = EPWM1B：GPIO1 = 丝印 IO.1，J2 pin39                        */
/*   舵机动力电源走外部 5V(BEC)，信号线接 IO.0/IO.1，务必与板卡共地。    */
/* ------------------------------------------------------------------ */
#define BOARD_SERVO_GPIO        0U            /* 舵机1：IO.0 (J2 pin40) */
#define BOARD_SERVO_PINCFG      GPIO_0_EPWM1_A
#define BOARD_SERVO2_GPIO       1U            /* 舵机2：IO.1 (J2 pin39) */
#define BOARD_SERVO2_PINCFG     GPIO_1_EPWM1_B
#define BOARD_SERVO_EPWM_BASE   EPWM1_BASE

/* ------------------------------------------------------------------ */
/* 转向闪光灯（GPIO 经三极管/MOS 驱动，勿直接大电流拉；高电平点亮）      */
/*   左 = 丝印 IO.2 (J2 pin38)、右 = 丝印 IO.3 (J2 pin37)               */
/* ------------------------------------------------------------------ */
#define BOARD_LIGHT_LEFT_GPIO   2U
#define BOARD_LIGHT_RIGHT_GPIO  3U

/* ------------------------------------------------------------------ */
/* 蜂鸣器                                                              */
/*   有源(默认 BUZZER_ACTIVE=1)：GPIO 通断 → 丝印 IO.4，J2 pin36        */
/*   无源(BUZZER_ACTIVE=0)：EPWM4A 载频 → 丝印 IO.6，Site2 J6           */
/* ------------------------------------------------------------------ */
#define BOARD_BUZZER_GPIO       4U            /* 有源蜂鸣器：IO.4 (J2 pin36) */
#define BOARD_BUZZER_EPWM_BASE  EPWM4_BASE
#define BOARD_BUZZER_PINCFG     GPIO_6_EPWM4_A
#define BOARD_BUZZER_PWM_GPIO   6U            /* 无源蜂鸣器：IO.6  (Site2 J6) */

/* ------------------------------------------------------------------ */
/* 一键主备热冗余按钮（三针模块输入，3.3V 供电，按下输出高）            */
/*   丝印 IO.15，J2 pin35                                               */
/* ------------------------------------------------------------------ */
#define BOARD_BUTTON_GPIO       15U

/* ------------------------------------------------------------------ */
/* OLED2 显示（开漏软件 I2C）：SCL = GPIO43 = 丝印 IO.43，               */
/*                       SDA = GPIO26 = 丝印 IO.26                      */
/*   GPIO 软件模拟 I2C，地址 0x3C/0x3D 与线序自动探测；详见 oled2.c。     */
/* ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */
/* 串口屏（4Pin：5V/RX/TX/GND，走板上唯一的硬件 SCI = SCIA）             */
/*   TX = GPIO29 = 丝印 IO.29，RX = GPIO28 = 丝印 IO.28                 */
/*   接线：屏 RX ← MCU TX(IO.29)，屏 TX → MCU RX(IO.28)，5V/GND 另接。   */
/* ------------------------------------------------------------------ */
#define BOARD_SCREEN_TX_GPIO    29U
#define BOARD_SCREEN_RX_GPIO    28U
#define BOARD_SCREEN_TX_PINCFG  GPIO_29_SCIA_TX
#define BOARD_SCREEN_RX_PINCFG  GPIO_28_SCIA_RX

/* ------------------------------------------------------------------ */
/* ASR-PRO 语音识别模块（LINA SCI 兼容模式，驱动见 asr_pro.c）         */
/*   F280025C 只有一个硬件 SCI(SCIA，已分给串口屏)，此路借用 LINA 外设   */
/*   的 SCI 兼容异步模式(8N1)充当第二个 UART，不使用 LIN 报文/主从时序。 */
/*   TX = GPIO22 = 丝印 IO.22，RX = GPIO23 = 丝印 IO.23                 */
/* ------------------------------------------------------------------ */
#define BOARD_ASRPRO_TX_GPIO    22U
#define BOARD_ASRPRO_RX_GPIO    23U
#define BOARD_ASRPRO_TX_PINCFG  GPIO_22_LINA_TX
#define BOARD_ASRPRO_RX_PINCFG  GPIO_23_LINA_RX

/* ------------------------------------------------------------------ */
/* 板载状态 LED（F280025C LaunchPad 板载 LED4/LED5，无需外部接线）       */
/* ------------------------------------------------------------------ */
#define BOARD_LED_RUN_GPIO      31U           /* 板载 LED4：运行心跳灯 */
#define BOARD_LED_FAULT_GPIO    34U           /* 板载 LED5：故障/降级灯 */

/* ------------------------------------------------------------------ */
/* 初始化接口                                                          */
/* ------------------------------------------------------------------ */
void Board_init(void);                        /* 时钟/GPIO/外设总初始化 */
void Board_startControlTimer(void);           /* 启动 1 kHz CPU Timer0 */
void Board_serviceWatchdog(void);             /* 喂片上看门狗(控制环跑完一圈时调用) */
uint16_t Board_getResetReason(void);          /* 读取上电/复位原因 */
bool Board_wasWatchdogReset(void);

/* 舵机脉宽(us)设定，[SERVO_MIN_US, SERVO_MAX_US] 会被内部钳制 */
void Board_setServoMicros(uint16_t micros);   /* 舵机1 = EPWM1A */
void Board_setServo2Micros(uint16_t micros);  /* 舵机2 = EPWM1B */

/* GPIO 快捷读写 */
void Board_writeGpio(uint16_t gpio, bool high);
bool Board_readGpio(uint16_t gpio);

#endif /* BOARD_H_ */
