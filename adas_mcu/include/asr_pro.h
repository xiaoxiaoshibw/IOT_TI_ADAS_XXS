/*
 * asr_pro.h — ASR-PRO 语音识别模块 UART 驱动（LINA SCI 兼容模式）
 *
 * 硬件连接（见 board.h / 引脚接线清单.md）：
 *   MCU GPIO22 (LINA_TX) → ASR-PRO RX
 *   MCU GPIO23 (LINA_RX) → ASR-PRO TX
 *   波特率 115200, 8N1
 *
 * 协议帧格式（ASR-PRO → MCU）：
 *   帧头 0x0C 0x0D + 命令字节 [+ 可选附加字节]
 *   例：停车 = 0x0C 0x0D 0x01
 *       车速查询 = 0x0C 0x0D 0x0E 0xED
 *
 * 帧边界识别：协议规定帧尾没有显式结束符，靠"下一帧的 0x0C 头"作为当前
 * 帧的隐式结束。状态机在 ASR_WAIT_ARG 状态下遇到 0x0C 必须立即转入
 * ASR_WAIT_HDR1；不得吞掉 0x0C 当作附加字节。该约束已用 host 单元测试
 * tests/test_asr_parser_host.c 锁住，回归即失败。
 *
 * 调用约定：
 *   AsrPro_init()         在 Board_init() 中调用一次
 *   AsrPro_service()      在空闲背景任务中轮询（与 Dgus_service 同路径）
 *   AsrPro_pollCommand()  在主循环中取走最新命令
 */
#ifndef ASR_PRO_H_
#define ASR_PRO_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>  /* uint8_t on C2000 toolchain */

/* ASR-PRO 命令码（与 ASR 固件中 Serial1.write 对应） */
#define ASR_CMD_STOP            0x01U   /* 停车 */
#define ASR_CMD_QUERY_STATUS    0x0AU   /* 查询系统状态 */
#define ASR_CMD_QUERY_SPEED     0x0EU   /* 查询车速（后跟校验/附加字节） */
#define ASR_CMD_FAULT_INJECT    0x0FU   /* 故障注入 */

/* 上电初始化 LINA(SCI 模式) + GPIO22/23 + 收发状态机复位 */
void AsrPro_init(void);

/* 非阻塞轮询：消费 RX 字节、解析帧。放在空闲背景任务中调用。 */
void AsrPro_service(void);

/* 取走最近一次语音命令（先到先得，取走前不被新帧覆盖）。
 * cmd  写入命令字节（如 ASR_CMD_STOP）
 * arg  写入附加字节（无附加字节时为 0）
 * 返回 true 表示有新命令；无待处理命令返回 false。 */
bool AsrPro_pollCommand(uint16_t *cmd, uint16_t *arg);

/* 发送字节到 ASR-PRO（MCU→模块方向，如需下发控制指令）。阻塞直到发送完成。
 * C28x char 为 16 位，data 仅低 8 位有效。 */
void AsrPro_sendByte(uint16_t data);

/* 是否收到过合法帧（用于状态展示）。 */
bool AsrPro_isPresent(void);

/* 调试 accessor：累计字节/帧头/帧数、最近命令码、解析状态机编号。
 * OLED2 DBG 页用这些字段定位 ASR-PRO 无响应的具体原因。 */
uint16_t AsrPro_byteCount(void);
uint16_t AsrPro_hdr0Count(void);
uint16_t AsrPro_frameCount(void);
uint16_t AsrPro_lastCmd(void);
uint16_t AsrPro_rxState(void);

/* ---------- 仅 host 测试入口（tests/test_asr_parser_host.c） ---------- */
void AsrPro_resetForTest(void);
/* 直接灌一个字节到帧解析状态机。仅 host 测试使用；其它代码应通过
 * AsrPro_service 走真实 RX FIFO。
 * 类型用 uint16_t 是 C28x 原生（硬件外设与 driverlib 的 uint8_t 实际为
 * uint16_t），避免 stdint.h 在 include 顺序敏感下不可见。 */
void asr_rx_feed_for_test(uint16_t b);
/* 取走解析后待处理的命令。仅 host 测试可见。生产路径用 AsrPro_pollCommand。 */
bool AsrPro_pollCommand(uint16_t *cmd, uint16_t *arg);

#endif /* ASR_PRO_H_ */
