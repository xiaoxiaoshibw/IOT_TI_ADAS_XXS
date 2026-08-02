/*
 * 第二块 SSD1306 OLED（128x64，GPIO 开漏软件 I2C）车辆数据页。
 * IO43/IO26 自动探测 SCL/SDA 线序和 7 位地址；与第一块 OLED 总线独立。
 */
#ifndef OLED2_H_
#define OLED2_H_

#include <stdbool.h>
#include <stdint.h>
#include "adas_types.h"

void Oled2_init(void);
void Oled2_update(uint32_t now_ms, const ControlOutput_t *out,
                  const SocCommand_t *primary, const McuStatus_t *status,
                  bool can_healthy);
bool Oled2_isPresent(void);

/* 推送最近一次 ASR-PRO 语音命令到 OLED2 显示：3 秒内翻页覆盖显示
 * "ASR:STOP/STATUS/SPEED/FAULT"，过期后回到正常车辆数据翻页。 */
void Oled2_setAsrCommand(uint16_t cmd, uint32_t now_ms);

#endif /* OLED2_H_ */
