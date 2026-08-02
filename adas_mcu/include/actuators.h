/*
 * actuators.h — 物理外设驱动：舵机 / 左右闪光灯 / 蜂鸣器 / 状态 LED
 *
 * 把 ControlOutput_t + McuStatus_t 翻译为引脚动作。所有"闪烁/鸣叫节奏"
 * 都是本模块内的时间状态机（以 1ms tick 驱动），上层只给"要不要亮/要不要叫"。
 */
#ifndef ACTUATORS_H_
#define ACTUATORS_H_

#include <stdint.h>
#include <stdbool.h>
#include "adas_types.h"

void Actuators_init(void);

/* 每 1ms：驱动舵机脉宽、转向灯闪烁、蜂鸣器节奏、状态 LED。
 * 返回本 tick 蜂鸣器是否发声（用于心跳 SF_BUZZER_ON 上报）。 */
bool Actuators_update(uint32_t now_ms, const ControlOutput_t *out,
                      const McuStatus_t *st);

/* HMI 触发一次性短鸣提示（如手动切源确认） */
void Actuators_chirp(uint32_t now_ms);

#endif /* ACTUATORS_H_ */
