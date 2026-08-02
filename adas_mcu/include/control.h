/*
 * control.h — 实时控制映射（横向限幅+纵向 accel→油门/制动+平顺 ramp）
 *
 * 输入：当前系统态(system_state)、采信的控制源命令(cmd, 可能为空)、AEB 请求。
 * 输出：ControlOutput_t（最终转角/加速度/油门/制动/转向灯）。
 *
 * 本模块是"最后一道执行器保护层"：无状态决策交给 safety.c，这里只做
 *   1) NaN/范围钳制 2) 转角/加速度变化率(slew/jerk)限制 3) 工程量→执行量映射。
 * 所有相邻 tick 的输出都从上一帧 slew 逼近目标 → 源切换/接管天然无冲击(bumpless)。
 */
#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdint.h>
#include <stdbool.h>
#include "adas_types.h"

void Control_init(void);

/*
 * 执行一个控制 tick。
 *   system_state : safety.c 给出的 SYS_MODE_*（决定横/纵向是否放行、是否安全制动）
 *   cmd          : 采信源命令；cmd_valid=false 时表示当前无可信源
 *   out          : 输出最终执行量（内部保有跨 tick 记忆）
 */
void Control_step(uint16_t system_state, const SocCommand_t *cmd,
                  bool cmd_valid, ControlOutput_t *out);

#endif /* CONTROL_H_ */
