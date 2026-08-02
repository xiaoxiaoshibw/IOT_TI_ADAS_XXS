/*
 * hmi.h — 面板按钮手势识别（去抖 + 短按/长按）
 *
 * v3 重构（2026-07）：短按不再授权 ARM（会话已自驱）。仅保留：
 *   - 短按：无语义（保留手势识别以备将来扩展，REDUNDANCY_ENABLE 时可挂切源）
 *   - 长按 ≥ BUTTON_LONGPRESS_MS：清故障 + 请求 CAN 重新入网（本地诊断/恢复辅助）
 * 实板三针按钮模块高有效（3.3V 供电，按下 OUT=高）。
 */
#ifndef HMI_H_
#define HMI_H_

#include <stdbool.h>
#include <stdint.h>

void Hmi_init(void);

/* 每 1ms 轮询按钮；长按时回调 Safety_requestClear。 */
void Hmi_poll(uint32_t now_ms);

typedef enum
{
    HMI_BUTTON_IDLE = 0,
    HMI_BUTTON_PRESSED,
    HMI_BUTTON_CLEAR
} HmiButtonStatus_t;

/* 按钮反馈：仅显示 PRESSED / CLEAR 瞬态。 */
HmiButtonStatus_t Hmi_buttonStatus(uint32_t now_ms);

#endif /* HMI_H_ */