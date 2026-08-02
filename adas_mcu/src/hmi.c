/*
 * hmi.c — 按钮去抖与手势识别实现
 *
 * 去抖：连续 BUTTON_DEBOUNCE_MS 稳定电平才确认状态翻转。
 * 长按：按下保持超过 BUTTON_LONGPRESS_MS 立即触发一次"清故障"，并置抑制标志，
 *       避免松开时再误判成短按。
 * 短按：v3 重构后无语义（会话自驱，不再需要按钮授权 ARM）。
 */
#include "hmi.h"
#include "board.h"
#include "actuators.h"
#include "safety.h"
#include "adas_config.h"

/* 稳定态：true=按下(按钮模块 OUT 为高) */
static bool     s_pressed        = false;
static bool     s_raw_last       = false;
static uint32_t s_stable_ref_ms  = 0U;
static uint32_t s_press_start_ms = 0U;
static bool     s_long_fired     = false;
static HmiButtonStatus_t s_button_status = HMI_BUTTON_IDLE;
static uint32_t s_button_status_ms = 0U;

#define HMI_BUTTON_FEEDBACK_MS 2000U

void Hmi_init(void)
{
    s_pressed = false;
    s_raw_last = false;
    s_stable_ref_ms = 0U;
    s_press_start_ms = 0U;
    s_long_fired = false;
    s_button_status = HMI_BUTTON_IDLE;
    s_button_status_ms = 0U;
}

void Hmi_poll(uint32_t now_ms)
{
    /* 实板三针按钮模块为高有效：按下 OUT=1，松开 OUT=0。 */
    bool raw = Board_readGpio(BOARD_BUTTON_GPIO);

    /* 去抖：电平变化则重置计时；稳定超过窗口才接受 */
    if (raw != s_raw_last)
    {
        s_raw_last = raw;
        s_stable_ref_ms = now_ms;
        return;
    }
    if ((now_ms - s_stable_ref_ms) < BUTTON_DEBOUNCE_MS)
    {
        /* 尚未稳定；但长按判定要在"持续按下"期间进行，见下 */
    }

    /* 确认稳定态翻转 */
    if (raw != s_pressed && (now_ms - s_stable_ref_ms) >= BUTTON_DEBOUNCE_MS)
    {
        s_pressed = raw;
        if (s_pressed)
        {
            /* 刚按下：起始计时 */
            s_press_start_ms = now_ms;
            s_long_fired = false;
            s_button_status = HMI_BUTTON_PRESSED;
            s_button_status_ms = now_ms;
        }
        /* 短按松开：v3 后无语义（不再授权 ARM），保留为将来扩展占位 */
    }

    /* 持续按下期间检测长按 */
    if (s_pressed && !s_long_fired &&
        (now_ms - s_press_start_ms) >= BUTTON_LONGPRESS_MS)
    {
        Safety_requestClear();
        Actuators_chirp(now_ms);   /* 长按确认 */
        s_long_fired = true;
        s_button_status = HMI_BUTTON_CLEAR;
        s_button_status_ms = now_ms;
    }
}

HmiButtonStatus_t Hmi_buttonStatus(uint32_t now_ms)
{
    if (s_pressed) { return HMI_BUTTON_PRESSED; }
    if (s_button_status == HMI_BUTTON_CLEAR) { return s_button_status; }
    if (s_button_status != HMI_BUTTON_IDLE &&
        (now_ms - s_button_status_ms) >= HMI_BUTTON_FEEDBACK_MS)
    {
        s_button_status = HMI_BUTTON_IDLE;
    }
    return s_button_status;
}