/*
 * control.c — 控制映射实现
 *
 * 关键约束（对齐 ADAS0.0.2/SoC 控制栈）：
 *  - 横纵向对称：转角与加速度都过变化率限制器（镜像关系）。
 *  - bumpless：每帧从上一帧输出向目标 slew，切源/接管不产生冲击。
 *  - 安全优先：EMERGENCY/FAILSAFE 时纵向收敛到安全地板制动，横向按态处理
 *    （MRM 保持车道跟随，纯通信丢失则方向回中）。
 */
#include "control.h"
#include "adas_config.h"
#include "adas_can_protocol.h"

/* 跨 tick 记忆（当前实际下发值，slew 的起点） */
static float s_steer_deg   = 0.0f;   /* 当前最终转角 */
static float s_accel_ms2   = 0.0f;   /* 当前最终加速度 */
static bool  s_turn_left   = false;  /* 转向灯滞回状态 */
static bool  s_turn_right  = false;

/* 有限值+范围钳制（等价 command_gate 的 NaN 门 + 串口范围 clamp）：
 *  - NaN：所有比较均为假，用 v!=v 捕获，置 0；
 *  - ±Inf：+Inf>hi→hi、-Inf<lo→lo，clamp 天然收敛，无需 isinf。 */
static float sanitize(float v, float lo, float hi)
{
    if (!(v == v)) { return 0.0f; }   /* v!=v 捕获 NaN */
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

/* 把 cur 朝 target 以 max_step 逼近（对称变化率限制） */
static float slew(float cur, float target, float max_step)
{
    float d = target - cur;
    if (d >  max_step) { d =  max_step; }
    if (d < -max_step) { d = -max_step; }
    return cur + d;
}

/* 加速度(m/s^2) → 油门/制动(0..1)。正加速给油门，负加速给制动。 */
static void accel_to_pedals(float accel, float *throttle, float *brake)
{
    if (accel >= 0.0f)
    {
        float t = accel * ACCEL_TO_THROTTLE_GAIN;
        *throttle = sanitize(t, 0.0f, 1.0f);
        *brake = 0.0f;
    }
    else
    {
        float b = (-accel) * DECEL_TO_BRAKE_GAIN;
        *throttle = 0.0f;
        *brake = sanitize(b, 0.0f, 1.0f);
    }
}

/* 转向灯滞回：|转角| 越过 ON 门限点亮对应侧，回落到 OFF 门限熄灭 */
static void update_turn_signal(float steer_deg)
{
    float a = (steer_deg >= 0.0f) ? steer_deg : -steer_deg;
    bool left  = (steer_deg > 0.0f);   /* 约定：+ 为左打（与 SERVO_INVERT 无关，纯指示） */

    if (a >= TURN_SIGNAL_ON_DEG)
    {
        s_turn_left  = left;
        s_turn_right = !left;
    }
    else if (a <= TURN_SIGNAL_OFF_DEG)
    {
        s_turn_left  = false;
        s_turn_right = false;
    }
    /* 介于两门限之间：保持上一状态（滞回） */
}

void Control_init(void)
{
    s_steer_deg  = 0.0f;
    s_accel_ms2  = 0.0f;
    s_turn_left  = false;
    s_turn_right = false;
}

void Control_step(uint16_t system_state, const SocCommand_t *cmd,
                  bool cmd_valid, ControlOutput_t *out)
{
    const float dt = CTRL_DT_S;
    const float accel_step = ACCEL_SLEW_MS3 * dt;      /* jerk 限制 → 每 tick 加速度步长 */

    float steer_target;
    float accel_target;
    float steer_step;
    bool  lateral_ok;
    bool  hazard = false;
    bool  immediate = false;                            /* 跳过 jerk 限幅(急停) */

    /* -------- 1) 依系统态决定横/纵向目标 -------- */
    switch (system_state)
    {
        case SYS_MODE_EMERGENCY_BRAKE:
            /* AEB 全制动 / 急停：立即安全地板全力制动，方向保持(不主动转) */
            accel_target = -MAX_DECEL_MS2;
            steer_target = s_steer_deg;                 /* 冻结当前方向 */
            lateral_ok   = false;
            hazard = true;
#if ESTOP_IMMEDIATE
            immediate = true;
#endif
            break;

        case SYS_MODE_FAILSAFE:
            /* 通信全丢：受控减速到地板(走 jerk)，方向平滑回中 */
            accel_target = -FAILSAFE_DECEL_MS2;
            steer_target = 0.0f;
            lateral_ok   = true;                        /* 允许方向以回中速率动作 */
            steer_step   = STEER_RETURN_DPS * dt;
            hazard = true;
            /* 纵向 slew，横向单独用回中速率 */
            s_accel_ms2 = slew(s_accel_ms2, accel_target, accel_step);
            s_steer_deg = slew(s_steer_deg, steer_target, steer_step);
            goto emit;

        case SYS_MODE_FAULT_LOCK:
            /* 看门狗/自检故障后的锁定态可能发生在车辆运动中。锁定态必须输出
             * 确定性安全制动，不能与冷启动 STANDBY 一样零制动滑行。 */
            accel_target = -MAX_DECEL_MS2;
            steer_target = s_steer_deg;
            lateral_ok = false;
            hazard = true;
            immediate = true;
            break;

        case SYS_MODE_MRM:
            /* 最小风险策略：保持车道跟随(用源转角)，纵向受控减速停车 */
            accel_target = -MRM_DECEL_MS2;              /* gentler than FAILSAFE */
            steer_target = (cmd_valid && cmd->lateral_enable)
                           ? sanitize(cmd->target_steer_deg, -MAX_STEER_DEG, MAX_STEER_DEG)
                           : s_steer_deg;
            lateral_ok = true;
            hazard = true;
            break;

        case SYS_MODE_ACTIVE:
        case SYS_MODE_DEGRADED:
            if (cmd_valid)
            {
                /* 横向目标 */
                if (cmd->lateral_enable)
                {
                    steer_target = sanitize(cmd->target_steer_deg,
                                            -MAX_STEER_DEG, MAX_STEER_DEG);
                    lateral_ok = true;
                }
                else
                {
                    steer_target = 0.0f;    /* 横向未使能：回中 */
                    lateral_ok = true;
                }

                /* 纵向目标：默认取目标加速度；制动请求/AEB 取更强的减速（更安全者胜） */
                if (cmd->longitudinal_enable)
                {
                    accel_target = sanitize(cmd->target_accel_ms2,
                                            -MAX_DECEL_MS2, MAX_ACCEL_MS2);
                }
                else
                {
                    accel_target = 0.0f;
                }
                if (cmd->brake_request_pct > 0U)
                {
                    float brk_decel = -((float)cmd->brake_request_pct * 0.01f)
                                      * MAX_DECEL_MS2;
                    if (brk_decel < accel_target) { accel_target = brk_decel; }
                }
                if (cmd->aeb_risk >= AEB_RISK_PARTIAL && cmd->obstacle_valid)
                {
                    float aeb_decel = -cmd->aeb_required_decel;
                    if (aeb_decel < accel_target) { accel_target = aeb_decel; }
                    if (cmd->aeb_risk >= AEB_RISK_FULL) { immediate = true; }
                }
            }
            else
            {
                /* 态为 ACTIVE/DEGRADED 但本 tick 无有效命令：保持并轻收油 */
                steer_target = s_steer_deg;
                accel_target = 0.0f;
                lateral_ok = true;
            }
            hazard = (system_state == SYS_MODE_DEGRADED);
            break;

        case SYS_MODE_INIT:
        case SYS_MODE_STANDBY:
        default:
            /* 未激活 / 锁定：不驱动，方向回中，纵向缓收到 0（不主动制动，
             * 除非上层已把态切到 FAILSAFE/EMERGENCY） */
            steer_target = 0.0f;
            accel_target = 0.0f;
            lateral_ok = true;
            break;
    }

    /* -------- 2) 变化率限制（对称 slew / jerk） -------- */
    steer_step = MCU_MAX_STEER_RATE_DPS * dt;
    if (cmd_valid && lateral_ok && cmd->target_steer_rate_dps > 0.0f)
    {
        float src_step = cmd->target_steer_rate_dps * dt;   /* 源自带速率上限，取更严者 */
        if (src_step < steer_step) { steer_step = src_step; }
    }
    if (lateral_ok)
    {
        s_steer_deg = slew(s_steer_deg, steer_target, steer_step);
    }

    if (immediate)
    {
        s_accel_ms2 = accel_target;                 /* 急停：不经 jerk 限幅 */
    }
    else
    {
        s_accel_ms2 = slew(s_accel_ms2, accel_target, accel_step);
    }

emit:
    /* -------- 3) 组装输出 + 映射 + 转向灯 -------- */
    s_steer_deg = sanitize(s_steer_deg, -MAX_STEER_DEG, MAX_STEER_DEG);
    s_accel_ms2 = sanitize(s_accel_ms2, -MAX_DECEL_MS2, MAX_ACCEL_MS2);

    out->final_steer_deg = s_steer_deg;
    out->final_accel_ms2 = s_accel_ms2;
    accel_to_pedals(s_accel_ms2, &out->final_throttle, &out->final_brake);
    out->drive_dir = (cmd_valid ? cmd->drive_dir : DIR_NEUTRAL);

    update_turn_signal(s_steer_deg);
    out->hazard = hazard;
    /* 危险报警时双闪优先；否则用转向灯滞回状态 */
    out->turn_left  = hazard ? true : s_turn_left;
    out->turn_right = hazard ? true : s_turn_right;
}
