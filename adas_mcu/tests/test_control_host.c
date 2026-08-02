#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "control.h"
#include "adas_can_protocol.h"
#include "adas_config.h"

static void test_emergency_is_immediate_and_exclusive(void)
{
    ControlOutput_t out;
    memset(&out, 0, sizeof(out));
    Control_init();
    Control_step(SYS_MODE_EMERGENCY_BRAKE, 0, false, &out);
    assert(out.final_accel_ms2 == -MAX_DECEL_MS2);
    assert(out.final_throttle == 0.0f);
    assert(out.final_brake == 1.0f);
    assert(out.hazard);
}

static void test_brake_request_overrides_acceleration(void)
{
    SocCommand_t cmd;
    ControlOutput_t out;
    memset(&cmd, 0, sizeof(cmd));
    memset(&out, 0, sizeof(out));
    cmd.longitudinal_enable = true;
    cmd.target_accel_ms2 = MAX_ACCEL_MS2;
    cmd.brake_request_pct = 100U;
    cmd.drive_dir = DIR_DRIVE;
    Control_init();
    Control_step(SYS_MODE_ACTIVE, &cmd, true, &out);
    assert(out.final_accel_ms2 < 0.0f);
    assert(out.final_throttle == 0.0f);
    assert(out.final_brake > 0.0f);
}

static void test_standby_never_commands_drive(void)
{
    SocCommand_t cmd;
    ControlOutput_t out;
    memset(&cmd, 0, sizeof(cmd));
    memset(&out, 0, sizeof(out));
    cmd.drive_dir = DIR_DRIVE;
    Control_init();
    Control_step(SYS_MODE_STANDBY, &cmd, false, &out);
    assert(out.final_throttle == 0.0f);
    assert(out.final_brake == 0.0f);
    assert(out.drive_dir == DIR_NEUTRAL);
}

static void test_fault_lock_commands_immediate_safe_brake(void)
{
    ControlOutput_t out;
    memset(&out, 0, sizeof(out));
    Control_init();
    Control_step(SYS_MODE_FAULT_LOCK, 0, false, &out);
    assert(out.final_accel_ms2 == -MAX_DECEL_MS2);
    assert(out.final_throttle == 0.0f);
    assert(out.final_brake == 1.0f);
    assert(out.hazard);
}

static void test_mrm_uses_configured_deceleration(void)
{
    ControlOutput_t out;
    uint16_t i;
    memset(&out, 0, sizeof(out));
    Control_init();
    for (i = 0U; i < 1000U; i++)
    {
        Control_step(SYS_MODE_MRM, 0, false, &out);
    }
    assert(out.final_accel_ms2 == -MRM_DECEL_MS2);
}

int main(void)
{
    test_emergency_is_immediate_and_exclusive();
    test_brake_request_overrides_acceleration();
    test_standby_never_commands_drive();
    test_fault_lock_commands_immediate_safe_brake();
    test_mrm_uses_configured_deceleration();
    puts("control host tests: PASS");
    return 0;
}
