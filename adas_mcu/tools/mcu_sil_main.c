#include "adas_can_protocol.h"
#include "adas_config.h"
#include "adas_types.h"
#include "can_comm.h"
#include "control.h"
#include "hil_session.h"
#include "safety.h"
#include "self_test.h"

#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static volatile sig_atomic_t g_stop = 0;

static void stop_handler(int signal_number)
{
    (void)signal_number;
    g_stop = 1;
}

static void sleep_one_ms(void)
{
    struct timespec delay = {0, 1000000L};
    nanosleep(&delay, NULL);
}

static bool primary_control_fresh(uint32_t now_ms)
{
    const LinkMonitor_t *primary = CanComm_getLink(SRC_IDX_PRIMARY);
    return primary != NULL && primary->protocol_ok && primary->lat_valid &&
           primary->lon_valid && primary->status_valid && primary->seq_seen &&
           primary->lat_seen && primary->lon_seen && primary->status_seen &&
           (now_ms - primary->last_hb_ms <= HB_TIMEOUT_MS) &&
           (now_ms - primary->last_lat_ms <= CTRL_TIMEOUT_MS) &&
           (now_ms - primary->last_lon_ms <= CTRL_TIMEOUT_MS) &&
           (now_ms - primary->last_status_ms <= STATUS_TIMEOUT_MS);
}

static long duration_ms_from_args(int argc, char **argv)
{
    for (int i = 1; i + 1 < argc; i++) {
        if (strcmp(argv[i], "--duration") == 0) {
            char *end = NULL;
            const long seconds = strtol(argv[i + 1], &end, 10);
            if (end == argv[i + 1] || seconds < 0L) return 0L;
            return seconds * 1000L;
        }
    }
    return -1L;
}

int main(int argc, char **argv)
{
    McuStatus_t status = {0};
    ControlOutput_t output = {0};
    SafetyDecision_t decision = {0};
    HilSession_t session;
    uint16_t self_test_mask = SELF_TEST_OK;
    bool self_test_ok;
    uint32_t now_ms = 0U;
    uint16_t mcu_sequence = 0U;
    uint16_t alive = 0U;
    uint32_t last_control_tx = 0U;
    uint32_t last_heartbeat_tx = 0U;
    uint32_t last_diag_tx = 0U;
    uint16_t previous_state = SYS_MODE_INIT;
    long duration_ms = duration_ms_from_args(argc, argv);

    signal(SIGINT, stop_handler);
    signal(SIGTERM, stop_handler);
    CanComm_init();
    Control_init();
    self_test_ok = SelfTest_run(&self_test_mask);
    Safety_init(0U, false, self_test_ok, self_test_mask);
    HilSession_init(&session);

    fprintf(stdout, "MCU SIL host started on %s\n",
            getenv("ADAS_MCU_CAN_INTERFACE") != NULL
                ? getenv("ADAS_MCU_CAN_INTERFACE") : "vcan0");
    fflush(stdout);

    while (!g_stop && (duration_ms < 0L || (long)now_ms < duration_ms)) {
        uint16_t crc_errors = 0U;
        const SocCommand_t *command;

        CanComm_service(now_ms);
        (void)CanComm_pollRx(now_ms, &crc_errors);
        {
            uint16_t inject_command;
            uint16_t inject_parameter;
            if (CanComm_getInjectCommand(&inject_command, &inject_parameter)) {
                Safety_applyInject(inject_command, inject_parameter);
                CanComm_sendInjectResponse(inject_command, inject_parameter, &status);
            }
        }

        Safety_setCanState(CanComm_isBusHealthy(), CanComm_recoveryExhausted());
        Safety_evaluateLinks(now_ms, crc_errors);
        Safety_step(now_ms, false, &status, &decision);

        {
            uint16_t request = HIL_REQ_NONE;
            uint16_t sequence = 0U;
            uint32_t session_id = 0U;
            HilSessionInputs_t inputs;
            inputs.control_fresh = primary_control_fresh(now_ms);
            inputs.safe_to_arm = !status.estop &&
                status.system_state != SYS_MODE_FAULT_LOCK &&
                status.fault_level < FAULT_LEVEL_FATAL;
            inputs.fatal_fault = status.system_state == SYS_MODE_FAULT_LOCK ||
                status.fault_level >= FAULT_LEVEL_FATAL;
            if (CanComm_getSessionRequest(&request, &session_id, &sequence)) {
                (void)HilSession_handle(&session, request, session_id, sequence, &inputs);
            } else {
                HilSession_tick(&session, &inputs);
            }
            {
                SafetySessionContext_t context;
                context.control_authorized = HilSession_controlEnabled(&session);
                context.ever_active = session.ever_active;
                context.normal_shutdown = false;
                context.session_epoch = session.session_id;
                Safety_setSessionContext(&context);
            }
        }

        command = (decision.cmd_valid && HilSession_controlEnabled(&session))
                    ? CanComm_getCommand(decision.active_idx) : NULL;
        if (!HilSession_controlEnabled(&session) &&
            (decision.system_state == SYS_MODE_ACTIVE ||
             decision.system_state == SYS_MODE_DEGRADED)) {
            decision.system_state = HilSession_requiresFailsafe(&session)
                                      ? SYS_MODE_FAILSAFE : SYS_MODE_STANDBY;
            status.system_state = decision.system_state;
            if (HilSession_requiresFailsafe(&session)) {
                status.active_source = SRC_WATCHDOG;
                status.degraded = true;
                status.fault_level = FAULT_LEVEL_SEVERE;
            } else {
                status.active_source = SRC_NONE;
            }
        }
        Control_step(decision.system_state, command,
                     decision.cmd_valid && HilSession_controlEnabled(&session), &output);

        if ((now_ms - last_control_tx) >= TX_CONTROL_PERIOD_MS) {
            last_control_tx = now_ms;
            CanComm_sendControl(&output, &status, mcu_sequence++);
        }
        if ((now_ms - last_heartbeat_tx) >= TX_HEARTBEAT_PERIOD_MS) {
            last_heartbeat_tx = now_ms;
            CanComm_sendHeartbeat(&status, mcu_sequence, alive++, false);
            CanComm_sendSessionStatus(session.ack, session.session_id,
                                       session.last_sequence);
        }
        if ((now_ms - last_diag_tx) >= TX_DIAG_PERIOD_MS) {
            last_diag_tx = now_ms;
            CanComm_sendDiag(&status);
            CanComm_sendE2eDiag(&status);
            CanComm_sendLinkStats();
        }

        if (status.system_state != previous_state) {
            fprintf(stdout, "MCU SIL state %u -> %u session=%u\n",
                    previous_state, status.system_state, session.ack);
            fflush(stdout);
            previous_state = status.system_state;
        }
        sleep_one_ms();
        now_ms++;
    }

    return 0;
}
