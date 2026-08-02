#include "self_test.h"
#include "adas_config.h"
#include "adas_can_protocol.h"
#include "crc8.h"

bool SelfTest_run(uint16_t *failure_mask)
{
    static const uint16_t crc_vector[9] =
        { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    uint16_t failures = SELF_TEST_OK;

    if (CTRL_LOOP_HZ == 0U || CTRL_TICK_US == 0U ||
        MAX_DECEL_MS2 <= 0.0f || MAX_ACCEL_MS2 <= 0.0f ||
        MAX_STEER_DEG <= 0.0f || SERVO_MIN_US >= SERVO_MID_US ||
        SERVO_MID_US >= SERVO_MAX_US || CTRL_TIMEOUT_MS >= HB_TIMEOUT_MS ||
        STATUS_TIMEOUT_MS < HB_TIMEOUT_MS)
    {
        failures |= SELF_TEST_CONFIG;
    }
    if (crc8_compute(crc_vector, 9U) != 0xA2U)
    {
        failures |= SELF_TEST_CRC;
    }
    if (ADAS_PROTOCOL_VERSION == 0U || ADAS_PROTOCOL_VERSION > HB_VERSION_MASK ||
        CAN_FRAME_DLC != 8U || CANID_PRIMARY_BASE == CANID_BACKUP_BASE)
    {
        failures |= SELF_TEST_PROTOCOL;
    }
    if (failure_mask != 0) { *failure_mask = failures; }
    return (failures == SELF_TEST_OK);
}
