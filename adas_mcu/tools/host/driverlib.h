#ifndef ADAS_MCU_SIL_DRIVERLIB_H_
#define ADAS_MCU_SIL_DRIVERLIB_H_

#include <stdbool.h>
#include <stdint.h>

#define CAN_MSG_FRAME_STD 0U
#define CAN_MSG_OBJ_TYPE_RX 0U
#define CAN_MSG_OBJ_TYPE_TX 1U
#define CAN_MSG_OBJ_NO_FLAGS 0U
#define CAN_STATUS_BUS_OFF (1UL << 7)

void CAN_setupMessageObject(uint32_t base, uint16_t object,
                            uint32_t can_id, uint16_t frame_type,
                            uint16_t object_type, uint32_t msg_id_mask,
                            uint32_t flags, uint16_t dlc);
void CAN_initModule(uint32_t base);
void CAN_setBitRate(uint32_t base, uint32_t sysclk_hz,
                    uint32_t bitrate, uint16_t bit_time);
void CAN_enableRetry(uint32_t base);
void CAN_startModule(uint32_t base);
uint32_t CAN_getStatus(uint32_t base);
bool CAN_readMessage(uint32_t base, uint16_t object, uint16_t* data);
void CAN_sendMessage(uint32_t base, uint16_t object, uint16_t dlc,
                     const uint16_t* data);

#endif  // ADAS_MCU_SIL_DRIVERLIB_H_
