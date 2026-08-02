/*
 * crc8.c — CRC-8 (poly 0x31, MSB-first, init 0x00) 实现
 *
 * 采用逐位运算(无查表)，代码小、无需 256*2B RAM/Flash 表，1 kHz 下开销可忽略。
 * 若需极致速度可改查表；两种实现结果一致。
 */
#include "crc8.h"

uint16_t crc8_compute(const uint16_t *data, uint16_t len)
{
    uint16_t crc = 0x00U;
    uint16_t i;
    uint16_t b;

    for (i = 0U; i < len; i++)
    {
        crc ^= (data[i] & 0x00FFU);
        for (b = 0U; b < 8U; b++)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint16_t)(((crc << 1) ^ 0x31U) & 0x00FFU);
            }
            else
            {
                crc = (uint16_t)((crc << 1) & 0x00FFU);
            }
        }
    }
    return (crc & 0x00FFU);
}

uint16_t crc8_compute_frame(uint32_t can_id, const uint16_t *data, uint16_t len)
{
    uint16_t prefixed[10];
    uint16_t i;
    prefixed[0] = (uint16_t)(can_id & 0xFFU);
    prefixed[1] = (uint16_t)((can_id >> 8) & 0xFFU);
    for (i = 0U; i < len; i++) { prefixed[i + 2U] = data[i] & 0xFFU; }
    return crc8_compute(prefixed, (uint16_t)(len + 2U));
}
