/*
 * crc8.h — CRC-8 校验（多项式 0x31, MSB-first, 初值 0x00, 无反转/无异或输出）
 *
 * 与 ADAS0.0.2/SoC 通信契约及 tools/orin_can_reference.* 完全同算法，
 * 保证 SoC 侧与 MCU 侧字节级一致。用于所有 CAN 帧的完整性校验。
 *
 * 输入 data 为 uint16_t 数组（C28x 平台每元素承载 1 字节，低 8 位有效）。
 */
#ifndef CRC8_H_
#define CRC8_H_

#include <stdint.h>

/* 计算 data[0..len-1]（各元素低 8 位）的 CRC-8。返回 0..255。 */
uint16_t crc8_compute(const uint16_t *data, uint16_t len);
uint16_t crc8_compute_frame(uint32_t can_id, const uint16_t *data, uint16_t len);

#endif /* CRC8_H_ */
