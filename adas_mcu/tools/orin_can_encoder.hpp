// orin_can_encoder.hpp — Orin/Jetson 侧 CAN 编码器（主/备两路，字节级契约单一来源）
//
// 与 MCU 解码器 src/can_comm.c、Python 版 orin_can_reference.py 三方【字节级一致】。
// ★ 直接复用 adas_can_protocol.h（与 MCU 同一份契约头）——纯宏，平台无关。
//
// 本头把编码器抽成可复用单元：orin_can_reference.cpp（联调打印/SocketCAN 参考）与
// orin_shadow_sender.cpp（CANalyst-II 影子备源真发）共用同一份 OrinCanEncoder，杜绝抄错。
#ifndef ORIN_CAN_ENCODER_HPP_
#define ORIN_CAN_ENCODER_HPP_

#include <cstdint>
#include <cstring>

#include "adas_can_protocol.h"  // 与 MCU 共享的线格式契约（编译加 -I../include）

namespace adas_can {

// ------------------------------------------------------------------ //
// CRC-8：poly 0x31, MSB-first, init 0x00（与 crc8.c / serial_protocol.py 一致）
// ------------------------------------------------------------------ //
inline uint8_t crc8(const uint8_t* data, int len) {
  uint8_t crc = 0x00;
  for (int i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31)
                         : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

// 工程量 → int16（四舍五入 + 饱和，half away from zero），与 Python _q_i16 一致
inline int16_t q_i16(float value, float scale) {
  float q = value / scale;
  long v = (q >= 0.0f) ? static_cast<long>(q + 0.5f) : static_cast<long>(q - 0.5f);
  if (v > 32767) v = 32767;
  if (v < -32768) v = -32768;
  return static_cast<int16_t>(v);
}

// 小端打包 int16 到 data[i], data[i+1]
inline void put_i16(uint8_t* d, int i, int16_t v) {
  d[i] = static_cast<uint8_t>(v & 0xFF);
  d[i + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

// 填 byte7 = CRC(CAN_ID低,高 + byte0..6)
inline void finish(uint32_t can_id, uint8_t* d) {
  uint8_t protected_data[CAN_FRAME_DLC + 1];
  protected_data[0] = static_cast<uint8_t>(can_id & 0xFFU);
  protected_data[1] = static_cast<uint8_t>((can_id >> 8) & 0xFFU);
  std::memcpy(&protected_data[2], d, CAN_FRAME_DLC - 1);
  d[CAN_FRAME_DLC - 1] = crc8(protected_data, CAN_FRAME_DLC + 1);
}

// ------------------------------------------------------------------ //
// Orin 一个控制周期要下发的全部量（工程单位）
// ------------------------------------------------------------------ //
struct AdasCommand {
  // 横向（0x101/0x111）
  float target_steer_deg = 0.0f;
  float target_steer_rate_dps = 400.0f;
  bool lateral_enable = false;
  bool lka_active = false;
  // 纵向（0x102/0x112）
  float target_accel_ms2 = 0.0f;
  float target_speed_ms = 0.0f;
  int brake_request_pct = 0;
  bool longitudinal_enable = false;
  bool acc_active = false;
  bool parking_brake = false;
  int drive_dir = DIR_DRIVE;
  // ADAS 状态与安全（0x103/0x113）
  uint16_t status_word = 0;
  int aeb_risk = AEB_RISK_NONE;
  float aeb_required_decel = 0.0f;
  bool emergency_stop = false;
  bool mrm_request = false;
  bool obstacle_valid = false;
  // SoC 心跳（0x100/0x110）
  int system_mode = SYS_MODE_ACTIVE;
  int soc_health = 1;
  int fault_level = FAULT_LEVEL_INFO;
  bool control_authority = true;
};

// 一条待发帧（8 字节数据场）
struct Frame {
  uint32_t id = 0;
  uint8_t data[CAN_FRAME_DLC] = {0};
};

// ------------------------------------------------------------------ //
// 编码器：把 AdasCommand 编成 4 条帧；SRC_PRIMARY→0x10x，SRC_BACKUP→0x11x
// ------------------------------------------------------------------ //
class OrinCanEncoder {
 public:
  explicit OrinCanEncoder(int source_id = SRC_PRIMARY)
      : source_id_(source_id),
        base_(source_id == SRC_PRIMARY ? CANID_PRIMARY_BASE : CANID_BACKUP_BASE) {}

  Frame build_heartbeat(const AdasCommand& c) {
    Frame f;
    f.id = base_ + CANID_OFFS_HEARTBEAT;
    uint8_t* d = f.data;
    std::memset(d, 0, CAN_FRAME_DLC);
    d[HB_B_SYS_MODE] = static_cast<uint8_t>(c.system_mode);
    d[HB_B_SOC_HEALTH] = static_cast<uint8_t>(c.soc_health);
    d[HB_B_FAULT_LEVEL] = static_cast<uint8_t>(c.fault_level);
    d[HB_B_AUTHORITY] = static_cast<uint8_t>((ADAS_PROTOCOL_VERSION << HB_VERSION_SHIFT) |
                                             (c.control_authority ? HB_F_AUTHORITY : 0U));
    d[HB_B_SOURCE_ID] = static_cast<uint8_t>(source_id_);
    d[HB_B_SEQ] = next_seq(seq_hb_);
    alive_ = static_cast<uint8_t>(alive_ + 1);
    d[HB_B_ALIVE] = alive_;
    finish(f.id, d);
    return f;
  }

  Frame build_lateral(const AdasCommand& c) {
    Frame f;
    f.id = base_ + CANID_OFFS_LATERAL;
    uint8_t* d = f.data;
    std::memset(d, 0, CAN_FRAME_DLC);
    put_i16(d, LAT_B_ANGLE_LO, q_i16(c.target_steer_deg, SCALE_STEER_DEG));
    put_i16(d, LAT_B_RATE_LO, q_i16(c.target_steer_rate_dps, SCALE_STEER_RATE_DPS));
    uint8_t flags = 0;
    if (c.lateral_enable) flags |= LAT_F_ENABLE;
    if (c.lka_active) flags |= LAT_F_LKA_ACTIVE;
    d[LAT_B_FLAGS] = flags;
    d[LAT_B_SEQ] = next_seq(seq_lat_);
    finish(f.id, d);
    return f;
  }

  Frame build_longitudinal(const AdasCommand& c) {
    Frame f;
    f.id = base_ + CANID_OFFS_LONGITUDINAL;
    uint8_t* d = f.data;
    std::memset(d, 0, CAN_FRAME_DLC);
    put_i16(d, LON_B_ACC_LO, q_i16(c.target_accel_ms2, SCALE_ACCEL_MS2));
    put_i16(d, LON_B_SPD_LO, q_i16(c.target_speed_ms, SCALE_SPEED_MS));
    int brk = c.brake_request_pct;
    if (brk < 0) brk = 0;
    if (brk > 100) brk = 100;
    d[LON_B_BRAKE] = static_cast<uint8_t>(brk);
    uint8_t flags = 0;
    if (c.longitudinal_enable) flags |= LON_F_ENABLE;
    if (c.acc_active) flags |= LON_F_ACC_ACTIVE;
    if (c.parking_brake) flags |= LON_F_PARKING;
    flags |= static_cast<uint8_t>((c.drive_dir & LON_F_DIR_MASK) << LON_F_DIR_SHIFT);
    d[LON_B_FLAGS] = flags;
    d[LON_B_SEQ] = next_seq(seq_lon_);
    finish(f.id, d);
    return f;
  }

  Frame build_adas_status(const AdasCommand& c) {
    Frame f;
    f.id = base_ + CANID_OFFS_ADAS_STATUS;
    uint8_t* d = f.data;
    std::memset(d, 0, CAN_FRAME_DLC);
    d[AD_B_STATUS_LO] = static_cast<uint8_t>(c.status_word & 0xFF);
    d[AD_B_STATUS_HI] = static_cast<uint8_t>((c.status_word >> 8) & 0xFF);
    d[AD_B_AEB_RISK] = static_cast<uint8_t>(c.aeb_risk);
    float decel = c.aeb_required_decel < 0 ? -c.aeb_required_decel : c.aeb_required_decel;
    put_i16(d, AD_B_DECEL_LO, q_i16(decel, SCALE_ACCEL_MS2));
    uint8_t flags = 0;
    if (c.emergency_stop) flags |= AD_F_ESTOP;
    if (c.mrm_request) flags |= AD_F_MRM;
    if (c.obstacle_valid) flags |= AD_F_OBSTACLE_VALID;
    d[AD_B_FLAGS] = flags;
    d[AD_B_SEQ] = next_seq(seq_status_);
    finish(f.id, d);
    return f;
  }

  // 本周期 4 条帧（心跳/横向/纵向/ADAS），与 Python build_all 顺序一致
  int encode_all(const AdasCommand& c, Frame out[4]) {
    out[0] = build_heartbeat(c);
    out[1] = build_lateral(c);
    out[2] = build_longitudinal(c);
    out[3] = build_adas_status(c);
    return 4;
  }

  int source_id() const { return source_id_; }
  uint32_t base() const { return base_; }

 private:
  static uint8_t next_seq(uint8_t& counter) {
    uint8_t s = counter;
    counter = static_cast<uint8_t>(counter + 1);
    return s;
  }
  int source_id_;
  uint32_t base_;
  uint8_t seq_hb_ = 0, seq_lat_ = 0, seq_lon_ = 0, seq_status_ = 0;
  uint8_t alive_ = 0;
};

}  // namespace adas_can

#endif  // ORIN_CAN_ENCODER_HPP_
