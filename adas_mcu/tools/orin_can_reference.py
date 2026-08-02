#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
orin_can_reference.py — Orin Nano 侧 CAN 编码参考实现（与 MCU 字节级一致）

★ 这是给 Jetson Orin Nano 端（尚未部署）用的"发送侧"参考。MCU 的解码
  (ADAS0.0.2/MCU/src/can_comm.c) 与本文件的编码必须逐字节对齐；任何
  字段/缩放/字节序改动都要两侧同步，否则会被 MCU 的 CRC 或范围校验拒绝。

协议契约来源：ADAS0.0.2/MCU/include/adas_can_protocol.h
  - 经典 CAN 2.0A，11 位标准帧，500 kbps，8 字节数据场
  - 多字节整型小端；每帧 byte7 = CRC-8(poly 0x31, MSB-first, init 0x00)
  - CRC 规则与 ADAS0.0.2/SoC 通信契约及 MCU crc8.c 保持一致

用法（真实发送需 python-can；Jetson HIL 上为 PEAK PCAN-USB → SocketCAN can1，
     USB CANalyst-II 仅作为调试备源）：
    python3 orin_can_reference.py            # 打印各帧样例的十六进制
    # 集成到 Orin：
    #   from orin_can_reference import OrinCanEncoder
    #   enc = OrinCanEncoder()
    #   frames = enc.build_all(cmd)           # 返回 [(can_id, bytes8), ...]
    #   for cid, data in frames: bus.send(can.Message(arbitration_id=cid,
    #                                                  data=data, is_extended_id=False))
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field

# ------------------------------------------------------------------ #
# CAN ID（与 adas_can_protocol.h 一致）
# ------------------------------------------------------------------ #
CANID_PRIMARY_BASE = 0x100
CANID_BACKUP_BASE = 0x110
OFFS_HEARTBEAT = 0x0
OFFS_LATERAL = 0x1
OFFS_LONGITUDINAL = 0x2
OFFS_ADAS_STATUS = 0x3
CANID_MCU_CONTROL = 0x201
CANID_MCU_HEARTBEAT = 0x202
CANID_MCU_DIAG = 0x203
CANID_MCU_E2E_DIAG = 0x204
CANID_MCU_LINK_STATS = 0x205
CANID_MCU_SESSION_STATUS = 0x206
CANID_FAULT_INJECT = 0x301
CANID_MCU_FAULT_RESPONSE = 0x302

# 定点缩放
SCALE_STEER_DEG = 0.01
SCALE_STEER_RATE_DPS = 0.1
SCALE_ACCEL_MS2 = 0.001
SCALE_SPEED_MS = 0.01

PROTOCOL_VERSION = 0x03  # v3: session 由 MCU 自驱授权（2026-07 重构）；CRC/序列/字段编码与 v2 兼容

# 系统模式
SYS_MODE_INIT, SYS_MODE_STANDBY, SYS_MODE_ACTIVE, SYS_MODE_DEGRADED = 0, 1, 2, 3
SYS_MODE_MRM, SYS_MODE_EMERGENCY_BRAKE, SYS_MODE_FAILSAFE, SYS_MODE_FAULT_LOCK = 4, 5, 6, 7

FAULT_LEVEL_INFO, FAULT_LEVEL_WARN, FAULT_LEVEL_SEVERE, FAULT_LEVEL_FATAL = 0, 1, 2, 3
AEB_RISK_NONE, AEB_RISK_WARNING, AEB_RISK_PARTIAL, AEB_RISK_FULL = 0, 1, 2, 3
SRC_PRIMARY, SRC_BACKUP = 1, 2
DIR_NEUTRAL, DIR_DRIVE, DIR_REVERSE = 0, 1, 2

# 状态字位（0x103）
ST_CONTROL_ENABLE = 1 << 0
ST_LATERAL_ENABLE = 1 << 1
ST_LONGITUDINAL_ENABLE = 1 << 2
ST_LKA_ACTIVE = 1 << 3
ST_ACC_ACTIVE = 1 << 4
ST_AEB_WARNING = 1 << 5
ST_AEB_BRAKING = 1 << 6
ST_EMERGENCY_STOP = 1 << 7
ST_DEGRADED_MODE = 1 << 8
ST_TAKEOVER_REQUEST = 1 << 9
ST_MRM_REQUEST = 1 << 10
ST_PERCEPTION_VALID = 1 << 11
ST_LOCALIZATION_VALID = 1 << 12
ST_PLANNING_VALID = 1 << 13

# 横向/纵向/ADAS 帧的 flags 位
LAT_F_ENABLE = 1 << 0
LAT_F_LKA_ACTIVE = 1 << 1
LON_F_ENABLE = 1 << 0
LON_F_ACC_ACTIVE = 1 << 1
LON_F_PARKING = 1 << 2
LON_F_DIR_SHIFT = 4
AD_F_ESTOP = 1 << 0
AD_F_MRM = 1 << 1
AD_F_OBSTACLE_VALID = 1 << 2


def crc8(data: bytes) -> int:
    """CRC-8 / poly 0x31, MSB-first, init 0x00（与 crc8.c / serial_protocol.py 一致）。"""
    crc = 0x00
    for byte in data:
        crc ^= byte & 0xFF
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc & 0xFF


def _q_i16(value: float, scale: float) -> int:
    """工程量 → int16（四舍五入 + 饱和），小端由 struct 负责。"""
    q = value / scale
    q = int(q + 0.5) if q >= 0 else int(q - 0.5)
    return max(-32768, min(32767, q))


def _finish(can_id: int, buf: bytearray) -> bytes:
    """填 CRC 到 byte7 并定长为 8。"""
    assert len(buf) == 8, "帧必须 8 字节（CRC 前 7 + CRC 1）"
    data_id = bytes((can_id & 0xFF, (can_id >> 8) & 0xFF))
    buf[7] = crc8(data_id + bytes(buf[:7]))
    return bytes(buf)


def decode_mcu_frame(can_id: int, data: bytes) -> dict[str, int | float | bool]:
    """Decode and authenticate MCU protocol-v3 feedback frames."""
    if len(data) != 8:
        raise ValueError("MCU CAN frame must contain exactly 8 bytes")
    expected = crc8(bytes((can_id & 0xFF, (can_id >> 8) & 0xFF)) + data[:7])
    if data[7] != expected:
        raise ValueError("MCU CAN frame CRC/Data-ID check failed")
    if can_id == CANID_MCU_CONTROL:
        steer_raw, accel_raw = struct.unpack_from("<hh", data, 0)
        return {"steer_deg": steer_raw * SCALE_STEER_DEG,
                "accel_ms2": accel_raw * SCALE_ACCEL_MS2,
                "throttle_pct": data[4], "brake_pct": data[5], "seq": data[6]}
    if can_id == CANID_MCU_HEARTBEAT:
        return {"state": data[0], "active_source": data[1], "safety_flags": data[2],
                "fault_level": data[3], "seq": data[4], "alive": data[5],
                "loop_load_pct": data[6]}
    if can_id == CANID_MCU_DIAG:
        return {"fault_code": data[0] | (data[1] << 8), "reset_reason": data[2],
                "primary_timeout": data[3], "backup_timeout": data[4],
                "crc_errors": data[5], "loop_overruns": data[6]}
    if can_id == CANID_MCU_E2E_DIAG:
        return {"primary_seq_errors": data[0], "backup_seq_errors": data[1],
                "protocol_flags": data[2], "can_recovery_count": data[3],
                "self_test_mask": data[4], "test_build": bool(data[5] & 1),
                "protocol_version": data[6]}
    raise ValueError("unsupported MCU CAN ID: 0x{:03X}".format(can_id))


@dataclass
class AdasCommand:
    """Orin 一个控制周期要下发的全部量（工程单位）。"""

    # 横向（0x101）
    target_steer_deg: float = 0.0
    target_steer_rate_dps: float = 400.0
    lateral_enable: bool = False
    lka_active: bool = False
    # 纵向（0x102）
    target_accel_ms2: float = 0.0
    target_speed_ms: float = 0.0
    brake_request_pct: int = 0
    longitudinal_enable: bool = False
    acc_active: bool = False
    parking_brake: bool = False
    drive_dir: int = DIR_DRIVE
    # ADAS 状态与安全（0x103）
    status_word: int = 0
    aeb_risk: int = AEB_RISK_NONE
    aeb_required_decel: float = 0.0
    emergency_stop: bool = False
    mrm_request: bool = False
    obstacle_valid: bool = False
    # SoC 心跳（0x100）
    system_mode: int = SYS_MODE_ACTIVE
    soc_health: int = 1
    fault_level: int = FAULT_LEVEL_INFO
    control_authority: bool = True


class OrinCanEncoder:
    """把 AdasCommand 编码成 4 条 CAN 帧；主/备两路各一份。"""

    def __init__(self, source_id: int = SRC_PRIMARY):
        self.source_id = source_id
        self.base = CANID_PRIMARY_BASE if source_id == SRC_PRIMARY else CANID_BACKUP_BASE
        self._seq = {"hb": 0, "lat": 0, "lon": 0, "status": 0}
        self._alive = 0

    def _next_seq(self, channel: str) -> int:
        s = self._seq[channel] & 0xFF
        self._seq[channel] = (s + 1) & 0xFF
        return s

    def build_heartbeat(self, cmd: AdasCommand) -> tuple[int, bytes]:
        buf = bytearray(8)
        buf[0] = cmd.system_mode & 0xFF
        buf[1] = cmd.soc_health & 0xFF
        buf[2] = cmd.fault_level & 0xFF
        buf[3] = (PROTOCOL_VERSION << 4) | (0x01 if cmd.control_authority else 0x00)
        buf[4] = self.source_id & 0xFF
        buf[5] = self._next_seq("hb")
        self._alive = (self._alive + 1) & 0xFF
        buf[6] = self._alive
        can_id = self.base + OFFS_HEARTBEAT
        return can_id, _finish(can_id, buf)

    def build_lateral(self, cmd: AdasCommand) -> tuple[int, bytes]:
        buf = bytearray(8)
        struct.pack_into("<h", buf, 0, _q_i16(cmd.target_steer_deg, SCALE_STEER_DEG))
        struct.pack_into("<h", buf, 2, _q_i16(cmd.target_steer_rate_dps, SCALE_STEER_RATE_DPS))
        flags = 0
        if cmd.lateral_enable:
            flags |= LAT_F_ENABLE
        if cmd.lka_active:
            flags |= LAT_F_LKA_ACTIVE
        buf[4] = flags
        buf[5] = self._next_seq("lat")
        buf[6] = 0
        can_id = self.base + OFFS_LATERAL
        return can_id, _finish(can_id, buf)

    def build_longitudinal(self, cmd: AdasCommand) -> tuple[int, bytes]:
        buf = bytearray(8)
        struct.pack_into("<h", buf, 0, _q_i16(cmd.target_accel_ms2, SCALE_ACCEL_MS2))
        struct.pack_into("<h", buf, 2, _q_i16(cmd.target_speed_ms, SCALE_SPEED_MS))
        buf[4] = max(0, min(100, int(cmd.brake_request_pct))) & 0xFF
        flags = 0
        if cmd.longitudinal_enable:
            flags |= LON_F_ENABLE
        if cmd.acc_active:
            flags |= LON_F_ACC_ACTIVE
        if cmd.parking_brake:
            flags |= LON_F_PARKING
        flags |= (cmd.drive_dir & 0x3) << LON_F_DIR_SHIFT
        buf[5] = flags
        buf[6] = self._next_seq("lon")
        can_id = self.base + OFFS_LONGITUDINAL
        return can_id, _finish(can_id, buf)

    def build_adas_status(self, cmd: AdasCommand) -> tuple[int, bytes]:
        buf = bytearray(8)
        buf[0] = cmd.status_word & 0xFF
        buf[1] = (cmd.status_word >> 8) & 0xFF
        buf[2] = cmd.aeb_risk & 0xFF
        struct.pack_into("<h", buf, 3, _q_i16(abs(cmd.aeb_required_decel), SCALE_ACCEL_MS2))
        flags = 0
        if cmd.emergency_stop:
            flags |= AD_F_ESTOP
        if cmd.mrm_request:
            flags |= AD_F_MRM
        if cmd.obstacle_valid:
            flags |= AD_F_OBSTACLE_VALID
        buf[5] = flags
        buf[6] = self._next_seq("status")
        can_id = self.base + OFFS_ADAS_STATUS
        return can_id, _finish(can_id, buf)

    def build_all(self, cmd: AdasCommand) -> list[tuple[int, bytes]]:
        """返回本周期 4 条帧 [(can_id, 8bytes), ...]。"""
        return [
            self.build_heartbeat(cmd),
            self.build_lateral(cmd),
            self.build_longitudinal(cmd),
            self.build_adas_status(cmd),
        ]


def _demo() -> None:
    enc = OrinCanEncoder(source_id=SRC_PRIMARY)
    cmd = AdasCommand(
        target_steer_deg=6.5,
        target_steer_rate_dps=300.0,
        lateral_enable=True,
        lka_active=True,
        target_accel_ms2=1.2,
        target_speed_ms=13.9,
        longitudinal_enable=True,
        acc_active=True,
        drive_dir=DIR_DRIVE,
        status_word=(ST_CONTROL_ENABLE | ST_LATERAL_ENABLE | ST_LONGITUDINAL_ENABLE
                     | ST_LKA_ACTIVE | ST_ACC_ACTIVE | ST_PERCEPTION_VALID
                     | ST_LOCALIZATION_VALID | ST_PLANNING_VALID),
        aeb_risk=AEB_RISK_NONE,
        system_mode=SYS_MODE_ACTIVE,
    )
    print("周期样例帧（Orin 主路 → MCU）：")
    for cid, data in enc.build_all(cmd):
        print("  0x{:03X}  {}".format(cid, " ".join("{:02X}".format(b) for b in data)))

    # AEB 全制动样例
    aeb = AdasCommand(
        longitudinal_enable=True,
        target_accel_ms2=-8.0,
        brake_request_pct=100,
        aeb_risk=AEB_RISK_FULL,
        aeb_required_decel=8.0,
        emergency_stop=True,
        obstacle_valid=True,
        status_word=(ST_CONTROL_ENABLE | ST_LONGITUDINAL_ENABLE
                     | ST_AEB_BRAKING | ST_EMERGENCY_STOP),
        system_mode=SYS_MODE_EMERGENCY_BRAKE,
        fault_level=FAULT_LEVEL_SEVERE,
    )
    print("\nAEB 全制动样例帧：")
    for cid, data in enc.build_all(aeb):
        print("  0x{:03X}  {}".format(cid, " ".join("{:02X}".format(b) for b in data)))

    # CRC 自检向量
    print("\nCRC-8 自检: crc8(b'123456789') = 0x{:02X}".format(crc8(b"123456789")))


if __name__ == "__main__":
    _demo()
