#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_servo.py — 通过 CANalyst-II 发送 CAN 帧操控 MCU 舵机

步骤：
  1. v3 会话握手 (0x104 → 0x206)
  2. 持续发送 SoC 心跳 + 横向控制帧
  3. 实时打印 MCU 回传的 0x201(最终控制量) 和 0x202(心跳)
  4. 按 '+'/'-' 键盘交互调整转角

按 Ctrl+C 退出时自动发送急停帧然后关闭。

协议契约：MCU/include/adas_can_protocol.h + tools/orin_can_reference.py
"""

from __future__ import annotations

import can
import struct
import sys
import time
import signal
import threading
import select
import termios
import tty

# ── CRC-8 ──────────────────────────────────────────────────────────────
def crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte & 0xFF
        for _ in range(8):
            crc = (crc << 1) ^ 0x31 if crc & 0x80 else crc << 1
            crc &= 0xFF
    return crc

def _finish(can_id: int, buf: bytearray) -> bytes:
    assert len(buf) == 8
    did = bytes((can_id & 0xFF, (can_id >> 8) & 0xFF))
    buf[7] = crc8(did + bytes(buf[:7]))
    return bytes(buf)

# ── CAN ID ─────────────────────────────────────────────────────────────
CID_HEARTBEAT      = 0x100
CID_LATERAL        = 0x101
CID_LONGITUDINAL   = 0x102
CID_STATUS         = 0x103
CID_SESSION_CTRL   = 0x104

CID_MCU_CONTROL    = 0x201
CID_MCU_HEARTBEAT  = 0x202
CID_MCU_DIAG       = 0x203
CID_MCU_SESSION    = 0x206

# ── v3 Session ─────────────────────────────────────────────────────────
HIL_REQ_ANNOUNCE      = 1
HIL_REQ_SYNC_STANDBY  = 2
HIL_REQ_COMMIT_ACTIVE = 4
HIL_REQ_ACTIVE        = 5
HIL_ACK_SESSION_ACCEPTED = 1
HIL_ACK_READY         = 2
HIL_ACK_ARMED         = 3
HIL_ACK_ACTIVE        = 4

PROTOCOL_VERSION   = 3
SYSMODE_ACTIVE     = 2

MCU_STATE_NAMES = {
    0: "INIT", 1: "STANDBY", 2: "ACTIVE", 3: "DEGRADED",
    4: "MRM", 5: "EM_BRAKE", 6: "FAILSAFE", 7: "FAULT_LOCK"
}

SESSION_NAMES = {
    0: "BOOT_WAIT", 1: "SESSION_ACCEPTED", 2: "READY",
    3: "ARMED", 4: "ACTIVE", 5: "RECOVERY_REQUIRED",
    6: "REJECTED", 7: "FAULT_LOCK"
}

# ── Global state ───────────────────────────────────────────────────────
running = True
steer_deg = 0.0
seq = {"hb": 0, "lat": 0, "lon": 0, "status": 0, "session": 0}
alive = 0
session_id = 0  # MAC-style: random per restart, MCU remembers

# ── Build frames ───────────────────────────────────────────────────────
def build_session_frame(phase: int) -> tuple[int, bytes]:
    global session_id
    buf = bytearray(8)
    buf[0] = PROTOCOL_VERSION
    buf[1] = phase
    buf[2] = session_id & 0xFF
    buf[3] = (session_id >> 8) & 0xFF
    buf[6] = seq["session"]
    seq["session"] = (seq["session"] + 1) & 0xFF
    return CID_SESSION_CTRL, _finish(CID_SESSION_CTRL, buf)

def build_heartbeat() -> tuple[int, bytes]:
    global alive
    buf = bytearray(8)
    buf[0] = SYSMODE_ACTIVE
    buf[1] = 1  # soc_health
    buf[2] = 0  # fault_level
    buf[3] = (PROTOCOL_VERSION << 4) | 0x01  # authority
    buf[4] = 1  # source_id = PRIMARY
    buf[5] = seq["hb"]
    seq["hb"] = (seq["hb"] + 1) & 0xFF
    alive = (alive + 1) & 0xFF
    buf[6] = alive
    return CID_HEARTBEAT, _finish(CID_HEARTBEAT, buf)

def build_lateral(deg: float) -> tuple[int, bytes]:
    buf = bytearray(8)
    # steer: ±327.67° range, 0.01°/LSB
    raw = int(deg / 0.01)
    raw = max(-32768, min(32767, raw))
    struct.pack_into("<h", buf, 0, raw)
    # rate limit: 400°/s default
    struct.pack_into("<h", buf, 2, int(400 / 0.1))
    # flags: lateral_enable = bit0, no LKA active
    buf[4] = 0x01
    buf[5] = seq["lat"]
    seq["lat"] = (seq["lat"] + 1) & 0xFF
    return CID_LATERAL, _finish(CID_LATERAL, buf)

def build_longitudinal() -> tuple[int, bytes]:
    buf = bytearray(8)
    struct.pack_into("<h", buf, 0, 0)  # accel=0
    struct.pack_into("<h", buf, 2, 0)  # speed=0
    buf[4] = 0    # brake=0
    buf[5] = 0    # no enable
    buf[6] = seq["lon"]
    seq["lon"] = (seq["lon"] + 1) & 0xFF
    return CID_LONGITUDINAL, _finish(CID_LONGITUDINAL, buf)

def build_status() -> tuple[int, bytes]:
    buf = bytearray(8)
    # status_word: lateral_enable + control_enable + perception_valid
    status = (1 << 0) | (1 << 1) | (1 << 11) | (1 << 12) | (1 << 13)
    buf[0] = status & 0xFF
    buf[1] = (status >> 8) & 0xFF
    buf[2] = 0     # AEB risk none
    buf[3] = 0     # decel
    buf[4] = 0
    buf[5] = 0
    buf[6] = seq["status"]
    seq["status"] = (seq["status"] + 1) & 0xFF
    return CID_STATUS, _finish(CID_STATUS, buf)

def build_estop() -> tuple[int, bytes]:
    """Emergency stop: zero everything, set estop flag."""
    buf = bytearray(8)
    buf[0] = (1 << 7) & 0xFF               # EMERGENCY_STOP
    buf[1] = (1 << 7) >> 8
    buf[2] = 2                              # AEB_PARTIAL
    buf[3] = 0
    buf[4] = 0
    buf[5] = 0x01                           # ESTOP flag
    buf[6] = seq["status"]
    return CID_STATUS, _finish(CID_STATUS, buf)

# ── Decode MCU responses ───────────────────────────────────────────────
def decode_mcu_data(can_id: int, data: bytes) -> str:
    """Decode a single MCU frame into human-readable string."""
    if len(data) != 8:
        return f"  ← bad len={len(data)}"
    # Verify CRC
    did = bytes((can_id & 0xFF, (can_id >> 8) & 0xFF))
    expected = crc8(did + data[:7])
    crc_ok = data[7] == expected

    prefix = "✓" if crc_ok else "✗CRC"

    if can_id == CID_MCU_CONTROL:
        steer_raw, = struct.unpack_from("<h", data, 0)
        return f"{prefix} steer={steer_raw*0.01:.2f}°  seq={data[6]}"
    elif can_id == CID_MCU_HEARTBEAT:
        state = data[0]
        return f"{prefix} state={MCU_STATE_NAMES.get(state, state)}  src={data[1]}  sf=0x{data[2]:02X}  load={data[6]}%"
    elif can_id == CID_MCU_SESSION:
        ack = data[1]
        rcvd_sid = data[2] | (data[3] << 8)
        return f"{prefix} ack={SESSION_NAMES.get(ack, ack)}  sid=0x{rcvd_sid:04X}"
    elif can_id == CID_MCU_DIAG:
        fc = data[0] | (data[1] << 8)
        return f"{prefix} fault=0x{fc:04X}  crc_err={data[5]}  overrun={data[6]}"
    else:
        return f"{prefix} raw={' '.join(f'{b:02X}' for b in data)}"

# ── Key reader thread ──────────────────────────────────────────────────
def key_reader():
    global steer_deg, running
    restore = False
    try:
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setraw(fd)
        restore = True
    except termios.error:
        pass
    try:
        while running:
            if not restore:
                time.sleep(0.2)
                continue
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not r:
                continue
            ch = sys.stdin.read(1)
            if ch == '+':
                steer_deg = min(30.0, steer_deg + 5.0)
            elif ch == '-':
                steer_deg = max(-30.0, steer_deg - 5.0)
            elif ch == '0':
                steer_deg = 0.0
            elif ch in ('q', '\x03'):
                running = False
    finally:
        if restore:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

# ── Main loop ──────────────────────────────────────────────────────────
def main():
    global running, steer_deg, session_id

    import random
    session_id = random.randint(0x0100, 0xFFFF)  # pseudo-random MAC
    print(f"🔌 打开 CANalyst-II @ 500kbps ... session_id=0x{session_id:04X}")
    bus = can.interface.Bus(interface='canalystii', channel=1, bitrate=500000)
    print(f"   总线: {bus}")

    # Start key reader thread
    key_thread = threading.Thread(target=key_reader, daemon=True)
    key_thread.start()

    signal.signal(signal.SIGINT, lambda s, f: setattr(sys.modules[__name__], 'running', False))

    session_established = False
    last_session_announce = 0
    last_frame_time = 0
    frame_interval = 0.010   # 10ms for lateral/long
    hb_interval = 0.020      # 20ms for heartbeat/status

    is_interactive = sys.stdin.isatty()
    if is_interactive:
        print("\n" + "=" * 70)
        print("🎮  舵机操控模式 (交互)")
        print("   +  =  左转 5°    -  =  右转 5°    0  =  回中")
        print("   q  =  退出(自动急停)")
        print("=" * 70)
    else:
        print("\n" + "=" * 70)
        print("🤖  自动扫舵模式 (非交互)")
        print("   ±15° 正弦摆动，每周期 2 秒")
        print("=" * 70)

    try:
        t0 = time.monotonic()
        last_hb = t0
        last_print = t0
        session_phase = 0
        last_actual_steer = 0.0

        while running:
            now = time.monotonic()
            elapsed = now - t0

            # Auto-sweep in non-interactive mode
            if not is_interactive:
                steer_deg = 15.0 * __import__('math').sin(now * __import__('math').pi)

            # ── Phase 1: Session handshake progression ──
            if not session_established and now - last_session_announce > 0.100:
                if session_phase < HIL_ACK_SESSION_ACCEPTED:
                    # Not yet accepted: send ANNOUNCE to establish
                    cid, data = build_session_frame(HIL_REQ_ANNOUNCE)
                elif session_phase < HIL_ACK_READY:
                    # SESSION_ACCEPTED → need SYNC_STANDBY to reach READY
                    cid, data = build_session_frame(HIL_REQ_SYNC_STANDBY)
                elif session_phase < HIL_ACK_ACTIVE:
                    # READY/ARMED → send COMMIT_ACTIVE
                    cid, data = build_session_frame(HIL_REQ_COMMIT_ACTIVE)
                else:
                    # ACTIVE reached
                    cid, data = build_session_frame(HIL_REQ_ACTIVE)
                bus.send(can.Message(arbitration_id=cid, data=data, is_extended_id=False))
                last_session_announce = now

            # ── Phase 2: Control frames ──
            if now - last_frame_time >= frame_interval:
                msgs = [
                    build_lateral(steer_deg),
                    build_longitudinal(),
                ]
                for cid, data in msgs:
                    bus.send(can.Message(arbitration_id=cid, data=data, is_extended_id=False))
                last_frame_time = now

            # Heartbeat + Status at 20ms
            if now - last_hb >= hb_interval:
                cid_h, d_h = build_heartbeat()
                bus.send(can.Message(arbitration_id=cid_h, data=d_h, is_extended_id=False))
                cid_s, d_s = build_status()
                bus.send(can.Message(arbitration_id=cid_s, data=d_s, is_extended_id=False))
                last_hb = now

            # ── Read MCU responses ──
            msg = bus.recv(timeout=0.001)
            while msg is not None:
                cid = msg.arbitration_id
                info = decode_mcu_data(cid, msg.data)
                if cid == CID_MCU_SESSION:
                    ack = msg.data[1]
                    new_phase = ack
                    if new_phase != session_phase:
                        session_phase = new_phase
                        print(f"[t={elapsed:5.1f}s] 📡 MCU会话: {SESSION_NAMES.get(ack, ack)} ({info})")
                    if new_phase >= HIL_ACK_ACTIVE:
                        session_established = True
                elif cid == CID_MCU_HEARTBEAT:
                    state = msg.data[0]
                    if state >= 7:
                        print(f"[t={elapsed:5.1f}s] ⚠️  {info}")
                elif cid == CID_MCU_CONTROL:
                    steer_raw, = struct.unpack_from("<h", msg.data, 0)
                    last_actual_steer = steer_raw * 0.01
                elif cid == CID_MCU_DIAG and msg.data[5] > 0:
                    print(f"[t={elapsed:5.1f}s] 🔧 {info}")
                msg = bus.recv(timeout=0.0)

            # ── Periodic status print ──
            if now - last_print >= 0.5:
                phase_str = SESSION_NAMES.get(session_phase, '?')
                print(f"[t={elapsed:5.1f}s] 目标={steer_deg:+6.2f}°  实际={last_actual_steer:+6.2f}°  会话={phase_str}")
                last_print = now

            time.sleep(0.002)

    finally:
        # ── Clean exit: send E-Stop ──
        print("\n🛑 发送急停帧...")
        for _ in range(5):
            cid, data = build_estop()
            bus.send(can.Message(arbitration_id=cid, data=data, is_extended_id=False))
            cid_h, d_h = build_heartbeat()
            bus.send(can.Message(arbitration_id=cid_h, data=d_h, is_extended_id=False))
            time.sleep(0.020)

        # Drain remaining messages
        for _ in range(10):
            msg = bus.recv(timeout=0.01)
            if msg:
                print(f"  ↳ {decode_mcu_data(msg.arbitration_id, msg.data)}")

        bus.shutdown()
        print("✅ CAN 总线已关闭。")

if __name__ == "__main__":
    main()
