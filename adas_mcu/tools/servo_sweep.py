#!/usr/bin/env python3
"""
servo_sweep.py — 通过 CANalyst-II 向 MCU 发帧让舵机左右摆动 + 转向灯验证

用法：
    python3 servo_sweep.py           # 默认通道1（面板通道2），500kbps
    python3 servo_sweep.py --ch 1    # 指定通道
    python3 servo_sweep.py --list    # 只列出 MCU 回传帧
"""
from __future__ import annotations

import argparse
import can
import sys
import time
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from orin_can_reference import (
    OrinCanEncoder, AdasCommand, crc8,
    SRC_PRIMARY, SYS_MODE_ACTIVE, DIR_DRIVE,
    FAULT_LEVEL_INFO, AEB_RISK_NONE,
    ST_CONTROL_ENABLE, ST_LATERAL_ENABLE, ST_LONGITUDINAL_ENABLE,
    ST_LKA_ACTIVE, ST_ACC_ACTIVE,
    ST_PERCEPTION_VALID, ST_LOCALIZATION_VALID, ST_PLANNING_VALID,
    CANID_MCU_CONTROL, CANID_MCU_HEARTBEAT, CANID_MCU_DIAG, CANID_MCU_E2E_DIAG,
    decode_mcu_frame,
)

CAN_CHANNEL = 1        # CANalyst-II 面板通道2
CAN_BITRATE = 500000

# v3 session protocol
CANID_SESSION_REQ = 0x104
CANID_SESSION_ACK = 0x206
HIL_REQ_ANNOUNCE = 1
HIL_REQ_SYNC_STANDBY = 2
PROTOCOL_VERSION = 3

ACK_NAMES = ["BOOT_WAIT", "SESSION_ACCEPTED", "READY", "ARMED",
             "ACTIVE", "RECOVERY_REQUIRED", "REJECTED", "FAULT_LOCK"]
STATE_NAMES = ["INIT", "STANDBY", "ACTIVE", "DEGRADED",
               "MRM", "EMERGENCY_BRAKE", "FAILSAFE", "FAULT_LOCK"]


def build_session_req(request_code: int, session_id: int, seq: int) -> tuple[int, bytes]:
    """构建 0x104 session request 帧"""
    buf = bytearray(8)
    buf[0] = PROTOCOL_VERSION
    buf[1] = request_code
    buf[2] = session_id & 0xFF
    buf[3] = (session_id >> 8) & 0xFF
    buf[4] = (session_id >> 16) & 0xFF
    buf[5] = (session_id >> 24) & 0xFF
    buf[6] = seq & 0xFF
    data_id = bytes((CANID_SESSION_REQ & 0xFF, (CANID_SESSION_REQ >> 8) & 0xFF))
    buf[7] = crc8(data_id + bytes(buf[:7]))
    return CANID_SESSION_REQ, bytes(buf)


def decode_session_ack(data: bytes) -> dict:
    """解码 0x206 session ACK"""
    can_id = CANID_SESSION_ACK
    exp_crc = crc8(bytes((can_id & 0xFF, (can_id >> 8) & 0xFF)) + data[:7])
    crc_ok = data[7] == exp_crc
    return {
        "version": data[0],
        "ack": data[1],
        "session_id": data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24),
        "sequence": data[6],
        "crc_ok": crc_ok,
    }


def servo_sweep(bus):
    """舵机左右扫 + 转向灯联动 + MCU 回显"""
    enc = OrinCanEncoder(source_id=SRC_PRIMARY)
    session_seq = 0
    session_id = 0x12345678
    mcu_ack = 0        # 最近观测到的 ACK
    mcu_state = 0       # 最近观测到的系统状态

    base_cmd = AdasCommand(
        target_steer_deg=0.0,
        lateral_enable=True, lka_active=True,
        target_accel_ms2=0.0, target_speed_ms=0.0,
        longitudinal_enable=True, acc_active=True,
        drive_dir=DIR_DRIVE,
        status_word=(ST_CONTROL_ENABLE | ST_LATERAL_ENABLE | ST_LONGITUDINAL_ENABLE
                     | ST_LKA_ACTIVE | ST_ACC_ACTIVE | ST_PERCEPTION_VALID
                     | ST_LOCALIZATION_VALID | ST_PLANNING_VALID),
        system_mode=SYS_MODE_ACTIVE,
    )

    print("=" * 60)
    print("MCU 舵机测试")
    print("1) 发送 0x104 ANNOUNCE → 建立 session")
    print("2) 自动 READY→ARMED→ACTIVE (~300ms fresh 链路)")
    print("3) 舵机跟随 target_steer_deg 偏转")
    print("   转向灯在转角 > ±5° 时闪烁")
    print("=" * 60)

    # ---- 阶段1：Session 握手 ----
    print("\n[阶段1] ANNOUNCE → 建立 session...")
    for tick in range(200):  # 2 秒
        now_ms = tick * 10

        # 发送 ANNOUNCE 直到 MCU 应答 SESSION_ACCEPTED
        if mcu_ack < 1:  # 还没到 SESSION_ACCEPTED
            sid, sdata = build_session_req(HIL_REQ_ANNOUNCE, session_id, session_seq)
            bus.send(can.Message(arbitration_id=sid, data=sdata, is_extended_id=False))
            if tick % 2 == 0:
                session_seq = (session_seq + 1) & 0xFF
        # 一旦 SESSION_ACCEPTED，发 SYNC_STANDBY 推进到 READY
        elif mcu_ack == 1:  # SESSION_ACCEPTED → 需要 SYNC_STANDBY
            sid, sdata = build_session_req(HIL_REQ_SYNC_STANDBY, session_id, session_seq)
            bus.send(can.Message(arbitration_id=sid, data=sdata, is_extended_id=False))
            session_seq = (session_seq + 1) & 0xFF

        # 控制帧（含使能位，让链路保持 fresh）
        for cid, data in enc.build_all(base_cmd):
            bus.send(can.Message(arbitration_id=cid, data=data, is_extended_id=False))

        # 收 MCU 回传
        msg = bus.recv(timeout=0.001)
        if msg is None:
            continue
        aid = msg.arbitration_id

        if aid == CANID_SESSION_ACK:
            try:
                d = decode_session_ack(msg.data)
                if d["crc_ok"] and d["ack"] != mcu_ack:
                    mcu_ack = d["ack"]
                    an = ACK_NAMES[d["ack"]] if d["ack"] < len(ACK_NAMES) else f"?({d['ack']})"
                    print(f"  Session ACK → {an}  (id=0x{d['session_id']:08X})")
            except:
                pass

        elif aid == CANID_MCU_HEARTBEAT:
            try:
                d = decode_mcu_frame(aid, msg.data)
                s = d['state']
                if s != mcu_state:
                    mcu_state = s
                    sn = STATE_NAMES[s] if s < len(STATE_NAMES) else f"?({s})"
                    srcs = {0: "NONE", 1: "PRIMARY", 2: "BACKUP", 9: "WATCHDOG"}
                    src = srcs.get(d['active_source'], f"?({d['active_source']})")
                    print(f"  MCU 状态 → {sn}  source={src}  fault={d['fault_level']}")
            except:
                pass

        time.sleep(0.010)

    # 如果还没到 ACTIVE，再给点时间
    if mcu_ack < 4 or mcu_state < 2:
        print(f"\n[阶段2] 等待授权... 当前 ack={ACK_NAMES[mcu_ack] if mcu_ack < len(ACK_NAMES) else mcu_ack}, state={STATE_NAMES[mcu_state] if mcu_state < len(STATE_NAMES) else mcu_state}")
        for tick in range(400):  # 再试 4 秒
            now_ms = tick * 10

            # 持续发 ANNOUNCE（幂等，MCU 会忽略重复）
            if tick % 3 == 0:
                sid, sdata = build_session_req(HIL_REQ_ANNOUNCE, session_id, session_seq)
                bus.send(can.Message(arbitration_id=sid, data=sdata, is_extended_id=False))
                session_seq = (session_seq + 1) & 0xFF

            for cid, data in enc.build_all(base_cmd):
                bus.send(can.Message(arbitration_id=cid, data=data, is_extended_id=False))

            msg = bus.recv(timeout=0.001)
            if msg:
                aid = msg.arbitration_id
                if aid == CANID_SESSION_ACK:
                    try:
                        d = decode_session_ack(msg.data)
                        if d["crc_ok"] and d["ack"] != mcu_ack:
                            mcu_ack = d["ack"]
                            an = ACK_NAMES[d["ack"]] if d["ack"] < len(ACK_NAMES) else f"?({d['ack']})"
                            print(f"  Session ACK → {an}")
                    except:
                        pass
                elif aid == CANID_MCU_HEARTBEAT:
                    try:
                        d = decode_mcu_frame(aid, msg.data)
                        if d['state'] != mcu_state:
                            mcu_state = d['state']
                            sn = STATE_NAMES[mcu_state] if mcu_state < len(STATE_NAMES) else f"?({mcu_state})"
                            print(f"  MCU 状态 → {sn}")
                    except:
                        pass

            time.sleep(0.010)

    if mcu_ack >= 4 and mcu_state >= 2:
        print(f"\n✅ MCU 已进入 ACTIVE！开始舵机扫角\n")
    else:
        print(f"\n⚠️  MCU 未进入 ACTIVE (ack={mcu_ack}, state={mcu_state})，尝试继续发控制帧...\n")

    # ---- 阶段3：舵机扫角 ----
    sweep_angles = [
        (0.0, "居中"),
        (15.0, "右转 15°"),
        (0.0, "回中"),
        (-15.0, "左转 15°"),
        (0.0, "回中"),
        (25.0, "右转 25°（大角度）"),
        (-25.0, "左转 25°（大角度）"),
        (0.0, "回中"),
        (10.0, "右转 10°"),
        (-10.0, "左转 10°"),
        (0.0, "回中"),
        (5.5, "右转 5.5°（转向灯应点亮）"),
        (2.0, "回中（转向灯应熄灭）"),
        (0.0, "最终居中"),
    ]

    try:
        for angle_deg, label in sweep_angles:
            print(f">>> {label}  ({angle_deg:+.1f}°)")
            cmd = AdasCommand(
                target_steer_deg=angle_deg,
                lateral_enable=True, lka_active=True,
                target_accel_ms2=0.0, target_speed_ms=0.0,
                longitudinal_enable=True, acc_active=True,
                drive_dir=DIR_DRIVE,
                status_word=(ST_CONTROL_ENABLE | ST_LATERAL_ENABLE
                             | ST_LONGITUDINAL_ENABLE | ST_LKA_ACTIVE
                             | ST_ACC_ACTIVE | ST_PERCEPTION_VALID
                             | ST_LOCALIZATION_VALID | ST_PLANNING_VALID),
                system_mode=SYS_MODE_ACTIVE,
            )

            for tick in range(150):  # 1.5s
                for cid, data in enc.build_all(cmd):
                    bus.send(can.Message(arbitration_id=cid, data=data,
                                         is_extended_id=False))
                time.sleep(0.010)

                # 每 50ms 收一次
                if tick % 5 == 0:
                    msg = bus.recv(timeout=0.001)
                    if msg and msg.arbitration_id == CANID_MCU_CONTROL:
                        try:
                            d = decode_mcu_frame(msg.arbitration_id, msg.data)
                            print(f"  [MCU] steer={d['steer_deg']:+.2f}°"
                                  f"  throttle={d['throttle_pct']}%"
                                  f"  brake={d['brake_pct']}%")
                        except ValueError:
                            pass
                    elif msg and msg.arbitration_id == CANID_MCU_HEARTBEAT:
                        try:
                            d = decode_mcu_frame(msg.arbitration_id, msg.data)
                            s = d['state']
                            sn = STATE_NAMES[s] if s < len(STATE_NAMES) else f"?({s})"
                            if s != mcu_state:
                                mcu_state = s
                                print(f"  [MCU] 状态变化 → {sn}")
                        except:
                            pass

    except KeyboardInterrupt:
        print("\n\n用户中断")

    print("\n[复位] 舵机回中...")
    cmd_center = AdasCommand(
        target_steer_deg=0.0,
        lateral_enable=True, lka_active=True,
        longitudinal_enable=True,
        system_mode=SYS_MODE_ACTIVE,
        status_word=(ST_CONTROL_ENABLE | ST_LATERAL_ENABLE | ST_LONGITUDINAL_ENABLE
                     | ST_PERCEPTION_VALID | ST_LOCALIZATION_VALID | ST_PLANNING_VALID),
    )
    for _ in range(30):
        for cid, data in enc.build_all(cmd_center):
            bus.send(can.Message(arbitration_id=cid, data=data, is_extended_id=False))
        time.sleep(0.010)
    print("完成。")


def listen_only(bus):
    """只收 MCU 回传帧"""
    print("收听 MCU 回传帧（Ctrl+C 退出）...")
    try:
        while True:
            msg = bus.recv(timeout=1)
            if msg:
                aid = msg.arbitration_id
                if aid == CANID_SESSION_ACK:
                    d = decode_session_ack(msg.data)
                    an = ACK_NAMES[d["ack"]] if d["ack"] < len(ACK_NAMES) else f"?({d['ack']})"
                    print(f"0x{aid:03X}  ACK={an}  id=0x{d['session_id']:08X}  seq={d['sequence']}  CRC_OK={d['crc_ok']}")
                elif aid in (CANID_MCU_CONTROL, CANID_MCU_HEARTBEAT,
                             CANID_MCU_DIAG, CANID_MCU_E2E_DIAG):
                    try:
                        d = decode_mcu_frame(aid, msg.data)
                        print(f"0x{aid:03X}  {d}")
                    except ValueError as e:
                        print(f"0x{aid:03X}  [CRC 错] {e}")
                else:
                    print(f"0x{aid:03X}  {' '.join(f'{b:02X}' for b in msg.data)}")
    except KeyboardInterrupt:
        print("停止收听。")


def main():
    parser = argparse.ArgumentParser(description="MCU 舵机扫角测试")
    parser.add_argument("--ch", type=int, default=CAN_CHANNEL,
                        help=f"CANalyst-II 通道 (默认 {CAN_CHANNEL})")
    parser.add_argument("--list", action="store_true",
                        help="仅收不发送（收听模式）")
    args = parser.parse_args()

    bus = can.Bus(interface="canalystii", channel=args.ch,
                  bitrate=CAN_BITRATE, receive_own_messages=False)

    if args.list:
        listen_only(bus)
    else:
        servo_sweep(bus)

    bus.shutdown()


if __name__ == "__main__":
    main()
