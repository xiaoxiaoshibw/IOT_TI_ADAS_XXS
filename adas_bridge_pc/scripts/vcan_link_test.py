#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vcan0 上的 CAN v3 链路实测：
反馈方向（MCU→SoC/PC）：真实 SocketCAN 发送模拟 MCU 帧，用桥的
SocketCanReceiver + McuFeedbackGuard 认证，验证四个阶段：
  1) 心跳+E2E+3 帧控制 → 解锁，执行量透传
  2) 注入 CRC 坏帧 → 立即重新闭锁（fail-closed brake=1.0）
  3) 恢复 3 帧有效控制 → 再次解锁
  4) 停发 → 超时 fail-closed（stale + brake=1.0）

用法（需先建虚拟总线，无需任何 CAN 硬件）：
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up
    python3 scripts/vcan_link_test.py     # 退出码 0 = 全部通过
真实硬件在位时把 IFACE 改成本机接口名即可复用（PC PEAK 与 Jetson 板载均为 can0）。
"""
import socket
import struct
import sys
import time

import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'carla_ros2_bridge', 'ws', 'src', 'adas_carla_bridge'))
from adas_carla_bridge.can_protocol import (   # noqa: E402
    CANID_MCU_CONTROL, CANID_MCU_E2E_DIAG, CANID_MCU_HEARTBEAT,
    PROTOCOL_VERSION, SocketCanReceiver, frame_crc,
)

CAN_FRAME = struct.Struct('=IB3x8s')
IFACE = 'vcan0'


def make(can_id, values):
    data = bytearray(values)
    data[7] = frame_crc(can_id, data)
    return bytes(data)


def control_frame(seq, steer_centideg=300, brake_pct=0, throttle_pct=40):
    payload = bytearray(8)
    struct.pack_into('<hh', payload, 0, steer_centideg, -1200)
    payload[4] = throttle_pct if not brake_pct else 0
    payload[5] = brake_pct
    payload[6] = seq & 0xFF
    return make(CANID_MCU_CONTROL, payload)


def main():
    tx = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    tx.bind((IFACE,))
    rx = SocketCanReceiver(IFACE, feedback_timeout_s=0.1)

    def send(can_id, data):
        tx.send(CAN_FRAME.pack(can_id, 8, data))

    def send_health():
        send(CANID_MCU_HEARTBEAT, make(CANID_MCU_HEARTBEAT,
                                       [2, 1, 0, 0, 0, 1, 10, 0]))
        send(CANID_MCU_E2E_DIAG, make(CANID_MCU_E2E_DIAG,
                                      [0, 0, 7, 0, 0, 0, PROTOCOL_VERSION, 0]))

    failures = []

    def check(name, condition, snapshot):
        status = 'PASS' if condition else 'FAIL'
        if not condition:
            failures.append(name)
        print('[%s] %s  brake=%.2f throttle=%.2f steer=%.3f stale=%s '
              'latched=%s age=%.3fs' % (
                  status, name, snapshot['brake'], snapshot['throttle'],
                  snapshot['steer'], snapshot['stale'],
                  snapshot['invalid_latched'], snapshot['age_s']))

    seq = 0
    # 阶段 0：初始必须 fail-closed
    snap = rx.current()
    check('初始闭锁 brake=1.0', snap['invalid_latched'] and snap['brake'] == 1.0,
          snap)

    # 阶段 1：健康帧 + 3 帧控制 → 解锁
    send_health()
    for _ in range(3):
        seq += 1
        send(CANID_MCU_CONTROL, control_frame(seq))
        time.sleep(0.02)
    time.sleep(0.05)
    snap = rx.current()
    check('3帧后解锁, 油门40%透传',
          not snap['invalid_latched'] and abs(snap['throttle'] - 0.40) < 1e-6
          and abs(snap['steer'] - 300 * 0.01 / 30.0) < 1e-6, snap)

    # 阶段 2：CRC 坏帧 → 立即闭锁
    bad = bytearray(control_frame(seq + 1))
    bad[7] ^= 0xFF
    send(CANID_MCU_CONTROL, bytes(bad))
    time.sleep(0.05)
    snap = rx.current()
    check('CRC坏帧重新闭锁 brake=1.0',
          snap['invalid_latched'] and snap['brake'] == 1.0, snap)

    # 阶段 3：再喂健康帧 + 3 帧有效控制 → 恢复
    send_health()
    for _ in range(3):
        seq += 1
        send(CANID_MCU_CONTROL, control_frame(seq, brake_pct=25, throttle_pct=0))
        time.sleep(0.02)
    time.sleep(0.05)
    snap = rx.current()
    check('恢复3帧后再解锁, 制动25%透传',
          not snap['invalid_latched'] and abs(snap['brake'] - 0.25) < 1e-6, snap)

    # 阶段 4：停发 0.3 s → 超时 fail-closed
    time.sleep(0.3)
    snap = rx.current()
    check('断流超时 fail-closed', snap['stale'] and snap['brake'] == 1.0, snap)

    rx.close()
    tx.close()
    print()
    if failures:
        print('结果：%d 项失败: %s' % (len(failures), ', '.join(failures)))
        return 1
    print('结果：全部通过（SocketCAN 实链路 + v3 认证 + fail-closed 行为）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
