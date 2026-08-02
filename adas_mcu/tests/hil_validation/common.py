"""Common helpers for HIL validation scripts.

- `OpenBus()` returns a configured python-can Bus.
- `WaitForId()` polls the bus for a frame with given CAN ID until timeout.
- `decode_mcu_frame()` mirrors the MCU-side byte layout (see
  include/adas_can_protocol.h).
"""
from __future__ import annotations
import can
import sys
import time
from pathlib import Path

# Bring in the protocol constants from the reference encoder so we don't
# duplicate byte layouts.
REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "ADAS0.0.2" / "MCU" / "tools"))
from orin_can_reference import (  # noqa: E402
    CANID_MCU_CONTROL, CANID_MCU_HEARTBEAT, CANID_MCU_DIAG,
    CANID_MCU_E2E_DIAG, CANID_MCU_FAULT_RESPONSE,
    decode_mcu_frame,
    SCALE_STEER_DEG, SCALE_ACCEL_MS2,
)

# orin_can_reference 暂未定义 HIL 测试用的协议细节常量；这里本地定义，
# 数值必须与 include/adas_can_protocol.h 严格对齐。
SYS_MODE_INIT              = 0
SYS_MODE_STANDBY           = 1
SYS_MODE_ACTIVE            = 2
SYS_MODE_DEGRADED          = 3
SYS_MODE_MRM               = 4
SYS_MODE_EMERGENCY_BRAKE   = 5
SYS_MODE_FAILSAFE          = 6
SYS_MODE_FAULT_LOCK         = 7

INJ_CMD_CLEAR             = 0
INJ_CMD_FORCE_DEGRADE      = 1
INJ_CMD_FORCE_ESTOP        = 2
INJ_CMD_DROP_PRIMARY       = 3
INJ_CMD_DROP_BACKUP        = 4
INJ_CMD_FORCE_LOCK         = 5
INJ_CMD_DROP_ALL           = 6
INJ_CMD_FORCE_MRM          = 7
INJ_CMD_CAN_EXHAUSTED      = 8
INJ_CMD_SELF_TEST_FAIL     = 9

INJ_RESP_B_CMD            = 0
INJ_RESP_B_PARAM          = 1
INJ_RESP_B_STATE          = 2
INJ_RESP_B_FAULT_LEVEL    = 3
INJ_RESP_B_ALIVE          = 4
INJ_RESP_B_SEQ            = 5
INJ_RESP_B_FAULT_LO       = 6
INJ_RESP_B_CRC            = 7


def open_bus(channel: int = 1) -> can.Bus:
    """按 OS 选择 driver：Windows 默认 PCAN，Linux 用 canalystii。

    HIL 实测两种环境都常见；调用方也可显式传入 `--windows` 或 `--linux`。"""
    import platform
    if platform.system() == "Windows":
        return can.Bus(interface="pcan", channel=f"PCAN_USBBUS{channel}",
                       bitrate=500_000)
    return can.Bus(interface="canalystii", channel=channel - 1,
                   bitrate=500_000)


def wait_for_id(bus: can.Bus, can_id: int, timeout_s: float,
                min_count: int = 1) -> list:
    """滚动等待直到收到 `min_count` 帧 ID == can_id，或超时。"""
    deadline = time.time() + timeout_s
    captured = []
    while time.time() < deadline and len(captured) < min_count:
        msg = bus.recv(timeout=max(0.0, deadline - time.time()))
        if msg is None:
            continue
        if msg.arbitration_id == can_id:
            captured.append(msg)
    return captured


def send_inject(bus: can.Bus, cmd: int, param: int = 0) -> None:
    """发 0x301 故障注入命令（含 CRC）。依赖 orin_can_reference 编码。"""
    from orin_can_reference import _finish, CANID_FAULT_INJECT
    data = bytearray(8)
    data[0] = cmd
    data[1] = param
    data[5] = 0  # seq placeholder
    protected = bytearray(_finish(CANID_FAULT_INJECT, data))
    bus.send(can.Message(arbitration_id=CANID_FAULT_INJECT,
                         data=protected, is_extended_id=False))


def decode_control(msg: can.Message) -> dict:
    return decode_mcu_frame(CANID_MCU_CONTROL,
                            bytes(msg.data))
