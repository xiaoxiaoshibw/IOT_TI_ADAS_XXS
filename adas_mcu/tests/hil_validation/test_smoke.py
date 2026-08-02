"""HIL validation smoke test.

打开总线，等 ≤2 s 看到 0x201/0x202 中任一帧则视为 MCU 在跑。
"""
from __future__ import annotations
from common import open_bus, wait_for_id, CANID_MCU_CONTROL, CANID_MCU_HEARTBEAT


def run(record_dir=None) -> dict:
    bus = open_bus(channel=1)
    try:
        for can_id in (CANID_MCU_HEARTBEAT, CANID_MCU_CONTROL):
            frames = wait_for_id(bus, can_id, timeout_s=2.0, min_count=1)
            if frames:
                return {"ok": True, "saw_can_id": hex(can_id),
                        "first_data": [int(b) for b in frames[0].data]}
        return {"ok": False,
                "note": "no 0x201 nor 0x202 within 2s — check wiring/power/CAN rate"}
    finally:
        bus.shutdown()
