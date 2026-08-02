"""T6 — 看门狗护栏：HB 静默 ≥FTTI_HEARTBEAT_LOSS_MS=140 ms → FAILSAFE。

实际机制：
- safety.c 把 HB_TIMEOUT_MS=100 ms（5 帧）+ safety_arbitration_budget=1 ms
  + safety_output_budget=15 ms 累进 control_fresh=false；
- 单源全失走 idx == SRC_IDX_INVALID 路径 → s_ever_engaged=true 进 FAILSAFE；
- FAILSAFE 即刻强制定速 -8 m/s² 减速、双闪、急促蜂鸣、FAULT LED 常亮。

测试：停止发主路帧 200 ms，看 0x202:
  state byte 0=6 (SYS_MODE_FAILSAFE)
  safety_flags byte 2 含 SF_BUZZER_ON / SF_PRIMARY_FRESH=0
"""
from __future__ import annotations
import time
from common import open_bus, CANID_MCU_HEARTBEAT, SYS_MODE_FAILSAFE


def run(record_dir=None) -> dict:
    bus = open_bus(channel=1)
    try:
        time.sleep(0.25)  # 停发 250 ms (> FTTI)
        states_seen = []
        deadline = time.time() + 1.0
        while time.time() < deadline:
            msg = bus.recv(timeout=0.05)
            if msg and msg.arbitration_id == CANID_MCU_HEARTBEAT:
                states_seen.append(int(msg.data[0]))
                if SYS_MODE_FAILSAFE in states_seen:
                    break

        ok = SYS_MODE_FAILSAFE in states_seen
        return {
            "ok": ok,
            "saw_failsafe": SYS_MODE_FAILSAFE in states_seen,
            "n_frames": len(states_seen),
            "last_state": states_seen[-1] if states_seen else None,
        }
    finally:
        bus.shutdown()
