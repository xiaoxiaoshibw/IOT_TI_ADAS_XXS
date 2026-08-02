"""T2 — 0x302 故障注入响应闭环。

PC 灌 `0x301 cmd=DROP_ALL, param=0`；MCU 收到后回送 `0x302`：
  byte0 = cmd (== DROP_ALL)
  byte1 = param
  byte2 = system_state (FAILSAFE)
  byte5 = seq (≥1)
"""
from __future__ import annotations
import time
from common import open_bus, send_inject, wait_for_id, \
    CANID_MCU_FAULT_RESPONSE, INJ_CMD_DROP_ALL, INJ_RESP_B_SEQ, SYS_MODE_FAILSAFE


def run(record_dir=None) -> dict:
    bus = open_bus(channel=1)
    try:
        send_inject(bus, INJ_CMD_DROP_ALL, param=0)
        frames = wait_for_id(bus, CANID_MCU_FAULT_RESPONSE,
                             timeout_s=1.0, min_count=1)
        if not frames:
            return {"ok": False, "note": "no 0x302 reply within 1s"}
        f = frames[-1].data
        checks = {
            "cmd_echo": int(f[0]) == INJ_CMD_DROP_ALL,
            "seq_positive": int(f[INJ_RESP_B_SEQ]) >= 1,
            "state_fail": int(f[2]) == SYS_MODE_FAILSAFE,
        }
        ok = all(checks.values())
        return {"ok": ok, "checks": checks,
                "raw": [int(b) for b in f]}
    finally:
        bus.shutdown()
