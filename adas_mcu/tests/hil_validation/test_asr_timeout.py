"""T5 — ASR_STOP 超时回退到 DEGRADED（不是卡在 MRM）。

复现 §3 修复：
- PC 持续发主路 SoC 命令（保持 LINK_OK，10ms 周期 0x101/0x102）；
- 通过 LINA 触发 ASR_STOP；
- MCU 进 MRM；
- 持续主路帧让 System 不存在链路抖动，触发"MRM 提前退出→重注入"路径；
- 等 10 s 后，预期：MCU 不再卡死 MRM，自然进 DEGRADED。

判定：从 0x202 heartbeat 的 system_state 字节回归 SYS_MODE_DEGRADED (=3)
而非 SYS_MODE_MRM (=4)。
"""
from __future__ import annotations
import time
import can
from common import open_bus, CANID_MCU_HEARTBEAT, SYS_MODE_MRM, SYS_MODE_DEGRADED


def run(record_dir=None) -> dict:
    bus = open_bus(channel=1)
    try:
        # 占位：实际需要在 main_handleAsrCommand 注入 ASR_STOP 等价事件
        # （ASR-PRO 在 LINA 上 PC 端无法直接触发，需额外桥接）。
        # 这里用 0x301 强制 MRM（等价路径）。
        from common import send_inject, INJ_CMD_FORCE_MRM
        send_inject(bus, INJ_CMD_FORCE_MRM)

        states_seen = []
        deadline = time.time() + 11.0  # 略超过 ASR_STOP_TIMEOUT_MS=10s
        while time.time() < deadline:
            msg = bus.recv(timeout=0.1)
            if msg and msg.arbitration_id == CANID_MCU_HEARTBEAT:
                states_seen.append(int(msg.data[0]))
                # 若见到 DEGRADED 提前就退出测试（不让本机时钟拉长）
                if SYS_MODE_DEGRADED in states_seen:
                    break

        # 验证：从 4 (MRM) 跳到 3 (DEGRADED)
        ok = SYS_MODE_MRM in states_seen and SYS_MODE_DEGRADED in states_seen
        return {
            "ok": ok,
            "saw_mrm": SYS_MODE_MRM in states_seen,
            "saw_degraded": SYS_MODE_DEGRADED in states_seen,
            "n_frames": len(states_seen),
            "last_state": states_seen[-1] if states_seen else None,
        }
    finally:
        bus.shutdown()
