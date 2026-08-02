"""T1 — ASR-PRO 帧边界回归（MCU v0.8.0 §2 修复的板上验证）。

灌 `0x0C 0x0D 0x0E 0xED 0x0C 0x0D 0x01` 字节流验证 MCU 解析为两帧。
注意：本用例的字节流与 host 单元测试 `test_asr_parser_host.c` 中
`test_speed_query_then_stop_does_not_lose_stop` 严格对齐，确保 host
锁住的反例在 target 上同样成立。
"""
from __future__ import annotations
import time
from common import open_bus, send_inject, INJ_CMD_CLEAR


ASR_BYTE_STREAM = bytes([0x0C, 0x0D, 0x0E, 0xED, 0x0C, 0x0D, 0x01])


def run(record_dir=None) -> dict:
    bus = open_bus(channel=1)
    try:
        if record_dir:
            bus = _with_recorder(bus, record_dir / "T1.asc")

        # 复位：发 CLEAR 注入恢复 ACTIVE-ish 状态。
        send_inject(bus, INJ_CMD_CLEAR)

        # 灌两帧 ASR-PRO 命令字流。MCU 通过 LINA 读到后会触发 main.c
        # 的 ASR_CMD_QUERY_SPEED + ASR_CMD_STOP 两条命令。
        from common import can
        msg = can.Message(arbitration_id=0x000,  # ASR-PRO 不在 CAN 上，是 LINA 直连
                         data=ASR_BYTE_STREAM, is_extended_id=False)
        # 注意：本测试要求 ASR-PRO 物理上挂在 LINA 上，或用脚本把这 7 字节
        # 直接写到 LINA 模拟口（实板通过 SCIA/LINA bridge）。

        # 实测桩：这里无法在 PC 这一侧直接给 LINA 灌字节，
        # 必须通过 §"不直接覆写 main.c" 的旁路（如 SOC 发相应命令替代）。
        # 本骨架给出占位返回，供用户按实物接线补充。

        return {"ok": False,
                "note": "ASR-PRO 在 LINA 上，PC 端无法直接灌字节；"
                        "实板测试需在 main.c 临时启用'ASR 自注入测试模式'，"
                        "或额外接一条 USB-TTL 串口到 LINA 引脚。"}
    finally:
        bus.shutdown()


def _with_recorder(bus, path):
    """可选：把总线流量存成 .asc 给 cansniffer 看。实际需 can.io.AsyncWriter。"""
    return bus
