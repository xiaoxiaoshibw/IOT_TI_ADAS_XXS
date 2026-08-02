"""T3 — OLED 行 7 残留像素（§4 修复的板上验证）。

需要 OLED 拍照或 SSD1306 framebuffer dump 工具。
本骨架给出"先触发 ASR 停车，再读 0x204 中 'INPUT:...' 间接判定" 的占位实现
—— 真正的像素比对需要给 OLED 加摄像头+ 图像 diff。

更直接的实操路径：在 DGUS_ENABLE=1 + 接屏的状态下用 grep 看
返回的 ASCII 字符串内是否仍含 "AUTH" 而非干净的 "ASR:PARKING"。
"""
from __future__ import annotations
from common import open_bus, send_inject, INJ_CMD_CLEAR


def run(record_dir=None) -> dict:
    bus = open_bus(channel=1)
    try:
        # 复位确保进入 STANDBY/INIT
        send_inject(bus, INJ_CMD_CLEAR)
        # 这里需要 OLED dump 才能判定 — 留给 HIL 接线扩展。
        return {"ok": False,
                "note": "OLED 像素残留需 OLED 帧捕获工具（USB 摄像头或 SSD1306 "
                        "framebuffer dump）。本骨架给出占位返回。"}
    finally:
        bus.shutdown()
