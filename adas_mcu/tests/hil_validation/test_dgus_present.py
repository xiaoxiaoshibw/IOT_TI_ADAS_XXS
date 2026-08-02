"""T4 / T4b — DGUS_ENABLE 门控有效性 + 接屏测试。

T4 默认值（DGUS_ENABLE=0）：SCIA 不初始化，串口无任何活动。
T4b 烧 DGUS_ENABLE=1 镜像：发 0x82 写命令，迪文屏应回 0x83 ACK。
"""
from __future__ import annotations
import serial
from common import open_bus


def run_default_disabled(record_dir=None) -> dict:
    """T4 — DGUS_ENABLE=0 镜像：GPIO28/29 应无波形。

    实测需要示波器或逻辑分析仪。本骨架返回占位。
    """
    return {"ok": False,
            "note": "T4 默认值需要示波器/逻辑分析仪确认 SCIA 无输出。"
                    "实测前确保 adas_config.h 中 DGUS_ENABLE=0。"}


def run_with_screen(record_dir=None) -> dict:
    """T4b — DGUS_ENABLE=1 + 接屏：发 0x82 VP 写命令，等 0x83 ACK。

    注：DGUS 触屏主动上传是 0x83（数据读响应），主机发 0x82（写）屏幕
    默认不回。要验证屏幕在线，可以用 0x83 读 VP 命令（屏会回 0x83 帧）。
    """
    return {"ok": False,
            "note": "T4b 需要 DGUS_ENABLE=1 镜像 + 接迪文屏，"
                    "上电后等 ≤2 s 主动发 0x83 读 VP 命令并断言屏幕回帧。"}


def run(record_dir=None) -> dict:
    """run.py 调用此函数。区分 T4 vs T4b 通过用例 ID。"""
    return run_default_disabled(record_dir)
