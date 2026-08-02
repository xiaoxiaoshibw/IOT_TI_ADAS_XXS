#!/usr/bin/env python3
"""Hil preflight integration test: spawn fake "good" SoC traffic on vcan0,
then run the preflight script and assert it returns 0 (PRECONDITION PASS).

This test requires:
  - A writable vcan0 (created via `sudo modprobe vcan && sudo ip link set up vcan0`)
  - python-can installed
  - The preflight script at tools/harness/hil_preflight_before_mcu.sh

Usage:
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan || true
    sudo ip link set up vcan0
    python3 tests/hil_preflight_vcan_integration.py
"""
from __future__ import annotations

import os
import subprocess
import sys
import threading
import time
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "scripts" / "hil_preflight_before_mcu.sh"

# Skip if vcan0 isn't present
VCAN_IF = "vcan0"


def _iface_up(iface: str) -> bool:
    """vcan 设备的 state 字段是 UNKNOWN（虚拟设备），不是 UP——但只要
    存在且 <UP,...> 链路标志带 UP 就视作可工作。"""
    out = subprocess.run(["ip", "-details", "link", "show", iface],
                         capture_output=True, text=True)
    if out.returncode != 0:
        return False
    first = out.stdout.splitlines()[0] if out.stdout else ""
    return ",UP" in first or "UP," in first


def _iface_bitrate(iface: str) -> int:
    out = subprocess.run(["ip", "-details", "link", "show", iface],
                         capture_output=True, text=True)
    for line in out.stdout.splitlines():
        if "bitrate" in line:
            for tok in line.split():
                if tok.isdigit():
                    return int(tok)
    return 0


@unittest.skipUnless(_iface_up(VCAN_IF), "需要 vcan0 已 UP")
class PreflightVcanIntegration(unittest.TestCase):
    """端到端集成测试：在 vcan0 上注入 0x100~0x103 模拟 SoC 流，
    验证 preflight 通过。"""

    @classmethod
    def setUpClass(cls):
        # 检查 bitrate（vcan 没有真实 bitrate，但 ip 输出会有占位）
        br = _iface_bitrate(VCAN_IF)
        # vcan0 不会报 bitrate，跳过严格检查

    def _spawn_traffic(self, duration_s: float, bitrate_quirk: bool = False):
        """起一个线程：把 0x100/0x101/0x102/0x103 周期性发到 vcan0
        （10ms/20ms 标称，CRC 用 0x00 占位——preflight 不做 CRC 校验）。"""
        import can

        def sender():
            try:
                bus = can.interface.Bus(interface="socketcan", channel=VCAN_IF,
                                         bitrate=500000)
            except Exception as e:
                print(f"[sender] 打开 {VCAN_IF} 失败: {e}", file=sys.stderr)
                return
            t0 = time.time()
            hb_tick = 0
            ctrl_tick = 0
            while time.time() - t0 < duration_s:
                now = time.time() - t0
                # 0x100/0x103: 20ms
                # 0x101/0x102: 10ms
                if int(now * 50) != hb_tick:  # 50Hz
                    hb_tick = int(now * 50)
                    bus.send(can.Message(arbitration_id=0x100,
                                         data=bytes(8), is_extended_id=False))
                    bus.send(can.Message(arbitration_id=0x103,
                                         data=bytes(8), is_extended_id=False))
                if int(now * 100) != ctrl_tick:  # 100Hz
                    ctrl_tick = int(now * 100)
                    bus.send(can.Message(arbitration_id=0x101,
                                         data=bytes(8), is_extended_id=False))
                    bus.send(can.Message(arbitration_id=0x102,
                                         data=bytes(8), is_extended_id=False))
                time.sleep(0.002)  # 2ms
            bus.shutdown()

        th = threading.Thread(target=sender, daemon=True)
        th.start()
        return th

    def test_preflight_passes_with_good_traffic(self):
        """vcan0 注入正常 SoC 流 → preflight 应在 ROS2 仍缺失时 fail in ROS，
        但所有 CAN 项（接口/帧/连续性）应 OK。这是 dev 环境下对 CAN 路径
        的端到端验证。"""
        th = self._spawn_traffic(duration_s=20.0)
        time.sleep(1.0)
        proc = subprocess.run(
            [str(SCRIPT),
             "--can-if", VCAN_IF,
             "--stable-seconds", "3",
             "--sample-seconds", "4",
             "--timeout-seconds", "12",
             "--allow-no-bitrate",
             "--verbose"],
            capture_output=True, text=True, timeout=30,
        )
        th.join(timeout=5)
        # ROS2 在本机未起 → 必然 fail（exit 6 timeout 或 1 ros2 not ready）
        # 但 CAN 帧连续性应全程 OK；这是 dev 验证 CAN 解析路径的关键。
        self.assertIn(proc.returncode, (1, 6),
                      f"预期 ROS2 缺失致 exit 1/6，实际 {proc.returncode}")
        # 每个采样窗都应输出 OK:cnt=... 行
        ok_lines = [ln for ln in proc.stdout.splitlines() if ln.startswith("OK:cnt=")]
        self.assertGreaterEqual(len(ok_lines), 2,
                                f"应至少 2 个连续 OK 采样窗，实际 {len(ok_lines)}\n{proc.stdout[-2000:]}")
        # 不应出现 FAIL_MISSING / FAIL_NOT_CONTINUOUS
        bad_lines = [ln for ln in proc.stdout.splitlines()
                     if "FAIL_MISSING" in ln or "FAIL_NOT_CONTINUOUS" in ln]
        self.assertEqual(bad_lines, [],
                         f"CAN 路径应全程 OK，发现连续性错误：{bad_lines}")

    def test_preflight_fails_on_missing_id(self):
        """只发 0x100 + 0x101 → preflight 应报缺失 0x102/0x103。"""
        import can

        def sender():
            bus = can.interface.Bus(interface="socketcan", channel=VCAN_IF, bitrate=500000)
            t0 = time.time()
            while time.time() - t0 < 10.0:
                now = time.time() - t0
                if int(now * 50) != getattr(sender, "hb_tick", -1):
                    sender.hb_tick = int(now * 50)
                    bus.send(can.Message(arbitration_id=0x100, data=bytes(8), is_extended_id=False))
                if int(now * 100) != getattr(sender, "ctrl_tick", -1):
                    sender.ctrl_tick = int(now * 100)
                    bus.send(can.Message(arbitration_id=0x101, data=bytes(8), is_extended_id=False))
                time.sleep(0.002)
            bus.shutdown()

        th = threading.Thread(target=sender, daemon=True)
        th.start()
        time.sleep(1.0)
        proc = subprocess.run(
            [str(SCRIPT),
             "--can-if", VCAN_IF,
             "--stable-seconds", "2",
             "--sample-seconds", "3",
             "--timeout-seconds", "8",
             "--allow-no-bitrate",
             "--verbose"],
            capture_output=True, text=True, timeout=20,
        )
        th.join(timeout=5)
        # 缺 0x102/0x103：每个采样窗的分析器会输出 FAIL_MISSING；
        # 整体仍因 ROS2 缺失走 exit 6 timeout，但 stdout 应能看到 FAIL_MISSING。
        fail_lines = [ln for ln in proc.stdout.splitlines() if "FAIL_MISSING" in ln]
        self.assertGreater(len(fail_lines), 0,
                           f"应检测到关键 ID 缺失，实际未发现\n{proc.stdout[-2000:]}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
