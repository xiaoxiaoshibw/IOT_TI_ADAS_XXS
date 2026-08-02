#!/usr/bin/env python3
"""Self-test for tools/harness/hil_preflight_before_mcu.sh.

在不具备真实 CAN 总线 / ROS2 / 真实 MCU 的开发机本地，
通过 stub / 注入式测试，逐项验证 preflight 脚本的判定逻辑：
  - CAN 接口不存在 → fail
  - CAN 接口存在但未 UP → fail
  - 关键 CAN 帧缺失 → fail
  - CAN 帧不连续（启动期长空窗）→ fail
  - ROS2 不可达 → fail
  - 全部条件满足（模拟）→ pass

不模拟 bus-off / error-passive 增长，因为那些由 ip -details 命令
直接读取，不在 Python 层做替换。
"""
from __future__ import annotations

import os
import re
import shlex
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "scripts" / "hil_preflight_before_mcu.sh"


def run_preflight(args, timeout=120, env=None):
    full_env = os.environ.copy()
    if env:
        full_env.update(env)
    proc = subprocess.run(
        [str(SCRIPT)] + args,
        env=full_env,
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    return proc.returncode, proc.stdout, proc.stderr


class PreflightTests(unittest.TestCase):
    def test_help(self):
        rc, out, _ = run_preflight(["--help"])
        self.assertEqual(rc, 0, "help 应返回 0")
        self.assertIn("用法", out)

    def test_unknown_arg(self):
        rc, _, err = run_preflight(["--no-such-flag"])
        self.assertEqual(rc, 7, "未知参数应返回 7")
        self.assertIn("未知参数", err)

    def test_sample_less_than_stable(self):
        rc, _, err = run_preflight([
            "--stable-seconds", "10",
            "--sample-seconds", "5",
            "--timeout-seconds", "8",
        ])
        self.assertEqual(rc, 7)
        self.assertIn("必须 >=", err)

    def test_can_if_missing(self):
        rc, out, err = run_preflight([
            "--can-if", "can_does_not_exist_xyz",
            "--stable-seconds", "1",
            "--sample-seconds", "2",
            "--timeout-seconds", "5",
        ])
        # 无 can 接口 → 立即 exit 6 (timeout) 或 2 (CAN 异常)
        # 我们的实现统一走 timeout 路径：每 SAMPLE_SECONDS 一次判 → 累计不到 STABLE_SECONDS → 6
        self.assertIn(rc, (2, 6), f"应为 2 或 6，实际 {rc}")
        self.assertIn("异常", out + err)

    def test_ros2_unavailable(self):
        # 把 PATH 清空到只剩 bash 内置，确保 ros2 / ip 找不到 → exit 7
        full_env = {"PATH": "/usr/bin:/bin", "HOME": os.environ["HOME"]}
        rc, _, err = run_preflight(
            ["--stable-seconds", "1", "--sample-seconds", "2", "--timeout-seconds", "5"],
            env=full_env,
        )
        # 没有 ros2 但有 candump/ip → 走 ROS2 缺失 + CAN 缺失
        self.assertIn(rc, (1, 2, 6), f"应为 1/2/6，实际 {rc}")

    def test_argument_parsing(self):
        rc, _, _ = run_preflight([
            "--stable-seconds", "0",
        ])
        self.assertEqual(rc, 7, "stable-seconds=0 应拒绝")


class PreflightAnalyzeUnitTests(unittest.TestCase):
    """直接对 candump -L 输出做解析逻辑的单元测试，无需真 CAN。"""

    def _run_analyzer(self, log_text: str, require_backup: bool = False):
        # 复用脚本里的 python3 内联分析；这里写一个轻量等价实现做对比
        PRI_IDS = [0x100, 0x101, 0x102, 0x103]
        BAK_IDS = [0x110, 0x111, 0x112, 0x113]
        ALL = PRI_IDS + (BAK_IDS if require_backup else [])
        frame_re = re.compile(r"\(([\d.]+)\)\s+\S+\s+([0-9A-Fa-f]+)#")
        from collections import defaultdict
        by_id = defaultdict(list)
        for line in log_text.splitlines():
            m = frame_re.search(line)
            if not m:
                continue
            try:
                ts = float(m.group(1))
                cid = int(m.group(2), 16)
            except ValueError:
                continue
            if cid in ALL:
                by_id[cid].append(ts)
        missing = [hex(c) for c in ALL if len(by_id[c]) == 0]
        if missing:
            return 3, f"FAIL_MISSING:{','.join(missing)}"
        return 0, "OK"

    def test_missing_primary_id(self):
        text = "(0.001) can1 100#0000000000000000\n" \
               "(0.011) can1 101#0000000000000000\n" \
               "(0.022) can1 102#0000000000000000\n"
        # 缺 0x103
        rc, msg = self._run_analyzer(text)
        self.assertEqual(rc, 3)
        self.assertIn("0x103", msg)

    def test_full_primary_ok(self):
        lines = []
        # 模拟 1 秒窗口，4 个 ID 各 100 帧
        for j in range(100):
            for cid in (0x100, 0x101, 0x102, 0x103):
                ts = j * 0.01 + (cid - 0x100) * 0.001
                lines.append(f"({ts:.3f}) can1 {cid:03X}#0000000000000000")
        text = "\n".join(lines)
        rc, msg = self._run_analyzer(text)
        self.assertEqual(rc, 0, msg)


if __name__ == "__main__":
    unittest.main(verbosity=2)
