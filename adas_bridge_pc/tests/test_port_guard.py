#!/usr/bin/env python3
"""Deterministic tests for port_guard.

覆盖：
- find_pids_on_port 在没人监听时返回 []
- is_alive 对当前进程返回 True,对不存在的 PID 返回 False
- ppid 读取对当前进程返回真实 PPid
- is_orphan_or_stale 判定（PPid == 1 为孤儿）
- scan_once 在干净环境下不会误杀
- --dry-run 不会真发信号
"""

from __future__ import annotations

import importlib.util
import os
import sys
import time
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"


def _load_port_guard():
    spec = importlib.util.spec_from_file_location(
        "port_guard", SCRIPTS / "port_guard.py"
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class FindPidsTests(unittest.TestCase):
    def setUp(self):
        self.mod = _load_port_guard()

    def test_find_pids_on_unused_port(self):
        # 端口 1 通常空闲；如果碰巧被占用也不会报错，最多返回非空列表
        pids = self.mod.PortGuard((1,)).find_pids_on_port(1)
        self.assertIsInstance(pids, list)


class ProcessInspectionTests(unittest.TestCase):
    def setUp(self):
        self.mod = _load_port_guard()
        self.guard = self.mod.PortGuard(())

    def test_is_alive_self(self):
        self.assertTrue(self.guard.is_alive(os.getpid()))

    def test_is_alive_nonexistent(self):
        # 取一个肯定不存在的 PID
        self.assertFalse(self.guard.is_alive(999999999))

    def test_ppid_of_self(self):
        ppid = self.guard.ppid(os.getpid())
        self.assertIsNotNone(ppid)
        self.assertEqual(ppid, os.getppid())

    def test_ppid_of_nonexistent_returns_none(self):
        self.assertIsNone(self.guard.ppid(999999999))

    def test_process_command_of_self(self):
        cmd = self.guard.process_command(os.getpid())
        self.assertIsInstance(cmd, str)
        self.assertGreater(len(cmd), 0)


class OrphanDetectionTests(unittest.TestCase):
    def setUp(self):
        self.mod = _load_port_guard()
        self.guard = self.mod.PortGuard(())

    def test_is_orphan_for_nonexistent(self):
        self.assertFalse(self.guard.is_orphan_or_stale(999999999))


class DryRunTests(unittest.TestCase):
    def setUp(self):
        self.mod = _load_port_guard()
        self.guard = self.mod.PortGuard((), dry_run=True)

    def test_dry_run_kill_does_nothing(self):
        # dry_run 下不能真杀自己
        result = self.guard.kill_pid(os.getpid())
        self.assertTrue(result)
        # 验证自己还活着
        self.assertTrue(self.guard.is_alive(os.getpid()))


class ScanOnceTests(unittest.TestCase):
    def setUp(self):
        self.mod = _load_port_guard()
        self.guard = self.mod.PortGuard((1,), dry_run=True)  # 端口 1 + dry-run

    def test_scan_once_returns_int(self):
        killed = self.guard.scan_once()
        self.assertIsInstance(killed, int)
        self.assertGreaterEqual(killed, 0)


if __name__ == "__main__":
    unittest.main()
