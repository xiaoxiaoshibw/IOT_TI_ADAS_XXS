#!/usr/bin/env python3
"""Deterministic tests for carla_invoker.

覆盖：
- CarlaLock 互斥语义（两个实例不能同时持有）
- 直连模式 status（无 CARLA 时返回 stopped）
- 直连模式 stop 在无 CARLA 时返回 already_stopped
- start 在 CARLA 二进制不存在时返回 error
- supervisor 不可达时自动降级直连
- parse_args 各 action 的基本可用性
"""

from __future__ import annotations

import importlib.util
import json
import os
import socket
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"


def _load_invoker():
    """通过 importlib 加载 carla_invoker，避免污染 sys.modules。"""
    spec = importlib.util.spec_from_file_location(
        "carla_invoker", SCRIPTS / "carla_invoker.py"
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class CarlaLockTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.lock_path = Path(self.tmp.name) / "test.lock"
        # 把 invoker 模块里的 CARLA_LOCK_PATH 改到临时位置
        self._original = None
        self.mod = _load_invoker()
        self._original = self.mod.CARLA_LOCK_PATH
        self.mod.CARLA_LOCK_PATH = self.lock_path

    def tearDown(self):
        if self._original is not None:
            self.mod.CARLA_LOCK_PATH = self._original
        self.tmp.cleanup()

    def test_first_lock_acquires_second_rejects(self):
        a = self.mod.CarlaLock(self.lock_path)
        b = self.mod.CarlaLock(self.lock_path)
        with a:
            self.assertTrue(a.acquired)
            with b:
                self.assertFalse(b.acquired)

    def test_lock_released_after_context_exit(self):
        a = self.mod.CarlaLock(self.lock_path)
        with a:
            self.assertTrue(a.acquired)
        b = self.mod.CarlaLock(self.lock_path)
        with b:
            self.assertTrue(b.acquired)

    def test_pid_recorded_in_lock_file(self):
        a = self.mod.CarlaLock(self.lock_path)
        with a:
            content = self.lock_path.read_text()
        self.assertIn(str(os.getpid()), content)


class StatusTests(unittest.TestCase):
    def setUp(self):
        self.mod = _load_invoker()

    def test_status_when_nothing_listening(self):
        # 找一个肯定空闲的端口
        ns = self.mod.build_parser().parse_args(
            ["status", "--host", "127.0.0.1", "--port", "1"]
        )
        rc = self.mod.cmd_status(ns)
        self.assertEqual(rc, 0)
        out = sys.stdout.getvalue() if hasattr(sys.stdout, "getvalue") else None
        # 直接重定向捕获
        import io
        buf = io.StringIO()
        old = sys.stdout
        sys.stdout = buf
        try:
            self.mod.cmd_status(ns)
        finally:
            sys.stdout = old
        payload = json.loads(buf.getvalue().strip())
        self.assertEqual(payload["action"], "status")
        self.assertEqual(payload["state"], "stopped")
        self.assertEqual(payload["mode"], "direct")


class StartWithoutCarlaTests(unittest.TestCase):
    """CARLA 二进制不存在时,start 应该返回非零退出码 + error 状态。"""

    def setUp(self):
        # 必须在加载模块前覆盖；DEFAULT_CARLA_BIN 在 import 时解析。
        self._original_root = os.environ.get("CARLA_ROOT")
        os.environ["CARLA_ROOT"] = "/nonexistent/carla_root_for_test"
        self.mod = _load_invoker()

    def tearDown(self):
        if self._original_root is None:
            os.environ.pop("CARLA_ROOT", None)
        else:
            os.environ["CARLA_ROOT"] = self._original_root

    def test_start_returns_error_when_binary_missing(self):
        import io
        buf = io.StringIO()
        old = sys.stdout
        sys.stdout = buf
        try:
            ns = self.mod.build_parser().parse_args(
                ["start", "--host", "127.0.0.1", "--port", "1"]
            )
            rc = self.mod.cmd_start(ns)
        finally:
            sys.stdout = old
        self.assertEqual(rc, 1)
        payload = json.loads(buf.getvalue().strip())
        self.assertEqual(payload["action"], "start")
        self.assertEqual(payload["state"], "error")


class StopWithoutCarlaTests(unittest.TestCase):
    def setUp(self):
        self.mod = _load_invoker()

    def test_stop_when_nothing_running(self):
        import io
        buf = io.StringIO()
        old = sys.stdout
        sys.stdout = buf
        try:
            ns = self.mod.build_parser().parse_args(
                ["stop", "--host", "127.0.0.1", "--port", "1"]
            )
            rc = self.mod.cmd_stop(ns)
        finally:
            sys.stdout = old
        self.assertEqual(rc, 0)
        payload = json.loads(buf.getvalue().strip())
        self.assertEqual(payload["action"], "stop")
        self.assertEqual(payload["state"], "already_stopped")


class TransitionWithoutSupervisorTests(unittest.TestCase):
    """transition 在 supervisor 不可用时应返回 error + not_supported 提示。"""

    def setUp(self):
        self.mod = _load_invoker()

    def test_transition_legacy_mode_returns_error(self):
        import io
        buf = io.StringIO()
        old = sys.stdout
        sys.stdout = buf
        try:
            ns = self.mod.build_parser().parse_args(
                ["transition", "--kind", "soft", "--legacy"]
            )
            rc = self.mod.cmd_transition(ns)
        finally:
            sys.stdout = old
        self.assertEqual(rc, 1)
        payload = json.loads(buf.getvalue().strip())
        self.assertEqual(payload["state"], "error")
        self.assertIn("supervisor", payload["error"])


class SupervisorFallbackTests(unittest.TestCase):
    """当 /tmp/adas_carla_supervisor.sock 不存在时,supervisor_request 应返回 None。"""

    def setUp(self):
        self.mod = _load_invoker()

    def test_is_supervisor_available_false(self):
        # 默认环境里我们没装 supervisor
        # 把 socket 路径改到临时空文件保证不可达
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            fake = Path(d) / "no.sock"
            original = self.mod.SUPERVISOR_SOCKET
            self.mod.SUPERVISOR_SOCKET = fake
            try:
                self.assertFalse(self.mod.is_supervisor_available())
            finally:
                self.mod.SUPERVISOR_SOCKET = original

    def test_supervisor_request_returns_none(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            fake = Path(d) / "no.sock"
            original = self.mod.SUPERVISOR_SOCKET
            self.mod.SUPERVISOR_SOCKET = fake
            try:
                self.assertIsNone(self.mod.supervisor_request({"action": "status"}))
            finally:
                self.mod.SUPERVISOR_SOCKET = original


class ParseArgsTests(unittest.TestCase):
    def setUp(self):
        self.mod = _load_invoker()

    def test_start_args(self):
        ns = self.mod.build_parser().parse_args(
            ["start", "--scenario", "lka", "--town", "Town04"]
        )
        self.assertEqual(ns.action, "start")
        self.assertEqual(ns.scenario, "lka")
        self.assertEqual(ns.town, "Town04")

    def test_transition_kind_required(self):
        with self.assertRaises(SystemExit):
            self.mod.build_parser().parse_args(["transition"])

    def test_status_default_port(self):
        ns = self.mod.build_parser().parse_args(["status"])
        self.assertEqual(ns.port, self.mod.DEFAULT_PORT)


if __name__ == "__main__":
    unittest.main()
