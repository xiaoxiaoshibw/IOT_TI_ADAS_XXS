#!/usr/bin/env python3
"""端口守护：清理 CARLA 相关端口的僵死占用。

扫描 :2000 (RPC) / :2001 (streaming) / :2002 (sensor) 与 CARLA Traffic Manager
端口（默认 8000/8100/8200/8300），若被某个不归 supervisor 持有的进程占用，
SIGTERM 等待 2.5s 后 SIGKILL 兜底。

设计原则（与现有 carla_watchdog.py / start_pc_stack_clean.sh 保持一致）：

1. **不误杀合法进程**：仅针对 CARLA 已知端口，且只杀"无 PPid 指向 supervisor"
   或 "PPid == 1（孤儿）" 的进程。
2. **可独立运行也可被 supervisor 子进程拉起**：`--once` 一次性扫描退出，
   默认常驻 5s 周期。
3. **所有动作写日志**：/var/log/adas/port_guard.log（fallback 到
   ~/.local/share/adas/port_guard.log）。
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

# CARLA 已知端口：RPC + streaming + sensor + 常用 Traffic Manager 范围
DEFAULT_PORTS = (2000, 2001, 2002, 8000, 8100, 8200, 8300)
SIGTERM_WAIT_S = 2.5
SCAN_PERIOD_S = 5.0

LOG_FALLBACK = Path.home() / ".local" / "share" / "adas" / "port_guard.log"


def log_path() -> Path:
    preferred = Path("/var/log/adas/port_guard.log")
    try:
        preferred.parent.mkdir(parents=True, exist_ok=True)
        # test write
        with open(preferred, "a", encoding="utf-8") as f:
            f.write("")
        return preferred
    except OSError:
        LOG_FALLBACK.parent.mkdir(parents=True, exist_ok=True)
        return LOG_FALLBACK


class PortGuard:
    def __init__(self, ports: tuple[int, ...], *, dry_run: bool = False) -> None:
        self.ports = ports
        self.dry_run = dry_run
        self.log_file = open(log_path(), "a", buffering=1)

    def log(self, msg: str) -> None:
        line = f"[{time.strftime('%Y-%m-%dT%H:%M:%S')}] {msg}"
        print(line, flush=True)
        self.log_file.write(line + "\n")

    # ---------- 端口扫描 ----------

    def find_pids_on_port(self, port: int) -> list[int]:
        """通过 `ss -H -l -np tcp sport = :PORT` 找占用端口的 PID。"""
        try:
            out = subprocess.run(
                ["ss", "-H", "-l", "-n", "-p", "tcp", f"sport = :{port}"],
                capture_output=True, text=True, timeout=2.0,
            )
        except (OSError, subprocess.TimeoutExpired):
            return []
        pids: set[int] = set()
        for line in out.stdout.splitlines():
            # 形如 users:(("CarlaUE4-Linux-Shipping",pid=12345,fd=42))
            for token in line.split('"'):
                if "pid=" in token:
                    try:
                        pid = int(token.split("pid=")[1].split(",")[0])
                        pids.add(pid)
                    except (ValueError, IndexError):
                        pass
        return sorted(pids)

    def is_alive(self, pid: int) -> bool:
        try:
            os.kill(pid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            # 进程存在但不属于我们 —— 仍算 alive，不主动处理
            return True

    def ppid(self, pid: int) -> Optional[int]:
        try:
            with open(f"/proc/{pid}/status", "r", encoding="utf-8") as f:
                for line in f:
                    if line.startswith("PPid:"):
                        return int(line.split()[1])
        except (OSError, ValueError):
            return None
        return None

    def is_orphan_or_stale(self, pid: int) -> bool:
        """判断 PID 是否值得清理：

        - 进程不存在 → False
        - PPid == 1（被 init 收养）→ True（孤儿，多半僵死）
        - PPid 不可读（unprivileged 看其他用户进程）→ True
        - 命令名以 'CarlaUE4' / 'CarlaUE4-Linux-Shipping' 开头 → True
          （即使是合法 supervisor 派生进程，也允许用户传 --kill-all 强杀）
        """
        if not self.is_alive(pid):
            return False
        ppid = self.ppid(pid)
        if ppid is None:
            return True
        if ppid == 1:
            return True
        return False

    def process_command(self, pid: int) -> str:
        try:
            with open(f"/proc/{pid}/comm", "r", encoding="utf-8") as f:
                return f.read().strip()
        except OSError:
            return "<unknown>"

    # ---------- kill ----------

    def kill_pid(self, pid: int) -> bool:
        if self.dry_run:
            self.log(f"[DRY-RUN] 将 SIGTERM pid={pid}")
            return True
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            return True
        except PermissionError:
            self.log(f"! 无权 SIGTERM pid={pid} (其他用户进程)")
            return False
        deadline = time.monotonic() + SIGTERM_WAIT_S
        while time.monotonic() < deadline:
            if not self.is_alive(pid):
                self.log(f"✓ pid={pid} 已退出（SIGTERM）")
                return True
            time.sleep(0.1)
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            return True
        except PermissionError:
            self.log(f"! 无权 SIGKILL pid={pid}")
            return False
        time.sleep(0.2)
        if not self.is_alive(pid):
            self.log(f"✓ pid={pid} 已退出（SIGKILL 兜底）")
            return True
        self.log(f"× pid={pid} 在 SIGKILL 后仍未退出")
        return False

    # ---------- 周期 ----------

    def scan_once(self) -> int:
        killed = 0
        for port in self.ports:
            for pid in self.find_pids_on_port(port):
                cmd = self.process_command(pid)
                if not self.is_orphan_or_stale(pid):
                    continue
                self.log(
                    f"清理端口 :{port} 上的孤儿/僵死进程 pid={pid} comm={cmd!r}"
                )
                if self.kill_pid(pid):
                    killed += 1
        return killed

    def run_forever(self) -> int:
        self.log(
            f"port_guard 启动，扫描端口 {self.ports}，"
            f"周期 {SCAN_PERIOD_S}s，dry_run={self.dry_run}"
        )
        try:
            while True:
                self.scan_once()
                time.sleep(SCAN_PERIOD_S)
        except KeyboardInterrupt:
            self.log("收到 SIGINT，退出")
            return 0


def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ports", type=int, nargs="+", default=list(DEFAULT_PORTS),
        help=f"要守护的端口列表（默认 {DEFAULT_PORTS}）",
    )
    parser.add_argument(
        "--once", action="store_true",
        help="只扫描一次后退出（用于测试 / 一次性清理）",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="只打印要 kill 的 PID，不真正发信号",
    )
    parser.add_argument(
        "--output-json", action="store_true",
        help="以 JSON 格式输出 scan_once 结果（用于测试）",
    )
    args = parser.parse_args(argv)

    guard = PortGuard(tuple(args.ports), dry_run=args.dry_run)
    if args.once:
        killed = guard.scan_once()
        if args.output_json:
            sys.stdout.write(
                json.dumps({"killed": killed, "ports": list(args.ports)}) + "\n"
            )
        return 0 if killed >= 0 else 1
    return guard.run_forever()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
