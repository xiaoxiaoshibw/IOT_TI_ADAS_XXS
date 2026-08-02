#!/usr/bin/env python3
"""CARLA RPC 看门狗：1s 周期探测，>3s 无响应触发自动恢复。

固化的是今天联调 session 里人工做过的那套恢复动作：桥接崩溃可能把 CARLA
的 synchronous_mode 卡死在半初始化状态，此后连端口都还开着但 RPC 层
（get_world()）永久挂起。恢复步骤：停桥接 → 停 CARLA（进程组信号，
CarlaUE4.sh 会派生真正的 UE4 子进程，只杀壳脚本没用）→ 用 start_carla.sh
重新拉起 → 轮询到真正 ready（不是端口监听就算数）→ 用 start_bridge.sh
（继承 common.sh 的 CycloneDDS 环境）重新拉起桥接。

用法：
    python3 carla_watchdog.py --session-dir logs/hil_run_XXXXXXXX_XXXXXX
                               [--pid-dir logs/hil_run_XXXXXXXX_XXXXXX]
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import hil_common as hc  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
PROBE_PERIOD_S = 1.0
TIMEOUT_TO_FAULT_S = 3.0
RESTART_TIMEOUT_S = 90.0


def read_pid(pid_file: Path) -> int | None:
    try:
        return int(pid_file.read_text().strip())
    except (OSError, ValueError):
        return None


def stop_process_group(pid_file: Path, label: str, log) -> None:
    pid = read_pid(pid_file)
    if pid is None:
        log(f"{label}: 无 PID 文件或内容非法，跳过停止")
        return
    try:
        pgid = os.getpgid(pid)
    except ProcessLookupError:
        log(f"{label}: PID {pid} 已不存在")
        return
    log(f"{label}: SIGTERM 进程组 {pgid}")
    try:
        os.killpg(pgid, signal.SIGTERM)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        try:
            os.killpg(pgid, 0)
        except ProcessLookupError:
            return
        time.sleep(0.2)
    log(f"{label}: 5s 超时未退出，SIGKILL 进程组 {pgid}")
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def spawn_detached(cmd: list[str], log_path: Path, pid_path: Path) -> int:
    log_f = open(log_path, "a", buffering=1)
    proc = subprocess.Popen(
        cmd, stdout=log_f, stderr=subprocess.STDOUT, cwd=REPO_ROOT,
        start_new_session=True,
    )
    pid_path.write_text(str(proc.pid))
    return proc.pid


def recover(session_dir: Path, carla_host: str, carla_port: int, log) -> bool:
    log("恢复流程开始：CARLA_RPC_TIMEOUT")
    stop_process_group(session_dir / "bridge.pid", "bridge", log)
    stop_process_group(session_dir / "carla.pid", "carla", log)

    log("重新拉起 CARLA (start_carla.sh)")
    carla_pid = spawn_detached(
        [str(REPO_ROOT / "start_carla.sh")], session_dir / "carla.log", session_dir / "carla.pid"
    )
    log(f"CARLA pid={carla_pid}，等待 RPC 真正就绪...")

    deadline = time.monotonic() + RESTART_TIMEOUT_S
    ready = False
    while time.monotonic() < deadline:
        if hc.carla_probe(carla_host, carla_port, timeout_s=3.0):
            ready = True
            break
        time.sleep(1.0)
    if not ready:
        log(f"CARLA 在 {RESTART_TIMEOUT_S}s 内未恢复 RPC 响应，恢复失败")
        return False
    log("CARLA RPC 已恢复响应")

    log("重新拉起桥接 (start_bridge.sh)")
    bridge_pid = spawn_detached(
        [str(REPO_ROOT / "start_bridge.sh")], session_dir / "bridge.log", session_dir / "bridge.pid"
    )
    log(f"桥接 pid={bridge_pid} 已重新拉起")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-dir", type=Path, required=True)
    parser.add_argument("--carla-host", default="127.0.0.1")
    parser.add_argument("--carla-port", type=int, default=2000)
    args = parser.parse_args()

    session_dir = args.session_dir
    session_dir.mkdir(parents=True, exist_ok=True)
    watchdog_log = open(session_dir / "carla_watchdog.log", "a", buffering=1)

    def log(msg: str) -> None:
        line = f"[{time.strftime('%Y-%m-%dT%H:%M:%S')}] {msg}"
        print(line, flush=True)
        watchdog_log.write(line + "\n")

    fault_codes = hc.load_fault_codes()
    fail_since: float | None = None
    faulted = False

    log(f"carla_watchdog 启动，探测 {args.carla_host}:{args.carla_port}，周期 {PROBE_PERIOD_S}s")
    while True:
        alive = hc.carla_probe(args.carla_host, args.carla_port, timeout_s=2.0)
        now = time.monotonic()
        if alive:
            if faulted:
                log("CARLA RPC 恢复正常")
                hc.clear_fault("CARLA_RPC_TIMEOUT", session_dir)
            fail_since = None
            faulted = False
        else:
            if fail_since is None:
                fail_since = now
            elapsed = now - fail_since
            if elapsed >= TIMEOUT_TO_FAULT_S and not faulted:
                faulted = True
                fc = fault_codes[100]
                ev = hc.FaultEvent(
                    code=100, name=fc["name"], level=fc["level"], source="carla_watchdog",
                    detail=f"RPC 无响应已持续 {elapsed:.1f}s", category=fc.get("category"),
                )
                hc.emit_fault(ev, session_dir)
                ok = recover(session_dir, args.carla_host, args.carla_port, log)
                if ok:
                    faulted = False
                    fail_since = None
                    hc.clear_fault("CARLA_RPC_TIMEOUT", session_dir)
                else:
                    log("恢复失败，等待下一轮探测重试")
                    fail_since = time.monotonic()  # 避免恢复失败后立刻重复触发恢复序列
        time.sleep(PROBE_PERIOD_S)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
