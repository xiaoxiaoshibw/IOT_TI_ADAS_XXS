#!/usr/bin/env python3
"""统一 CARLA 调用入口。

所有 CARLA 进程启动/停止/状态查询都走这个脚本：

    carla_invoker.py start   --scenario X --town Y [--quality-level Low]
    carla_invoker.py stop
    carla_invoker.py status  (输出 JSON 给 C++ 端 QProcess::readAllStandardOutput 解析)
    carla_invoker.py restart --scenario X --town Y
    carla_invoker.py transition --kind hot|soft|hard --scenario X --town Y

两种运行模式：

1. Supervisor 模式（默认）：通过 Unix socket 与 carla-supervisor.py 通信，
   supervisor 是唯一持 /tmp/adas_carla.lock 的进程，所有并发请求串行化。

2. 直连回退模式：supervisor 不可用时（dev box / 测试环境 / supervisor 没装），
   自己持 flock 启动 CarlaUE4.sh，保证双启防护仍生效。

GUI (`adas_bridge_pc/carla_ros2_bridge/ws/src/adas_gui/src/process_manager.cpp`)、
orchestrator.py、start_pc_stack.sh 全部改走这个入口，删除各自重复的
flock/端口探测逻辑，杜绝"同时两个 CARLA"的可能。
"""

from __future__ import annotations

import argparse
import errno
import fcntl
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "adas_bridge_pc" / "scripts"))

import hil_common as hc  # noqa: E402

CARLA_LOCK_PATH = Path(os.environ.get("ADAS_CARLA_LOCK", "/tmp/adas_carla.lock"))
CARLA_SCENE_LOCK_PATH = Path(os.environ.get("ADAS_SCENE_LOCK", "/tmp/adas_scene.lock"))
SUPERVISOR_SOCKET = Path(
    os.environ.get("ADAS_CARLA_SUPERVISOR_SOCK", "/tmp/adas_carla_supervisor.sock")
)
SUPERVISOR_TIMEOUT_S = float(os.environ.get("ADAS_INVOKER_SUPERVISOR_TIMEOUT", "0.5"))

# CARLA 默认路径：与 common.sh carla_executable() 完全一致。
DEFAULT_CARLA_ROOT = os.environ.get("CARLA_ROOT") or os.path.expanduser("~/CARLA_0.9.16")
DEFAULT_CARLA_BIN = os.path.join(DEFAULT_CARLA_ROOT, "CarlaUE4.sh")

# 默认 town 与 quality —— 与 common.sh 默认值对齐。
DEFAULT_TOWN = os.environ.get("TOWN", "Town04")
DEFAULT_QUALITY = os.environ.get("CARLA_QUALITY", "Epic")
DEFAULT_PORT = int(os.environ.get("CARLA_PORT", "2000"))
DEFAULT_RPC_HOST = os.environ.get("CARLA_HOST", "127.0.0.1")


# ---------------------------------------------------------------------------
# 输出辅助
# ---------------------------------------------------------------------------


def emit(payload: dict, *, exit_code: int = 0) -> int:
    """输出 JSON 行 + 退出码。GUI 通过 stdout 解析状态。"""
    payload.setdefault("ts", time.time())
    payload.setdefault("invoker_pid", os.getpid())
    sys.stdout.write(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n")
    sys.stdout.flush()
    return exit_code


def err(action: str, message: str, **extra) -> int:
    return emit(
        {"action": action, "state": "error", "error": message, **extra},
        exit_code=1,
    )


# ---------------------------------------------------------------------------
# Supervisor 客户端
# ---------------------------------------------------------------------------


def supervisor_request(payload: dict) -> Optional[dict]:
    """向 carla-supervisor 发送请求；supervisor 不可达返回 None。"""
    if not SUPERVISOR_SOCKET.exists():
        return None
    try:
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        client.settimeout(SUPERVISOR_TIMEOUT_S)
        client.connect(str(SUPERVISOR_SOCKET))
        try:
            client.sendall((json.dumps(payload) + "\n").encode("utf-8"))
            chunks = []
            deadline = time.monotonic() + 30.0
            while time.monotonic() < deadline:
                chunk = client.recv(65536)
                if not chunk:
                    break
                chunks.append(chunk)
                if b"\n" in chunk:
                    break
            data = b"".join(chunks).decode("utf-8", errors="replace").strip()
            if not data:
                return None
            return json.loads(data.splitlines()[-1])
        finally:
            client.close()
    except (OSError, json.JSONDecodeError) as exc:
        sys.stderr.write(f"[invoker] supervisor 不可达: {exc}\n")
        return None


def is_supervisor_available() -> bool:
    if not SUPERVISOR_SOCKET.exists():
        return False
    try:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(0.2)
        s.connect(str(SUPERVISOR_SOCKET))
        s.close()
        return True
    except OSError:
        return False


# ---------------------------------------------------------------------------
# 直连模式：flock + CarlaUE4.sh
# ---------------------------------------------------------------------------


class CarlaLock:
    """非阻塞 flock；用于串行化所有 CARLA 进程操作。

    用法:
        with CarlaLock() as lock:
            if not lock.acquired:
                ... 报告 "已有进行中的操作"
            ... 执行启动/停止
    """

    def __init__(self, path: Path = CARLA_LOCK_PATH):
        self.path = path
        self.fd: Optional[int] = None
        self.acquired = False

    def __enter__(self) -> "CarlaLock":
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.fd = os.open(str(self.path), os.O_RDWR | os.O_CREAT, 0o644)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.acquired = True
        except OSError as exc:
            if exc.errno not in (errno.EWOULDBLOCK, errno.EAGAIN):
                raise
            self.acquired = False
        # 写入 PID 便于外部诊断（不覆盖原值）
        try:
            os.lseek(self.fd, 0, os.SEEK_SET)
            os.write(self.fd, f"{os.getpid()}\n".encode())
        except OSError:
            pass
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self.fd is not None:
            try:
                if self.acquired:
                    fcntl.flock(self.fd, fcntl.LOCK_UN)
            finally:
                os.close(self.fd)
                self.fd = None


def carla_already_running(host: str, port: int) -> bool:
    return hc.carla_probe(host, port, timeout_s=1.0)


def find_running_carla_pid() -> Optional[int]:
    """查找 CarlaUE4 主进程 PID（不依赖 supervisor）。"""
    try:
        out = subprocess.run(
            ["pgrep", "-f", "CarlaUE4-Linux-Shipping"],
            capture_output=True, text=True, timeout=2.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if out.returncode != 0:
        return None
    for line in out.stdout.splitlines():
        try:
            return int(line.strip())
        except ValueError:
            continue
    return None


def kill_carla_process_group(timeout_s: float = 5.0) -> bool:
    """SIGTERM 整个 CarlaUE4 进程组，超时后 SIGKILL。"""
    pid = find_running_carla_pid()
    if pid is None:
        return False
    try:
        pgid = os.getpgid(pid)
    except ProcessLookupError:
        return False
    try:
        os.killpg(pgid, signal.SIGTERM)
    except ProcessLookupError:
        return True
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            os.killpg(pgid, 0)
        except ProcessLookupError:
            return True
        time.sleep(0.1)
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    return True


def start_carla_direct(args: argparse.Namespace) -> int:
    """直连模式启动 CARLA。"""
    if not os.path.isfile(DEFAULT_CARLA_BIN) or not os.access(DEFAULT_CARLA_BIN, os.X_OK):
        return err(
            "start",
            f"CarlaUE4.sh 不存在或不可执行: {DEFAULT_CARLA_BIN}",
            carla_bin=DEFAULT_CARLA_BIN,
        )

    if carla_already_running(args.host, args.port):
        # 已运行：复用而不是起第二个。这是关键的"防双启"行为。
        return emit(
            {
                "action": "start", "state": "already_running",
                "host": args.host, "port": args.port,
                "town": args.town, "scenario": args.scenario,
                "note": "CARLA 已在运行（RPC 探活成功），复用现有实例",
            }
        )

    cmd = [
        DEFAULT_CARLA_BIN,
        "-nosound",
        f"-carla-rpc-port={args.port}",
        f"-quality-level={args.quality_level}",
    ]
    if args.render_offscreen:
        cmd.append("-RenderOffScreen")
    log_path = Path(args.log_file)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_f = open(log_path, "a", buffering=1)
    try:
        proc = subprocess.Popen(
            cmd, stdout=log_f, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except OSError as exc:
        log_f.close()
        return err("start", f"Popen 失败: {exc}", command=cmd)
    # 不在此处关闭 log_f —— 由 Popen 持有；但 CARLA 的 stdout 是非阻塞缓冲，
    # 关掉 file desc 后 Popen 仍能写。父进程退出后由 supervisor 接管日志。
    pid = proc.pid
    # 探测 readiness：与 carla_readiness.py 逻辑一致，但只校验 RPC 是否可达。
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        if hc.carla_probe(args.host, args.port, timeout_s=1.0):
            return emit(
                {
                    "action": "start", "state": "started",
                    "pid": pid, "host": args.host, "port": args.port,
                    "town": args.town, "scenario": args.scenario,
                    "command": cmd, "log": str(log_path),
                }
            )
        # Popen 已退出
        if proc.poll() is not None:
            return err(
                "start",
                f"CarlaUE4.sh 启动后立即退出 (rc={proc.returncode})",
                pid=pid, command=cmd,
            )
        time.sleep(0.5)
    return err(
        "start",
        f"CarlaUE4 在 {args.timeout}s 内未通过 RPC 探活",
        pid=pid, command=cmd,
    )


def stop_carla_direct(args: argparse.Namespace) -> int:
    """直连模式停止 CARLA。"""
    if not carla_already_running(args.host, args.port):
        return emit(
            {"action": "stop", "state": "already_stopped",
             "host": args.host, "port": args.port}
        )
    killed = kill_carla_process_group(timeout_s=args.shutdown_timeout)
    if not killed:
        return err(
            "stop",
            "未找到 CarlaUE4 进程组或停止失败",
            host=args.host, port=args.port,
        )
    return emit(
        {"action": "stop", "state": "stopped",
         "host": args.host, "port": args.port}
    )


def status_direct(args: argparse.Namespace) -> int:
    alive = carla_already_running(args.host, args.port)
    map_name = hc.carla_map_name(args.host, args.port, timeout_s=1.0)
    pid = find_running_carla_pid() if alive else None
    return emit(
        {
            "action": "status", "state": "running" if alive else "stopped",
            "host": args.host, "port": args.port,
            "map": map_name, "pid": pid,
            "expected_town": args.town, "town_matches": (
                map_name is not None and map_name.endswith("/" + args.town)
            ),
            "mode": "direct",
        }
    )


# ---------------------------------------------------------------------------
# 路由
# ---------------------------------------------------------------------------


def route_to_supervisor(action_args: argparse.Namespace) -> Optional[int]:
    """尝试把请求交给 supervisor；supervisor 不可用时返回 None。"""
    payload = {
        "action": action_args.action,
        "host": action_args.host, "port": action_args.port,
        "town": action_args.town, "scenario": action_args.scenario,
        "quality_level": action_args.quality_level,
        "render_offscreen": action_args.render_offscreen,
        "timeout": action_args.timeout,
        "shutdown_timeout": action_args.shutdown_timeout,
        "log_file": action_args.log_file,
        "kind": getattr(action_args, "kind", None),
    }
    payload = {k: v for k, v in payload.items() if v is not None}
    response = supervisor_request(payload)
    if response is None:
        return None
    if action_args.action == "status":
        response["mode"] = "supervisor"
    sys.stdout.write(json.dumps(response, ensure_ascii=False, sort_keys=True) + "\n")
    sys.stdout.flush()
    return int(response.get("exit_code", 0))


def cmd_start(args: argparse.Namespace) -> int:
    if not args.legacy:
        rc = route_to_supervisor(args)
        if rc is not None:
            return rc
        sys.stderr.write("[invoker] supervisor 不可用，降级直连模式\n")
    with CarlaLock() as lock:
        if not lock.acquired:
            return err(
                "start",
                "已有进行中的 CARLA 操作（/tmp/adas_carla.lock 被其他进程持有）",
                lock=str(CARLA_LOCK_PATH),
            )
        return start_carla_direct(args)


def cmd_stop(args: argparse.Namespace) -> int:
    if not args.legacy:
        rc = route_to_supervisor(args)
        if rc is not None:
            return rc
    with CarlaLock() as lock:
        if not lock.acquired:
            return err(
                "stop",
                "已有进行中的 CARLA 操作（lock 被其他进程持有）",
                lock=str(CARLA_LOCK_PATH),
            )
        return stop_carla_direct(args)


def cmd_status(args: argparse.Namespace) -> int:
    if not args.legacy:
        rc = route_to_supervisor(args)
        if rc is not None:
            return rc
    return status_direct(args)


def cmd_restart(args: argparse.Namespace) -> int:
    if not args.legacy:
        rc = route_to_supervisor(args)
        if rc is not None:
            return rc
    with CarlaLock() as lock:
        if not lock.acquired:
            return err(
                "restart",
                "已有进行中的 CARLA 操作（lock 被其他进程持有）",
                lock=str(CARLA_LOCK_PATH),
            )
        stop_rc = stop_carla_direct(args)
        if stop_rc != 0:
            return stop_rc
        return start_carla_direct(args)


def cmd_transition(args: argparse.Namespace) -> int:
    """场景切换（仅 supervisor 模式支持，direct 模式返回 NOT_SUPPORTED）。"""
    if args.legacy or not is_supervisor_available():
        return err(
            "transition",
            "transition 仅在 supervisor 模式下支持；请安装并启用 carla-supervisor",
            kind=args.kind,
        )
    rc = route_to_supervisor(args)
    if rc is not None:
        return rc
    return err("transition", "supervisor 不可达")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _common_parser() -> argparse.ArgumentParser:
    """父 parser：所有 subcommand 共享的连接/场景/日志参数。

    用 argparse 的 parents 机制让 subparser 自动继承，而不是重复定义。
    """
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument(
        "--legacy", action="store_true",
        help="跳过 supervisor，直接走 flock 直连模式",
    )
    common.add_argument("--host", default=DEFAULT_RPC_HOST)
    common.add_argument("--port", type=int, default=DEFAULT_PORT)
    common.add_argument("--town", default=DEFAULT_TOWN)
    common.add_argument("--scenario", default="free")
    common.add_argument("--quality-level", default=DEFAULT_QUALITY,
                        choices=["Low", "Epic"])
    common.add_argument("--render-offscreen", action="store_true",
                        help="以 -RenderOffScreen 启动 CARLA")
    common.add_argument("--timeout", type=float, default=60.0,
                        help="启动 readiness 超时（秒）")
    common.add_argument("--shutdown-timeout", type=float, default=5.0,
                        help="停止时 SIGTERM → SIGKILL 等待时间")
    common.add_argument(
        "--log-file",
        default=str(REPO_ROOT / "logs" / "carla_invoker" / "carla.log"),
        help="直连模式下 CarlaUE4.sh 的输出日志路径",
    )
    return common


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        parents=[_common_parser()],
    )
    sub = parser.add_subparsers(dest="action", required=True)

    p_start = sub.add_parser(
        "start", help="启动 CARLA（已运行则复用）",
        parents=[_common_parser()],
    )
    p_start.set_defaults(func=cmd_start)

    p_stop = sub.add_parser(
        "stop", help="停止 CARLA（已停止则 noop）",
        parents=[_common_parser()],
    )
    p_stop.set_defaults(func=cmd_stop)

    p_status = sub.add_parser(
        "status", help="查询 CARLA 状态（输出 JSON）",
        parents=[_common_parser()],
    )
    p_status.set_defaults(func=cmd_status)

    p_restart = sub.add_parser(
        "restart", help="停止并重新启动 CARLA",
        parents=[_common_parser()],
    )
    p_restart.set_defaults(func=cmd_restart)

    p_trans = sub.add_parser(
        "transition", help="场景切换（仅 supervisor）",
        parents=[_common_parser()],
    )
    p_trans.add_argument(
        "--kind", choices=["hot", "soft", "hard"], required=True,
        help="hot=不重启 CARLA，只重发 spawn 配置；"
             "soft=不重启 CARLA，但 destroy actors 并重新 spawn；"
             "hard=town 改变，必须重启 CARLA",
    )
    p_trans.set_defaults(func=cmd_transition)

    return parser


def main(argv: Optional[list] = None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
