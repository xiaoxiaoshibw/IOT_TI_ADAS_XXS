#!/usr/bin/env python3
"""CARLA Supervisor 守护进程。

唯一持有 /tmp/adas_carla.lock 的进程，所有 GUI / orchestrator / shell 的
CARLA start/stop 请求都通过 Unix socket 与它通信，串行化执行，避免
双启 / 场景切换竞态。

暴露的命令：
    start    --host X --port Y --town Z --scenario S --quality-level L
    stop     --host X --port Y --shutdown-timeout 5.0
    restart  (start + stop 的组合)
    status
    transition --kind hot|soft|hard --scenario S --town Z

内部 transition state machine 决策（仅 HardReset 需要真重启 CARLA，
SoftReset 只销毁 actors 重新 spawn，HotReload 重发 spawn 配置不动 world）。
bridge_node.py 在 SoftReset / HotReload 时通过 SIGUSR1 reload 不退进程。

systemd 集成（carla-supervisor.service）：
    Type=notify
    ExecStart=/usr/bin/python3 /home/xxs/bowen_ADAS/adas_bridge_pc/scripts/carla_supervisor.py --foreground
    NotifyAccess=all
"""

from __future__ import annotations

import argparse
import fcntl
import json
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "adas_bridge_pc" / "scripts"))

import carla_invoker as inv  # noqa: E402
import hil_common as hc  # noqa: E402

# Supervisor 单例锁：被它持有时所有外部 CARLA 调用都不应直接走 flock 直连路径
SUPERVISOR_LOCK_PATH = Path(
    os.environ.get("ADAS_CARLA_SUPERVISOR_LOCK", "/tmp/adas_carla_supervisor.lock")
)


class Supervisor:
    def __init__(self, socket_path: Path, foreground: bool) -> None:
        self.socket_path = socket_path
        self.foreground = foreground
        self.server: Optional[socket.socket] = None
        self.carla_proc: Optional[subprocess.Popen] = None
        self.carla_pid_file = Path("/tmp/adas_carla_supervisor.pid")
        self.carla_log_file = Path(
            os.environ.get(
                "ADAS_CARLA_SUPERVISOR_LOG",
                str(REPO_ROOT / "logs" / "carla_supervisor" / "carla.log"),
            )
        )
        self._shutdown = False
        # 持有 supervisor 自己的 flock（与 carla_invoker 的 flock 互斥）
        self._lock_fd: Optional[int] = None

    # ---------- 锁 ----------

    def _acquire_supervisor_lock(self) -> bool:
        """非阻塞持有 supervisor 单例锁：第二个 supervisor 进程应直接退出。"""
        SUPERVISOR_LOCK_PATH.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(
            str(SUPERVISOR_LOCK_PATH),
            os.O_RDWR | os.O_CREAT, 0o644,
        )
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            if exc.errno in (os.errno.EWOULDBLOCK, os.errno.EAGAIN):
                os.close(fd)
                return False
            raise
        os.lseek(fd, 0, os.SEEK_SET)
        os.write(fd, f"{os.getpid()}\n".encode())
        self._lock_fd = fd
        return True

    # ---------- 通知 systemd ----------

    def _notify(self, state: str) -> None:
        if not self.foreground:
            return
        notify_socket = os.environ.get("NOTIFY_SOCKET")
        if not notify_socket:
            return
        try:
            if notify_socket.startswith("@"):
                sock_path = "\0" + notify_socket[1:]
            else:
                sock_path = notify_socket
            s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            s.connect(sock_path)
            s.sendall(f"{state}\n".encode())
            s.close()
        except OSError as exc:
            sys.stderr.write(f"[supervisor] sd_notify({state}) 失败: {exc}\n")

    # ---------- 信号处理 ----------

    def _install_signal_handlers(self) -> None:
        signal.signal(signal.SIGTERM, self._on_signal)
        signal.signal(signal.SIGINT, self._on_signal)

    def _on_signal(self, signum: int, frame) -> None:
        self._shutdown = True

    # ---------- CARLA 进程管理（supervisor 自己直接 fork CarlaUE4） ----------

    def _ensure_carla_lock_held(self) -> bool:
        """supervisor 是唯一该持 carla_lock 的进程。直连模式下 flock 由
        invoker 临时持有；supervisor 模式下 supervisor 全程持有。
        """
        fd = os.open(str(inv.CARLA_LOCK_PATH), os.O_RDWR | os.O_CREAT, 0o644)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._carla_lock_fd = fd
            return True
        except OSError as exc:
            os.close(fd)
            if exc.errno in (os.errno.EWOULDBLOCK, os.errno.EAGAIN):
                return False
            raise

    _carla_lock_fd: Optional[int] = None  # 标记持有

    def _release_carla_lock(self) -> None:
        if getattr(self, "_carla_lock_fd", None) is not None:
            try:
                fcntl.flock(self._carla_lock_fd, fcntl.LOCK_UN)
            finally:
                os.close(self._carla_lock_fd)
                self._carla_lock_fd = None

    def _start_carla(self, payload: dict) -> dict:
        if not self._ensure_carla_lock_held():
            return {
                "action": "start", "state": "busy",
                "error": "另一实例正持有 /tmp/adas_carla.lock（直连模式残留？）",
            }
        try:
            if hc.carla_probe(payload["host"], payload["port"], timeout_s=1.0):
                return {
                    "action": "start", "state": "already_running",
                    "host": payload["host"], "port": payload["port"],
                    "note": "CARLA 已在运行（supervisor 模式探测到）",
                }
            bin_path = os.path.join(
                os.environ.get("CARLA_ROOT", os.path.expanduser("~/CARLA_0.9.16")),
                "CarlaUE4.sh",
            )
            if not os.path.isfile(bin_path) or not os.access(bin_path, os.X_OK):
                return {
                    "action": "start", "state": "error",
                    "error": f"CarlaUE4.sh 不存在或不可执行: {bin_path}",
                }
            cmd = [
                bin_path, "-nosound",
                f"-carla-rpc-port={payload['port']}",
                f"-quality-level={payload.get('quality_level', 'Epic')}",
            ]
            self.carla_log_file.parent.mkdir(parents=True, exist_ok=True)
            log_f = open(self.carla_log_file, "a", buffering=1)
            self.carla_proc = subprocess.Popen(
                cmd, stdout=log_f, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            self.carla_pid_file.write_text(str(self.carla_proc.pid))
            # readiness 探测
            deadline = time.monotonic() + float(payload.get("timeout", 60.0))
            while time.monotonic() < deadline:
                if hc.carla_probe(
                    payload["host"], payload["port"], timeout_s=1.0
                ):
                    return {
                        "action": "start", "state": "started",
                        "pid": self.carla_proc.pid,
                        "host": payload["host"], "port": payload["port"],
                        "command": cmd, "log": str(self.carla_log_file),
                    }
                if self.carla_proc.poll() is not None:
                    return {
                        "action": "start", "state": "error",
                        "error": f"CarlaUE4.sh 启动后立即退出 rc={self.carla_proc.returncode}",
                        "pid": self.carla_proc.pid,
                    }
                time.sleep(0.5)
            return {
                "action": "start", "state": "error",
                "error": "readiness 超时",
                "pid": self.carla_proc.pid,
            }
        finally:
            # CARLA 启动成功后仍持有 lock —— supervisor 是 lock 的唯一长期持有者
            if self.carla_proc is None or (
                self.carla_proc.poll() is not None
                and not hc.carla_probe(payload["host"], payload["port"], timeout_s=0.5)
            ):
                self._release_carla_lock()

    def _stop_carla(self, payload: dict) -> dict:
        if not hc.carla_probe(payload["host"], payload["port"], timeout_s=1.0):
            return {
                "action": "stop", "state": "already_stopped",
                "host": payload["host"], "port": payload["port"],
            }
        pid = inv.find_running_carla_pid()
        if pid is None:
            return {
                "action": "stop", "state": "error",
                "error": "RPC 可达但找不到 CarlaUE4 进程",
            }
        try:
            pgid = os.getpgid(pid)
        except ProcessLookupError:
            return {"action": "stop", "state": "stopped"}
        try:
            os.killpg(pgid, signal.SIGTERM)
        except ProcessLookupError:
            return {"action": "stop", "state": "stopped"}
        deadline = time.monotonic() + float(payload.get("shutdown_timeout", 5.0))
        while time.monotonic() < deadline:
            try:
                os.killpg(pgid, 0)
            except ProcessLookupError:
                self._release_carla_lock()
                return {"action": "stop", "state": "stopped"}
            time.sleep(0.1)
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self._release_carla_lock()
        return {"action": "stop", "state": "stopped"}

    def _status(self, payload: dict) -> dict:
        alive = hc.carla_probe(payload["host"], payload["port"], timeout_s=1.0)
        map_name = hc.carla_map_name(
            payload["host"], payload["port"], timeout_s=1.0
        )
        return {
            "action": "status", "state": "running" if alive else "stopped",
            "host": payload["host"], "port": payload["port"],
            "map": map_name, "pid": (
                inv.find_running_carla_pid() if alive else None
            ),
            "expected_town": payload.get("town"),
            "town_matches": (
                map_name is not None
                and payload.get("town") is not None
                and map_name.endswith("/" + payload["town"])
            ),
        }

    def _transition(self, payload: dict) -> dict:
        """场景切换 —— 当前 HardReset 等价 stop+start（CARLA 0.9.16 没有
        动态 reload town 的官方 API），SoftReset / HotReload 需要
        bridge_node.py 支持 SIGUSR1 reload（Phase 3 落地）。

        本版本仅实现 HardReset；SoftReset/HotReload 收到请求时返回
        'soft_reset_requires_bridge_signal' 提示用户升级 bridge_node。
        """
        kind = payload.get("kind")
        if kind == "hard":
            stop_resp = self._stop_carla(payload)
            if stop_resp.get("state") != "stopped":
                return stop_resp
            start_resp = self._start_carla(payload)
            start_resp["transition"] = "hard"
            return start_resp
        if kind == "soft":
            # TODO Phase 3: bridge_node 加 --enable-reload-on-signal 后，
            # 这里应 SIGUSR1 当前 bridge 进程触发 reload，而不是重启 CARLA。
            return {
                "action": "transition", "state": "not_implemented",
                "kind": "soft",
                "note": "SoftReset 需要 bridge_node 支持 SIGUSR1 reload，"
                        "目前降级为 HardReset",
                "fallback": "hard",
            }
        if kind == "hot":
            return {
                "action": "transition", "state": "not_implemented",
                "kind": "hot",
                "note": "HotReload 需要 bridge_node SIGUSR2 hot-reload "
                        "支持，目前降级为 noop",
            }
        return {"action": "transition", "state": "error", "error": f"未知 kind: {kind}"}

    # ---------- 客户端分发 ----------

    def _dispatch(self, payload: dict) -> dict:
        action = payload.get("action")
        if action == "start":
            return self._start_carla(payload)
        if action == "stop":
            return self._stop_carla(payload)
        if action == "status":
            return self._status(payload)
        if action == "restart":
            stop_resp = self._stop_carla(payload)
            if stop_resp.get("state") != "stopped":
                return stop_resp
            return self._start_carla(payload)
        if action == "transition":
            return self._transition(payload)
        return {"action": action, "state": "error",
                "error": f"未知 action: {action}"}

    def _serve_one(self, client: socket.socket) -> None:
        client.settimeout(30.0)
        try:
            buf = b""
            while not buf.endswith(b"\n"):
                chunk = client.recv(65536)
                if not chunk:
                    break
                buf += chunk
            line = buf.decode("utf-8", errors="replace").strip()
            if not line:
                return
            try:
                payload = json.loads(line)
            except json.JSONDecodeError as exc:
                client.sendall(
                    (json.dumps({"state": "error", "error": f"bad json: {exc}"}) + "\n").encode()
                )
                return
            response = self._dispatch(payload)
            client.sendall((json.dumps(response, sort_keys=True) + "\n").encode())
        except (OSError, socket.timeout):
            return
        finally:
            try:
                client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            client.close()

    # ---------- 主循环 ----------

    def run(self) -> int:
        if not self._acquire_supervisor_lock():
            sys.stderr.write(
                f"[supervisor] 已有 supervisor 持 {SUPERVISOR_LOCK_PATH}，退出\n"
            )
            return 2
        self._install_signal_handlers()
        # 删除旧 socket 文件
        try:
            self.socket_path.unlink()
        except FileNotFoundError:
            pass
        self.socket_path.parent.mkdir(parents=True, exist_ok=True)
        self.server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.server.bind(str(self.socket_path))
        os.chmod(self.socket_path, 0o660)
        self.server.listen(8)
        self.server.settimeout(0.5)
        self._notify("READY=1")
        sys.stderr.write(
            f"[supervisor] 监听 {self.socket_path}（pid={os.getpid()}）\n"
        )
        try:
            while not self._shutdown:
                try:
                    client, _ = self.server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    if self._shutdown:
                        break
                    raise
                self._serve_one(client)
        finally:
            try:
                self.server.close()
            except OSError:
                pass
            try:
                self.socket_path.unlink()
            except FileNotFoundError:
                pass
            # 退出时若 CARLA 仍跑，不主动 kill（外部 watchdog 接管）
            if self._lock_fd is not None:
                try:
                    fcntl.flock(self._lock_fd, fcntl.LOCK_UN)
                    os.close(self._lock_fd)
                except OSError:
                    pass
        return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--foreground", action="store_true",
        help="前台运行（systemd Type=notify 需要）",
    )
    parser.add_argument(
        "--socket", type=Path,
        default=Path(os.environ.get(
            "ADAS_CARLA_SUPERVISOR_SOCK", "/tmp/adas_carla_supervisor.sock"
        )),
        help="Unix socket 路径",
    )
    return parser


def main(argv: Optional[list] = None) -> int:
    args = build_parser().parse_args(argv)
    sup = Supervisor(socket_path=args.socket, foreground=args.foreground)
    return sup.run()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
