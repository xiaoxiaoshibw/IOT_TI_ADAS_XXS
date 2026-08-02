#!/usr/bin/env python3
"""HIL Supervisor 共享库：故障码、CARLA 探活、ROS2 拓扑/新鲜度判定、会话日志。

供 check_hil_ready.py / carla_watchdog.py / topic_monitor.py 共用，避免各脚本
各写一套探测逻辑。QoS 约定：三个感知话题（objects_raw/kinematic_state/
lane_state）在 bridge_node.py 里用的是 SENSOR_QOS = RELIABLE + VOLATILE +
depth 5，不是 BEST_EFFORT/SensorData —— 这里全部对齐，避免用错 QoS 导致
订阅端收不到消息却误判为链路故障（今天诊断时踩过这个坑）。
"""

from __future__ import annotations

import json
import os
import socket
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
FAULT_CODES_PATH = Path(__file__).resolve().parent / "fault_codes.yaml"
LOGS_ROOT = REPO_ROOT / "logs"

# 与 bridge_node.py 完全一致的 QoS：RELIABLE + VOLATILE + depth 5。
try:
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

    SENSOR_QOS = QoSProfile(depth=5)
    SENSOR_QOS.durability = QoSDurabilityPolicy.VOLATILE
    SENSOR_QOS.reliability = QoSReliabilityPolicy.RELIABLE
except ImportError:  # 允许在未 source ROS 环境时仅做 yaml/探测相关的单元测试
    SENSOR_QOS = None

# Jetson 侧安全链关键节点（root CLAUDE.md 记录的生命周期激活顺序）。
JETSON_STACK_NODES = (
    "vehicle_interface",
    "command_gate",
    "safety_monitor",
    "aeb",
    "trajectory_follower",
    "trajectory_planner",
    "behavior_planner",
    "object_tracker",
)

TOPIC_OBJECTS = "/adas/perception/objects_raw"
TOPIC_ODOM = "/adas/localization/kinematic_state"
TOPIC_LANE = "/adas/perception/lane_state"
TOPIC_SAFETY_STATUS = "/adas/system/safety_status"

# 用户给定阈值：objects_raw 正常 20Hz / <5Hz 异常 / 0Hz 故障。三个感知话题共用。
HZ_OK = 20.0
HZ_WARN = 5.0


# ---------------------------------------------------------------------------
# 故障码
# ---------------------------------------------------------------------------


def load_fault_codes() -> dict:
    with open(FAULT_CODES_PATH, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


@dataclass
class FaultEvent:
    code: int
    name: str
    level: str
    source: str
    detail: str = ""
    category: Optional[str] = None
    ts: str = field(default_factory=lambda: datetime.now().isoformat(timespec="seconds"))

    def line(self) -> str:
        return (
            f"FAULT_CODE={self.code} FAULT_NAME={self.name} LEVEL={self.level} "
            f"SOURCE={self.source} CATEGORY={self.category or '-'} "
            f"DETAIL={self.detail!r} TIMESTAMP={self.ts}"
        )


def emit_fault(event: FaultEvent, session_dir: Optional[Path] = None) -> None:
    """统一输出故障事件：stdout 一行 + 追加进 session 目录的 system_status.json。"""
    print(event.line(), flush=True)
    if session_dir is None:
        return
    status = read_status(session_dir)
    status.setdefault("fault_history", []).append(asdict(event))
    status["current_faults"] = [
        e for e in status.get("current_faults", []) if e.get("name") != event.name
    ]
    status["current_faults"].append(asdict(event))
    status["last_updated"] = datetime.now().isoformat(timespec="seconds")
    write_status(session_dir, status)


def clear_fault(name: str, session_dir: Optional[Path] = None) -> None:
    """故障恢复后从 current_faults 里摘除（fault_history 保留作为审计轨迹）。"""
    if session_dir is None:
        return
    status = read_status(session_dir)
    status["current_faults"] = [
        e for e in status.get("current_faults", []) if e.get("name") != name
    ]
    status["last_updated"] = datetime.now().isoformat(timespec="seconds")
    write_status(session_dir, status)


# ---------------------------------------------------------------------------
# session_status.json 读写（原子写，多进程并发追加安全靠"读-改-写+os.replace"）
# ---------------------------------------------------------------------------


def status_path(session_dir: Path) -> Path:
    return session_dir / "system_status.json"


def read_status(session_dir: Path) -> dict:
    p = status_path(session_dir)
    if not p.exists():
        return {
            "start_time": datetime.now().isoformat(timespec="seconds"),
            "current_faults": [],
            "fault_history": [],
            "topic_hz": {},
            "jetson_online": None,
            "can_online": None,
            "last_updated": None,
        }
    try:
        with open(p, "r", encoding="utf-8") as f:
            return json.load(f)
    except (json.JSONDecodeError, OSError):
        return {
            "start_time": datetime.now().isoformat(timespec="seconds"),
            "current_faults": [],
            "fault_history": [],
            "topic_hz": {},
            "jetson_online": None,
            "can_online": None,
            "last_updated": None,
        }


def write_status(session_dir: Path, status: dict) -> None:
    session_dir.mkdir(parents=True, exist_ok=True)
    p = status_path(session_dir)
    tmp = p.with_suffix(".json.tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(status, f, ensure_ascii=False, indent=2)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, p)


def new_session_dir() -> Path:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    d = LOGS_ROOT / f"hil_run_{ts}"
    d.mkdir(parents=True, exist_ok=True)
    return d


def latest_session_dir() -> Optional[Path]:
    if not LOGS_ROOT.exists():
        return None
    runs = sorted(LOGS_ROOT.glob("hil_run_*"), key=lambda p: p.name)
    return runs[-1] if runs else None


# ---------------------------------------------------------------------------
# CARLA 探活 —— 端口开着不等于 RPC 层活着（今天实测过的死锁模式）
# ---------------------------------------------------------------------------


def port_open(host: str, port: int, timeout_s: float = 1.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True
    except OSError:
        return False


def carla_probe(host: str = "127.0.0.1", port: int = 2000, timeout_s: float = 3.0) -> bool:
    """真正的 CARLA 就绪判定：不满足于端口可连接，必须 RPC 调用有响应。

    今天的故障案例里，CARLA 进程存活、端口 2000 也在监听，但 RPC 层因为
    synchronous_mode 卡死，get_world() 会无限期挂起 —— 只查端口会漏检这种
    情况，必须实际调用一次 get_world()。
    """
    if not port_open(host, port, timeout_s=min(timeout_s, 2.0)):
        return False
    try:
        import carla  # 延迟导入：CARLA 的 PythonAPI 可能不在所有环境里
    except ImportError:
        return port_open(host, port, timeout_s)
    try:
        client = carla.Client(host, port)
        client.set_timeout(timeout_s)
        client.get_world()
        return True
    except RuntimeError:
        return False


def carla_map_name(host: str = "127.0.0.1", port: int = 2000, timeout_s: float = 3.0) -> Optional[str]:
    try:
        import carla
    except ImportError:
        return None
    try:
        client = carla.Client(host, port)
        client.set_timeout(timeout_s)
        return client.get_world().get_map().name
    except RuntimeError:
        return None


def can_adapter_probe(device: int = 0, channel: int = 1, bitrate: int = 500000) -> tuple[bool, str]:
    """探测 PC 侧 USB CANalyst-II 调试适配器是否可打开。

    这是调试备源（root CLAUDE.md：PC 的 USB CANalyst-II 没有 SocketCAN 驱动，
    走 python-can 的 canalystii 后端，MCU 在 channel 1），不是安全执行链路，
    打不开只降级为 WARNING，不阻断 HIL 启动。
    """
    try:
        import can
    except ImportError:
        return False, "python-can 未安装（调试适配器不可用，不影响主链路）"
    bus = None
    try:
        bus = can.Bus(interface="canalystii", device=device, channel=channel, bitrate=bitrate)
        return True, "CANalyst-II 适配器可打开"
    except Exception as exc:  # noqa: BLE001 - 硬件探测，异常类型来自底层驱动，无法穷举
        return False, f"CANalyst-II 不可用: {exc}"
    finally:
        if bus is not None:
            try:
                bus.shutdown()
            except Exception:  # noqa: BLE001
                pass


def carla_process_alive() -> bool:
    try:
        import subprocess

        out = subprocess.run(
            ["pgrep", "-f", "CarlaUE4-Linux-Shipping"], capture_output=True, text=True
        )
        return out.returncode == 0 and bool(out.stdout.strip())
    except OSError:
        return False


# ---------------------------------------------------------------------------
# ROS2 拓扑/新鲜度 —— 对齐 adas_gui/src/ros_bridge.cpp::updateHealthSnapshot()
# ---------------------------------------------------------------------------


def has_node(node_names: list[str], wanted: str) -> bool:
    """匹配裸名或 '/xxx/wanted' 形式的完全限定名，对齐 ros_bridge.cpp 的 has_node()。"""
    for n in node_names:
        if n == wanted or n.endswith("/" + wanted):
            return True
    return False


def node_presence(node, names: tuple[str, ...] = JETSON_STACK_NODES) -> dict[str, bool]:
    current = [n for n, _ in node.get_node_names_and_namespaces()]
    return {name: has_node(current, name) for name in names}


class FreshnessTracker:
    """跟踪某个话题最近一次收到消息的时间 + 滚动窗口内的消息计数（用于估算 Hz）。"""

    def __init__(self):
        self.last_stamp: Optional[float] = None
        self._recent: list[float] = []

    def on_message(self, _msg=None) -> None:
        now = time.monotonic()
        self.last_stamp = now
        self._recent.append(now)
        cutoff = now - 5.0
        self._recent = [t for t in self._recent if t >= cutoff]

    def age(self) -> float:
        if self.last_stamp is None:
            return float("inf")
        return time.monotonic() - self.last_stamp

    def hz(self, window_s: float = 3.0) -> float:
        now = time.monotonic()
        cutoff = now - window_s
        count = sum(1 for t in self._recent if t >= cutoff)
        if count <= 1:
            return 0.0
        span = max(self._recent[-1] - max(self._recent[0], cutoff), 1e-3)
        return (count - 1) / span

    def classify(self, warn: float = HZ_WARN) -> str:
        """三档：0Hz/长时间无消息=ERROR(故障)；<warn Hz=WARNING(异常)；否则 OK。"""
        if self.age() > 1.0 or self.hz() <= 0.0:
            return "ERROR"
        if self.hz() < warn:
            return "WARNING"
        return "OK"


# ---------------------------------------------------------------------------
# 故障分类（启发式，仅用于日志/展示，不参与 MRM 决策）
# ---------------------------------------------------------------------------


def classify_failure(
    *,
    carla_ok: bool,
    bridge_process_alive: bool,
    bridge_log_tail: str = "",
    jetson_nodes_present: int,
    jetson_nodes_total: int,
    objects_status: str,
    lane_status: str,
    odom_status: str,
) -> Optional[str]:
    """返回 PERCEPTION_FAILURE / SIMULATION_FAILURE / COMMUNICATION_FAILURE / None。

    判定顺序即优先级：先看仿真链路本身是否断了，再看跨机通信是否非对称失败，
    最后才归到"只是某个感知话题掉线"。
    """
    if not carla_ok or not bridge_process_alive or "waiting for the simulator" in bridge_log_tail:
        return "SIMULATION_FAILURE"

    if jetson_nodes_total > 0 and jetson_nodes_present == 0 and odom_status == "OK":
        # 本地桥接一切正常（PC 自己在发布），但 Jetson 侧关键节点在 ROS 图里
        # 整体消失 —— 对应 common.sh 注释里点名过的 PC/Orin 非对称发现问题。
        return "COMMUNICATION_FAILURE"

    bad = [s for s in (objects_status, lane_status) if s != "OK"]
    if bad and odom_status == "OK" and jetson_nodes_present == jetson_nodes_total:
        return "PERCEPTION_FAILURE"

    return None
