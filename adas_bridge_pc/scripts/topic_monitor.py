#!/usr/bin/env python3
"""常驻 ROS2 话题健康监控：objects_raw / kinematic_state / lane_state 的
Hz、Publisher 数量、以及 Jetson 节点存在性（safety_status 心跳代理）。

周期打印 `[OK]/[WARNING]/[ERROR] <topic>: <hz>Hz`，并把状态快照写进
session 目录的 system_status.json，供 GUI/复盘使用。故障分类
（PERCEPTION/SIMULATION/COMMUNICATION_FAILURE）仅用于日志标注，不做任何
安全决策 —— MRM 触发与否完全由 Orin 上的 safety_monitor 自己判定。

用法：
    python3 topic_monitor.py --session-dir logs/hil_run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import hil_common as hc  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from adas_msgs.msg import LaneState, SafetyStatus, TrackedObjectArray  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402

PRINT_PERIOD_S = 1.0
STATUS_FLUSH_PERIOD_S = 2.0


class TopicMonitorNode(Node):
    def __init__(self, session_dir: Path, carla_host: str, carla_port: int):
        super().__init__("hil_topic_monitor")
        self.session_dir = session_dir
        self.carla_host = carla_host
        self.carla_port = carla_port
        self.fault_codes = hc.load_fault_codes()

        self.objects = hc.FreshnessTracker()
        self.odom = hc.FreshnessTracker()
        self.lane = hc.FreshnessTracker()
        self.safety = hc.FreshnessTracker()

        self.create_subscription(TrackedObjectArray, hc.TOPIC_OBJECTS,
                                  lambda m: self.objects.on_message(m), hc.SENSOR_QOS)
        self.create_subscription(Odometry, hc.TOPIC_ODOM,
                                  lambda m: self.odom.on_message(m), hc.SENSOR_QOS)
        self.create_subscription(LaneState, hc.TOPIC_LANE,
                                  lambda m: self.lane.on_message(m), hc.SENSOR_QOS)
        self.create_subscription(SafetyStatus, hc.TOPIC_SAFETY_STATUS,
                                  lambda m: self.safety.on_message(m), 10)

        self._last_print = 0.0
        self._last_flush = 0.0
        self._active_fault: str | None = None
        self.create_timer(0.2, self._tick)

    def _bridge_log_tail(self, n_bytes: int = 4096) -> str:
        p = self.session_dir / "bridge.log"
        if not p.exists():
            return ""
        try:
            with open(p, "rb") as f:
                f.seek(max(0, p.stat().st_size - n_bytes))
                return f.read().decode("utf-8", errors="ignore")
        except OSError:
            return ""

    def _tick(self) -> None:
        now = time.monotonic()
        if now - self._last_print >= PRINT_PERIOD_S:
            self._last_print = now
            self._print_and_evaluate()
        if now - self._last_flush >= STATUS_FLUSH_PERIOD_S:
            self._last_flush = now
            self._flush_status()

    def _print_and_evaluate(self) -> None:
        rows = {
            "objects_raw": self.objects,
            "kinematic_state": self.odom,
            "lane_state": self.lane,
        }
        statuses = {}
        for label, tracker in rows.items():
            status = tracker.classify()
            statuses[label] = status
            hz = tracker.hz()
            print(f"[{status}] {label}: {hz:.1f}Hz", flush=True)

        names = [n for n, _ in self.get_node_names_and_namespaces()]
        presence = {name: hc.has_node(names, name) for name in hc.JETSON_STACK_NODES}
        jetson_present = sum(presence.values())
        jetson_total = len(presence)

        bridge_alive = hc.has_node(names, "carla_bridge")
        carla_ok = hc.carla_probe(self.carla_host, self.carla_port, timeout_s=1.5)

        category = hc.classify_failure(
            carla_ok=carla_ok,
            bridge_process_alive=bridge_alive,
            bridge_log_tail=self._bridge_log_tail(),
            jetson_nodes_present=jetson_present,
            jetson_nodes_total=jetson_total,
            objects_status=statuses["objects_raw"],
            lane_status=statuses["lane_state"],
            odom_status=statuses["kinematic_state"],
        )

        any_error = any(s == "ERROR" for s in statuses.values()) or jetson_present == 0
        if any_error and category:
            code_num = {"SIMULATION_FAILURE": 100, "COMMUNICATION_FAILURE": 300,
                        "PERCEPTION_FAILURE": 200}[category]
            fc = self.fault_codes[code_num]
            if self._active_fault != fc["name"]:
                self._active_fault = fc["name"]
                ev = hc.FaultEvent(code=code_num, name=fc["name"], level=fc["level"],
                                    source="topic_monitor",
                                    detail=f"objects={statuses['objects_raw']} "
                                           f"odom={statuses['kinematic_state']} "
                                           f"lane={statuses['lane_state']} "
                                           f"jetson={jetson_present}/{jetson_total}",
                                    category=category)
                hc.emit_fault(ev, self.session_dir)
        elif self._active_fault is not None:
            hc.clear_fault(self._active_fault, self.session_dir)
            self._active_fault = None

        self._latest = {
            "topic_hz": {k: round(v.hz(), 1) for k, v in rows.items()},
            "topic_status": statuses,
            "jetson_online": jetson_present == jetson_total,
            "jetson_nodes": f"{jetson_present}/{jetson_total}",
            "can_online": None,
        }

    def _flush_status(self) -> None:
        if not hasattr(self, "_latest"):
            return
        status = hc.read_status(self.session_dir)
        status.update(self._latest)
        status["last_updated"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        hc.write_status(self.session_dir, status)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-dir", type=Path, required=True)
    parser.add_argument("--carla-host", default="127.0.0.1")
    parser.add_argument("--carla-port", type=int, default=2000)
    args = parser.parse_args()

    args.session_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = TopicMonitorNode(args.session_dir, args.carla_host, args.carla_port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
