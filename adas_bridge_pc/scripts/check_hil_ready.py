#!/usr/bin/env python3
"""HIL 启动自检：CARLA / ROS2 Bridge / 感知话题 / Jetson 连接 / CAN。

用法：
    python3 check_hil_ready.py [--session-dir logs/hil_run_XXXXXXXX_XXXXXX]
                                [--sample-s 4.0]

设计成"先启动，再体检"——Publisher count>0 这类检测本身就要求桥接已经在
跑，所以本脚本假定 CARLA/bridge/GUI 已经由 start_pc_stack.sh 拉起，检查的
是"起来了没起对"，不是"要不要起"。全部检测项独立执行、互不short-circuit，
方便一次性看到所有问题；只要有 ERROR/CRITICAL 级别的失败就退出 1。
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


def check_carla(host: str, port: int) -> tuple[bool, str]:
    if not hc.carla_process_alive():
        return False, "CarlaUE4 进程未找到"
    if not hc.carla_probe(host, port, timeout_s=5.0):
        return False, f"RPC 无响应 {host}:{port}（端口可能在监听但 get_world() 挂起）"
    map_name = hc.carla_map_name(host, port)
    if not map_name:
        return False, "地图未加载"
    return True, f"地图={map_name}"


class ReadyProbeNode(Node):
    def __init__(self):
        super().__init__("check_hil_ready")
        self.objects = hc.FreshnessTracker()
        self.odom = hc.FreshnessTracker()
        self.lane = hc.FreshnessTracker()
        self.safety = hc.FreshnessTracker()
        self.sub_objects = self.create_subscription(
            TrackedObjectArray, hc.TOPIC_OBJECTS, lambda m: self.objects.on_message(m), hc.SENSOR_QOS
        )
        self.sub_odom = self.create_subscription(
            Odometry, hc.TOPIC_ODOM, lambda m: self.odom.on_message(m), hc.SENSOR_QOS
        )
        self.sub_lane = self.create_subscription(
            LaneState, hc.TOPIC_LANE, lambda m: self.lane.on_message(m), hc.SENSOR_QOS
        )
        self.sub_safety = self.create_subscription(
            SafetyStatus, hc.TOPIC_SAFETY_STATUS, lambda m: self.safety.on_message(m), 10
        )

    def publisher_count(self, topic: str) -> int:
        return self.count_publishers(topic)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-dir", type=Path, default=None)
    parser.add_argument("--carla-host", default="127.0.0.1")
    parser.add_argument("--carla-port", type=int, default=2000)
    parser.add_argument("--sample-s", type=float, default=4.0)
    args = parser.parse_args()

    session_dir = args.session_dir or hc.new_session_dir()
    fault_codes = hc.load_fault_codes()
    failures: list[hc.FaultEvent] = []
    passed: list[str] = []

    def fail(code: int, detail: str):
        fc = fault_codes[code]
        ev = hc.FaultEvent(code=code, name=fc["name"], level=fc["level"],
                            source="check_hil_ready", detail=detail, category=fc.get("category"))
        failures.append(ev)
        hc.emit_fault(ev, session_dir)

    def ok(label: str, detail: str = ""):
        passed.append(label)
        print(f"[OK] {label}{': ' + detail if detail else ''}")

    # 1. CARLA
    carla_ok, carla_detail = check_carla(args.carla_host, args.carla_port)
    if carla_ok:
        ok("CARLA", carla_detail)
    else:
        fail(100, carla_detail)

    # 2-4. ROS2 Bridge / Topic 健康 / Jetson 连接（都需要 rclpy 图信息，一个节点跑完）
    rclpy.init()
    try:
        node = ReadyProbeNode()
        deadline = time.monotonic() + args.sample_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        names = [n for n, _ in node.get_node_names_and_namespaces()]
        if hc.has_node(names, "carla_bridge"):
            ok("ROS2 Bridge", "carla_bridge 节点已发现")
        else:
            fail(200, "carla_bridge 节点未在 ROS 图中发现")

        for label, topic, tracker in (
            ("objects_raw", hc.TOPIC_OBJECTS, node.objects),
            ("kinematic_state", hc.TOPIC_ODOM, node.odom),
            ("lane_state", hc.TOPIC_LANE, node.lane),
        ):
            pub_count = node.publisher_count(topic)
            status = tracker.classify()
            hz = tracker.hz()
            if pub_count == 0:
                fail(200, f"{topic} Publisher count=0")
            elif status == "ERROR":
                fail(200, f"{topic} 0Hz（发布者存在但收不到消息）")
            elif status == "WARNING":
                print(f"[WARNING] {label}: {hz:.1f}Hz (<{hc.HZ_WARN}Hz)")
            else:
                ok(label, f"{hz:.1f}Hz, publishers={pub_count}")

        presence = hc.node_presence(node)
        present_count = sum(presence.values())
        total = len(presence)
        safety_fresh = node.safety.age() < 1.0
        # Jetson 存活判据以 safety_status 心跳（safety_monitor 的 proof-of-life）为准，
        # 节点名枚举仅作补充。原因：Orin 跑 Humble、上位机跑 Jazzy，两者的
        # ros_discovery_info(ParticipantEntitiesInfo) USER_DATA type-hash 不兼容，
        # Jazzy 侧解析 Humble 参与者失败，get_node_names_and_namespaces() 结构性地
        # 枚举不到 Orin 节点名（会刷 "Failed to parse type hash" WARN）——但 DDS 话题
        # 收发按名字+类型匹配，与图元数据无关，数据面完全正常。若仍以节点名在线
        # 作硬门禁，跨发行版下 present_count 恒为 0，闭环再健康也永远判 CRITICAL。
        if safety_fresh:
            if present_count == total:
                ok("Jetson", f"{total}/{total} 关键节点在线，safety_status 心跳新鲜")
            else:
                ok("Jetson",
                   f"safety_status 心跳新鲜（存活判据）；ROS 图节点名可见 "
                   f"{present_count}/{total}——Humble↔Jazzy 跨发行版下节点名枚举不可用，"
                   f"属已知限制，不影响数据面")
        else:
            missing = [k for k, v in presence.items() if not v]
            fail(300, f"safety_status 心跳丢失（age≥1.0s）；ROS 图节点名可见 "
                      f"{present_count}/{total}，缺失 {missing}")

        node.destroy_node()
    finally:
        rclpy.shutdown()

    # 5. CAN（调试适配器，失败降级为 WARNING，不阻断）
    can_ok, can_detail = hc.can_adapter_probe()
    if can_ok:
        ok("CAN", can_detail)
    else:
        print(f"[WARNING] CAN: {can_detail}")

    status = hc.read_status(session_dir)
    status["checked_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    status["passed"] = passed
    status["ready"] = len(failures) == 0
    hc.write_status(session_dir, status)

    print()
    if not failures:
        print("HIL READY")
        print(f"CARLA       OK")
        print(f"ROS2        OK")
        print(f"Jetson      OK")
        print(f"CAN         {'OK' if can_ok else 'WARN'}")
        print(f"Safety      OK")
        return 0

    print(f"HIL NOT READY — {len(failures)} 项失败:")
    for ev in failures:
        print(f"  {ev.line()}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
