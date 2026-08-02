#!/usr/bin/env python3
"""ROS 2 helpers for active Phase 3 tests; publishes navigation requests only."""

from __future__ import annotations

import math
import time
from collections import deque
from pathlib import Path
from typing import Any, Callable

from adas_msgs.msg import GlobalRoute, LaneGraph
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped transforms with tf2
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

from validation_common import atomic_json


def transient_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def reliable_qos() -> QoSProfile:
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


def wait_for(node: Node, predicate: Callable[[], Any], timeout_s: float, label: str) -> Any:
    import rclpy

    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        value = predicate()
        if value:
            return value
    raise TimeoutError(f"timeout waiting for {label} ({timeout_s:.1f}s)")


def nearest_lane(graph: LaneGraph, pose: Pose) -> Any:
    candidates = [lane for lane in graph.lanes if lane.centerline]
    if not candidates:
        raise RuntimeError("lane graph contains no centerline")
    return min(
        candidates,
        key=lambda lane: min(
            (sample.position.x - pose.position.x) ** 2 + (sample.position.y - pose.position.y) ** 2
            for sample in lane.centerline
        ),
    )


def choose_reachable_goal(
    graph: LaneGraph,
    start_pose: Pose,
    desired_distance_m: float,
    required_maneuver: int | None = None,
) -> tuple[Pose, list[int], float]:
    """BFS lane topology and select a real reachable lane endpoint.

    required_maneuver uses LaneConnection constants (STRAIGHT=0, LEFT=1,
    RIGHT=2); no Town04 coordinates are invented.
    """
    lanes = {int(lane.id): lane for lane in graph.lanes}
    start = nearest_lane(graph, start_pose)
    queue = deque([(int(start.id), [int(start.id)], 0.0, False)])
    visited: set[tuple[int, bool]] = set()
    best: tuple[Any, list[int], float] | None = None
    while queue:
        lane_id, path, distance, seen = queue.popleft()
        lane = lanes.get(lane_id)
        if lane is None or (lane_id, seen) in visited:
            continue
        visited.add((lane_id, seen))
        lane_length = sum(
            math.hypot(b.position.x - a.position.x, b.position.y - a.position.y)
            for a, b in zip(lane.centerline, lane.centerline[1:])
        )
        total = distance + lane_length
        if lane.centerline and (required_maneuver is None or seen):
            best = (lane.centerline[-1], path, total)
            if total >= desired_distance_m:
                return best
        for edge in lane.outgoing:
            target = int(edge.to_lane_id)
            if target in path or target not in lanes:
                continue
            edge_seen = seen or required_maneuver is not None and int(edge.maneuver) == required_maneuver
            queue.append((target, path + [target], total + float(edge.extra_cost_m), edge_seen))
    if best is None:
        qualifier = "" if required_maneuver is None else f" with maneuver {required_maneuver}"
        raise RuntimeError(f"no reachable lane goal{qualifier}")
    return best


class NavigationProbe(Node):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.graph: LaneGraph | None = None
        self.odom: Odometry | None = None
        self.route: GlobalRoute | None = None
        self.routes: list[tuple[float, GlobalRoute]] = []
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LaneGraph, "/adas/map/lane_graph", self._on_graph, transient_qos())
        self.create_subscription(Odometry, "/adas/localization/kinematic_state", self._on_odom, 10)
        self.create_subscription(GlobalRoute, "/adas/navigation/global_route", self._on_route, transient_qos())
        self.goal_pub = self.create_publisher(PoseStamped, "/adas/navigation/goal_pose", reliable_qos())

    def _on_graph(self, msg: LaneGraph) -> None:
        self.graph = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _on_route(self, msg: GlobalRoute) -> None:
        self.route = msg
        self.routes.append((time.time(), msg))

    def prerequisites(self, timeout_s: float) -> tuple[LaneGraph, Odometry]:
        wait_for(self, lambda: self.graph and self.odom, timeout_s, "lane graph and odometry")
        assert self.graph is not None and self.odom is not None
        normalized_map = self.graph.map_id.rstrip("/").split("/")[-1]
        if normalized_map != "Town04":
            raise RuntimeError(f"Phase 3 requires Town04, got {self.graph.map_id!r}")
        return self.graph, self.odom

    def current_pose_map(self, timeout_s: float = 2.0) -> Pose:
        if self.odom is None or self.graph is None:
            raise RuntimeError("odometry and lane graph are required")
        source = PoseStamped()
        source.header = self.odom.header
        source.pose = self.odom.pose.pose
        target = self.graph.header.frame_id or "map"
        if source.header.frame_id == target:
            return source.pose
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            import rclpy

            rclpy.spin_once(self, timeout_sec=0.05)
            if self.tf_buffer.can_transform(target, source.header.frame_id, rclpy.time.Time()):
                return self.tf_buffer.transform(source, target, timeout=Duration(seconds=0.1)).pose
        raise RuntimeError(f"cannot transform vehicle pose {source.header.frame_id!r} -> {target!r}")

    def publish_goal(self, pose: Pose) -> float:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose = pose
        self.goal_pub.publish(msg)
        return time.time()

    def await_new_valid_route(self, previous_id: int, timeout_s: float) -> GlobalRoute:
        return wait_for(
            self,
            lambda: self.route
            if self.route is not None
            and self.route.route_id > previous_id
            and self.route.status == GlobalRoute.STATUS_VALID
            else None,
            timeout_s,
            "new VALID GlobalRoute",
        )


def write_result(path: Path, result: dict[str, Any]) -> int:
    result.setdefault("evidence", "live ROS 2 measurements; no synthetic samples")
    atomic_json(path, result)
    return 0 if result.get("passed") else 1
