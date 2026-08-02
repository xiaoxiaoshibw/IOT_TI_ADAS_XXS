#!/usr/bin/env python3
"""Verify VALID->CANCELLED invalidation and downstream smooth stop on real HIL."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import rclpy
from adas_msgs.msg import BehaviorState, GlobalRoute, Trajectory
from std_msgs.msg import Empty

from ros_validation import NavigationProbe, choose_reachable_goal, reliable_qos, wait_for, write_result

__test__ = False


class CancelProbe(NavigationProbe):
    def __init__(self) -> None:
        super().__init__("phase3_cancel_route_test")
        self.behavior = None
        self.trajectory = None
        self.cancelled_received_wall = None
        self.create_subscription(BehaviorState, "/adas/planning/behavior", self._behavior, reliable_qos())
        self.create_subscription(Trajectory, "/adas/planning/trajectory", self._trajectory, reliable_qos())
        self.cancel_pub = self.create_publisher(Empty, "/adas/navigation/cancel", reliable_qos())

    def _behavior(self, msg):
        self.behavior = msg

    def _trajectory(self, msg):
        self.trajectory = msg

    def _on_route(self, msg: GlobalRoute) -> None:
        super()._on_route(msg)
        if msg.status == GlobalRoute.STATUS_CANCELLED:
            self.cancelled_received_wall = time.time()

    def speed(self) -> float:
        if self.odom is None:
            return -1.0
        v = self.odom.twist.twist.linear
        return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--distance", type=float, default=100.0)
    parser.add_argument("--moving-speed", type=float, default=0.5)
    parser.add_argument("--stopped-speed", type=float, default=0.1)
    args = parser.parse_args()
    rclpy.init()
    node = CancelProbe()
    result = {"test": "navigation cancel safety", "passed": False}
    try:
        graph, _ = node.prerequisites(args.timeout)
        goal, _, _ = choose_reachable_goal(graph, node.current_pose_map(), args.distance, 0)
        previous = int(node.route.route_id) if node.route else 0
        node.publish_goal(goal)
        valid = node.await_new_valid_route(previous, args.timeout)
        wait_for(node, lambda: node.speed() >= args.moving_speed, args.timeout, "vehicle motion before cancel")
        cancel_sent = time.time()
        node.cancel_pub.publish(Empty())
        cancelled = wait_for(
            node,
            lambda: node.route
            if node.route
            and node.route.route_id > valid.route_id
            and node.route.status == GlobalRoute.STATUS_CANCELLED
            else None,
            args.timeout,
            "CANCELLED route",
        )
        behavior = wait_for(
            node,
            lambda: node.behavior if node.behavior and node.behavior.state == BehaviorState.STATE_STOPPING else None,
            args.timeout,
            "STATE_STOPPING",
        )
        trajectory = wait_for(
            node,
            lambda: node.trajectory
            if node.trajectory
            and node.trajectory.points
            and abs(node.trajectory.points[-1].longitudinal_velocity_mps) <= args.stopped_speed
            else None,
            args.timeout,
            "terminal stopping trajectory",
        )
        stop_wall = time.time() if wait_for(node, lambda: node.speed() <= args.stopped_speed, args.timeout, "vehicle stop") is not None else None
        route_cleared = not cancelled.points
        result.update(
            passed=route_cleared and behavior is not None and trajectory is not None,
            map_id=graph.map_id,
            valid_route_id=int(valid.route_id),
            cancelled_route_id=int(cancelled.route_id),
            cancelled_points=len(cancelled.points),
            cancel_timestamp=cancel_sent,
            cancelled_route_latency_s=(node.cancelled_received_wall - cancel_sent) if node.cancelled_received_wall else None,
            stop_timestamp=stop_wall,
            stopping_latency_s=(stop_wall - cancel_sent) if stop_wall else None,
            terminal_trajectory_speed_mps=float(trajectory.points[-1].longitudinal_velocity_mps),
        )
    except Exception as exc:
        result["error"] = str(exc)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return write_result(args.output, result)


if __name__ == "__main__":
    raise SystemExit(main())
