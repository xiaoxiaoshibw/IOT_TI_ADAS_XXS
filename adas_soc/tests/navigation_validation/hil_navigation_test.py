#!/usr/bin/env python3
"""Validate that Town04 navigation traverses the complete SoC->MCU HIL chain."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import rclpy
from adas_msgs.msg import GateStatus, McuStatus, NavigationStatus, Trajectory

from ros_validation import NavigationProbe, choose_reachable_goal, reliable_qos, write_result
from validation_common import validate_route

__test__ = False


class HilProbe(NavigationProbe):
    def __init__(self) -> None:
        super().__init__("phase3_hil_navigation_test")
        self.trajectory = None
        self.gate = None
        self.mcu = None
        self.navigation = None
        self.poses = []
        self.samples = 0
        self.full_chain_samples = 0
        self.create_subscription(Trajectory, "/adas/planning/trajectory", self._trajectory, reliable_qos())
        self.create_subscription(GateStatus, "/adas/control/gate/status", self._gate, reliable_qos())
        self.create_subscription(McuStatus, "/adas/mcu/status", self._mcu, reliable_qos())
        self.create_subscription(NavigationStatus, "/adas/navigation/status", self._navigation, reliable_qos())
        self.create_timer(0.05, self._sample)

    def _trajectory(self, msg):
        self.trajectory = msg

    def _gate(self, msg):
        self.gate = msg

    def _mcu(self, msg):
        self.mcu = msg

    def _navigation(self, msg):
        self.navigation = msg

    def _sample(self):
        self.samples += 1
        if self.odom:
            p = self.odom.pose.pose.position
            self.poses.append((time.time(), float(p.x), float(p.y)))
        if (
            self.trajectory
            and self.trajectory.points
            and self.gate
            and self.gate.selected_source == GateStatus.SOURCE_FOLLOWER
            and self.mcu
            and self.mcu.system_state == McuStatus.SYS_ACTIVE
            and self.mcu.active_source == McuStatus.SRC_PRIMARY
            and self.mcu.feedback_age_s >= 0.0
        ):
            self.full_chain_samples += 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--distance", type=float, default=500.0)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--completion-ratio", type=float, default=0.8)
    parser.add_argument("--maneuver", type=int, default=0, choices=range(5),
                        help="required LaneConnection maneuver (0..4)")
    args = parser.parse_args()
    rclpy.init()
    node = HilProbe()
    result = {"test": "Town04 real HIL navigation", "passed": False}
    try:
        graph, _ = node.prerequisites(30.0)
        goal, lane_path, topology_distance = choose_reachable_goal(
            graph, node.current_pose_map(), args.distance, args.maneuver)
        previous = int(node.route.route_id) if node.route else 0
        sent = node.publish_goal(goal)
        route = node.await_new_valid_route(previous, 30.0)
        route_check = validate_route(route)
        deadline = time.monotonic() + args.timeout
        endpoint_distance = math.inf
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.odom:
                p = node.current_pose_map(0.5).position
                endpoint_distance = math.hypot(p.x - goal.position.x, p.y - goal.position.y)
                if endpoint_distance <= 2.0:
                    break
            if node.navigation and node.navigation.state == NavigationStatus.FAILED:
                break
        travelled = sum(
            math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(node.poses, node.poses[1:])
        )
        required_travel = min(float(route.length), topology_distance) * args.completion_ratio
        chain_ratio = node.full_chain_samples / node.samples if node.samples else 0.0
        arrived = endpoint_distance <= 2.0 or bool(
            node.navigation and node.navigation.state == NavigationStatus.ARRIVED
        )
        expected_route_maneuver = args.maneuver + 1
        semantic_maneuver_found = any(
            int(point.maneuver) == expected_route_maneuver for point in route.points)
        result.update(
            passed=(route_check.ok and semantic_maneuver_found
                    and topology_distance >= args.distance * 0.95
                    and arrived and travelled >= required_travel and chain_ratio >= 0.95),
            map_id=graph.map_id,
            route_id=int(route.route_id),
            route_length_m=float(route.length),
            topology_distance_m=topology_distance,
            topology_lane_path=lane_path,
            required_edge_maneuver=args.maneuver,
            expected_route_maneuver=expected_route_maneuver,
            semantic_maneuver_found=semantic_maneuver_found,
            goal_sent_timestamp=sent,
            elapsed_s=time.time() - sent,
            travelled_distance_m=travelled,
            required_travel_distance_m=required_travel,
            endpoint_distance_m=endpoint_distance if math.isfinite(endpoint_distance) else None,
            arrived=arrived,
            full_chain_sample_ratio=chain_ratio,
            full_chain_definition="trajectory + Gate follower + MCU ACTIVE/PRIMARY + fresh 0x201 feedback",
            route_errors=list(route_check.errors),
            runtime_navigation_state=(int(node.navigation.state) if node.navigation else None),
        )
    except Exception as exc:
        result["error"] = str(exc)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return write_result(args.output, result)


if __name__ == "__main__":
    raise SystemExit(main())
