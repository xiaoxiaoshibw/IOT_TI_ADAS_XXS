#!/usr/bin/env python3
"""Exercise LEFT, RIGHT and STRAIGHT routes selected from the live Town04 graph."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import rclpy

from ros_validation import NavigationProbe, choose_reachable_goal, write_result
from validation_common import ROUTE_MANEUVER_NAMES, validate_route

__test__ = False

# LaneConnection maneuver -> RoutePoint maneuver
CASES = (
    ("STRAIGHT", 0, 1),
    ("LEFT", 1, 2),
    ("RIGHT", 2, 3),
    ("LANE_CHANGE_LEFT", 3, 4),
    ("LANE_CHANGE_RIGHT", 4, 5),
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--route-csv", required=True, type=Path)
    parser.add_argument("--distance", type=float, default=80.0)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    rclpy.init()
    node = NavigationProbe("phase3_junction_test")
    result = {"test": "Town04 junction navigation", "passed": False, "cases": []}
    rows = []
    try:
        graph, _ = node.prerequisites(args.timeout)
        for label, edge_maneuver, route_maneuver in CASES:
            # Refresh odometry because the HIL vehicle can move between cases.
            graph, _ = node.prerequisites(args.timeout)
            goal, lane_path, topology_distance = choose_reachable_goal(
                graph, node.current_pose_map(), args.distance, edge_maneuver
            )
            previous = int(node.route.route_id) if node.route else 0
            sent = node.publish_goal(goal)
            route = node.await_new_valid_route(previous, args.timeout)
            check = validate_route(route)
            found = any(int(point.maneuver) == route_maneuver for point in route.points)
            case = {
                "maneuver": label,
                "passed": check.ok and found,
                "route_id": int(route.route_id),
                "planning_latency_s": __import__("time").time() - sent,
                "topology_distance_m": topology_distance,
                "topology_lane_path": lane_path,
                "semantic_maneuver_found": found,
                "errors": list(check.errors),
                "maximum_adjacent_gap_m": check.maximum_adjacent_gap_m,
                "maximum_heading_jump_rad": check.maximum_heading_jump_rad,
                "maximum_reverse_progress_m": check.maximum_reverse_progress_m,
            }
            result["cases"].append(case)
            for index, point in enumerate(route.points):
                rows.append((label, route.route_id, index, point.x, point.y,
                             point.yaw, point.lane_id, point.road_id,
                             point.speed_limit,
                             ROUTE_MANEUVER_NAMES.get(point.maneuver, point.maneuver)))
        result.update(
            passed=all(case["passed"] for case in result["cases"]),
            map_id=graph.map_id,
            route_csv=str(args.route_csv),
        )
        args.route_csv.parent.mkdir(parents=True, exist_ok=True)
        with args.route_csv.open("x", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(("case", "route_id", "index", "x", "y", "yaw",
                             "lane_id", "road_id", "speed_limit", "maneuver"))
            writer.writerows(rows)
    except Exception as exc:
        result["error"] = str(exc)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return write_result(args.output, result)


if __name__ == "__main__":
    raise SystemExit(main())
