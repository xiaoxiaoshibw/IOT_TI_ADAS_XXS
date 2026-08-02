#!/usr/bin/env python3
"""Validate semantic fields of a live Town04 GlobalRoute and export topology."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import rclpy

from ros_validation import NavigationProbe, wait_for, write_result
from validation_common import ROUTE_MANEUVER_NAMES, validate_route

__test__ = False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--topology-csv", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    rclpy.init()
    node = NavigationProbe("phase3_semantic_route_test")
    result = {"test": "Town04 lane semantic navigation", "passed": False}
    try:
        graph, _ = node.prerequisites(args.timeout)
        route = wait_for(
            node,
            lambda: node.route if node.route and node.route.status == node.route.STATUS_VALID else None,
            args.timeout,
            "VALID GlobalRoute",
        )
        check = validate_route(route)
        transitions = sum(
            1 for a, b in zip(route.points, route.points[1:]) if a.lane_id != b.lane_id
        )
        args.topology_csv.parent.mkdir(parents=True, exist_ok=True)
        with args.topology_csv.open("x", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(("index", "x", "y", "yaw", "lane_id", "road_id", "speed_limit", "maneuver"))
            for index, point in enumerate(route.points):
                writer.writerow((index, point.x, point.y, point.yaw, point.lane_id, point.road_id, point.speed_limit, ROUTE_MANEUVER_NAMES.get(point.maneuver, point.maneuver)))
        result.update(
            passed=check.ok,
            map_id=graph.map_id,
            route_id=int(route.route_id),
            point_count=len(route.points),
            lane_transition_count=transitions,
            road_ids=sorted({int(point.road_id) for point in route.points}),
            maneuver_counts={
                name: sum(1 for p in route.points if ROUTE_MANEUVER_NAMES.get(p.maneuver) == name)
                for name in ROUTE_MANEUVER_NAMES.values()
            },
            errors=list(check.errors),
            warnings=list(check.warnings),
            route_length_m=check.route_length_m,
            maximum_adjacent_gap_m=check.maximum_adjacent_gap_m,
            maximum_heading_jump_rad=check.maximum_heading_jump_rad,
            maximum_reverse_progress_m=check.maximum_reverse_progress_m,
            largest_gap_point_index=check.largest_gap_point_index,
            topology_csv=str(args.topology_csv),
        )
    except Exception as exc:
        result["error"] = str(exc)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return write_result(args.output, result)


if __name__ == "__main__":
    raise SystemExit(main())
