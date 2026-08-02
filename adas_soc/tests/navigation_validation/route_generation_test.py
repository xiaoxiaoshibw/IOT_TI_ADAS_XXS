#!/usr/bin/env python3
"""Generate and validate a topology-selected Town04 route."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import rclpy

from ros_validation import NavigationProbe, choose_reachable_goal, write_result
from validation_common import validate_route

__test__ = False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--distance", type=float, default=500.0)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    rclpy.init()
    node = NavigationProbe("phase3_route_generation_test")
    result = {"test": "Town04 route generation", "passed": False}
    try:
        graph, _ = node.prerequisites(args.timeout)
        goal, lane_path, available = choose_reachable_goal(
            graph, node.current_pose_map(), args.distance, required_maneuver=0
        )
        previous = int(node.route.route_id) if node.route else 0
        sent = node.publish_goal(goal)
        route = node.await_new_valid_route(previous, args.timeout)
        check = validate_route(route)
        endpoint_error = ((route.points[-1].x - goal.position.x) ** 2 + (route.points[-1].y - goal.position.y) ** 2) ** 0.5
        result.update(
            passed=check.ok and endpoint_error <= 2.0 and available >= args.distance * 0.95,
            timestamp=time.time(),
            goal_sent_timestamp=sent,
            planning_latency_s=time.time() - sent,
            map_id=graph.map_id,
            map_hash=graph.map_hash,
            requested_distance_m=args.distance,
            topology_distance_m=available,
            route_length_m=float(route.length),
            route_id=int(route.route_id),
            point_count=len(route.points),
            endpoint_error_m=endpoint_error,
            topology_lane_path=lane_path,
            errors=list(check.errors),
            warnings=list(check.warnings),
            maximum_adjacent_gap_m=check.maximum_adjacent_gap_m,
            maximum_heading_jump_rad=check.maximum_heading_jump_rad,
            maximum_reverse_progress_m=check.maximum_reverse_progress_m,
            largest_gap_point_index=check.largest_gap_point_index,
        )
    except Exception as exc:
        result["error"] = str(exc)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return write_result(args.output, result)


if __name__ == "__main__":
    raise SystemExit(main())
