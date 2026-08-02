#!/usr/bin/env python3
"""Require finite map->odom->base_link transforms from the live Town04 stack."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from ros_validation import write_result

__test__ = False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()
    rclpy.init()
    node = Node("phase3_tf_chain_test")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    # Keep the listener alive for the full lookup window.
    result = {"test": "TF chain map-odom-base_link", "passed": False, "transforms": []}
    try:
        deadline = time.monotonic() + args.timeout
        pairs = (("map", "odom"), ("odom", "base_link"))
        transforms = []
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(buffer.can_transform(a, b, rclpy.time.Time(), timeout=Duration()) for a, b in pairs):
                transforms = [buffer.lookup_transform(a, b, rclpy.time.Time()) for a, b in pairs]
                break
        if not transforms:
            raise TimeoutError("required TF chain not available")
        finite = True
        for transform in transforms:
            t = transform.transform.translation
            q = transform.transform.rotation
            values = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
            finite &= all(math.isfinite(float(v)) for v in values)
            result["transforms"].append(
                {"parent": transform.header.frame_id, "child": transform.child_frame_id, "values": list(values)}
            )
        result["passed"] = finite
        if not finite:
            result["error"] = "TF contains non-finite values"
    except Exception as exc:
        result["error"] = str(exc)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return write_result(args.output, result)


if __name__ == "__main__":
    raise SystemExit(main())
