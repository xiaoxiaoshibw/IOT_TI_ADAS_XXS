#!/usr/bin/env python3
"""Measure ROS topic rates, jitter, source age and approximate stage latency.

This observer does not modify the stack. Stage latency is calculated from the
latest upstream header stamp not newer than the downstream output stamp. It is
therefore a scheduling/freshness metric, not a distributed tracing substitute.
"""

import argparse
import json
import math
import pathlib
import statistics
import time
from collections import defaultdict, deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from adas_msgs.msg import ActuationCommand, BehaviorState, Control, TrackedObjectArray
from adas_msgs.msg import Trajectory
from nav_msgs.msg import Odometry

from safety_budget import validate_report


TOPICS = {
    "odom": ("/adas/localization/kinematic_state", Odometry, 50.0, True),
    "objects": ("/adas/perception/objects", TrackedObjectArray, 20.0, True),
    "behavior": ("/adas/planning/behavior", BehaviorState, 10.0, False),
    "trajectory": ("/adas/planning/trajectory", Trajectory, 20.0, False),
    "follower": ("/adas/control/trajectory_follower/control_cmd", Control, 50.0, False),
    "gate": ("/adas/control/gate/control_cmd", Control, 50.0, False),
    "actuation": ("/adas/vehicle/actuation_cmd", ActuationCommand, 50.0, False),
}

STAGES = (
    ("objects_to_behavior", "objects", "behavior"),
    ("behavior_to_trajectory", "behavior", "trajectory"),
    ("trajectory_to_follower", "trajectory", "follower"),
    ("follower_to_gate", "follower", "gate"),
    ("gate_to_actuation", "gate", "actuation"),
)


def percentile(values, q):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * q
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def summarize_ms(values):
    if not values:
        return {"samples": 0}
    scaled = [value * 1000.0 for value in values]
    return {
        "samples": len(scaled),
        "mean_ms": statistics.fmean(scaled),
        "p50_ms": percentile(scaled, 0.50),
        "p95_ms": percentile(scaled, 0.95),
        "p99_ms": percentile(scaled, 0.99),
        "max_ms": max(scaled),
    }


class RuntimeMonitor(Node):
    def __init__(self, warmup_s):
        super().__init__("adas_runtime_performance_monitor")
        self.started = time.monotonic()
        self.warmup_s = warmup_s
        self.last_receive = {}
        self.intervals = defaultdict(list)
        self.source_ages = defaultdict(list)
        self.stamps = {name: deque(maxlen=512) for name in TOPICS}
        self.stage_latency = defaultdict(list)
        self.last_stage_upstream = {}
        for name, (topic, msg_type, _, sensor_qos) in TOPICS.items():
            qos = qos_profile_sensor_data if sensor_qos else 100
            self.create_subscription(msg_type, topic, self._callback(name), qos)

    def collecting(self):
        return time.monotonic() - self.started >= self.warmup_s

    def _callback(self, name):
        def receive(msg):
            received = time.monotonic()
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            now_ros = self.get_clock().now().nanoseconds * 1e-9
            if self.collecting():
                previous = self.last_receive.get(name)
                if previous is not None:
                    self.intervals[name].append(received - previous)
                age = now_ros - stamp
                if 0.0 <= age < 10.0:
                    self.source_ages[name].append(age)
                for stage, upstream, downstream in STAGES:
                    if downstream != name:
                        continue
                    candidates = self.stamps[upstream]
                    if candidates:
                        upstream_stamp = max((s for s in candidates if s <= stamp), default=None)
                        if (upstream_stamp is not None and stamp - upstream_stamp < 2.0 and
                                self.last_stage_upstream.get(stage) != upstream_stamp):
                            self.stage_latency[stage].append(stamp - upstream_stamp)
                            self.last_stage_upstream[stage] = upstream_stamp
            self.last_receive[name] = received
            self.stamps[name].append(stamp)

        return receive

    def report(self):
        report = {"topics": {}, "stages": {}}
        for name, (_, _, expected_hz, _) in TOPICS.items():
            intervals = self.intervals[name]
            mean_period = statistics.fmean(intervals) if intervals else 0.0
            report["topics"][name] = {
                "expected_hz": expected_hz,
                "measured_hz": 1.0 / mean_period if mean_period > 0.0 else 0.0,
                "period": summarize_ms(intervals),
                "source_age": summarize_ms(self.source_ages[name]),
            }
        for stage, _, _ in STAGES:
            report["stages"][stage] = summarize_ms(self.stage_latency[stage])
        return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--warmup", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--enforce", action="store_true")
    parser.add_argument("--output", type=pathlib.Path,
                        help="write the evidence JSON to this file")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = RuntimeMonitor(args.warmup)
    deadline = time.monotonic() + args.warmup + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        report = node.report()
        failures = validate_report(report) if args.enforce else []
        report["verdict"] = "PASS" if not failures else "FAIL"
        report["failures"] = failures
        rendered = json.dumps(report, ensure_ascii=False, indent=2)
        print(rendered)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(rendered + "\n", encoding="utf-8")
        if failures:
            raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
