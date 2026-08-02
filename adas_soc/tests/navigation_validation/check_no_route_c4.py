#!/usr/bin/env python3
"""Evaluate the Phase 3.4 no-route C4 gate from recorded, read-only evidence."""

import argparse
import csv
import json
import math
from pathlib import Path


def rows(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def numbers(records, field):
    return [float(row[field]) for row in records if row.get(field, "") != ""]


def evaluate(timeline_path, carla_path, metrics_dir):
    timeline = rows(timeline_path)
    carla = rows(carla_path)
    metrics_dir = Path(metrics_dir)
    mcu = rows(metrics_dir / "mcu_control.csv")
    can = rows(metrics_dir / "can_metrics.csv")
    vehicle = rows(metrics_dir / "vehicle_state.csv")

    by_topic = {}
    for row in timeline:
        by_topic.setdefault(row["topic"], []).append(row)

    route = by_topic.get("/adas/navigation/global_route", [])
    path = by_topic.get("/adas/planning/global_route", [])
    trajectory = by_topic.get("/adas/planning/trajectory", [])
    behavior = by_topic.get("/adas/planning/behavior", [])
    gate_status = by_topic.get("/adas/control/gate/status", [])
    gate_cmd = by_topic.get("/adas/control/gate/control_cmd", [])
    actuation = by_topic.get("/adas/vehicle/actuation_cmd", [])

    # The generic collector writes an all-zero placeholder after the bridge
    # destroys the hero at a planned-duration exit.  It is actor absence, not
    # a teleport, and must not be folded into the in-run displacement.
    vehicle_present = [row for row in vehicle
                       if abs(float(row["x"])) + abs(float(row["y"])) > 1e-9]
    x = numbers(vehicle_present, "x")
    y = numbers(vehicle_present, "y")
    displacement = max((math.hypot(px - x[0], py - y[0])
                        for px, py in zip(x, y)), default=float("inf"))
    speed = numbers(carla, "ego_v")
    carla_throttle = numbers(carla, "throttle")
    carla_brake = numbers(carla, "brake")
    mcu_throttle = numbers(mcu, "throttle")
    mcu_brake = numbers(mcu, "brake")
    berr_tx = numbers(can, "berr_tx")
    berr_rx = numbers(can, "berr_rx")
    errors = numbers(can, "error")

    checks = {
        "global_route_invalid_empty": bool(route) and all(
            int(row["route_status"]) != 1 and int(row["point_count"]) == 0
            for row in route),
        "legacy_path_empty": bool(path) and all(int(row["point_count"]) == 0 for row in path),
        "trajectory_empty": bool(trajectory) and all(
            int(row["point_count"]) == 0 and int(row["valid"]) == 0
            for row in trajectory),
        "behavior_stop": bool(behavior) and all(
            float(row["target_speed_mps"]) == 0.0 for row in behavior),
        "gate_builtin_stop": bool(gate_status) and all(
            int(row["control_source"]) == 2 for row in gate_status),
        "gate_non_driving": bool(gate_cmd) and all(
            float(row["velocity_mps"]) == 0.0
            and float(row["acceleration_mps2"]) <= 0.0 for row in gate_cmd),
        "soc_actuation_no_throttle": bool(actuation) and all(
            float(row["throttle"]) == 0.0 for row in actuation),
        "mcu_feedback_stop": bool(mcu_throttle) and max(mcu_throttle) == 0.0
            and min(mcu_brake) >= 0.6,
        "carla_control_stop": bool(carla_throttle) and max(carla_throttle) == 0.0
            and min(carla_brake) >= 0.3,
        "vehicle_speed_static": bool(speed) and max(speed) <= 0.1,
        "vehicle_displacement_static": displacement <= 0.5,
        "can_healthy": bool(can) and max(berr_tx) == 0.0 and max(berr_rx) == 0.0
            and max(errors) == min(errors),
    }
    return {
        "schema": 1,
        "result": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "metrics": {
            "maximum_speed_mps": max(speed, default=None),
            "maximum_displacement_xy_m": displacement,
            "maximum_carla_throttle": max(carla_throttle, default=None),
            "minimum_carla_brake": min(carla_brake, default=None),
            "maximum_mcu_throttle": max(mcu_throttle, default=None),
            "minimum_mcu_brake": min(mcu_brake, default=None),
            "gate_sources": sorted({int(row["control_source"]) for row in gate_status}),
            "berr_tx_max": max(berr_tx, default=None),
            "berr_rx_max": max(berr_rx, default=None),
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeline", required=True)
    parser.add_argument("--carla-csv", required=True)
    parser.add_argument("--metrics-dir", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    report = evaluate(args.timeline, args.carla_csv, args.metrics_dir)
    Path(args.output).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["result"] == "PASS" else 1)


if __name__ == "__main__":
    main()
