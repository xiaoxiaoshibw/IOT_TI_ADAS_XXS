#!/usr/bin/env python3
"""Record the no-route ROS control chain without publishing any message.

Each callback is written immediately with both monotonic and ROS/header time.
The resulting event stream identifies the first non-zero requested speed or
acceleration before correlating it with raw CAN and CARLA CSV timestamps.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path
from typing import Any

import rclpy
from adas_msgs.msg import (
    ActuationCommand,
    BehaviorState,
    Control,
    GateStatus,
    GlobalRoute,
    McuStatus,
    SafetyStatus,
    Trajectory,
)
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


FIELDS = (
    "monotonic_s",
    "wall_time_s",
    "ros_time_s",
    "header_time_s",
    "topic",
    "publisher_nodes",
    "route_id",
    "route_status",
    "point_count",
    "behavior_state",
    "target_speed_mps",
    "control_source",
    "velocity_mps",
    "acceleration_mps2",
    "steering",
    "throttle",
    "brake",
    "valid",
    "fresh",
    "age_ms",
    "sequence",
    "safety_level",
    "mcu_state",
    "mcu_active_source",
)


def reliable_qos(transient: bool = False) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE,
    )


def sensor_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def stamp_seconds(message: Any) -> float | None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def event_payload(topic: str, message: Any) -> dict[str, Any]:
    """Extract comparable numeric fields without inventing missing values."""
    row: dict[str, Any] = {field: "" for field in FIELDS}
    row["topic"] = topic
    header_s = stamp_seconds(message)
    row["header_time_s"] = "" if header_s is None else f"{header_s:.9f}"

    if isinstance(message, GlobalRoute):
        row.update(
            route_id=int(message.route_id),
            route_status=int(message.status),
            point_count=len(message.points),
            valid=int(message.status == GlobalRoute.STATUS_VALID and bool(message.points)),
        )
    elif isinstance(message, PathMessage):
        row.update(point_count=len(message.poses), valid=int(bool(message.poses)))
    elif isinstance(message, BehaviorState):
        row.update(
            behavior_state=int(message.state),
            target_speed_mps=f"{float(message.target_speed_mps):.6f}",
            valid=int(math.isfinite(float(message.target_speed_mps))),
        )
    elif isinstance(message, Trajectory):
        row["point_count"] = len(message.points)
        if message.points:
            velocities = [float(point.longitudinal_velocity_mps) for point in message.points]
            row["target_speed_mps"] = f"{max(velocities):.6f}"
            row["valid"] = int(all(math.isfinite(value) for value in velocities))
        else:
            row.update(target_speed_mps="0.000000", valid=0)
    elif isinstance(message, Control):
        values = (
            float(message.lateral.steering_tire_angle_rad),
            float(message.longitudinal.velocity_mps),
            float(message.longitudinal.acceleration_mps2),
        )
        row.update(
            steering=f"{values[0]:.7f}",
            velocity_mps=f"{values[1]:.6f}",
            acceleration_mps2=f"{values[2]:.6f}",
            valid=int(all(math.isfinite(value) for value in values)),
        )
    elif isinstance(message, ActuationCommand):
        values = (float(message.steer), float(message.throttle), float(message.brake))
        row.update(
            steering=f"{values[0]:.7f}",
            throttle=f"{values[1]:.6f}",
            brake=f"{values[2]:.6f}",
            valid=int(all(math.isfinite(value) for value in values)),
        )
    elif isinstance(message, GateStatus):
        row.update(control_source=int(message.selected_source), valid=1)
    elif isinstance(message, SafetyStatus):
        row.update(safety_level=int(message.overall), valid=1)
    elif isinstance(message, McuStatus):
        row.update(
            mcu_state=int(message.system_state),
            mcu_active_source=int(message.active_source),
            age_ms=f"{float(message.command_age_s) * 1000.0:.3f}",
            fresh=int(bool(message.primary_fresh or message.backup_fresh)),
            valid=int(bool(message.protocol_version_ok)),
        )
    elif isinstance(message, Odometry):
        velocity = float(message.twist.twist.linear.x)
        row.update(velocity_mps=f"{velocity:.6f}", valid=int(math.isfinite(velocity)))
    return row


class NoRouteControlAudit(Node):
    def __init__(self, output: Path, duration_s: float) -> None:
        super().__init__("phase3_4_no_route_control_audit")
        self.output = output
        self.duration_s = duration_s
        self.started_monotonic = time.monotonic()
        self.events = 0
        self.first_nonzero: dict[str, Any] | None = None
        output.parent.mkdir(parents=True, exist_ok=True)
        self.stream = output.open("x", encoding="utf-8", newline="")
        self.writer = csv.DictWriter(self.stream, fieldnames=FIELDS)
        self.writer.writeheader()

        subscriptions = (
            (GlobalRoute, "/adas/navigation/global_route", reliable_qos(True)),
            (PathMessage, "/adas/planning/global_route", reliable_qos(True)),
            (BehaviorState, "/adas/planning/behavior", reliable_qos()),
            (Trajectory, "/adas/planning/trajectory", reliable_qos()),
            (Control, "/adas/control/trajectory_follower/control_cmd", reliable_qos()),
            (Control, "/adas/control/aeb/emergency_cmd", reliable_qos()),
            (Control, "/adas/control/gate/control_cmd", reliable_qos()),
            (GateStatus, "/adas/control/gate/status", reliable_qos(True)),
            (SafetyStatus, "/adas/system/safety_status", reliable_qos(True)),
            (ActuationCommand, "/adas/vehicle/actuation_cmd", reliable_qos()),
            (ActuationCommand, "/adas/mcu/actuation_feedback", reliable_qos()),
            (McuStatus, "/adas/mcu/status", reliable_qos()),
            (Odometry, "/adas/localization/kinematic_state", sensor_qos()),
        )
        self._subscriptions = [
            self.create_subscription(
                message_type,
                topic,
                lambda message, topic=topic: self.record(topic, message),
                qos,
            )
            for message_type, topic, qos in subscriptions
        ]
        self.create_timer(0.1, self.check_duration)

    def publisher_names(self, topic: str) -> str:
        names = {
            f"{info.node_namespace.rstrip('/')}/{info.node_name}"
            for info in self.get_publishers_info_by_topic(topic)
        }
        return ";".join(sorted(name.replace("//", "/") for name in names))

    @staticmethod
    def is_nonzero(row: dict[str, Any]) -> bool:
        for field in ("target_speed_mps", "velocity_mps", "throttle"):
            value = row.get(field, "")
            if value != "" and float(value) > 1e-6:
                return True
        value = row.get("acceleration_mps2", "")
        return value != "" and float(value) > 1e-6

    def record(self, topic: str, message: Any) -> None:
        now_monotonic = time.monotonic()
        row = event_payload(topic, message)
        row.update(
            monotonic_s=f"{now_monotonic:.9f}",
            wall_time_s=f"{time.time():.9f}",
            ros_time_s=f"{self.get_clock().now().nanoseconds * 1e-9:.9f}",
            publisher_nodes=self.publisher_names(topic),
        )
        if self.first_nonzero is None and self.is_nonzero(row):
            self.first_nonzero = dict(row)
        self.writer.writerow(row)
        self.events += 1
        self.stream.flush()

    def check_duration(self) -> None:
        if time.monotonic() - self.started_monotonic >= self.duration_s:
            rclpy.shutdown()

    def close(self) -> None:
        if not self.stream.closed:
            self.stream.flush()
            self.stream.close()
        summary = {
            "schema": 1,
            "read_only": True,
            "duration_s": time.monotonic() - self.started_monotonic,
            "event_count": self.events,
            "first_nonzero": self.first_nonzero,
        }
        summary_path = self.output.with_suffix(".summary.json")
        with summary_path.open("x", encoding="utf-8") as stream:
            json.dump(summary, stream, ensure_ascii=False, indent=2)
            stream.write("\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration", type=float, default=30.0)
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    if args.output.exists() or args.output.with_suffix(".summary.json").exists():
        parser.error("refusing to overwrite an existing audit artifact")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = NoRouteControlAudit(args.output, args.duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
