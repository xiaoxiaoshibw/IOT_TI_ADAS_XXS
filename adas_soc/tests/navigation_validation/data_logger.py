#!/usr/bin/env python3
"""Record synchronized Phase 3 evidence from the live ROS 2 HIL graph.

Missing inputs are written as empty cells.  The logger never synthesizes a
measurement and never publishes control, route, or fault-injection messages.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped transforms with tf2
from adas_msgs.msg import (
    ActuationCommand,
    BehaviorState,
    Control,
    GateStatus,
    GlobalRoute,
    LaneState,
    McuStatus,
    NavigationStatus,
    SafetyStatus,
    SteeringReport,
    Trajectory,
    TrackedObjectArray,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

from validation_common import atomic_json, nearest_route_point, quaternion_yaw


FIELDS = (
    "timestamp",
    "elapsed_s",
    "vehicle_x",
    "vehicle_y",
    "vehicle_yaw",
    "vehicle_frame",
    "vehicle_odom_x",
    "vehicle_odom_y",
    "velocity",
    "steering",
    "route_id",
    "route_status",
    "lane_id",
    "road_id",
    "maneuver",
    "route_x",
    "route_y",
    "speed_limit",
    "target_speed",
    "lateral_error",
    "speed_error",
    "behavior_state",
    "control_source",
    "lead_id",
    "lead_gap_m",
    "lead_speed_mps",
    "nav_state",
    "nav_distance_to_goal_m",
    "safety_state",
    "safety_fault_count",
    "gate_longitudinal_accel_mps2",
    "lane_valid",
    "lane_lateral_offset_m",
    "lane_heading_error_rad",
    "lane_curvature",
    "throttle",
    "brake",
    "mcu_state",
    "mcu_active_source",
    "heartbeat_age",
    "crc_error",
    "can_drop",
)


def sensor_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def reliable_qos(transient: bool = False) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=(DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE),
    )


class HilDataLogger(Node):
    def __init__(self, output: Path, rate_hz: float, duration_s: float | None) -> None:
        super().__init__("phase3_hil_data_logger")
        self.output = output
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.metadata_path = output.with_suffix(".metadata.json")
        self.started_wall = time.time()
        self.duration_s = duration_s
        self.samples = 0
        self.received: set[str] = set()
        self.route: GlobalRoute | None = None
        self.odom: Odometry | None = None
        self.steering: SteeringReport | None = None
        self.trajectory: Trajectory | None = None
        self.behavior: BehaviorState | None = None
        self.gate: GateStatus | None = None
        self.mcu: McuStatus | None = None
        self.objects: TrackedObjectArray | None = None
        self.nav_status: NavigationStatus | None = None
        self.safety_status: SafetyStatus | None = None
        self.gate_cmd: Control | None = None
        self.actuation: ActuationCommand | None = None
        self.lane_state: LaneState | None = None
        self.route_cursor = 0
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.stream = output.open("x", encoding="utf-8", newline="")
        self.writer = csv.DictWriter(self.stream, fieldnames=FIELDS)
        self.writer.writeheader()

        self.create_subscription(Odometry, "/adas/localization/kinematic_state", self._odom, sensor_qos())
        self.create_subscription(SteeringReport, "/adas/vehicle/steering_report", self._steering, sensor_qos())
        self.create_subscription(GlobalRoute, "/adas/navigation/global_route", self._route, reliable_qos(True))
        self.create_subscription(Trajectory, "/adas/planning/trajectory", self._trajectory, reliable_qos())
        self.create_subscription(BehaviorState, "/adas/planning/behavior", self._behavior, reliable_qos())
        self.create_subscription(GateStatus, "/adas/control/gate/status", self._gate, reliable_qos(True))
        self.create_subscription(McuStatus, "/adas/mcu/status", self._mcu, reliable_qos())
        # Commit 1a — additional topics required for the validation plan:
        # lead selection (objects), navigation status, safety state, gate
        # longitudinal output, throttle/brake actuation.
        self.create_subscription(TrackedObjectArray, "/adas/perception/objects", self._objects, sensor_qos())
        self.create_subscription(NavigationStatus, "/adas/navigation/status", self._nav_status, reliable_qos(True))
        self.create_subscription(SafetyStatus, "/adas/system/safety_status", self._safety_status, reliable_qos(True))
        self.create_subscription(Control, "/adas/control/gate/control_cmd", self._gate_cmd, reliable_qos())
        self.create_subscription(ActuationCommand, "/adas/vehicle/actuation_cmd", self._actuation, reliable_qos())
        self.create_subscription(LaneState, "/adas/perception/lane_state", self._lane_state, sensor_qos())
        self.create_timer(1.0 / rate_hz, self._sample)

    def _store(self, name: str, value: Any) -> None:
        setattr(self, name, value)
        self.received.add(name)

    def _odom(self, msg: Odometry) -> None:
        self._store("odom", msg)

    def _steering(self, msg: SteeringReport) -> None:
        self._store("steering", msg)

    def _route(self, msg: GlobalRoute) -> None:
        if self.route is None or msg.route_id != self.route.route_id:
            self.route_cursor = 0
        self._store("route", msg)

    def _trajectory(self, msg: Trajectory) -> None:
        self._store("trajectory", msg)

    def _behavior(self, msg: BehaviorState) -> None:
        self._store("behavior", msg)

    def _gate(self, msg: GateStatus) -> None:
        self._store("gate", msg)

    def _mcu(self, msg: McuStatus) -> None:
        self._store("mcu", msg)

    def _objects(self, msg: TrackedObjectArray) -> None:
        self._store("objects", msg)

    def _nav_status(self, msg: NavigationStatus) -> None:
        self._store("nav_status", msg)

    def _safety_status(self, msg: SafetyStatus) -> None:
        self._store("safety_status", msg)

    def _gate_cmd(self, msg: Control) -> None:
        self._store("gate_cmd", msg)

    def _actuation(self, msg: ActuationCommand) -> None:
        self._store("actuation", msg)

    def _lane_state(self, msg: LaneState) -> None:
        self._store("lane_state", msg)

    @staticmethod
    def _blank_row() -> dict[str, Any]:
        return {field: "" for field in FIELDS}

    def _sample(self) -> None:
        elapsed = time.time() - self.started_wall
        row = self._blank_row()
        row["timestamp"] = f"{time.time():.6f}"
        row["elapsed_s"] = f"{elapsed:.6f}"
        speed: float | None = None
        x: float | None = None
        y: float | None = None
        odom_x: float | None = None
        odom_y: float | None = None
        if self.odom is not None:
            pose = self.odom.pose.pose
            twist = self.odom.twist.twist.linear
            odom_x, odom_y = float(pose.position.x), float(pose.position.y)
            speed = math.sqrt(float(twist.x) ** 2 + float(twist.y) ** 2 + float(twist.z) ** 2)
            row.update(
                vehicle_odom_x=f"{odom_x:.9f}",
                vehicle_odom_y=f"{odom_y:.9f}",
                velocity=f"{speed:.6f}",
            )
            stamped = PoseStamped()
            stamped.header = self.odom.header
            stamped.pose = pose
            try:
                mapped = (
                    stamped
                    if stamped.header.frame_id == "map"
                    else self.tf_buffer.transform(stamped, "map", timeout=Duration(seconds=0.01))
                )
                x, y = float(mapped.pose.position.x), float(mapped.pose.position.y)
                row.update(
                    vehicle_x=f"{x:.9f}",
                    vehicle_y=f"{y:.9f}",
                    vehicle_yaw=f"{quaternion_yaw(mapped.pose.orientation):.9f}",
                    vehicle_frame="map",
                )
            except TransformException:
                # The raw odom coordinates remain available, but route-relative
                # fields stay empty rather than mixing coordinate frames.
                pass
        if self.steering is not None:
            row["steering"] = f"{float(self.steering.steering_tire_angle_rad):.7f}"
        if self.route is not None:
            row["route_id"] = int(self.route.route_id)
            row["route_status"] = int(self.route.status)
            if self.route.status == GlobalRoute.STATUS_VALID and self.route.points and x is not None and y is not None:
                self.route_cursor, point, lateral = nearest_route_point(
                    self.route.points, x, y, max(0, self.route_cursor - 10)
                )
                row.update(
                    lane_id=int(point.lane_id),
                    road_id=int(point.road_id),
                    maneuver=int(point.maneuver),
                    route_x=f"{float(point.x):.9f}",
                    route_y=f"{float(point.y):.9f}",
                    speed_limit=f"{float(point.speed_limit):.6f}",
                    lateral_error=f"{lateral:.6f}",
                )
        target_speed: float | None = None
        if self.trajectory is not None and self.trajectory.points:
            if odom_x is not None and odom_y is not None:
                nearest = min(
                    self.trajectory.points,
                    key=lambda p: (float(p.pose.position.x) - odom_x) ** 2 + (float(p.pose.position.y) - odom_y) ** 2,
                )
            else:
                nearest = self.trajectory.points[0]
            target_speed = float(nearest.longitudinal_velocity_mps)
            row["target_speed"] = f"{target_speed:.6f}"
        if speed is not None and target_speed is not None:
            row["speed_error"] = f"{speed - target_speed:.6f}"
        if self.behavior is not None:
            row["behavior_state"] = int(self.behavior.state)
        if self.gate is not None:
            row["control_source"] = int(self.gate.selected_source)
        if self.objects is not None and int(self.objects.primary_lead_id) >= 0:
            row["lead_id"] = int(self.objects.primary_lead_id)
            row["lead_gap_m"] = f"{float(self.objects.primary_lead_gap_m):.3f}"
            row["lead_speed_mps"] = f"{float(self.objects.primary_lead_speed_mps):.3f}"
        if self.nav_status is not None:
            row["nav_state"] = int(self.nav_status.state)
            row["nav_distance_to_goal_m"] = (
                f"{float(self.nav_status.remaining_distance_m):.3f}"
            )
        if self.safety_status is not None:
            row["safety_state"] = int(self.safety_status.overall)
            row["safety_fault_count"] = len(self.safety_status.failed_components)
        if self.gate_cmd is not None:
            row["gate_longitudinal_accel_mps2"] = (
                f"{float(self.gate_cmd.longitudinal.acceleration_mps2):.3f}"
            )
        # `lane_valid` and lane measurements come from /adas/perception/lane_state.
        if self.lane_state is not None:
            row["lane_valid"] = int(bool(self.lane_state.valid))
            row["lane_lateral_offset_m"] = f"{float(self.lane_state.lateral_offset):.4f}"
            row["lane_heading_error_rad"] = f"{float(self.lane_state.heading_error):.4f}"
            row["lane_curvature"] = f"{float(self.lane_state.curvature):.6f}"
        if self.actuation is not None:
            row["throttle"] = f"{float(self.actuation.throttle):.4f}"
            row["brake"] = f"{float(self.actuation.brake):.4f}"
        if self.mcu is not None:
            row.update(
                mcu_state=int(self.mcu.system_state),
                mcu_active_source=int(self.mcu.active_source),
                heartbeat_age=f"{float(self.mcu.heartbeat_age_s):.6f}",
                crc_error=int(self.mcu.crc_error_count),
            )
        # can_drop requires raw bus observation; it is intentionally left empty here
        # and is supplied by the existing PC tools/hil collector during analysis.
        self.writer.writerow(row)
        self.samples += 1
        if self.samples % 20 == 0:
            self.stream.flush()
        if self.duration_s is not None and elapsed >= self.duration_s:
            rclpy.shutdown()

    def close(self) -> None:
        if self.stream.closed:
            return
        self.stream.flush()
        self.stream.close()
        required = {"odom", "route", "trajectory", "behavior", "gate", "mcu"}
        atomic_json(
            self.metadata_path,
            {
                "schema": 1,
                "csv": str(self.output),
                "samples": self.samples,
                "duration_s": time.time() - self.started_wall,
                "received_inputs": sorted(self.received),
                "missing_required_inputs": sorted(required - self.received),
                "can_drop_source": "TODO unless merged from raw PC HIL CAN collector",
                "real_measurements_only": True,
            },
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=None)
    args = parser.parse_args()
    if args.rate <= 0.0 or args.duration is not None and args.duration <= 0.0:
        parser.error("rate and duration must be positive")
    if args.output.exists():
        parser.error(f"refusing to overwrite {args.output}")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = HilDataLogger(args.output, args.rate, args.duration)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    missing = set(("odom", "route", "trajectory", "behavior", "gate", "mcu")) - node.received
    if missing:
        print(f"ERROR: missing live topics: {', '.join(sorted(missing))}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
