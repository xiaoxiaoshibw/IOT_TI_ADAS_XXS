#!/usr/bin/env python3
"""ROS-independent helpers shared by the Phase 3 validation executables."""

from __future__ import annotations

import json
import math
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


VALID_ROUTE_STATUS = 2
VALID_MANEUVERS = frozenset(range(7))
ROUTE_MANEUVER_NAMES = {
    0: "UNKNOWN",
    1: "STRAIGHT",
    2: "LEFT",
    3: "RIGHT",
    4: "LANE_CHANGE_LEFT",
    5: "LANE_CHANGE_RIGHT",
    6: "STOP",
}


@dataclass(frozen=True)
class RouteCheck:
    errors: tuple[str, ...]
    warnings: tuple[str, ...]
    route_length_m: float = 0.0
    maximum_adjacent_gap_m: float = 0.0
    maximum_heading_jump_rad: float = 0.0
    maximum_reverse_progress_m: float = 0.0
    largest_gap_point_index: int = 0

    @property
    def ok(self) -> bool:
        return not self.errors


def quaternion_yaw(q: Any) -> float:
    """Return REP-103 yaw from an object with x/y/z/w members."""
    siny = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
    cosy = 1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2)
    return math.atan2(siny, cosy)


def encoded_road_id(lane_id: int) -> int:
    """Decode road_id exactly like semantic_route.cpp (upper bits above bit 23)."""
    return max(0, int(lane_id) >> 24) if int(lane_id) > 0 else 0


def polyline_length(points: Sequence[Any]) -> float:
    return sum(
        math.hypot(float(b.x) - float(a.x), float(b.y) - float(a.y))
        for a, b in zip(points, points[1:])
    )


def validate_route(
    route: Any,
    *,
    max_adjacent_gap_m: float = 3.0,
    min_point_spacing_m: float = 0.05,
    max_heading_jump_rad: float = 1.2,
    max_reverse_progress_m: float = 0.5,
) -> RouteCheck:
    errors: list[str] = []
    warnings: list[str] = []
    points = list(route.points)
    frame = str(route.frame_id or route.header.frame_id)
    if int(route.status) == VALID_ROUTE_STATUS:
        if not points:
            errors.append("VALID route has no points")
        if int(route.route_id) == 0:
            errors.append("VALID route_id is zero")
    elif points:
        errors.append("non-VALID route retained points")
    if frame != "map":
        errors.append(f"route frame must be map, got {frame!r}")
    if route.frame_id and route.header.frame_id and route.frame_id != route.header.frame_id:
        errors.append("frame_id and header.frame_id disagree")
    maximum_gap = 0.0
    maximum_heading_jump = 0.0
    maximum_reverse_progress = 0.0
    largest_gap_index = 0
    duplicates = 0
    for index, point in enumerate(points):
        values = (point.x, point.y, point.yaw, point.speed_limit)
        if not all(math.isfinite(float(value)) for value in values):
            errors.append(f"point[{index}] has a non-finite value")
        if float(point.speed_limit) <= 0.0:
            errors.append(f"point[{index}] speed_limit is not positive")
        if int(point.maneuver) not in VALID_MANEUVERS:
            errors.append(f"point[{index}] has invalid maneuver {point.maneuver}")
        decoded = encoded_road_id(int(point.lane_id))
        if decoded != int(point.road_id):
            errors.append(
                f"point[{index}] lane road {decoded} != road_id {point.road_id}"
            )
        if index:
            previous = points[index - 1]
            dx = float(point.x) - float(previous.x)
            dy = float(point.y) - float(previous.y)
            gap = math.hypot(dx, dy)
            if gap > maximum_gap:
                maximum_gap = gap
                largest_gap_index = index
            heading_jump = abs(math.atan2(
                math.sin(float(point.yaw) - float(previous.yaw)),
                math.cos(float(point.yaw) - float(previous.yaw)),
            ))
            maximum_heading_jump = max(maximum_heading_jump, heading_jump)
            forward = dx * math.cos(float(previous.yaw)) + dy * math.sin(float(previous.yaw))
            reverse = max(0.0, -forward)
            maximum_reverse_progress = max(maximum_reverse_progress, reverse)
            duplicates += gap < min_point_spacing_m
            if gap > max_adjacent_gap_m:
                errors.append(
                    f"point[{index}] adjacent gap {gap:.3f} m exceeds "
                    f"{max_adjacent_gap_m:.3f} m"
                )
            if heading_jump > max_heading_jump_rad:
                errors.append(
                    f"point[{index}] heading jump {heading_jump:.3f} rad exceeds "
                    f"{max_heading_jump_rad:.3f} rad"
                )
            if reverse > max_reverse_progress_m:
                errors.append(
                    f"point[{index}] reverse progress {reverse:.3f} m exceeds "
                    f"{max_reverse_progress_m:.3f} m"
                )
    if points and int(points[-1].maneuver) != 6:
        errors.append("last point is not marked STOP")
    if len(points) > 1 and duplicates / (len(points) - 1) > 0.1:
        errors.append("duplicate point ratio exceeds 0.1")
    measured = polyline_length(points)
    tolerance = max(0.5, measured * 0.01)
    if points and abs(float(route.length) - measured) > tolerance:
        errors.append(
            f"declared length {float(route.length):.3f} differs from geometry "
            f"{measured:.3f}"
        )
    return RouteCheck(
        tuple(errors), tuple(warnings), measured, maximum_gap,
        maximum_heading_jump, maximum_reverse_progress, largest_gap_index,
    )


def nearest_route_point(
    points: Sequence[Any], x: float, y: float, start: int = 0, window: int = 250
) -> tuple[int, Any, float]:
    if not points:
        raise ValueError("route is empty")
    lower = min(max(0, int(start)), len(points) - 1)
    upper = min(len(points), lower + max(1, int(window)))
    index = min(
        range(lower, upper),
        key=lambda i: (float(points[i].x) - x) ** 2 + (float(points[i].y) - y) ** 2,
    )
    point = points[index]
    dx = x - float(point.x)
    dy = y - float(point.y)
    lateral = -math.sin(float(point.yaw)) * dx + math.cos(float(point.yaw)) * dy
    return index, point, lateral


def _percentile(values: Sequence[float], pct: float) -> float | None:
    """Return the pct-quantile (0..100) of values via linear interpolation.

    Empty input returns None. Values need not be sorted.
    """
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (pct / 100.0) * (len(ordered) - 1)
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    frac = rank - lower
    return ordered[lower] * (1.0 - frac) + ordered[upper] * frac


def _safe_float(value: Any) -> float | None:
    """Parse a CSV cell into float, returning None when blank or invalid."""
    if value is None:
        return None
    if isinstance(value, str) and value == "":
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result


def _column(rows: Sequence[Mapping[str, Any]], name: str) -> list[float | None]:
    """Project a named column to per-row floats (None where blank/invalid)."""
    return [_safe_float(r.get(name)) for r in rows]


def _safe_diff_series(values: Sequence[float | None]) -> list[float]:
    """Compute per-row d(value)/dt using elapsed_s from `rows` (paired arg).

    Returns only entries with two valid samples. Caller passes (values, times).
    """
    raise NotImplementedError("use _time_derivative instead")


def _time_derivative(
    values: Sequence[float | None], times: Sequence[float | None]
) -> list[float]:
    """Compute d(value)/dt between consecutive valid pairs of (value, time)."""
    out: list[float] = []
    prev_v: float | None = None
    prev_t: float | None = None
    for v, t in zip(values, times):
        if v is None or t is None:
            prev_v, prev_t = None, None
            continue
        if prev_v is None or prev_t is None:
            prev_v, prev_t = v, t
            continue
        dt = t - prev_t
        if dt <= 0.0:
            prev_v, prev_t = v, t
            continue
        out.append((v - prev_v) / dt)
        prev_v, prev_t = v, t
    return out


def _count_transitions(values: Sequence[float | None], *, trigger: float | None = None) -> int:
    """Count rising-edge transitions of `values`. If `trigger` is given, also
    require the new value to equal `trigger` (e.g. lane_valid going False)."""
    prev: float | None = None
    count = 0
    for v in values:
        if v is None:
            continue
        if prev is not None and v != prev:
            if trigger is None or v == trigger:
                count += 1
        prev = v
    return count


def navigation_metrics(rows: Iterable[Mapping[str, Any]]) -> dict[str, Any]:
    samples = list(rows)
    lateral = [float(r["lateral_error"]) for r in samples if r.get("lateral_error") not in (None, "")]
    speed = [float(r["speed_error"]) for r in samples if r.get("speed_error") not in (None, "")]
    poses = [
        (float(r["vehicle_x"]), float(r["vehicle_y"]))
        for r in samples
        if r.get("vehicle_x") not in (None, "") and r.get("vehicle_y") not in (None, "")
    ]
    distance = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(poses, poses[1:]))

    # Commit 1b — extend the per-scenario aggregate. All new columns are
    # backward-compatible: blank cells parse as None and the corresponding
    # metric stays None. None values are kept distinct from zero so callers
    # can tell "not measured" from "measured zero".
    times = _column(samples, "elapsed_s")
    steering = _column(samples, "steering")
    velocity = _column(samples, "velocity")
    gate_accel = _column(samples, "gate_longitudinal_accel_mps2")
    lane_offset = _column(samples, "lane_lateral_offset_m")
    lane_heading = _column(samples, "lane_heading_error_rad")
    lane_valid = _column(samples, "lane_valid")
    lead_id = _column(samples, "lead_id")
    lead_gap = _column(samples, "lead_gap_m")
    lead_speed = _column(samples, "lead_speed_mps")
    throttle = _column(samples, "throttle")
    brake = _column(samples, "brake")
    safety_state = _column(samples, "safety_state")

    abs_offset = [abs(v) for v in lane_offset if v is not None]
    abs_heading = [abs(v) for v in lane_heading if v is not None]
    lead_gap_clean = [v for v in lead_gap if v is not None]
    lead_speed_clean = [v for v in lead_speed if v is not None]
    velocity_clean = [v for v in velocity if v is not None]

    steering_rate = _time_derivative(steering, times)
    accel_from_odom = _time_derivative(velocity, times)
    jerk_from_odom = _time_derivative(accel_from_odom, times)

    # throttle/brake simultaneous — a sign of PID oscillation. Use a small
    # band so quiet idle cells don't count.
    sim_count = 0
    for t, b in zip(throttle, brake):
        if t is None or b is None:
            continue
        if t > 0.05 and b > 0.05:
            sim_count += 1

    # Time spent in MRM (LEVEL_MRM_COMFORT=2 or LEVEL_MRM_EMERGENCY=3).
    mrm_states = {2, 3}
    mrm_samples = sum(1 for v in safety_state if v is not None and int(v) in mrm_states)
    safety_total = sum(1 for v in safety_state if v is not None)
    aeb_rising = _count_transitions(safety_state, trigger=3.0)

    return {
        "sample_count": len(samples),
        "lateral_error_rms_m": math.sqrt(sum(v * v for v in lateral) / len(lateral)) if lateral else None,
        "lateral_error_max_abs_m": max(map(abs, lateral)) if lateral else None,
        "speed_error_rms_mps": math.sqrt(sum(v * v for v in speed) / len(speed)) if speed else None,
        "travel_distance_m": distance if poses else None,
        # Commit 1b additions below.
        "lane_lateral_offset_abs_p95_m": _percentile(abs_offset, 95.0),
        "lane_lateral_offset_abs_max_m": max(abs_offset) if abs_offset else None,
        "lane_heading_error_abs_max_rad": max(abs_heading) if abs_heading else None,
        "steering_rate_p95_rad_s": _percentile([abs(v) for v in steering_rate], 95.0),
        "steering_rate_max_abs_rad_s": max(map(abs, steering_rate)) if steering_rate else None,
        "ego_speed_mean_mps": (sum(velocity_clean) / len(velocity_clean)) if velocity_clean else None,
        "accel_p95_abs_mps2": _percentile([abs(v) for v in accel_from_odom], 95.0),
        "jerk_p95_abs_mps3": _percentile([abs(v) for v in jerk_from_odom], 95.0),
        "lead_id_switches_count": _count_transitions(lead_id),
        "lead_gap_mean_m": (sum(lead_gap_clean) / len(lead_gap_clean)) if lead_gap_clean else None,
        "lead_speed_mean_mps": (sum(lead_speed_clean) / len(lead_speed_clean)) if lead_speed_clean else None,
        "throttle_brake_simultaneous_count": sim_count,
        "lane_invalid_transitions": _count_transitions(lane_valid, trigger=0.0),
        "aeb_trigger_count": aeb_rising,
        "safety_state_time_in_mrm_fraction": (mrm_samples / safety_total) if safety_total else None,
    }


def atomic_json(path: os.PathLike[str] | str, value: Any) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary = tempfile.mkstemp(prefix=f".{destination.name}.", dir=destination.parent)
    try:
        with os.fdopen(handle, "w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, destination)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise
