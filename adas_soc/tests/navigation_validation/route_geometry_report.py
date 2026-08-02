#!/usr/bin/env python3
"""Audit a GlobalRoute topology CSV without moving a vehicle."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def angle_difference(first: float, second: float) -> float:
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


def load_points(path: Path, selected_case: str = "") -> list[dict[str, object]]:
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    points: list[dict[str, object]] = []
    for row in rows:
        if selected_case and row.get("case") != selected_case:
            continue
        points.append(
            {
                "index": int(row["index"]),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "yaw": float(row["yaw"]),
                "lane_id": int(row["lane_id"]),
                "road_id": int(row["road_id"]),
                "speed_limit": float(row["speed_limit"]),
                "maneuver": str(row["maneuver"]),
            }
        )
    return points


def audit(points: list[dict[str, object]], max_gap_m: float) -> dict[str, object]:
    gaps: list[float] = []
    heading_jumps: list[float] = []
    reverse_progress: list[float] = []
    for before, after in zip(points, points[1:]):
        dx = float(after["x"]) - float(before["x"])
        dy = float(after["y"]) - float(before["y"])
        gaps.append(math.hypot(dx, dy))
        heading_jumps.append(
            angle_difference(float(after["yaw"]), float(before["yaw"]))
        )
        forward = dx * math.cos(float(before["yaw"])) + dy * math.sin(
            float(before["yaw"])
        )
        reverse_progress.append(max(0.0, -forward))
    largest = max(range(len(gaps)), key=gaps.__getitem__) + 1 if gaps else 0
    errors: list[str] = []
    if len(points) < 2:
        errors.append("route has fewer than two points")
    if any(not math.isfinite(float(p[key])) for p in points for key in ("x", "y", "yaw", "speed_limit")):
        errors.append("route contains non-finite values")
    if gaps and max(gaps) > max_gap_m:
        errors.append(f"maximum adjacent gap exceeds {max_gap_m:.3f} m")
    if reverse_progress and max(reverse_progress) > 0.5:
        errors.append("maximum reverse progress exceeds 0.500 m")
    if points and str(points[-1]["maneuver"]) != "STOP":
        errors.append("endpoint maneuver is not STOP")
    return {
        "point_count": len(points),
        "route_length_m": sum(gaps),
        "maximum_adjacent_gap_m": max(gaps, default=0.0),
        "maximum_heading_jump_rad": max(heading_jumps, default=0.0),
        "maximum_reverse_progress_m": max(reverse_progress, default=0.0),
        "lane_transition_count": sum(
            before["lane_id"] != after["lane_id"]
            for before, after in zip(points, points[1:])
        ),
        "largest_gap_point_index": largest,
        "largest_gap_before_point": points[largest - 1] if largest else None,
        "largest_gap_after_point": points[largest] if largest else None,
        "validation_result": "PASS" if not errors else "FAIL",
        "validation_errors": errors,
    }


def plot_route(points: list[dict[str, object]], report: dict[str, object], prefix: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(8.0, 6.0))
    lane_ids = list(dict.fromkeys(int(point["lane_id"]) for point in points))
    styles = ("-", "--", "-.", ":")
    for lane_index, lane_id in enumerate(lane_ids):
        lane_points = [point for point in points if int(point["lane_id"]) == lane_id]
        axis.plot(
            [float(point["x"]) for point in lane_points],
            [float(point["y"]) for point in lane_points],
            color=str(0.15 + 0.65 * lane_index / max(1, len(lane_ids) - 1)),
            linestyle=styles[lane_index % len(styles)],
            linewidth=1.2,
            label=f"lane {lane_id}",
        )
    if points:
        axis.scatter(float(points[0]["x"]), float(points[0]["y"]), marker="o", color="black", label="start")
        axis.scatter(float(points[-1]["x"]), float(points[-1]["y"]), marker="s", facecolors="none", edgecolors="black", label="goal")
    largest = int(report["largest_gap_point_index"])
    if largest:
        before, after = points[largest - 1], points[largest]
        axis.plot(
            [float(before["x"]), float(after["x"])],
            [float(before["y"]), float(after["y"])],
            color="black",
            linewidth=2.5,
            label="maximum gap",
        )
    for before, after in zip(points, points[1:]):
        if before["lane_id"] != after["lane_id"]:
            axis.scatter(float(after["x"]), float(after["y"]), marker="x", color="black")
            axis.annotate(str(after["maneuver"]), (float(after["x"]), float(after["y"])), fontsize=7)
    axis.set_xlabel("Map X (m)")
    axis.set_ylabel("Map Y (m)")
    axis.set_title("Global Route Geometry")
    axis.axis("equal")
    axis.grid(True, color="0.85", linewidth=0.6)
    axis.legend(fontsize=7, loc="best")
    figure.tight_layout()
    prefix.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(prefix.with_suffix(".png"), dpi=180)
    figure.savefig(prefix.with_suffix(".svg"))
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--route-id", type=int, default=0)
    parser.add_argument("--case", default="", help="select one case from a multi-route CSV")
    parser.add_argument("--max-gap-m", type=float, default=3.0)
    parser.add_argument("--plot-prefix", type=Path)
    args = parser.parse_args()
    points = load_points(args.input, args.case)
    report = audit(points, args.max_gap_m)
    report.update(
        scenario=args.scenario,
        route_id=args.route_id,
        source_csv=str(args.input.resolve()),
        maximum_adjacent_gap_limit_m=args.max_gap_m,
        selected_case=args.case,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.plot_prefix:
        plot_route(points, report, args.plot_prefix)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["validation_result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
