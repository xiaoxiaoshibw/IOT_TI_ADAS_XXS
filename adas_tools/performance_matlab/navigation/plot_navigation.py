#!/usr/bin/env python3
"""Generate five grayscale, paper-style Phase 3 figures from real CSV data."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def rows(path):
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def numbers(data, key):
    return [(i, float(row[key])) for i, row in enumerate(data) if row.get(key) not in (None, "")]


def series(data, xkey, ykey):
    return [(float(row[xkey]), float(row[ykey])) for row in data if row.get(xkey) not in (None, "") and row.get(ykey) not in (None, "")]


def save(fig, output, stem):
    fig.tight_layout()
    fig.savefig(output / f"{stem}.png", dpi=300, bbox_inches="tight")
    fig.savefig(output / f"{stem}.svg", bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--can-metrics", type=Path)
    args = parser.parse_args()
    data = rows(args.csv)
    if not data:
        parser.error("input CSV is empty")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Noto Sans CJK SC", "SimHei", "DejaVu Sans"],
        "axes.unicode_minus": False,
        "axes.grid": True,
        "grid.color": "0.85",
        "lines.linewidth": 1.2,
    })

    vehicle = series(data, "vehicle_x", "vehicle_y")
    route = series(data, "route_x", "route_y")
    fig, ax = plt.subplots(figsize=(6.2, 4.2))
    if route:
        ax.plot(*zip(*route), color="0.15", linestyle="--", label="全局路线")
    if vehicle:
        ax.plot(*zip(*vehicle), color="0.45", label="车辆轨迹")
    ax.set(xlabel="X / m", ylabel="Y / m", title="Town04 全局路线跟踪")
    ax.axis("equal"); ax.legend()
    save(fig, args.output_dir, "figure1_global_route_tracking")

    fig, ax = plt.subplots(figsize=(6.2, 3.5))
    lateral = series(data, "elapsed_s", "lateral_error")
    if lateral: ax.plot(*zip(*lateral), color="0.2")
    ax.set(xlabel="时间 / s", ylabel="横向误差 $e_y$ / m", title="横向跟踪误差")
    save(fig, args.output_dir, "figure2_lateral_error")

    fig, ax = plt.subplots(figsize=(6.2, 3.5))
    actual = series(data, "elapsed_s", "velocity"); target = series(data, "elapsed_s", "target_speed")
    if target: ax.plot(*zip(*target), color="0.15", linestyle="--", label="目标速度")
    if actual: ax.plot(*zip(*actual), color="0.5", label="实际速度")
    ax.set(xlabel="时间 / s", ylabel="速度 / (m/s)", title="速度跟踪"); ax.legend()
    save(fig, args.output_dir, "figure3_speed_tracking")

    fig, ax = plt.subplots(figsize=(6.2, 3.5))
    mcu = series(data, "elapsed_s", "mcu_active_source")
    state = series(data, "elapsed_s", "mcu_state")
    if mcu: ax.step(*zip(*mcu), where="post", color="0.15", label="MCU 控制源")
    if state: ax.step(*zip(*state), where="post", color="0.6", linestyle="--", label="MCU 状态")
    ax.set(xlabel="时间 / s", ylabel="离散状态", title="F280025C 安全接管时间轴"); ax.legend()
    save(fig, args.output_dir, "figure4_mcu_takeover")

    fig, ax = plt.subplots(figsize=(6.2, 3.5))
    if args.can_metrics and args.can_metrics.exists():
        can = rows(args.can_metrics)
        times = [float(r["time"]) for r in can]
        if times:
            times = [v - times[0] for v in times]
            ax.step(times, [int(r["drop"]) for r in can], where="post", color="0.15", label="丢帧计数")
            ax.step(times, [int(r["error"]) for r in can], where="post", color="0.55", linestyle="--", label="CAN 错误")
    crc = series(data, "elapsed_s", "crc_error")
    if crc: ax.step(*zip(*crc), where="post", color="0.75", linestyle=":", label="MCU CRC 错误")
    ax.set(xlabel="时间 / s", ylabel="累计计数", title="CAN 可靠性"); ax.legend()
    save(fig, args.output_dir, "figure5_can_reliability")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

