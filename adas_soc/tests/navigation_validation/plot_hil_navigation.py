#!/usr/bin/env python3
"""Generate reproducible grayscale Phase 3 navigation figures from real CSVs."""

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def values(rows, key):
    return [(float(row["elapsed_s"]), float(row[key])) for row in rows if row.get(key)]


def save(figure, path):
    figure.tight_layout()
    figure.savefig(path.with_suffix(".png"), dpi=180)
    figure.savefig(path.with_suffix(".svg"))
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--hil-csv", required=True)
    parser.add_argument("--can-csv", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--scenario", required=True)
    args = parser.parse_args()
    rows = read(args.hil_csv)
    can = read(args.can_csv)
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(7.2, 5.2))
    vehicle = [(float(r["vehicle_x"]), float(r["vehicle_y"])) for r in rows
               if r.get("vehicle_x") and r.get("vehicle_y")]
    route = [(float(r["route_x"]), float(r["route_y"])) for r in rows
             if r.get("route_x") and r.get("route_y")]
    if route:
        ax.plot(*zip(*route), color="0.65", linewidth=2.0, label="route")
    if vehicle:
        ax.plot(*zip(*vehicle), color="0.05", linewidth=1.1, label="vehicle")
    ax.set(xlabel="Map X (m)", ylabel="Map Y (m)", title="Global Route Tracking")
    ax.axis("equal"); ax.grid(color="0.85"); ax.legend()
    save(fig, out / "global_route_tracking")

    for key, title, ylabel, filename in (
        ("lateral_error", "Lateral Error", "Error (m)", "lateral_error"),
        ("velocity", "Speed Tracking", "Speed (m/s)", "speed_tracking"),
    ):
        fig, ax = plt.subplots(figsize=(7.2, 4.0))
        data = values(rows, key)
        if data:
            ax.plot(*zip(*data), color="0.05", label=("actual" if key == "velocity" else "error"))
        if key == "velocity":
            target = values(rows, "target_speed")
            if target:
                ax.plot(*zip(*target), color="0.55", linestyle="--", label="target")
        ax.set(xlabel="Time (s)", ylabel=ylabel, title=title)
        ax.grid(color="0.85"); ax.legend()
        save(fig, out / filename)

    fig, ax = plt.subplots(figsize=(7.2, 4.0))
    for key, label, style in (("mcu_state", "MCU state", "-"),
                              ("mcu_active_source", "active source", "--"),
                              ("control_source", "Gate source", ":")):
        data = values(rows, key)
        if data:
            ax.step(*zip(*data), where="post", color=str(0.1 + 0.3 * len(ax.lines)),
                    linestyle=style, label=label)
    ax.set(xlabel="Time (s)", ylabel="Protocol value", title="MCU State / Active Source")
    ax.grid(color="0.85"); ax.legend()
    save(fig, out / "mcu_state_active_source")

    fig, ax = plt.subplots(figsize=(7.2, 4.0))
    if can:
        t0 = float(can[0]["time"])
        ts = [float(r["time"]) - t0 for r in can]
        for key, label, style in (("berr_tx", "berr tx", "-"),
                                  ("berr_rx", "berr rx", "--"),
                                  ("error", "interface errors", ":")):
            ax.step(ts, [float(r[key]) for r in can], where="post",
                    linestyle=style, color="0.1", label=label)
    ax.set(xlabel="Time (s)", ylabel="Count", title="CAN Reliability")
    ax.grid(color="0.85"); ax.legend()
    save(fig, out / "can_reliability")
    print(f"generated 10 files for {args.scenario} in {out}")


if __name__ == "__main__":
    main()
