#!/usr/bin/env python3
"""Read-only-input CARLA ego idle audit for the Phase 3.4 N0 gate.

The script never enables autopilot, Traffic Manager, target velocity, or
constant velocity, and never applies a vehicle control.  It records CARLA's
native default actor state for a fixed number of synchronous ticks, then
destroys only the actor it created.
"""

import argparse
import csv
import json
import math
import time
from pathlib import Path

import carla


def speed_mps(vector):
    return math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--town", default="Town04")
    parser.add_argument("--spawn-index", type=int, default=30)
    parser.add_argument("--ticks", type=int, default=100)
    parser.add_argument("--fixed-delta", type=float, default=0.02)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    summary_path = output.with_suffix(".summary.json")

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    world = client.get_world()
    if args.town not in world.get_map().name:
        world = client.load_world(args.town)

    original = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = args.fixed_delta
    world.apply_settings(settings)

    actor = None
    actor_id = None
    rows = []
    start = time.monotonic()
    try:
        points = world.get_map().get_spawn_points()
        if not points:
            raise RuntimeError("map has no spawn points")
        bp = world.get_blueprint_library().find("vehicle.tesla.model3")
        bp.set_attribute("role_name", "hero_phase34_n0")
        actor = world.try_spawn_actor(bp, points[args.spawn_index % len(points)])
        if actor is None:
            raise RuntimeError("selected spawn point is occupied")
        actor_id = actor.id

        # The actor is not materialized until the first synchronous tick; state
        # queried immediately after try_spawn_actor may still be all zero.
        world.tick()
        initial = actor.get_location()
        for tick_index in range(args.ticks + 1):
            if tick_index:
                world.tick()
            loc = actor.get_location()
            velocity = actor.get_velocity()
            angular = actor.get_angular_velocity()
            control = actor.get_control()
            rows.append({
                "monotonic_s": time.monotonic(),
                "tick": tick_index,
                "x": loc.x,
                "y": loc.y,
                "z": loc.z,
                "displacement_3d_m": math.sqrt(
                    (loc.x - initial.x) ** 2 + (loc.y - initial.y) ** 2
                    + (loc.z - initial.z) ** 2),
                "speed_mps": speed_mps(velocity),
                "vx": velocity.x,
                "vy": velocity.y,
                "vz": velocity.z,
                "angular_speed_deg_s": speed_mps(angular),
                "throttle": control.throttle,
                "brake": control.brake,
                "steer": control.steer,
                "hand_brake": int(control.hand_brake),
                "manual_gear_shift": int(control.manual_gear_shift),
                "gear": control.gear,
            })
    finally:
        if actor is not None:
            actor.destroy()
        world.apply_settings(original)

    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    summary = {
        "schema": 1,
        "input_control_published": False,
        "autopilot_enabled": False,
        "traffic_manager_used": False,
        "target_velocity_used": False,
        "constant_velocity_used": False,
        "town": world.get_map().name,
        "actor_id": actor_id,
        "ticks": args.ticks,
        "duration_monotonic_s": time.monotonic() - start,
        "maximum_speed_mps": max(row["speed_mps"] for row in rows),
        "maximum_displacement_3d_m": max(row["displacement_3d_m"] for row in rows),
        "maximum_displacement_xy_m": max(math.hypot(
            row["x"] - rows[0]["x"], row["y"] - rows[0]["y"])
            for row in rows),
        "final_speed_mps": rows[-1]["speed_mps"],
        "final_displacement_3d_m": rows[-1]["displacement_3d_m"],
        "initial_control": {key: rows[0][key] for key in (
            "throttle", "brake", "steer", "hand_brake", "gear")},
    }
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
