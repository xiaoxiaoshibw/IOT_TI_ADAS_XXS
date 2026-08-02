#!/usr/bin/env python3
"""Fail-closed CARLA readiness gate used before any world reload or bridge start."""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import time
from pathlib import Path


def atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    os.replace(temporary, path)


def tcp_reachable(host: str, port: int, timeout_s: float = 0.5) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True
    except OSError:
        return False


def normalize_town(name: str) -> str:
    return name.rstrip("/").split("/")[-1]


def validate_sample(client, expected_town: str) -> dict:
    client_version = str(client.get_client_version())
    server_version = str(client.get_server_version())
    if client_version != server_version:
        raise RuntimeError(
            f"CARLA client/server version mismatch: {client_version} != {server_version}"
        )
    world = client.get_world()
    carla_map = world.get_map()
    settings = world.get_settings()
    is_sync = bool(settings.synchronous_mode)
    # Readiness must not wait for the bridge to become the synchronous tick owner.
    # Async frame progression is checked across samples in wait_until_ready().
    snapshot = world.get_snapshot()
    if snapshot is None or not math.isfinite(float(snapshot.timestamp.elapsed_seconds)):
        raise RuntimeError("CARLA world snapshot is unavailable or has an invalid timestamp")
    return {
        "client_version": client_version,
        "server_version": server_version,
        "map": str(carla_map.name),
        "expected_town": expected_town,
        "town_matches": normalize_town(str(carla_map.name)) == normalize_town(expected_town),
        "frame": int(snapshot.frame),
        "elapsed_seconds": float(snapshot.timestamp.elapsed_seconds),
        "synchronous_mode": bool(settings.synchronous_mode),
        "fixed_delta_seconds": settings.fixed_delta_seconds,
    }


def wait_until_ready(
    carla_module,
    host: str,
    port: int,
    expected_town: str,
    timeout_s: float,
    stabilization_s: float,
    interval_s: float,
    stable_samples: int,
) -> dict:
    started = time.monotonic()
    deadline = started + timeout_s
    first_rpc_success = None
    consecutive = 0
    samples = []
    last_error = "TCP endpoint not reachable"
    previous_frame = None
    previous_elapsed = None

    while time.monotonic() < deadline:
        try:
            if not tcp_reachable(host, port):
                raise RuntimeError("CARLA RPC connection failed: TCP endpoint not reachable")
            # Recreate after every cold-start failure: a timed-out CARLA rpc client
            # is not guaranteed to reconnect when the server later becomes ready.
            client = carla_module.Client(host, port)
            client.set_timeout(min(5.0, max(0.1, deadline - time.monotonic())))
            sample = validate_sample(client, expected_town)
            now = time.monotonic()
            if not sample["synchronous_mode"] and previous_frame is not None:
                if sample["frame"] <= previous_frame or sample["elapsed_seconds"] <= previous_elapsed:
                    raise RuntimeError(
                        f"CARLA asynchronous snapshot did not advance (frame {previous_frame}->{sample['frame']}, elapsed {previous_elapsed}->{sample['elapsed_seconds']})"
                    )
            if first_rpc_success is None:
                first_rpc_success = now
            consecutive += 1
            sample["probe_offset_s"] = now - started
            samples.append(sample)
            stable_for = now - first_rpc_success
            if consecutive >= stable_samples and stable_for >= stabilization_s:
                if not sample["town_matches"]:
                    world = client.load_world(expected_town)
                    if normalize_town(world.get_map().name) != normalize_town(expected_town):
                        raise RuntimeError(f"load_world did not select {expected_town}")
                    # A reload invalidates prior World objects. Re-probe only through Client.
                    sample = validate_sample(client, expected_town)
                    sample["probe_offset_s"] = time.monotonic() - started
                    samples.append(sample)
                return {
                    "valid": True,
                    "reason": "CARLA RPC and world remained stable before bridge start",
                    "host": host,
                    "port": port,
                    "stabilization_s": stabilization_s,
                    "stable_samples_required": stable_samples,
                    "sample_count": len(samples),
                    "carla_module": str(getattr(carla_module, "__file__", "unknown")),
                    "final": samples[-1],
                    "samples": samples,
                }
            last_error = "stabilization interval not yet satisfied"
            previous_frame = sample["frame"]
            previous_elapsed = sample["elapsed_seconds"]
        except Exception as exc:  # CARLA raises RuntimeError for RPC failures.
            last_error = str(exc)
            consecutive = 0
            first_rpc_success = None
            previous_frame = None
            previous_elapsed = None
        time.sleep(interval_s)

    return {
        "valid": False,
        "reason": last_error,
        "host": host,
        "port": port,
        "timeout_s": timeout_s,
        "stabilization_s": stabilization_s,
        "sample_count": len(samples),
        "samples": samples,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--expected-town", default="Town04")
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--stabilization", type=float, default=10.0)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--stable-samples", type=int, default=3)
    parser.add_argument("--output", required=True, type=Path)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if (
        args.port <= 0
        or args.timeout <= 0.0
        or args.stabilization < 0.0
        or args.interval <= 0.0
        or args.stable_samples < 1
    ):
        raise SystemExit("invalid readiness configuration")
    import carla

    result = wait_until_ready(
        carla,
        args.host,
        args.port,
        args.expected_town,
        args.timeout,
        args.stabilization,
        args.interval,
        args.stable_samples,
    )
    result["timestamp"] = time.time()
    atomic_json(args.output, result)
    print(json.dumps(result, sort_keys=True))
    return 0 if result["valid"] else 1


if __name__ == "__main__":
    sys.exit(main())
