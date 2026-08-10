#!/usr/bin/env python3
"""Validate the repository's fixed-scenario catalog using stdlib only."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import re
import sys
from pathlib import Path
from typing import Any


SCHEMA_VERSION = 1
BACKENDS = {"carla", "sil"}
CLASSIFICATIONS = {"vehicle", "pedestrian", "cyclist", "unknown"}
LANES = {"ego", "left", "right", "crossing"}
LEGACY_ROLES = {"lead", "pedestrian"}
MIN_INITIAL_SEPARATION_M = 2.0
MAX_SPEED_MPS = 80.0
MAX_ACCEL_MPS2 = 20.0


class ScenarioValidationError(ValueError):
    """A scenario contract violation with a stable human-readable message."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ScenarioValidationError(message)


def _number(value: Any, field: str) -> float:
    _require(isinstance(value, (int, float)) and not isinstance(value, bool),
             f"{field}: expected a number")
    result = float(value)
    _require(math.isfinite(result), f"{field}: expected a finite number")
    return result


def _integer(value: Any, field: str, minimum: int = 0) -> int:
    _require(isinstance(value, int) and not isinstance(value, bool),
             f"{field}: expected an integer")
    _require(value >= minimum, f"{field}: must be >= {minimum}")
    return value


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ScenarioValidationError(f"{path}: cannot load JSON: {exc}") from exc
    _require(isinstance(value, dict), f"{path}: root must be an object")
    return value


def validate_actor(actor: Any, where: str) -> None:
    _require(isinstance(actor, dict), f"{where}: actor must be an object")
    _integer(actor.get("id"), f"{where}.id", 1)
    _require(actor.get("classification") in CLASSIFICATIONS,
             f"{where}.classification: unsupported value")
    _require(actor.get("lane") in LANES, f"{where}.lane: unsupported value")
    if "legacy_role" in actor:
        _require(actor["legacy_role"] in LEGACY_ROLES,
                 f"{where}.legacy_role: unsupported value")

    _number(actor.get("initial_station_m"), f"{where}.initial_station_m")
    _number(actor.get("initial_lateral_m"), f"{where}.initial_lateral_m")
    initial_speed = _number(actor.get("initial_speed_mps"),
                            f"{where}.initial_speed_mps")
    _require(0.0 <= initial_speed <= MAX_SPEED_MPS,
             f"{where}.initial_speed_mps: outside [0, {MAX_SPEED_MPS}]")
    accel = _number(actor.get("accel_limit_mps2"),
                    f"{where}.accel_limit_mps2")
    _require(0.0 < accel <= MAX_ACCEL_MPS2,
             f"{where}.accel_limit_mps2: outside (0, {MAX_ACCEL_MPS2}]")

    profile = actor.get("speed_profile")
    _require(isinstance(profile, list) and profile,
             f"{where}.speed_profile: expected a non-empty array")
    previous_time = -1.0
    for index, point in enumerate(profile):
        point_where = f"{where}.speed_profile[{index}]"
        _require(isinstance(point, list) and len(point) == 2,
                 f"{point_where}: expected [time_s, target_speed_mps]")
        time_s = _number(point[0], f"{point_where}[0]")
        speed_mps = _number(point[1], f"{point_where}[1]")
        _require(time_s >= 0.0 and time_s > previous_time,
                 f"{point_where}: times must be non-negative and strictly increasing")
        _require(0.0 <= speed_mps <= MAX_SPEED_MPS,
                 f"{point_where}: speed outside [0, {MAX_SPEED_MPS}]")
        previous_time = time_s
    _require(float(profile[0][0]) == 0.0,
             f"{where}.speed_profile: first time must be 0")
    _require(float(profile[0][1]) == initial_speed,
             f"{where}.speed_profile: first speed must equal initial_speed_mps")

    if "hard_brake_window_s" in actor:
        window = actor["hard_brake_window_s"]
        _require(isinstance(window, list) and len(window) == 2,
                 f"{where}.hard_brake_window_s: expected [start_s, end_s]")
        start = _number(window[0], f"{where}.hard_brake_window_s[0]")
        end = _number(window[1], f"{where}.hard_brake_window_s[1]")
        _require(0.0 <= start < end,
                 f"{where}.hard_brake_window_s: expected 0 <= start < end")

    if actor.get("legacy_role") == "pedestrian":
        _require(actor["classification"] == "pedestrian",
                 f"{where}: pedestrian legacy role requires pedestrian classification")
        gap = _number(actor.get("trigger_ego_gap_m"),
                      f"{where}.trigger_ego_gap_m")
        crossing_speed = _number(actor.get("crossing_speed_mps"),
                                 f"{where}.crossing_speed_mps")
        _require(gap > 0.0, f"{where}.trigger_ego_gap_m: must be positive")
        _require(0.0 < crossing_speed <= 15.0,
                 f"{where}.crossing_speed_mps: outside (0, 15]")


def validate_scenario(data: dict[str, Any], source: str = "scenario") -> None:
    _require(data.get("schema_version") == SCHEMA_VERSION,
             f"{source}.schema_version: expected {SCHEMA_VERSION}")
    _require(isinstance(data.get("id"), str) and data["id"],
             f"{source}.id: expected a non-empty string")
    _require(re.fullmatch(r"[a-z0-9_]+", data["id"]) is not None,
             f"{source}.id: expected lowercase letters, digits, and underscores")
    _require(isinstance(data.get("name"), str) and data["name"],
             f"{source}.name: expected a non-empty string")
    _require(data.get("town") == "Town04", f"{source}.town: v1 requires Town04")
    backends = data.get("supported_backends")
    _require(isinstance(backends, list) and backends,
             f"{source}.supported_backends: expected a non-empty array")
    _require(len(backends) == len(set(backends)) and set(backends) <= BACKENDS,
             f"{source}.supported_backends: duplicate or unsupported backend")
    _integer(data.get("seed"), f"{source}.seed")
    duration = _number(data.get("duration_s"), f"{source}.duration_s")
    _require(duration >= 0.0, f"{source}.duration_s: must be non-negative")
    ego = data.get("ego")
    _require(isinstance(ego, dict), f"{source}.ego: expected an object")
    _integer(ego.get("spawn_index"), f"{source}.ego.spawn_index")
    _require(isinstance(data.get("notes"), list) and
             all(isinstance(note, str) for note in data["notes"]),
             f"{source}.notes: expected an array of strings")

    actors = data.get("actors")
    _require(isinstance(actors, list), f"{source}.actors: expected an array")
    ids: set[int] = set()
    roles: set[str] = set()
    positions: list[tuple[int, float, float]] = []
    for index, actor in enumerate(actors):
        where = f"{source}.actors[{index}]"
        validate_actor(actor, where)
        actor_id = actor["id"]
        _require(actor_id not in ids, f"{where}.id: duplicate actor ID {actor_id}")
        ids.add(actor_id)
        role = actor.get("legacy_role")
        if role:
            _require(role not in roles, f"{where}.legacy_role: duplicate role {role}")
            roles.add(role)
        station = float(actor["initial_station_m"])
        lateral = float(actor["initial_lateral_m"])
        for other_id, other_station, other_lateral in positions:
            separation = math.hypot(station - other_station, lateral - other_lateral)
            _require(separation >= MIN_INITIAL_SEPARATION_M,
                     f"{where}: initial position overlaps actor {other_id}")
        positions.append((actor_id, station, lateral))


def legacy_projection(data: dict[str, Any]) -> dict[str, Any]:
    lead = None
    pedestrian = None
    for actor in data["actors"]:
        if actor.get("legacy_role") == "lead":
            lead = {
                "gap0": actor["initial_station_m"],
                "profile": actor["speed_profile"],
                "hard_brake": actor.get("hard_brake_window_s"),
            }
        elif actor.get("legacy_role") == "pedestrian":
            pedestrian = {
                "ahead_m": actor["initial_station_m"],
                "start_lateral_m": actor["initial_lateral_m"],
                # end_lateral_m 可省略：runner 会按 -start_lateral_m 对称推断。
                # 仅当 JSON 显式给出时校验器才要求 legacy 同步出现。
                "end_lateral_m": actor.get("end_lateral_m"),
                "trigger_ego_gap_m": actor["trigger_ego_gap_m"],
                "speed_mps": actor["crossing_speed_mps"],
            }
    return {
        "name": data["name"],
        "duration": data["duration_s"],
        "spawn_index": data["ego"]["spawn_index"],
        "lead": lead,
        "pedestrian": pedestrian,
        "notes": data["notes"],
    }


def _json_shape(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: _json_shape(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_shape(item) for item in value]
    return value


def load_legacy_scenarios(repo_root: Path) -> tuple[dict[str, Any], list[str]]:
    path = (repo_root / "adas_bridge_pc/carla_ros2_bridge/ws/src/"
            "adas_carla_bridge/adas_carla_bridge/scenarios.py")
    spec = importlib.util.spec_from_file_location("legacy_adas_scenarios", path)
    _require(spec is not None and spec.loader is not None,
             f"cannot import legacy scenarios from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.SCENARIOS, module.ORDER


def validate_catalog(catalog_path: Path, repo_root: Path) -> dict[str, Any]:
    catalog = load_json(catalog_path)
    _require(catalog.get("schema_version") == SCHEMA_VERSION,
             f"catalog.schema_version: expected {SCHEMA_VERSION}")
    entries = catalog.get("scenarios")
    _require(isinstance(entries, list) and entries,
             "catalog.scenarios: expected a non-empty array")

    seen_ids: set[str] = set()
    seen_files: set[str] = set()
    loaded: dict[str, dict[str, Any]] = {}
    scenario_root = (repo_root / "scenarios").resolve()
    for index, entry in enumerate(entries):
        where = f"catalog.scenarios[{index}]"
        _require(isinstance(entry, dict), f"{where}: expected an object")
        scenario_id = entry.get("id")
        _require(isinstance(scenario_id, str) and scenario_id,
                 f"{where}.id: expected a non-empty string")
        _require(scenario_id not in seen_ids, f"{where}.id: duplicate {scenario_id}")
        seen_ids.add(scenario_id)
        relative_file = entry.get("file")
        _require(isinstance(relative_file, str) and relative_file,
                 f"{where}.file: expected a non-empty string")
        _require(relative_file not in seen_files,
                 f"{where}.file: duplicate {relative_file}")
        seen_files.add(relative_file)
        scenario_path = (repo_root / relative_file).resolve()
        _require(scenario_path.parent == scenario_root,
                 f"{where}.file: must be directly below scenarios/")
        data = load_json(scenario_path)
        validate_scenario(data, scenario_id)
        _require(data["id"] == scenario_id, f"{where}: ID does not match file")
        _require(entry.get("display_name") == data["name"],
                 f"{where}.display_name: does not match scenario name")
        _require(entry.get("default_town") == data["town"],
                 f"{where}.default_town: does not match scenario town")
        _require(entry.get("supported_backends") == data["supported_backends"],
                 f"{where}.supported_backends: does not match scenario")
        _require(entry.get("default_seed") == data["seed"],
                 f"{where}.default_seed: does not match scenario seed")
        _require(entry.get("expected_actor_count") == len(data["actors"]),
                 f"{where}.expected_actor_count: does not match actor count")
        _require(isinstance(entry.get("description"), str) and entry["description"],
                 f"{where}.description: expected a non-empty string")
        _require(isinstance(entry.get("acceptance_profile"), str) and
                 entry["acceptance_profile"],
                 f"{where}.acceptance_profile: expected a non-empty string")
        _require(entry.get("operator_tunable") == [],
                 f"{where}.operator_tunable: Phase 0 requires an empty array")
        loaded[scenario_id] = data

    legacy, order = load_legacy_scenarios(repo_root)
    _require(set(order) == set(legacy), "legacy ORDER and SCENARIOS IDs differ")
    missing = set(legacy) - set(loaded)
    _require(not missing, f"catalog: missing legacy scenario IDs {sorted(missing)}")
    for scenario_id, expected in legacy.items():
        actual = legacy_projection(loaded[scenario_id])
        _require(_json_shape(actual) == _json_shape(expected),
                 f"{scenario_id}: normalized JSON is not legacy-equivalent")

    return {
        "schema_version": SCHEMA_VERSION,
        "status": "PASS",
        "scenario_count": len(loaded),
        "legacy_scenario_count": len(legacy),
        "actor_count": sum(len(item["actors"]) for item in loaded.values()),
        "scenarios": [
            {"id": item["id"], "actor_count": len(item["actors"]),
             "backends": item["supported_backends"]}
            for item in loaded.values()
        ],
    }


def main(argv: list[str] | None = None) -> int:
    default_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=default_root)
    parser.add_argument("--catalog", type=Path)
    parser.add_argument("--json", action="store_true",
                        help="emit a machine-readable report")
    args = parser.parse_args(argv)
    repo_root = args.repo_root.resolve()
    catalog_path = (args.catalog or repo_root / "scenarios/catalog.json").resolve()
    try:
        report = validate_catalog(catalog_path, repo_root)
    except ScenarioValidationError as exc:
        report = {"schema_version": SCHEMA_VERSION, "status": "FAIL",
                  "error": str(exc)}
        if args.json:
            print(json.dumps(report, ensure_ascii=False, sort_keys=True))
        else:
            print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    else:
        print("PASS: validated {scenario_count} scenarios "
              "({legacy_scenario_count} legacy), {actor_count} actors".format(
                  **report))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
