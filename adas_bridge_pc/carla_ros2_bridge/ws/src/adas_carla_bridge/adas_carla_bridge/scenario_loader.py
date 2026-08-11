#!/usr/bin/env python3
"""Load frozen scenario-v1 JSON with legacy-ID fallback."""

import json
import math
import re
from pathlib import Path

from adas_carla_bridge.scenarios import ORDER, SCENARIOS, legacy_to_scripted


SCHEMA_VERSION = 1
SUPPORTED_BACKENDS = {'carla', 'sil'}
CLASSIFICATIONS = {'vehicle', 'pedestrian', 'cyclist', 'unknown'}
LANES = {'ego', 'left', 'right', 'crossing'}


class ScenarioLoadError(ValueError):
    """Raised when a scenario cannot be loaded without guessing."""


def _require(condition, message):
    if not condition:
        raise ScenarioLoadError(message)


def _finite_number(value, field):
    _require(isinstance(value, (int, float)) and not isinstance(value, bool),
             '%s must be a number' % field)
    value = float(value)
    _require(math.isfinite(value), '%s must be finite' % field)
    return value


def find_repo_root(start=None):
    """Find the checkout containing scenarios/catalog.json."""
    starts = [Path(start).resolve()] if start else []
    starts.extend([Path.cwd().resolve(), Path(__file__).resolve()])
    visited = set()
    for candidate in starts:
        base = candidate if candidate.is_dir() else candidate.parent
        for parent in (base, *base.parents):
            if parent in visited:
                continue
            visited.add(parent)
            if (parent / 'scenarios/catalog.json').is_file():
                return parent
    raise ScenarioLoadError('cannot locate repository scenarios/catalog.json')


def load_catalog(repo_root=None):
    """Return a catalog dictionary keyed by stable scenario ID."""
    root = Path(repo_root).resolve() if repo_root else find_repo_root()
    path = root / 'scenarios/catalog.json'
    try:
        catalog = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ScenarioLoadError('cannot load catalog %s: %s' %
                                (path, error)) from error
    _require(catalog.get('schema_version') == SCHEMA_VERSION,
             'unsupported catalog schema_version')
    result = {}
    for entry in catalog.get('scenarios', []):
        scenario_id = entry.get('id')
        _require(isinstance(scenario_id, str) and scenario_id,
                 'catalog entry has invalid id')
        _require(scenario_id not in result,
                 'duplicate catalog scenario id %s' % scenario_id)
        result[scenario_id] = entry
    return result


def validate_loaded_scenario(scenario):
    """Validate fields required by the CARLA scripted-actor runtime."""
    _require(isinstance(scenario, dict), 'scenario root must be an object')
    _require(scenario.get('schema_version') == SCHEMA_VERSION,
             'unsupported scenario schema_version')
    _require(isinstance(scenario.get('id'), str) and scenario['id'],
             'scenario id must be a non-empty string')
    _require(re.fullmatch(r'[a-z0-9_]+', scenario['id']) is not None,
             'scenario id must use lowercase letters, digits, and underscores')
    _require(scenario.get('town') == 'Town04', 'schema v1 requires Town04')
    backends = scenario.get('supported_backends')
    _require(isinstance(backends, list) and 'carla' in backends and
             set(backends) <= SUPPORTED_BACKENDS,
             'scenario does not support the CARLA backend')
    seed = scenario.get('seed')
    _require(isinstance(seed, int) and not isinstance(seed, bool) and seed >= 0,
             'seed must be an unsigned integer')
    duration = _finite_number(scenario.get('duration_s'), 'duration_s')
    _require(duration >= 0.0, 'duration_s must be non-negative')
    ego = scenario.get('ego')
    _require(isinstance(ego, dict) and
             isinstance(ego.get('spawn_index'), int),
             'ego.spawn_index must be an integer')

    actors = scenario.get('actors')
    _require(isinstance(actors, list), 'actors must be an array')
    seen_ids = set()
    previous_id = -1
    positions = []
    for index, actor in enumerate(actors):
        field = 'actors[%d]' % index
        _require(isinstance(actor, dict), '%s must be an object' % field)
        actor_id = actor.get('id')
        _require(isinstance(actor_id, int) and actor_id > 0,
                 '%s.id must be positive' % field)
        _require(actor_id not in seen_ids,
                 '%s.id duplicates %d' % (field, actor_id))
        _require(actor_id > previous_id,
                 'actors must be ordered by ascending id')
        seen_ids.add(actor_id)
        previous_id = actor_id
        _require(actor.get('classification') in CLASSIFICATIONS,
                 '%s has unsupported classification' % field)
        _require(actor.get('lane') in LANES,
                 '%s has unsupported lane' % field)
        station = _finite_number(actor.get('initial_station_m'),
                                 '%s.initial_station_m' % field)
        lateral = _finite_number(actor.get('initial_lateral_m'),
                                 '%s.initial_lateral_m' % field)
        initial_speed = _finite_number(actor.get('initial_speed_mps'),
                                       '%s.initial_speed_mps' % field)
        accel_limit = _finite_number(actor.get('accel_limit_mps2'),
                                     '%s.accel_limit_mps2' % field)
        _require(0.0 <= initial_speed <= 80.0,
                 '%s initial speed is out of range' % field)
        _require(0.0 < accel_limit <= 20.0,
                 '%s acceleration limit is out of range' % field)
        for other_id, other_station, other_lateral in positions:
            _require(math.hypot(station - other_station,
                                lateral - other_lateral) >= 2.0,
                     '%s overlaps actor %d' % (field, other_id))
        positions.append((actor_id, station, lateral))

        profile = actor.get('speed_profile')
        _require(isinstance(profile, list) and profile,
                 '%s.speed_profile must be non-empty' % field)
        previous_time = -1.0
        for point in profile:
            _require(isinstance(point, list) and len(point) == 2,
                     '%s speed profile point must be [time, speed]' % field)
            time_s = _finite_number(point[0], '%s profile time' % field)
            speed = _finite_number(point[1], '%s profile speed' % field)
            _require(time_s >= 0.0 and time_s > previous_time,
                     '%s profile times must be strictly increasing' % field)
            _require(0.0 <= speed <= 80.0,
                     '%s profile speed is out of range' % field)
            previous_time = time_s
        _require(float(profile[0][0]) == 0.0 and
                 float(profile[0][1]) == initial_speed,
                 '%s profile must start at initial speed at t=0' % field)
    return scenario


def _read_scenario(path):
    try:
        scenario = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ScenarioLoadError('cannot load scenario %s: %s' %
                                (path, error)) from error
    return validate_loaded_scenario(scenario)


def load_scenario(scenario_id='acc', scenario_file=None, seed=None,
                  repo_root=None):
    """Load an explicit file first, otherwise catalog, then legacy fallback."""
    root = None
    if repo_root is not None:
        root = Path(repo_root).resolve()
    if scenario_file:
        path = Path(scenario_file).expanduser()
        if not path.is_absolute():
            path = ((root or find_repo_root()) / path).resolve()
        scenario = _read_scenario(path)
    else:
        try:
            root = root or find_repo_root()
        except ScenarioLoadError:
            if scenario_id not in SCENARIOS:
                raise
            scenario = validate_loaded_scenario(
                legacy_to_scripted(scenario_id))
            path = None
        else:
            entry = load_catalog(root).get(scenario_id)
            if entry is None:
                raise ScenarioLoadError('unknown scenario id %s' % scenario_id)
            path = (root / entry['file']).resolve()
            _require(path.parent == (root / 'scenarios').resolve(),
                     'scenario file must be directly below scenarios/')
            scenario = _read_scenario(path)

    scenario = dict(scenario)
    scenario['actors'] = [dict(actor) for actor in scenario['actors']]
    if seed is not None:
        _require(isinstance(seed, int) and seed >= 0,
                 'seed override must be an unsigned integer')
        scenario['seed'] = seed
    scenario['_source_file'] = str(path) if path else ''
    return scenario


def known_scenario_ids(repo_root=None):
    """Catalog IDs with a hard-coded fallback for legacy launchers."""
    try:
        return list(load_catalog(repo_root))
    except ScenarioLoadError:
        return list(ORDER)
