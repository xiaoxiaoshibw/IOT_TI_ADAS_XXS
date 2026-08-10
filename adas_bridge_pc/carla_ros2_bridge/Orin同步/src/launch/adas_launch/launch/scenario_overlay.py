"""Translate audited scenario-v1 JSON into ROS-native flat parameter arrays."""

import json
import math
from pathlib import Path


SCHEMA_VERSION = 1
MAX_ACTORS = 64
CLASSIFICATIONS = {
    'unknown': 0,
    'vehicle': 1,
    'pedestrian': 3,
    'cyclist': 4,
}


class ScenarioOverlayError(ValueError):
    """The scenario cannot be represented safely by the SIL backend."""


def _require(condition, message):
    if not condition:
        raise ScenarioOverlayError(message)


def _number(value, field):
    _require(isinstance(value, (int, float)) and not isinstance(value, bool),
             '%s must be numeric' % field)
    result = float(value)
    _require(math.isfinite(result), '%s must be finite' % field)
    return result


def find_scenario_file(scenario_id, start=None):
    """Resolve a catalog ID in a checkout without adding a runtime JSON library."""
    _require(scenario_id, 'scenario_id is required when scenario_file is empty')
    bases = [Path(start or Path.cwd()).resolve(), Path(__file__).resolve()]
    visited = set()
    for base in bases:
        if base.is_file():
            base = base.parent
        for parent in (base, *base.parents):
            if parent in visited:
                continue
            visited.add(parent)
            candidate = parent / 'scenarios' / (scenario_id + '.json')
            if candidate.is_file():
                return candidate
    raise ScenarioOverlayError('cannot locate scenario id %s' % scenario_id)


def load_scenario_overlay(scenario_file='', scenario_id='', seed=None, start=None):
    """Return ``(metadata, sim_parameters)`` for local-three-machine launch."""
    path = (Path(scenario_file).expanduser().resolve() if scenario_file else
            find_scenario_file(scenario_id, start=start))
    try:
        scenario = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ScenarioOverlayError('cannot load %s: %s' % (path, error)) from error

    _require(scenario.get('schema_version') == SCHEMA_VERSION,
             'unsupported scenario schema_version')
    _require(isinstance(scenario.get('id'), str) and scenario['id'],
             'scenario id must be non-empty')
    if scenario_id:
        _require(scenario['id'] == scenario_id,
                 'scenario_id does not match scenario file')
    _require('sil' in scenario.get('supported_backends', []),
             'scenario does not support SIL')
    file_seed = scenario.get('seed')
    _require(isinstance(file_seed, int) and not isinstance(file_seed, bool) and
             file_seed >= 0, 'seed must be an unsigned integer')
    if seed is not None:
        _require(isinstance(seed, int) and not isinstance(seed, bool) and seed >= 0,
                 'seed override must be an unsigned integer')
    effective_seed = file_seed if seed is None else seed
    duration_s = _number(scenario.get('duration_s'), 'duration_s')
    _require(duration_s >= 0.0, 'duration_s must be non-negative')
    actors = scenario.get('actors')
    _require(isinstance(actors, list), 'actors must be an array')
    _require(len(actors) <= MAX_ACTORS, 'actor count exceeds SIL safety limit')

    values = {
        'scripted.enabled': True,
        'scripted.ids': [],
        'scripted.classifications': [],
        'scripted.initial_station_m': [],
        'scripted.initial_lateral_m': [],
        'scripted.initial_speed_mps': [],
        'scripted.accel_limit_mps2': [],
        'scripted.profile_offsets': [0],
        'scripted.profile_times_s': [],
        'scripted.profile_speeds_mps': [],
        'scripted.hard_brake_start_s': [],
        'scripted.hard_brake_end_s': [],
        'scripted.trigger_ego_gap_m': [],
        'scripted.crossing_end_lateral_m': [],
        'scripted.crossing_speed_mps': [],
    }
    previous_id = 0
    max_station = 500.0
    for index, actor in enumerate(actors):
        field = 'actors[%d]' % index
        _require(isinstance(actor, dict), '%s must be an object' % field)
        actor_id = actor.get('id')
        _require(isinstance(actor_id, int) and not isinstance(actor_id, bool) and
                 actor_id > previous_id, 'actor IDs must be positive and sorted')
        previous_id = actor_id
        classification = actor.get('classification')
        _require(classification in CLASSIFICATIONS,
                 '%s classification is unsupported' % field)
        station = _number(actor.get('initial_station_m'), field + '.initial_station_m')
        lateral = _number(actor.get('initial_lateral_m'), field + '.initial_lateral_m')
        speed = _number(actor.get('initial_speed_mps'), field + '.initial_speed_mps')
        accel = _number(actor.get('accel_limit_mps2'), field + '.accel_limit_mps2')
        _require(speed >= 0.0 and accel > 0.0, '%s speed/acceleration is invalid' % field)
        profile = actor.get('speed_profile')
        _require(isinstance(profile, list) and profile,
                 '%s speed_profile must be non-empty' % field)
        previous_time = -1.0
        profile_speeds = []
        for point in profile:
            _require(isinstance(point, list) and len(point) == 2,
                     '%s profile point must be [time,speed]' % field)
            time_s = _number(point[0], field + '.profile_time')
            target = _number(point[1], field + '.profile_speed')
            _require(time_s > previous_time and target >= 0.0,
                     '%s profile must be increasing and non-negative' % field)
            previous_time = time_s
            profile_speeds.append(target)
            values['scripted.profile_times_s'].append(time_s)
            values['scripted.profile_speeds_mps'].append(target)
        _require(float(profile[0][0]) == 0.0 and float(profile[0][1]) == speed,
                 '%s profile must begin with initial speed' % field)
        values['scripted.profile_offsets'].append(
            len(values['scripted.profile_times_s']))
        values['scripted.ids'].append(actor_id)
        values['scripted.classifications'].append(CLASSIFICATIONS[classification])
        values['scripted.initial_station_m'].append(station)
        values['scripted.initial_lateral_m'].append(lateral)
        values['scripted.initial_speed_mps'].append(speed)
        values['scripted.accel_limit_mps2'].append(accel)
        brake = actor.get('hard_brake_window_s', [-1.0, -1.0])
        _require(isinstance(brake, list) and len(brake) == 2,
                 '%s hard_brake_window_s must have two values' % field)
        values['scripted.hard_brake_start_s'].append(_number(brake[0], field + '.brake_start'))
        values['scripted.hard_brake_end_s'].append(_number(brake[1], field + '.brake_end'))
        trigger = _number(actor.get('trigger_ego_gap_m', -1.0), field + '.trigger_gap')
        crossing_speed = _number(actor.get('crossing_speed_mps', speed),
                                 field + '.crossing_speed')
        values['scripted.trigger_ego_gap_m'].append(trigger)
        values['scripted.crossing_end_lateral_m'].append(
            -lateral if classification == 'pedestrian' else lateral)
        values['scripted.crossing_speed_mps'].append(
            crossing_speed if classification == 'pedestrian' else 0.0)
        max_station = max(max_station, station + max(profile_speeds) * duration_s + 100.0)

    # Preserve the baseline track for old YAML; an explicit JSON scenario gets a
    # long deterministic straight track so actors cannot disappear before duration_s.
    values['track.segment_lengths_m'] = [max(1200.0, max_station)]
    values['track.segment_curvatures'] = [0.0]
    values['track.lane_width_m'] = 3.5
    # Empty arrays cannot be type-inferred by some ROS launch parameter adapters.
    if not actors:
        values = {key: value for key, value in values.items()
                  if value != []}

    metadata = {
        'id': scenario['id'],
        'seed': effective_seed,
        'actor_count': len(actors),
        'actor_ids': [actor['id'] for actor in actors],
        'duration_s': duration_s,
        'source_file': str(path),
    }
    return metadata, values
