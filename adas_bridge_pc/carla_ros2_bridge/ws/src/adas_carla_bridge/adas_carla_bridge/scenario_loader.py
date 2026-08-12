#!/usr/bin/env python3
"""Load frozen scenario-v1 JSON with legacy-ID fallback."""

import json
import math
import re
from pathlib import Path

from adas_carla_bridge.scenarios import ORDER, SCENARIOS, legacy_to_scripted


SCHEMA_VERSION = 1
# P1.E: 仓库 commit 以来，scenario catalog 唯一真相。10 个 ID 必须严格一一对应；
# 任何字段缺失/ID 错误/蓝图与 actor 数量不一致，都视作非法 catalog。
EXPECTED_CATALOG_IDS = frozenset({
    'lka', 'acc', 'acc_stop_and_go', 'acc_slow_truck', 'overtake',
    'aeb', 'aeb_stationary', 'aeb_pedestrian', 'free', 'dense_overtake_v1',
})
REQUIRED_CATALOG_FIELDS = (
    'id', 'display_name', 'file', 'default_town', 'supported_backends',
    'default_seed', 'expected_actor_count', 'duration_s', 'actor_blueprints',
    'acceptance_profile', 'nav_distance_m',
)
SUPPORTED_BACKENDS = {'carla', 'sil'}
CLASSIFICATIONS = {'vehicle', 'pedestrian', 'cyclist', 'unknown'}
LANES = {'ego', 'left', 'right', 'crossing'}


class ScenarioLoadError(ValueError):
    """Raised when a scenario cannot be loaded without guessing."""


class CatalogValidationError(ScenarioLoadError):
    """P1.E: catalog 不合法（缺字段、ID 集合不一致、蓝图数量不匹配等）。"""


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
    _require(isinstance(catalog.get('schema_version'), int) and
             catalog['schema_version'] >= SCHEMA_VERSION,
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


def known_scenario_ids(repo_root=None, strict=True):
    """P1.E: 默认严格返回 catalog ID；catalog 缺失/解析失败时不再静默回退。

    strict=False 仅供遗留 CLI/测试使用——业务代码（GUI、桥节点）必须传 strict=True
    或直接调用 validate_catalog() 自行检查。"""
    if not strict:
        try:
            return list(load_catalog(repo_root))
        except ScenarioLoadError:
            return list(ORDER)
    validate_catalog(repo_root)
    return sorted(load_catalog(repo_root))


def validate_catalog(repo_root=None):
    """P1.E: 静态校验 catalog 作为唯一真相。

    - 必填字段、类型、ID 集合（10 个）严格一致；
    - 每个 scenario 的 file 引用必须存在；
    - actor_blueprints 长度必须 == expected_actor_count；
    - 任何错误以 CatalogValidationError 抛出，错误信息以 '; ' 串联所有问题。"""
    root = Path(repo_root).resolve() if repo_root else find_repo_root()
    path = root / 'scenarios/catalog.json'
    try:
        catalog = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise CatalogValidationError('cannot load catalog %s: %s' %
                                      (path, error)) from error
    errors = []
    schema = catalog.get('schema_version')
    if not isinstance(schema, int) or schema < SCHEMA_VERSION:
        errors.append('schema_version must be int >= %d, got %r' %
                      (SCHEMA_VERSION, schema))
    scenarios = catalog.get('scenarios')
    if not isinstance(scenarios, list) or not scenarios:
        errors.append('scenarios must be a non-empty array')
    if scenarios:
        ids = [s.get('id') for s in scenarios
               if isinstance(s, dict) and isinstance(s.get('id'), str)]
        seen = set()
        dups = sorted({i for i in ids if i in seen or seen.add(i)})
        if dups:
            errors.append('duplicate ids: %s' % ','.join(dups))
        missing = sorted(EXPECTED_CATALOG_IDS - set(ids))
        extra = sorted(set(ids) - EXPECTED_CATALOG_IDS)
        if missing:
            errors.append('missing expected ids: %s' % ','.join(missing))
        if extra:
            errors.append('unexpected ids: %s' % ','.join(extra))
    if errors:
        raise CatalogValidationError('; '.join(errors))
    for entry in scenarios:
        scenario_id = entry.get('id', '<unknown>')
        for field in REQUIRED_CATALOG_FIELDS:
            if field not in entry:
                errors.append('%s: missing required field %r' %
                              (scenario_id, field))
        if errors:
            break
        if not isinstance(entry['expected_actor_count'], int) or \
                entry['expected_actor_count'] < 0:
            errors.append('%s: expected_actor_count must be non-negative int' %
                          scenario_id)
        duration = entry['duration_s']
        if not isinstance(duration, (int, float)) or isinstance(duration, bool) or \
                not math.isfinite(duration) or duration < 0.0:
            errors.append('%s: duration_s must be finite non-negative number' %
                          scenario_id)
        nav = entry['nav_distance_m']
        if not isinstance(nav, (int, float)) or isinstance(nav, bool) or \
                not math.isfinite(nav) or nav < 0.0:
            errors.append('%s: nav_distance_m must be finite non-negative number'
                          % scenario_id)
        bps = entry['actor_blueprints']
        if not isinstance(bps, list) or len(bps) != entry['expected_actor_count']:
            errors.append(
                '%s: actor_blueprints length %d != expected_actor_count %d' %
                (scenario_id, len(bps) if isinstance(bps, list) else -1,
                 entry['expected_actor_count']))
        # P1.E: actor_blueprints 中 null / "-" 都视作"使用默认 blueprint"；
        # 非空字符串必须以字母+数字开头且不含空格。
        for i, bp in enumerate(bps if isinstance(bps, list) else []):
            if bp is None:
                continue
            if not isinstance(bp, str):
                errors.append('%s: actor_blueprints[%d] must be string or null' %
                              (scenario_id, i))
                continue
            if not bp or bp == '-':
                continue
            if ' ' in bp or not bp[0].isalpha():
                errors.append(
                    '%s: actor_blueprints[%d]=%r is not a valid CARLA blueprint id'
                    % (scenario_id, i, bp))
        # 验证 file 字段引用必须存在。
        file_path = root / entry['file']
        if not file_path.is_file():
            errors.append('%s: file %s does not exist' % (scenario_id, file_path))
        acceptance = entry['acceptance_profile']
        if not isinstance(acceptance, str) or not acceptance:
            errors.append('%s: acceptance_profile must be non-empty str' %
                          scenario_id)
        backends = entry['supported_backends']
        if not isinstance(backends, list) or 'carla' not in backends:
            errors.append('%s: supported_backends must include carla' %
                          scenario_id)
    if errors:
        raise CatalogValidationError('; '.join(errors))
    return True


if __name__ == '__main__':  # pragma: no cover
    import argparse
    import sys
    parser = argparse.ArgumentParser(description='catalog validator / scenario loader')
    parser.add_argument('--repo-root', default=None)
    sub = parser.add_subparsers(dest='cmd', required=False)
    p_val = sub.add_parser('validate-catalog',
                           help='静态校验 scenarios/catalog.json（10 ID、字段、蓝图数量）')
    p_val.add_argument('--repo-root', default=None)
    p_load = sub.add_parser('load',
                            help='载入单条 scenario（兼容旧 CLI）')
    p_load.add_argument('--scenario-id', default='acc')
    p_load.add_argument('--scenario-file', default='')
    p_load.add_argument('--seed', type=int, default=None)
    p_load.add_argument('--repo-root', default=None)
    args = parser.parse_args()
    if args.cmd == 'validate-catalog':
        try:
            validate_catalog(args.repo_root)
        except CatalogValidationError as error:
            print('catalog invalid: %s' % error, file=sys.stderr)
            sys.exit(2)
        print('catalog OK: 10 scenarios, fields and IDs match')
        sys.exit(0)
    if args.cmd == 'load':
        try:
            scenario = load_scenario(
                scenario_id=args.scenario_id, scenario_file=args.scenario_file,
                seed=args.seed, repo_root=args.repo_root)
        except ScenarioLoadError as error:
            print('error: %s' % error, file=sys.stderr)
            sys.exit(2)
        import json as _json
        print(_json.dumps(scenario, ensure_ascii=False, indent=2))
        sys.exit(0)
    parser.print_help()
    sys.exit(1)
