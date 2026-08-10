import importlib.util
import json
from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).parents[1] / 'launch' / 'scenario_overlay.py'
SPEC = importlib.util.spec_from_file_location('scenario_overlay', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_dense_scenario_flattens_stable_ids_and_profiles():
    scenario_file = MODULE.find_scenario_file(
        'dense_overtake_v1', start=Path(__file__))
    metadata, params = MODULE.load_scenario_overlay(
        scenario_file, seed=7)
    assert metadata['id'] == 'dense_overtake_v1'
    assert metadata['seed'] == 7
    assert metadata['actor_count'] == 20
    assert params['scripted.ids'] == list(range(1001, 1021))
    assert len(params['scripted.profile_offsets']) == 21
    assert params['scripted.profile_offsets'][-1] == len(
        params['scripted.profile_times_s'])
    assert params['track.segment_lengths_m'][0] >= 1200.0


def test_invalid_or_oversized_scenario_is_rejected(tmp_path):
    actors = []
    for actor_id in range(1, MODULE.MAX_ACTORS + 2):
        actors.append({
            'id': actor_id, 'classification': 'vehicle',
            'initial_station_m': actor_id * 3.0, 'initial_lateral_m': 0.0,
            'initial_speed_mps': 1.0, 'accel_limit_mps2': 1.0,
            'speed_profile': [[0.0, 1.0]],
        })
    path = tmp_path / 'oversized.json'
    path.write_text(json.dumps({
        'schema_version': 1, 'id': 'oversized',
        'supported_backends': ['sil'], 'seed': 0, 'duration_s': 10.0,
        'actors': actors,
    }), encoding='utf-8')
    with pytest.raises(MODULE.ScenarioOverlayError):
        MODULE.load_scenario_overlay(path)


def test_explicit_id_must_match_file(tmp_path):
    path = tmp_path / 'empty.json'
    path.write_text(json.dumps({
        'schema_version': 1, 'id': 'empty', 'supported_backends': ['sil'],
        'seed': 0, 'duration_s': 1.0, 'actors': [],
    }), encoding='utf-8')
    with pytest.raises(MODULE.ScenarioOverlayError):
        MODULE.load_scenario_overlay(path, scenario_id='different')
