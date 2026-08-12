import copy
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[6]
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
VALIDATOR_PATH = REPO_ROOT / "tools/validate_scenario.py"
SPEC = importlib.util.spec_from_file_location("scenario_validator", VALIDATOR_PATH)
assert SPEC is not None and SPEC.loader is not None
validator = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = validator
SPEC.loader.exec_module(validator)

from adas_carla_bridge import scenario_loader


def load_scenario(scenario_id):
    return json.loads(
        (REPO_ROOT / "scenarios" / f"{scenario_id}.json").read_text(
            encoding="utf-8"))


def test_repository_catalog_is_valid_and_legacy_complete():
    report = validator.validate_catalog(
        REPO_ROOT / "scenarios/catalog.json", REPO_ROOT)

    assert report["status"] == "PASS"
    assert report["scenario_count"] == 10
    assert report["legacy_scenario_count"] == 9
    dense = next(item for item in report["scenarios"]
                 if item["id"] == "dense_overtake_v1")
    assert dense["actor_count"] == 20


def test_all_legacy_scenarios_are_field_equivalent():
    legacy, order = validator.load_legacy_scenarios(REPO_ROOT)

    assert len(order) == 9
    for scenario_id in order:
        projected = validator.legacy_projection(load_scenario(scenario_id))
        assert validator._json_shape(projected) == validator._json_shape(
            legacy[scenario_id])


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (lambda data: data["actors"].append(copy.deepcopy(data["actors"][0])),
         "duplicate actor ID"),
        (lambda data: data["actors"][0]["speed_profile"].append([0.0, 3.0]),
         "strictly increasing"),
        (lambda data: data["actors"][1].update(
            initial_station_m=data["actors"][0]["initial_station_m"],
            initial_lateral_m=data["actors"][0]["initial_lateral_m"]),
         "initial position overlaps"),
        (lambda data: data.update(supported_backends=["carla", "gazebo"]),
         "unsupported backend"),
    ],
)
def test_invalid_scenario_is_rejected(mutate, message):
    data = load_scenario("dense_overtake_v1")
    mutate(data)

    with pytest.raises(validator.ScenarioValidationError, match=message):
        validator.validate_scenario(data, "test")


def test_validator_cli_emits_machine_readable_report():
    completed = subprocess.run(
        [sys.executable, str(VALIDATOR_PATH), "--json"],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    report = json.loads(completed.stdout)
    assert report["status"] == "PASS"
    assert report["scenario_count"] == 10


def test_runtime_loader_prefers_explicit_file_and_applies_seed():
    scenario = scenario_loader.load_scenario(
        scenario_id="lka",
        scenario_file="scenarios/dense_overtake_v1.json",
        seed=7,
        repo_root=REPO_ROOT,
    )

    assert scenario["id"] == "dense_overtake_v1"
    assert scenario["seed"] == 7
    assert len(scenario["actors"]) == 20
    assert scenario["_source_file"].endswith("dense_overtake_v1.json")


def test_runtime_loader_legacy_fallback_preserves_ids(monkeypatch):
    def unavailable(_start=None):
        raise scenario_loader.ScenarioLoadError("catalog unavailable")

    monkeypatch.setattr(scenario_loader, "find_repo_root", unavailable)
    scenario = scenario_loader.load_scenario("aeb_pedestrian")

    assert scenario["id"] == "aeb_pedestrian"
    assert [actor["id"] for actor in scenario["actors"]] == [2]
    assert scenario["actors"][0]["legacy_role"] == "pedestrian"
    assert scenario["_source_file"] == ""


def test_runtime_loader_rejects_unsorted_actor_ids(tmp_path):
    scenario = load_scenario("dense_overtake_v1")
    scenario["actors"][0], scenario["actors"][1] = (
        scenario["actors"][1], scenario["actors"][0])
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(scenario, ensure_ascii=False), encoding="utf-8")

    with pytest.raises(scenario_loader.ScenarioLoadError,
                       match="ascending id"):
        scenario_loader.load_scenario(scenario_file=path)


# ── P1.E: catalog 作为唯一真相的静态校验 ──

def test_catalog_validate_passes_for_repository_default():
    """P1.E: 仓库自带 catalog 必须通过静态校验。"""
    scenario_loader.validate_catalog(REPO_ROOT)


def test_catalog_known_ids_equals_expected_set():
    """P1.E: 默认严格 known_scenario_ids 必须返回排序后的 10 个 ID。"""
    ids = set(scenario_loader.known_scenario_ids(REPO_ROOT, strict=True))
    assert ids == set(scenario_loader.EXPECTED_CATALOG_IDS)
    assert len(ids) == 10


def test_catalog_rejects_missing_actor_blueprint_field(tmp_path):
    """P1.E: 缺 actor_blueprints 字段必须报错。"""
    bad = json.loads((REPO_ROOT / "scenarios/catalog.json").read_text(encoding="utf-8"))
    del bad["scenarios"][0]["actor_blueprints"]
    fake_scenarios = tmp_path / "scenarios"
    fake_scenarios.mkdir()
    (fake_scenarios / "catalog.json").write_text(
        json.dumps(bad, ensure_ascii=False), encoding="utf-8")
    with pytest.raises(scenario_loader.CatalogValidationError,
                       match="actor_blueprints"):
        scenario_loader.validate_catalog(tmp_path)


def test_catalog_rejects_blueprint_count_mismatch(tmp_path):
    """P1.E: actor_blueprints 长度 != expected_actor_count 必须报错。"""
    bad = json.loads((REPO_ROOT / "scenarios/catalog.json").read_text(encoding="utf-8"))
    bad["scenarios"][1]["actor_blueprints"] = []  # acc 需要 1 个，被改成 0
    fake_scenarios = tmp_path / "scenarios"
    fake_scenarios.mkdir()
    for entry in bad["scenarios"]:
        (fake_scenarios / Path(entry["file"]).name).write_text(
            '{"schema_version": 1, "id": "placeholder"}', encoding="utf-8")
    (fake_scenarios / "catalog.json").write_text(
        json.dumps(bad, ensure_ascii=False), encoding="utf-8")
    with pytest.raises(scenario_loader.CatalogValidationError,
                       match="actor_blueprints length"):
        scenario_loader.validate_catalog(tmp_path)


def test_catalog_rejects_missing_expected_id(tmp_path):
    """P1.E: 少一个 ID 必须报错（10 个集合严格一致）。"""
    bad = json.loads((REPO_ROOT / "scenarios/catalog.json").read_text(encoding="utf-8"))
    bad["scenarios"] = [s for s in bad["scenarios"] if s["id"] != "lka"]
    fake_scenarios = tmp_path / "scenarios"
    fake_scenarios.mkdir()
    (fake_scenarios / "catalog.json").write_text(
        json.dumps(bad, ensure_ascii=False), encoding="utf-8")
    with pytest.raises(scenario_loader.CatalogValidationError,
                       match="missing expected ids"):
        scenario_loader.validate_catalog(tmp_path)


def test_catalog_strict_known_ids_does_not_silently_fallback(monkeypatch):
    """P1.E: strict 模式下 catalog 加载失败时禁止回退到硬编码 ID。"""
    def unavailable(_start=None):
        raise scenario_loader.CatalogValidationError("catalog unavailable")
    monkeypatch.setattr(scenario_loader, "find_repo_root", unavailable)
    with pytest.raises(scenario_loader.CatalogValidationError):
        scenario_loader.known_scenario_ids(None, strict=True)
    # 显式 opt-in 仍允许走遗留回退（仅供测试 / 旧 CLI 兜底）。
    fallback = scenario_loader.known_scenario_ids(None, strict=False)
    assert "acc" in fallback
