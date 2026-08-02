import importlib.util
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("check_no_route_c4.py")
SPEC = importlib.util.spec_from_file_location("check_no_route_c4", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_numbers_ignores_empty_values():
    assert MODULE.numbers([{"value": ""}, {"value": "1.5"}], "value") == [1.5]


def test_c4_thresholds_are_conservative():
    source = MODULE_PATH.read_text(encoding="utf-8")
    assert "max(speed) <= 0.1" in source
    assert "displacement <= 0.5" in source
    assert "max(carla_throttle) == 0.0" in source
