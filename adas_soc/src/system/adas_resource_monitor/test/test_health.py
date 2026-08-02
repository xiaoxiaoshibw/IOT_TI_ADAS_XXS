from adas_resource_monitor import health
from adas_resource_monitor.health import LEVEL_ERROR, LEVEL_OK, LEVEL_WARN, evaluate


THRESHOLDS = {
    "memory_available_warn": 15.0, "memory_available_error": 7.0,
    "disk_free_warn": 15.0, "disk_free_error": 7.0,
    "temperature_warn_c": 80.0, "temperature_error_c": 90.0,
    "normalized_load_warn": 1.0, "normalized_load_error": 1.5,
}


def metrics():
    return {"memory_available_pct": 50.0, "disk_free_pct": 50.0,
            "max_temperature_c": 55.0, "normalized_load_1m": 0.2,
            "can_interface_up": True, "log_path_writable": True}


def test_nominal():
    assert evaluate(metrics(), THRESHOLDS)[0] == LEVEL_OK


def test_temperature_warning():
    value = metrics()
    value["max_temperature_c"] = 82.0
    assert evaluate(value, THRESHOLDS)[0] == LEVEL_WARN


def test_can_down_is_error():
    value = metrics()
    value["can_interface_up"] = False
    assert evaluate(value, THRESHOLDS)[0] == LEVEL_ERROR


def test_disk_exhaustion_is_error():
    value = metrics()
    value["disk_free_pct"] = 3.0
    assert evaluate(value, THRESHOLDS)[0] == LEVEL_ERROR


def test_temperature_reader_skips_unavailable_sysfs_entry(monkeypatch):
    monkeypatch.setattr(health.glob, "glob", lambda _pattern: ["bad", "good"])

    def read_text(path, **_kwargs):
        if str(path) == "bad":
            raise TypeError("thermal driver returned no payload")
        return "55000\n"

    monkeypatch.setattr(health.pathlib.Path, "read_text", read_text)
    assert health.max_temperature_c() == 55.0
