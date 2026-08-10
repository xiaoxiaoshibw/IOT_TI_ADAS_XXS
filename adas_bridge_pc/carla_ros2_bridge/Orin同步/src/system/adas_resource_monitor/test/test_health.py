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


def test_can_unknown_is_not_healthy():
    assert not health.can_operstate_is_healthy("unknown")
    value = metrics()
    value["can_interface_up"] = health.can_operstate_is_healthy("unknown")
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


def test_gpu_temperature_reader_and_swap_usage(tmp_path):
    gpu = tmp_path / "gpu_temp"
    gpu.write_text("85000\n", encoding="ascii")
    assert health.gpu_temperature_c([gpu]) == 85.0

    meminfo = tmp_path / "meminfo"
    meminfo.write_text("SwapTotal:       1000 kB\nSwapFree:         250 kB\n",
                       encoding="ascii")
    assert health.swap_used_pct(meminfo) == 75.0


def test_swap_and_gpu_pressure_are_reported():
    value = metrics()
    value["swap_used_pct"] = 25.0
    value["gpu_temperature_c"] = 85.0
    thresholds = dict(THRESHOLDS, swap_used_warn=20.0, swap_used_error=50.0,
                      gpu_temperature_warn_c=80.0, gpu_temperature_error_c=90.0)
    assert evaluate(value, thresholds)[0] == LEVEL_WARN


def test_process_snapshot_reads_cpu_time_and_rss(tmp_path):
    process = tmp_path / "1234"
    process.mkdir()
    (process / "comm").write_text("trajectory_follower_node\n", encoding="ascii")
    fields = ["1234", "trajectory_follower_node", "S"] + ["0"] * 21
    fields[13] = "40"
    fields[14] = "20"
    fields[23] = "1024"
    (process / "stat").write_text(" ".join(fields), encoding="ascii")
    snapshot = health.process_snapshot(["trajectory_follower_node"], tmp_path,
                                       clock_ticks=100, page_size=4096)
    assert snapshot["trajectory_follower_node"]["cpu_time_s"] == 0.6
    assert snapshot["trajectory_follower_node"]["rss_mb"] == 4.0


def test_missing_critical_process_is_error():
    value = metrics()
    value["required_processes"] = ["can_gateway_node"]
    value["critical_processes"] = {}
    assert evaluate(value, THRESHOLDS)[0] == LEVEL_ERROR
