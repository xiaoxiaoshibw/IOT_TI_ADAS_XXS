import importlib.util
import csv
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "tools" / "hil" / "hil_metrics_collector.py"
SPEC = importlib.util.spec_from_file_location("hil_metrics_collector", MODULE_PATH)
assert SPEC and SPEC.loader
collector = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(collector)


def make_frame(can_id, first_seven):
    data = bytes(first_seven) + b"\x00"
    return (data[:7] + bytes([collector.frame_crc(can_id, data)])).hex().upper()


def test_decode_control_and_heartbeat():
    control = collector.decode_feedback(
        0x201, make_frame(0x201, [0x2C, 0x01, 0x18, 0xFC, 25, 0, 42])
    )
    assert control["sequence"] == 42
    assert control["steering"] == 0.1
    assert control["acceleration_mps2"] == -1.0
    assert control["throttle"] == 0.25
    heartbeat = collector.decode_feedback(
        0x202, make_frame(0x202, [2, 1, 0, 0, 9, 10, 11])
    )
    assert heartbeat["state"] == 2
    assert heartbeat["active_source"] == 1
    diagnostic = collector.decode_feedback(
        0x203, make_frame(0x203, [0, 0, 2, 1, 0, 0, 0])
    )
    assert diagnostic["reset_reason"] == 2
    assert diagnostic["primary_timeout_count"] == 1
    e2e = collector.decode_feedback(
        0x204, make_frame(0x204, [0, 0, 5, 3, 0, 0, 3])
    )
    assert e2e["can_recovery_count"] == 3


def test_parse_ip_link_stats():
    sample = """7: can1: <NOARP,UP,LOWER_UP> mtu 16 state UP mode DEFAULT
    can state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 100
      re-started bus-errors arbit-lost error-warn error-pass bus-off
      0          12         0          0          0          0
    RX: bytes  packets  errors  dropped missed  mcast
    1000       200      3       4       0       0
    TX: bytes  packets  errors  dropped carrier collsns
    2000       300      5       6       0       0
"""
    parsed = collector.parse_ip_link_stats(sample)
    assert parsed == {
        "rx_count": 200,
        "tx_count": 300,
        "error": 20,
        "drop": 10,
        "bus_error": 12,
        "bus_off": 0,
        "berr_tx": 0,
        "berr_rx": 0,
        "can_state": "ERROR-ACTIVE",
    }


def test_crc_error_is_rejected():
    payload = make_frame(0x201, [0, 0, 0, 0, 0, 0, 1])
    corrupted = payload[:-2] + "FF"
    try:
        collector.decode_feedback(0x201, corrupted)
    except ValueError:
        pass
    else:
        raise AssertionError("CRC error was accepted")


def test_write_timing_uses_all_rows(tmp_path):
    control_path = tmp_path / "mcu_control.csv"
    with control_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "timestamp", "sequence", "steering", "acceleration_mps2",
            "throttle", "brake", "state", "active_source",
        ])
        writer.writerow(["1.000", 10, 0, 0, 0, 0, 2, 1])
        writer.writerow(["1.010", 11, 0, 0, 0, 0, 2, 1])
        writer.writerow(["1.021", 12, 0, 0, 0, 0, 2, 1])
    timing_path = tmp_path / "mcu_timing.csv"
    metrics = collector.write_timing(control_path, timing_path)
    assert abs(metrics["period_average_ms"] - 10.5) < 1e-9
    assert metrics["sequence_gaps"] == 0
    rows = list(csv.DictReader(timing_path.open(encoding="utf-8")))
    assert rows[-1]["running_average_ms"] == "10.500000"
