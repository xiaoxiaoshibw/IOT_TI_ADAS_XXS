import json
import pathlib

from adas_dtc_recorder.store import DtcStore


CATALOG = [{
    "code": "DTC-1", "name": "test", "source": "unit", "severity": "CRITICAL",
    "safety_action": "MRM", "latched": False,
}, {
    "code": "DTC-2", "name": "latched", "source": "unit2", "severity": "LOCKED",
    "safety_action": "FAULT_LOCK", "latched": True,
}]


def test_persists_occurrences_and_duration(tmp_path):
    path = tmp_path / "dtc.json"
    store = DtcStore(path, CATALOG)
    store.observe("DTC-1", 10.0, {"speed_mps": 4.0})
    store.observe("DTC-1", 11.0, {"speed_mps": 3.0})
    store.clear("DTC-1", 13.5)
    store.flush()

    restored = DtcStore(path, CATALOG)
    record = restored.records["DTC-1"]
    assert record["occurrences"] == 1
    assert record["total_active_s"] == 3.5
    assert record["first_freeze_frame"]["speed_mps"] == 4.0
    assert record["last_freeze_frame"]["speed_mps"] == 3.0


def test_latched_record_cannot_auto_clear(tmp_path):
    store = DtcStore(tmp_path / "dtc.json", CATALOG)
    store.observe("DTC-2", 1.0, {})
    store.clear("DTC-2", 2.0)
    assert store.records["DTC-2"]["active"]


def test_mcu_fault_catalog_covers_all_fc_bits(tmp_path):
    """FC_* bits 0..13 (MCU/include/safety.h) must each map to one unique DTC."""
    catalog_path = (pathlib.Path(__file__).resolve().parents[1]
                    / "config" / "fault_catalog.json")
    catalog = json.loads(catalog_path.read_text(encoding="utf-8"))["faults"]
    mcu = [item for item in catalog if "mcu_fault_bit" in item]
    assert sorted(item["mcu_fault_bit"] for item in mcu) == list(range(14))
    assert len({item["code"] for item in mcu}) == len(mcu)
    # 锁定类故障必须 latched，禁止自动清除
    for item in mcu:
        if item["safety_action"] == "FAULT_LOCK":
            assert item["latched"], item["code"]
        assert item["severity"] in (
            "OK", "DEGRADED", "WARNING", "CRITICAL", "EMERGENCY", "LOCKED")

    store = DtcStore(tmp_path / "dtc.json", catalog)
    fault_code = (1 << 10) | (1 << 4)   # fault_lock + all_sources_lost
    for item in mcu:
        code = item["code"]
        if fault_code & (1 << item["mcu_fault_bit"]):
            store.observe(code, 1.0, {"fault_code": fault_code})
        else:
            store.clear(code, 1.0)
    active = {code for code, record in store.records.items() if record["active"]}
    assert active == {"DTC-MCU-2011", "DTC-MCU-2005"}
    # fault_lock 是 latched：故障位消失也不允许自动清除
    store.clear("DTC-MCU-2011", 2.0)
    assert store.records["DTC-MCU-2011"]["active"]


def test_corrupt_file_does_not_prevent_startup(tmp_path):
    path = tmp_path / "dtc.json"
    path.write_text("not json", encoding="utf-8")
    store = DtcStore(path, CATALOG)
    assert store.records == {}
    store.observe("DTC-1", 1.0, {})
    store.flush()
    assert json.loads(path.read_text(encoding="utf-8"))["schema_version"] == 1
