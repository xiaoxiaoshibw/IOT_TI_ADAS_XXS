#!/usr/bin/env python3
import unittest

from safety_hmi import format_age, render_dtc, render_status


def status_fixture(**overrides):
    status = {
        "system_state": 2, "active_source": 1, "fault_level": 0,
        "fault_code": 0, "primary_fresh": True, "backup_fresh": True,
        "aeb_floor_active": False, "degraded": False,
        "manual_override": False, "estop": False,
        "protocol_version": 3, "protocol_version_ok": True, "test_build": False,
        "heartbeat_age_s": 0.05, "feedback_age_s": 0.02, "command_age_s": 0.01,
        "degrade_reason": "none",
    }
    status.update(overrides)
    return status


class SafetyHmiTest(unittest.TestCase):
    def test_active_frame_shows_source_and_state(self):
        text = "\n".join(render_status(status_fixture()))
        self.assertIn("MCU ACTIVE", text)
        self.assertIn("control source : PRIMARY", text)
        self.assertIn("fault code     : 0x0000", text)
        self.assertNotIn("HEARTBEAT STALE", text)

    def test_fault_lock_shows_reason_and_flags(self):
        text = "\n".join(render_status(status_fixture(
            system_state=7, active_source=9, fault_code=0x2811,
            degrade_reason="primary_timeout+fault_lock+can_bus_off",
            aeb_floor_active=True)))
        self.assertIn("MCU FAULT_LOCK", text)
        self.assertIn("MCU_WATCHDOG", text)
        self.assertIn("0x2811", text)
        self.assertIn("primary_timeout+fault_lock+can_bus_off", text)
        self.assertIn("AEB_FLOOR", text)

    def test_missing_data_is_reported_not_hidden(self):
        text = "\n".join(render_status({}))
        self.assertIn("NO DATA", text)
        self.assertIn("HEARTBEAT STALE", text)
        self.assertIn("command age    : never", text)

    def test_stale_heartbeat_is_flagged(self):
        text = "\n".join(render_status(status_fixture(heartbeat_age_s=0.8)))
        self.assertIn("HEARTBEAT STALE", text)

    def test_age_formatting(self):
        self.assertEqual(format_age(-1.0), "never")
        self.assertEqual(format_age(None), "never")
        self.assertEqual(format_age(0.05), "50 ms")
        self.assertEqual(format_age(2.34), "2.3 s")

    def test_dtc_rendering_lists_active_latched_records(self):
        history = {"records": [
            {"code": "DTC-MCU-2011", "name": "mcu_fault_lock", "active": True,
             "occurrences": 2, "safety_action": "FAULT_LOCK", "latched": True},
            {"code": "DTC-MCU-2001", "name": "mcu_primary_timeout", "active": False,
             "occurrences": 5, "safety_action": "DEGRADED", "latched": False},
        ]}
        text = "\n".join(render_dtc(history))
        self.assertIn("active DTCs      : 1", text)
        self.assertIn("DTC-MCU-2011", text)
        self.assertIn("[LATCHED]", text)
        self.assertNotIn("DTC-MCU-2001", text)


if __name__ == "__main__":
    unittest.main()
