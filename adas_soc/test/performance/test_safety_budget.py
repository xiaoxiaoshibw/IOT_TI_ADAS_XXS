#!/usr/bin/env python3
import unittest

from safety_budget import validate_report


def valid_report():
    topics = {}
    for name in ("odom", "objects", "behavior", "trajectory", "follower", "gate", "actuation"):
        topics[name] = {"expected_hz": 10.0, "measured_hz": 10.0,
                        "period": {"p99_ms": 20.0}}
    stages = {name: {"p99_ms": 20.0} for name in (
        "objects_to_behavior", "behavior_to_trajectory", "trajectory_to_follower",
        "follower_to_gate", "gate_to_actuation")}
    return {"topics": topics, "stages": stages}


class SafetyBudgetTest(unittest.TestCase):
    def test_valid_report_passes(self):
        self.assertEqual(validate_report(valid_report()), [])

    def test_missing_stage_evidence_fails(self):
        report = valid_report()
        del report["stages"]["gate_to_actuation"]
        self.assertIn("gate_to_actuation stage P99 missing", validate_report(report))

    def test_slow_control_period_fails(self):
        report = valid_report()
        report["topics"]["gate"]["period"]["p99_ms"] = 22.1
        self.assertTrue(any("gate period P99" in item for item in validate_report(report)))


if __name__ == "__main__":
    unittest.main()
