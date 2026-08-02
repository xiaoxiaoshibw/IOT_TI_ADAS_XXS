#!/usr/bin/env python3
import json
import pathlib
import unittest

from run_mcu_fault_matrix import CANID_FAULT_INJECT, crc8, injection_frame, protect, valid


class FaultMatrixRunnerTest(unittest.TestCase):
    def test_standard_crc_vector(self):
        self.assertEqual(crc8(b"123456789"), 0xA2)

    def test_injection_frame_is_data_id_protected(self):
        frame = injection_frame(2, 9)
        self.assertTrue(valid(CANID_FAULT_INJECT, frame))
        self.assertEqual(frame[0], 2)
        self.assertEqual(frame[5], 9)
        self.assertFalse(valid(0x302, frame))

    def test_matrix_ids_and_commands_are_unique(self):
        path = pathlib.Path(__file__).with_name("fault_matrix.json")
        matrix = json.loads(path.read_text(encoding="utf-8"))
        ids = [case["id"] for case in matrix["cases"]]
        commands = [case["command"] for case in matrix["cases"]]
        self.assertEqual(len(ids), len(set(ids)))
        self.assertEqual(len(commands), len(set(commands)))
        self.assertTrue(all(0 < command <= 9 for command in commands))
        self.assertTrue(all(0 <= case["expected_state"] <= 7 for case in matrix["cases"]))

    def test_matrix_covers_p0_fault_states(self):
        """P0 acceptance: FAILSAFE, MRM and both FAULT_LOCK causes must be injectable."""
        path = pathlib.Path(__file__).with_name("fault_matrix.json")
        matrix = json.loads(path.read_text(encoding="utf-8"))
        expected_states = {case["expected_state"] for case in matrix["cases"]}
        # DEGRADED(3), MRM(4), EMERGENCY_BRAKE(5), FAILSAFE(6), FAULT_LOCK(7)
        self.assertTrue({3, 4, 5, 6, 7}.issubset(expected_states))
        names = {case["name"] for case in matrix["cases"]}
        self.assertIn("can_recovery_exhausted", names)
        self.assertIn("self_test_fail", names)
        self.assertIn("drop_all_sources", names)

    def test_lock_states_require_full_brake_evidence(self):
        path = pathlib.Path(__file__).with_name("fault_matrix.json")
        matrix = json.loads(path.read_text(encoding="utf-8"))
        for case in matrix["cases"]:
            if case["expected_state"] in (5, 7):
                self.assertTrue(case["require_full_brake"], case["id"])
            if case["expected_state"] in (4, 6):
                self.assertTrue(case.get("require_zero_throttle"), case["id"])


if __name__ == "__main__":
    unittest.main()
