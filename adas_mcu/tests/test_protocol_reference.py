import pathlib
import sys
import unittest

TOOLS = pathlib.Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))

from orin_can_reference import (  # noqa: E402
    CANID_MCU_CONTROL, CANID_MCU_E2E_DIAG, _finish, decode_mcu_frame
)


class ProtocolReferenceTest(unittest.TestCase):
    def test_control_feedback_decode(self):
        data = bytearray((0x8A, 0x02, 0x50, 0xFB, 0, 75, 9, 0))
        decoded = decode_mcu_frame(CANID_MCU_CONTROL, _finish(CANID_MCU_CONTROL, data))
        self.assertAlmostEqual(decoded["steer_deg"], 6.5)
        self.assertAlmostEqual(decoded["accel_ms2"], -1.2)
        self.assertEqual(decoded["brake_pct"], 75)
        self.assertEqual(decoded["seq"], 9)

    def test_data_id_masquerade_is_rejected(self):
        data = bytearray((0, 0, 0, 0, 0, 0, 1, 0))
        protected = _finish(CANID_MCU_CONTROL, data)
        with self.assertRaises(ValueError):
            decode_mcu_frame(CANID_MCU_E2E_DIAG, protected)

    def test_e2e_diag_decode(self):
        data = bytearray((2, 3, 7, 1, 0, 0, 2, 0))
        decoded = decode_mcu_frame(CANID_MCU_E2E_DIAG,
                                   _finish(CANID_MCU_E2E_DIAG, data))
        self.assertEqual(decoded["primary_seq_errors"], 2)
        self.assertEqual(decoded["protocol_version"], 2)
        self.assertFalse(decoded["test_build"])


if __name__ == "__main__":
    unittest.main()
