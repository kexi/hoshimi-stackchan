"""USBシリアル接続がCoreS3へ不要な制御線を送らないことを保証する。"""

import sys
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

import serial_connection  # noqa: E402


class SerialConnectionTest(unittest.TestCase):
    def test_disables_control_lines_before_opening(self):
        """CoreS3のポートを開く間もDTR/RTSへioctlを発行しない。"""

        class RecordingConnection:
            def __init__(self):
                self.port = None
                self.baudrate = None
                self.timeout = None
                self.dtr = True
                self.rts = True
                self.opened = False
                self.control_line_updates = []

            def _update_dtr_state(self):
                self.control_line_updates.append("dtr")

            def _update_rts_state(self):
                self.control_line_updates.append("rts")

            def open(self):
                self.opened = True
                test_case.assertEqual(self.port, "/dev/fake")
                test_case.assertEqual(self.baudrate, 115200)
                test_case.assertEqual(self.timeout, 0.25)
                test_case.assertFalse(self.dtr)
                test_case.assertFalse(self.rts)
                test_case.assertEqual(self.control_line_updates, [])

        test_case = self
        connection = RecordingConnection()
        with mock.patch.object(
            serial_connection,
            "PassiveControlLineSerial",
            return_value=connection,
        ):
            opened = serial_connection.open_serial_without_reset(
                "/dev/fake", timeout=0.25
            )

        self.assertIs(opened, connection)
        self.assertTrue(connection.opened)


if __name__ == "__main__":
    unittest.main()
