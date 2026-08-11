"""RTC設定プロトコルの形式と境界を保証する。"""

import sys
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

import set_device_time  # noqa: E402


class ClockCommandTest(unittest.TestCase):
    def test_builds_ascii_line_command(self):
        """有効なUnix秒をT始まり・改行終わりで送る。"""
        self.assertEqual(set_device_time.clock_command(1_786_442_400), b"T1786442400\n")

    def test_rejects_out_of_range_time(self):
        """未同期値や2100年以後をデバイスへ送らない。"""
        for invalid in (0, 1_699_999_999, 4_102_444_800):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    set_device_time.clock_command(invalid)

    @mock.patch.object(set_device_time.time, "time", return_value=1_786_442_400)
    @mock.patch.object(set_device_time.os.path, "exists", return_value=True)
    def test_close_error_does_not_hide_ack(self, _exists, _time):
        """ACK受信後のUSB切断でcloseが失敗しても、設定成功を保持する。"""

        class DisconnectingConnection:
            def reset_input_buffer(self):
                pass

            def write(self, _command):
                pass

            def flush(self):
                pass

            def readline(self):
                return b"clock=1786442400\n"

            def close(self):
                raise set_device_time.serial.SerialException("disconnected")

        with mock.patch.object(
            set_device_time,
            "open_serial_without_reset",
            return_value=DisconnectingConnection(),
        ):
            synced = set_device_time.set_device_time("/dev/fake", timeout_seconds=0.1)

        self.assertEqual(synced, 1_786_442_400)


if __name__ == "__main__":
    unittest.main()
