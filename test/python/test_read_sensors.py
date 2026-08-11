"""センサースナップショットを正しいWi-Fi APIから取得することを保証する。"""

import sys
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

import read_sensors  # noqa: E402


class SensorRequestTest(unittest.TestCase):
    @mock.patch("read_sensors.request_json")
    def test_reads_sensor_snapshot_without_changing_device_state(self, request_json):
        """GETだけで診断値と校正進捗を取得し、実機へ書き込まない。"""
        request_json.side_effect = [
            {
                "magnetometer": {"core": {"x": 1.0, "y": 2.0, "z": 3.0}},
                "heading": {"valid": True, "bodyTrueDegrees": 123.0},
            },
            {"calibrationSamples": 48, "calibrationCoverage": 0.75},
        ]

        snapshot = read_sensors.read_sensors("stackchan-a1b2c3.local")

        self.assertTrue(snapshot["heading"]["valid"])
        self.assertEqual(snapshot["control"]["calibrationSamples"], 48)
        self.assertEqual(
            request_json.call_args_list,
            [
                mock.call("http://stackchan-a1b2c3.local/api/sensors"),
                mock.call("http://stackchan-a1b2c3.local/api/status"),
            ],
        )


if __name__ == "__main__":
    unittest.main()
