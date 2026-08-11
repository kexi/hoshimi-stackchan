"""校正の試行値を安全な範囲でWi-Fi APIへ渡すことを保証する。"""

import sys
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

import configure_calibration  # noqa: E402


class MaximumTiltTest(unittest.TestCase):
    def test_accepts_finite_value_in_safe_range(self):
        """境界を含む1〜30度を受け付ける。"""
        self.assertEqual(configure_calibration.parse_maximum_tilt_degrees("1"), 1.0)
        self.assertEqual(configure_calibration.parse_maximum_tilt_degrees("10.5"), 10.5)
        self.assertEqual(configure_calibration.parse_maximum_tilt_degrees("30"), 30.0)

    def test_rejects_non_finite_or_out_of_range_value(self):
        """NaNや安全範囲外を実機へ送らない。"""
        for value in ("nan", "inf", "0", "31", "degrees"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                configure_calibration.parse_maximum_tilt_degrees(value)


class CalibrationRequestTest(unittest.TestCase):
    @mock.patch("configure_calibration.request_json")
    def test_posts_runtime_value_and_checks_response(self, request_json):
        """指定値を校正APIへPOSTし、反映値まで検証する。"""
        request_json.return_value = {
            "accepted": True,
            "maxTiltDegrees": 10.0,
            "samples": 0,
        }

        response = configure_calibration.configure_calibration(
            "stackchan-a1b2c3.local", 10.0
        )

        self.assertTrue(response["accepted"])
        request_json.assert_called_once_with(
            "http://stackchan-a1b2c3.local/api/calibration?maxTiltDegrees=10.0",
            method="POST",
        )


if __name__ == "__main__":
    unittest.main()
