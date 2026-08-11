#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""校正の試行値をWi-Fi経由で実行中のStack-chanへ適用する。"""

import math
import os
import sys
import urllib.error
import urllib.parse

from select_target import base_url, discover_host, request_json


def parse_maximum_tilt_degrees(value: str) -> float:
    """許容角を有限な1〜30度へ制限する。"""
    try:
        degrees = float(value)
    except ValueError as error:
        raise ValueError(f"傾き許容角は数値で指定してください: {value}") from error

    value_is_finite = math.isfinite(degrees)
    value_is_in_range = 1.0 <= degrees <= 30.0
    if not value_is_finite or not value_is_in_range:
        raise ValueError(f"傾き許容角は1〜30度で指定してください: {value}")
    return degrees


def configure_calibration(host: str, maximum_tilt_degrees: float) -> dict:
    """試行値をRAMへ適用し、校正セッションを再開始する。"""
    root = base_url(host)
    query = urllib.parse.urlencode({"maxTiltDegrees": maximum_tilt_degrees})
    response = request_json(f"{root}/api/calibration?{query}", method="POST")
    applied = response.get("maxTiltDegrees")
    value_was_applied = isinstance(applied, (int, float)) and math.isclose(
        float(applied), maximum_tilt_degrees, abs_tol=0.05
    )
    if not value_was_applied:
        raise RuntimeError(f"校正設定が反映されませんでした: {response}")
    return response


def main() -> int:
    if len(sys.argv) not in (2, 3):
        print("usage: configure_calibration.py <max-tilt-degrees> [host]", file=sys.stderr)
        return 2

    try:
        maximum_tilt_degrees = parse_maximum_tilt_degrees(sys.argv[1])
        explicit_host = sys.argv[2].strip() if len(sys.argv) == 3 else ""
        configured_host = explicit_host or os.environ.get("STACKCHAN_HOST", "").strip()
        host = configured_host or discover_host()
        response = configure_calibration(host, maximum_tilt_degrees)
    except (OSError, RuntimeError, ValueError, urllib.error.URLError) as error:
        print(f"calibrate: {error}", file=sys.stderr)
        return 1

    print(
        f"{host}: maxTiltDegrees={response['maxTiltDegrees']:.1f} "
        "calibration restarted (volatile)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
