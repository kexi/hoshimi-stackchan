#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""Stack-chanの最新センサースナップショットをWi-Fi経由で取得する。"""

import json
import os
import sys
import urllib.error

from select_target import base_url, discover_host, request_json


def read_sensors(host: str) -> dict:
    """指定ホストのセンサー値と校正進捗を読む。"""
    root = base_url(host)
    snapshot = request_json(f"{root}/api/sensors")
    snapshot["control"] = request_json(f"{root}/api/status")
    return snapshot


def main() -> int:
    if len(sys.argv) not in (1, 2):
        print("usage: read_sensors.py [host]", file=sys.stderr)
        return 2

    try:
        explicit_host = sys.argv[1].strip() if len(sys.argv) == 2 else ""
        configured_host = explicit_host or os.environ.get("STACKCHAN_HOST", "").strip()
        host = configured_host or discover_host()
        snapshot = read_sensors(host)
    except (OSError, RuntimeError, ValueError, urllib.error.URLError) as error:
        print(f"sensors: {error}", file=sys.stderr)
        return 1

    print(json.dumps(snapshot, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
