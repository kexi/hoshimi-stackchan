#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = ["pyserial==3.5"]
# ///
"""CoreS3へ現在のUnix時刻を送り、RTC設定完了応答を待つ。"""

from __future__ import annotations

import datetime as dt
import os
import sys
import time

import serial

from serial_connection import open_serial_without_reset


MIN_VALID_UNIX_SECONDS = 1_700_000_000
MAX_VALID_UNIX_SECONDS = 4_102_444_800


def clock_command(unix_seconds: int) -> bytes:
    """ファームウェアの時刻設定プロトコルを組み立てる。"""
    is_plausible = MIN_VALID_UNIX_SECONDS <= unix_seconds < MAX_VALID_UNIX_SECONDS
    if not is_plausible:
        raise ValueError(f"時刻が有効範囲外です: {unix_seconds}")
    return f"T{unix_seconds}\n".encode("ascii")


def set_device_time(port: str, timeout_seconds: float = 40.0) -> int:
    """現在時刻を送信し、同じ秒のackを受け取ったらその値を返す。"""
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        if not os.path.exists(port):
            time.sleep(0.2)
            continue

        connection = None
        try:
            connection = open_serial_without_reset(port)
            connection.reset_input_buffer()
            unix_seconds = int(time.time())
            connection.write(clock_command(unix_seconds))
            connection.flush()

            response_deadline = min(deadline, time.monotonic() + 2.0)
            while time.monotonic() < response_deadline:
                response = connection.readline().decode("utf-8", "replace").strip()
                if response == f"clock={unix_seconds}":
                    return unix_seconds
                if response == "clock=invalid":
                    raise RuntimeError("CoreS3が時刻を拒否しました")
        except (OSError, serial.SerialException):
            time.sleep(0.2)
        finally:
            if connection is not None:
                try:
                    connection.close()
                except (OSError, serial.SerialException):
                    pass

    raise TimeoutError(f"CoreS3からRTC設定応答がありません: {port}")


def main() -> int:
    if len(sys.argv) not in (2, 3):
        print("usage: set_device_time.py <port> [timeout_seconds]", file=sys.stderr)
        return 2

    port = sys.argv[1]
    timeout_seconds = float(sys.argv[2]) if len(sys.argv) == 3 else 40.0
    try:
        unix_seconds = set_device_time(port, timeout_seconds)
    except (RuntimeError, TimeoutError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 1

    local_time = dt.datetime.fromtimestamp(unix_seconds).astimezone()
    print(f"RTCを {local_time:%Y-%m-%d %H:%M:%S %Z} に同期しました")
    return 0


if __name__ == "__main__":
    sys.exit(main())
