#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = ["pyserial==3.5"]
# ///
"""実機の状態をシリアルから読み続ける。

NVS 経由では動作中の状態が取れない。フラッシュ読み出しは esptool のリセットを
伴うので、何度読んでも「起動直後」しか観測できないため。

USB JTAG/serial はハブ経由だと数秒で切れることがあるが、ファームは動き続けて
いる。切れたら開き直して読み続ける。

usage: watch_serial.py <port> [seconds]
"""

import os
import sys
import time

from serial_connection import open_serial_without_reset


def watch(port, seconds, on_line=None):
    """切断を挟みながら指定秒数ぶん行を読む。読めた行を返す。"""
    deadline = time.time() + seconds
    lines = []
    buffer = b""

    while time.time() < deadline:
        if not os.path.exists(port):
            time.sleep(0.2)
            continue
        try:
            connection = open_serial_without_reset(port)
        except OSError:
            time.sleep(0.2)
            continue

        try:
            while time.time() < deadline:
                chunk = connection.read(4096)
                if not chunk:
                    continue
                buffer += chunk
                while b"\n" in buffer:
                    raw, buffer = buffer.split(b"\n", 1)
                    text = raw.decode("utf-8", "replace").strip()
                    # 切断のたびに行の途中から再開するので、先頭が揃った行だけ採る
                    if text.startswith("phase="):
                        lines.append(text)
                        print(f"{time.strftime('%H:%M:%S')} {text}", flush=True)
                        has_callback = on_line is not None
                        if has_callback:
                            should_continue = on_line(connection, text)
                            should_stop = should_continue is False
                            if should_stop:
                                return lines
        except OSError:
            pass
        finally:
            try:
                connection.close()
            except OSError:
                pass
        time.sleep(0.2)

    return lines


def main():
    if len(sys.argv) < 2:
        print(__doc__, file=sys.stderr)
        return 2
    port = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    lines = watch(port, seconds)
    print(f"=== {len(lines)} 行 ===")
    return 0 if lines else 1


if __name__ == "__main__":
    sys.exit(main())
