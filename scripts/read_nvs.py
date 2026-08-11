#!/usr/bin/env python3
"""実機の NVS パーティションをフラッシュから読み出して表示する。

CoreS3 の USB CDC シリアルが列挙されない個体でも、計測結果を回収できるようにする
ためのもの。ファーム側が Preferences (NVS) に書いた値を、esptool でフラッシュごと
吸い出してパースする。

usage: read_nvs.py <port> <esptool.py path> [namespace-filter]
"""

import struct
import subprocess
import sys
import tempfile

# partitions.csv の nvs エントリと合わせること
NVS_OFFSET = 0x9000
NVS_SIZE = 0x6000

PAGE_SIZE = 4096
ENTRY_SIZE = 32
HEADER_SIZE = 64

TYPES = {
    0x01: "u8",
    0x11: "i8",
    0x02: "u16",
    0x12: "i16",
    0x04: "u32",
    0x14: "i32",
    0x08: "u64",
    0x18: "i64",
    0x21: "str",
    0x42: "blob",
}

SCALAR_FORMATS = {
    "u8": ("<B", 1),
    "i8": ("<b", 1),
    "u16": ("<H", 2),
    "i16": ("<h", 2),
    "u32": ("<I", 4),
    "i32": ("<i", 4),
    "u64": ("<Q", 8),
    "i64": ("<q", 8),
}


def read_flash(port, esptool_path, destination):
    # --before no-reset / --after no-reset を付けないと、esptool が読み出しの前後で
    # デバイスをリセットする。そうすると毎回「起動直後」の NVS しか読めず、
    # 動いている最中の状態が取れない (実際に inPhase が常に 468ms で張り付いた)。
    subprocess.run(
        [
            sys.executable,
            esptool_path,
            "--port",
            port,
            "--before",
            "no-reset",
            "--after",
            "no-reset",
            "read-flash",
            hex(NVS_OFFSET),
            hex(NVS_SIZE),
            destination,
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def parse_entries(data):
    """NVS のページを走査してスカラー値のエントリを返す。

    後から書かれたものが後ろに出るので、呼び出し側は最後の値を採用すればよい。
    """
    entries = []
    for page_start in range(0, len(data), PAGE_SIZE):
        page = data[page_start : page_start + PAGE_SIZE]
        if len(page) < HEADER_SIZE:
            continue
        # 未使用ページ (state が全 1) は飛ばす
        if struct.unpack("<I", page[0:4])[0] == 0xFFFFFFFF:
            continue

        body = page[HEADER_SIZE:]
        for offset in range(0, len(body) - ENTRY_SIZE + 1, ENTRY_SIZE):
            entry = body[offset : offset + ENTRY_SIZE]
            namespace_index, type_code = entry[0], entry[1]
            if namespace_index == 0xFF or type_code == 0xFF:
                continue

            type_name = TYPES.get(type_code)
            if type_name not in SCALAR_FORMATS:
                continue

            key_bytes = entry[8:24].split(b"\x00")[0]
            try:
                key = key_bytes.decode("ascii")
            except UnicodeDecodeError:
                continue
            if not key.isprintable() or not key:
                continue

            fmt, size = SCALAR_FORMATS[type_name]
            value = struct.unpack(fmt, entry[24 : 24 + size])[0]
            entries.append((namespace_index, key, type_name, value))
    return entries


def main():
    if len(sys.argv) < 3:
        print(__doc__, file=sys.stderr)
        return 2

    port = sys.argv[1]
    esptool_path = sys.argv[2]
    key_filter = sys.argv[3] if len(sys.argv) > 3 else None

    with tempfile.NamedTemporaryFile(suffix=".bin") as dump:
        try:
            read_flash(port, esptool_path, dump.name)
        except subprocess.CalledProcessError:
            print(f"フラッシュを読めませんでした: {port}", file=sys.stderr)
            return 1
        data = open(dump.name, "rb").read()

    # 同じキーが複数回書かれている場合は最後の値が現在値
    latest = {}
    for namespace_index, key, type_name, value in parse_entries(data):
        latest[(namespace_index, key)] = (type_name, value)

    rows = sorted(latest.items(), key=lambda item: (item[0][0], item[0][1]))
    printed = 0
    for (namespace_index, key), (type_name, value) in rows:
        if key_filter and key_filter not in key:
            continue
        print(f"ns={namespace_index:<3} {key:<16} {type_name:<4} {value}")
        printed += 1

    if printed == 0:
        print("(該当するエントリがありません)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
