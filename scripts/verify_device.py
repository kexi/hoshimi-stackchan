#!/usr/bin/env python3
"""実機の動作を NVS 経由で検証する。

デバイスが復帰したらこれを走らせる。キャリブレーション・時刻・方位・
ターゲット切り替えの状態を順に確かめ、何が足りないかを具体的に出す。

USB CDC シリアルが列挙されない個体なので、状態はすべて NVS から読む。

usage: verify_device.py <port> <esptool.py path>
"""

import subprocess
import sys
import time

# read_nvs.py と同じパース処理を使う
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from read_nvs import NVS_OFFSET, NVS_SIZE, parse_entries  # noqa: E402

# 状態機械の局面。app::Phase と同じ順序。
PHASE_NAMES = [
    "InitHardware",
    "ConnectWifi",
    "SyncTime",
    "Calibrating",
    "Idle",
    "ToMeasurePose",
    "Measuring",
    "Pointing",
    "Tracking",
    "Error",
]

TARGET_NAMES = ["North", "Sun", "Moon", "Mercury", "Venus", "Mars", "Jupiter", "Saturn"]


def read_state(port, esptool_path):
    """NVS を吸い出してキーと値の辞書にする。"""
    import tempfile

    with tempfile.NamedTemporaryFile(suffix=".bin") as dump:
        result = subprocess.run(
            [
                sys.executable,
                esptool_path,
                "--port",
                port,
                "read-flash",
                hex(NVS_OFFSET),
                hex(NVS_SIZE),
                dump.name,
            ],
            capture_output=True,
        )
        if result.returncode != 0:
            return None
        data = open(dump.name, "rb").read()

    latest = {}
    for _, key, _, value in parse_entries(data):
        latest[key] = value
    return latest


def describe(state):
    """読めた状態を人が読める形にする。"""
    if not state:
        return ["デバイスが応答しません (フラッシュを読めない)"]

    lines = []

    if "lvOk" not in state and "valid" not in state:
        lines.append("× キャリブレーション未実施 — 本体を水平に一回転させる")
    elif state.get("lvOk", state.get("valid", 0)):
        lines.append("○ キャリブレーション済み")
    else:
        lines.append("× キャリブレーションが採用されていない")

    if state.get("timeOk"):
        lines.append("○ 時刻あり — 天体を計算できる")
    else:
        lines.append("× 時刻なし — just set-time で入れる (真北のみ指せる)")

    if state.get("hdgOk"):
        heading = state.get("hdg", 0) / 10.0
        lines.append(f"○ 方位が確定 ({heading:.1f} 度)")
    else:
        lines.append("× 方位が未確定")

    phase = state.get("phase")
    if phase is not None and 0 <= phase < len(PHASE_NAMES):
        lines.append(f"  局面: {PHASE_NAMES[phase]}")

    target = state.get("target")
    if target is not None and 0 <= target < len(TARGET_NAMES):
        lines.append(f"  ターゲット: {TARGET_NAMES[target]}")

    # 首が指している方位と、目標の方位を突き合わせる
    if "neckAbs" in state and "tgtAz" in state:
        neck = state["neckAbs"] / 10.0
        target_azimuth = state["tgtAz"] / 10.0
        error = (neck - target_azimuth + 540.0) % 360.0 - 180.0
        mark = "○" if abs(error) < 10.0 else "×"
        lines.append(f"{mark} 首 {neck:.1f} 度 / 目標 {target_azimuth:.1f} 度 (差 {error:+.1f} 度)")

    return lines


def main():
    if len(sys.argv) < 3:
        print(__doc__, file=sys.stderr)
        return 2

    port, esptool_path = sys.argv[1], sys.argv[2]

    print("=== 実機の状態 ===")
    state = read_state(port, esptool_path)
    for line in describe(state):
        print(line)

    if not state:
        print()
        print("復帰の手順:")
        print("  1. USB を Mac に直挿しする (ハブ経由は不安定)")
        print("  2. リセットボタン (左側面) を 2 秒以上長押し")
        print("  3. just set-time で書き込む")
        return 1

    # ターゲットが切り替わるかを見る。スワイプしてもらって差分を取る。
    print()
    print("ターゲットを切り替えてください (スワイプ)。20 秒待ちます...")
    before = state.get("target")
    time.sleep(20)
    after_state = read_state(port, esptool_path)
    after = after_state.get("target") if after_state else None

    if after is None:
        print("× 読み取りに失敗しました")
        return 1
    if before != after:
        name = TARGET_NAMES[after] if 0 <= after < len(TARGET_NAMES) else "?"
        print(f"○ ターゲットが切り替わりました → {name}")
        for line in describe(after_state):
            print(line)
        return 0

    print("× ターゲットが変わりませんでした")
    print("  スワイプが検出されていないか、状態が更新されていません")
    return 1


if __name__ == "__main__":
    sys.exit(main())
