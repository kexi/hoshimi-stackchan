#!/usr/bin/env python3
"""実機の動作をシリアル経由で検証する。

キャリブレーション・時刻・方位・ターゲット切り替えを順に確かめ、
何が足りないかを具体的に出す。

Why not NVS: フラッシュ読み出しは esptool のリセットを伴うので、何度読んでも
「起動直後」の値しか取れない。スワイプで切り替わった結果が観測できない。

usage: verify_device.py <port>
"""

import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from watch_serial import watch  # noqa: E402

TARGET_NAMES = ["North", "Sun", "Moon", "Mercury", "Venus", "Mars", "Jupiter", "Saturn"]

# 首の絶対方位と目標方位がこの範囲に収まっていれば「指せている」とみなす。
# サーボの静定精度が ±8.7 度なので、それを超える値にする。
POINTING_TOLERANCE_DEGREES = 10.0


def parse(line):
    """`key=value` を並べた 1 行を辞書にする。"""
    fields = {}
    for token in line.split():
        if "=" not in token:
            continue
        key, _, value = token.partition("=")
        try:
            fields[key] = float(value)
        except ValueError:
            fields[key] = value
    return fields


def target_name(value):
    index = int(value)
    return TARGET_NAMES[index] if 0 <= index < len(TARGET_NAMES) else f"?({index})"


def describe(sample):
    """1 サンプルを人が読める形にする。"""
    lines = []

    if sample.get("timeOk"):
        lines.append("○ 時刻あり — 天体を計算できる")
    else:
        lines.append("× 時刻なし — just set-time で入れる (真北のみ指せる)")

    if sample.get("hdgOk"):
        lines.append(f"○ 方位が確定 ({sample.get('hdg', 0.0):.1f} 度)")
    else:
        lines.append(f"× 方位が未確定 (棄却理由 {int(sample.get('rej', -1))})")

    lines.append(f"  局面: {sample.get('phase')}")
    lines.append(f"  ターゲット: {target_name(sample.get('target', -1))}")

    neck = sample.get("neckAbs")
    azimuth = sample.get("tgtAz")
    if neck is not None and azimuth is not None:
        error = (neck - azimuth + 540.0) % 360.0 - 180.0
        reached = abs(error) < POINTING_TOLERANCE_DEGREES
        mark = "○" if reached else "×"
        note = "" if reached else "  ← 首の可動域 (±128 度) を超えている"
        lines.append(f"{mark} 首 {neck:.1f} 度 / 目標 {azimuth:.1f} 度 (差 {error:+.1f} 度){note}")
    return lines


def main():
    if len(sys.argv) < 2:
        print(__doc__, file=sys.stderr)
        return 2
    port = sys.argv[1]

    print("=== 実機の状態 ===")
    print("スワイプでターゲットを切り替えてください。60 秒観測します...")
    print()

    samples = [parse(line) for line in watch(port, 60.0)]
    if not samples:
        print("デバイスが応答しません")
        print()
        print("確認すること:")
        print("  1. USB が Mac に直挿しされているか (ハブ経由は不安定)")
        print("  2. just set-time で書き込み済みか")
        return 1

    print()
    print("--- 最新の状態 ---")
    for line in describe(samples[-1]):
        print(line)

    # ターゲットが切り替わったか。スワイプが効いているかの判定。
    seen = []
    for sample in samples:
        name = target_name(sample.get("target", -1))
        if not seen or seen[-1] != name:
            seen.append(name)

    print()
    if len(seen) > 1:
        print(f"○ ターゲットが切り替わりました → {seen[-1]}")
        print(f"  経路: {' → '.join(seen)}")
        return 0

    touches = int(samples[-1].get("touchN", 0))
    print("× ターゲットが変わりませんでした")
    if touches == 0:
        print("  タッチが 1 度も検出されていません (touchN=0)")
    else:
        print(f"  タッチは {touches} 回検出されています。切り替えロジックを見る必要があります")
    return 1


if __name__ == "__main__":
    sys.exit(main())
