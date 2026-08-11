#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = ["pyserial==3.5"]
# ///
"""実機の動作をシリアル経由で検証する。

キャリブレーション・時刻・方位・ターゲット切り替えを順に確かめ、
何が足りないかを具体的に出す。

Why not NVS: フラッシュ読み出しは esptool のリセットを伴うので、何度読んでも
「起動直後」の値しか取れない。スワイプで切り替わった結果が観測できない。

usage: verify_device.py <port> [seconds]
"""

import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from watch_serial import watch  # noqa: E402

TARGET_NAMES = ["North", "Sun", "Moon", "Mercury", "Venus", "Mars", "Jupiter", "Saturn"]

# 首の絶対方位と目標方位がこの範囲に収まっていれば「指せている」とみなす。
# サーボの静定精度が ±8.7 度なので、それを超える値にする。
POINTING_TOLERANCE_DEGREES = 10.0
STABLE_SAMPLES_BEFORE_ADVANCE = 4
ADVANCE = "advance"
STOP = "stop"

RESET_REASON_NAMES = {
    0: "unknown",
    1: "power-on",
    2: "external-pin",
    3: "software",
    4: "panic",
    5: "interrupt-watchdog",
    6: "task-watchdog",
    7: "watchdog",
    8: "deep-sleep",
    9: "brownout",
    10: "SDIO",
    11: "USB",
    12: "JTAG",
    13: "efuse",
    14: "power-glitch",
    15: "CPU-lockup",
}


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


def circular_error_degrees(actual, target):
    """方位の差を -180..180 度へ畳む。"""
    return (actual - target + 540.0) % 360.0 - 180.0


def is_stable_tracking(sample):
    """実角度を合否判定できる安定した追尾サンプルか。"""
    is_tracking = sample.get("phase") == "Tracking"
    is_settled = bool(sample.get("settled"))
    return is_tracking and is_settled


class TargetAdvanceController:
    """連続安定値を待ち、切り替え指令の再送要否を決める。"""

    def __init__(self):
        self.current_target = None
        self.stable_count = 0
        self.completed_targets = set()

    def observe(self, sample):
        """必要ならADVANCE/STOPを返し、それ以外はNoneを返す。"""
        target = int(sample.get("target", -1))
        is_known_target = 0 <= target < len(TARGET_NAMES)
        if not is_known_target:
            self.stable_count = 0
            return None

        target_changed = target != self.current_target
        if target_changed:
            self.current_target = target
            self.stable_count = 0

        if not is_stable_tracking(sample):
            self.stable_count = 0
            return None

        self.stable_count += 1
        has_enough_samples = self.stable_count >= STABLE_SAMPLES_BEFORE_ADVANCE
        if not has_enough_samples:
            return None

        # 指令を受け取る前の同一ターゲットが続いた場合に備え、次の4サンプルを
        # 数え直して再送する。毎サンプル送るとシリアル入力を埋めるので採らない。
        self.stable_count = 0
        already_completed = target in self.completed_targets
        if already_completed:
            return ADVANCE

        self.completed_targets.add(target)
        all_targets_completed = len(self.completed_targets) == len(TARGET_NAMES)
        return STOP if all_targets_completed else ADVANCE


class DeviceLifecycleMonitor:
    """起動後時刻の巻き戻りから検証中の再起動を検出する。"""

    def __init__(self):
        self.last_uptime = None
        self.restart_reason = None

    def observe(self, sample):
        """再起動を初めて検出した周期だけTrueを返す。"""
        uptime = sample.get("up")
        has_uptime = isinstance(uptime, (int, float))
        if not has_uptime:
            return False

        restarted = self.last_uptime is not None and uptime < self.last_uptime
        self.last_uptime = uptime
        if not restarted:
            return False

        reason = int(sample.get("rst", -1))
        self.restart_reason = RESET_REASON_NAMES.get(reason, f"code-{reason}")
        return True


def evaluate_target(sample):
    """1ターゲットの両軸追尾と天体方向への到達を評価する。"""
    required_fields = (
        "yaw",
        "pitch",
        "cmdYaw",
        "cmdPitch",
        "neckAbs",
        "neckAlt",
        "tgtAz",
        "alt",
        "eph",
        "clampY",
        "clampP",
    )
    missing = [field for field in required_fields if field not in sample]
    if missing:
        return False, [f"× 検証ログ不足: {', '.join(missing)}"]

    servo_yaw_error = (sample["yaw"] - sample["cmdYaw"]) / 10.0
    servo_pitch_error = (sample["pitch"] - sample["cmdPitch"]) / 10.0
    azimuth_error = circular_error_degrees(sample["neckAbs"], sample["tgtAz"])
    altitude_error = sample["neckAlt"] - sample["alt"]

    target = int(sample.get("target", -1))
    expected_ephemeris = "fixed" if target == 0 else "jpl-db"
    uses_expected_ephemeris = sample["eph"] == expected_ephemeris

    yaw_servo_reached = abs(servo_yaw_error) <= POINTING_TOLERANCE_DEGREES
    pitch_servo_reached = abs(servo_pitch_error) <= POINTING_TOLERANCE_DEGREES
    yaw_clamped = bool(sample["clampY"])
    pitch_clamped = bool(sample["clampP"])
    azimuth_reached = not yaw_clamped and abs(azimuth_error) <= POINTING_TOLERANCE_DEGREES
    altitude_reached = not pitch_clamped and abs(altitude_error) <= POINTING_TOLERANCE_DEGREES

    lines = [
        f"{'○' if uses_expected_ephemeris else '×'} 天体暦 {sample['eph']}",
        f"{'○' if yaw_servo_reached else '×'} yaw追従差 {servo_yaw_error:+.1f}°",
        f"{'○' if pitch_servo_reached else '×'} pitch追従差 {servo_pitch_error:+.1f}°",
    ]
    if yaw_clamped:
        lines.append(f"△ 方位は首の可動域外（残差 {azimuth_error:+.1f}°）")
    else:
        lines.append(f"{'○' if azimuth_reached else '×'} 方位差 {azimuth_error:+.1f}°")
    if pitch_clamped:
        lines.append(f"△ 高度は首の可動域外（残差 {altitude_error:+.1f}°）")
    else:
        lines.append(f"{'○' if altitude_reached else '×'} 高度差 {altitude_error:+.1f}°")

    reached = (
        uses_expected_ephemeris
        and yaw_servo_reached
        and pitch_servo_reached
        and azimuth_reached
        and altitude_reached
    )
    return reached, lines


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
        error = circular_error_degrees(neck, azimuth)
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
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0

    print("=== 全8ターゲット実機検証 ===")
    print(f"シリアル指令で順番に切り替え、最大 {seconds:.0f} 秒観測します...")
    print()

    advance_controller = TargetAdvanceController()
    lifecycle_monitor = DeviceLifecycleMonitor()

    def advance_after_settle(connection, line):
        """各ターゲットの安定値を4回観測してから次へ進める。"""
        sample = parse(line)
        restarted = lifecycle_monitor.observe(sample)
        if restarted:
            return False
        action = advance_controller.observe(sample)
        should_stop = action == STOP
        if should_stop:
            return False
        should_advance = action == ADVANCE
        if should_advance:
            connection.write(b">")
            connection.flush()
        return True

    samples = [parse(line) for line in watch(port, seconds, advance_after_settle)]
    if not samples:
        print("デバイスが応答しません")
        print()
        print("確認すること:")
        print("  1. USB が Mac に直挿しされているか (ハブ経由は不安定)")
        print("  2. just set-time で書き込み済みか")
        return 1

    if lifecycle_monitor.restart_reason is not None:
        print()
        print(
            "× 検証中にCoreS3が再起動しました "
            f"(理由: {lifecycle_monitor.restart_reason})"
        )
        print("  電源とサーボ負荷を確認するまで検証を停止します")
        return 1

    latest_stable_by_target = {}
    for sample in samples:
        if is_stable_tracking(sample):
            latest_stable_by_target[int(sample.get("target", -1))] = sample

    print()
    print("--- ターゲット別結果 ---")
    all_reached = True
    for target, name in enumerate(TARGET_NAMES):
        print(f"{name}:")
        sample = latest_stable_by_target.get(target)
        if sample is None:
            print("  × 安定した追尾値を観測できませんでした")
            all_reached = False
            continue
        reached, result_lines = evaluate_target(sample)
        for result_line in result_lines:
            print(f"  {result_line}")
        all_reached = all_reached and reached

    print()
    if all_reached:
        print("○ 全8ターゲットが両軸とも許容差内です")
        return 0
    print("× 全8ターゲット到達の実機確認は未完了です")
    return 1


if __name__ == "__main__":
    sys.exit(main())
