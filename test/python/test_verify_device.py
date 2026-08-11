"""実機検証器の判定とターゲット巡回を保証する。"""

import math
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

import verify_device  # noqa: E402


def tracking_sample(target=0, settled=1):
    """安定追尾ログの最小サンプルを作る。"""
    return {"phase": "Tracking", "target": target, "settled": settled}


def reached_sample(target=1):
    """両軸が目標へ届いた評価用サンプルを作る。"""
    return {
        "phase": "Tracking",
        "target": target,
        "settled": 1,
        "yaw": 100,
        "pitch": 450,
        "cmdYaw": 100,
        "cmdPitch": 450,
        "neckAbs": 20.0,
        "neckAlt": 0.0,
        "tgtAz": 20.0,
        "alt": 0.0,
        "eph": "fixed" if target == 0 else "jpl-db",
        "clampY": 0,
        "clampP": 0,
    }


class TargetAdvanceControllerTest(unittest.TestCase):
    def test_requires_consecutive_stable_samples(self):
        """不安定な周期を挟んだ値を安定4回として数えない。"""
        controller = verify_device.TargetAdvanceController()
        for _ in range(3):
            self.assertIsNone(controller.observe(tracking_sample()))
        self.assertIsNone(controller.observe(tracking_sample(settled=0)))
        for _ in range(3):
            self.assertIsNone(controller.observe(tracking_sample()))
        self.assertEqual(controller.observe(tracking_sample()), verify_device.ADVANCE)

    def test_retries_advance_when_target_does_not_change(self):
        """最初のシリアル指令が失われても4サンプル後に再送する。"""
        controller = verify_device.TargetAdvanceController()
        actions = [controller.observe(tracking_sample()) for _ in range(8)]
        self.assertEqual(
            actions,
            [None, None, None, verify_device.ADVANCE] * 2,
        )

    def test_stops_only_after_all_known_targets(self):
        """未知番号を完了数へ含めず、8ターゲット後だけ停止する。"""
        controller = verify_device.TargetAdvanceController()
        for _ in range(4):
            self.assertIsNone(controller.observe(tracking_sample(target=-1)))

        for target in range(len(verify_device.TARGET_NAMES)):
            action = None
            for _ in range(4):
                action = controller.observe(tracking_sample(target=target))
            expected = (
                verify_device.STOP
                if target == len(verify_device.TARGET_NAMES) - 1
                else verify_device.ADVANCE
            )
            self.assertEqual(action, expected)


class DeviceLifecycleMonitorTest(unittest.TestCase):
    def test_stops_when_uptime_moves_backwards(self):
        """実機が再起動した周期を見逃さず、リセット理由を保持する。"""
        monitor = verify_device.DeviceLifecycleMonitor()
        self.assertFalse(monitor.observe({"up": 12_000, "rst": 1}))
        self.assertFalse(monitor.observe({"up": 12_500, "rst": 1}))
        self.assertTrue(monitor.observe({"up": 500, "rst": 9}))
        self.assertEqual(monitor.restart_reason, "brownout")

    def test_ignores_logs_without_uptime(self):
        """旧ファームのログを再起動と誤判定しない。"""
        monitor = verify_device.DeviceLifecycleMonitor()
        self.assertFalse(monitor.observe({"phase": "Tracking"}))
        self.assertIsNone(monitor.restart_reason)


class TargetEvaluationTest(unittest.TestCase):
    def test_accepts_reached_target_with_expected_ephemeris(self):
        """正しい暦で両軸が到達したサンプルを合格にする。"""
        reached, lines = verify_device.evaluate_target(reached_sample())
        self.assertTrue(reached, lines)

    def test_rejects_clamped_axis(self):
        """サーボ追従差が小さくても可動域クランプは未到達とする。"""
        sample = reached_sample()
        sample["clampY"] = 1
        reached, _ = verify_device.evaluate_target(sample)
        self.assertFalse(reached)

    def test_rejects_non_finite_direction(self):
        """NaNを許容差内として扱わない。"""
        sample = reached_sample()
        sample["neckAbs"] = math.nan
        reached, _ = verify_device.evaluate_target(sample)
        self.assertFalse(reached)


if __name__ == "__main__":
    unittest.main()
