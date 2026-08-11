"""Wi-Fiターゲット指定の名前解決とmDNS出力解析を保証する。"""

import sys
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

import select_target  # noqa: E402


class TargetIndexTest(unittest.TestCase):
    def test_accepts_english_japanese_and_number(self):
        """同じ天体を英語、日本語、番号のいずれでも指定できる。"""
        self.assertEqual(select_target.target_index("moon"), 2)
        self.assertEqual(select_target.target_index("月"), 2)
        self.assertEqual(select_target.target_index("2"), 2)

    def test_rejects_unknown_target(self):
        """未知名や範囲外番号を誤った対象へ変換しない。"""
        with self.assertRaises(ValueError):
            select_target.target_index("pluto")
        with self.assertRaises(ValueError):
            select_target.target_index("8")


class MdnsOutputTest(unittest.TestCase):
    def test_parses_dns_sd_browse_and_resolve(self):
        """Bonjourの標準出力から専用サービスのホストだけを得る。"""
        browse = """
Timestamp     A/R Flags if Domain Service Type Instance Name
22:15:00.000  Add     2  14 local. _stackchan._tcp. stackchan-a1b2c3
22:15:00.001  Add     2  14 local. _http._tcp. unrelated
"""
        resolved = (
            "stackchan-a1b2c3._stackchan._tcp.local. can be reached at "
            "stackchan-a1b2c3.local.:80"
        )
        self.assertEqual(select_target.parse_dns_sd_instances(browse), ["stackchan-a1b2c3"])
        self.assertEqual(select_target.parse_dns_sd_host(resolved), "stackchan-a1b2c3.local")

    def test_parses_avahi_resolved_record(self):
        """Avahiの解決済みレコードからホスト名を得る。"""
        output = (
            "=;en0;IPv4;stackchan-a1b2c3;_stackchan._tcp;local;"
            "stackchan-a1b2c3.local;192.0.2.1;80;path=/api/target\n"
        )
        self.assertEqual(select_target.parse_avahi_hosts(output), ["stackchan-a1b2c3.local"])


class TargetSelectionTest(unittest.TestCase):
    @mock.patch.object(select_target.time, "sleep", return_value=None)
    @mock.patch.object(select_target, "request_json")
    def test_waits_until_target_is_fixed(self, request_json, _sleep):
        """対象が偶然一致しても、自動巡回停止まで成功扱いしない。"""
        request_json.side_effect = [
            {"accepted": True, "target": 7},
            {"target": 7, "autoCycle": True},
            {"target": 7, "autoCycle": False},
        ]

        status = select_target.select_target("stackchan-test.local", 7)

        self.assertFalse(status["autoCycle"])
        self.assertEqual(request_json.call_count, 3)


if __name__ == "__main__":
    unittest.main()
