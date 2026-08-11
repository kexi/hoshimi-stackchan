"""PlatformIOの補助イメージがNVSを上書きしない配置を保証する。"""

import csv
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PARTITIONS = ROOT / "firmware" / "partitions.csv"


def partition_rows() -> dict[str, tuple[int, int]]:
    """パーティション名ごとのoffsetとsizeを数値で返す。"""
    rows = {}
    with PARTITIONS.open(encoding="utf-8", newline="") as source:
        for row in csv.reader(line for line in source if not line.startswith("#")):
            name = row[0].strip()
            rows[name] = (int(row[3].strip(), 0), int(row[4].strip(), 0))
    return rows


class PartitionLayoutTest(unittest.TestCase):
    def test_boot_app_region_does_not_overlap_nvs(self):
        """0xE000へ書かれるboot_app0.binからNVS全体を保護する。"""
        rows = partition_rows()
        nvs_offset, nvs_size = rows["nvs"]
        otadata_offset, otadata_size = rows["otadata"]

        self.assertLessEqual(nvs_offset + nvs_size, 0xE000)
        self.assertEqual((otadata_offset, otadata_size), (0xE000, 0x2000))


if __name__ == "__main__":
    unittest.main()
