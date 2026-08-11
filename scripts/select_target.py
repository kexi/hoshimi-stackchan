#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""mDNSでStack-chanを見つけ、Wi-Fi経由で対象を選ぶ。"""

import json
import os
import re
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request


TARGET_NAMES = ["North", "Sun", "Moon", "Mercury", "Venus", "Mars", "Jupiter", "Saturn"]
TARGET_ALIASES = {
    "north": 0,
    "北": 0,
    "sun": 1,
    "太陽": 1,
    "moon": 2,
    "月": 2,
    "mercury": 3,
    "水星": 3,
    "venus": 4,
    "金星": 4,
    "mars": 5,
    "火星": 5,
    "jupiter": 6,
    "木星": 6,
    "saturn": 7,
    "土星": 7,
}
SERVICE_TYPE = "_stackchan._tcp"


def target_index(value: str) -> int:
    """英語名、日本語名、0〜7を対象番号へ変換する。"""
    normalized = value.strip().lower()
    if normalized in TARGET_ALIASES:
        return TARGET_ALIASES[normalized]

    try:
        index = int(normalized)
    except ValueError as error:
        raise ValueError(f"対象が不明です: {value}") from error
    is_known_target = 0 <= index < len(TARGET_NAMES)
    if not is_known_target:
        raise ValueError(f"対象番号は0〜7で指定してください: {value}")
    return index


def command_output_until_timeout(command: list[str], timeout: float) -> str:
    """常駐型のmDNSコマンドから、制限時間までに得た出力を返す。"""
    try:
        completed = subprocess.run(command, capture_output=True, timeout=timeout, check=False)
        output = completed.stdout
    except subprocess.TimeoutExpired as error:
        output = error.stdout or b""
    if isinstance(output, bytes):
        return output.decode("utf-8", "replace")
    return output


def parse_dns_sd_instances(output: str) -> list[str]:
    """dns-sdのbrowse出力から追加されたインスタンス名を得る。"""
    instances = []
    for line in output.splitlines():
        fields = line.split(None, 6)
        has_service_fields = len(fields) == 7
        is_addition = has_service_fields and fields[1] == "Add"
        is_stackchan = has_service_fields and fields[5].rstrip(".") == SERVICE_TYPE
        if is_addition and is_stackchan:
            instances.append(fields[6].strip())
    return sorted(set(instances))


def parse_dns_sd_host(output: str) -> str | None:
    """dns-sdのresolve出力からホスト名を得る。"""
    match = re.search(r"can be reached at\s+([^\s:]+):\d+", output)
    if match is None:
        return None
    return match.group(1).rstrip(".")


def discover_with_dns_sd() -> list[str]:
    """macOSのBonjourで専用サービスを探す。"""
    if shutil.which("dns-sd") is None:
        return []

    browse_output = command_output_until_timeout(
        ["dns-sd", "-B", SERVICE_TYPE, "local."], 3.0
    )
    hosts = []
    for instance in parse_dns_sd_instances(browse_output):
        resolve_output = command_output_until_timeout(
            ["dns-sd", "-L", instance, SERVICE_TYPE, "local."], 2.0
        )
        host = parse_dns_sd_host(resolve_output)
        if host is not None:
            hosts.append(host)
    return sorted(set(hosts))


def parse_avahi_hosts(output: str) -> list[str]:
    """avahi-browseのparsable出力から解決済みホスト名を得る。"""
    hosts = []
    for line in output.splitlines():
        fields = line.split(";")
        is_resolved = len(fields) >= 9 and fields[0] == "="
        is_stackchan = is_resolved and fields[4] == SERVICE_TYPE
        if is_resolved and is_stackchan:
            hosts.append(fields[6].rstrip("."))
    return sorted(set(hosts))


def discover_with_avahi() -> list[str]:
    """LinuxのAvahiで専用サービスを探す。"""
    if shutil.which("avahi-browse") is None:
        return []
    completed = subprocess.run(
        ["avahi-browse", "--resolve", "--terminate", "--parsable", SERVICE_TYPE],
        capture_output=True,
        text=True,
        timeout=5.0,
        check=False,
    )
    return parse_avahi_hosts(completed.stdout)


def discover_host() -> str:
    """利用できるmDNSクライアントで一台のStack-chanを見つける。"""
    hosts = discover_with_dns_sd() or discover_with_avahi()
    if not hosts:
        raise RuntimeError("Stack-chanが見つかりません。ホスト名を明示してください")
    has_multiple_devices = len(hosts) > 1
    if has_multiple_devices:
        joined = ", ".join(hosts)
        raise RuntimeError(f"複数台見つかりました。ホスト名を明示してください: {joined}")
    return hosts[0]


def base_url(host: str) -> str:
    """ホスト指定を制御APIのURLへ変換する。"""
    stripped = host.strip().rstrip("/")
    has_scheme = stripped.startswith("http://") or stripped.startswith("https://")
    return stripped if has_scheme else f"http://{stripped}"


def request_json(url: str, method: str = "GET") -> dict:
    """制御APIへ要求し、JSON応答を読む。"""
    request = urllib.request.Request(url, method=method)
    with urllib.request.urlopen(request, timeout=3.0) as response:
        return json.load(response)


def select_target(host: str, index: int) -> dict:
    """対象を指示し、状態へ反映されたことまで確認する。"""
    root = base_url(host)
    query = urllib.parse.urlencode({"target": index})
    request_json(f"{root}/api/target?{query}", method="POST")

    deadline = time.monotonic() + 5.0
    latest = {}
    while time.monotonic() < deadline:
        latest = request_json(f"{root}/api/status")
        target_was_applied = latest.get("target") == index
        auto_cycle_was_disabled = latest.get("autoCycle") is False
        if target_was_applied and auto_cycle_was_disabled:
            return latest
        time.sleep(0.1)
    raise TimeoutError(f"対象変更が反映されませんでした: {latest}")


def main() -> int:
    if len(sys.argv) not in (2, 3):
        print("usage: select_target.py <target> [host]", file=sys.stderr)
        return 2

    try:
        index = target_index(sys.argv[1])
        explicit_host = sys.argv[2].strip() if len(sys.argv) == 3 else ""
        configured_host = explicit_host or os.environ.get("STACKCHAN_HOST", "").strip()
        host = configured_host or discover_host()
        status = select_target(host, index)
    except (OSError, RuntimeError, TimeoutError, ValueError, urllib.error.URLError) as error:
        print(f"target: {error}", file=sys.stderr)
        return 1

    print(
        f"{status.get('host', host)}: {TARGET_NAMES[index]} ({index}) "
        f"phase={status.get('phase', '?')} autoCycle={status.get('autoCycle', '?')}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
