#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""JPL Horizonsの地心・見かけ赤道座標をファームウェア用DBへ変換する。"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import io
import math
import re
import struct
import subprocess
import urllib.parse
import zlib
from dataclasses import dataclass
from pathlib import Path


API_URL = "https://ssd.jpl.nasa.gov/api/horizons.api"
DEFAULT_START = dt.datetime(2025, 1, 1, tzinfo=dt.timezone.utc)
DEFAULT_STOP = dt.datetime(2031, 1, 1, tzinfo=dt.timezone.utc)
VALIDATION_START = dt.datetime(2025, 1, 2, 1, 17, tzinfo=dt.timezone.utc)


@dataclass(frozen=True)
class BodyConfig:
    cpp_name: str
    horizons_command: str
    step_seconds: int


@dataclass(frozen=True)
class Sample:
    unix_seconds: int
    right_ascension_degrees: float
    declination_degrees: float
    distance_au: float


@dataclass(frozen=True)
class Series:
    body: BodyConfig
    samples: tuple[Sample, ...]
    source: str
    api_version: str


BODIES = (
    BodyConfig("Sun", "10", 12 * 60 * 60),
    BodyConfig("Moon", "301", 6 * 60 * 60),
    BodyConfig("Mercury", "199", 12 * 60 * 60),
    BodyConfig("Venus", "299", 12 * 60 * 60),
    BodyConfig("Mars", "499", 12 * 60 * 60),
    BodyConfig("Jupiter", "599", 12 * 60 * 60),
    BodyConfig("Saturn", "699", 12 * 60 * 60),
)


def horizons_time(value: dt.datetime) -> str:
    return value.strftime("%Y-%m-%d %H:%M")


def request_text(body: BodyConfig, start: dt.datetime, stop: dt.datetime, step: str) -> str:
    parameters = {
        "format": "text",
        "COMMAND": f"'{body.horizons_command}'",
        "OBJ_DATA": "'NO'",
        "MAKE_EPHEM": "'YES'",
        "EPHEM_TYPE": "'OBSERVER'",
        "CENTER": "'500@399'",
        "START_TIME": f"'{horizons_time(start)}'",
        "STOP_TIME": f"'{horizons_time(stop)}'",
        "STEP_SIZE": f"'{step}'",
        "TIME_TYPE": "'UT'",
        "QUANTITIES": "'2,20'",
        "REF_SYSTEM": "'ICRF'",
        "RANGE_UNITS": "'AU'",
        "SUPPRESS_RANGE_RATE": "'YES'",
        "CSV_FORMAT": "'YES'",
        "CAL_FORMAT": "'JD'",
        "ANG_FORMAT": "'DEG'",
        "APPARENT": "'AIRLESS'",
        "EXTRA_PREC": "'YES'",
    }
    url = f"{API_URL}?{urllib.parse.urlencode(parameters)}"
    completed = subprocess.run(
        [
            "curl",
            "--fail",
            "--silent",
            "--show-error",
            "--max-time",
            "180",
            "--user-agent",
            "compass-stackchan/1",
            url,
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return completed.stdout


def julian_date_to_unix_seconds(julian_date: float) -> int:
    return round((julian_date - 2440587.5) * 86400.0)


def parse_response(text: str) -> tuple[tuple[Sample, ...], str, str]:
    start_marker = text.find("$$SOE")
    stop_marker = text.find("$$EOE")
    has_ephemeris_rows = start_marker >= 0 and stop_marker > start_marker
    if not has_ephemeris_rows:
        raise RuntimeError(f"Horizons応答に暦データがありません:\n{text[:1000]}")

    rows_text = text[start_marker + len("$$SOE") : stop_marker]
    samples: list[Sample] = []
    for row in csv.reader(io.StringIO(rows_text)):
        values = [value.strip() for value in row if value.strip()]
        has_required_columns = len(values) >= 4
        if not has_required_columns:
            continue
        samples.append(
            Sample(
                julian_date_to_unix_seconds(float(values[0])),
                float(values[1]),
                float(values[2]),
                float(values[3]),
            )
        )

    has_samples = bool(samples)
    if not has_samples:
        raise RuntimeError("Horizons応答を解析できませんでした")

    source_match = re.search(r"Target body name:.*?\{source: ([^}]+)\}", text)
    api_match = re.search(r"API VERSION:\s*([^\s]+)", text)
    source = source_match.group(1).strip() if source_match else "unknown"
    api_version = api_match.group(1).strip() if api_match else "unknown"
    return tuple(samples), source, api_version


def validate_uniform_grid(series: Series, expected_start: int, expected_stop: int) -> None:
    has_expected_boundaries = (
        series.samples[0].unix_seconds == expected_start
        and series.samples[-1].unix_seconds == expected_stop
    )
    if not has_expected_boundaries:
        raise RuntimeError(
            f"{series.body.cpp_name}: DB境界が不正です "
            f"({series.samples[0].unix_seconds}, {series.samples[-1].unix_seconds})"
        )

    for previous, current in zip(series.samples, series.samples[1:]):
        has_expected_step = current.unix_seconds - previous.unix_seconds == series.body.step_seconds
        if not has_expected_step:
            raise RuntimeError(
                f"{series.body.cpp_name}: {previous.unix_seconds}以後の間隔が不正です"
            )


def fetch_series(body: BodyConfig, start: dt.datetime, stop: dt.datetime) -> Series:
    step_hours = body.step_seconds // 3600
    text = request_text(body, start, stop, f"{step_hours} h")
    samples, source, api_version = parse_response(text)
    series = Series(body, samples, source, api_version)
    validate_uniform_grid(series, int(start.timestamp()), int(stop.timestamp()))
    return series


def fetch_validation_series(body: BodyConfig, start: dt.datetime, stop: dt.datetime) -> Series:
    text = request_text(body, start, stop, "37 d")
    samples, source, api_version = parse_response(text)
    return Series(body, samples, source, api_version)


def fetch_interpolation_checks(
    body: BodyConfig, start: dt.datetime, stop: dt.datetime
) -> tuple[Sample, ...]:
    samples: list[Sample] = []
    for numerator in (1, 2, 3):
        offset_seconds = body.step_seconds * numerator // 4
        check_start = start + dt.timedelta(seconds=offset_seconds)
        check_stop = stop - dt.timedelta(seconds=body.step_seconds - offset_seconds)
        step_hours = body.step_seconds // 3600
        text = request_text(body, check_start, check_stop, f"{step_hours} h")
        phase_samples, _, _ = parse_response(text)
        samples.extend(phase_samples)
    return tuple(samples)


def float32(value: float) -> float:
    return struct.unpack("<f", struct.pack("<f", value))[0]


def angular_difference(from_degrees: float, to_degrees: float) -> float:
    difference = (to_degrees - from_degrees) % 360.0
    is_long_way_round = difference > 180.0
    if is_long_way_round:
        difference -= 360.0
    return difference


def lagrange4(y0: float, y1: float, y2: float, y3: float, x: float) -> float:
    weight0 = -((x - 1.0) * (x - 2.0) * (x - 3.0)) / 6.0
    weight1 = (x * (x - 2.0) * (x - 3.0)) / 2.0
    weight2 = -(x * (x - 1.0) * (x - 3.0)) / 2.0
    weight3 = (x * (x - 1.0) * (x - 2.0)) / 6.0
    return weight0 * y0 + weight1 * y1 + weight2 * y2 + weight3 * y3


def interpolate(series: Series, unix_seconds: int) -> Sample:
    sample_position = (unix_seconds - series.samples[0].unix_seconds) / series.body.step_seconds
    lower_index = int(sample_position)
    start_index = max(0, lower_index - 1)
    extends_past_end = start_index + 3 >= len(series.samples)
    if extends_past_end:
        start_index = len(series.samples) - 4
    x = sample_position - start_index

    stored = tuple(
        Sample(
            sample.unix_seconds,
            float32(sample.right_ascension_degrees),
            float32(sample.declination_degrees),
            float32(sample.distance_au),
        )
        for sample in series.samples[start_index : start_index + 4]
    )
    reference_ra = stored[1].right_ascension_degrees
    unwrapped_ra = tuple(
        reference_ra + angular_difference(reference_ra, sample.right_ascension_degrees)
        for sample in stored
    )
    right_ascension = lagrange4(*unwrapped_ra, x) % 360.0
    declination = lagrange4(
        *(sample.declination_degrees for sample in stored), x
    )
    distance = lagrange4(*(sample.distance_au for sample in stored), x)
    return Sample(unix_seconds, right_ascension, declination, distance)


def angular_separation_degrees(actual: Sample, expected: Sample) -> float:
    actual_dec = math.radians(actual.declination_degrees)
    expected_dec = math.radians(expected.declination_degrees)
    delta_ra = angular_difference(
        expected.right_ascension_degrees, actual.right_ascension_degrees
    )
    delta_ra = math.radians(delta_ra)
    # atan2型の大円距離は、極小誤差でもacosの丸め落ちを起こさない。
    sin_half_dec = math.sin((actual_dec - expected_dec) / 2.0)
    sin_half_ra = math.sin(delta_ra / 2.0)
    haversine_value = (
        sin_half_dec * sin_half_dec
        + math.cos(actual_dec)
        * math.cos(expected_dec)
        * sin_half_ra
        * sin_half_ra
    )
    angle = 2.0 * math.asin(min(1.0, math.sqrt(haversine_value)))
    return math.degrees(angle)


def validate_interpolation(series: Series, checks: tuple[Sample, ...]) -> tuple[float, float]:
    max_angular_error = 0.0
    max_relative_distance_error = 0.0
    for expected in checks:
        actual = interpolate(series, expected.unix_seconds)
        max_angular_error = max(
            max_angular_error, angular_separation_degrees(actual, expected)
        )
        relative_distance_error = abs(actual.distance_au - expected.distance_au) / expected.distance_au
        max_relative_distance_error = max(
            max_relative_distance_error, relative_distance_error
        )

    angular_error_is_acceptable = max_angular_error <= 0.001
    distance_error_is_acceptable = max_relative_distance_error <= 2e-6
    is_acceptable = angular_error_is_acceptable and distance_error_is_acceptable
    if not is_acceptable:
        raise RuntimeError(
            f"{series.body.cpp_name}: 4点補間の誤差が上限超過 "
            f"(angle={max_angular_error:.9f} deg, "
            f"distance={max_relative_distance_error:.3e})"
        )
    return max_angular_error, max_relative_distance_error


def cpp_float(value: float) -> str:
    rendered = format(value, ".9g")
    has_decimal_marker = "." in rendered or "e" in rendered.lower()
    if not has_decimal_marker:
        rendered += ".0"
    return f"{rendered}F"


def database_crc32(series_collection: tuple[Series, ...]) -> int:
    checksum = 0
    for series in series_collection:
        checksum = zlib.crc32(
            struct.pack("<qI", series.samples[0].unix_seconds, series.body.step_seconds), checksum
        )
        for sample in series.samples:
            checksum = zlib.crc32(
                struct.pack(
                    "<fff",
                    sample.right_ascension_degrees,
                    sample.declination_degrees,
                    sample.distance_au,
                ),
                checksum,
            )
    return checksum & 0xFFFFFFFF


def render_database(series_collection: tuple[Series, ...]) -> str:
    sources = sorted({series.source for series in series_collection})
    api_versions = sorted({series.api_version for series in series_collection})
    source_label = (
        f"JPL Horizons API {','.join(api_versions)} / {','.join(sources)} / "
        "geocentric airless apparent true-equator/equinox-of-date UT"
    )
    start_unix = series_collection[0].samples[0].unix_seconds
    stop_unix = series_collection[0].samples[-1].unix_seconds
    checksum = database_crc32(series_collection)

    lines = [
        "// Generated by scripts/generate_ephemeris_db.py. Do not edit.",
        f'constexpr char kGeneratedSource[] = "{source_label}";',
        "constexpr EphemerisDatabaseInfo kGeneratedDatabaseInfo = {",
        f"    {start_unix}LL, {stop_unix}LL, kGeneratedSource, 0x{checksum:08X}U}};",
        "",
    ]
    for series in series_collection:
        array_name = f"k{series.body.cpp_name}Samples"
        lines.append(f"constexpr StoredSample {array_name}[] = {{")
        for sample in series.samples:
            lines.append(
                "    {"
                f"{cpp_float(sample.right_ascension_degrees)}, "
                f"{cpp_float(sample.declination_degrees)}, "
                f"{cpp_float(sample.distance_au)}"
                "},"
            )
        lines.extend(("};", ""))

    lines.append("constexpr StoredSeries kGeneratedSeries[] = {")
    for series in series_collection:
        array_name = f"k{series.body.cpp_name}Samples"
        lines.append(
            "    {"
            f"{series.samples[0].unix_seconds}LL, {series.body.step_seconds}U, "
            f"sizeof({array_name}) / sizeof({array_name}[0]), {array_name}"
            "},"
        )
    lines.extend(("};", ""))
    return "\n".join(lines)


def render_validation(series_collection: tuple[Series, ...]) -> str:
    lines = [
        "// Generated by scripts/generate_ephemeris_db.py from separate off-grid Horizons rows.",
        "constexpr JplDatabaseExpected kJplDatabaseExpected[] = {",
    ]
    for series in series_collection:
        for sample in series.samples:
            lines.append(
                "    {"
                f"astro::EphemerisBody::{series.body.cpp_name}, {sample.unix_seconds}LL, "
                f"{sample.right_ascension_degrees:.12f}, "
                f"{sample.declination_degrees:.12f}, {sample.distance_au:.15f}"
                "},"
            )
    lines.extend(("};", ""))
    return "\n".join(lines)


def write_generated(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--database-output",
        type=Path,
        default=Path("lib/astro_core/src/ephemeris_db_data.inc"),
    )
    parser.add_argument(
        "--validation-output",
        type=Path,
        default=Path("test/data/jpl_ephemeris_validation.inc"),
    )
    return parser.parse_args()


def main() -> None:
    arguments = parse_arguments()
    series_collection = tuple(fetch_series(body, DEFAULT_START, DEFAULT_STOP) for body in BODIES)
    interpolation_results: list[tuple[str, float, float, int]] = []
    for series in series_collection:
        checks = fetch_interpolation_checks(series.body, DEFAULT_START, DEFAULT_STOP)
        max_angle, max_distance = validate_interpolation(series, checks)
        interpolation_results.append(
            (series.body.cpp_name, max_angle, max_distance, len(checks))
        )
    validation_collection = tuple(
        fetch_validation_series(body, VALIDATION_START, DEFAULT_STOP) for body in BODIES
    )
    write_generated(arguments.database_output, render_database(series_collection))
    write_generated(arguments.validation_output, render_validation(validation_collection))

    total_samples = sum(len(series.samples) for series in series_collection)
    print(
        f"generated {total_samples} samples for {len(series_collection)} bodies "
        f"({DEFAULT_START.date()}..{DEFAULT_STOP.date()})"
    )
    for body_name, max_angle, max_distance, check_count in interpolation_results:
        print(
            f"validated {body_name}: {check_count} off-grid points, "
            f"max angle={max_angle:.9f} deg, max distance={max_distance:.3e}"
        )


if __name__ == "__main__":
    main()
