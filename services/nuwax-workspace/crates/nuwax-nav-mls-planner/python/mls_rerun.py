#!/usr/bin/env python3
"""Consume MLS debug-event NDJSON through the optional Rerun 0.32 SDK."""

from __future__ import annotations

import argparse
import importlib
import json
import math
import sys
from pathlib import Path
from typing import Any, BinaryIO, Iterable, Iterator


MAX_ENTITY_LENGTH = 1_024
MAX_POINTS = 100_000
MAX_STRIPS = 100_000
MAX_POINTS_PER_STRIP = 100_000
MAX_STRIP_POINTS = 100_000
MAX_LINE_BYTES = 8 * 1024 * 1024
MAX_TOTAL_BYTES = 64 * 1024 * 1024
MAX_RECORDS = 100_000


def _number(value: Any, label: str) -> int | float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a JSON number")
    if not math.isfinite(value):
        raise ValueError(f"{label} must be finite")
    return value


def _bounded_list(value: Any, label: str, limit: int) -> list[Any]:
    if not isinstance(value, list):
        raise ValueError(f"{label} must be a JSON array")
    if len(value) > limit:
        raise ValueError(f"{label} exceeds the {limit} item limit")
    return value


def _point(value: Any, label: str) -> list[int | float]:
    coordinates = _bounded_list(value, label, 3)
    if len(coordinates) != 3:
        raise ValueError(f"{label} must contain exactly 3 coordinates")
    return [_number(coordinate, f"{label}[{index}]") for index, coordinate in enumerate(coordinates)]


def _points(value: Any, label: str, limit: int = MAX_POINTS) -> list[list[int | float]]:
    points = _bounded_list(value, label, limit)
    return [_point(point, f"{label}[{index}]") for index, point in enumerate(points)]


def _colors(value: Any, label: str, item_count: int) -> list[list[int | float]] | None:
    if value is None:
        return None
    colors = _bounded_list(value, label, MAX_POINTS)
    if len(colors) not in (1, item_count):
        raise ValueError(f"{label} must contain 1 or {item_count} colors")
    validated = []
    for color_index, color in enumerate(colors):
        channels = _bounded_list(color, f"{label}[{color_index}]", 4)
        if len(channels) not in (3, 4):
            raise ValueError(f"{label}[{color_index}] must contain 3 or 4 channels")
        validated.append(
            [
                _number(channel, f"{label}[{color_index}][{channel_index}]")
                for channel_index, channel in enumerate(channels)
            ]
        )
    return validated


def _radii(value: Any, label: str, item_count: int) -> list[int | float] | None:
    if value is None:
        return None
    radii = _bounded_list(value, label, MAX_POINTS)
    if len(radii) not in (1, item_count):
        raise ValueError(f"{label} must contain 1 or {item_count} radii")
    validated = [_number(radius, f"{label}[{index}]") for index, radius in enumerate(radii)]
    if any(radius < 0 for radius in validated):
        raise ValueError(f"{label} must not contain negative values")
    return validated


def _entity(event: dict[str, Any]) -> str:
    entity = event.get("entity")
    if not isinstance(entity, str) or not entity or len(entity) > MAX_ENTITY_LENGTH:
        raise ValueError(f"event entity must be a non-empty string of at most {MAX_ENTITY_LENGTH} characters")
    return entity


def _sdk() -> Any:
    try:
        return importlib.import_module("rerun")
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "rerun-sdk 0.32 is required for MLS .rrd export; install the optional visualization runtime"
        ) from exc


def _records(lines: Iterable[str]) -> list[dict[str, Any]]:
    records = []
    total_bytes = 0
    for line_number, line in enumerate(lines, 1):
        line_bytes = len(line.encode("utf-8"))
        if line_bytes > MAX_LINE_BYTES:
            raise ValueError(f"line {line_number} exceeds the {MAX_LINE_BYTES}-byte limit")
        total_bytes += line_bytes
        if total_bytes > MAX_TOTAL_BYTES:
            raise ValueError(f"input exceeds the {MAX_TOTAL_BYTES}-byte cumulative limit")
        if not line.strip():
            continue
        if len(records) >= MAX_RECORDS:
            raise ValueError(f"input exceeds the {MAX_RECORDS}-record limit")
        try:
            record = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid NDJSON at line {line_number}: {exc}") from exc
        if not isinstance(record, dict):
            raise ValueError(f"line {line_number} must contain a JSON object")
        timestamp = record.get("timestamp")
        try:
            _number(timestamp, f"line {line_number} timestamp")
        except ValueError as exc:
            raise ValueError(str(exc)) from exc
        event = record.get("event")
        if not isinstance(event, dict):
            raise ValueError(f"line {line_number} has no event object")
        records.append(record)
    return records


def _raw_lines(stream: BinaryIO) -> Iterator[str]:
    total_bytes = 0
    line_number = 0
    while True:
        remaining_total = MAX_TOTAL_BYTES - total_bytes
        read_limit = min(MAX_LINE_BYTES + 1, remaining_total + 1)
        raw_line = stream.readline(read_limit)
        if not raw_line:
            return

        line_number += 1
        if len(raw_line) > MAX_LINE_BYTES:
            raise ValueError(f"line {line_number} exceeds the {MAX_LINE_BYTES}-byte limit")
        if len(raw_line) > remaining_total:
            raise ValueError(f"input exceeds the {MAX_TOTAL_BYTES}-byte cumulative limit")
        total_bytes += len(raw_line)

        try:
            yield raw_line.decode("utf-8", errors="strict")
        except UnicodeDecodeError as exc:
            raise ValueError(f"invalid UTF-8 at line {line_number}: {exc}") from exc


def _validate_event(event: dict[str, Any]) -> None:
    kind = event.get("kind")
    if kind == "points3d":
        positions = _points(event.get("positions"), "points3d positions")
        _colors(event.get("colors"), "points3d colors", len(positions))
        _radii(event.get("radii"), "points3d radii", len(positions))
        _entity(event)
        return
    if kind == "line_strips3d":
        raw_strips = _bounded_list(event.get("strips"), "line_strips3d strips", MAX_STRIPS)
        total_points = 0
        for strip_index, raw_strip in enumerate(raw_strips):
            strip = _points(raw_strip, f"line_strips3d strips[{strip_index}]", MAX_POINTS_PER_STRIP)
            total_points += len(strip)
            if total_points > MAX_STRIP_POINTS:
                raise ValueError(f"line_strips3d strips exceed the {MAX_STRIP_POINTS} total point limit")
        _colors(event.get("colors"), "line_strips3d colors", len(raw_strips))
        _radii(event.get("radii"), "line_strips3d radii", len(raw_strips))
        _entity(event)
        return
    if kind == "metrics":
        values = event.get("values")
        if not isinstance(values, dict):
            raise ValueError("metrics event requires a values object")
        for name, value in values.items():
            if not isinstance(name, str) or not name or len(name) > MAX_ENTITY_LENGTH:
                raise ValueError("metric names must be non-empty bounded strings")
            _number(value, f"metric {name!r}")
        return
    raise ValueError(f"unsupported MLS Rerun event kind: {kind!r}")


def _log(rr: Any, event: dict[str, Any]) -> None:
    kind = event.get("kind")
    if kind == "points3d":
        positions = _points(event.get("positions"), "points3d positions")
        colors = _colors(event.get("colors"), "points3d colors", len(positions))
        radii = _radii(event.get("radii"), "points3d radii", len(positions))
        rr.log(
            _entity(event),
            rr.Points3D(
                positions,
                colors=colors,
                radii=radii,
            ),
        )
        return
    if kind == "line_strips3d":
        raw_strips = _bounded_list(event.get("strips"), "line_strips3d strips", MAX_STRIPS)
        strips = []
        total_points = 0
        for strip_index, raw_strip in enumerate(raw_strips):
            strip = _points(
                raw_strip,
                f"line_strips3d strips[{strip_index}]",
                MAX_POINTS_PER_STRIP,
            )
            total_points += len(strip)
            if total_points > MAX_STRIP_POINTS:
                raise ValueError(f"line_strips3d strips exceed the {MAX_STRIP_POINTS} total point limit")
            strips.append(strip)
        colors = _colors(event.get("colors"), "line_strips3d colors", len(strips))
        radii = _radii(event.get("radii"), "line_strips3d radii", len(strips))
        rr.log(
            _entity(event),
            rr.LineStrips3D(
                strips,
                colors=colors,
                radii=radii,
            ),
        )
        return
    if kind == "metrics":
        values = event.get("values")
        if not isinstance(values, dict):
            raise ValueError("metrics event requires a values object")
        for name, value in values.items():
            if not isinstance(name, str) or not name or len(name) > MAX_ENTITY_LENGTH:
                raise ValueError("metric names must be non-empty bounded strings")
            rr.log(f"metrics/{name}", rr.Scalars(_number(value, f"metric {name!r}")))
        return
    raise ValueError(f"unsupported MLS Rerun event kind: {kind!r}")


def run(
    lines: Iterable[str],
    *,
    application_id: str,
    recording_id: str,
    output: Path | None,
    spawn: bool,
    rr: Any | None = None,
) -> int:
    records = _records(lines)
    for record in records:
        _validate_event(record["event"])

    sdk = rr if rr is not None else _sdk()
    sdk.init(application_id, recording_id=recording_id)
    if output is not None:
        sdk.save(str(output))
    if spawn:
        sdk.spawn()

    count = 0
    for record in records:
        sdk.set_time("ts", timestamp=record["timestamp"])
        _log(sdk, record["event"])
        count += 1
    return count


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--application-id", required=True)
    parser.add_argument("--recording-id", required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--spawn", action="store_true")
    args = parser.parse_args()
    try:
        count = run(
            _raw_lines(sys.stdin.buffer),
            application_id=args.application_id,
            recording_id=args.recording_id,
            output=args.output,
            spawn=args.spawn,
        )
    except (RuntimeError, ValueError, KeyError, TypeError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    print(json.dumps({"events": count, "output": str(args.output) if args.output else None}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
