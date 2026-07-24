"""Rerun 0.32 adapter for ray-tracing comparison responses."""

from __future__ import annotations

import importlib
import math
from collections.abc import Mapping, Sequence
from numbers import Integral, Real
from typing import Any


class RerunUnavailableError(RuntimeError):
    """The optional Rerun SDK is absent or incompatible."""


def log_comparison(response: dict[str, Any], *, rr: Any | None = None, root: str = "raytrace") -> None:
    """Log comparison events using Rerun 0.32 archetypes.

    Passing ``rr`` supports dependency injection in tests. Without it, the
    optional SDK is imported dynamically and missing SDKs fail closed.
    """

    if not isinstance(response, Mapping) or response.get("mode") != "comparison":
        raise ValueError("a comparison response is required")
    events = _validate_response(response)
    if rr is None:
        try:
            rr = importlib.import_module("rerun")
        except ImportError as error:
            raise RerunUnavailableError("rerun-sdk 0.32 is required") from error
    required = ("log", "Points3D", "Arrows3D", "Transform3D", "LineStrips3D", "Quaternion")
    missing = [name for name in required if not hasattr(rr, name)]
    if missing:
        raise RerunUnavailableError(f"rerun SDK is missing: {', '.join(missing)}")

    for index, event in enumerate(events):
        event_root = f"{root}/events/{index}"
        rotation = event["robot_rotation_xyzw"]
        rr.log(
            f"{event_root}/robot",
            rr.Transform3D(
                translation=event["robot_position"],
                quaternion=rr.Quaternion(xyzw=rotation),
            ),
        )
        rr.log(f"{event_root}/sensor", rr.Points3D(event["sensor_points"]))
        rr.log(
            f"{event_root}/trajectory",
            rr.LineStrips3D([event["trajectory"]]),
        )
        for variant in event["variants"]:
            path = f"{event_root}/variants/{variant['name']}"
            color = variant["color_rgb"]
            rr.log(path, rr.Points3D(variant["world_points"], colors=color))
            normals = variant["normals"]
            if normals:
                rr.log(
                    f"{path}/normals",
                    rr.Arrows3D(
                        origins=[normal["origin"] for normal in normals],
                        vectors=[normal["vector"] for normal in normals],
                        colors=color,
                    ),
                )


def _validate_response(response: Any) -> list[Mapping[str, Any]]:
    if not isinstance(response, Mapping):
        raise ValueError("comparison response must be an object")
    outputs = response.get("outputs")
    if not isinstance(outputs, list):
        raise ValueError("comparison response outputs must be a list")
    for index, event in enumerate(outputs):
        _validate_event(event, f"outputs[{index}]")
    return outputs


def _validate_event(event: Any, path: str) -> None:
    if not isinstance(event, Mapping):
        raise ValueError(f"{path} must be an object")
    _finite_number(event.get("timestamp"), f"{path}.timestamp")
    frame_count = event.get("frame_count")
    if isinstance(frame_count, bool) or not isinstance(frame_count, Integral) or frame_count < 0:
        raise ValueError(f"{path}.frame_count must be a non-negative integer")
    _vector(event.get("robot_position"), 3, f"{path}.robot_position")
    _vector(event.get("robot_rotation_xyzw"), 4, f"{path}.robot_rotation_xyzw")
    _vector_list(event.get("sensor_points"), f"{path}.sensor_points")
    _vector_list(event.get("trajectory"), f"{path}.trajectory")

    variants = event.get("variants")
    if not isinstance(variants, list):
        raise ValueError(f"{path}.variants must be a list")
    for variant_index, variant in enumerate(variants):
        variant_path = f"{path}.variants[{variant_index}]"
        if not isinstance(variant, Mapping):
            raise ValueError(f"{variant_path} must be an object")
        if not isinstance(variant.get("name"), str) or not variant["name"]:
            raise ValueError(f"{variant_path}.name must be a non-empty string")
        color = variant.get("color_rgb")
        if (
            not isinstance(color, Sequence)
            or isinstance(color, (str, bytes))
            or len(color) != 3
            or any(
                isinstance(value, bool) or not isinstance(value, Integral) or not 0 <= value <= 255 for value in color
            )
        ):
            raise ValueError(f"{variant_path}.color_rgb must contain three byte values")
        _vector_list(variant.get("world_points"), f"{variant_path}.world_points")
        normals = variant.get("normals")
        if not isinstance(normals, list):
            raise ValueError(f"{variant_path}.normals must be a list")
        for normal_index, normal in enumerate(normals):
            normal_path = f"{variant_path}.normals[{normal_index}]"
            if not isinstance(normal, Mapping):
                raise ValueError(f"{normal_path} must be an object")
            _vector(normal.get("origin"), 3, f"{normal_path}.origin")
            _vector(normal.get("vector"), 3, f"{normal_path}.vector")


def _vector_list(value: Any, path: str) -> None:
    if not isinstance(value, list):
        raise ValueError(f"{path} must be a list")
    for index, item in enumerate(value):
        _vector(item, 3, f"{path}[{index}]")


def _vector(value: Any, size: int, path: str) -> None:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != size:
        raise ValueError(f"{path} must contain exactly {size} numbers")
    for index, item in enumerate(value):
        _finite_number(item, f"{path}[{index}]")


def _finite_number(value: Any, path: str) -> None:
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{path} must be a finite number")
