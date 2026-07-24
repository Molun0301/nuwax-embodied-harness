"""Optional Rerun SDK adapter for AprilTag detection results."""

from __future__ import annotations

import importlib
import math
from pathlib import Path
from types import ModuleType
from typing import Any, Mapping, Optional, Sequence


class RerunUnavailableError(RuntimeError):
    """The optional Rerun SDK cannot be used in this environment."""


def load_rerun_sdk() -> ModuleType:
    try:
        return importlib.import_module("rerun")
    except ImportError as exc:
        raise RerunUnavailableError(
            "Rerun SDK is not installed; install the optional 'rerun-sdk' package to enable marker recording"
        ) from exc


class RerunMarkerAdapter:
    """Own one Rerun RecordingStream and log structured marker geometry."""

    def __init__(
        self,
        application_id: str = "nuwax_mapping_markers",
        recording_id: Optional[str] = None,
        rerun_module: Optional[ModuleType] = None,
    ) -> None:
        if not isinstance(application_id, str) or not application_id.strip():
            raise ValueError("application_id must be non-empty")
        if recording_id is not None and (not isinstance(recording_id, str) or not recording_id.strip()):
            raise ValueError("recording_id must be a non-empty string when supplied")
        self._rerun = rerun_module if rerun_module is not None else load_rerun_sdk()
        recording_type = getattr(self._rerun, "RecordingStream", None)
        if recording_type is None:
            raise RerunUnavailableError("installed Rerun SDK does not provide RecordingStream")
        kwargs = {"recording_id": recording_id} if recording_id else {}
        self._recording = recording_type(application_id, **kwargs)
        self._closed = False

    def log_detections(self, detections: Sequence[Mapping[str, Any]], entity_path: str = "mapping/apriltags") -> None:
        if self._closed:
            raise RuntimeError("Rerun marker recording is closed")
        if not isinstance(entity_path, str) or not entity_path.strip():
            raise ValueError("entity_path must be non-empty")
        centers = []
        labels = []
        strips = []
        for detection in detections:
            if not isinstance(detection, Mapping):
                raise ValueError("each marker detection must be a mapping")
            marker_id = detection.get("id")
            if type(marker_id) is not int or marker_id < 0:
                raise ValueError("marker id must be a non-negative integer")
            center = _finite_point(detection.get("center"), "marker center")
            corners_value = detection.get("corners")
            if (
                not isinstance(corners_value, Sequence)
                or isinstance(corners_value, (str, bytes))
                or len(corners_value) != 4
            ):
                raise ValueError("marker geometry must contain one center and four 2D corners")
            corners = [_finite_point(point, "marker corner") for point in corners_value]
            centers.append(center)
            labels.append("tag %s" % marker_id)
            closed_strip = list(corners)
            closed_strip.append(list(closed_strip[0]))
            strips.append(closed_strip)
        self._recording.log(
            entity_path + "/centers",
            self._rerun.Points2D(centers, labels=labels),
        )
        self._recording.log(
            entity_path + "/corners",
            self._rerun.LineStrips2D(strips),
        )

    def save(self, path: object) -> None:
        if self._closed:
            raise RuntimeError("Rerun marker recording is closed")
        output = Path(path)
        if output.suffix.lower() != ".rrd":
            raise ValueError("Rerun recording path must use the .rrd extension")
        self._recording.save(str(output))

    def close(self) -> None:
        if self._closed:
            return
        disconnect = getattr(self._recording, "disconnect", None)
        if callable(disconnect):
            disconnect()
        self._closed = True

    def __enter__(self) -> "RerunMarkerAdapter":
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        self.close()


def _finite_point(value: object, field: str) -> list[float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 2:
        raise ValueError("marker geometry must contain one center and four 2D corners")
    if any(type(coordinate) is bool for coordinate in value):
        raise ValueError(f"{field} must contain finite coordinates")
    try:
        point = [float(value[0]), float(value[1])]
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{field} must contain finite coordinates") from exc
    if not all(math.isfinite(coordinate) for coordinate in point):
        raise ValueError(f"{field} must contain finite coordinates")
    return point
