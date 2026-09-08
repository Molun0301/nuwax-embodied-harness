from __future__ import annotations

from dataclasses import dataclass
from datetime import timezone
import itertools
import math

from contracts.image import CameraInfo, ImageFrame
from contracts.perception import BoundingBox2D, Detection2D, Detection3D, Track, TrackState
from services.perception.base import TrackerBackend, TrackerBackendSpec
from services.perception.postprocess import _valid_detection
from typing import Dict, List, Optional, Set, Tuple


@dataclass(frozen=True)
class TrackLifecyclePolicy:
    """二维轨迹生命周期策略。"""

    iou_match_threshold: float = 0.3
    tentative_hits_required: int = 2
    lost_ttl_frames: int = 2


@dataclass
class _TrackRecord:
    """内存中的活动轨迹。"""

    track_id: str
    camera_id: str
    label: str
    bbox: Optional[BoundingBox2D]
    score: float
    hits: int = 1
    misses: int = 0
    state: TrackState = TrackState.TENTATIVE
    attributes: Optional[Dict[str, object]] = None

    def to_contract(self) -> Track:
        return Track(
            track_id=self.track_id,
            label=self.label,
            state=self.state,
            score=self.score,
            bbox=self.bbox,
            attributes={**dict(self.attributes or {}), "camera_id": self.camera_id},
        )


def _bbox_iou(left: Optional[BoundingBox2D], right: Optional[BoundingBox2D]) -> float:
    if left is None or right is None:
        return 0.0
    left_x1 = left.x_px
    left_y1 = left.y_px
    left_x2 = left.x_px + left.width_px
    left_y2 = left.y_px + left.height_px
    right_x1 = right.x_px
    right_y1 = right.y_px
    right_x2 = right.x_px + right.width_px
    right_y2 = right.y_px + right.height_px

    inter_x1 = max(left_x1, right_x1)
    inter_y1 = max(left_y1, right_y1)
    inter_x2 = min(left_x2, right_x2)
    inter_y2 = min(left_y2, right_y2)
    inter_width = max(0.0, inter_x2 - inter_x1)
    inter_height = max(0.0, inter_y2 - inter_y1)
    intersection = inter_width * inter_height
    if intersection <= 0.0:
        return 0.0
    left_area = left.width_px * left.height_px
    right_area = right.width_px * right.height_px
    union = left_area + right_area - intersection
    return intersection / union if math.isfinite(union) and union > 0.0 else 0.0


def _accept_source_frame(last_frames: Dict[str, tuple], image_frame: ImageFrame, records: Dict[str, _TrackRecord]) -> bool:
    """同相机只推进新的采集帧；图像几何改变后不能沿用旧像素轨迹。"""

    timestamp = image_frame.timestamp
    if timestamp.tzinfo is None:
        timestamp = timestamp.replace(tzinfo=timezone.utc)
    frame_key = (timestamp, image_frame.frame_id, image_frame.width_px, image_frame.height_px)
    previous = last_frames.get(image_frame.camera_id)
    if previous is not None:
        if timestamp < previous[0]:
            return False
        if frame_key[1:] != previous[1:]:
            records.clear()
        elif timestamp == previous[0]:
            return False
    last_frames[image_frame.camera_id] = frame_key
    return True


class Basic2DTrackerBackend(TrackerBackend):
    """基于标签和 IoU 的首版二维跟踪器。"""

    def __init__(
        self,
        *,
        name: str = "basic_2d_tracker",
        lifecycle_policy: Optional[TrackLifecyclePolicy] = None,
    ) -> None:
        self.spec = TrackerBackendSpec(
            name=name,
            backend_kind="2d_iou_tracker",
            metadata={"future_extension": "预留 3D 轨迹关联接口"},
        )
        self._policy = lifecycle_policy or TrackLifecyclePolicy()
        self._records_by_camera: Dict[str, Dict[str, _TrackRecord]] = {}
        self._last_frames_by_camera: Dict[str, tuple] = {}
        self._track_counter = itertools.count(1)

    def update(
        self,
        image_frame: ImageFrame,
        *,
        detections_2d: Tuple[Detection2D, ...] = (),
        detections_3d: Tuple[Detection3D, ...] = (),
        camera_info: Optional[CameraInfo] = None,
    ) -> Tuple[Track, ...]:
        del detections_3d, camera_info
        camera_id = image_frame.camera_id
        records = self._records_by_camera.setdefault(camera_id, {})
        if not _accept_source_frame(self._last_frames_by_camera, image_frame, records):
            return tuple(record.to_contract() for record in records.values())
        matched_track_ids: Set[str] = set()
        emitted: List[Track] = []

        for detection in sorted(detections_2d, key=lambda item: item.score, reverse=True):
            if not _valid_detection(detection) or detection.camera_id not in (None, camera_id):
                continue
            matched = self._match_record(records, detection, matched_track_ids)
            if matched is None:
                matched = self._create_record(camera_id, detection)
                records[matched.track_id] = matched
            else:
                matched.hits += 1
                matched.misses = 0
                matched.bbox = detection.bbox
                matched.score = detection.score
                matched.attributes = {
                    **dict(matched.attributes or {}),
                    **dict(detection.attributes),
                }

            matched.state = (
                TrackState.TRACKED
                if matched.hits >= self._policy.tentative_hits_required
                else TrackState.TENTATIVE
            )
            matched_track_ids.add(matched.track_id)
            emitted.append(matched.to_contract())

        removed_track_ids: List[str] = []
        for track_id, record in records.items():
            if track_id in matched_track_ids:
                continue
            record.misses += 1
            if record.misses > self._policy.lost_ttl_frames:
                record.state = TrackState.REMOVED
                emitted.append(record.to_contract())
                removed_track_ids.append(track_id)
                continue
            record.state = TrackState.LOST
            emitted.append(record.to_contract())

        for track_id in removed_track_ids:
            records.pop(track_id, None)

        return tuple(emitted)

    def reset(self, camera_id: Optional[str] = None) -> None:
        if camera_id is None:
            self._records_by_camera.clear()
            self._last_frames_by_camera.clear()
            return
        self._records_by_camera.pop(camera_id, None)
        self._last_frames_by_camera.pop(camera_id, None)

    def _match_record(
        self,
        records: Dict[str, _TrackRecord],
        detection: Detection2D,
        matched_track_ids: Set[str],
    ) -> Optional[_TrackRecord]:
        best_record: Optional[_TrackRecord] = None
        best_iou = 0.0
        for record in records.values():
            if record.track_id in matched_track_ids:
                continue
            if record.label != detection.label:
                continue
            if record.state == TrackState.REMOVED:
                continue
            iou = _bbox_iou(record.bbox, detection.bbox)
            if iou < self._policy.iou_match_threshold:
                continue
            if iou > best_iou:
                best_iou = iou
                best_record = record
        return best_record

    def _create_record(self, camera_id: str, detection: Detection2D) -> _TrackRecord:
        suffix = next(self._track_counter)
        safe_camera = self._normalize_fragment(camera_id)
        safe_label = self._normalize_fragment(detection.label)
        track_id = f"trk_{safe_camera}_{safe_label}_{suffix:04d}"
        return _TrackRecord(
            track_id=track_id,
            camera_id=camera_id,
            label=detection.label,
            bbox=detection.bbox,
            score=detection.score,
            attributes=dict(detection.attributes),
        )

    def _normalize_fragment(self, value: str) -> str:
        result = []
        for char in value.lower():
            if char.isalnum():
                result.append(char)
            else:
                result.append("_")
        normalized = "".join(result).strip("_")
        return normalized or "item"


class ByteTrackTrackerBackend(TrackerBackend):
    """ByteTrack 风格二维多目标跟踪后端。"""

    def __init__(
        self,
        *,
        name: str = "bytetrack_tracker",
        high_threshold: float = 0.5,
        low_threshold: float = 0.1,
        iou_match_threshold: float = 0.3,
        lost_ttl_frames: int = 30,
        trajectory_limit: int = 12,
    ) -> None:
        self.spec = TrackerBackendSpec(
            name=name,
            backend_kind="bytetrack",
            metadata={
                "algorithm": "ByteTrack",
                "high_threshold": max(0.0, min(1.0, float(high_threshold))),
                "low_threshold": max(0.0, min(1.0, float(low_threshold))),
                "iou_match_threshold": max(0.0, min(1.0, float(iou_match_threshold))),
                "lost_ttl_frames": max(1, int(lost_ttl_frames)),
                "trajectory_limit": max(2, int(trajectory_limit)),
            },
        )
        self._high_threshold = float(self.spec.metadata["high_threshold"])
        self._low_threshold = float(self.spec.metadata["low_threshold"])
        self._iou_match_threshold = float(self.spec.metadata["iou_match_threshold"])
        self._lost_ttl_frames = int(self.spec.metadata["lost_ttl_frames"])
        self._trajectory_limit = int(self.spec.metadata["trajectory_limit"])
        self._records_by_camera: Dict[str, Dict[str, _TrackRecord]] = {}
        self._last_frames_by_camera: Dict[str, tuple] = {}
        self._track_counter = itertools.count(1)

    def update(
        self,
        image_frame: ImageFrame,
        *,
        detections_2d: Tuple[Detection2D, ...] = (),
        detections_3d: Tuple[Detection3D, ...] = (),
        camera_info: Optional[CameraInfo] = None,
    ) -> Tuple[Track, ...]:
        del detections_3d, camera_info
        camera_id = image_frame.camera_id
        records = self._records_by_camera.setdefault(camera_id, {})
        if not _accept_source_frame(self._last_frames_by_camera, image_frame, records):
            return tuple(record.to_contract() for record in records.values())
        matched_track_ids: Set[str] = set()
        emitted: List[Track] = []
        candidate_detections = [
            item for item in detections_2d
            if _valid_detection(item) and item.camera_id in (None, camera_id)
            and float(item.score) >= self._low_threshold
        ]
        high_detections = [item for item in candidate_detections if float(item.score) >= self._high_threshold]
        low_detections = [item for item in candidate_detections if float(item.score) < self._high_threshold]

        for detection in sorted(high_detections, key=lambda item: item.score, reverse=True):
            record = self._match_record(records, detection, matched_track_ids)
            if record is None:
                record = self._create_record(camera_id, detection, image_frame.timestamp)
                records[record.track_id] = record
            else:
                self._update_record(record, detection, image_frame.timestamp)
            matched_track_ids.add(record.track_id)
            emitted.append(record.to_contract())

        for detection in sorted(low_detections, key=lambda item: item.score, reverse=True):
            record = self._match_record(records, detection, matched_track_ids)
            if record is None:
                continue
            self._update_record(record, detection, image_frame.timestamp)
            matched_track_ids.add(record.track_id)
            emitted.append(record.to_contract())

        removed_track_ids: List[str] = []
        for track_id, record in records.items():
            if track_id in matched_track_ids:
                continue
            record.misses += 1
            record.state = TrackState.LOST
            self._refresh_record_metadata(record, last_seen_only=True)
            if record.misses > self._lost_ttl_frames:
                record.state = TrackState.REMOVED
                emitted.append(record.to_contract())
                removed_track_ids.append(track_id)
                continue
            emitted.append(record.to_contract())

        for track_id in removed_track_ids:
            records.pop(track_id, None)

        return tuple(emitted)

    def reset(self, camera_id: Optional[str] = None) -> None:
        if camera_id is None:
            self._records_by_camera.clear()
            self._last_frames_by_camera.clear()
            return
        self._records_by_camera.pop(camera_id, None)
        self._last_frames_by_camera.pop(camera_id, None)

    def _match_record(
        self,
        records: Dict[str, _TrackRecord],
        detection: Detection2D,
        matched_track_ids: Set[str],
    ) -> Optional[_TrackRecord]:
        best_record: Optional[_TrackRecord] = None
        best_iou = 0.0
        for record in records.values():
            if record.track_id in matched_track_ids:
                continue
            if record.label != detection.label:
                continue
            if record.state == TrackState.REMOVED:
                continue
            iou = _bbox_iou(record.bbox, detection.bbox)
            if iou < self._iou_match_threshold:
                continue
            if iou > best_iou:
                best_iou = iou
                best_record = record
        return best_record

    def _create_record(self, camera_id: str, detection: Detection2D, timestamp) -> _TrackRecord:
        suffix = next(self._track_counter)
        track_id = "bt_%s_%s_%04d" % (
            self._normalize_fragment(camera_id),
            self._normalize_fragment(detection.label),
            suffix,
        )
        record = _TrackRecord(
            track_id=track_id,
            camera_id=camera_id,
            label=detection.label,
            bbox=detection.bbox,
            score=detection.score,
            hits=1,
            misses=0,
            state=TrackState.TRACKED,
            attributes=dict(detection.attributes),
        )
        record.attributes = {
            **dict(record.attributes or {}),
            "tracker": "bytetrack",
            "first_seen": timestamp.isoformat(),
            "last_seen": timestamp.isoformat(),
            "velocity_hint": {"du_px_per_sec": 0.0, "dv_px_per_sec": 0.0},
            "short_trajectory": [self._trajectory_sample(detection, timestamp)],
            "track_age_frames": 1,
            "track_miss_frames": 0,
        }
        return record

    def _update_record(self, record: _TrackRecord, detection: Detection2D, timestamp) -> None:
        previous_center = self._bbox_center(record.bbox)
        previous_timestamp = str((record.attributes or {}).get("last_seen") or "")
        record.hits += 1
        record.misses = 0
        record.bbox = detection.bbox
        record.score = detection.score
        record.state = TrackState.TRACKED
        attributes = {**dict(record.attributes or {}), **dict(detection.attributes)}
        velocity_hint = {"du_px_per_sec": 0.0, "dv_px_per_sec": 0.0}
        current_center = self._bbox_center(detection.bbox)
        elapsed_sec = self._elapsed_seconds(previous_timestamp, timestamp)
        if previous_center is not None and current_center is not None and elapsed_sec > 0.0:
            velocity_hint = {
                "du_px_per_sec": round((current_center[0] - previous_center[0]) / elapsed_sec, 6),
                "dv_px_per_sec": round((current_center[1] - previous_center[1]) / elapsed_sec, 6),
            }
        trajectory = list(attributes.get("short_trajectory") or [])
        trajectory.append(self._trajectory_sample(detection, timestamp))
        attributes.update(
            {
                "tracker": "bytetrack",
                "last_seen": timestamp.isoformat(),
                "velocity_hint": velocity_hint,
                "short_trajectory": trajectory[-self._trajectory_limit :],
                "track_age_frames": record.hits,
                "track_miss_frames": record.misses,
            }
        )
        record.attributes = attributes

    def _refresh_record_metadata(self, record: _TrackRecord, *, last_seen_only: bool = False) -> None:
        attributes = dict(record.attributes or {})
        attributes.update(
            {
                "tracker": "bytetrack",
                "track_age_frames": record.hits,
                "track_miss_frames": record.misses,
            }
        )
        if not last_seen_only:
            attributes.setdefault("last_seen", "")
        record.attributes = attributes

    def _trajectory_sample(self, detection: Detection2D, timestamp) -> Dict[str, object]:
        center = self._bbox_center(detection.bbox) or (0.0, 0.0)
        return {
            "timestamp": timestamp.isoformat(),
            "center_u_px": round(float(center[0]), 4),
            "center_v_px": round(float(center[1]), 4),
            "bbox": {
                "x_px": round(float(detection.bbox.x_px), 4),
                "y_px": round(float(detection.bbox.y_px), 4),
                "width_px": round(float(detection.bbox.width_px), 4),
                "height_px": round(float(detection.bbox.height_px), 4),
            },
            "score": round(float(detection.score), 6),
        }

    def _bbox_center(self, bbox: Optional[BoundingBox2D]) -> Optional[Tuple[float, float]]:
        if bbox is None:
            return None
        return float(bbox.x_px) + (float(bbox.width_px) * 0.5), float(bbox.y_px) + (float(bbox.height_px) * 0.5)

    def _elapsed_seconds(self, previous_timestamp: str, current_timestamp) -> float:
        if not previous_timestamp:
            return 0.0
        try:
            previous = current_timestamp.__class__.fromisoformat(previous_timestamp)
        except ValueError:
            return 0.0
        return max(0.0, (current_timestamp - previous).total_seconds())

    def _normalize_fragment(self, value: str) -> str:
        result = []
        for char in value.lower():
            if char.isalnum():
                result.append(char)
            else:
                result.append("_")
        normalized = "".join(result).strip("_")
        return normalized or "item"
