from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

from contracts.base import MetadataDict
from contracts.image import CameraInfo, ImageFrame
from contracts.perception import Detection2D, Detection3D, SegmentationMask2D, SegmentationQuality, Track
from contracts.runtime_views import SceneSummary
from typing import Optional, Tuple


@dataclass(frozen=True)
class DetectorBackendSpec:
    """检测后端规范。"""

    name: str
    backend_kind: str
    version: str = "0.1.0"
    supports_2d: bool = True
    supports_3d: bool = False
    metadata: MetadataDict = field(default_factory=dict)


@dataclass(frozen=True)
class DetectionBundle:
    """检测处理结果。"""

    detections_2d: Tuple[Detection2D, ...] = field(default_factory=tuple)
    detections_3d: Tuple[Detection3D, ...] = field(default_factory=tuple)
    metadata: MetadataDict = field(default_factory=dict)


class DetectorBackend(ABC):
    """检测后端接口。"""

    spec: DetectorBackendSpec

    @abstractmethod
    def detect(
        self,
        image_frame: ImageFrame,
        camera_info: Optional[CameraInfo] = None,
    ) -> DetectionBundle:
        """处理单帧图像并返回标准化检测结果。"""


@dataclass(frozen=True)
class SegmentationCandidate:
    """分割候选摘要。"""

    label: str = ""
    score: float = 0.0
    segmentation: Optional[SegmentationMask2D] = None
    metadata: MetadataDict = field(default_factory=dict)


@dataclass(frozen=True)
class SegmentationBackendSpec:
    """分割后端规范。"""

    name: str
    backend_kind: str
    version: str = "0.1.0"
    metadata: MetadataDict = field(default_factory=dict)


@dataclass(frozen=True)
class SegmentationBundle:
    """分割处理结果。"""

    segments: Tuple[SegmentationCandidate, ...] = field(default_factory=tuple)
    metadata: MetadataDict = field(default_factory=dict)


class SegmentationBackend(ABC):
    """分割后端接口。"""

    spec: SegmentationBackendSpec

    @abstractmethod
    def segment(
        self,
        image_frame: ImageFrame,
        *,
        detections_2d: Tuple[Detection2D, ...] = (),
        camera_info: Optional[CameraInfo] = None,
    ) -> SegmentationBundle:
        """处理单帧图像并返回标准化分割结果。"""


class DisabledSegmentationBackend(SegmentationBackend):
    """默认禁用的分割后端。"""

    def __init__(
        self,
        *,
        name: str = "segmentation_disabled",
        configured_enabled: bool = False,
        configured_backend_name: Optional[str] = None,
        configured_backend_kind: str = "disabled_segmentation",
        reason: str = "disabled_by_config",
        metadata: Optional[MetadataDict] = None,
    ) -> None:
        spec_metadata = {
            "configured_enabled": bool(configured_enabled),
            "configured_backend_name": str(configured_backend_name or name or "segmentation_disabled"),
            "configured_backend_kind": str(configured_backend_kind or "disabled_segmentation"),
            "reason": str(reason or "disabled_by_config"),
        }
        spec_metadata.update(dict(metadata or {}))
        self.spec = SegmentationBackendSpec(
            name=name,
            backend_kind="disabled_segmentation",
            metadata=spec_metadata,
        )

    def segment(
        self,
        image_frame: ImageFrame,
        *,
        detections_2d: Tuple[Detection2D, ...] = (),
        camera_info: Optional[CameraInfo] = None,
    ) -> SegmentationBundle:
        del image_frame, detections_2d, camera_info
        return SegmentationBundle(
            metadata={
                **dict(self.spec.metadata),
                "backend": self.spec.name,
                "backend_kind": self.spec.backend_kind,
                "configured_enabled": self.spec.metadata.get("configured_enabled", False),
                "reason": self.spec.metadata.get("reason", "disabled_by_config"),
                "segment_count": 0,
            }
        )


class BBoxOnlySegmentationBackend(SegmentationBackend):
    """使用检测框生成最低可用分割形状的回退后端。"""

    def __init__(
        self,
        *,
        name: str = "bbox_only_segmentation",
        configured_backend_name: Optional[str] = None,
        configured_backend_kind: str = "bbox_only_fallback",
        metadata: Optional[MetadataDict] = None,
    ) -> None:
        spec_metadata = {
            "configured_enabled": True,
            "configured_backend_name": str(configured_backend_name or name),
            "configured_backend_kind": str(configured_backend_kind or "bbox_only_fallback"),
            "source": "bbox_only_fallback",
            "reason": "bbox_only_fallback",
        }
        spec_metadata.update(dict(metadata or {}))
        self.spec = SegmentationBackendSpec(
            name=name,
            backend_kind="bbox_only_fallback",
            metadata=spec_metadata,
        )

    def segment(
        self,
        image_frame: ImageFrame,
        *,
        detections_2d: Tuple[Detection2D, ...] = (),
        camera_info: Optional[CameraInfo] = None,
    ) -> SegmentationBundle:
        del camera_info
        segments = []
        frame_area = float(max(1, image_frame.width_px) * max(1, image_frame.height_px))
        for index, detection in enumerate(detections_2d):
            x1, y1, x2, y2 = self._bbox_xyxy(detection)
            area_px = max(0.0, x2 - x1) * max(0.0, y2 - y1)
            polygon = ((x1, y1), (x2, y1), (x2, y2), (x1, y2))
            segmentation = SegmentationMask2D(
                polygon_px=polygon,
                centroid_uv=((x1 + x2) / 2.0, (y1 + y2) / 2.0),
                bbox_xyxy=(x1, y1, x2, y2),
                area_px=area_px,
                area_ratio=area_px / frame_area,
                quality=SegmentationQuality(score=None),
                frame_id=image_frame.frame_id,
                camera_id=detection.camera_id or image_frame.camera_id,
                source="bbox_only_fallback",
                attributes={"detection_index": int(index)},
            )
            segments.append(
                SegmentationCandidate(
                    label=detection.label,
                    score=detection.score,
                    segmentation=segmentation,
                    metadata={
                        "detection_index": int(index),
                        "bbox_xyxy": [x1, y1, x2, y2],
                        "mask_center_px": [(x1 + x2) / 2.0, (y1 + y2) / 2.0],
                        "mask_polygon_px": [[px, py] for px, py in polygon],
                        "segmentation_polygon_px": [[px, py] for px, py in polygon],
                        "area_px": area_px,
                        "area_ratio": area_px / frame_area,
                        "segmentation_available": False,
                        "segmentation_status": "bbox_only_fallback",
                        "segmentation_source": "bbox_only_fallback",
                    },
                )
            )
        return SegmentationBundle(
            segments=tuple(segments),
            metadata={
                **dict(self.spec.metadata),
                "backend": self.spec.name,
                "backend_kind": self.spec.backend_kind,
                "source": "bbox_only_fallback",
                "segment_count": len(segments),
                "detection_count": len(detections_2d),
            },
        )

    def _bbox_xyxy(self, detection: Detection2D) -> Tuple[float, float, float, float]:
        bbox = detection.bbox
        return (
            float(bbox.x_px),
            float(bbox.y_px),
            float(bbox.x_px + bbox.width_px),
            float(bbox.y_px + bbox.height_px),
        )


class DetectionAttributesSegmentationBackend(SegmentationBackend):
    """从检测属性中收敛分割结果的轻量后端。"""

    _SEGMENTATION_KEYS = (
        "mask",
        "mask_center_px",
        "mask_polygon_px",
        "segmentation_polygon_px",
        "area_px",
        "area_ratio",
        "contour",
        "crop_box",
        "segmentation_score",
        "predicted_iou",
        "stability_score",
        "depth_coverage",
        "map_projection_coverage",
        "reprojection_error",
        "truncation_flags",
        "occlusion_flags",
    )

    def __init__(
        self,
        *,
        name: str = "detection_attributes_segmentation",
        backend_kind: str = "detection_attributes",
        configured_backend_name: Optional[str] = None,
        configured_backend_kind: Optional[str] = None,
        metadata: Optional[MetadataDict] = None,
    ) -> None:
        spec_metadata = {
            "configured_enabled": True,
            "configured_backend_name": str(configured_backend_name or name),
            "configured_backend_kind": str(configured_backend_kind or backend_kind or "detection_attributes"),
            "source": "detection_attributes",
        }
        spec_metadata.update(dict(metadata or {}))
        self.spec = SegmentationBackendSpec(
            name=name,
            backend_kind="detection_attributes",
            metadata=spec_metadata,
        )

    def segment(
        self,
        image_frame: ImageFrame,
        *,
        detections_2d: Tuple[Detection2D, ...] = (),
        camera_info: Optional[CameraInfo] = None,
    ) -> SegmentationBundle:
        del camera_info
        segments = []
        for index, detection in enumerate(detections_2d):
            metadata = self._extract_segment_metadata(detection, image_frame, detection_index=index)
            if not metadata:
                continue
            segmentation = self._build_structured_segmentation(metadata)
            segments.append(
                SegmentationCandidate(
                    label=detection.label,
                    score=float(metadata.get("segmentation_score", detection.score)),
                    segmentation=segmentation,
                    metadata=metadata,
                )
            )
        return SegmentationBundle(
            segments=tuple(segments),
            metadata={
                **dict(self.spec.metadata),
                "backend": self.spec.name,
                "backend_kind": self.spec.backend_kind,
                "source": "detection_attributes",
                "segment_count": len(segments),
                "detection_count": len(detections_2d),
            },
        )

    def _extract_segment_metadata(
        self,
        detection: Detection2D,
        image_frame: ImageFrame,
        *,
        detection_index: int,
    ) -> MetadataDict:
        attributes = dict(detection.attributes or {})
        metadata = {
            key: attributes[key]
            for key in self._SEGMENTATION_KEYS
            if key in attributes and attributes[key] is not None
        }
        if not metadata:
            return {}
        metadata.setdefault("bbox_xyxy", self._bbox_xyxy(detection))
        metadata.setdefault("frame_id", image_frame.frame_id)
        metadata.setdefault("camera_id", detection.camera_id or image_frame.camera_id)
        metadata.setdefault("detection_index", int(detection_index))
        return metadata

    def _bbox_xyxy(self, detection: Detection2D) -> Tuple[float, float, float, float]:
        bbox = detection.bbox
        return (
            float(bbox.x_px),
            float(bbox.y_px),
            float(bbox.x_px + bbox.width_px),
            float(bbox.y_px + bbox.height_px),
        )

    def _build_structured_segmentation(self, metadata: MetadataDict) -> SegmentationMask2D:
        polygon = self._coerce_polygon(metadata.get("mask_polygon_px") or metadata.get("segmentation_polygon_px"))
        centroid = self._coerce_point(metadata.get("centroid_uv") or metadata.get("mask_center_px"))
        bbox = self._coerce_box(metadata.get("bbox_xyxy"))
        crop_box = self._coerce_box(metadata.get("crop_box"))
        quality = SegmentationQuality(
            score=self._coerce_float(metadata.get("segmentation_score")),
            predicted_iou=self._coerce_float(metadata.get("predicted_iou")),
            stability_score=self._coerce_float(metadata.get("stability_score")),
            depth_coverage=self._coerce_float(metadata.get("depth_coverage")),
            map_projection_coverage=self._coerce_float(metadata.get("map_projection_coverage")),
            reprojection_error=self._coerce_float(metadata.get("reprojection_error")),
        )
        return SegmentationMask2D(
            polygon_px=polygon,
            centroid_uv=centroid,
            bbox_xyxy=bbox,
            area_px=self._coerce_float(metadata.get("area_px")),
            area_ratio=self._coerce_float(metadata.get("area_ratio")),
            contour=dict(metadata.get("contour") or {}) if isinstance(metadata.get("contour"), dict) else {},
            crop_box=crop_box,
            quality=quality,
            frame_id=str(metadata.get("frame_id") or "") or None,
            camera_id=str(metadata.get("camera_id") or "") or None,
            source=str(metadata.get("source") or "") or None,
            truncation_flags=self._coerce_flags(metadata.get("truncation_flags")),
            occlusion_flags=self._coerce_flags(metadata.get("occlusion_flags")),
            attributes={
                key: value
                for key, value in metadata.items()
                if key not in self._SEGMENTATION_KEYS and key not in {"bbox_xyxy", "frame_id", "camera_id", "detection_index"}
            },
        )

    def _coerce_polygon(self, payload: object) -> Tuple[Tuple[float, float], ...]:
        if not isinstance(payload, (list, tuple)):
            return ()
        points = []
        for item in payload:
            point = self._coerce_point(item)
            if point is not None:
                points.append(point)
        return tuple(points)

    def _coerce_point(self, payload: object) -> Optional[Tuple[float, float]]:
        if not isinstance(payload, (list, tuple)) or len(payload) < 2:
            return None
        try:
            return float(payload[0]), float(payload[1])
        except (TypeError, ValueError):
            return None

    def _coerce_box(self, payload: object) -> Optional[Tuple[float, float, float, float]]:
        if not isinstance(payload, (list, tuple)) or len(payload) < 4:
            return None
        try:
            return float(payload[0]), float(payload[1]), float(payload[2]), float(payload[3])
        except (TypeError, ValueError):
            return None

    def _coerce_float(self, payload: object) -> Optional[float]:
        if payload is None:
            return None
        try:
            return float(payload)
        except (TypeError, ValueError):
            return None

    def _coerce_flags(self, payload: object) -> Tuple[str, ...]:
        if payload is None:
            return ()
        if isinstance(payload, str):
            return (payload,) if payload else ()
        if isinstance(payload, (list, tuple, set)):
            return tuple(str(item) for item in payload if str(item))
        return (str(payload),) if str(payload) else ()


@dataclass(frozen=True)
class TrackerBackendSpec:
    """跟踪后端规范。"""

    name: str
    backend_kind: str
    version: str = "0.1.0"
    supports_2d: bool = True
    supports_3d: bool = False
    metadata: MetadataDict = field(default_factory=dict)


class TrackerBackend(ABC):
    """跟踪后端接口。"""

    spec: TrackerBackendSpec

    @abstractmethod
    def update(
        self,
        image_frame: ImageFrame,
        *,
        detections_2d: Tuple[Detection2D, ...] = (),
        detections_3d: Tuple[Detection3D, ...] = (),
        camera_info: Optional[CameraInfo] = None,
    ) -> Tuple[Track, ...]:
        """基于当前帧检测结果更新轨迹。"""

    def reset(self, camera_id: Optional[str] = None) -> None:
        """重置一个或全部相机的跟踪状态。"""

        del camera_id


@dataclass(frozen=True)
class SceneDescriptionBackendSpec:
    """场景摘要后端规范。"""

    name: str
    backend_kind: str
    version: str = "0.1.0"
    metadata: MetadataDict = field(default_factory=dict)


class SceneDescriptionBackend(ABC):
    """场景摘要后端接口。"""

    spec: SceneDescriptionBackendSpec

    @abstractmethod
    def describe(
        self,
        *,
        camera_id: str,
        detections_2d: Tuple[Detection2D, ...],
        detections_3d: Tuple[Detection3D, ...],
        tracks: Tuple[Track, ...],
        image_frame: ImageFrame,
        camera_info: Optional[CameraInfo] = None,
    ) -> SceneSummary:
        """把检测与跟踪结果聚合成中文场景摘要。"""
