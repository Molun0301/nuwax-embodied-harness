from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Iterable, List, Optional, Tuple

from contracts.image import CameraInfo, ImageFrame
from contracts.perception import BoundingBox2D, Detection2D, Detection3D, SegmentationMask2D, SegmentationQuality
from gateways.errors import GatewayError
from services.model_providers import resolve_detection_provider_spec
from services.perception.base import DetectionBundle, DetectorBackend, DetectorBackendSpec
from services.perception.image_utils import decode_image_frame_to_bgr


DETECTOR_LOGGER = logging.getLogger("nuwax_robot_bridge.perception.detectors")
_TORCHVISION_NMS_PATCHED = False


def _box_iou_for_nms(boxes_left, boxes_right):
    """计算两组框的 IoU（交并比）。"""

    try:
        import torch
    except ImportError as exc:
        raise RuntimeError("当前环境未安装 torch，无法执行 NMS 回退。") from exc

    if boxes_left.numel() == 0 or boxes_right.numel() == 0:
        return torch.zeros(
            (boxes_left.shape[0], boxes_right.shape[0]),
            dtype=boxes_left.dtype,
            device=boxes_left.device,
        )

    left_top = torch.maximum(boxes_left[:, None, :2], boxes_right[None, :, :2])
    right_bottom = torch.minimum(boxes_left[:, None, 2:], boxes_right[None, :, 2:])
    wh = (right_bottom - left_top).clamp(min=0)
    inter = wh[..., 0] * wh[..., 1]

    area_left = ((boxes_left[:, 2] - boxes_left[:, 0]).clamp(min=0)) * (
        (boxes_left[:, 3] - boxes_left[:, 1]).clamp(min=0)
    )
    area_right = ((boxes_right[:, 2] - boxes_right[:, 0]).clamp(min=0)) * (
        (boxes_right[:, 3] - boxes_right[:, 1]).clamp(min=0)
    )
    union = area_left[:, None] + area_right[None, :] - inter
    return inter / union.clamp(min=1e-12)


def _torch_nms_fallback(boxes, scores, iou_threshold):
    """使用纯 torch 实现 NMS，避免依赖 torchvision 的 C++ 扩展。"""

    try:
        import torch
    except ImportError as exc:
        raise RuntimeError("当前环境未安装 torch，无法执行 NMS 回退。") from exc

    if boxes.ndim != 2 or boxes.shape[-1] != 4:
        raise RuntimeError("NMS 输入框张量必须是 [N, 4] 形状。")
    if scores.ndim != 1 or scores.shape[0] != boxes.shape[0]:
        raise RuntimeError("NMS 分数张量必须与框数量一致。")
    if boxes.numel() == 0:
        return torch.empty((0,), dtype=torch.int64, device=boxes.device)

    order = scores.argsort(descending=True)
    keep = []
    while order.numel() > 0:
        current = order[0]
        keep.append(current)
        if order.numel() == 1:
            break
        remaining = order[1:]
        current_box = boxes[current].unsqueeze(0)
        iou = _box_iou_for_nms(current_box, boxes[remaining]).squeeze(0)
        order = remaining[iou <= float(iou_threshold)]

    return torch.stack(keep).to(dtype=torch.int64)


def _ensure_torchvision_nms_fallback() -> None:
    """在 torchvision NMS 不可用时安装纯 torch 回退实现。"""

    global _TORCHVISION_NMS_PATCHED
    if _TORCHVISION_NMS_PATCHED:
        return

    try:
        import torch
        import torchvision
        import torchvision.ops as torchvision_ops
        import torchvision.ops.boxes as torchvision_boxes
    except ImportError:
        return

    probe_boxes = torch.tensor([[0.0, 0.0, 4.0, 4.0], [0.5, 0.5, 3.5, 3.5]], dtype=torch.float32)
    probe_scores = torch.tensor([0.9, 0.8], dtype=torch.float32)
    try:
        torchvision_ops.nms(probe_boxes, probe_scores, 0.5)
        return
    except Exception as exc:
        DETECTOR_LOGGER.warning(
            "检测到 torchvision NMS 不可用，已切换到纯 torch NMS 回退。error=%s",
            exc,
        )

    torchvision_ops.nms = _torch_nms_fallback
    torchvision_boxes.nms = _torch_nms_fallback
    _TORCHVISION_NMS_PATCHED = True


def _clamp_score(value: object) -> float:
    try:
        score = float(value)
    except (TypeError, ValueError):
        score = 0.0
    return max(0.0, min(1.0, score))


def _normalize_label(value: object) -> str:
    label = str(value or "unknown").strip().lower()
    return label or "unknown"


def _clamp_bbox(
    *,
    x_px: object,
    y_px: object,
    width_px: object,
    height_px: object,
    image_frame: ImageFrame,
) -> BoundingBox2D:
    frame_width = max(1.0, float(image_frame.width_px))
    frame_height = max(1.0, float(image_frame.height_px))
    x = max(0.0, min(float(x_px), frame_width - 1.0))
    y = max(0.0, min(float(y_px), frame_height - 1.0))
    width = max(1.0, min(float(width_px), frame_width - x))
    height = max(1.0, min(float(height_px), frame_height - y))
    return BoundingBox2D(x_px=x, y_px=y, width_px=width, height_px=height)


def normalize_detection_2d(raw_detection: object, image_frame: ImageFrame) -> Detection2D:
    """把检测后端的输出统一规整到标准二维检测契约。"""

    if isinstance(raw_detection, Detection2D):
        bbox = _clamp_bbox(
            x_px=raw_detection.bbox.x_px,
            y_px=raw_detection.bbox.y_px,
            width_px=raw_detection.bbox.width_px,
            height_px=raw_detection.bbox.height_px,
            image_frame=image_frame,
        )
        return raw_detection.model_copy(
            update={
                "label": _normalize_label(raw_detection.label),
                "score": _clamp_score(raw_detection.score),
                "bbox": bbox,
                "camera_id": raw_detection.camera_id or image_frame.camera_id,
                "track_id": raw_detection.track_id,
                "attributes": dict(raw_detection.attributes),
            },
            deep=True,
        )

    if not isinstance(raw_detection, dict):
        raise GatewayError(f"检测结果类型非法：{type(raw_detection)!r}")

    bbox_payload = dict(raw_detection.get("bbox") or {})
    bbox = _clamp_bbox(
        x_px=bbox_payload.get("x_px", raw_detection.get("x_px", 0.0)),
        y_px=bbox_payload.get("y_px", raw_detection.get("y_px", 0.0)),
        width_px=bbox_payload.get("width_px", raw_detection.get("width_px", image_frame.width_px)),
        height_px=bbox_payload.get("height_px", raw_detection.get("height_px", image_frame.height_px)),
        image_frame=image_frame,
    )
    known_keys = {
        "label",
        "score",
        "bbox",
        "x_px",
        "y_px",
        "width_px",
        "height_px",
        "camera_id",
        "track_id",
        "segmentation",
        "attributes",
    }
    merged_attributes = dict(raw_detection.get("attributes") or {})
    for key, value in raw_detection.items():
        if key in known_keys:
            continue
        merged_attributes.setdefault(str(key), value)
    return Detection2D(
        label=_normalize_label(raw_detection.get("label")),
        score=_clamp_score(raw_detection.get("score", 0.0)),
        bbox=bbox,
        camera_id=str(raw_detection.get("camera_id") or image_frame.camera_id),
        track_id=str(raw_detection.get("track_id")).strip() or None if raw_detection.get("track_id") else None,
        segmentation=raw_detection.get("segmentation"),
        attributes=merged_attributes,
    )


def normalize_detection_3d(raw_detection: object) -> Detection3D:
    """把检测后端的输出统一规整到标准三维检测契约。"""

    if isinstance(raw_detection, Detection3D):
        return raw_detection.model_copy(
            update={
                "label": _normalize_label(raw_detection.label),
                "score": _clamp_score(raw_detection.score),
                "attributes": dict(raw_detection.attributes),
            },
            deep=True,
        )
    if not isinstance(raw_detection, dict):
        raise GatewayError(f"三维检测结果类型非法：{type(raw_detection)!r}")
    known_keys = {
        "label",
        "score",
        "pose",
        "size_x_m",
        "size_y_m",
        "size_z_m",
        "attributes",
    }
    merged_attributes = dict(raw_detection.get("attributes") or {})
    for key, value in raw_detection.items():
        if key in known_keys:
            continue
        merged_attributes.setdefault(str(key), value)
    return Detection3D.model_validate(
        {
            **raw_detection,
            "label": _normalize_label(raw_detection.get("label")),
            "score": _clamp_score(raw_detection.get("score", 0.0)),
            "attributes": merged_attributes,
        }
    )


class MetadataDrivenDetectorBackend(DetectorBackend):
    """基于图像元数据的基础检测后端。

    该后端用于平台首版管线验证：
    1. 统一多来源图像输入后的检测结构；
    2. 在没有重模型依赖时仍能验证检测标准化和上层链路；
    3. 后续接入 YOLO、GroundingDINO、VLM 时只需替换后端。
    """

    def __init__(
        self,
        *,
        name: str = "metadata_detector",
        backend_kind: str = "metadata",
    ) -> None:
        provider_metadata = resolve_detection_provider_spec(
            backend_name=name,
            backend_kind=backend_kind,
        ).to_metadata()
        self.spec = DetectorBackendSpec(
            name=name,
            backend_kind=backend_kind,
            metadata={
                "input_contract": "ImageFrame/CameraInfo",
                "supports_runtime_switch": True,
                "provider": provider_metadata,
                **provider_metadata,
            },
        )

    def detect(
        self,
        image_frame: ImageFrame,
        camera_info: Optional[CameraInfo] = None,
    ) -> DetectionBundle:
        del camera_info
        raw_2d = self._iter_raw_items(image_frame.metadata.get("detections_2d"))
        raw_3d = self._iter_raw_items(image_frame.metadata.get("detections_3d"))
        detections_2d = tuple(normalize_detection_2d(item, image_frame) for item in raw_2d)
        detections_3d = tuple(normalize_detection_3d(item) for item in raw_3d)
        return DetectionBundle(
            detections_2d=detections_2d,
            detections_3d=detections_3d,
            metadata={
                "backend_name": self.spec.name,
                "backend_kind": self.spec.backend_kind,
                "detector_provider": dict(self.spec.metadata.get("provider") or {}),
                "raw_detection_count": len(detections_2d) + len(detections_3d),
            },
        )

    def _iter_raw_items(self, payload: object) -> Tuple[object, ...]:
        if payload is None:
            return ()
        if isinstance(payload, tuple):
            return payload
        if isinstance(payload, list):
            return tuple(payload)
        if isinstance(payload, Iterable) and not isinstance(payload, (str, bytes, dict)):
            return tuple(payload)
        raise GatewayError("检测元数据必须是列表或可迭代对象。")


class UltralyticsYoloDetectorBackend(DetectorBackend):
    """基于 Ultralytics YOLO 的生产级本地检测后端。

    默认优先使用 TensorRT 引擎文件：
    1. 若明确给出 `.engine`，直接按 TensorRT 方式加载；
    2. 若给出 `.pt` 且存在同名或显式指定的 `.engine`，优先加载 `.engine`；
    3. 若 TensorRT 引擎不存在，则自动回退到 `.pt`，避免宿主机启动失败。
    """

    def __init__(
        self,
        *,
        name: str = "yolo_local",
        weights: str = "yolo26n.pt",
        runtime_preference: str = "tensorrt",
        engine_path: str = "",
        device: str = "",
        half: bool = True,
        int8: bool = False,
        confidence_threshold: float = 0.25,
        iou_threshold: float = 0.45,
        image_size: int = 640,
        max_detections: int = 100,
    ) -> None:
        self._weights = str(weights).strip() or "yolo26n.pt"
        self._runtime_preference = str(runtime_preference or "tensorrt").strip().lower()
        self._engine_path = str(engine_path).strip()
        self._device = str(device).strip()
        self._half = bool(half)
        self._int8 = bool(int8)
        self._confidence_threshold = max(0.0, min(1.0, float(confidence_threshold)))
        self._iou_threshold = max(0.0, min(1.0, float(iou_threshold)))
        self._image_size = max(32, int(image_size))
        self._max_detections = max(1, int(max_detections))
        self._model = None
        self._resolved_model_source = ""
        self._resolved_backend_kind = "ultralytics_yolo"
        provider_metadata = resolve_detection_provider_spec(
            backend_name=name,
            backend_kind="ultralytics_yolo",
            model_name=self._weights,
        ).to_metadata()
        self.spec = DetectorBackendSpec(
            name=name,
            backend_kind="ultralytics_yolo",
            metadata={
                "provider": provider_metadata,
                **provider_metadata,
                "weights": self._weights,
                "runtime_preference": self._runtime_preference,
                "engine_path": self._engine_path,
                "device": self._device or "auto",
                "half": self._half,
                "int8": self._int8,
                "supports_runtime_switch": True,
                "input_contract": "ImageFrame/CameraInfo",
            },
        )

    def detect(
        self,
        image_frame: ImageFrame,
        camera_info: Optional[CameraInfo] = None,
    ) -> DetectionBundle:
        del camera_info
        image_bgr = decode_image_frame_to_bgr(image_frame)
        results = self._predict(image_bgr)
        first_result = results[0] if results else None
        detections_2d = self._normalize_yolo_result(first_result, image_frame)
        return DetectionBundle(
            detections_2d=detections_2d,
            detections_3d=(),
            metadata={
                "backend_name": self.spec.name,
                "backend_kind": self._resolved_backend_kind or self.spec.backend_kind,
                "detector_provider": dict(self.spec.metadata.get("provider") or {}),
                "raw_detection_count": len(detections_2d),
                "weights": self._weights,
                "model_source": self._resolved_model_source,
                "runtime_preference": self._runtime_preference,
                "device": self._device or "auto",
            },
        )

    def _predict(self, image_bgr) -> List[Any]:
        model = self._get_model()
        kwargs = {
            "conf": self._confidence_threshold,
            "iou": self._iou_threshold,
            "imgsz": self._image_size,
            "max_det": self._max_detections,
            "verbose": False,
        }
        if self._device:
            kwargs["device"] = self._device
        return list(model.predict(image_bgr, **kwargs))

    def _get_model(self):
        if self._model is not None:
            return self._model
        _ensure_torchvision_nms_fallback()
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "当前环境未安装 ultralytics，无法启用本地 YOLO 检测。"
            ) from exc
        model_source, backend_kind = self._resolve_model_source()
        self._model = YOLO(model_source)
        self._resolved_model_source = model_source
        self._resolved_backend_kind = backend_kind
        DETECTOR_LOGGER.info(
            "YOLO 本地检测后端已加载 source=%s backend=%s preference=%s",
            model_source,
            backend_kind,
            self._runtime_preference,
        )
        return self._model

    def _resolve_model_source(self) -> Tuple[str, str]:
        weights_path = Path(self._weights)
        explicit_engine_path = Path(self._engine_path) if self._engine_path else None

        if weights_path.suffix.lower() == ".engine":
            return str(weights_path), "ultralytics_yolo_tensorrt"

        engine_candidates = []
        if explicit_engine_path is not None:
            engine_candidates.append(explicit_engine_path)
        if weights_path.suffix:
            engine_candidates.append(weights_path.with_suffix(".engine"))

        if self._runtime_preference in ("tensorrt", "auto"):
            for candidate in engine_candidates:
                if candidate.exists():
                    return str(candidate), "ultralytics_yolo_tensorrt"
            if self._runtime_preference == "tensorrt":
                DETECTOR_LOGGER.warning(
                    "未找到 TensorRT 引擎，已回退到原始权重 source=%s candidates=%s",
                    self._weights,
                    [str(item) for item in engine_candidates],
                )

        return self._weights, "ultralytics_yolo_pytorch"

    def _normalize_yolo_result(
        self,
        result: Optional[Any],
        image_frame: ImageFrame,
    ) -> Tuple[Detection2D, ...]:
        if result is None:
            return ()
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return ()
        names = getattr(result, "names", None)
        if not isinstance(names, dict):
            names = getattr(self._get_model(), "names", {})

        xyxy_items = self._to_nested_list(getattr(boxes, "xyxy", ()))
        conf_items = self._to_list(getattr(boxes, "conf", ()))
        class_items = self._to_list(getattr(boxes, "cls", ()))
        mask_polygon_items = self._extract_mask_polygons(getattr(result, "masks", None))
        mask_quality_items = self._extract_mask_quality(getattr(result, "masks", None))

        detections = []
        for index, xyxy in enumerate(xyxy_items):
            if len(xyxy) < 4:
                continue
            x1, y1, x2, y2 = [float(value) for value in xyxy[:4]]
            width = max(1.0, x2 - x1)
            height = max(1.0, y2 - y1)
            class_id = int(class_items[index]) if index < len(class_items) else -1
            score = float(conf_items[index]) if index < len(conf_items) else 0.0
            label = self._resolve_label(names, class_id)
            attributes = {
                "backend_name": self.spec.name,
                "backend_kind": self._resolved_backend_kind or self.spec.backend_kind,
                "model_name": self._resolved_model_source or self._weights,
                "class_id": class_id,
                "grounding_score": _clamp_score(score),
                "relative_area": round((width * height) / float(image_frame.width_px * image_frame.height_px), 6),
                "center_x_ratio": round(((x1 + x2) * 0.5) / float(image_frame.width_px), 6),
                "center_y_ratio": round(((y1 + y2) * 0.5) / float(image_frame.height_px), 6),
                "segmentation_available": False,
                "segmentation_status": "bbox_only",
                "segmentation_priority": "bbox_only",
            }
            segmentation = None
            if index < len(mask_polygon_items):
                quality_metadata = mask_quality_items[index] if index < len(mask_quality_items) else {}
                mask_attributes, segmentation = self._build_mask_segmentation(
                    mask_polygon_items[index],
                    score,
                    image_frame,
                    quality_metadata=quality_metadata,
                )
                attributes.update(mask_attributes)
            detection = normalize_detection_2d(
                {
                    "label": label,
                    "score": score,
                    "bbox": {
                        "x_px": x1,
                        "y_px": y1,
                        "width_px": width,
                        "height_px": height,
                    },
                    "camera_id": image_frame.camera_id,
                    "attributes": attributes,
                },
                image_frame,
            )
            if segmentation is not None:
                detection = detection.model_copy(update={"segmentation": segmentation}, deep=True)
            detections.append(detection)
        return tuple(detections)

    def _extract_mask_polygons(self, masks: Any) -> List[List[List[float]]]:
        if masks is None:
            return []
        xy_payload = getattr(masks, "xy", None)
        if xy_payload is None:
            return []
        if hasattr(xy_payload, "tolist"):
            xy_payload = xy_payload.tolist()
        polygons = []
        for polygon in xy_payload:
            if hasattr(polygon, "tolist"):
                polygon = polygon.tolist()
            points = []
            for point in polygon or []:
                if hasattr(point, "tolist"):
                    point = point.tolist()
                if not isinstance(point, (list, tuple)) or len(point) < 2:
                    continue
                try:
                    points.append([float(point[0]), float(point[1])])
                except (TypeError, ValueError):
                    continue
            polygons.append(points)
        return polygons

    def _extract_mask_quality(self, masks: Any) -> List[dict]:
        if masks is None:
            return []
        quality_count = max(len(self._to_list_or_empty(getattr(masks, key, ()))) for key in ("predicted_iou", "stability_score", "scores"))
        quality_items = []
        for index in range(quality_count):
            metadata = {}
            predicted_iou = self._item_at(getattr(masks, "predicted_iou", ()), index)
            stability_score = self._item_at(getattr(masks, "stability_score", ()), index)
            segmentation_score = self._item_at(getattr(masks, "scores", ()), index)
            if predicted_iou is not None:
                metadata["predicted_iou"] = _clamp_score(predicted_iou)
            if stability_score is not None:
                metadata["stability_score"] = _clamp_score(stability_score)
            if segmentation_score is not None:
                metadata["segmentation_score"] = _clamp_score(segmentation_score)
            quality_items.append(metadata)
        return quality_items

    def _build_mask_segmentation(
        self,
        polygon: List[List[float]],
        score: float,
        image_frame: ImageFrame,
        *,
        quality_metadata: Optional[dict] = None,
    ) -> Tuple[dict, Optional[SegmentationMask2D]]:
        quality_metadata = dict(quality_metadata or {})
        if len(polygon) < 3:
            return {
                "segmentation_available": False,
                "segmentation_status": "invalid_polygon",
                "segmentation_priority": "bbox_only",
            }, None
        area_px = abs(self._polygon_signed_area(polygon))
        if area_px <= 0.0:
            return {
                "segmentation_available": False,
                "segmentation_status": "empty_polygon",
                "segmentation_priority": "bbox_only",
            }, None
        area_ratio = round(area_px / float(max(1, image_frame.width_px * image_frame.height_px)), 6)
        centroid = self._polygon_centroid(polygon)
        normalized_polygon = tuple((float(point[0]), float(point[1])) for point in polygon)
        centroid_uv = None
        if centroid is not None:
            centroid_uv = (round(centroid[0], 3), round(centroid[1], 3))
        bbox_xyxy = self._polygon_bbox(normalized_polygon)
        segmentation_score = _clamp_score(quality_metadata.get("segmentation_score", score))
        predicted_iou = self._optional_score(quality_metadata.get("predicted_iou"))
        stability_score = self._optional_score(quality_metadata.get("stability_score"))
        quality = SegmentationQuality(
            score=segmentation_score,
            predicted_iou=predicted_iou,
            stability_score=stability_score,
        )
        segmentation = SegmentationMask2D(
            polygon_px=normalized_polygon,
            centroid_uv=centroid_uv,
            bbox_xyxy=bbox_xyxy,
            area_px=round(area_px, 3),
            area_ratio=area_ratio,
            contour={"point_count": len(normalized_polygon)},
            crop_box=bbox_xyxy,
            quality=quality,
            frame_id=image_frame.frame_id,
            camera_id=image_frame.camera_id,
            source="ultralytics_yolo_masks",
            attributes={"grounding_score": _clamp_score(score)},
        )
        attributes = {
            "mask_polygon_px": [[float(x), float(y)] for x, y in normalized_polygon],
            "segmentation_polygon_px": [[float(x), float(y)] for x, y in normalized_polygon],
            "bbox_xyxy": [float(value) for value in bbox_xyxy],
            "crop_box": [float(value) for value in bbox_xyxy],
            "area_px": round(area_px, 3),
            "area_ratio": area_ratio,
            "contour": {"point_count": len(normalized_polygon)},
            "grounding_score": _clamp_score(score),
            "segmentation_score": segmentation_score,
            "segmentation_available": True,
            "segmentation_status": "segmentation_preferred",
            "segmentation_priority": "segmentation_over_bbox",
            "segmentation_source": "ultralytics_yolo_masks",
            "frame_id": image_frame.frame_id,
            "camera_id": image_frame.camera_id,
        }
        if predicted_iou is not None:
            attributes["predicted_iou"] = predicted_iou
        if stability_score is not None:
            attributes["stability_score"] = stability_score
        if centroid_uv is not None:
            attributes["mask_center_px"] = [float(centroid_uv[0]), float(centroid_uv[1])]
            attributes["centroid_uv"] = attributes["mask_center_px"]
        return attributes, segmentation

    def _polygon_signed_area(self, polygon: List[List[float]]) -> float:
        area = 0.0
        for index, point in enumerate(polygon):
            next_point = polygon[(index + 1) % len(polygon)]
            area += (point[0] * next_point[1]) - (next_point[0] * point[1])
        return area * 0.5

    def _polygon_centroid(self, polygon: List[List[float]]) -> Optional[Tuple[float, float]]:
        signed_area = self._polygon_signed_area(polygon)
        if abs(signed_area) < 1e-6:
            return None
        cx = 0.0
        cy = 0.0
        for index, point in enumerate(polygon):
            next_point = polygon[(index + 1) % len(polygon)]
            cross = (point[0] * next_point[1]) - (next_point[0] * point[1])
            cx += (point[0] + next_point[0]) * cross
            cy += (point[1] + next_point[1]) * cross
        scale = 1.0 / (6.0 * signed_area)
        return cx * scale, cy * scale

    def _polygon_bbox(self, polygon: Tuple[Tuple[float, float], ...]) -> Tuple[float, float, float, float]:
        xs = [point[0] for point in polygon]
        ys = [point[1] for point in polygon]
        return min(xs), min(ys), max(xs), max(ys)

    def _optional_score(self, value: object) -> Optional[float]:
        if value is None:
            return None
        return _clamp_score(value)

    def _to_list_or_empty(self, payload: Any) -> List[float]:
        if payload is None:
            return []
        if hasattr(payload, "tolist"):
            payload = payload.tolist()
        if isinstance(payload, (list, tuple)):
            return list(payload)
        return []

    def _item_at(self, payload: Any, index: int) -> Optional[object]:
        items = self._to_list_or_empty(payload)
        if index >= len(items):
            return None
        return items[index]

    def _resolve_label(self, names: Any, class_id: int) -> str:
        if isinstance(names, dict) and class_id in names:
            return str(names[class_id])
        return "class_%s" % class_id

    def _to_list(self, payload: Any) -> List[float]:
        if payload is None:
            return []
        if hasattr(payload, "tolist"):
            payload = payload.tolist()
        if isinstance(payload, tuple):
            return [float(item) for item in payload]
        if isinstance(payload, list):
            return [float(item) for item in payload]
        return [float(payload)]

    def _to_nested_list(self, payload: Any) -> List[List[float]]:
        if payload is None:
            return []
        if hasattr(payload, "tolist"):
            payload = payload.tolist()
        if not isinstance(payload, list):
            return []
        if payload and not isinstance(payload[0], list):
            return [[float(item) for item in payload]]
        return [[float(item) for item in row] for row in payload]


class DetectorPipeline:
    """检测后端选择与调度器。"""

    def __init__(
        self,
        backends: Tuple[DetectorBackend, ...],
        *,
        default_backend_name: Optional[str] = None,
    ) -> None:
        if not backends:
            raise ValueError("DetectorPipeline 至少需要一个检测后端。")
        self._backends = {backend.spec.name: backend for backend in backends}
        self._default_backend_name = default_backend_name or backends[0].spec.name
        if self._default_backend_name not in self._backends:
            raise ValueError("默认检测后端未注册。")

    def list_backends(self) -> Tuple[DetectorBackendSpec, ...]:
        """返回当前所有已注册检测后端规范。"""

        return tuple(backend.spec for backend in self._backends.values())

    @property
    def default_backend_name(self) -> str:
        """返回默认检测后端名称。"""

        return self._default_backend_name

    def detect(
        self,
        image_frame: ImageFrame,
        camera_info: Optional[CameraInfo] = None,
        *,
        backend_name: Optional[str] = None,
    ) -> Tuple[DetectionBundle, DetectorBackendSpec]:
        """使用指定后端处理图像。"""

        resolved_backend_name = backend_name or self._default_backend_name
        backend = self._backends.get(resolved_backend_name)
        if backend is None:
            raise GatewayError(f"未注册检测后端：{resolved_backend_name}")
        bundle = backend.detect(image_frame, camera_info)
        return bundle, backend.spec
