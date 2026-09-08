"""同相机检测结果的高重叠去重，保留替代标签以供核对。

跨标签抑制是可配置的单帧启发式；高度重叠不能证明真实物体身份相同。
"""

from __future__ import annotations

from typing import Dict, List, Tuple
import math

import numpy as np

from contracts.perception import BoundingBox2D, Detection2D


def _bbox_iou(left: BoundingBox2D, right: BoundingBox2D) -> float:
    """两个 bbox 的 IoU（交并比）。与 trackers._bbox_iou 同义，独立一份避免跨模块耦合。"""

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


def _valid_detection(detection: Detection2D) -> bool:
    """拒绝非有限或空框，防止非法模型结果进入去重和轨迹状态。"""

    bbox = detection.bbox
    values = (detection.score, bbox.x_px, bbox.y_px, bbox.width_px, bbox.height_px)
    return (
        all(math.isfinite(float(value)) for value in values)
        and 0.0 <= detection.score <= 1.0
        and bbox.x_px >= 0.0 and bbox.y_px >= 0.0
        and bbox.width_px > 0.0 and bbox.height_px > 0.0
        and math.isfinite(bbox.width_px * bbox.height_px)
        and math.isfinite(bbox.x_px + bbox.width_px) and math.isfinite(bbox.y_px + bbox.height_px)
    )


class ClassAgnosticNmsPostProcessor:
    """类无关 NMS 后处理器。

    输入一帧的检测框元组，输出去重后的检测框元组与统计信息。被抑制的框其标签会
    合并进代表框的 ``attributes["alternative_labels"]``（去重、保序）。
    """

    def __init__(self, *, iou_threshold: float = 0.8, enabled: bool = True) -> None:
        # 阈值钳到 [0,1]；偏高（默认 0.8）只合并几乎完全重合的框。
        self._iou_threshold = max(0.0, min(1.0, float(iou_threshold)))
        self._enabled = bool(enabled)

    @property
    def iou_threshold(self) -> float:
        return self._iou_threshold

    @property
    def enabled(self) -> bool:
        return self._enabled

    def process(
        self,
        detections: Tuple[Detection2D, ...],
    ) -> Tuple[Tuple[Detection2D, ...], Dict[str, int]]:
        """对单帧检测做类无关 NMS。

        返回 ``(kept_detections, stats)``；stats 含 raw_count / kept_count /
        suppressed_count，便于 overlay 与实机观测。
        """

        raw_count = len(detections)
        detections = tuple(item for item in detections if _valid_detection(item))
        invalid_count = raw_count - len(detections)
        if not self._enabled or len(detections) <= 1:
            return detections, {
                "raw_count": raw_count,
                "kept_count": len(detections),
                "suppressed_count": 0,
                "invalid_count": invalid_count,
            }

        # 按置信度从高到低排序；忽略标签。保留原始索引仅用于稳定性（同分时保序）。
        order = sorted(
            range(len(detections)),
            key=lambda idx: (detections[idx].score, -idx),
            reverse=True,
        )

        # 常见少量检测直接比较，避免数组准备与逐代表框调度的固定开销。
        boxes = None
        if len(detections) > 32:
            boxes = np.asarray([
                [item.bbox.x_px, item.bbox.y_px, item.bbox.x_px + item.bbox.width_px,
                 item.bbox.y_px + item.bbox.height_px]
                for item in detections
            ], dtype=np.float64)
            areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
            cameras = np.asarray([item.camera_id or "" for item in detections], dtype=object)
        suppressed = np.zeros(len(detections), dtype=bool)
        kept: List[Detection2D] = []
        for i_pos, idx in enumerate(order):
            if suppressed[idx]:
                continue
            representative = detections[idx]
            alternative_labels: List[str] = []
            if boxes is None:
                duplicate_ids = []
                for jdx in order[i_pos + 1 :]:
                    other = detections[jdx]
                    if suppressed[jdx] or (other.camera_id or "") != (representative.camera_id or ""):
                        continue
                    overlap = _bbox_iou(representative.bbox, other.bbox)
                    if overlap > 0.0 and overlap >= self._iou_threshold:
                        duplicate_ids.append(jdx)
            else:
                remaining = np.asarray(order[i_pos + 1 :], dtype=np.intp)
                remaining = remaining[~suppressed[remaining] & (cameras[remaining] == cameras[idx])]
                intersections = np.maximum(0.0, np.minimum(boxes[idx, 2:], boxes[remaining, 2:])
                                           - np.maximum(boxes[idx, :2], boxes[remaining, :2])).prod(axis=1)
                unions = areas[idx] + areas[remaining] - intersections
                overlaps = np.divide(intersections, unions, out=np.zeros_like(intersections), where=unions > 0.0)
                duplicate_ids = remaining[(intersections > 0.0) & (overlaps >= self._iou_threshold)]
            # 数值比较批量完成，只有被抑制框需要读取标签。
            for jdx in duplicate_ids:
                other = detections[jdx]
                suppressed[jdx] = True
                if other.label and other.label != representative.label:
                    alternative_labels.append(other.label)
                alternatives = other.attributes.get("alternative_labels")
                if isinstance(alternatives, (list, tuple)):
                    alternative_labels.extend(str(item) for item in alternatives if str(item) != representative.label)

            if alternative_labels:
                kept.append(self._attach_alternative_labels(representative, alternative_labels))
            else:
                kept.append(representative)

        suppressed_count = int(np.count_nonzero(suppressed))
        stats = {
            "raw_count": raw_count,
            "kept_count": len(kept),
            "suppressed_count": suppressed_count,
            "invalid_count": invalid_count,
        }
        return tuple(kept), stats

    @staticmethod
    def _attach_alternative_labels(
        detection: Detection2D,
        new_labels: List[str],
    ) -> Detection2D:
        """把被抑制框的标签合并进代表框 attributes（去重、保序），返回新副本。"""

        attributes = dict(detection.attributes or {})
        existing = attributes.get("alternative_labels")
        merged: List[str] = []
        seen = set()
        if isinstance(existing, (list, tuple)):
            for item in existing:
                text = str(item).strip()
                if text and text not in seen:
                    seen.add(text)
                    merged.append(text)
        for item in new_labels:
            text = str(item).strip()
            if text and text not in seen:
                seen.add(text)
                merged.append(text)
        attributes["alternative_labels"] = merged
        return detection.model_copy(update={"attributes": attributes}, deep=False)
