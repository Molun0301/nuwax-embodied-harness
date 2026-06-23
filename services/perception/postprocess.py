"""检测结果后处理：类无关 NMS（class-agnostic NMS）。

杠杆二（见方案文档第 6.2 节）：开放词汇模型会对同一图像区域同时输出多个语义
相近的标签框（如玻璃门同时被框为「门 / 玻璃 / 窗户」），它们框定几乎同一块像素，
仅标签不同。模型内置 NMS 通常只抑制同类别重叠，跨类别的重叠保留下来，于是同一
物体进入平台后变成多个框 / 多个语义实例。

本模块在 detector 之后、tracker 之前做一次**类无关 NMS**：忽略标签，凡 bbox IoU
高于阈值（偏高，默认 0.8）的重叠框只保留置信度最高的一个，被抑制框的标签存入代表
框的 ``attributes["alternative_labels"]`` 以备追溯。纯单帧、无状态，下游 tracker /
实例关联 / 记忆摄入无需改动。

设计依据：一帧图像里两个真正不同的物体不可能占据几乎相同的像素区域，因此「框高度
重叠」本身就是「同一物体」的充分证据，与标签内容无关——无需人工维护别名表。高阈值
保证只合并几乎完全重合的框，空间分离的同类物体（两扇窗、并排两把椅子）IoU 远不到
阈值，不会被误并。
"""

from __future__ import annotations

from typing import Dict, List, Tuple

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
    union = max(1.0, left_area + right_area - intersection)
    return intersection / union


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
        if not self._enabled or raw_count <= 1:
            return detections, {
                "raw_count": raw_count,
                "kept_count": raw_count,
                "suppressed_count": 0,
            }

        # 按置信度从高到低排序；忽略标签。保留原始索引仅用于稳定性（同分时保序）。
        order = sorted(
            range(raw_count),
            key=lambda idx: (detections[idx].score, -idx),
            reverse=True,
        )

        suppressed: List[bool] = [False] * raw_count
        kept: List[Detection2D] = []
        for i_pos, idx in enumerate(order):
            if suppressed[idx]:
                continue
            representative = detections[idx]
            rep_bbox = representative.bbox
            alternative_labels: List[str] = []
            # 与后续更低分框逐一比较，IoU 超阈值者抑制并收集其标签。
            for jdx in order[i_pos + 1 :]:
                if suppressed[jdx]:
                    continue
                other = detections[jdx]
                if _bbox_iou(rep_bbox, other.bbox) < self._iou_threshold:
                    continue
                suppressed[jdx] = True
                if other.label and other.label != representative.label:
                    alternative_labels.append(other.label)

            if alternative_labels:
                kept.append(self._attach_alternative_labels(representative, alternative_labels))
            else:
                kept.append(representative)

        suppressed_count = sum(1 for flag in suppressed if flag)
        stats = {
            "raw_count": raw_count,
            "kept_count": len(kept),
            "suppressed_count": suppressed_count,
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
