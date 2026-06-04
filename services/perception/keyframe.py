from __future__ import annotations

import math
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Optional, Tuple

from contracts.geometry import Pose
from services.memory.semantic_map_builder import SemanticMapBuildResult


@dataclass(frozen=True)
class KeyframeDecision:
    """关键帧判定结果。"""

    should_process: bool
    reason: str
    metadata: Dict[str, object]


class PerceptionKeyframeSelector:
    """关键帧选择器。

    默认规则采用“时间 + 位姿 + 区域/拓扑变化”的组合触发：
    1. 首帧直接进入关键帧；
    2. 位姿平移或朝向变化达到阈值时触发；
    3. 进入新的语义区域或拓扑节点时，仅在缺少稳定位姿上下文时作为补充触发；
    4. 如果位姿几乎不变，则区域/拓扑变化和最大间隔都会继续被抑制，避免静止时重复调用视觉 API；
    5. 只有在缺少位姿上下文时，才回退使用最大间隔兜底触发。
    """

    def __init__(
        self,
        *,
        min_interval_sec: float,
        translation_threshold_m: float,
        yaw_threshold_deg: float,
    ) -> None:
        self._min_interval_sec = max(0.05, float(min_interval_sec))
        self._translation_threshold_m = max(0.0, float(translation_threshold_m))
        self._yaw_threshold_rad = math.radians(max(0.0, float(yaw_threshold_deg)))
        self._last_keyframe_at = None
        self._last_keyframe_pose = None
        self._last_semantic_region_id = None
        self._last_topo_node_id = None

    def decide(
        self,
        *,
        now: datetime,
        current_pose: Optional[Pose],
        map_context: Optional[SemanticMapBuildResult],
    ) -> KeyframeDecision:
        anchor_metadata = self._resolve_anchor_metadata(current_pose=current_pose, map_context=map_context)
        if self._last_keyframe_at is None:
            return KeyframeDecision(True, "bootstrap", anchor_metadata)

        elapsed_sec = max(0.0, (now - self._last_keyframe_at).total_seconds())
        metadata = {
            **anchor_metadata,
            "elapsed_sec": round(elapsed_sec, 6),
        }
        if elapsed_sec < self._min_interval_sec:
            return KeyframeDecision(False, "below_min_interval", metadata)

        has_pose_context = current_pose is not None and self._last_keyframe_pose is not None
        pose_is_static = False
        if current_pose is not None and self._last_keyframe_pose is not None:
            translation_m = self._distance(current_pose, self._last_keyframe_pose)
            yaw_delta_rad = self._yaw_delta(current_pose, self._last_keyframe_pose)
            metadata.update(
                {
                    "translation_m": round(translation_m, 6),
                    "yaw_delta_deg": round(math.degrees(yaw_delta_rad), 6),
                }
            )
            if translation_m >= self._translation_threshold_m:
                return KeyframeDecision(True, "pose_translation", metadata)
            if yaw_delta_rad >= self._yaw_threshold_rad:
                return KeyframeDecision(True, "pose_rotation", metadata)
            pose_is_static = True

        semantic_region_id = anchor_metadata.get("semantic_region_id")
        topo_node_id = anchor_metadata.get("topo_node_id")
        if semantic_region_id and semantic_region_id != self._last_semantic_region_id:
            metadata.update(
                {
                    "anchor_change_kind": "semantic_region",
                    "anchor_change_from": self._last_semantic_region_id,
                    "anchor_change_to": semantic_region_id,
                }
            )
            if has_pose_context and pose_is_static:
                metadata["stationary_suppressed"] = True
                return KeyframeDecision(False, "stationary_anchor_hold", metadata)
            return KeyframeDecision(True, "semantic_region_changed", metadata)
        if topo_node_id and topo_node_id != self._last_topo_node_id:
            metadata.update(
                {
                    "anchor_change_kind": "topology_node",
                    "anchor_change_from": self._last_topo_node_id,
                    "anchor_change_to": topo_node_id,
                }
            )
            if has_pose_context and pose_is_static:
                metadata["stationary_suppressed"] = True
                return KeyframeDecision(False, "stationary_anchor_hold", metadata)
            return KeyframeDecision(True, "topology_node_changed", metadata)

        return KeyframeDecision(False, "no_significant_change", metadata)

    def mark_processed(
        self,
        *,
        timestamp: datetime,
        current_pose: Optional[Pose],
        map_context: Optional[SemanticMapBuildResult],
    ) -> None:
        """记录最近一次关键帧。"""

        anchor_metadata = self._resolve_anchor_metadata(current_pose=current_pose, map_context=map_context)
        self._last_keyframe_at = timestamp
        self._last_keyframe_pose = current_pose
        self._last_semantic_region_id = str(anchor_metadata.get("semantic_region_id") or "") or None
        self._last_topo_node_id = str(anchor_metadata.get("topo_node_id") or "") or None

    def _resolve_anchor_metadata(
        self,
        *,
        current_pose: Optional[Pose],
        map_context: Optional[SemanticMapBuildResult],
    ) -> Dict[str, object]:
        metadata: Dict[str, object] = {
            "semantic_region_id": None,
            "topo_node_id": None,
            "map_version_id": map_context.map_version_id if map_context is not None else None,
        }
        if current_pose is None or map_context is None:
            return metadata

        best_region_id = None
        best_region_distance = None
        for region in map_context.semantic_regions:
            if region.centroid is None:
                continue
            distance_m = self._distance(current_pose, region.centroid)
            if best_region_distance is None or distance_m < best_region_distance:
                best_region_distance = distance_m
                best_region_id = region.region_id

        best_topo_node_id = None
        best_topo_distance = None
        for node_id, node in map_context.topology_nodes_by_id.items():
            try:
                node_x = float(node.get("x"))
                node_y = float(node.get("y"))
                node_z = float(node.get("z", 0.0))
            except (TypeError, ValueError):
                continue
            dx = float(current_pose.position.x) - node_x
            dy = float(current_pose.position.y) - node_y
            dz = float(current_pose.position.z) - node_z
            distance_m = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
            if best_topo_distance is None or distance_m < best_topo_distance:
                best_topo_distance = distance_m
                best_topo_node_id = node_id

        metadata["semantic_region_id"] = best_region_id
        metadata["topo_node_id"] = best_topo_node_id
        return metadata

    def _distance(self, left: Pose, right: Pose) -> float:
        dx = float(left.position.x) - float(right.position.x)
        dy = float(left.position.y) - float(right.position.y)
        dz = float(left.position.z) - float(right.position.z)
        return math.sqrt((dx * dx) + (dy * dy) + (dz * dz))

    def _yaw_delta(self, left: Pose, right: Pose) -> float:
        left_yaw = self._yaw_from_pose(left)
        right_yaw = self._yaw_from_pose(right)
        delta = left_yaw - right_yaw
        while delta > math.pi:
            delta -= 2.0 * math.pi
        while delta < -math.pi:
            delta += 2.0 * math.pi
        return abs(delta)

    def _yaw_from_pose(self, pose: Pose) -> float:
        orientation = pose.orientation
        siny_cosp = 2.0 * ((orientation.w * orientation.z) + (orientation.x * orientation.y))
        cosy_cosp = 1.0 - (2.0 * ((orientation.y * orientation.y) + (orientation.z * orientation.z)))
        return math.atan2(siny_cosp, cosy_cosp)
