from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from services.calibration.config import (
    get_calibration_min_frames,
    get_calibration_static_angular_threshold,
    get_calibration_static_velocity_threshold,
)


LOGGER = logging.getLogger(__name__)


@dataclass
class FramePair:
    """一对时间同步的点云-图像帧。"""

    image_bytes: bytes
    image_width_px: int
    image_height_px: int
    point_cloud_xyz: np.ndarray  # shape (N, 3)
    point_cloud_frame_id: str
    timestamp_ns: int
    depth_range_m: Tuple[float, float]  # (min_depth, max_depth)
    metadata: Dict[str, object] = field(default_factory=dict)


class StaticDetector:
    """静止检测器：基于线速度和角速度判断机器人是否处于静止状态。"""

    def __init__(
        self,
        *,
        linear_velocity_threshold: float = 0.02,
        angular_velocity_threshold: float = 0.05,
        stable_duration_sec: float = 0.5,
    ) -> None:
        self._linear_threshold = linear_velocity_threshold
        self._angular_threshold = angular_velocity_threshold
        self._stable_duration_sec = stable_duration_sec
        self._last_motion_time: float = time.time()
        self._is_static: bool = False

    def update(self, linear_velocity_mps: float, angular_velocity_rps: float) -> bool:
        """更新运动状态，返回是否静止。"""
        now = time.time()
        if (
            linear_velocity_mps > self._linear_threshold
            or angular_velocity_rps > self._angular_threshold
        ):
            self._last_motion_time = now
            self._is_static = False
        elif now - self._last_motion_time >= self._stable_duration_sec:
            self._is_static = True
        return self._is_static

    @property
    def is_static(self) -> bool:
        return self._is_static


class CalibrationDataCollector:
    """标定数据采集器：仅在机器人静止时采集时间同步的点云-图像帧对。"""

    def __init__(
        self,
        *,
        min_frames: int = 0,
        max_frames: int = 50,
        min_point_count: int = 500,
        diversity_distance_m: float = 0.5,
    ) -> None:
        self._min_frames = min_frames or get_calibration_min_frames()
        self._max_frames = max_frames
        self._min_point_count = min_point_count
        self._diversity_distance_m = diversity_distance_m
        self._static_detector = StaticDetector(
            linear_velocity_threshold=get_calibration_static_velocity_threshold(),
            angular_velocity_threshold=get_calibration_static_angular_threshold(),
        )
        self._collected_pairs: List[FramePair] = []
        self._ready = False

    @property
    def is_ready(self) -> bool:
        """是否已采集足够帧对。"""
        return self._ready

    @property
    def collected_count(self) -> int:
        return len(self._collected_pairs)

    @property
    def required_count(self) -> int:
        return self._min_frames

    def get_collected_pairs(self) -> List[FramePair]:
        """获取已采集的帧对（只读副本）。"""
        return list(self._collected_pairs)

    def update_motion_state(
        self, linear_velocity_mps: float, angular_velocity_rps: float
    ) -> None:
        """更新机器人运动状态。"""
        self._static_detector.update(linear_velocity_mps, angular_velocity_rps)

    def try_collect(
        self,
        *,
        image_bytes: bytes,
        image_width_px: int,
        image_height_px: int,
        point_cloud_xyz: np.ndarray,
        point_cloud_frame_id: str,
        timestamp_ns: int,
    ) -> bool:
        """尝试采集一对帧。返回是否成功采集。"""
        if self._ready:
            return False
        if not self._static_detector.is_static:
            return False
        if point_cloud_xyz.shape[0] < self._min_point_count:
            LOGGER.debug(
                "点云密度不足: %d < %d", point_cloud_xyz.shape[0], self._min_point_count
            )
            return False
        if len(self._collected_pairs) >= self._max_frames:
            self._ready = True
            return False

        # 计算深度范围
        distances = np.linalg.norm(point_cloud_xyz, axis=1)
        valid_mask = distances > 0.1
        if not np.any(valid_mask):
            return False
        valid_distances = distances[valid_mask]
        depth_range = (float(np.min(valid_distances)), float(np.max(valid_distances)))

        # 场景多样性检查（基于深度分布）
        if not self._check_diversity(depth_range):
            return False

        pair = FramePair(
            image_bytes=image_bytes,
            image_width_px=image_width_px,
            image_height_px=image_height_px,
            point_cloud_xyz=point_cloud_xyz,
            point_cloud_frame_id=point_cloud_frame_id,
            timestamp_ns=timestamp_ns,
            depth_range_m=depth_range,
        )
        self._collected_pairs.append(pair)
        LOGGER.info(
            "标定帧对已采集: %d/%d", len(self._collected_pairs), self._min_frames
        )

        if len(self._collected_pairs) >= self._min_frames:
            self._ready = True
            LOGGER.info(
                "标定数据采集完成，共 %d 帧对。", len(self._collected_pairs)
            )

        return True

    def reset(self) -> None:
        """重置采集状态。"""
        self._collected_pairs.clear()
        self._ready = False

    def _check_diversity(self, depth_range: Tuple[float, float]) -> bool:
        """检查新帧对是否与已有帧对有足够差异。"""
        if not self._collected_pairs:
            return True
        new_center = (depth_range[0] + depth_range[1]) / 2.0
        for existing in self._collected_pairs[-5:]:
            existing_center = (
                existing.depth_range_m[0] + existing.depth_range_m[1]
            ) / 2.0
            if abs(new_center - existing_center) < self._diversity_distance_m:
                # 前10帧宽松采集，之后严格要求多样性
                if len(self._collected_pairs) > 10:
                    return False
        return True
