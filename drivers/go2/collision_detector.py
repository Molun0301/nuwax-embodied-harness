from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Tuple

import numpy as np

from contracts.geometry import Pose, Vector3
from contracts.maps import CostMap

try:
    import numba
    _HAS_NUMBA = True
except ImportError:
    _HAS_NUMBA = False


class CollisionStatus(IntEnum):
    SAFE = 0
    WARNING = 1
    DANGER = 2


@dataclass
class Go2CollisionDetectorConfig:
    enabled: bool = True
    safety_radius_m: float = 0.3
    warning_distance_m: float = 0.6
    danger_distance_m: float = 0.3
    emergency_stop_timeout_sec: float = 0.1
    min_lidar_range_m: float = 0.05
    max_lidar_range_m: float = 25.0
    front_arc_degrees: float = 120.0


class Go2CollisionDetector:
    """基于激光雷达的实时碰撞检测器。"""

    def __init__(self, config: Go2CollisionDetectorConfig) -> None:
        self.config = config
        self._last_scan_ranges: Optional[np.ndarray] = None
        self._last_scan_yaw_rad: float = 0.0

    def update_scan(self, ranges: np.ndarray, yaw_rad: float = 0.0) -> None:
        self._last_scan_ranges = np.asarray(ranges, dtype=np.float32)
        self._last_scan_yaw_rad = yaw_rad

    def check_collision(
        self,
        current_pose: Pose,
        scan_ranges: Optional[np.ndarray] = None,
    ) -> CollisionStatus:
        if not bool(self.config.enabled):
            return CollisionStatus.SAFE

        if scan_ranges is None:
            if self._last_scan_ranges is None:
                return CollisionStatus.SAFE
            scan_ranges = self._last_scan_ranges

        valid_mask = (scan_ranges >= self.config.min_lidar_range_m) & (
            scan_ranges <= self.config.max_lidar_range_m
        )
        if not np.any(valid_mask):
            return CollisionStatus.SAFE

        valid_ranges = scan_ranges[valid_mask]
        min_distance = float(np.min(valid_ranges))

        if min_distance <= self.config.danger_distance_m:
            return CollisionStatus.DANGER
        elif min_distance <= self.config.warning_distance_m:
            return CollisionStatus.WARNING

        return CollisionStatus.SAFE

    def compute_emergency_stop(
        self,
        current_pose: Pose,
        scan_ranges: Optional[np.ndarray] = None,
    ) -> Tuple[float, float]:
        status = self.check_collision(current_pose, scan_ranges)
        if status == CollisionStatus.DANGER:
            return (0.0, 0.0)
        elif status == CollisionStatus.WARNING:
            return (0.1, 0.0)
        return None

    def get_obstacle_direction(
        self,
        scan_ranges: Optional[np.ndarray] = None,
    ) -> Optional[float]:
        if scan_ranges is None:
            if self._last_scan_ranges is None:
                return None
            scan_ranges = self._last_scan_ranges

        valid_mask = (scan_ranges >= self.config.min_lidar_range_m) & (
            scan_ranges <= self.config.max_lidar_range_m
        )
        if not np.any(valid_mask):
            return None

        danger_mask = valid_mask & (scan_ranges <= self.config.warning_distance_m)
        if not np.any(danger_mask):
            return None

        angles = np.linspace(
            -float(self.config.front_arc_degrees) / 2.0,
            float(self.config.front_arc_degrees) / 2.0,
            len(scan_ranges),
        )
        danger_angles = angles[danger_mask]
        danger_ranges = scan_ranges[danger_mask]

        if len(danger_angles) == 0:
            return None

        weights = 1.0 / (danger_ranges + 1e-6)
        weighted_angle = np.sum(danger_angles * weights) / np.sum(weights)
        return float(weighted_angle) + self._last_scan_yaw_rad
