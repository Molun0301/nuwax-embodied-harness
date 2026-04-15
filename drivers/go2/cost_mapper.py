from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from contracts.geometry import Pose, Quaternion, Vector3
from contracts.maps import CostMap, OccupancyGrid

try:
    from numba import njit
    _HAS_NUMBA = True
except ImportError:
    _HAS_NUMBA = False

    def njit(cache=False):
        def decorator(func):
            return func
        return decorator


@dataclass
class Go2CostMapperConfig:
    enabled: bool = True
    resolution_m: float = 0.05
    width: int = 200
    height: int = 200
    frame_id: str = "odom"
    min_obstacle_height_m: float = 0.05
    max_obstacle_height_m: float = 1.20
    traversability_threshold: float = 0.65
    lethal_threshold: float = 0.20
    inflation_radius_m: float = 0.05
    height_variance_weight: float = 50.0
    height_gradient_weight: float = 30.0


@dataclass
class Go2CostMapperResult:
    occupancy_grid: OccupancyGrid
    cost_map: CostMap
    traversability_grid: np.ndarray


class Go2CostMapper:
    """基于高度分析的2D代价地图计算器。"""

    def __init__(self, config: Go2CostMapperConfig) -> None:
        self.config = config
        self._min_height_map: Optional[np.ndarray] = None
        self._max_height_map: Optional[np.ndarray] = None
        self._height_sum_map: Optional[np.ndarray] = None
        self._count_map: Optional[np.ndarray] = None
        self._origin_x: float = 0.0
        self._origin_y: float = 0.0
        self._initialized: bool = False
        self._map_id_counter: int = 0

    def _ensure_initialized(self, sensor_x: float, sensor_y: float) -> None:
        if self._initialized:
            return
        height, width = int(self.config.height), int(self.config.width)
        self._min_height_map = np.full((height, width), np.nan, dtype=np.float32)
        self._max_height_map = np.full((height, width), np.nan, dtype=np.float32)
        self._height_sum_map = np.zeros((height, width), dtype=np.float32)
        self._count_map = np.zeros((height, width), dtype=np.int32)
        self._origin_x = sensor_x - (width / 2.0) * self.config.resolution_m
        self._origin_y = sensor_y - (height / 2.0) * self.config.resolution_m
        self._initialized = True

    def process_pointcloud(
        self,
        points_xyz: np.ndarray,
        sensor_x: float,
        sensor_y: float,
    ) -> Optional[Go2CostMapperResult]:
        if not bool(self.config.enabled):
            return None
        if points_xyz.size == 0:
            return None

        self._ensure_initialized(sensor_x, sensor_y)

        self._update_height_maps(points_xyz)

        cost_grid = self._compute_cost_grid()

        traversability_grid = self._compute_traversability(cost_grid)

        occupancy_data = self._compute_occupancy_data(traversability_grid)

        self._map_id_counter += 1
        map_id = f"go2_cost_map_{self._map_id_counter:06d}"
        origin_pose = Pose(
            frame_id=self.config.frame_id,
            position=Vector3(x=self._origin_x, y=self._origin_y, z=0.0),
            orientation=Quaternion(w=1.0),
        )

        occupancy_grid = OccupancyGrid(
            map_id=map_id,
            frame_id=self.config.frame_id,
            width=int(self.config.width),
            height=int(self.config.height),
            resolution_m=float(self.config.resolution_m),
            origin=origin_pose,
            data=occupancy_data,
        )

        cost_map = CostMap(
            map_id=map_id,
            frame_id=self.config.frame_id,
            width=int(self.config.width),
            height=int(self.config.height),
            resolution_m=float(self.config.resolution_m),
            origin=origin_pose,
            data=cost_grid.flatten().tolist(),
        )

        return Go2CostMapperResult(
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
            traversability_grid=traversability_grid,
        )

    def _update_height_maps(self, points_xyz: np.ndarray) -> None:
        if _HAS_NUMBA and points_xyz.shape[0] > 100:
            self._update_height_maps_numba(
                points_xyz,
                self._min_height_map,
                self._max_height_map,
                self._height_sum_map,
                self._count_map,
                self._origin_x,
                self._origin_y,
                1.0 / self.config.resolution_m,
                int(self.config.width),
                int(self.config.height),
                float(self.config.min_obstacle_height_m),
                float(self.config.max_obstacle_height_m),
            )
        else:
            self._update_height_maps_fallback(points_xyz)

    @staticmethod
    @njit(cache=True)
    def _update_height_maps_numba(
        points_xyz: np.ndarray,
        min_height_map: np.ndarray,
        max_height_map: np.ndarray,
        height_sum_map: np.ndarray,
        count_map: np.ndarray,
        origin_x: float,
        origin_y: float,
        inv_res: float,
        width: int,
        height: int,
        min_h: float,
        max_h: float,
    ) -> None:
        n = points_xyz.shape[0]
        for i in range(n):
            x = float(points_xyz[i, 0])
            y = float(points_xyz[i, 1])
            z = float(points_xyz[i, 2])

            if z < min_h or z > max_h:
                continue

            gx = int((x - origin_x) * inv_res + 0.5)
            gy = int((y - origin_y) * inv_res + 0.5)

            if 0 <= gx < width and 0 <= gy < height:
                if np.isnan(min_height_map[gy, gx]):
                    min_height_map[gy, gx] = z
                    max_height_map[gy, gx] = z
                    height_sum_map[gy, gx] = z
                else:
                    if z < min_height_map[gy, gx]:
                        min_height_map[gy, gx] = z
                    if z > max_height_map[gy, gx]:
                        max_height_map[gy, gx] = z
                    height_sum_map[gy, gx] += z
                count_map[gy, gx] += 1

    def _update_height_maps_fallback(self, points_xyz: np.ndarray) -> None:
        for i in range(points_xyz.shape[0]):
            x, y, z = points_xyz[i]
            if z < self.config.min_obstacle_height_m or z > self.config.max_obstacle_height_m:
                continue
            gx = int((x - self._origin_x) / self.config.resolution_m + 0.5)
            gy = int((y - self._origin_y) / self.config.resolution_m + 0.5)
            if 0 <= gx < self.config.width and 0 <= gy < self.config.height:
                if np.isnan(self._min_height_map[gy, gx]):
                    self._min_height_map[gy, gx] = z
                    self._max_height_map[gy, gx] = z
                    self._height_sum_map[gy, gx] = z
                else:
                    if z < self._min_height_map[gy, gx]:
                        self._min_height_map[gy, gx] = z
                    if z > self._max_height_map[gy, gx]:
                        self._max_height_map[gy, gx] = z
                    self._height_sum_map[gy, gx] += z
                self._count_map[gy, gx] += 1

    def _compute_cost_grid(self) -> np.ndarray:
        cost_grid = np.zeros((int(self.config.height), int(self.config.width)), dtype=np.float32)
        height_range = self.config.max_obstacle_height_m - self.config.min_obstacle_height_m

        has_data_mask = ~np.isnan(self._min_height_map) & (self._count_map > 0)

        if np.any(has_data_mask):
            height_diff = self._max_height_map - self._min_height_map
            cost_grid[has_data_mask] = np.clip(
                height_diff[has_data_mask] / height_range * self.config.height_variance_weight,
                0.0,
                100.0,
            )

        cost_grid = self._inflate_costmap(cost_grid)

        return cost_grid

    def _inflate_costmap(self, cost_grid: np.ndarray) -> np.ndarray:
        if self.config.inflation_radius_m <= 0:
            return cost_grid

        radius_cells = max(1, int(np.ceil(self.config.inflation_radius_m / self.config.resolution_m)))

        from scipy import ndimage
        structure = np.ones((2 * radius_cells + 1, 2 * radius_cells + 1), dtype=np.float32)
        inflated = ndimage.maximum_filter(cost_grid, footprint=structure)

        return inflated

    def _compute_traversability(self, cost_grid: np.ndarray) -> np.ndarray:
        return np.clip(1.0 - cost_grid / 100.0, 0.0, 1.0)

    def _compute_occupancy_data(self, traversability_grid: np.ndarray) -> list:
        occupancy_data = np.full((int(self.config.height), int(self.config.width)), -1, dtype=np.int8)
        occupancy_data[traversability_grid >= self.config.traversability_threshold] = 0
        occupancy_data[traversability_grid < self.config.lethal_threshold] = 100
        return occupancy_data.flatten().tolist()

    def reset(self) -> None:
        self._min_height_map = None
        self._max_height_map = None
        self._height_sum_map = None
        self._count_map = None
        self._initialized = False

    def get_traversability_at(self, x: float, y: float) -> Optional[float]:
        if not self._initialized:
            return None

        gx = int((x - self._origin_x) / self.config.resolution_m + 0.5)
        gy = int((y - self._origin_y) / self.config.resolution_m + 0.5)

        if 0 <= gx < self.config.width and 0 <= gy < self.config.height:
            if self._count_map is not None and self._count_map[gy, gx] > 0:
                return float(1.0 - self._min_height_map[gy, gx] / 100.0)
        return None
