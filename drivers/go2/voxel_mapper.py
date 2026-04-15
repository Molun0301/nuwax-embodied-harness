from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Tuple

import numpy as np

try:
    import open3d as o3d
    import open3d.core as o3c
    _HAS_OPEN3D = o3c.cuda.is_available() if hasattr(o3d, 'core') else False
except ImportError:
    _HAS_OPEN3D = False

from contracts.geometry import Pose, Quaternion, Vector3


class VoxelCellStatus(IntEnum):
    UNKNOWN = 0
    FREE = 1
    OCCUPIED = 2


@dataclass
class Go2VoxelMapperConfig:
    enabled: bool = True
    voxel_size: float = 0.05
    block_count: int = 2_000_000
    device: str = "CUDA:0"
    carve_columns: bool = True
    frame_id: str = "odom"
    publish_interval_sec: float = 0.5
    min_z_m: float = -0.20
    max_z_m: float = 1.20


@dataclass
class Go2VoxelMapperResult:
    points_xyz: np.ndarray
    voxel_count: int
    timestamp: float


class Go2VoxelMapper:
    """带列雕刻的体素地图构建器。

    参考 dimos VoxelGridMapper，实现：
    1. Open3D VoxelBlockGrid 存储
    2. 列雕刻：动态物体离开后自动清除残影
    3. GPU/CPU 自适应
    """

    def __init__(self, config: Go2VoxelMapperConfig) -> None:
        self.config = config
        self._voxel_block_grid: Optional[object] = None
        self._hashmap: Optional[object] = None
        self._key_dtype: Optional[object] = None
        self._dev: Optional[object] = None
        self._voxel_hashmap: Optional[object] = None
        self._latest_frame_ts: float = 0.0
        self._initialized: bool = False
        self._frame_count: int = 0
        self._fallback_points_xyz: Optional[np.ndarray] = None

    def _ensure_initialized(self) -> None:
        if self._initialized:
            return

        if not _HAS_OPEN3D:
            self._voxel_block_grid = None
            self._voxel_hashmap = None
            self._initialized = True
            return

        dev_str = str(self.config.device)
        if dev_str.startswith("CUDA") and not o3c.cuda.is_available():
            dev_str = "CPU:0"

        self._dev = o3c.Device(dev_str)

        self._voxel_block_grid = o3d.t.geometry.VoxelBlockGrid(
            attr_names=("dummy",),
            attr_dtypes=(o3c.uint8,),
            attr_channels=(o3c.SizeVector([1]),),
            voxel_size=float(self.config.voxel_size),
            block_resolution=1,
            block_count=int(self.config.block_count),
            device=self._dev,
        )

        self._voxel_hashmap = self._voxel_block_grid.hashmap()
        self._key_dtype = self._voxel_hashmap.key_tensor().dtype
        self._initialized = True

    def add_frame(
        self,
        points_xyz: np.ndarray,
        timestamp: float = 0.0,
    ) -> Optional[Go2VoxelMapperResult]:
        if not bool(self.config.enabled):
            return None

        self._ensure_initialized()

        if points_xyz.size == 0:
            return None

        self._latest_frame_ts = timestamp
        self._frame_count += 1

        filtered_mask = (
            (points_xyz[:, 2] >= self.config.min_z_m) &
            (points_xyz[:, 2] <= self.config.max_z_m)
        )
        filtered_points = points_xyz[filtered_mask]

        if filtered_points.shape[0] == 0:
            return Go2VoxelMapperResult(
                points_xyz=points_xyz,
                voxel_count=0,
                timestamp=timestamp,
            )

        if self._voxel_block_grid is None:
            self._update_fallback_pointcloud(filtered_points)
            return Go2VoxelMapperResult(
                points_xyz=filtered_points,
                voxel_count=self.get_voxel_count(),
                timestamp=timestamp,
            )

        if bool(self.config.carve_columns):
            self._carve_and_insert(filtered_points)
        else:
            self._simple_insert(filtered_points)

        return Go2VoxelMapperResult(
            points_xyz=filtered_points,
            voxel_count=int(self._voxel_hashmap.size()) if self._voxel_hashmap else 0,
            timestamp=timestamp,
        )

    def _carve_and_insert(self, new_points: np.ndarray) -> None:
        if self._voxel_block_grid is None or self._voxel_hashmap is None:
            return

        voxel_size = float(self.config.voxel_size)
        dev = self._dev

        pts = np.asarray(new_points[:, :3], dtype=np.float32)
        pts_tensor = o3c.Tensor.from_numpy(pts).to(dev, o3c.float32)
        keys = (pts_tensor / voxel_size).floor().to(self._key_dtype)
        keys_Nx3 = keys.contiguous()

        xy_keys = keys_Nx3[:, :2].contiguous()

        xy_hashmap = o3c.HashMap(
            init_capacity=xy_keys.shape[0],
            key_dtype=self._key_dtype,
            key_element_shape=o3c.SizeVector([2]),
            value_dtypes=[o3c.uint8],
            value_element_shapes=[o3c.SizeVector([1])],
            device=dev,
        )
        dummy_vals = o3c.Tensor.zeros((xy_keys.shape[0], 1), o3c.uint8, dev)
        xy_hashmap.insert(xy_keys, dummy_vals)

        active_indices = self._voxel_hashmap.active_buf_indices()
        if active_indices.shape[0] == 0:
            self._voxel_hashmap.activate(keys_Nx3)
            return

        existing_keys = self._voxel_hashmap.key_tensor()[active_indices]
        existing_xy = existing_keys[:, :2].contiguous()

        _, found_mask = xy_hashmap.find(existing_xy)

        to_erase = existing_keys[found_mask]
        if to_erase.shape[0] > 0:
            self._voxel_hashmap.erase(to_erase)

        self._voxel_hashmap.activate(keys_Nx3)

    def _simple_insert(self, new_points: np.ndarray) -> None:
        if self._voxel_block_grid is None or self._voxel_hashmap is None:
            return

        voxel_size = float(self.config.voxel_size)
        dev = self._dev

        pts = np.asarray(new_points[:, :3], dtype=np.float32)
        pts_tensor = o3c.Tensor.from_numpy(pts).to(dev, o3c.float32)
        keys = (pts_tensor / voxel_size).floor().to(self._key_dtype)
        keys_Nx3 = keys.contiguous()

        self._voxel_hashmap.activate(keys_Nx3)

    def get_global_pointcloud(self) -> Optional[np.ndarray]:
        if self._voxel_block_grid is None:
            if self._fallback_points_xyz is None:
                return None
            return np.asarray(self._fallback_points_xyz, dtype=np.float32).copy()

        try:
            pcd = self._voxel_block_grid.get_point_cloud()
            if pcd.is_empty():
                return None
            points = pcd.point["positions"].numpy()
            return points
        except Exception:
            return None

    def get_voxel_count(self) -> int:
        if self._voxel_hashmap is None:
            return int(self._fallback_points_xyz.shape[0]) if self._fallback_points_xyz is not None else 0
        return int(self._voxel_hashmap.size())

    def has_data(self) -> bool:
        return self.get_voxel_count() > 0

    def restore_pointcloud(
        self,
        points_xyz: np.ndarray,
        timestamp: float = 0.0,
    ) -> Optional[Go2VoxelMapperResult]:
        """从持久化点云恢复体素地图。"""

        self.reset()
        return self.add_frame(points_xyz, timestamp=timestamp)

    def reset(self) -> None:
        if self._initialized and _HAS_OPEN3D:
            self._voxel_block_grid = None
            self._voxel_hashmap = None
        self._initialized = False
        self._frame_count = 0
        self._latest_frame_ts = 0.0
        self._fallback_points_xyz = None

    def _update_fallback_pointcloud(self, new_points: np.ndarray) -> None:
        """在无 Open3D 环境下，使用量化点云维持最小体素语义。"""

        voxel_size = max(float(self.config.voxel_size), 1e-6)
        point_by_key = {}

        if self._fallback_points_xyz is not None and self._fallback_points_xyz.size > 0:
            existing_points = np.asarray(self._fallback_points_xyz[:, :3], dtype=np.float32)
            existing_keys = np.floor(existing_points / voxel_size).astype(np.int64)
            active_xy_keys = set()
            if bool(self.config.carve_columns):
                new_xy_keys = np.floor(np.asarray(new_points[:, :3], dtype=np.float32) / voxel_size).astype(np.int64)
                active_xy_keys = {(int(key[0]), int(key[1])) for key in new_xy_keys.tolist()}
            for key, point in zip(existing_keys.tolist(), existing_points.tolist()):
                if active_xy_keys and (int(key[0]), int(key[1])) in active_xy_keys:
                    continue
                point_by_key[(int(key[0]), int(key[1]), int(key[2]))] = point

        new_points_xyz = np.asarray(new_points[:, :3], dtype=np.float32)
        new_keys = np.floor(new_points_xyz / voxel_size).astype(np.int64)
        for key, point in zip(new_keys.tolist(), new_points_xyz.tolist()):
            point_by_key[(int(key[0]), int(key[1]), int(key[2]))] = point

        if not point_by_key:
            self._fallback_points_xyz = None
            return
        self._fallback_points_xyz = np.asarray(list(point_by_key.values()), dtype=np.float32)

    def to_occupancy_grid(
        self,
        resolution: float = 0.10,
        width: int = 200,
        height: int = 200,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
    ) -> Optional[np.ndarray]:
        pcd_points = self.get_global_pointcloud()
        if pcd_points is None or pcd_points.shape[0] == 0:
            return None

        occupancy = np.full((height, width), -1, dtype=np.int8)

        for i in range(pcd_points.shape[0]):
            x, y, z = pcd_points[i]
            if z < self.config.min_z_m or z > self.config.max_z_m:
                continue

            gx = int((x - origin_x) / resolution + 0.5)
            gy = int((y - origin_y) / resolution + 0.5)

            if 0 <= gx < width and 0 <= gy < height:
                if z > 0.05:
                    occupancy[gy, gx] = 100
                elif occupancy[gy, gx] == -1:
                    occupancy[gy, gx] = 0

        return occupancy
