"""延迟点云重投影(射线视锥 cone 粗筛)。

纯几何模块：给定累积点云(map frame)与一次历史观测的视锥(光心 origin、
轴向 direction、半角 half_angle)，选出落在锥内、距离窗内的点，取质心作为
对象坐标。

为何用 cone 而非完整像素重投影：deferred_projection_cache 只可靠保存了 bbox
中心射线(origin+direction)与 bbox 角张成的半角，缺相机完整旋转矩阵与逐像素
内参映射。cone 粗筛只需点积判据(廉价、稳健)，等同方案 §3.2 的"廉价 frustum
体检"，对 L2 地面物品(强激光覆盖区)有真实价值。

两阶段 fast-fail：
1. 锥内点计数 < min_points → 返回 (None, count)，标 insufficient，由调用方退避重试。
2. count 足够 → 取锥内点质心(可选距离窗剔背景)。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class ConeProjectionResult:
    """视锥重投影输出。"""

    centroid: Tuple[float, float, float]
    point_count: int
    mean_range_m: float


def project_cone(
    points_xyz: np.ndarray,
    *,
    origin: Tuple[float, float, float],
    direction: Tuple[float, float, float],
    half_angle_rad: float,
    min_points: int = 5,
    max_range_m: float = 8.0,
    min_range_m: float = 0.05,
) -> Tuple[Optional[ConeProjectionResult], int]:
    """对累积点云做视锥锥内取点 + 质心估计。

    参数:
        points_xyz: N×3 累积点云(map frame，float)。
        origin: 视锥锥顶(相机光心，map frame)。
        direction: 视锥轴向单位向量(map frame)。
        half_angle_rad: 视锥半角(弧度)。
        min_points: 锥内最少点数，不足则 fast-fail。
        max_range_m: 距锥顶最大距离(剔背景/远处噪声)。
        min_range_m: 距锥顶最小距离(剔贴近相机的伪点)。

    返回:
        (ConeProjectionResult 或 None, 锥内点计数)。
        点数不足时返回 (None, count)，count 供调用方判退避。
    """

    if points_xyz is None:
        return None, 0
    points = np.asarray(points_xyz)
    if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] == 0:
        return None, 0
    origin_arr = np.asarray(origin, dtype=np.float64)
    dir_arr = np.asarray(direction, dtype=np.float64)
    if origin_arr.shape != (3,) or dir_arr.shape != (3,):
        return None, 0
    if not np.isfinite(origin_arr).all() or not np.isfinite(dir_arr).all():
        return None, 0
    dir_norm = float(np.linalg.norm(dir_arr))
    if not math.isfinite(dir_norm) or dir_norm < 1e-9:
        return None, 0
    dir_arr = dir_arr / dir_norm
    half_angle = float(half_angle_rad)
    if not math.isfinite(half_angle) or not 0.0 <= half_angle < math.pi / 2:
        return None, 0
    if not (math.isfinite(min_range_m) and math.isfinite(max_range_m)
            and 0.0 <= min_range_m < max_range_m):
        return None, 0
    cos_squared = math.cos(max(1e-4, half_angle)) ** 2
    count = 0
    point_sum = np.zeros(3, dtype=np.float64)
    range_sum = 0.0
    # 分块限定临时内存；平方距离筛选避免对整图开方和多线程矩阵调度。
    for start in range(0, len(points), 65536):
        block = np.asarray(points[start:start + 65536], dtype=np.float64)
        rel = block - origin_arr
        squared_ranges = np.einsum("ij,ij->i", rel, rel)
        forward = np.einsum("ij,j->i", rel, dir_arr)
        in_cone = (np.isfinite(squared_ranges) & (squared_ranges >= min_range_m ** 2)
                   & (squared_ranges <= max_range_m ** 2) & (forward > 0.0)
                   & (forward * forward >= cos_squared * squared_ranges))
        selected_count = int(np.count_nonzero(in_cone))
        if selected_count:
            count += selected_count
            point_sum += block[in_cone].sum(axis=0)
            range_sum += float(np.sqrt(squared_ranges[in_cone]).sum())
    if count < max(1, int(min_points)):
        return None, count
    centroid = point_sum / count
    return (
        ConeProjectionResult(
            centroid=(float(centroid[0]), float(centroid[1]), float(centroid[2])),
            point_count=count,
            mean_range_m=range_sum / count,
        ),
        count,
    )
