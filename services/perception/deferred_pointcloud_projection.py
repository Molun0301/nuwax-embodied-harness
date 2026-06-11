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
    points = np.asarray(points_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] == 0:
        return None, 0

    origin_arr = np.asarray(origin, dtype=np.float64)
    dir_arr = np.asarray(direction, dtype=np.float64)
    dir_norm = float(np.linalg.norm(dir_arr))
    if dir_norm < 1e-9:
        return None, 0
    dir_arr = dir_arr / dir_norm

    half_angle = float(max(1e-4, half_angle_rad))
    cos_half = math.cos(half_angle)

    # 各点相对锥顶的向量与距离
    rel = points - origin_arr
    ranges = np.linalg.norm(rel, axis=1)
    valid_range = (ranges >= float(min_range_m)) & (ranges <= float(max_range_m))
    if not bool(np.any(valid_range)):
        return None, 0

    # 余弦判据：点向量与轴向夹角 ≤ half_angle ⟺ cos(angle) ≥ cos(half_angle)
    safe_ranges = np.where(ranges > 1e-9, ranges, 1.0)
    cos_angles = (rel @ dir_arr) / safe_ranges
    in_cone = valid_range & (cos_angles >= cos_half)
    count = int(np.count_nonzero(in_cone))
    if count < max(1, int(min_points)):
        return None, count

    selected = points[in_cone]
    centroid = selected.mean(axis=0)
    mean_range = float(np.mean(ranges[in_cone]))
    return (
        ConeProjectionResult(
            centroid=(float(centroid[0]), float(centroid[1]), float(centroid[2])),
            point_count=count,
            mean_range_m=mean_range,
        ),
        count,
    )
