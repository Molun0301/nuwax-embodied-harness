"""多视角射线三角化(midpoint / 最小二乘)。

纯几何模块：给定多条 map frame 射线(光心 + 单位方向)，求到所有射线
距离平方和最小的最近交点。不依赖点云，绕开 L2 稀疏问题。

数学：min_x  Σ_i ‖(I − d_i d_iᵀ)(x − o_i)‖²
法方程：A = Σ_i (I − d_i d_iᵀ)，b = Σ_i (I − d_i d_iᵀ) o_i，x = A⁻¹ b

退化处理：射线近平行/单视角 → A 近奇异 → 返回 None。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class TriangulationResult:
    """三角化输出。"""

    point: Tuple[float, float, float]
    parallax_deg: float
    residual_m: float
    ray_count: int


def _normalize_rows(matrix: np.ndarray) -> Optional[np.ndarray]:
    norms = np.linalg.norm(matrix, axis=1, keepdims=True)
    if not np.all(norms > 1e-9):
        return None
    return matrix / norms


def _max_parallax_deg(directions: np.ndarray) -> float:
    """方向向量两两夹角的最大值(度)。"""

    count = directions.shape[0]
    if count < 2:
        return 0.0
    max_angle = 0.0
    for i in range(count):
        for j in range(i + 1, count):
            dot = float(np.clip(np.dot(directions[i], directions[j]), -1.0, 1.0))
            angle = float(np.degrees(np.arccos(abs(dot))))
            # 取夹角(0~90)，越大视差越好
            max_angle = max(max_angle, min(angle, 180.0 - angle) if angle > 90.0 else angle)
    return max_angle


def triangulate_rays(
    origins: Sequence[Sequence[float]],
    directions: Sequence[Sequence[float]],
    *,
    min_rays: int = 2,
    min_parallax_deg: float = 5.0,
    max_residual_m: float = 0.5,
) -> Optional[TriangulationResult]:
    """对多条射线做最小二乘三角化。

    参数:
        origins: N×3 射线光心(map frame)。
        directions: N×3 射线方向(map frame，内部会归一化)。
        min_rays: 最少射线数。
        min_parallax_deg: 最小视差角(度)，低于则视为退化返回 None。
        max_residual_m: 最大残差(米)，超过则返回 None。

    返回:
        TriangulationResult 或 None(射线不足/视差不足/残差过大/数值退化)。
    """

    origins_arr = np.asarray(origins, dtype=np.float64)
    directions_arr = np.asarray(directions, dtype=np.float64)
    if origins_arr.ndim != 2 or origins_arr.shape[1] != 3:
        return None
    if directions_arr.shape != origins_arr.shape:
        return None
    ray_count = origins_arr.shape[0]
    if ray_count < max(2, int(min_rays)):
        return None

    unit_dirs = _normalize_rows(directions_arr)
    if unit_dirs is None:
        return None

    parallax_deg = _max_parallax_deg(unit_dirs)
    if parallax_deg < float(min_parallax_deg):
        return None

    identity = np.eye(3, dtype=np.float64)
    a_mat = np.zeros((3, 3), dtype=np.float64)
    b_vec = np.zeros(3, dtype=np.float64)
    for i in range(ray_count):
        d = unit_dirs[i]
        proj = identity - np.outer(d, d)  # 投影到垂直于射线方向的平面
        a_mat += proj
        b_vec += proj @ origins_arr[i]

    # 条件数检查：近奇异(射线近平行)直接判退化
    if np.linalg.cond(a_mat) > 1e8:
        return None
    try:
        point = np.linalg.solve(a_mat, b_vec)
    except np.linalg.LinAlgError:
        return None

    # 残差：点到各射线的垂直距离 RMS
    residuals: List[float] = []
    for i in range(ray_count):
        d = unit_dirs[i]
        diff = point - origins_arr[i]
        perp = diff - (np.dot(diff, d) * d)
        residuals.append(float(np.linalg.norm(perp)))
    residual_rms = float(np.sqrt(np.mean(np.square(residuals)))) if residuals else 0.0
    if residual_rms > float(max_residual_m):
        return None

    return TriangulationResult(
        point=(float(point[0]), float(point[1]), float(point[2])),
        parallax_deg=float(parallax_deg),
        residual_m=residual_rms,
        ray_count=int(ray_count),
    )


def triangulation_confidence(
    *,
    ray_count: int,
    residual_m: float,
    max_residual_m: float = 0.5,
) -> float:
    """三角化置信度(量纲归一化，上限 0.7)。

    confidence = clamp(0.4 + 0.05·min(ray_count,6) − 0.3·(residual/MAX), 0.3, 0.7)
    上限 0.7 反映外参仅 URDF 建模值的系统偏差天花板。
    """

    normalized_residual = float(residual_m) / max(1e-6, float(max_residual_m))
    raw = 0.4 + 0.05 * min(int(ray_count), 6) - 0.3 * normalized_residual
    return float(max(0.3, min(0.7, raw)))
