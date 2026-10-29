"""检测的三维支持点选择：从框或掩码内的投影点中挑出代表物体主体的点。

输出位置和尺寸都由这里选中的点决定，同时给出本次测量的不确定度，
供实例关联设定门限和多次观察加权融合。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class SupportSelection:
    """indices 为候选数组中的下标，按深度升序，数量不超过 max_points。"""

    indices: np.ndarray
    method: str
    candidate_point_count: int
    cluster_point_count: int
    depth_spread_m: Optional[float] = None
    measurement_sigma_m: Optional[float] = None
    cluster_indices: Optional[np.ndarray] = None
    sampling_sigma_m: Optional[float] = None


def select_object_support(
    *,
    candidate_indices: np.ndarray,
    camera_points: np.ndarray,
    projected_pixels: np.ndarray,
    bbox_xyxy: Tuple[float, float, float, float],
    from_mask: bool,
    max_points: int,
    focal_lengths_px: Optional[Tuple[float,float]] = None,
) -> SupportSelection:
    """从候选点中选择物体主体支持点。

    candidate_indices 指向 camera_points/projected_pixels 中位于检测框（from_mask=False）
    或分割掩码（from_mask=True）内的点。
    """
    candidate_indices = np.asarray(candidate_indices, dtype=np.int64)
    if not from_mask and candidate_indices.size:
        x0,y0,x1,y1 = bbox_xyxy
        dx,dy = (x1-x0)*0.15,(y1-y0)*0.15
        uv = projected_pixels[candidate_indices]
        keep = (uv[:,0]>=x0+dx)&(uv[:,0]<=x1-dx)&(uv[:,1]>=y0+dy)&(uv[:,1]<=y1-dy)
        candidate_indices = candidate_indices[keep]
    count = len(candidate_indices)
    if not count:
        return SupportSelection(candidate_indices, "dominant_depth_cluster", 0, 0, cluster_indices=candidate_indices)
    depths = camera_points[candidate_indices,2]
    order = np.argsort(depths, kind="stable")
    sorted_indices, depths = candidate_indices[order], depths[order]
    # 深度噪声随距离平方增长；间隙至少为3厘米，避免把同一表面拆成小簇。
    gaps = np.diff(depths)
    cuts = np.flatnonzero(gaps > np.maximum(0.03, 0.01 + 0.01*depths[:-1]**2)) + 1
    boundaries = np.concatenate(([0],cuts,[count]))
    largest = int(np.argmax(np.diff(boundaries)))
    cluster = sorted_indices[boundaries[largest]:boundaries[largest+1]]
    points = camera_points[cluster]
    low,high = np.quantile(points,[0.05,0.95],axis=0)
    spread = float(high[2]-low[2])
    median_depth = float(np.median(points[:,2]))
    robust_sigma = float(np.max((high-low)/3.29)) if len(cluster)>=3 else 0.0
    sampling_sigma=0.0
    if focal_lengths_px is not None:
        # 像素采样单元的均匀量化误差；少于3点时用检测内框跨度，不能由两点推离散度。
        if len(cluster)<3:
            span=np.array((bbox_xyxy[2]-bbox_xyxy[0],bbox_xyxy[3]-bbox_xyxy[1]))
            spacing=np.maximum(1.,span*(1. if from_mask else .7)/np.sqrt(len(cluster)))
        else:
            span=np.ptp(projected_pixels[cluster],axis=0)
            spacing=np.maximum(1.,span/max(1.,np.sqrt(len(cluster))-1.))
        angular_step=float(np.max(spacing/np.asarray(focal_lengths_px)))
        sampling_sigma=median_depth*angular_step/np.sqrt(12.)
    # 系统深度误差不随点数消失；只把簇离散的独立分量除以根号点数。
    sensor_floor = 0.005 + 0.002*median_depth**2
    sigma = float(np.sqrt(sensor_floor**2 + (robust_sigma/np.sqrt(len(cluster)))**2 + sampling_sigma**2))
    selected = cluster[np.linspace(0,len(cluster)-1,min(max_points,len(cluster)),dtype=np.int64)]
    return SupportSelection(selected,"dominant_depth_cluster",count,len(cluster),spread,sigma,cluster,sampling_sigma)
