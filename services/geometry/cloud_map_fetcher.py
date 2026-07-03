"""从 RTAB-Map 数据库抓取累计点云并落盘为 PCD。

历史模块曾支持「订阅 /cloud_map ROS2 话题」路径；Phase 3 去 ROS2 后该路径下线，
点云由 nuwax-geometry-runtime 持续维护 SHM（参见 services/geometry/pointcloud_map.py），
平台层走 ``export_cloud_map_pcd_from_database``（rtabmap-export --scan）路径即可。

本模块保留 ``fetch_cloud_map_xyz`` 入口签名作为 no-op（始终返回 ``None``），
仅用于让上层调用方不报错地走「目标态 mmap 路径」。
"""
from __future__ import annotations

import logging
import os
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

from contracts.geometry_runtime import DEFAULT_MAP_FRAME
from services.geometry.pointcloud_map import PointCloudMapError, read_ply_xyz, write_pcd_binary_xyz

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.cloud_map_fetcher")

FetchResult = Tuple[np.ndarray, str, str]


def default_cloud_map_topic() -> str:
    """保留 NUWAX_GEOMETRY_CLOUD_MAP_TOPIC → NUWAX_CLOUD_MAP_TOPIC 回退链路。

    Phase 3 目标态下「订阅 ROS2 话题」路径已下线，但默认话题名仍提供给上层做诊断日志。
    """
    preferred = str(os.environ.get("NUWAX_GEOMETRY_CLOUD_MAP_TOPIC") or "").strip()
    if preferred:
        return preferred
    legacy = str(os.environ.get("NUWAX_CLOUD_MAP_TOPIC") or "").strip()
    if legacy:
        return legacy
    return "/cloud_map"


def default_min_points() -> int:
    try:
        return max(1, int(os.environ.get("NUWAX_PCD_MIN_POINTS", "500")))
    except ValueError:
        return 500


def default_fetch_timeout_sec() -> float:
    try:
        return max(0.5, float(os.environ.get("NUWAX_CLOUD_MAP_FETCH_TIMEOUT_SEC", "5.0")))
    except ValueError:
        return 5.0


def default_db_export_timeout_sec() -> float:
    try:
        return max(5.0, float(os.environ.get("NUWAX_PCD_DB_EXPORT_TIMEOUT_SEC", "45")))
    except ValueError:
        return 45.0


def database_export_enabled() -> bool:
    return str(os.environ.get("NUWAX_PCD_EXPORT_FROM_DB", "true")).strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


def fetch_cloud_map_xyz(
    topic: Optional[str] = None,
    *,
    map_frame: str = DEFAULT_MAP_FRAME,
    timeout_sec: Optional[float] = None,
    min_points: Optional[int] = None,
) -> Optional[FetchResult]:
    """平台层累计点云获取入口（Phase 3 目标态 no-op）。

    历史「一次性订阅 /cloud_map 话题再写 mmap」路径已下线（ROS2 接触面已删除），
    sensor_msgs_py / PointCloud2 等 ROS2 接触面）。点云由 nuwax-geometry-runtime
    持续维护 SHM（参见 services/geometry/pointcloud_map.py），平台层走 mmap 读取即可。

    本函数保留签名以兼容现有调用方：始终返回 ``None``，调用方应改走
    ``export_cloud_map_pcd_from_database`` 或 runtime SHM 路径。
    """
    LOGGER.debug(
        "fetch_cloud_map_xyz 目标态走 nuwax-geometry-runtime mmap 路径，不再订阅 /cloud_map topic。"
    )
    return None


def export_cloud_map_pcd_from_database(
    database_path: Path,
    target_pcd_path: Path,
    *,
    timeout_sec: Optional[float] = None,
    min_points: Optional[int] = None,
) -> int:
    """使用 rtabmap-export --scan 从 .db 导出累计点云并转换为 PCD。"""

    resolved_db = database_path.expanduser().resolve()
    if not resolved_db.is_file():
        raise PointCloudMapError("RTAB-Map 数据库不存在：%s" % resolved_db)
    resolved_target = target_pcd_path.expanduser().resolve()
    resolved_target.parent.mkdir(parents=True, exist_ok=True)
    resolved_min_points = int(min_points if min_points is not None else default_min_points())
    resolved_timeout = float(timeout_sec if timeout_sec is not None else default_db_export_timeout_sec())
    output_name = resolved_target.stem
    with tempfile.TemporaryDirectory(prefix="nuwax_rtabmap_export_") as temp_dir:
        command = [
            "rtabmap-export",
            "--output_dir",
            temp_dir,
            "--output",
            output_name,
            "--scan",
            str(resolved_db),
        ]
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=max(5.0, resolved_timeout),
            check=False,
        )
        if completed.returncode != 0:
            stderr = (completed.stderr or "").strip()
            stdout = (completed.stdout or "").strip()
            detail = stderr or stdout or "rtabmap-export failed"
            raise PointCloudMapError(detail)
        ply_candidates = sorted(Path(temp_dir).glob("*.ply"))
        if not ply_candidates:
            raise PointCloudMapError("rtabmap-export 未生成 PLY 点云文件。")
        _, xyz = read_ply_xyz(ply_candidates[0])
        if int(xyz.shape[0]) < resolved_min_points:
            raise PointCloudMapError(
                "rtabmap-export 点云过少（%d < %d）。" % (int(xyz.shape[0]), resolved_min_points)
            )
        write_pcd_binary_xyz(resolved_target, xyz)
        return int(xyz.shape[0])