"""从 RTAB-Map 数据库导出累计点云并落盘为 PCD（rtabmap-export --scan CLI 路径）。

模块状态（DEPRECATED 标注，R-N14 v4 §R-N14-7 评估）:
- 目标态下点云由 nuwax-geometry-runtime 持续维护 SHM，平台层首选 mmap 路径
  （参见 services/geometry/pointcloud_map.py）。
- 本模块保留 ``export_cloud_map_pcd_from_database`` 路径作为
  - 当 SHM 路径不可用（例如 nuwax-geometry-runtime 未拉起、地图版本基线重建）时
    的回退导出；
  - 历史兼容性 / 离线导出（OOB 调试、CI 烟测）。
- ``fetch_cloud_map_xyz`` 仅保留为 DEPRECATED no-op（始终返回 ``None``），
  仅为让 services/geometry/runtime.py 中 ``_materialize_pcd_asset`` 的「订阅
  ROS2 话题」分支安全降级；R-N14 v4 后续 Phase 移除该分支后即可物理删除。

历史模块曾支持「订阅 /cloud_map ROS2 话题」路径；Phase 3 去 ROS2 后该路径下线，
本模块已完全脱离 ROS2 Python 接触面。
"""
from __future__ import annotations

import logging
import os
import subprocess
import tempfile
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

from contracts.geometry_runtime import DEFAULT_MAP_FRAME
from services.geometry.pointcloud_map import PointCloudMapError, read_ply_xyz, write_pcd_binary_xyz

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.cloud_map_fetcher")

FetchResult = Tuple[np.ndarray, str, str]


def default_min_points() -> int:
    """读取 NUWAX_PCD_MIN_POINTS（默认 500），用于 PCD 点数门槛判定。"""

    try:
        return max(1, int(os.environ.get("NUWAX_PCD_MIN_POINTS", "500")))
    except ValueError:
        return 500


def default_db_export_timeout_sec() -> float:
    """读取 NUWAX_PCD_DB_EXPORT_TIMEOUT_SEC（默认 45s），rtabmap-export 子进程超时。"""

    try:
        return max(5.0, float(os.environ.get("NUWAX_PCD_DB_EXPORT_TIMEOUT_SEC", "45")))
    except ValueError:
        return 45.0


def database_export_enabled() -> bool:
    """读取 NUWAX_PCD_EXPORT_FROM_DB（默认 true），是否启用 RTAB-Map DB 导出回退路径。"""

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
    """DEPRECATED:R-N14 v4 §R-N14-7 候选删除,目标态下走 nuwax-geometry-runtime SHM。

    历史「一次性订阅 /cloud_map 话题再写 mmap」路径已下线（ROS2 接触面已删除，
    ROS2 Python 客户端库全部下线）。本函数保留为 no-op（始终
    返回 ``None``）仅为兼容 services/geometry/runtime.py 的 ``_materialize_pcd_asset``
    中「ros_cloud_map source kind」分支；该分支后续 Phase 删除后即可物理删除本函数。
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