"""从 ROS /cloud_map 或 RTAB-Map 数据库抓取累计点云并落盘为 PCD。"""
from __future__ import annotations

import logging
import os
import shutil
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

from services.geometry.pointcloud_map import PointCloudMapError, read_ply_xyz, write_pcd_binary_xyz

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.cloud_map_fetcher")

FetchResult = Tuple[np.ndarray, str, str]


def default_cloud_map_topic() -> str:
    return str(os.environ.get("NUWAX_CLOUD_MAP_TOPIC", "/cloud_map") or "/cloud_map").strip() or "/cloud_map"


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
    map_frame: str = "map",
    timeout_sec: Optional[float] = None,
    min_points: Optional[int] = None,
) -> Optional[FetchResult]:
    """一次性订阅 RTAB-Map 发布的累计点云话题（默认 /cloud_map）。"""

    resolved_topic = str(topic or default_cloud_map_topic()).strip() or "/cloud_map"
    resolved_timeout = float(timeout_sec if timeout_sec is not None else default_fetch_timeout_sec())
    resolved_min_points = int(min_points if min_points is not None else default_min_points())
    try:
        from geometry_msgs.msg import PointCloud2
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs_py import point_cloud2

        from services.geometry.ros2_context import acquire_ros2_context, release_ros2_context
    except ImportError as exc:
        LOGGER.debug("ROS2 点云抓取依赖不可用：%s", exc)
        return None

    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    cyclonedds_xml = os.path.expanduser("~/cyclonedds_ws/cyclonedds.xml")
    if os.path.exists(cyclonedds_xml):
        os.environ.setdefault("CYCLONEDDS_URI", cyclonedds_xml)

    qos_profiles = (
        QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
        QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        ),
    )

    holder: Dict[str, object] = {"message": None}

    def _on_message(message: PointCloud2) -> None:
        holder["message"] = message

    rclpy = None
    node = None
    has_context = False
    try:
        rclpy = acquire_ros2_context()
        has_context = True
        node = rclpy.create_node("nuwax_cloud_map_fetcher")
        for qos in qos_profiles:
            node.create_subscription(PointCloud2, resolved_topic, _on_message, qos)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        deadline = time.monotonic() + resolved_timeout
        while time.monotonic() < deadline and holder["message"] is None:
            executor.spin_once(timeout_sec=0.1)
        message = holder["message"]
        if message is None:
            return None
        frame_id = str(getattr(message.header, "frame_id", "") or map_frame or "map")
        points = list(
            point_cloud2.read_points(
                message,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        )
        if not points:
            return None
        xyz = np.asarray(points, dtype=np.float32)
        if xyz.ndim != 2 or xyz.shape[1] != 3:
            return None
        finite_mask = np.isfinite(xyz).all(axis=1)
        xyz = xyz[finite_mask]
        if int(xyz.shape[0]) < resolved_min_points:
            LOGGER.info(
                "话题 %s 点云过少（%d < %d），跳过作为版本 PCD。",
                resolved_topic,
                int(xyz.shape[0]),
                resolved_min_points,
            )
            return None
        return xyz, frame_id, resolved_topic
    except Exception as exc:
        LOGGER.warning("从 ROS 话题 %s 抓取累计点云失败：%s", resolved_topic, exc)
        return None
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if has_context:
            release_ros2_context()


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
