from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import logging
import math
import struct
import threading
from typing import Any, Callable, Dict, Optional, Tuple

from contracts.geometry import Vector3
from contracts.pointcloud import PointCloudFrame
from services.geometry.config import PlatformGeometryConfig
from services.geometry.ros2_context import acquire_ros2_context, release_ros2_context

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.pointcloud_publisher")


class Ros2PointCloudPublisher:
    """把平台点云契约发布为 ROS2 PointCloud2。"""

    provider_name = "platform_ros2_pointcloud_publisher"
    provider_version = "0.1.0"

    def __init__(self, config: PlatformGeometryConfig, provider_resolver: Callable[[], Any]) -> None:
        self.config = config
        self.topic = str(config.topics.point_cloud_topic or "/utlidar/cloud")
        self.frame_id = str(config.frames.lidar_frame or "utlidar_lidar")
        self._provider_resolver = provider_resolver
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._started = False
        self._ros_ready = False
        self._has_ros2_context = False
        self._published_count = 0
        self._dropped_count = 0
        self._last_error: Optional[str] = None
        self._last_source_topic: Optional[str] = None
        self._last_published_signature: Optional[str] = None
        self._last_published_timestamp: Optional[datetime] = None
        self._node = None
        self._publisher = None
        self._rclpy = None
        self._pointcloud2_type = None
        self._point_field_type = None
        self._subscription_id: Optional[int] = None

    def start(self) -> None:
        if self._started:
            return
        self._stop_event.clear()
        try:
            self._start_ros2_publisher()
        except Exception as exc:
            self._last_error = str(exc)
            self._started = False
            self._ros_ready = False
            LOGGER.warning("启动 ROS2 点云发布桥失败：%s", exc)
            if self.config.require_ros2:
                raise
            return
        self._started = True
        self._ros_ready = True
        self._last_error = None
        self._subscribe_point_cloud_source()
        LOGGER.info("ROS2 点云发布桥已启动 point_cloud_topic=%s frame_id=%s", self.topic, self.frame_id)

    def stop(self) -> None:
        if not self._started and self._node is None:
            return
        node = self._node
        self._started = False
        self._ros_ready = False
        self._stop_event.set()
        self._unsubscribe_point_cloud_source()
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                LOGGER.debug("销毁 ROS2 点云发布节点失败。", exc_info=True)
        if self._has_ros2_context:
            try:
                release_ros2_context()
            except Exception:
                LOGGER.debug("释放 ROS2 点云发布上下文失败。", exc_info=True)
        self._node = None
        self._publisher = None
        self._rclpy = None
        self._pointcloud2_type = None
        self._point_field_type = None
        self._subscription_id = None
        self._has_ros2_context = False

    def is_available(self) -> bool:
        return bool(self._started and self._ros_ready and self._last_error is None)

    def get_status(self) -> Dict[str, object]:
        with self._lock:
            return {
                "provider_name": self.provider_name,
                "provider_version": self.provider_version,
                "started": self._started,
                "ros_ready": self._ros_ready,
                "point_cloud_topic": self.topic,
                "topic": self.topic,
                "frame_id": self.frame_id,
                "published_count": self._published_count,
                "dropped_count": self._dropped_count,
                "last_source_topic": self._last_source_topic,
                "source": self._last_source_topic,
                "last_published_timestamp": (
                    self._last_published_timestamp.isoformat() if self._last_published_timestamp is not None else None
                ),
                "last_error": self._last_error,
            }

    def _start_ros2_publisher(self) -> None:
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2, PointField

        node = None
        try:
            rclpy = acquire_ros2_context()
            self._has_ros2_context = True
            node = rclpy.create_node("nuwax_pointcloud_publisher")
            qos_profile = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._publisher = node.create_publisher(PointCloud2, self.topic, qos_profile)
            self._node = node
            self._rclpy = rclpy
            self._pointcloud2_type = PointCloud2
            self._point_field_type = PointField
        except Exception:
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    LOGGER.debug("清理 ROS2 点云发布节点失败。", exc_info=True)
            if self._has_ros2_context:
                release_ros2_context()
            self._node = None
            self._publisher = None
            self._rclpy = None
            self._pointcloud2_type = None
            self._point_field_type = None
            self._has_ros2_context = False
            raise

    def _subscribe_point_cloud_source(self) -> None:
        provider = self._provider_resolver()
        if provider is None:
            return
        subscriber = getattr(provider, "subscribe_point_cloud", None)
        if callable(subscriber):
            self._subscription_id = subscriber(self._publish_point_cloud)
            return
        self._last_error = "点云 provider 未实现 subscribe_point_cloud，未启用定时轮询伪流式。"

    def _unsubscribe_point_cloud_source(self) -> None:
        provider = self._provider_resolver()
        if provider is None or self._subscription_id is None:
            return
        unsubscriber = getattr(provider, "unsubscribe_point_cloud", None)
        if callable(unsubscriber):
            unsubscriber(self._subscription_id)
        self._subscription_id = None

    def _publish_point_cloud(self, point_cloud: PointCloudFrame) -> None:
        if self._publisher is None or self._pointcloud2_type is None or self._point_field_type is None:
            return
        if point_cloud is None or not point_cloud.points:
            return
        point_cloud = self._preprocess(point_cloud)
        if point_cloud is None:
            with self._lock:
                self._dropped_count += 1
            return
        message = self._build_pointcloud2_message(point_cloud)
        if message is None:
            with self._lock:
                self._dropped_count += 1
            return
        published_signature = self._build_payload_signature(point_cloud, message.data)
        if published_signature == self._last_published_signature:
            return
        self._publisher.publish(message)
        with self._lock:
            self._published_count += 1
            self._last_published_signature = published_signature
            self._last_published_timestamp = point_cloud.timestamp
            self._last_source_topic = str(point_cloud.metadata.get("source_topic") or "") or None
            self._last_error = None

    def _preprocess(self, point_cloud: PointCloudFrame) -> Optional[PointCloudFrame]:
        preprocess = self.config.rtabmap.point_cloud_preprocess
        raw_count = int(point_cloud.point_count or len(point_cloud.points))
        if not bool(preprocess.enabled):
            return self._downsample(point_cloud, raw_count=raw_count)

        min_range = max(0.0, float(preprocess.min_range_m))
        max_range = max(0.0, float(preprocess.max_range_m))
        crop_min_z = float(preprocess.crop_min_z_m)
        crop_max_z = float(preprocess.crop_max_z_m)
        crop_forward_min = float(preprocess.crop_forward_min_m)
        crop_forward_max = float(preprocess.crop_forward_max_m)
        crop_lateral = max(0.0, float(preprocess.crop_lateral_m))
        voxel_size = max(0.0, float(preprocess.voxel_size_m))
        filtered = []
        for point in point_cloud.points:
            x = float(getattr(point, "x", 0.0) or 0.0)
            y = float(getattr(point, "y", 0.0) or 0.0)
            z = float(getattr(point, "z", 0.0) or 0.0)
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            distance = math.sqrt(x * x + y * y + z * z)
            if distance < min_range or (max_range > 0.0 and distance > max_range):
                continue
            if z < crop_min_z or z > crop_max_z:
                continue
            if x < crop_forward_min or x > crop_forward_max:
                continue
            if crop_lateral > 0.0 and abs(y) > crop_lateral:
                continue
            filtered.append((x, y, z))
        if not filtered:
            return None

        if voxel_size > 1e-6:
            voxels: Dict[Tuple[int, int, int], Tuple[float, float, float]] = {}
            for x, y, z in filtered:
                key = (
                    int(math.floor(x / voxel_size)),
                    int(math.floor(y / voxel_size)),
                    int(math.floor(z / voxel_size)),
                )
                voxels.setdefault(key, (x, y, z))
            filtered = list(voxels.values())

        max_points = max(1, int(preprocess.max_points_per_scan))
        downsampled = False
        if len(filtered) > max_points:
            stride = max(1, int(math.ceil(len(filtered) / float(max_points))))
            filtered = filtered[::stride][:max_points]
            downsampled = True
        points = [Vector3(x=x, y=y, z=z) for x, y, z in filtered]
        metadata = {
            **dict(point_cloud.metadata),
            "ros2_bridge_preprocessed": True,
            "ros2_bridge_input_points": raw_count,
            "ros2_bridge_filtered_points": len(points),
            "ros2_bridge_removed_points": max(0, raw_count - len(points)),
            "ros2_bridge_downsampled": downsampled,
            "ros2_bridge_voxel_size_m": voxel_size,
            "ros2_bridge_min_range_m": min_range,
            "ros2_bridge_max_range_m": max_range,
            "ros2_bridge_crop_z_m": [crop_min_z, crop_max_z],
            "ros2_bridge_crop_forward_m": [crop_forward_min, crop_forward_max],
            "ros2_bridge_crop_lateral_m": crop_lateral,
        }
        return point_cloud.model_copy(
            update={"point_count": len(points), "points": points, "metadata": metadata},
            deep=True,
        )

    def _downsample(self, point_cloud: PointCloudFrame, *, raw_count: Optional[int] = None) -> PointCloudFrame:
        max_points = max(1, int(self.config.rtabmap.point_cloud_preprocess.max_points_per_scan))
        if point_cloud.point_count <= max_points:
            return point_cloud
        stride = max(1, int(math.ceil(point_cloud.point_count / float(max_points))))
        points = point_cloud.points[::stride][:max_points]
        return point_cloud.model_copy(
            update={
                "point_count": len(points),
                "points": points,
                "metadata": {
                    **dict(point_cloud.metadata),
                    "ros2_bridge_downsampled": True,
                    "ros2_bridge_input_points": int(raw_count or point_cloud.point_count),
                    "ros2_bridge_filtered_points": len(points),
                    "ros2_bridge_removed_points": max(0, int(raw_count or point_cloud.point_count) - len(points)),
                },
            },
            deep=True,
        )

    def _build_pointcloud2_message(self, point_cloud: PointCloudFrame):
        PointCloud2 = self._pointcloud2_type
        PointField = self._point_field_type
        if PointCloud2 is None or PointField is None:
            return None
        points = []
        for point in point_cloud.points:
            x = float(getattr(point, "x", 0.0) or 0.0)
            y = float(getattr(point, "y", 0.0) or 0.0)
            z = float(getattr(point, "z", 0.0) or 0.0)
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                points.append((x, y, z))
        if not points:
            return None

        message = PointCloud2()
        message.header.frame_id = str(point_cloud.frame_id or self.frame_id)
        stamp_sec, stamp_nanosec = self._timestamp_to_ros_stamp(self._select_publish_timestamp(point_cloud.timestamp))
        message.header.stamp.sec = stamp_sec
        message.header.stamp.nanosec = stamp_nanosec
        message.height = 1
        message.width = len(points)
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * len(points)
        message.is_dense = True
        payload = bytearray(message.row_step)
        for index, point in enumerate(points):
            struct.pack_into("<fff", payload, index * message.point_step, *point)
        message.data = bytes(payload)
        return message

    def _build_payload_signature(self, point_cloud: PointCloudFrame, payload: bytes) -> str:
        digest = hashlib.blake2b(payload, digest_size=12)
        digest.update(str(point_cloud.timestamp.isoformat()).encode("utf-8"))
        digest.update(str(point_cloud.point_count).encode("utf-8"))
        return digest.hexdigest()

    def _select_publish_timestamp(self, timestamp: datetime) -> datetime:
        normalized = timestamp
        if normalized.tzinfo is None:
            normalized = normalized.replace(tzinfo=timezone.utc)
        normalized = normalized.astimezone(timezone.utc)
        now = datetime.now(timezone.utc)
        age_sec = (now - normalized).total_seconds()
        quality = self.config.rtabmap.timestamp_quality
        max_age_sec = max(0.5, float(quality.max_point_cloud_age_sec) * 4.0)
        max_future_sec = max(0.1, float(quality.max_sensor_skew_sec))
        if age_sec > max_age_sec or age_sec < -max_future_sec:
            return now
        return normalized

    def _timestamp_to_ros_stamp(self, timestamp: datetime) -> Tuple[int, int]:
        normalized = timestamp
        if normalized.tzinfo is None:
            normalized = normalized.replace(tzinfo=timezone.utc)
        epoch_seconds = normalized.timestamp()
        seconds = int(epoch_seconds)
        nanoseconds = int(round((epoch_seconds - seconds) * 1_000_000_000))
        if nanoseconds >= 1_000_000_000:
            seconds += 1
            nanoseconds -= 1_000_000_000
        return seconds, max(0, nanoseconds)
