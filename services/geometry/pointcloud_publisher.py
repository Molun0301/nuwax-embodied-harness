from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import logging
import math
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple

import numpy as np

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
        self._last_published_payload: Optional[np.ndarray] = None
        self._worker_condition = threading.Condition()
        self._pending_point_cloud: Optional[PointCloudFrame] = None
        self._pending_sequence = 0
        self._processed_sequence = 0
        self._worker_thread: Optional[threading.Thread] = None
        self._last_publish_monotonic = 0.0
        self._worker_drop_count = 0
        self._duplicate_drop_count = 0
        self._rate_drop_count = 0
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
        self._worker_thread = threading.Thread(target=self._publish_worker_loop, name="nuwax_pointcloud_publish_worker", daemon=True)
        self._worker_thread.start()
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
        with self._worker_condition:
            self._worker_condition.notify_all()
        if self._worker_thread is not None and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=1.0)
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
        self._last_published_payload = None
        self._pending_point_cloud = None
        self._worker_thread = None
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
                "worker_drop_count": self._worker_drop_count,
                "duplicate_drop_count": self._duplicate_drop_count,
                "rate_drop_count": self._rate_drop_count,
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
            self._subscription_id = subscriber(self._enqueue_point_cloud)
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

    def _enqueue_point_cloud(self, point_cloud: PointCloudFrame) -> None:
        if point_cloud is None:
            return
        with self._worker_condition:
            if self._pending_sequence != self._processed_sequence:
                with self._lock:
                    self._worker_drop_count += 1
                    self._dropped_count += 1
            self._pending_sequence += 1
            self._pending_point_cloud = point_cloud
            self._worker_condition.notify()

    def _publish_worker_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._worker_condition:
                while not self._stop_event.is_set() and self._pending_sequence == self._processed_sequence:
                    self._worker_condition.wait(timeout=0.5)
                if self._stop_event.is_set():
                    return
                sequence = self._pending_sequence
                point_cloud = self._pending_point_cloud
                self._processed_sequence = sequence
            if point_cloud is None:
                continue
            self._publish_point_cloud(point_cloud)

    def _publish_point_cloud(self, point_cloud: PointCloudFrame) -> None:
        if self._publisher is None or self._pointcloud2_type is None or self._point_field_type is None:
            return
        if not point_cloud.points:
            return
        if self._should_skip_for_rate_limit():
            with self._lock:
                self._rate_drop_count += 1
                self._dropped_count += 1
            return
        points, metadata = self._preprocess_points(point_cloud)
        if points.size == 0:
            with self._lock:
                self._dropped_count += 1
            return
        published_signature = self._build_payload_signature(point_cloud, points, metadata)
        if published_signature == self._last_published_signature and self._is_duplicate_payload(points):
            with self._lock:
                self._duplicate_drop_count += 1
                self._dropped_count += 1
            return
        message = self._build_pointcloud2_message(point_cloud, points)
        if message is None:
            with self._lock:
                self._dropped_count += 1
            return
        self._publisher.publish(message)
        with self._lock:
            self._published_count += 1
            self._last_publish_monotonic = time.monotonic()
            self._last_published_signature = published_signature
            self._last_published_timestamp = point_cloud.timestamp
            self._last_source_topic = str(point_cloud.metadata.get("source_topic") or "") or None
            self._last_published_payload = points
            self._last_error = None

    def _should_skip_for_rate_limit(self) -> bool:
        max_age_sec = max(0.05, float(self.config.rtabmap.timestamp_quality.max_point_cloud_age_sec))
        min_interval_sec = max(0.02, min(0.2, max_age_sec * 0.5))
        return bool(self._last_publish_monotonic > 0.0 and time.monotonic() - self._last_publish_monotonic < min_interval_sec)

    def _preprocess_points(self, point_cloud: PointCloudFrame) -> Tuple[np.ndarray, dict[str, object]]:
        raw_points = self._points_to_array(point_cloud)
        raw_count = int(point_cloud.point_count or raw_points.shape[0])
        if raw_points.size == 0:
            return raw_points, dict(point_cloud.metadata)
        preprocess = self.config.rtabmap.point_cloud_preprocess
        max_points = max(1, int(preprocess.max_points_per_scan))
        metadata = dict(point_cloud.metadata)
        if not bool(preprocess.enabled):
            points, downsampled = self._downsample_array(raw_points, max_points=max_points)
            metadata.update(
                {
                    "ros2_bridge_downsampled": downsampled,
                    "ros2_bridge_input_points": raw_count,
                    "ros2_bridge_filtered_points": int(points.shape[0]),
                    "ros2_bridge_removed_points": max(0, raw_count - int(points.shape[0])),
                }
            )
            return points, metadata

        min_range = max(0.0, float(preprocess.min_range_m))
        max_range = max(0.0, float(preprocess.max_range_m))
        crop_min_z = float(preprocess.crop_min_z_m)
        crop_max_z = float(preprocess.crop_max_z_m)
        crop_forward_min = float(preprocess.crop_forward_min_m)
        crop_forward_max = float(preprocess.crop_forward_max_m)
        crop_lateral = max(0.0, float(preprocess.crop_lateral_m))
        voxel_size = max(0.0, float(preprocess.voxel_size_m))

        finite = np.isfinite(raw_points).all(axis=1)
        distances = np.linalg.norm(raw_points, axis=1)
        mask = finite & (distances >= min_range)
        if max_range > 0.0:
            mask &= distances <= max_range
        mask &= (raw_points[:, 2] >= crop_min_z) & (raw_points[:, 2] <= crop_max_z)
        mask &= (raw_points[:, 0] >= crop_forward_min) & (raw_points[:, 0] <= crop_forward_max)
        if crop_lateral > 0.0:
            mask &= np.abs(raw_points[:, 1]) <= crop_lateral
        points = raw_points[mask]
        if points.size == 0:
            return np.empty((0, 3), dtype=np.float32), metadata

        if voxel_size > 1e-6:
            voxel_keys = np.floor(points / voxel_size).astype(np.int64, copy=False)
            _, first_indices = np.unique(voxel_keys, axis=0, return_index=True)
            points = points[np.sort(first_indices)]

        points, downsampled = self._downsample_array(points, max_points=max_points)
        metadata.update(
            {
                "ros2_bridge_preprocessed": True,
                "ros2_bridge_input_points": raw_count,
                "ros2_bridge_filtered_points": int(points.shape[0]),
                "ros2_bridge_removed_points": max(0, raw_count - int(points.shape[0])),
                "ros2_bridge_downsampled": downsampled,
                "ros2_bridge_voxel_size_m": voxel_size,
                "ros2_bridge_min_range_m": min_range,
                "ros2_bridge_max_range_m": max_range,
                "ros2_bridge_crop_z_m": [crop_min_z, crop_max_z],
                "ros2_bridge_crop_forward_m": [crop_forward_min, crop_forward_max],
                "ros2_bridge_crop_lateral_m": crop_lateral,
            }
        )
        return np.ascontiguousarray(points, dtype=np.float32), metadata

    def _points_to_array(self, point_cloud: PointCloudFrame) -> np.ndarray:
        points = point_cloud.points
        if points is None:
            return np.empty((0, 3), dtype=np.float32)
        if isinstance(points, np.ndarray):
            array = np.asarray(points, dtype=np.float32)
        else:
            point_count = len(points)
            if point_count == 0:
                return np.empty((0, 3), dtype=np.float32)
            first_point = points[0]
            if isinstance(first_point, (tuple, list, np.ndarray)):
                array = np.asarray(points, dtype=np.float32)
            else:
                array = np.fromiter(
                    (coordinate for point in points for coordinate in (float(point.x), float(point.y), float(point.z))),
                    dtype=np.float32,
                    count=point_count * 3,
                ).reshape((-1, 3))
        if array.ndim != 2 or array.shape[1] < 3:
            return np.empty((0, 3), dtype=np.float32)
        return np.ascontiguousarray(array[:, :3], dtype=np.float32)

    def _downsample_array(self, points: np.ndarray, *, max_points: int) -> Tuple[np.ndarray, bool]:
        if points.shape[0] <= max_points:
            return np.ascontiguousarray(points, dtype=np.float32), False
        stride = max(1, int(math.ceil(points.shape[0] / float(max_points))))
        return np.ascontiguousarray(points[::stride][:max_points], dtype=np.float32), True

    def _build_pointcloud2_message(self, point_cloud: PointCloudFrame, points: np.ndarray):
        PointCloud2 = self._pointcloud2_type
        PointField = self._point_field_type
        if PointCloud2 is None or PointField is None:
            return None
        if points.size == 0:
            return None
        points = np.ascontiguousarray(points, dtype=np.float32)

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
        message.data = points.tobytes(order="C")
        return message

    def _is_duplicate_payload(self, points: np.ndarray) -> bool:
        previous = self._last_published_payload
        return bool(previous is not None and previous.shape == points.shape and np.array_equal(previous, points))

    def _build_payload_signature(self, point_cloud: PointCloudFrame, points: np.ndarray, metadata: dict[str, object]) -> str:
        if self._is_duplicate_payload(points):
            return self._last_published_signature or "same_payload"
        digest = hashlib.blake2b(digest_size=12)
        digest.update(str(point_cloud.timestamp.isoformat()).encode("utf-8"))
        digest.update(str(points.shape[0]).encode("utf-8"))
        digest.update(str(metadata.get("source_topic") or "").encode("utf-8"))
        if points.size:
            sample_indices = np.unique(np.asarray([0, points.shape[0] // 2, points.shape[0] - 1], dtype=np.int64))
            digest.update(np.ascontiguousarray(points[sample_indices], dtype=np.float32).tobytes())
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
