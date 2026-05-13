from __future__ import annotations

from datetime import datetime, timezone
import logging
import math
import threading
from typing import Any, Dict, List, Optional, Tuple

from contracts.geometry import Pose, Vector3
from services.geometry.config import PlatformGeometryConfig
from services.geometry.ros2_context import acquire_ros2_context, release_ros2_context

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.frontier_marker_provider")

_MARKER_ADD = 0
_MARKER_DELETE = 2
_MARKER_POINTS = 8
_MARKER_SPHERE = 2


class Ros2FrontierMarkerProvider:
    """订阅 go2slam explore_lite 的 MarkerArray 并暴露平台 frontier 元数据。"""

    provider_name = "platform_ros2_frontier_markers"
    provider_version = "0.1.0"

    def __init__(self, config: PlatformGeometryConfig) -> None:
        self.config = config
        self.topic = str(getattr(config.topics, "exploration_frontiers_topic", "") or "/explore/frontiers")
        self._lock = threading.RLock()
        self._started = False
        self._ros_ready = False
        self._has_ros2_context = False
        self._last_error: Optional[str] = None
        self._latest_frontiers: List[Dict[str, object]] = []
        self._latest_received_at: Optional[datetime] = None
        self._latest_source_timestamp: Optional[datetime] = None
        self._message_count = 0
        self._node = None
        self._executor = None
        self._thread: Optional[threading.Thread] = None
        self._rclpy = None
        self._subscriptions: List[Any] = []

    def start(self) -> None:
        if self._started:
            return
        try:
            self._start_ros2_subscriptions()
        except Exception as exc:
            self._last_error = str(exc)
            self._started = False
            self._ros_ready = False
            LOGGER.warning("启动 ROS2 frontier marker 订阅失败：%s", exc)
            if self.config.require_ros2:
                raise
            return
        self._started = True
        self._ros_ready = True
        self._last_error = None
        LOGGER.info("ROS2 frontier marker 订阅已启动 topic=%s", self.topic)

    def stop(self) -> None:
        if not self._started and self._node is None and self._executor is None:
            return
        executor = self._executor
        node = self._node
        self._started = False
        self._ros_ready = False
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=1.0)
            except TypeError:
                executor.shutdown()
            except Exception:
                LOGGER.debug("停止 ROS2 frontier marker executor 失败。", exc_info=True)
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                LOGGER.debug("销毁 ROS2 frontier marker 节点失败。", exc_info=True)
        if self._has_ros2_context:
            try:
                release_ros2_context()
            except Exception:
                LOGGER.debug("释放 ROS2 frontier marker 上下文失败。", exc_info=True)
        self._node = None
        self._executor = None
        self._thread = None
        self._rclpy = None
        self._subscriptions = []
        self._has_ros2_context = False

    def is_available(self) -> bool:
        return bool(self._started and self._ros_ready and self._last_error is None)

    def get_latest_frontiers(self) -> List[Dict[str, object]]:
        with self._lock:
            return [self._clone_frontier(frontier) for frontier in self._latest_frontiers]

    def get_status(self) -> Dict[str, object]:
        with self._lock:
            frontiers = [self._clone_frontier(frontier) for frontier in self._latest_frontiers]
            received_at = self._latest_received_at
            source_timestamp = self._latest_source_timestamp
            message_count = self._message_count
            last_error = self._last_error
        publisher_count = self._publisher_count(self.topic)
        now = datetime.now(timezone.utc)
        latest_age_sec = max(0.0, (now - received_at).total_seconds()) if received_at is not None else None
        provider_status = "available" if self.is_available() else ("error" if last_error else "starting" if self._started else "stopped")
        return {
            "provider_name": self.provider_name,
            "provider_version": self.provider_version,
            "provider_status": provider_status,
            "started": self._started,
            "ros_ready": self._ros_ready,
            "topic": self.topic,
            "publisher_count": publisher_count,
            "has_publisher": publisher_count is not None and publisher_count > 0,
            "message_count": message_count,
            "frontier_count": len(frontiers),
            "frontiers_ready": bool(frontiers),
            "latest_received_at": received_at.isoformat() if received_at is not None else None,
            "latest_source_timestamp": source_timestamp.isoformat() if source_timestamp is not None else None,
            "latest_age_sec": latest_age_sec,
            "last_error": last_error,
        }

    @classmethod
    def frontiers_from_marker_array(
        cls,
        message: Any,
        *,
        source_topic: str = "/explore/frontiers",
    ) -> List[Dict[str, object]]:
        markers = [marker for marker in list(getattr(message, "markers", []) or []) if cls._marker_action(marker) != _MARKER_DELETE]
        spheres: Dict[str, Any] = {}
        for marker in markers:
            if cls._marker_type(marker) == _MARKER_SPHERE:
                key = cls._frontier_key_for_marker(marker)
                spheres[key] = marker

        frontiers: List[Dict[str, object]] = []
        for marker in markers:
            if cls._marker_type(marker) != _MARKER_POINTS:
                continue
            points = cls._points_from_marker(marker)
            if not points:
                continue
            key = cls._frontier_key_for_marker(marker)
            centroid_marker = spheres.get(key)
            centroid = cls._position_from_marker(centroid_marker) if centroid_marker is not None else None
            if centroid is None:
                centroid = cls._centroid(points)
            frame_id = cls._frame_id_from_marker(marker, default="map")
            pose = Pose(frame_id=frame_id, position=Vector3(x=centroid[0], y=centroid[1], z=centroid[2]))
            marker_id = int(getattr(marker, "id", len(frontiers)) or 0)
            sphere_id = int(getattr(centroid_marker, "id", marker_id + 1) or marker_id + 1) if centroid_marker is not None else None
            scale = cls._scale_from_marker(centroid_marker or marker)
            frontier = {
                "id": "go2slam_frontier_%s" % marker_id,
                "geometry": {"type": "point", "coordinates": [centroid[0], centroid[1], centroid[2]]},
                "pose": pose.model_dump(mode="json"),
                "score": max(0.0, 1.0 - len(frontiers) * 0.05),
                "cluster_size": len(points),
                "info_gain_cells": len(points),
                "path_distance_m": None,
                "average_cost": None,
                "metadata": {
                    "source": "go2slam_explore_lite",
                    "source_topic": str(source_topic or "/explore/frontiers"),
                    "marker_namespace": str(getattr(marker, "ns", "") or ""),
                    "marker_ids": [marker_id] + ([sphere_id] if sphere_id is not None else []),
                    "frame_id": frame_id,
                    "points": [{"x": x, "y": y, "z": z} for x, y, z in points],
                    "centroid": {"x": centroid[0], "y": centroid[1], "z": centroid[2]},
                    "scale": scale,
                    "source_timestamp": cls._timestamp_from_marker(marker),
                },
            }
            frontiers.append(frontier)
        return frontiers

    @staticmethod
    def _clone_frontier(frontier: Dict[str, object]) -> Dict[str, object]:
        return {
            key: (
                dict(value)
                if isinstance(value, dict)
                else [dict(item) if isinstance(item, dict) else item for item in value]
                if isinstance(value, list)
                else value
            )
            for key, value in frontier.items()
        }

    @staticmethod
    def _marker_action(marker: Any) -> int:
        try:
            return int(getattr(marker, "action", _MARKER_ADD))
        except (TypeError, ValueError):
            return _MARKER_ADD

    @staticmethod
    def _marker_type(marker: Any) -> int:
        try:
            return int(getattr(marker, "type", -1))
        except (TypeError, ValueError):
            return -1

    @staticmethod
    def _frontier_key_for_marker(marker: Any) -> str:
        marker_id = int(getattr(marker, "id", 0) or 0)
        pair_id = marker_id - (marker_id % 2)
        namespace = str(getattr(marker, "ns", "") or "frontiers")
        return "%s:%s" % (namespace, pair_id)

    @classmethod
    def _points_from_marker(cls, marker: Any) -> List[Tuple[float, float, float]]:
        points: List[Tuple[float, float, float]] = []
        for point in list(getattr(marker, "points", []) or []):
            parsed = cls._point_tuple(point)
            if parsed is not None:
                points.append(parsed)
        return points

    @staticmethod
    def _point_tuple(point: Any) -> Optional[Tuple[float, float, float]]:
        try:
            x = float(getattr(point, "x"))
            y = float(getattr(point, "y"))
            z = float(getattr(point, "z", 0.0) or 0.0)
        except (TypeError, ValueError):
            return None
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            return None
        return (x, y, z)

    @classmethod
    def _position_from_marker(cls, marker: Any) -> Optional[Tuple[float, float, float]]:
        if marker is None:
            return None
        pose = getattr(marker, "pose", None)
        return cls._point_tuple(getattr(pose, "position", None))

    @staticmethod
    def _centroid(points: List[Tuple[float, float, float]]) -> Tuple[float, float, float]:
        count = max(1, len(points))
        return (
            sum(point[0] for point in points) / count,
            sum(point[1] for point in points) / count,
            sum(point[2] for point in points) / count,
        )

    @staticmethod
    def _frame_id_from_marker(marker: Any, *, default: str) -> str:
        header = getattr(marker, "header", None)
        frame_id = str(getattr(header, "frame_id", "") or "").strip()
        return frame_id or default

    @staticmethod
    def _scale_from_marker(marker: Any) -> Dict[str, float]:
        scale = getattr(marker, "scale", None)
        return {
            "x": float(getattr(scale, "x", 0.0) or 0.0),
            "y": float(getattr(scale, "y", 0.0) or 0.0),
            "z": float(getattr(scale, "z", 0.0) or 0.0),
        }

    @staticmethod
    def _timestamp_from_marker(marker: Any) -> Optional[str]:
        header = getattr(marker, "header", None)
        stamp = getattr(header, "stamp", None)
        try:
            sec = int(getattr(stamp, "sec"))
            nanosec = int(getattr(stamp, "nanosec", 0) or 0)
        except (TypeError, ValueError):
            return None
        return datetime.fromtimestamp(sec + nanosec / 1_000_000_000.0, timezone.utc).isoformat()

    @classmethod
    def _source_timestamp_from_message(cls, message: Any) -> Optional[datetime]:
        for marker in list(getattr(message, "markers", []) or []):
            timestamp = cls._timestamp_from_marker(marker)
            if timestamp:
                try:
                    return datetime.fromisoformat(timestamp)
                except ValueError:
                    return None
        return None

    def _publisher_count(self, topic: str) -> Optional[int]:
        if not topic or self._node is None:
            return None
        try:
            return len(self._node.get_publishers_info_by_topic(topic))
        except Exception:
            LOGGER.debug("读取 ROS2 frontier marker publisher 失败 topic=%s", topic, exc_info=True)
            return None

    def _start_ros2_subscriptions(self) -> None:
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from visualization_msgs.msg import MarkerArray

        node = None
        executor = None
        try:
            rclpy = acquire_ros2_context()
            self._has_ros2_context = True
            node = rclpy.create_node("nuwax_frontier_marker_provider")
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            qos_profile = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._subscriptions.append(
                node.create_subscription(MarkerArray, self.topic, self._on_marker_array, qos_profile)
            )
            self._rclpy = rclpy
            self._node = node
            self._executor = executor
            self._thread = threading.Thread(target=self._spin, name="nuwax_ros2_frontier_marker_provider", daemon=True)
            self._thread.start()
        except Exception:
            self._subscriptions = []
            if executor is not None:
                try:
                    executor.shutdown(timeout_sec=0.0)
                except TypeError:
                    executor.shutdown()
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    LOGGER.debug("清理 ROS2 frontier marker 节点失败。", exc_info=True)
            if self._has_ros2_context:
                release_ros2_context()
            self._node = None
            self._executor = None
            self._rclpy = None
            self._has_ros2_context = False
            raise

    def _spin(self) -> None:
        executor = self._executor
        if executor is None:
            return
        try:
            executor.spin()
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
                self._ros_ready = False
            LOGGER.warning("ROS2 frontier marker 订阅循环异常：%s", exc)

    def _on_marker_array(self, message: Any) -> None:
        frontiers = self.frontiers_from_marker_array(message, source_topic=self.topic)
        with self._lock:
            self._latest_frontiers = frontiers
            self._message_count += 1
            self._latest_received_at = datetime.now(timezone.utc)
            self._latest_source_timestamp = self._source_timestamp_from_message(message)
            self._last_error = None
