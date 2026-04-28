from __future__ import annotations

from datetime import datetime, timezone
import logging
import threading
from typing import Any, Dict, List, Optional

from adapters.helpers import build_pose
from adapters.mapping.slam_occupancy_adapter import SlamOccupancyAdapter
from contracts.maps import CostMap, OccupancyGrid, SemanticMap
from services.geometry.config import PlatformGeometryConfig
from services.geometry.ros2_context import acquire_ros2_context, release_ros2_context

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.live_map_provider")


class Ros2LiveMapProvider:
    """订阅 ROS2 实时地图并暴露平台地图契约。"""

    provider_name = "platform_ros2_live_map"
    provider_version = "0.1.0"

    def __init__(self, config: PlatformGeometryConfig) -> None:
        self.config = config
        self.occupancy_topic = str(config.topics.occupancy_topic or "/map")
        self.cost_map_topic = str(config.topics.cost_map_topic or "")
        self._lock = threading.RLock()
        self._adapter = SlamOccupancyAdapter(
            SlamOccupancyAdapter.build_default_config(
                name="ros2_live_occupancy",
                source_ref=self.occupancy_topic,
            )
        )
        self._latest_occupancy_grid: Optional[OccupancyGrid] = None
        self._latest_cost_map: Optional[CostMap] = None
        self._last_error: Optional[str] = None
        self._started = False
        self._ros_ready = False
        self._has_ros2_context = False
        self._occupancy_message_count = 0
        self._cost_map_message_count = 0
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
            LOGGER.warning("启动 ROS2 实时地图订阅失败：%s", exc)
            if self.config.require_ros2:
                raise
            return
        self._started = True
        self._ros_ready = True
        self._last_error = None
        LOGGER.info(
            "ROS2 实时地图订阅已启动 occupancy_topic=%s cost_map_topic=%s",
            self.occupancy_topic,
            self.cost_map_topic or "-",
        )

    def stop(self) -> None:
        if not self._started and self._node is None and self._executor is None:
            return
        executor = self._executor
        node = self._node
        rclpy = self._rclpy
        self._started = False
        self._ros_ready = False
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=1.0)
            except TypeError:
                executor.shutdown()
            except Exception:
                LOGGER.debug("停止 ROS2 实时地图 executor 失败。", exc_info=True)
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                LOGGER.debug("销毁 ROS2 实时地图节点失败。", exc_info=True)
        if self._has_ros2_context:
            try:
                release_ros2_context()
            except Exception:
                LOGGER.debug("释放 ROS2 上下文失败。", exc_info=True)
        self._node = None
        self._executor = None
        self._thread = None
        self._rclpy = None
        self._subscriptions = []
        self._has_ros2_context = False

    def is_available(self) -> bool:
        return bool(self._started and self._ros_ready and self._last_error is None)

    def is_map_available(self) -> bool:
        return self.get_occupancy_grid() is not None or self.get_cost_map() is not None

    def get_occupancy_grid(self) -> Optional[OccupancyGrid]:
        with self._lock:
            return (
                self._latest_occupancy_grid.model_copy(deep=True)
                if self._latest_occupancy_grid is not None
                else None
            )

    def get_cost_map(self) -> Optional[CostMap]:
        with self._lock:
            return self._latest_cost_map.model_copy(deep=True) if self._latest_cost_map is not None else None

    def get_semantic_map(self) -> Optional[SemanticMap]:
        return None

    def activate_platform_map_version(self, **_: Any) -> Dict[str, object]:
        return {
            "backend_switch_applied": False,
            "live_geometry_active": self.is_map_available(),
            "reason": "live_ros2_map_provider_observes_current_map_only",
            "provider_status": self.get_status(),
        }

    def get_status(self) -> Dict[str, object]:
        occupancy_grid = self.get_occupancy_grid()
        cost_map = self.get_cost_map()
        return {
            "provider_name": self.provider_name,
            "provider_version": self.provider_version,
            "started": self._started,
            "ros_ready": self._ros_ready,
            "occupancy_topic": self.occupancy_topic,
            "cost_map_topic": self.cost_map_topic or None,
            "occupancy_message_count": self._occupancy_message_count,
            "cost_map_message_count": self._cost_map_message_count,
            "occupancy_grid_ready": occupancy_grid is not None,
            "cost_map_ready": cost_map is not None,
            "occupancy_grid_size": (
                {"width": occupancy_grid.width, "height": occupancy_grid.height}
                if occupancy_grid is not None
                else None
            ),
            "latest_occupancy_timestamp": occupancy_grid.timestamp.isoformat() if occupancy_grid is not None else None,
            "latest_cost_map_timestamp": cost_map.timestamp.isoformat() if cost_map is not None else None,
            "last_error": self._last_error,
        }

    def _start_ros2_subscriptions(self) -> None:
        from nav_msgs.msg import OccupancyGrid as RosOccupancyGrid
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        node = None
        executor = None
        try:
            rclpy = acquire_ros2_context()
            self._has_ros2_context = True
            node = rclpy.create_node("nuwax_live_map_provider")
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            qos_profiles = (
                QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
                QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=5,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )
            for qos_profile in qos_profiles:
                self._subscriptions.append(
                    node.create_subscription(RosOccupancyGrid, self.occupancy_topic, self._on_occupancy_grid, qos_profile)
                )
                if self.cost_map_topic:
                    self._subscriptions.append(
                        node.create_subscription(RosOccupancyGrid, self.cost_map_topic, self._on_cost_map, qos_profile)
                    )
            self._rclpy = rclpy
            self._node = node
            self._executor = executor
            self._thread = threading.Thread(target=self._spin, name="nuwax_ros2_live_map_provider", daemon=True)
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
                    LOGGER.debug("清理 ROS2 实时地图节点失败。", exc_info=True)
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
            LOGGER.warning("ROS2 实时地图订阅循环异常：%s", exc)

    def _on_occupancy_grid(self, message: Any) -> None:
        try:
            occupancy_grid = self._adapter.adapt(self._build_grid_payload(message, map_id="live_occupancy"))
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
            LOGGER.debug("转换 ROS2 占据栅格失败。", exc_info=True)
            return
        with self._lock:
            self._latest_occupancy_grid = occupancy_grid
            self._occupancy_message_count += 1
            self._last_error = None

    def _on_cost_map(self, message: Any) -> None:
        try:
            payload = self._build_grid_payload(message, map_id="live_cost_map")
            cost_map = CostMap(
                timestamp=payload["timestamp"],
                map_id=str(payload["map_id"]),
                frame_id=str(payload["frame_id"]),
                width=int(payload["width"]),
                height=int(payload["height"]),
                resolution_m=float(payload["resolution_m"]),
                origin=build_pose(payload.get("origin") or {}, frame_id=str(payload["frame_id"]), timestamp=payload["timestamp"]),
                data=[float(value) for value in payload["data"]],
            )
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
            LOGGER.debug("转换 ROS2 代价地图失败。", exc_info=True)
            return
        with self._lock:
            self._latest_cost_map = cost_map
            self._cost_map_message_count += 1
            self._last_error = None

    def _build_grid_payload(self, message: Any, *, map_id: str) -> Dict[str, Any]:
        header = getattr(message, "header", None)
        info = getattr(message, "info", None)
        if info is None:
            raise ValueError("ROS2 OccupancyGrid 缺少 info 字段。")
        frame_id = str(getattr(header, "frame_id", "") or self.config.frames.map_frame or "map")
        return {
            "timestamp": self._timestamp_from_header(header),
            "map_id": map_id,
            "frame_id": frame_id,
            "width": int(getattr(info, "width")),
            "height": int(getattr(info, "height")),
            "resolution_m": float(getattr(info, "resolution")),
            "origin": self._pose_payload(getattr(info, "origin", None)),
            "data": [int(value) for value in list(getattr(message, "data", []) or [])],
        }

    def _timestamp_from_header(self, header: Any) -> datetime:
        stamp = getattr(header, "stamp", None)
        if stamp is None:
            return datetime.now(timezone.utc)
        seconds = int(getattr(stamp, "sec", 0) or 0)
        nanoseconds = int(getattr(stamp, "nanosec", 0) or 0)
        if seconds <= 0 and nanoseconds <= 0:
            return datetime.now(timezone.utc)
        return datetime.fromtimestamp(seconds + nanoseconds / 1_000_000_000.0, tz=timezone.utc)

    def _pose_payload(self, pose: Any) -> Dict[str, object]:
        position = getattr(pose, "position", None)
        orientation = getattr(pose, "orientation", None)
        return {
            "position": {
                "x": float(getattr(position, "x", 0.0) or 0.0),
                "y": float(getattr(position, "y", 0.0) or 0.0),
                "z": float(getattr(position, "z", 0.0) or 0.0),
            },
            "orientation": {
                "x": float(getattr(orientation, "x", 0.0) or 0.0),
                "y": float(getattr(orientation, "y", 0.0) or 0.0),
                "z": float(getattr(orientation, "z", 0.0) or 0.0),
                "w": float(getattr(orientation, "w", 1.0) if orientation is not None else 1.0),
            },
        }
