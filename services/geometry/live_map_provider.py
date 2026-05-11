from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import logging
import threading
from typing import Any, Callable, Dict, List, Optional

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
        self._latest_occupancy_metrics: Dict[str, object] = {}
        self._latest_cost_map_metrics: Dict[str, object] = {}
        self._latest_occupancy_received_at: Optional[datetime] = None
        self._latest_cost_map_received_at: Optional[datetime] = None
        self._occupancy_content_revision = 0
        self._cost_map_content_revision = 0
        self._update_sequence = 0
        self._map_update_subscribers: Dict[int, Callable[[Dict[str, object]], None]] = {}
        self._next_map_update_subscription_id = 1
        self._last_occupancy_content_hash: Optional[str] = None
        self._last_cost_map_content_hash: Optional[str] = None
        self._last_occupancy_known_cell_count = 0
        self._last_cost_map_known_cell_count = 0
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

    def subscribe_map_updates(self, callback: Callable[[Dict[str, object]], None]) -> int:
        with self._lock:
            subscription_id = self._next_map_update_subscription_id
            self._next_map_update_subscription_id += 1
            self._map_update_subscribers[subscription_id] = callback
            return subscription_id

    def unsubscribe_map_updates(self, subscription_id: int) -> bool:
        with self._lock:
            return self._map_update_subscribers.pop(subscription_id, None) is not None

    def activate_platform_map_version(self, **_: Any) -> Dict[str, object]:
        return {
            "backend_switch_applied": False,
            "live_geometry_active": self.is_map_available(),
            "reason": "live_ros2_map_provider_observes_current_map_only",
            "provider_status": self.get_status(),
        }

    def get_status(self) -> Dict[str, object]:
        with self._lock:
            occupancy_grid = (
                self._latest_occupancy_grid.model_copy(deep=True)
                if self._latest_occupancy_grid is not None
                else None
            )
            cost_map = self._latest_cost_map.model_copy(deep=True) if self._latest_cost_map is not None else None
            occupancy_metrics = dict(self._latest_occupancy_metrics)
            cost_map_metrics = dict(self._latest_cost_map_metrics)
            occupancy_count = self._occupancy_message_count
            cost_map_count = self._cost_map_message_count
            occupancy_revision = self._occupancy_content_revision
            cost_map_revision = self._cost_map_content_revision
            update_sequence = self._update_sequence
            occupancy_received_at = self._latest_occupancy_received_at
            cost_map_received_at = self._latest_cost_map_received_at
            last_error = self._last_error
        now = datetime.now(timezone.utc)
        occupancy_publisher_count = self._publisher_count(self.occupancy_topic)
        cost_map_publisher_count = self._publisher_count(self.cost_map_topic) if self.cost_map_topic else None
        rtabmap_info_publisher_count = self._publisher_count("/rtabmap/info")
        latest_occupancy_age_sec = (
            max(0.0, (now - occupancy_grid.timestamp).total_seconds()) if occupancy_grid is not None else None
        )
        latest_cost_map_age_sec = (
            max(0.0, (now - cost_map.timestamp).total_seconds()) if cost_map is not None else None
        )
        latest_occupancy_cache_age_sec = (
            max(0.0, (now - occupancy_received_at).total_seconds()) if occupancy_received_at is not None else None
        )
        latest_cost_map_cache_age_sec = (
            max(0.0, (now - cost_map_received_at).total_seconds()) if cost_map_received_at is not None else None
        )
        occupancy_time_skew_sec = (
            (occupancy_received_at - occupancy_grid.timestamp).total_seconds()
            if occupancy_grid is not None and occupancy_received_at is not None
            else None
        )
        cost_map_time_skew_sec = (
            (cost_map_received_at - cost_map.timestamp).total_seconds()
            if cost_map is not None and cost_map_received_at is not None
            else None
        )
        expected_frame_id = str(self.config.frames.map_frame or "").strip() or None
        occupancy_frame_mismatch = bool(
            occupancy_grid is not None
            and expected_frame_id
            and str(occupancy_grid.frame_id or "").strip()
            and str(occupancy_grid.frame_id or "").strip() != expected_frame_id
        )
        cost_map_frame_mismatch = bool(
            cost_map is not None
            and expected_frame_id
            and str(cost_map.frame_id or "").strip()
            and str(cost_map.frame_id or "").strip() != expected_frame_id
        )
        provider_status = "available" if self.is_available() else ("error" if last_error else "starting" if self._started else "stopped")
        return {
            "provider_name": self.provider_name,
            "provider_version": self.provider_version,
            "started": self._started,
            "ros_ready": self._ros_ready,
            "occupancy_topic": self.occupancy_topic,
            "cost_map_topic": self.cost_map_topic or None,
            "occupancy_publisher_count": occupancy_publisher_count,
            "cost_map_publisher_count": cost_map_publisher_count,
            "rtabmap_info_topic": "/rtabmap/info",
            "rtabmap_info_publisher_count": rtabmap_info_publisher_count,
            "occupancy_has_publisher": occupancy_publisher_count is not None and occupancy_publisher_count > 0,
            "cost_map_has_publisher": cost_map_publisher_count is not None and cost_map_publisher_count > 0,
            "rtabmap_info_has_publisher": rtabmap_info_publisher_count is not None and rtabmap_info_publisher_count > 0,
            "provider_status": provider_status,
            "expected_frame_id": expected_frame_id,
            "occupancy_message_count": occupancy_count,
            "cost_map_message_count": cost_map_count,
            "message_count": occupancy_count,
            "occupancy_sequence": occupancy_count,
            "cost_map_sequence": cost_map_count,
            "update_sequence": update_sequence,
            "occupancy_revision": occupancy_revision,
            "cost_map_revision": cost_map_revision,
            "occupancy_grid_ready": occupancy_grid is not None,
            "cost_map_ready": cost_map is not None,
            "occupancy_grid_size": (
                {"width": occupancy_grid.width, "height": occupancy_grid.height}
                if occupancy_grid is not None
                else None
            ),
            "latest_occupancy_frame_id": occupancy_grid.frame_id if occupancy_grid is not None else None,
            "latest_cost_map_frame_id": cost_map.frame_id if cost_map is not None else None,
            "latest_occupancy_timestamp": occupancy_grid.timestamp.isoformat() if occupancy_grid is not None else None,
            "latest_cost_map_timestamp": cost_map.timestamp.isoformat() if cost_map is not None else None,
            "latest_occupancy_source_timestamp": occupancy_grid.timestamp.isoformat() if occupancy_grid is not None else None,
            "latest_cost_map_source_timestamp": cost_map.timestamp.isoformat() if cost_map is not None else None,
            "latest_occupancy_received_at": occupancy_received_at.isoformat() if occupancy_received_at is not None else None,
            "latest_cost_map_received_at": cost_map_received_at.isoformat() if cost_map_received_at is not None else None,
            "last_message_at": occupancy_received_at.isoformat() if occupancy_received_at is not None else None,
            "latest_occupancy_age_sec": latest_occupancy_age_sec,
            "latest_cost_map_age_sec": latest_cost_map_age_sec,
            "latest_occupancy_source_age_sec": latest_occupancy_age_sec,
            "latest_cost_map_source_age_sec": latest_cost_map_age_sec,
            "latest_occupancy_cache_age_sec": latest_occupancy_cache_age_sec,
            "latest_cost_map_cache_age_sec": latest_cost_map_cache_age_sec,
            "occupancy_time_skew_sec": occupancy_time_skew_sec,
            "cost_map_time_skew_sec": cost_map_time_skew_sec,
            "time_skew_sec": occupancy_time_skew_sec,
            "occupancy_frame_mismatch": occupancy_frame_mismatch,
            "cost_map_frame_mismatch": cost_map_frame_mismatch,
            "frame_mismatch": occupancy_frame_mismatch or cost_map_frame_mismatch,
            "occupancy_metrics": occupancy_metrics,
            "cost_map_metrics": cost_map_metrics,
            "last_error": last_error,
        }

    def _publisher_count(self, topic: str) -> Optional[int]:
        if not topic or self._node is None:
            return None
        try:
            return len(self._node.get_publishers_info_by_topic(topic))
        except Exception:
            LOGGER.debug("读取 ROS2 topic publisher 失败 topic=%s", topic, exc_info=True)
            return None

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
        metrics = self._build_grid_metrics(occupancy_grid.data)
        with self._lock:
            previous_hash = self._last_occupancy_content_hash
            previous_known_count = self._last_occupancy_known_cell_count
            known_count = int(metrics.get("known_cell_count") or 0)
            content_hash = str(metrics.get("content_hash") or "")
            known_delta = known_count - previous_known_count if previous_hash is not None else known_count
            content_changed = previous_hash is None or content_hash != previous_hash or known_delta != 0
            if content_changed:
                self._occupancy_content_revision += 1
            self._occupancy_message_count += 1
            metrics = {
                **metrics,
                "content_revision": self._occupancy_content_revision,
                "known_cell_delta": known_delta,
                "message_count": self._occupancy_message_count,
            }
            self._latest_occupancy_grid = occupancy_grid
            self._latest_occupancy_metrics = metrics
            self._latest_occupancy_received_at = datetime.now(timezone.utc)
            self._last_occupancy_content_hash = content_hash
            self._last_occupancy_known_cell_count = known_count
            self._last_error = None
            payload = self._build_map_update_payload_locked(
                source_layer="occupancy",
                event_type="content_update" if content_changed else "heartbeat",
            )
            subscribers = list(self._map_update_subscribers.items())
        self._notify_map_update_subscribers(subscribers, payload)

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
        metrics = self._build_grid_metrics([int(value) for value in cost_map.data])
        with self._lock:
            previous_hash = self._last_cost_map_content_hash
            previous_known_count = self._last_cost_map_known_cell_count
            known_count = int(metrics.get("known_cell_count") or 0)
            content_hash = str(metrics.get("content_hash") or "")
            costmap_delta = known_count - previous_known_count if previous_hash is not None else known_count
            content_changed = previous_hash is None or content_hash != previous_hash or costmap_delta != 0
            if content_changed:
                self._cost_map_content_revision += 1
            self._cost_map_message_count += 1
            metrics = {
                **metrics,
                "content_revision": self._cost_map_content_revision,
                "known_cell_delta": costmap_delta,
                "costmap_delta": costmap_delta,
                "message_count": self._cost_map_message_count,
            }
            self._latest_cost_map = cost_map
            self._latest_cost_map_metrics = metrics
            self._latest_cost_map_received_at = datetime.now(timezone.utc)
            self._last_cost_map_content_hash = content_hash
            self._last_cost_map_known_cell_count = known_count
            self._last_error = None
            payload = self._build_map_update_payload_locked(
                source_layer="cost_map",
                event_type="content_update" if content_changed else "heartbeat",
            )
            subscribers = list(self._map_update_subscribers.items())
        self._notify_map_update_subscribers(subscribers, payload)

    def _build_map_update_payload_locked(self, *, source_layer: str, event_type: str) -> Dict[str, object]:
        self._update_sequence += 1
        include_layers = event_type == "content_update"
        occupancy_grid = (
            self._latest_occupancy_grid.model_copy(deep=True)
            if include_layers and self._latest_occupancy_grid is not None
            else None
        )
        cost_map = (
            self._latest_cost_map.model_copy(deep=True)
            if include_layers and self._latest_cost_map is not None
            else None
        )
        latest_occupancy = self._latest_occupancy_grid
        latest_cost_map = self._latest_cost_map
        occupancy_metrics = dict(self._latest_occupancy_metrics)
        cost_map_metrics = dict(self._latest_cost_map_metrics)
        metrics = occupancy_metrics if source_layer == "occupancy" else cost_map_metrics
        occupancy_revision = self._occupancy_content_revision
        cost_map_revision = self._cost_map_content_revision
        occupancy_hash = occupancy_metrics.get("content_hash")
        cost_map_hash = cost_map_metrics.get("content_hash")
        frame_id = None
        if latest_occupancy is not None:
            frame_id = latest_occupancy.frame_id
        elif latest_cost_map is not None:
            frame_id = latest_cost_map.frame_id
        metadata = {
            "event_type": event_type,
            "source_kind": "platform_live_map_provider",
            "map_snapshot_kind": "live_streaming",
            "live_streaming_ready": True,
            "rtabmap_live_streaming_ready": True,
            "source_alive": True,
            "frame_id": frame_id,
            "update_sequence": self._update_sequence,
            "source_layer": source_layer,
            "occupancy_sequence": self._occupancy_message_count,
            "cost_map_sequence": self._cost_map_message_count,
            "message_count": self._occupancy_message_count + self._cost_map_message_count,
            "occupancy_content_hash": occupancy_hash,
            "cost_map_content_hash": cost_map_hash,
            "content_hash": occupancy_hash,
            "occupancy_content_revision": occupancy_revision,
            "cost_map_content_revision": cost_map_revision,
            "content_revision": occupancy_revision + cost_map_revision,
            "occupancy_metrics": occupancy_metrics,
            "cost_map_metrics": cost_map_metrics,
            "metrics": metrics,
        }
        return {
            "source_name": self.provider_name,
            "occupancy_grid": occupancy_grid,
            "cost_map": cost_map,
            "semantic_map": None,
            "metadata": metadata,
        }

    def _notify_map_update_subscribers(
        self,
        subscribers: List[tuple[int, Callable[[Dict[str, object]], None]]],
        payload: Dict[str, object],
    ) -> None:
        for subscription_id, callback in subscribers:
            try:
                callback(payload)
            except Exception:
                LOGGER.debug("实时地图更新订阅回调失败 subscription_id=%s", subscription_id, exc_info=True)

    def _build_grid_metrics(self, data: List[int]) -> Dict[str, object]:
        unknown_count = 0
        free_count = 0
        occupied_count = 0
        digest = hashlib.sha256()
        for value in data:
            int_value = int(value)
            digest.update(int_value.to_bytes(2, byteorder="little", signed=True))
            if int_value < 0:
                unknown_count += 1
            elif int_value == 0:
                free_count += 1
            elif int_value >= 50:
                occupied_count += 1
        known_count = max(0, len(data) - unknown_count)
        return {
            "cell_count": len(data),
            "known_cell_count": known_count,
            "unknown_cell_count": unknown_count,
            "free_cell_count": free_count,
            "occupied_cell_count": occupied_count,
            "content_hash": digest.hexdigest(),
        }

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
