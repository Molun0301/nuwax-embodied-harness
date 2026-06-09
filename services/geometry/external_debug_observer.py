from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import logging
import math
import os
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from contracts.geometry import Pose, Quaternion, Vector3
from contracts.maps import OccupancyGrid
from services.geometry.config import PlatformGeometryConfig
from services.geometry.ros2_context import acquire_ros2_context, release_ros2_context

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.external_debug_observer")

_DUPLICATE_MAP_SUPPRESS_SEC = 0.2
_GOAL_FRESH_SEC = 10.0
_POSE_EMIT_MIN_INTERVAL_SEC = 0.08
_POSE_TIMER_PERIOD_SEC = 0.1


class ExternalGeometryDebugObserver:
    """Read-only ROS2 observer for externally launched GO2 RTAB-Map/Nav2 debug data."""

    def __init__(
        self,
        *,
        config: PlatformGeometryConfig,
        map_callback: Callable[[Dict[str, object]], None],
        frontier_callback: Callable[[Dict[str, object]], None],
        pose_callback: Optional[Callable[[Pose, Optional[object], Dict[str, object]], None]] = None,
    ) -> None:
        self.config = config
        self._map_callback = map_callback
        self._frontier_callback = frontier_callback
        self._pose_callback = pose_callback
        self._lock = threading.RLock()
        self._started = False
        self._has_ros2_context = False
        self._node = None
        self._executor = None
        self._thread: Optional[threading.Thread] = None
        self._rclpy = None
        self._subscriptions: List[Any] = []
        self._pose_timer = None
        self._tf_buffer = None
        self._tf_listener = None
        self._last_error: Optional[str] = None
        self._pose_sequence = 0
        self._last_pose_emit_at = 0.0
        self._map_message_count = 0
        self._last_map_signature: Optional[Tuple[object, ...]] = None
        self._last_map_signature_at = 0.0
        self._frontier_sequence = 0
        self._last_goal: Optional[Dict[str, object]] = None
        self._last_goal_at = 0.0
        self._last_status_text: Optional[str] = None
        self._last_terminal_status: Optional[str] = None
        self._last_planned_path: List[Dict[str, float]] = []
        self._last_plan_at = 0.0
        self._last_plan_emit_at = 0.0

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._started

    @property
    def last_error(self) -> Optional[str]:
        with self._lock:
            return self._last_error

    def start(self) -> None:
        with self._lock:
            if self._started:
                return
            try:
                from geometry_msgs.msg import PoseStamped
                from nav_msgs.msg import OccupancyGrid as RosOccupancyGrid
                from rclpy.executors import SingleThreadedExecutor
                from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
                from tf2_ros import Buffer, TransformListener
                from visualization_msgs.msg import MarkerArray

                os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
                cyclonedds_xml = os.path.expanduser("~/cyclonedds_ws/cyclonedds.xml")
                if os.path.exists(cyclonedds_xml):
                    os.environ["CYCLONEDDS_URI"] = cyclonedds_xml

                rclpy = acquire_ros2_context()
                self._has_ros2_context = True
                self._rclpy = rclpy
                self._node = rclpy.create_node("nuwax_external_geometry_debug_observer")
                self._executor = SingleThreadedExecutor()
                self._executor.add_node(self._node)

                map_topic = str(self.config.topics.occupancy_topic or "/map")
                frontier_topic = str(getattr(self.config.external, "frontier_topic", "") or "/frontiers")
                next_goal_topic = str(getattr(self.config.external, "next_goal_topic", "") or "/exploration/next_goal")
                plan_topic = str(
                    getattr(self.config.external, "plan_topic", "")
                    or os.environ.get("NUWAX_GEOMETRY_PLAN_TOPIC", "/plan")
                )

                reliable_volatile = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                )
                reliable_transient = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )

                self._subscriptions.append(
                    self._node.create_subscription(RosOccupancyGrid, map_topic, self._on_map, reliable_volatile)
                )
                try:
                    self._subscriptions.append(
                        self._node.create_subscription(RosOccupancyGrid, map_topic, self._on_map, reliable_transient)
                    )
                except Exception:
                    LOGGER.debug("创建 transient /map 订阅失败，保留 volatile 订阅。", exc_info=True)
                self._subscriptions.append(
                    self._node.create_subscription(MarkerArray, frontier_topic, self._on_frontiers, reliable_volatile)
                )
                self._subscriptions.append(
                    self._node.create_subscription(PoseStamped, next_goal_topic, self._on_next_goal, reliable_volatile)
                )
                try:
                    from nav_msgs.msg import Path as RosPath

                    self._subscriptions.append(
                        self._node.create_subscription(RosPath, plan_topic, self._on_plan, reliable_volatile)
                    )
                except Exception:
                    LOGGER.warning("无法订阅 Nav2 全局规划路径 topic=%s", plan_topic, exc_info=True)
                if self._pose_callback is not None:
                    self._tf_buffer = Buffer()
                    self._tf_listener = TransformListener(self._tf_buffer, self._node)
                    self._pose_timer = self._node.create_timer(_POSE_TIMER_PERIOD_SEC, self._on_pose_timer)
                self._thread = threading.Thread(
                    target=self._executor.spin,
                    name="nuwax_external_geometry_debug_observer",
                    daemon=True,
                )
                self._thread.start()
                self._started = True
                self._last_error = None
                LOGGER.info(
                    "GO2 外部几何 debug observer 已启动 map=%s frontiers=%s next_goal=%s plan=%s pose=%s",
                    map_topic,
                    frontier_topic,
                    next_goal_topic,
                    plan_topic,
                    "enabled" if self._pose_callback is not None else "disabled",
                )
            except Exception as exc:
                self._last_error = str(exc)
                self._cleanup_locked()
                LOGGER.warning("GO2 外部几何 debug observer 启动失败：%s", exc)

    def stop(self) -> None:
        with self._lock:
            self._cleanup_locked()

    def _cleanup_locked(self) -> None:
        executor = self._executor
        node = self._node
        self._started = False
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=0.0)
            except TypeError:
                executor.shutdown()
            except Exception:
                LOGGER.debug("停止 GO2 debug observer executor 失败。", exc_info=True)
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                LOGGER.debug("销毁 GO2 debug observer 节点失败。", exc_info=True)
        if self._has_ros2_context:
            try:
                release_ros2_context()
            except Exception:
                LOGGER.debug("释放 GO2 debug observer ROS2 上下文失败。", exc_info=True)
        self._node = None
        self._executor = None
        self._thread = None
        self._subscriptions = []
        self._pose_timer = None
        self._tf_buffer = None
        self._tf_listener = None
        self._has_ros2_context = False
        self._rclpy = None

    def _on_pose_timer(self) -> None:
        if self._pose_callback is None or self._tf_buffer is None:
            return
        map_frame = self._normalize_frame_id(self.config.frames.map_frame or "map", "map")
        base_frame = self._normalize_frame_id(self.config.frames.base_frame or "base_link", "base_link")
        try:
            from rclpy.time import Time
            from tf2_ros import TransformException

            transform = self._tf_buffer.lookup_transform(map_frame, base_frame, Time())
        except TransformException:
            return
        except Exception:
            LOGGER.debug("GO2 debug observer TF 查询失败。", exc_info=True)
            return
        try:
            pose = self._pose_from_transform(transform, map_frame)
        except Exception:
            LOGGER.debug("GO2 debug observer 位姿转换失败。", exc_info=True)
            return
        self._emit_pose(
            pose,
            {
                "source_kind": "external_tf_observer",
                "tf_target": base_frame,
                "tf_source": map_frame,
            },
        )

    def _emit_pose(self, pose: Pose, metadata: Dict[str, object]) -> None:
        if self._pose_callback is None:
            return
        now_monotonic = time.monotonic()
        if now_monotonic - self._last_pose_emit_at < _POSE_EMIT_MIN_INTERVAL_SEC:
            return
        self._last_pose_emit_at = now_monotonic
        with self._lock:
            self._pose_sequence += 1
            sequence = self._pose_sequence
        payload = {
            "source_name": "go2_rtabmap_debug_observer",
            "source_kind": "external_tf_observer",
            "sequence": sequence,
            **metadata,
        }
        try:
            self._pose_callback(pose, None, payload)
        except Exception:
            LOGGER.debug("投递 GO2 位姿 debug 帧失败。", exc_info=True)

    @staticmethod
    def _normalize_frame_id(frame_id: str, default: str) -> str:
        value = str(frame_id or "").strip().lstrip("/") or default
        return value

    def _pose_from_transform(self, transform: Any, frame_id: str) -> Pose:
        translation = getattr(getattr(transform, "transform", None), "translation", None)
        rotation = getattr(getattr(transform, "transform", None), "rotation", None)
        if translation is None or rotation is None:
            raise ValueError("transform missing translation or rotation")
        header = getattr(transform, "header", None)
        source_timestamp = self._timestamp_from_header(header)
        received_at = datetime.now(timezone.utc)
        return Pose(
            timestamp=source_timestamp or received_at,
            frame_id=self._normalize_frame_id(frame_id, "map"),
            position=Vector3(
                x=float(getattr(translation, "x", 0.0) or 0.0),
                y=float(getattr(translation, "y", 0.0) or 0.0),
                z=float(getattr(translation, "z", 0.0) or 0.0),
            ),
            orientation=Quaternion(
                x=float(getattr(rotation, "x", 0.0) or 0.0),
                y=float(getattr(rotation, "y", 0.0) or 0.0),
                z=float(getattr(rotation, "z", 0.0) or 0.0),
                w=float(getattr(rotation, "w", 1.0) or 1.0),
            ),
        )

    def _on_map(self, msg: Any) -> None:
        now_monotonic = time.monotonic()
        signature = self._map_signature(msg)
        with self._lock:
            if (
                signature == self._last_map_signature
                and now_monotonic - self._last_map_signature_at <= _DUPLICATE_MAP_SUPPRESS_SEC
            ):
                return
            self._last_map_signature = signature
            self._last_map_signature_at = now_monotonic
            self._map_message_count += 1
            message_count = self._map_message_count
        try:
            payload = self._map_payload_from_msg(msg, message_count=message_count)
        except Exception:
            LOGGER.debug("转换 GO2 /map debug 帧失败。", exc_info=True)
            return
        try:
            self._map_callback(payload)
        except Exception:
            LOGGER.debug("投递 GO2 /map debug 帧失败。", exc_info=True)

    def _map_payload_from_msg(self, msg: Any, *, message_count: int) -> Dict[str, object]:
        header = getattr(msg, "header", None)
        info = getattr(msg, "info", None)
        if info is None:
            raise ValueError("OccupancyGrid.info missing")
        width = int(getattr(info, "width", 0) or 0)
        height = int(getattr(info, "height", 0) or 0)
        resolution = float(getattr(info, "resolution", 0.0) or 0.0)
        data = [int(value) for value in list(getattr(msg, "data", []) or [])]
        if width <= 0 or height <= 0 or resolution <= 0.0 or len(data) != width * height:
            raise ValueError("invalid occupancy grid dimensions")
        frame_id = str(getattr(header, "frame_id", "") or self.config.frames.map_frame or "map")
        origin_msg = getattr(info, "origin", None)
        position_msg = getattr(origin_msg, "position", None)
        orientation_msg = getattr(origin_msg, "orientation", None)
        source_timestamp = self._timestamp_from_header(header)
        received_at = datetime.now(timezone.utc)
        known_count = sum(1 for value in data if value >= 0)
        occupied_count = sum(1 for value in data if value >= 50)
        content_hash = hashlib.blake2s(bytes((value + 1) & 0xFF for value in data), digest_size=8).hexdigest()
        occupancy_grid = OccupancyGrid(
            timestamp=source_timestamp or received_at,
            map_id="go2_rtabmap:%s:%d" % (self.config.topics.occupancy_topic or "/map", message_count),
            frame_id=frame_id,
            width=width,
            height=height,
            resolution_m=resolution,
            origin=Pose(
                timestamp=source_timestamp or received_at,
                frame_id=frame_id,
                position=Vector3(
                    x=float(getattr(position_msg, "x", 0.0) or 0.0),
                    y=float(getattr(position_msg, "y", 0.0) or 0.0),
                    z=float(getattr(position_msg, "z", 0.0) or 0.0),
                ),
                orientation=Quaternion(
                    x=float(getattr(orientation_msg, "x", 0.0) or 0.0),
                    y=float(getattr(orientation_msg, "y", 0.0) or 0.0),
                    z=float(getattr(orientation_msg, "z", 0.0) or 0.0),
                    w=float(getattr(orientation_msg, "w", 1.0) or 1.0),
                ),
            ),
            data=data,
        )
        metadata = {
            "event_type": "content_update",
            "source_kind": "platform_live_map_provider",
            "source_name": "go2_rtabmap_debug_observer",
            "source_topic": str(self.config.topics.occupancy_topic or "/map"),
            "map_snapshot_kind": "live_streaming",
            "live_streaming_ready": True,
            "rtabmap_live_streaming_ready": True,
            "source_alive": True,
            "streaming": True,
            "frame_id": frame_id,
            "message_count": message_count,
            "occupancy_message_count": message_count,
            "occupancy_sequence": message_count,
            "update_sequence": message_count,
            "occupancy_content_revision": message_count,
            "content_revision": message_count,
            "occupancy_content_hash": content_hash,
            "content_hash": content_hash,
            "known_cell_count": known_count,
            "occupied_cell_count": occupied_count,
            "last_message_at": received_at.isoformat(),
            "latest_occupancy_received_at": received_at.isoformat(),
            "latest_occupancy_timestamp": (source_timestamp or received_at).isoformat(),
            "latest_occupancy_source_timestamp": (source_timestamp or received_at).isoformat(),
            "occupancy_metrics": {
                "content_hash": content_hash,
                "content_revision": message_count,
                "known_cell_count": known_count,
                "occupied_cell_count": occupied_count,
            },
        }
        return {
            "source_name": "go2_rtabmap_debug_observer",
            "occupancy_grid": occupancy_grid,
            "cost_map": None,
            "semantic_map": None,
            "width": width,
            "height": height,
            "resolution": resolution,
            "frame_id": frame_id,
            "revision": message_count,
            "metadata": metadata,
        }

    def _on_frontiers(self, msg: Any) -> None:
        status = self._status_from_marker_array(msg)
        goal = self._goal_from_marker_array(msg)
        with self._lock:
            terminal = bool(status and status.get("terminal"))
            if status is not None:
                self._last_status_text = status.get("text")
                if terminal:
                    self._last_terminal_status = str(status.get("terminal_status") or "succeeded")
                    self._last_goal = None
                    self._last_goal_at = 0.0
            if goal is not None and not terminal:
                self._last_goal = goal
                self._last_goal_at = time.monotonic()
            elif status is None and goal is None and self._last_terminal_status is None:
                return
            terminal_status = self._last_terminal_status
        if terminal_status is not None:
            self._emit_frontier_payload()
            return
        self._emit_frontier_payload()

    def _on_next_goal(self, msg: Any) -> None:
        goal = self._frontier_from_pose_stamped(msg, role="active_goal", source="go2_next_goal")
        if goal is None:
            return
        with self._lock:
            self._last_goal = goal
            self._last_goal_at = time.monotonic()
        self._emit_frontier_payload()

    def _on_plan(self, msg: Any) -> None:
        path = self._planned_path_from_ros(msg)
        now = time.monotonic()
        with self._lock:
            self._last_planned_path = path
            self._last_plan_at = now
        if now - self._last_plan_emit_at >= 0.1:
            self._last_plan_emit_at = now
            self._emit_frontier_payload()

    @staticmethod
    def _planned_path_from_ros(msg: Any) -> List[Dict[str, float]]:
        points: List[Dict[str, float]] = []
        for pose_stamped in list(getattr(msg, "poses", []) or []):
            pose = getattr(pose_stamped, "pose", None)
            position = getattr(pose, "position", None)
            if position is None:
                continue
            x = float(getattr(position, "x", 0.0) or 0.0)
            y = float(getattr(position, "y", 0.0) or 0.0)
            z = float(getattr(position, "z", 0.0) or 0.0)
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            points.append({"x": x, "y": y, "z": z})
        return points

    def _emit_frontier_payload(self) -> None:
        with self._lock:
            current = self._last_goal if self._last_goal_at and time.monotonic() - self._last_goal_at <= _GOAL_FRESH_SEC else None
            frontiers = [current] if current is not None else []
            status_text = self._last_status_text
            terminal_status = self._last_terminal_status
            planned_path = list(self._last_planned_path)
            plan_fresh = self._last_plan_at and time.monotonic() - self._last_plan_at <= 5.0
            self._frontier_sequence += 1
            sequence = self._frontier_sequence
        now = datetime.now(timezone.utc)
        current_goal = None
        if current is not None:
            pose = current.get("pose") if isinstance(current.get("pose"), dict) else None
            if pose is not None:
                current_goal = {"target_pose": pose}
        payload = {
            "source_name": "go2_rtabmap_debug_observer",
            "source": "go2_rtabmap_debug_observer",
            "latest_frontiers": frontiers,
            "current_frontier": current,
            "frontier_count": len(frontiers),
            "planned_path": planned_path if plan_fresh else [],
            "current_goal": current_goal,
            "updated_at": now.isoformat(),
            "sequence": sequence,
            "metadata": {
                "source": "go2_rtabmap_debug_observer",
                "frontier_topic": "/frontiers",
                "next_goal_topic": "/exploration/next_goal",
                "plan_topic": os.environ.get("NUWAX_GEOMETRY_PLAN_TOPIC", "/plan"),
                "display_mode": "next_goal_only",
                "status_text": status_text,
                "exploration_terminal": terminal_status is not None,
                "terminal_status": terminal_status,
                "message_count": sequence,
                "planned_path": planned_path if plan_fresh else [],
                "plan_point_count": len(planned_path) if plan_fresh else 0,
                "plan_fresh": plan_fresh,
            },
        }
        try:
            self._frontier_callback(payload)
        except Exception:
            LOGGER.debug("投递 GO2 frontier debug 帧失败。", exc_info=True)

    def _status_from_marker_array(self, msg: Any) -> Optional[Dict[str, object]]:
        markers = list(getattr(msg, "markers", []) or [])
        for marker in markers:
            ns = str(getattr(marker, "ns", "") or "")
            if ns != "frontier_status":
                continue
            text = str(getattr(marker, "text", "") or "").strip()
            if not text:
                continue
            normalized = text.upper()
            done = (
                "DONE" in normalized
                or "COMPLETE" in normalized
                or "EXPLORATION FINISHED" in normalized
            )
            return {
                "text": text,
                "terminal": done,
                "terminal_status": "succeeded" if done else None,
            }
        return None

    def _goal_from_marker_array(self, msg: Any) -> Optional[Dict[str, object]]:
        markers = list(getattr(msg, "markers", []) or [])
        goal_candidates: List[Tuple[int, Any]] = []
        for index, marker in enumerate(markers):
            action = int(getattr(marker, "action", 0) or 0)
            if action == 3:  # DELETEALL
                continue
            ns = str(getattr(marker, "ns", "") or "")
            if ns == "active_frontier_goal":
                goal_candidates.insert(0, (index, marker))
            elif ns == "frontier_goal":
                goal_candidates.append((index, marker))
        if not goal_candidates:
            return None
        index, marker = goal_candidates[0]
        point = self._marker_point(marker)
        if point is None:
            return None
        x, y, z = point
        ns = str(getattr(marker, "ns", "") or "")
        marker_type = int(getattr(marker, "type", -1) or -1)
        frame_id = str(getattr(getattr(marker, "header", None), "frame_id", "") or self.config.frames.map_frame or "map")
        return self._frontier_from_xyz(
            x,
            y,
            z,
            frame_id=frame_id,
            role="active_goal" if ns == "active_frontier_goal" else "selected_goal",
            source="go2_frontier_goal_marker",
            frontier_id="go2_next_goal_marker",
            metadata={
                "marker_ns": ns,
                "marker_id": int(getattr(marker, "id", index) or index),
                "marker_type": marker_type,
            },
        )

    def _frontier_from_pose_stamped(self, msg: Any, *, role: str, source: str) -> Optional[Dict[str, object]]:
        pose_msg = getattr(msg, "pose", None)
        position = getattr(pose_msg, "position", None)
        if position is None:
            return None
        frame_id = str(getattr(getattr(msg, "header", None), "frame_id", "") or self.config.frames.map_frame or "map")
        orientation = getattr(pose_msg, "orientation", None)
        return self._frontier_from_xyz(
            float(getattr(position, "x", 0.0) or 0.0),
            float(getattr(position, "y", 0.0) or 0.0),
            float(getattr(position, "z", 0.0) or 0.0),
            frame_id=frame_id,
            role=role,
            source=source,
            frontier_id="go2_active_goal",
            orientation=orientation,
            metadata={},
        )

    def _frontier_from_xyz(
        self,
        x: float,
        y: float,
        z: float,
        *,
        frame_id: str,
        role: str,
        source: str,
        frontier_id: str,
        orientation: Optional[Any] = None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
            raise ValueError("frontier point must be finite")
        pose = {
            "frame_id": frame_id,
            "position": {"x": x, "y": y, "z": z},
            "orientation": {
                "x": float(getattr(orientation, "x", 0.0) or 0.0),
                "y": float(getattr(orientation, "y", 0.0) or 0.0),
                "z": float(getattr(orientation, "z", 0.0) or 0.0),
                "w": float(getattr(orientation, "w", 1.0) or 1.0),
            },
        }
        return {
            "frontier_id": frontier_id,
            "pose": pose,
            "geometry": {"type": "point", "coordinates": [x, y, z]},
            "size": 1,
            "cost": 0.0,
            "metadata": {"source": source, "role": role, **dict(metadata or {})},
        }

    @staticmethod
    def _marker_point(marker: Any) -> Optional[Tuple[float, float, float]]:
        points = list(getattr(marker, "points", []) or [])
        if points:
            point = points[-1]
            return (
                float(getattr(point, "x", 0.0) or 0.0),
                float(getattr(point, "y", 0.0) or 0.0),
                float(getattr(point, "z", 0.0) or 0.0),
            )
        pose = getattr(marker, "pose", None)
        position = getattr(pose, "position", None)
        if position is None:
            return None
        return (
            float(getattr(position, "x", 0.0) or 0.0),
            float(getattr(position, "y", 0.0) or 0.0),
            float(getattr(position, "z", 0.0) or 0.0),
        )

    @staticmethod
    def _timestamp_from_header(header: Any) -> Optional[datetime]:
        stamp = getattr(header, "stamp", None)
        if stamp is None:
            return None
        sec = int(getattr(stamp, "sec", 0) or 0)
        nanosec = int(getattr(stamp, "nanosec", 0) or 0)
        if sec <= 0 and nanosec <= 0:
            return None
        return datetime.fromtimestamp(sec + nanosec / 1_000_000_000.0, tz=timezone.utc)

    def _map_signature(self, msg: Any) -> Tuple[object, ...]:
        header = getattr(msg, "header", None)
        info = getattr(msg, "info", None)
        stamp = getattr(header, "stamp", None)
        origin = getattr(info, "origin", None)
        position = getattr(origin, "position", None)
        return (
            str(getattr(header, "frame_id", "") or ""),
            int(getattr(stamp, "sec", 0) or 0),
            int(getattr(stamp, "nanosec", 0) or 0),
            int(getattr(info, "width", 0) or 0),
            int(getattr(info, "height", 0) or 0),
            float(getattr(info, "resolution", 0.0) or 0.0),
            float(getattr(position, "x", 0.0) or 0.0),
            float(getattr(position, "y", 0.0) or 0.0),
        )
