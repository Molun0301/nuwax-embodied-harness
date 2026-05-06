from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import logging
import threading
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple

from pydantic import ValidationError

from contracts.geometry import FrameTree, Pose, Quaternion, Transform, Vector3
from services.geometry.config import PlatformGeometryConfig
from services.geometry.ros2_context import acquire_ros2_context, release_ros2_context
from services.localization.frame_tree_ops import resolve_frame_transform, transform_pose_to_frame

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.tf_localization_provider")


@dataclass(frozen=True)
class _StoredTransform:
    transform: Transform
    received_at: datetime
    source_topic: str
    static: bool = False


class Ros2TfLocalizationProvider:
    """把 ROS2 TF 树反哺为平台标准定位契约。"""

    provider_name = "platform_ros2_tf_localization"
    provider_version = "0.1.0"

    def __init__(self, config: PlatformGeometryConfig) -> None:
        self.config = config
        self.tf_topic = str(config.topics.tf_topic or "/tf")
        self.tf_static_topic = str(config.topics.tf_static_topic or "/tf_static")
        self.map_frame = self._normalize_frame_id(config.frames.map_frame) or "map"
        self.odom_frame = self._normalize_frame_id(config.frames.odom_frame) or "odom"
        self.base_frame = self._normalize_frame_id(config.frames.base_frame) or "body"
        self._max_tf_age_sec = max(0.05, float(config.rtabmap.timestamp_quality.max_tf_age_sec or 0.5))
        self._lock = threading.RLock()
        self._started = False
        self._ros_ready = False
        self._has_ros2_context = False
        self._last_error: Optional[str] = None
        self._dynamic_transforms: Dict[Tuple[str, str], _StoredTransform] = {}
        self._static_transforms: Dict[Tuple[str, str], _StoredTransform] = {}
        self._latest_pose: Optional[Pose] = None
        self._latest_frame_tree: Optional[FrameTree] = None
        self._latest_pose_received_at: Optional[datetime] = None
        self._latest_pose_source_timestamp: Optional[datetime] = None
        self._last_update_at: Optional[datetime] = None
        self._tf_message_count = 0
        self._tf_static_message_count = 0
        self._update_sequence = 0
        self._last_status: Dict[str, object] = {
            "tracking_state": "not_started",
            "map_to_base_ready": False,
        }
        self._localization_subscribers: Dict[int, Callable[[Pose, Optional[FrameTree], Dict[str, object]], None]] = {}
        self._next_localization_subscription_id = 1
        self._node = None
        self._executor = None
        self._thread: Optional[threading.Thread] = None
        self._rclpy = None
        self._subscriptions: List[Any] = []
        self._invalid_transform_keys: set = set()

    def start(self) -> None:
        if self._started:
            return
        try:
            self._start_ros2_subscriptions()
        except Exception as exc:
            self._last_error = str(exc)
            self._started = False
            self._ros_ready = False
            LOGGER.warning("启动 ROS2 TF 定位订阅失败：%s", exc)
            if self.config.require_ros2:
                raise
            return
        self._started = True
        self._ros_ready = True
        self._last_error = None
        LOGGER.info(
            "ROS2 TF 定位订阅已启动 tf_topic=%s tf_static_topic=%s target=%s source=%s",
            self.tf_topic,
            self.tf_static_topic,
            self.map_frame,
            self.base_frame,
        )

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
                LOGGER.debug("停止 ROS2 TF 定位 executor 失败。", exc_info=True)
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                LOGGER.debug("销毁 ROS2 TF 定位节点失败。", exc_info=True)
        if self._has_ros2_context:
            try:
                release_ros2_context()
            except Exception:
                LOGGER.debug("释放 ROS2 TF 定位上下文失败。", exc_info=True)
        self._node = None
        self._executor = None
        self._thread = None
        self._rclpy = None
        self._subscriptions = []
        self._has_ros2_context = False

    def is_available(self) -> bool:
        return bool(self._started and self._ros_ready and self._last_error is None)

    def is_localization_available(self) -> bool:
        with self._lock:
            return self._latest_pose is not None and self._latest_pose_is_fresh_locked()

    def get_current_pose(self) -> Optional[Pose]:
        with self._lock:
            if self._latest_pose is None or not self._latest_pose_is_fresh_locked():
                return None
            return self._latest_pose.model_copy(deep=True)

    def get_frame_tree(self) -> Optional[FrameTree]:
        with self._lock:
            return self._latest_frame_tree.model_copy(deep=True) if self._latest_frame_tree is not None else None

    def get_localization_state(self) -> Dict[str, object]:
        with self._lock:
            status = dict(self._last_status)
            pose_available = self._latest_pose is not None and self._latest_pose_is_fresh_locked()
            pose_age_sec = self._pose_age_sec_locked()
            status.update(
                {
                    "provider_name": self.provider_name,
                    "provider_version": self.provider_version,
                    "tracking_state": "tracking" if pose_available else status.get("tracking_state", "waiting_for_tf"),
                    "pose_available": pose_available,
                    "map_loaded": bool(status.get("map_to_odom_ready") and status.get("map_to_base_ready")),
                    "quality_score": 1.0 if pose_available else 0.0,
                    "pose_age_sec": pose_age_sec,
                    "max_tf_age_sec": self._max_tf_age_sec,
                    "last_error": None if pose_available else status.get("last_error") or self._last_error,
                }
            )
            return status

    def get_status(self) -> Dict[str, object]:
        with self._lock:
            status = dict(self._last_status)
            latest_pose = self._latest_pose.model_copy(deep=True) if self._latest_pose is not None else None
            latest_frame_tree = self._latest_frame_tree.model_copy(deep=True) if self._latest_frame_tree is not None else None
            pose_available = latest_pose is not None and self._latest_pose_is_fresh_locked()
            pose_age_sec = self._pose_age_sec_locked()
            last_update_at = self._last_update_at
            latest_pose_received_at = self._latest_pose_received_at
            latest_pose_source_timestamp = self._latest_pose_source_timestamp
            last_error = self._last_error
            dynamic_count = len(self._dynamic_transforms)
            static_count = len(self._static_transforms)
            tf_message_count = self._tf_message_count
            tf_static_message_count = self._tf_static_message_count
            update_sequence = self._update_sequence
        tf_publisher_count = self._publisher_count(self.tf_topic)
        tf_static_publisher_count = self._publisher_count(self.tf_static_topic)
        provider_status = "available" if self.is_available() else ("error" if last_error else "starting" if self._started else "stopped")
        return {
            "provider_name": self.provider_name,
            "provider_version": self.provider_version,
            "provider_status": provider_status,
            "started": self._started,
            "ros_ready": self._ros_ready,
            "tf_topic": self.tf_topic,
            "tf_static_topic": self.tf_static_topic,
            "tf_publisher_count": tf_publisher_count,
            "tf_static_publisher_count": tf_static_publisher_count,
            "tf_has_publisher": tf_publisher_count is not None and tf_publisher_count > 0,
            "tf_static_has_publisher": tf_static_publisher_count is not None and tf_static_publisher_count > 0,
            "tf_message_count": tf_message_count,
            "tf_static_message_count": tf_static_message_count,
            "message_count": tf_message_count + tf_static_message_count,
            "update_sequence": update_sequence,
            "dynamic_transform_count": dynamic_count,
            "static_transform_count": static_count,
            "transform_count": dynamic_count + static_count,
            "target_frame_id": self.map_frame,
            "source_frame_id": self.base_frame,
            "map_frame": self.map_frame,
            "odom_frame": self.odom_frame,
            "base_frame": self.base_frame,
            "pose_ready": pose_available,
            "pose_available": pose_available,
            "pose_frame_id": latest_pose.frame_id if latest_pose is not None else None,
            "pose_age_sec": pose_age_sec,
            "max_tf_age_sec": self._max_tf_age_sec,
            "latest_pose_received_at": latest_pose_received_at.isoformat() if latest_pose_received_at is not None else None,
            "latest_pose_source_timestamp": latest_pose_source_timestamp.isoformat() if latest_pose_source_timestamp is not None else None,
            "last_update_at": last_update_at.isoformat() if last_update_at is not None else None,
            "frame_tree_ready": latest_frame_tree is not None,
            "last_error": last_error,
            **status,
        }

    def subscribe_localization(
        self,
        callback: Callable[[Pose, Optional[FrameTree], Dict[str, object]], None],
    ) -> Optional[int]:
        if not callable(callback):
            return None
        immediate_payload = None
        with self._lock:
            subscription_id = self._next_localization_subscription_id
            self._next_localization_subscription_id += 1
            self._localization_subscribers[subscription_id] = callback
            if self._latest_pose is not None and self._latest_pose_is_fresh_locked():
                immediate_payload = (
                    self._latest_pose.model_copy(deep=True),
                    self._latest_frame_tree.model_copy(deep=True) if self._latest_frame_tree is not None else None,
                    self._build_stream_metadata_locked(source_topic="subscribe_replay"),
                )
        if immediate_payload is not None:
            try:
                callback(*immediate_payload)
            except Exception:
                LOGGER.debug("回放 ROS2 TF 定位订阅失败 subscription_id=%s", subscription_id, exc_info=True)
        return subscription_id

    def unsubscribe_localization(self, subscription_id: Optional[int]) -> None:
        if subscription_id is None:
            return
        with self._lock:
            self._localization_subscribers.pop(int(subscription_id), None)

    def ingest_transform_snapshot(
        self,
        transforms: Iterable[Transform],
        *,
        source_topic: str = "external_tf_snapshot",
        static: bool = False,
    ) -> None:
        """写入已适配的 TF 变换，供测试或外部适配器复用。"""

        now = datetime.now(timezone.utc)
        normalized_transforms = []
        for transform in transforms:
            normalized = self._normalize_contract_transform(transform)
            if normalized is not None:
                normalized_transforms.append(normalized)
        if not normalized_transforms:
            return
        self._ingest_contract_transforms(normalized_transforms, source_topic=source_topic, static=static, received_at=now)

    def _start_ros2_subscriptions(self) -> None:
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from tf2_msgs.msg import TFMessage

        node = None
        executor = None
        try:
            rclpy = acquire_ros2_context()
            self._has_ros2_context = True
            node = rclpy.create_node("nuwax_tf_localization_provider")
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            tf_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=100,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            tf_static_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=50,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._subscriptions.append(
                node.create_subscription(TFMessage, self.tf_topic, self._on_tf_message, tf_qos)
            )
            self._subscriptions.append(
                node.create_subscription(TFMessage, self.tf_static_topic, self._on_tf_static_message, tf_static_qos)
            )
            self._rclpy = rclpy
            self._node = node
            self._executor = executor
            self._thread = threading.Thread(target=self._spin, name="nuwax_ros2_tf_localization_provider", daemon=True)
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
                    LOGGER.debug("清理 ROS2 TF 定位节点失败。", exc_info=True)
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
            LOGGER.warning("ROS2 TF 定位订阅循环异常：%s", exc)

    def _on_tf_message(self, message: Any) -> None:
        self._on_tf_message_common(message, source_topic=self.tf_topic, static=False)

    def _on_tf_static_message(self, message: Any) -> None:
        self._on_tf_message_common(message, source_topic=self.tf_static_topic, static=True)

    def _on_tf_message_common(self, message: Any, *, source_topic: str, static: bool) -> None:
        transforms = []
        for item in list(getattr(message, "transforms", []) or []):
            transform = self._contract_transform_from_ros(item, authority=source_topic)
            if transform is not None:
                transforms.append(transform)
        if not transforms:
            return
        self._ingest_contract_transforms(
            transforms,
            source_topic=source_topic,
            static=static,
            received_at=datetime.now(timezone.utc),
        )

    def _ingest_contract_transforms(
        self,
        transforms: List[Transform],
        *,
        source_topic: str,
        static: bool,
        received_at: datetime,
    ) -> None:
        subscribers: Tuple[Tuple[int, Callable[[Pose, Optional[FrameTree], Dict[str, object]], None]], ...] = ()
        payload = None
        with self._lock:
            target = self._static_transforms if static else self._dynamic_transforms
            for transform in transforms:
                key = (transform.parent_frame_id, transform.child_frame_id)
                target[key] = _StoredTransform(
                    transform=transform.model_copy(deep=True),
                    received_at=received_at,
                    source_topic=source_topic,
                    static=static,
                )
            if static:
                self._tf_static_message_count += 1
            else:
                self._tf_message_count += 1
            self._last_update_at = received_at
            self._last_error = None
            if self._recompute_localization_locked(source_topic=source_topic):
                pose = self._latest_pose.model_copy(deep=True) if self._latest_pose is not None else None
                frame_tree = self._latest_frame_tree.model_copy(deep=True) if self._latest_frame_tree is not None else None
                if pose is not None:
                    payload = (pose, frame_tree, self._build_stream_metadata_locked(source_topic=source_topic))
                    subscribers = tuple(self._localization_subscribers.items())
        if payload is not None:
            self._notify_localization_update_subscribers(subscribers, payload)

    def _recompute_localization_locked(self, *, source_topic: str) -> bool:
        frame_tree = self._build_frame_tree_locked()
        self._latest_frame_tree = frame_tree
        map_to_odom_ready = self._transform_ready(frame_tree, source_frame_id=self.odom_frame, target_frame_id=self.map_frame)
        odom_to_base_ready = self._transform_ready(frame_tree, source_frame_id=self.base_frame, target_frame_id=self.odom_frame)
        base_pose = Pose(
            frame_id=self.base_frame,
            position=Vector3(),
            orientation=Quaternion(w=1.0),
        )
        pose = transform_pose_to_frame(frame_tree, base_pose, target_frame_id=self.map_frame)
        previous_sequence = self._update_sequence
        now = datetime.now(timezone.utc)
        if pose is None:
            missing = []
            if not map_to_odom_ready:
                missing.append("%s->%s" % (self.map_frame, self.odom_frame))
            if not odom_to_base_ready:
                missing.append("%s->%s" % (self.odom_frame, self.base_frame))
            detail = "缺少 TF 链：" + ", ".join(missing) if missing else "缺少 %s->%s TF 链。" % (self.map_frame, self.base_frame)
            self._last_status = {
                "tracking_state": "waiting_for_tf",
                "map_to_odom_ready": map_to_odom_ready,
                "odom_to_base_ready": odom_to_base_ready,
                "map_to_base_ready": False,
                "waiting_for": "tf",
                "last_error": detail,
            }
            return False
        self._update_sequence += 1
        pose_timestamp = self._latest_transform_timestamp_locked(frame_tree)
        pose = Pose(
            timestamp=pose_timestamp or now,
            frame_id=pose.frame_id,
            position=pose.position,
            orientation=pose.orientation,
        )
        self._latest_pose = pose
        self._latest_pose_received_at = now
        self._latest_pose_source_timestamp = pose_timestamp
        self._last_status = {
            "tracking_state": "tracking",
            "map_to_odom_ready": map_to_odom_ready,
            "odom_to_base_ready": odom_to_base_ready,
            "map_to_base_ready": True,
            "waiting_for": None,
            "last_error": None,
            "source_topic": source_topic,
        }
        return self._update_sequence != previous_sequence

    def _build_frame_tree_locked(self) -> FrameTree:
        transforms = [
            stored.transform.model_copy(deep=True)
            for stored in (
                list(self._static_transforms.values()) + list(self._dynamic_transforms.values())
            )
        ]
        return FrameTree(root_frame_id=self.map_frame, transforms=transforms)

    def _build_stream_metadata_locked(self, *, source_topic: str) -> Dict[str, object]:
        status = dict(self._last_status)
        return {
            "source_kind": "platform_ros2_tf_localization",
            "provider_name": self.provider_name,
            "provider_version": self.provider_version,
            "source_topic": source_topic,
            "target_frame_id": self.map_frame,
            "source_frame_id": self.base_frame,
            "map_frame": self.map_frame,
            "odom_frame": self.odom_frame,
            "base_frame": self.base_frame,
            "update_sequence": self._update_sequence,
            "tf_message_count": self._tf_message_count,
            "tf_static_message_count": self._tf_static_message_count,
            "transform_count": len(self._dynamic_transforms) + len(self._static_transforms),
            "pose_age_sec": self._pose_age_sec_locked(),
            "max_tf_age_sec": self._max_tf_age_sec,
            "latest_pose_received_at": self._latest_pose_received_at.isoformat() if self._latest_pose_received_at is not None else None,
            "latest_pose_source_timestamp": self._latest_pose_source_timestamp.isoformat() if self._latest_pose_source_timestamp is not None else None,
            "tf_status": status,
        }

    def _latest_pose_is_fresh_locked(self) -> bool:
        if self._latest_pose_received_at is None:
            return False
        return self._pose_age_sec_locked() is not None and self._pose_age_sec_locked() <= self._max_tf_age_sec * 4.0

    def _pose_age_sec_locked(self) -> Optional[float]:
        if self._latest_pose_received_at is None:
            return None
        return max(0.0, (datetime.now(timezone.utc) - self._latest_pose_received_at).total_seconds())

    def _latest_transform_timestamp_locked(self, frame_tree: FrameTree) -> Optional[datetime]:
        latest: Optional[datetime] = None
        for transform in frame_tree.transforms:
            timestamp = transform.timestamp
            if latest is None or timestamp > latest:
                latest = timestamp
        return latest

    def _transform_ready(self, frame_tree: FrameTree, *, source_frame_id: str, target_frame_id: str) -> bool:
        return resolve_frame_transform(
            frame_tree,
            source_frame_id=source_frame_id,
            target_frame_id=target_frame_id,
        ) is not None

    def _notify_localization_update_subscribers(
        self,
        subscribers: Tuple[Tuple[int, Callable[[Pose, Optional[FrameTree], Dict[str, object]], None]], ...],
        payload: Tuple[Pose, Optional[FrameTree], Dict[str, object]],
    ) -> None:
        pose, frame_tree, metadata = payload
        for subscription_id, callback in subscribers:
            try:
                callback(pose.model_copy(deep=True), frame_tree.model_copy(deep=True) if frame_tree is not None else None, dict(metadata))
            except Exception:
                LOGGER.debug("ROS2 TF 定位订阅回调失败 subscription_id=%s", subscription_id, exc_info=True)

    def _publisher_count(self, topic: str) -> Optional[int]:
        if not topic or self._node is None:
            return None
        try:
            return len(self._node.get_publishers_info_by_topic(topic))
        except Exception:
            LOGGER.debug("读取 ROS2 TF publisher 失败 topic=%s", topic, exc_info=True)
            return None

    def _contract_transform_from_ros(self, message: Any, *, authority: str) -> Optional[Transform]:
        header = getattr(message, "header", None)
        parent_frame_id = self._normalize_frame_id(getattr(header, "frame_id", None))
        child_frame_id = self._normalize_frame_id(getattr(message, "child_frame_id", None))
        if not parent_frame_id or not child_frame_id or parent_frame_id == child_frame_id:
            return None
        transform = getattr(message, "transform", None)
        translation = getattr(transform, "translation", None)
        rotation = getattr(transform, "rotation", None)
        try:
            return Transform(
                timestamp=self._timestamp_from_header(header),
                parent_frame_id=parent_frame_id,
                child_frame_id=child_frame_id,
                translation=Vector3(
                    x=float(getattr(translation, "x", 0.0) or 0.0),
                    y=float(getattr(translation, "y", 0.0) or 0.0),
                    z=float(getattr(translation, "z", 0.0) or 0.0),
                ),
                rotation=Quaternion(
                    x=float(getattr(rotation, "x", 0.0) or 0.0),
                    y=float(getattr(rotation, "y", 0.0) or 0.0),
                    z=float(getattr(rotation, "z", 0.0) or 0.0),
                    w=float(getattr(rotation, "w", 1.0) if rotation is not None else 1.0),
                ),
                authority=authority,
            )
        except ValidationError as exc:
            key = (parent_frame_id, child_frame_id)
            if key not in self._invalid_transform_keys:
                self._invalid_transform_keys.add(key)
                LOGGER.warning(
                    "跳过不合规 TF 变换 parent=%s child=%s authority=%s reason=%s",
                    parent_frame_id,
                    child_frame_id,
                    authority,
                    exc.errors()[0].get("msg") if exc.errors() else str(exc),
                )
            return None

    def _normalize_contract_transform(self, transform: Transform) -> Optional[Transform]:
        parent_frame_id = self._normalize_frame_id(transform.parent_frame_id)
        child_frame_id = self._normalize_frame_id(transform.child_frame_id)
        if not parent_frame_id or not child_frame_id or parent_frame_id == child_frame_id:
            return None
        return transform.model_copy(
            update={
                "parent_frame_id": parent_frame_id,
                "child_frame_id": child_frame_id,
            },
            deep=True,
        )

    def _timestamp_from_header(self, header: Any) -> datetime:
        stamp = getattr(header, "stamp", None)
        if stamp is None:
            return datetime.now(timezone.utc)
        seconds = int(getattr(stamp, "sec", 0) or 0)
        nanoseconds = int(getattr(stamp, "nanosec", 0) or 0)
        if seconds <= 0 and nanoseconds <= 0:
            return datetime.now(timezone.utc)
        return datetime.fromtimestamp(seconds + nanoseconds / 1_000_000_000.0, tz=timezone.utc)

    @staticmethod
    def _normalize_frame_id(value: object) -> str:
        return str(value or "").strip().lstrip("/")
