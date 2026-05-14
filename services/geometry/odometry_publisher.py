from __future__ import annotations

from datetime import datetime, timezone
import logging
import math
import threading
from typing import Any, Callable, Dict, Optional, Tuple

from contracts.events import RuntimeEventCategory, RuntimeEventSeverity
from contracts.geometry import Pose
from core.event_bus import EventBus
from services.geometry.config import PlatformGeometryConfig
from services.geometry.ros2_context import acquire_ros2_context, release_ros2_context

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.odometry_publisher")


class Ros2OdometryPublisher:
    """把平台定位数据面位姿发布为 ROS2 Odometry 输入。"""

    provider_name = "platform_ros2_odometry_publisher"
    provider_version = "0.1.0"

    def __init__(
        self,
        config: PlatformGeometryConfig,
        provider_resolver: Callable[[], Any],
        *,
        event_bus: Optional[EventBus] = None,
    ) -> None:
        self.config = config
        topic = str(config.topics.odom_topic or "").strip()
        if not topic:
            raise ValueError(
                "PlatformGeometryConfig.topics.odom_topic 必须显式配置，禁止使用兜底字面值。"
            )
        odom_frame = str(config.frames.odom_frame or "").strip()
        base_frame = str(config.frames.base_frame or "").strip()
        if not odom_frame or not base_frame:
            raise ValueError(
                "PlatformGeometryConfig.frames.odom_frame / base_frame 必须显式配置，禁止使用兜底字面值。"
            )
        self.topic = topic
        self.odom_frame = odom_frame
        self.base_frame = base_frame
        self._provider_resolver = provider_resolver
        self._event_bus = event_bus
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._started = False
        self._ros_ready = False
        self._has_ros2_context = False
        self._published_count = 0
        self._dropped_count = 0
        self._last_error: Optional[str] = None
        self._last_pose_timestamp: Optional[datetime] = None
        self._last_published_at: Optional[datetime] = None
        self._node = None
        self._publisher = None
        self._rclpy = None
        self._odometry_type = None
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
            LOGGER.warning("启动 ROS2 odom 发布桥失败：%s", exc)
            self._emit_failure_event("publisher_start_failed", str(exc))
            if self.config.require_ros2:
                raise
            return
        self._started = True
        self._ros_ready = True
        self._last_error = None
        self._subscribe_localization_source()
        LOGGER.info("ROS2 odom 发布桥已启动 odom_topic=%s odom_frame=%s base_frame=%s", self.topic, self.odom_frame, self.base_frame)

    def stop(self) -> None:
        if not self._started and self._node is None:
            return
        node = self._node
        self._started = False
        self._ros_ready = False
        self._stop_event.set()
        subscription_id = self._subscription_id
        provider = self._provider_resolver()
        self._subscription_id = None
        if provider is not None and subscription_id is not None:
            unsubscriber = getattr(provider, "unsubscribe_localization", None)
            if callable(unsubscriber):
                try:
                    unsubscriber(subscription_id)
                except Exception:
                    LOGGER.debug("取消 ROS2 odom 定位订阅失败。", exc_info=True)
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                LOGGER.debug("销毁 ROS2 odom 发布节点失败。", exc_info=True)
        if self._has_ros2_context:
            try:
                release_ros2_context()
            except Exception:
                LOGGER.debug("释放 ROS2 odom 发布上下文失败。", exc_info=True)
        self._node = None
        self._publisher = None
        self._rclpy = None
        self._odometry_type = None
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
                "odom_topic": self.topic,
                "topic": self.topic,
                "odom_frame": self.odom_frame,
                "base_frame": self.base_frame,
                "published_count": self._published_count,
                "dropped_count": self._dropped_count,
                "last_pose_timestamp": self._last_pose_timestamp.isoformat() if self._last_pose_timestamp is not None else None,
                "last_published_at": self._last_published_at.isoformat() if self._last_published_at is not None else None,
                "last_error": self._last_error,
            }

    def _start_ros2_publisher(self) -> None:
        from nav_msgs.msg import Odometry
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        node = None
        try:
            rclpy = acquire_ros2_context()
            self._has_ros2_context = True
            node = rclpy.create_node("nuwax_odometry_publisher")
            qos_profile = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=20,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._publisher = node.create_publisher(Odometry, self.topic, qos_profile)
            self._node = node
            self._rclpy = rclpy
            self._odometry_type = Odometry
        except Exception:
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    LOGGER.debug("清理 ROS2 odom 发布节点失败。", exc_info=True)
            if self._has_ros2_context:
                release_ros2_context()
            self._node = None
            self._publisher = None
            self._rclpy = None
            self._odometry_type = None
            self._has_ros2_context = False
            raise

    def _subscribe_localization_source(self) -> None:
        provider = self._provider_resolver()
        if provider is None:
            self._last_error = "定位 provider 不可用，未启用 ROS2 odom 发布。"
            self._emit_failure_event("provider_unavailable", self._last_error)
            return
        subscriber = getattr(provider, "subscribe_localization", None)
        if not callable(subscriber):
            self._last_error = "定位 provider 未实现 subscribe_localization，未启用定时轮询伪流式。"
            self._emit_failure_event("provider_subscribe_unavailable", self._last_error)
            return
        self._subscription_id = subscriber(lambda pose, frame_tree, metadata: self._publish_pose(pose))
        if self._subscription_id is None:
            self._last_error = "定位 provider 未返回有效订阅，ROS2 odom 发布未连接源头流。"
            self._emit_failure_event("provider_subscribe_returned_none", self._last_error)

    def _publish_pose(self, pose: Pose) -> None:
        if self._publisher is None or self._odometry_type is None or pose is None:
            return
        try:
            message = self._build_odometry_message(pose)
            if message is None:
                with self._lock:
                    self._dropped_count += 1
                return
            self._publisher.publish(message)
            now = datetime.now(timezone.utc)
            with self._lock:
                self._published_count += 1
                self._last_pose_timestamp = pose.timestamp
                self._last_published_at = now
                self._last_error = None
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
            LOGGER.debug("发布 ROS2 odom 失败。", exc_info=True)
            self._emit_failure_event("publish_exception", str(exc))

    def _emit_failure_event(self, reason: str, message: str) -> None:
        if self._event_bus is None:
            return
        try:
            self._event_bus.publish(
                self._event_bus.build_event(
                    "geometry.odom_publisher_failed",
                    category=RuntimeEventCategory.ROBOT,
                    source="platform_ros2_odometry_publisher",
                    severity=RuntimeEventSeverity.ERROR,
                    message=message,
                    payload={
                        "reason": reason,
                        "topic": self.topic,
                        "odom_frame": self.odom_frame,
                        "base_frame": self.base_frame,
                    },
                )
            )
        except Exception:
            LOGGER.debug("发布 odom_publisher_failed 事件失败。", exc_info=True)

    def _build_odometry_message(self, pose: Pose):
        Odometry = self._odometry_type
        if Odometry is None:
            return None
        if not self._pose_is_finite(pose):
            with self._lock:
                self._last_error = "定位位姿包含非有限数值，已丢弃。"
            return None
        message = Odometry()
        stamp_sec, stamp_nanosec = self._timestamp_to_ros_stamp(pose.timestamp)
        message.header.stamp.sec = stamp_sec
        message.header.stamp.nanosec = stamp_nanosec
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = float(pose.position.x)
        message.pose.pose.position.y = float(pose.position.y)
        message.pose.pose.position.z = float(pose.position.z)
        message.pose.pose.orientation.x = float(pose.orientation.x)
        message.pose.pose.orientation.y = float(pose.orientation.y)
        message.pose.pose.orientation.z = float(pose.orientation.z)
        message.pose.pose.orientation.w = float(pose.orientation.w)
        return message

    @staticmethod
    def _pose_is_finite(pose: Pose) -> bool:
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return all(math.isfinite(float(value)) for value in values)

    @staticmethod
    def _timestamp_to_ros_stamp(timestamp: datetime) -> Tuple[int, int]:
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
