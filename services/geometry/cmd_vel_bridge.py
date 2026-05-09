from __future__ import annotations

from datetime import datetime, timezone
import logging
import math
import threading
from typing import Any, Dict, List, Optional

from contracts.geometry import Twist, Vector3
from providers.motion import MotionControl
from services.geometry.config import PlatformGeometryConfig
from services.geometry.ros2_context import acquire_ros2_context, release_ros2_context

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.cmd_vel_bridge")


class Ros2CmdVelMotionBridge:
    """把 Nav2 速度输出桥接到机器人运动控制提供器。"""

    provider_name = "platform_ros2_cmd_vel_motion_bridge"
    provider_version = "0.1.0"

    def __init__(self, config: PlatformGeometryConfig, motion_provider_resolver) -> None:
        self.config = config
        self.topic = str(getattr(config.nav2, "cmd_vel_topic", "/cmd_vel") or "/cmd_vel")
        self.timeout_sec = max(0.1, float(getattr(config.nav2, "cmd_vel_timeout_sec", 0.6) or 0.6))
        self.frame_id = str(config.frames.base_frame or "body")
        self._motion_provider_resolver = motion_provider_resolver
        self._lock = threading.RLock()
        self._started = False
        self._ros_ready = False
        self._has_ros2_context = False
        self._node = None
        self._executor = None
        self._subscriptions: List[Any] = []
        self._subscribed_topics: tuple[str, ...] = ()
        self._thread: Optional[threading.Thread] = None
        self._watchdog_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._last_error: Optional[str] = None
        self._received_count = 0
        self._forwarded_count = 0
        self._dropped_count = 0
        self._stop_count = 0
        self._last_received_at: Optional[datetime] = None
        self._last_forwarded_at: Optional[datetime] = None
        self._last_command: Optional[Dict[str, float]] = None
        self._last_command_monotonic = 0.0
        self._motion_active = False

    def start(self) -> None:
        if self._started:
            return
        self._stop_event.clear()
        node = None
        executor = None
        try:
            from geometry_msgs.msg import Twist as RosTwist
            from geometry_msgs.msg import TwistStamped as RosTwistStamped
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

            rclpy = acquire_ros2_context()
            self._has_ros2_context = True
            node = rclpy.create_node("nuwax_cmd_vel_motion_bridge")
            qos_profile = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            subscriptions = []
            subscribed_topics = []

            def subscribe(message_type, topic: str, label: str) -> None:
                try:
                    subscriptions.append(node.create_subscription(message_type, topic, self._on_twist, qos_profile))
                    subscribed_topics.append(f"{topic}:{label}")
                except Exception:
                    LOGGER.debug("订阅 ROS2 速度话题失败 topic=%s type=%s", topic, label, exc_info=True)

            subscribe(RosTwist, self.topic, "Twist")
            subscribe(RosTwistStamped, self.topic, "TwistStamped")
            stamped_topic = self.topic if self.topic.endswith("_stamped") else f"{self.topic}_stamped"
            if stamped_topic != self.topic:
                subscribe(RosTwistStamped, stamped_topic, "TwistStamped")
            if not subscriptions:
                raise RuntimeError(f"无法订阅 ROS2 速度话题：{self.topic}")
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            self._node = node
            self._executor = executor
            self._subscriptions = subscriptions
            self._subscribed_topics = tuple(subscribed_topics)
            self._thread = threading.Thread(target=executor.spin, name="nuwax_cmd_vel_motion_bridge", daemon=True)
            self._thread.start()
            self._watchdog_thread = threading.Thread(target=self._watchdog_loop, name="nuwax_cmd_vel_watchdog", daemon=True)
            self._watchdog_thread.start()
            self._started = True
            self._ros_ready = True
            self._last_error = None
            LOGGER.info("ROS2 cmd_vel 运动桥已启动 topic=%s", self.topic)
        except Exception as exc:
            self._last_error = str(exc)
            self._started = False
            self._ros_ready = False
            if executor is not None:
                try:
                    executor.shutdown(timeout_sec=0.0)
                except TypeError:
                    executor.shutdown()
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    LOGGER.debug("清理 ROS2 cmd_vel 节点失败。", exc_info=True)
            if self._has_ros2_context:
                release_ros2_context()
            self._node = None
            self._executor = None
            self._subscriptions = []
            self._subscribed_topics = ()
            self._has_ros2_context = False
            LOGGER.warning("启动 ROS2 cmd_vel 运动桥失败：%s", exc)
            if self.config.require_ros2:
                raise

    def stop(self) -> None:
        if not self._started and self._node is None:
            return
        self._started = False
        self._ros_ready = False
        self._stop_event.set()
        self._stop_motion(reason="bridge_stop")
        executor = self._executor
        node = self._node
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=0.0)
            except TypeError:
                executor.shutdown()
            except Exception:
                LOGGER.debug("停止 ROS2 cmd_vel executor 失败。", exc_info=True)
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._watchdog_thread is not None and self._watchdog_thread.is_alive():
            self._watchdog_thread.join(timeout=1.0)
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                LOGGER.debug("销毁 ROS2 cmd_vel 节点失败。", exc_info=True)
        if self._has_ros2_context:
            try:
                release_ros2_context()
            except Exception:
                LOGGER.debug("释放 ROS2 cmd_vel 上下文失败。", exc_info=True)
        self._node = None
        self._executor = None
        self._subscriptions = []
        self._subscribed_topics = ()
        self._thread = None
        self._watchdog_thread = None
        self._has_ros2_context = False

    def is_available(self) -> bool:
        return bool(self._started and self._ros_ready and self._last_error is None)

    def get_status(self) -> Dict[str, object]:
        with self._lock:
            age = None
            if self._last_command_monotonic > 0.0:
                import time

                age = max(0.0, time.monotonic() - self._last_command_monotonic)
            return {
                "provider_name": self.provider_name,
                "provider_version": self.provider_version,
                "started": self._started,
                "ros_ready": self._ros_ready,
                "topic": self.topic,
                "cmd_vel_topic": self.topic,
                "subscribed_topics": list(self._subscribed_topics),
                "timeout_sec": self.timeout_sec,
                "received_count": self._received_count,
                "forwarded_count": self._forwarded_count,
                "dropped_count": self._dropped_count,
                "stop_count": self._stop_count,
                "motion_active": self._motion_active,
                "last_received_at": self._last_received_at.isoformat() if self._last_received_at is not None else None,
                "last_forwarded_at": self._last_forwarded_at.isoformat() if self._last_forwarded_at is not None else None,
                "last_command_age_sec": age,
                "last_command": dict(self._last_command) if self._last_command is not None else None,
                "last_error": self._last_error,
            }

    def _on_twist(self, msg: Any) -> None:
        import time

        twist_msg = getattr(msg, "twist", msg)
        linear = getattr(twist_msg, "linear", None)
        angular = getattr(twist_msg, "angular", None)
        vx = float(getattr(linear, "x", 0.0) or 0.0)
        vy = float(getattr(linear, "y", 0.0) or 0.0)
        vyaw = float(getattr(angular, "z", 0.0) or 0.0)
        now = datetime.now(timezone.utc)
        with self._lock:
            self._received_count += 1
            self._last_received_at = now
            self._last_command = {"vx": vx, "vy": vy, "vyaw": vyaw}
            self._last_command_monotonic = time.monotonic()
        if not self._finite(vx, vy, vyaw):
            with self._lock:
                self._dropped_count += 1
                self._last_error = "cmd_vel 包含非有限数值。"
            return
        provider = self._motion_provider()
        if provider is None:
            with self._lock:
                self._dropped_count += 1
                self._last_error = "机器人运动控制提供器不可用。"
            return
        twist = Twist(frame_id=self.frame_id, linear=Vector3(x=vx, y=vy, z=0.0), angular=Vector3(x=0.0, y=0.0, z=vyaw))
        try:
            provider.send_twist(twist)
        except Exception as exc:
            with self._lock:
                self._dropped_count += 1
                self._last_error = str(exc)
            LOGGER.warning("转发 Nav2 cmd_vel 到机器人运动控制失败：%s", exc)
            return
        with self._lock:
            self._forwarded_count += 1
            self._last_forwarded_at = now
            self._motion_active = bool(abs(vx) > 1e-4 or abs(vy) > 1e-4 or abs(vyaw) > 1e-4)
            self._last_error = None

    def _watchdog_loop(self) -> None:
        import time

        while not self._stop_event.wait(min(0.1, self.timeout_sec / 2.0)):
            with self._lock:
                active = self._motion_active
                last = self._last_command_monotonic
            if active and last > 0.0 and time.monotonic() - last > self.timeout_sec:
                self._stop_motion(reason="cmd_vel_timeout")

    def _stop_motion(self, *, reason: str) -> None:
        provider = self._motion_provider()
        if provider is None:
            return
        try:
            provider.stop_motion()
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
            LOGGER.debug("停止机器人运动失败 reason=%s", reason, exc_info=True)
            return
        with self._lock:
            self._motion_active = False
            self._stop_count += 1

    def _motion_provider(self) -> Optional[MotionControl]:
        provider = self._motion_provider_resolver()
        if provider is None or not isinstance(provider, MotionControl):
            return None
        return provider

    @staticmethod
    def _finite(*values: float) -> bool:
        return all(math.isfinite(float(value)) for value in values)
