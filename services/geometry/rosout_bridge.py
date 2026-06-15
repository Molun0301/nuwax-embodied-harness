from __future__ import annotations

import logging
import threading
from typing import Optional

from core.logging_utils import LogRateLimiter

LOGGER = logging.getLogger("nuwax_robot_bridge.ros2")

_NODE_PREFIXES = (
    "nuwax_",
    "nav2_",
    "rtabmap",
    "amcl",
    "bt_navigator",
    "controller_server",
    "planner_server",
    "odom_tf_bridge",
)

_HIGH_NOISE_NODES = frozenset({
    "rtabmap",
    "bt_navigator",
    "controller_server",
    "planner_server",
})

_RTABMAP_SPAM_PREFIXES = (
    "Odom: ",
    "Update map",
    "Localization: ",
    "Planning: ",
)

_ROS2_LEVEL_MAP = {
    10: logging.DEBUG,
    20: logging.INFO,
    30: logging.WARNING,
    40: logging.ERROR,
    50: logging.CRITICAL,
}

_MAX_TRACKED_KEYS = 2048


try:
    from rcl_interfaces.msg import Log as RosLogMsg
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False


class RosoutBridge:
    """订阅 /rosout 并将白名单节点日志桥接到 Python logging。"""

    def __init__(self) -> None:
        self._node = None
        self._spin_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._rate_limiter = LogRateLimiter()
        self._started = False

    def start(self) -> bool:
        if self._started:
            return True
        if not _ROS2_AVAILABLE:
            LOGGER.warning("rcl_interfaces 不可用，rosout 桥接跳过")
            return False

        from services.geometry.ros2_context import acquire_ros2_context
        rclpy = acquire_ros2_context()

        self._node = rclpy.create_node(
            "nuwax_rosout_bridge",
            enable_rosout=False,
        )
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=64,
        )
        self._node.create_subscription(
            RosLogMsg, "/rosout", self._on_rosout, qos
        )
        self._stop_event.clear()
        self._spin_thread = threading.Thread(
            target=self._spin_loop,
            name="rosout_bridge_spin",
            daemon=True,
        )
        self._spin_thread.start()
        self._started = True
        LOGGER.info("rosout 桥接已启动")
        return True

    def stop(self) -> None:
        if not self._started:
            return
        self._started = False
        self._stop_event.set()
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=3.0)
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        from services.geometry.ros2_context import release_ros2_context
        release_ros2_context()
        self._rate_limiter.reset()
        LOGGER.info("rosout 桥接已停止")

    def _spin_loop(self) -> None:
        import rclpy
        while not self._stop_event.is_set() and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.5)

    def _on_rosout(self, msg) -> None:
        node_name = msg.name or ""
        if not any(node_name.startswith(p) for p in _NODE_PREFIXES):
            return

        text = msg.msg or ""
        if node_name.startswith("rtabmap") and any(
            text.startswith(p) for p in _RTABMAP_SPAM_PREFIXES
        ):
            return

        py_level = _ROS2_LEVEL_MAP.get(msg.level, logging.DEBUG)

        is_high_noise = any(node_name.startswith(n) for n in _HIGH_NOISE_NODES)
        if is_high_noise and py_level <= logging.INFO:
            py_level = logging.DEBUG

        if not LOGGER.isEnabledFor(py_level):
            return

        self._trim_rate_limiter()

        if is_high_noise:
            self._rate_limiter.log(
                LOGGER, py_level, "[%s] %s", node_name, text,
                key=("rosout", node_name, text[:64]),
                interval_sec=10.0,
                extra={"source": "rosout", "ros_node": node_name},
            )
        else:
            LOGGER.log(
                py_level, "[%s] %s", node_name, text,
                extra={"source": "rosout", "ros_node": node_name},
            )

    def _trim_rate_limiter(self) -> None:
        with self._rate_limiter._lock:
            if len(self._rate_limiter._states) > _MAX_TRACKED_KEYS:
                self._rate_limiter._states.clear()
