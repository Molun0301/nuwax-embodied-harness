from __future__ import annotations

import threading

_LOCK = threading.RLock()
_REF_COUNT = 0
_RCLPY = None
_INITIALIZED_BY_US = False


def acquire_ros2_context():
    global _INITIALIZED_BY_US, _RCLPY, _REF_COUNT

    import rclpy

    with _LOCK:
        if not rclpy.ok():
            rclpy.init(args=None)
            _INITIALIZED_BY_US = True
        _RCLPY = rclpy
        _REF_COUNT += 1
        return rclpy


def release_ros2_context() -> None:
    global _INITIALIZED_BY_US, _RCLPY, _REF_COUNT

    with _LOCK:
        if _REF_COUNT > 0:
            _REF_COUNT -= 1
        if _REF_COUNT != 0 or not _INITIALIZED_BY_US or _RCLPY is None:
            return
        rclpy = _RCLPY
        _RCLPY = None
        _INITIALIZED_BY_US = False
        if rclpy.ok():
            rclpy.shutdown()
