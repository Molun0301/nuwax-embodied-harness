from __future__ import annotations

import os
import sys
import threading
from pathlib import Path

_LOCK = threading.RLock()
_REF_COUNT = 0
_RCLPY = None
_INITIALIZED_BY_US = False
_GO2_SLAM_PYTHON_PATHS_PREPARED = False


def _go2_slam_install_dirs() -> tuple[Path, ...]:
    candidates: list[Path] = []
    env_setup = str(os.environ.get("NUWAX_GEOMETRY_GO2SLAM_SETUP_SCRIPT", "") or "").strip()
    if env_setup:
        install_dir = Path(env_setup).expanduser().resolve().parent
        if install_dir.name == "install":
            candidates.append(install_dir)

    repo_root = Path(__file__).resolve().parents[2]
    default_install = repo_root / "services" / "geometry" / "ros2_ws" / "src" / "GO2_RTABNAV2" / "ws" / "install"
    if default_install.is_dir():
        candidates.append(default_install)

    unique: list[Path] = []
    for candidate in candidates:
        if candidate not in unique:
            unique.append(candidate)
    return tuple(unique)


def ensure_go2_slam_overlay_python_paths() -> bool:
    """确保 GO2 SLAM 工作区自定义消息包可被 Python 导入。"""

    global _GO2_SLAM_PYTHON_PATHS_PREPARED

    try:
        import explore_lite_msgs  # noqa: F401

        return True
    except ImportError:
        pass

    with _LOCK:
        if _GO2_SLAM_PYTHON_PATHS_PREPARED:
            try:
                import explore_lite_msgs  # noqa: F401

                return True
            except ImportError:
                return False

        added = False
        for install_dir in _go2_slam_install_dirs():
            for site_packages in sorted(install_dir.glob("*/lib/python*/site-packages")):
                site_path = str(site_packages)
                if site_path not in sys.path:
                    sys.path.insert(0, site_path)
                    added = True

        _GO2_SLAM_PYTHON_PATHS_PREPARED = True

    if not added:
        return False

    try:
        import explore_lite_msgs  # noqa: F401

        return True
    except ImportError:
        return False


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
