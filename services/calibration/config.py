"""相机-激光雷达标定服务配置（阈值类纯算法参数）。

================================ 本包冻结说明 ================================
services/calibration 整包当前【未接线、本期冻结】：CalibrationService 全仓零实例
化，feed_frame_pair/update_motion_state/quality_monitor.add_sample 均无调用方
（已 grep 核实）。

实际生效的标定链路【不在本包】，而是 CalibrationProvider 主线：
  drivers/go2/defaults.py（GO2_DEFAULT_CAMERA_INFO 内参 + GO2_DEFAULT_SENSOR_TOPOLOGY 外参）
  → drivers/go2/providers.py（get_camera_info / get_sensor_topology）
  → 内参经 gateways/runtime.py 注入 SHM 帧 metadata
  → 外参经 services/geometry/runtime.py 的 _inject_static_extrinsics 注入 frame_tree 静态边。

治理边界（01-P1-3 / 08-P1-3）：services 禁止从 env 直读相机物理参数，内参归 driver、
外参走 CalibrationProvider。因此本文件原 `load_sensor_topology_from_env`（从 env 直读
NUWAX_CAMERA_FX/FY/CX/CY、畸变、NUWAX_CALIBRATION_INITIAL_TX..RZ 外参初值等 27 键）
已删除。未来若复活 MI 在线标定，SensorTopology 应由 CalibrationProvider 注入
（见 CalibrationService.__init__ 的 topology 构造参数），不得再从 env 读物理参数。

MI 在线标定【严禁直接接线】：NMI 安全门控阈值低于理论下界恒通过（05-P2-8）、外参方向/
坐标系语义错位（05-P2-9），接线即错。本期仅保留为纯算法库 + 阈值配置。

本文件仅保留【阈值类纯算法参数】的 env 读取（开关 / 最小帧数 / 静止检测阈值 /
重投影阈值 / NMI 阈值 / 持久化目录），不属物理参数红线。
============================================================================
"""

from __future__ import annotations

import logging
import os


LOGGER = logging.getLogger(__name__)


def _env_str(key: str, default: str = "") -> str:
    return os.environ.get(key, default).strip()


def _env_float(key: str, default: float = 0.0) -> float:
    raw = os.environ.get(key, "").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except (ValueError, TypeError):
        LOGGER.warning("环境变量 %s 值无效: %r，使用默认值 %s", key, raw, default)
        return default


def _env_int(key: str, default: int = 0) -> int:
    raw = os.environ.get(key, "").strip()
    if not raw:
        return default
    try:
        return int(raw)
    except (ValueError, TypeError):
        LOGGER.warning("环境变量 %s 值无效: %r，使用默认值 %s", key, raw, default)
        return default


def _env_bool(key: str, default: bool = False) -> bool:
    raw = os.environ.get(key, "").strip().lower()
    if not raw:
        return default
    return raw in ("true", "1", "yes", "on")


def is_calibration_enabled() -> bool:
    """检查标定功能是否启用。"""
    return _env_bool("NUWAX_CALIBRATION_AUTO_ENABLED", False)


def get_calibration_min_frames() -> int:
    """获取触发标定所需的最小帧对数。"""
    return _env_int("NUWAX_CALIBRATION_MIN_FRAMES", 20)


def get_calibration_static_velocity_threshold() -> float:
    """获取静止检测线速度阈值（m/s）。"""
    return _env_float("NUWAX_CALIBRATION_STATIC_VELOCITY_THRESHOLD", 0.02)


def get_calibration_static_angular_threshold() -> float:
    """获取静止检测角速度阈值（rad/s）。"""
    return _env_float("NUWAX_CALIBRATION_STATIC_ANGULAR_THRESHOLD", 0.05)


def get_calibration_reprojection_threshold() -> float:
    """获取重投影误差合格阈值（像素）。"""
    return _env_float("NUWAX_CALIBRATION_REPROJECTION_THRESHOLD_PX", 5.0)


def get_calibration_nmi_threshold() -> float:
    """获取 NMI（归一化互信息）最小接受阈值。

    注意（05-P2-8）：compute_nmi 返回值 ∈ [1, 2]，默认阈值 0.3 低于理论下界 1.0，
    门控恒通过——属伪保护。MI 标定接线前必须把默认提到 >= 1.0（如 1.05）。本期不接线，
    保持原默认不动，仅在此标注缺陷。
    """
    return _env_float("NUWAX_CALIBRATION_NMI_MIN_THRESHOLD", 0.3)


def get_calibration_persist_dir() -> str:
    """获取标定结果持久化目录。"""
    return _env_str("NUWAX_CALIBRATION_PERSIST_DIR", "runtime_data/calibration")
