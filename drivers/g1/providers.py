"""G1 端侧 Provider 集合。

平台 provider 抽象（MotionControl / CalibrationProvider / PointCloudProvider 等）
与 G1 装配绑定。所有 SDK 原始对象不外泄，平台侧只通过 provider 拿数据。

设计要点：
- 端侧独占：本模块仅在 drivers/g1/ 子树内使用，platform services 禁 import。
- 物理参数归端侧（frame_id/topic/超时），与 cross-robot-generalization-foundations 一致。
- 6 个 provider 一一对应：MotionProvider（loco）+ ArmProvider（arm）+ PointCloudProvider（videohub 复路径）
  + IMUStateProvider（订阅 sportmodestate）+ AudioTTSProvider（AudioClient）+ RobotStateProvider（state 包装）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, TYPE_CHECKING

from contracts.calibration import SensorTopology
from contracts.geometry import FrameTree, Pose, Quaternion, Twist, Vector3
from contracts.image import CameraInfo, ImageEncoding, ImageFrame
from contracts.pointcloud import PointCloudFrame
from contracts.robot_state import IMUState, JointState, RobotControlMode, RobotState, SafetyState
from providers.calibration import CalibrationProvider
from providers.image import ImageProvider
from providers.motion import MotionControl
from providers.pointcloud import PointCloudProvider
from providers.safety import SafetyProvider
from providers.state import StateProvider

if TYPE_CHECKING:
    from drivers.g1.assembly import G1RobotAssembly
    from drivers.g1.control.loco_controller import LocoController
    from drivers.g1.control.arm_controller import ArmController


def _to_control_mode(mode: str) -> RobotControlMode:
    if mode == "loco":
        return RobotControlMode.HIGH_LEVEL
    if mode == "low":
        return RobotControlMode.LOW_LEVEL
    return RobotControlMode.UNKNOWN


@dataclass
class G1ProviderBundle(
    StateProvider,
    ImageProvider,
    PointCloudProvider,
    MotionControl,
    SafetyProvider,
    CalibrationProvider,
):
    """G1 默认提供器集合。"""

    assembly: "G1RobotAssembly"
    provider_name: str = "g1_default_bundle"
    provider_version: str = "0.1.0"
    safety_guard: Optional[object] = None

    # --- 内部缓存（避免每次调用都问 assembly）
    _loco_controller: Optional["LocoController"] = field(default=None, init=False, repr=False)
    _arm_controller: Optional["ArmController"] = field(default=None, init=False, repr=False)

    def bind_safety_guard(self, safety_guard: object) -> None:
        self.safety_guard = safety_guard

    def _ensure_controllers(self) -> None:
        """懒加载：从 assembly 拿 loco/arm controller 引用。"""
        if self._loco_controller is None:
            self._loco_controller = getattr(self.assembly, "loco_controller", None)
        if self._arm_controller is None:
            self._arm_controller = getattr(self.assembly, "arm_controller", None)

    def is_available(self) -> bool:
        return bool(getattr(self.assembly, "high_level_initialized", False))

    def is_state_available(self) -> bool:
        return self.is_available()

    def is_image_available(self) -> bool:
        # G1 视频由 videohub 统一发布（与 Go2 同 videohub_pc4 路径），platform 走 videohub data plane
        return self.is_available()

    # ------------------ RobotState（g1.get_state capability 走这里） ------------------

    def get_robot_state(self) -> RobotState:
        initialized = bool(getattr(self.assembly, "high_level_initialized", False))
        self._ensure_controllers()
        # get_state 误用防爆：loco_controller 未收到 SportModeState_ 帧时给空 snapshot
        if self._loco_controller is not None and not self._loco_controller.has_state_frame():
            return RobotState(
                robot_id="g1",
                robot_model="unitree_g1",
                control_mode=_to_control_mode("loco"),
                safety_state=SafetyState(level="unknown", message="no frame"),
                joint_states=[],
                metadata={
                    "ready": False,
                    "reason": "no frame",
                    "timestamp": 0.0,
                    "control_mode": "loco",
                    "low_level_ready": False,
                    "low_level_running": False,
                    "adapter_count": 0,
                    "healthy_adapter_count": 0,
                },
            )
        metadata = {
            "control_mode": "loco",
            "low_level_ready": False,
            "low_level_running": False,
            "adapter_count": len(getattr(self.assembly, "_adapter_runtime", {}) or {}),
            "healthy_adapter_count": 0,
        }
        return RobotState(
            robot_id="g1",
            robot_model="unitree_g1",
            control_mode=_to_control_mode("loco"),
            safety_state=SafetyState(level="ok", message="normal") if initialized else SafetyState(level="unknown", message="not initialized"),
            joint_states=[],
            metadata=metadata,
        )

    # ------------------ Image（占位；实机视频走 videohub，不在本 provider 暴露） ------------------

    def get_image(self, camera_id: str) -> Optional[ImageFrame]:
        # G1 视频由 videohub 统一发布（与 Go2 同 videohub_pc4 路径），platform 走 videohub data plane
        return None

    def get_camera_info(self, camera_id: str) -> Optional[CameraInfo]:
        # 物理内参由 drivers/g1/defaults.py 注入；本期沿用 Go2 内参占位（后续 G1 棋盘格重标后写回）
        return None

    # ------------------ PointCloud（videohub 复路径；本期不实装 SLAM） ------------------

    def get_latest_point_cloud(self, *, max_points: Optional[int] = None) -> Optional[PointCloudFrame]:
        # 本期不实装点云 provider（cross-robot-generalization-foundations / G1WebRtcSlamScript 降级）
        return None

    def get_latest_local_point_cloud(self, *, max_points: Optional[int] = None) -> Optional[PointCloudFrame]:
        return None

    # ------------------ Motion（g1.move capability 走这里） ------------------

    def send_twist(self, twist: Twist) -> None:
        self._ensure_controllers()
        if self._loco_controller is None or not self._loco_controller.is_running():
            raise RuntimeError("G1 LocoController 未初始化，无法 send_twist")
        vx = float(getattr(twist, "linear_x", 0.0) or 0.0)
        vy = float(getattr(twist, "linear_y", 0.0) or 0.0)
        vyaw = float(getattr(twist, "angular_z", 0.0) or 0.0)
        self._loco_controller.move(vx, vy, vyaw)

    def stop_motion(self) -> None:
        self._ensure_controllers()
        if self._loco_controller is None or not self._loco_controller.is_running():
            return
        self._loco_controller.move(0.0, 0.0, 0.0)

    # ------------------ Safety（占位） ------------------

    def get_safety_state(self) -> SafetyState:
        initialized = bool(getattr(self.assembly, "high_level_initialized", False))
        if not initialized:
            return SafetyState(level="unknown", message="G1 not initialized")
        return SafetyState(level="ok", message="G1 normal")

    # ------------------ Calibration（G1 物理外参/内参；本期从 defaults 注入） ------------------

    def get_sensor_topology(self) -> Optional[SensorTopology]:
        # G1 物理传感器拓扑本期不实装（待 SLAM 栈接入后再补）
        return None
