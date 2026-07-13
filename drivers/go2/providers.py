from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, TYPE_CHECKING

from contracts.calibration import SensorTopology
from contracts.geometry import FrameTree, Pose
from contracts.image import CameraInfo, ImageEncoding, ImageFrame
from contracts.pointcloud import PointCloudFrame
from contracts.robot_state import IMUState, JointState, RobotControlMode, RobotState, SafetyState
from drivers.go2.defaults import GO2_DEFAULT_CAMERA_INFO, GO2_DEFAULT_SENSOR_TOPOLOGY
from providers.calibration import CalibrationProvider
from providers.image import ImageProvider
from providers.localization import StreamingLocalizationProvider
from providers.pointcloud import PointCloudProvider, StreamingPointCloudProvider
from providers.state import StateProvider, StreamingStateProvider

if TYPE_CHECKING:
    from drivers.go2.assembly import Go2RobotAssembly


def _to_control_mode(mode: str) -> RobotControlMode:
    if mode == "low":
        return RobotControlMode.LOW_LEVEL
    if mode == "high":
        return RobotControlMode.HIGH_LEVEL
    return RobotControlMode.UNKNOWN


@dataclass
class Go2ProviderBundle(
    StateProvider,
    ImageProvider,
    PointCloudProvider,
    StreamingPointCloudProvider,
    StreamingLocalizationProvider,
    StreamingStateProvider,
    CalibrationProvider,
):
    """Go2 默认提供器集合。"""

    assembly: "Go2RobotAssembly"
    provider_name: str = "go2_default_bundle"
    provider_version: str = "0.1.0"
    safety_guard: Optional[object] = None

    def bind_safety_guard(self, safety_guard: object) -> None:
        self.safety_guard = safety_guard

    def is_available(self) -> bool:
        return self.assembly.high_level_initialized

    def is_state_available(self) -> bool:
        return self.assembly.high_level_initialized

    def is_image_available(self) -> bool:
        return self.assembly.high_level_initialized

    def get_robot_state(self) -> RobotState:
        metadata = {
            "control_mode": self.assembly.current_mode,
            "mode_state": {
                "control_mode": self.assembly.current_mode,
                "high_level_initialized": self.assembly.high_level_initialized,
                "device_data_plane": "sensor_only",
            },
            "safe_stop": self._get_safe_stop_state(),
            "default_sensors": list(self.assembly.manifest.default_sensors),
            "default_audio_backends": list(self.assembly.manifest.default_audio_backends),
        }
        try:
            metadata["robot_volume"] = self.assembly.get_vui_volume_info()
        except Exception:
            metadata["robot_volume"] = None
        metadata["volume_state"] = metadata["robot_volume"]
        if self.assembly.data_plane is not None:
            data_plane_status = self.assembly.data_plane.get_status()
            metadata["data_plane"] = data_plane_status

        return RobotState(
            robot_id=self.assembly.defaults.parameters["robot_id"],
            frame_id=self.assembly.defaults.frame_ids["base"],
            mode=_to_control_mode(self.assembly.current_mode),
            pose=self._read_current_pose_for_state(),
            joints=self.get_joint_states(),
            imu=self.get_imu_state(),
            safety=self.get_safety_state(),
            metadata=metadata,
        )

    def get_joint_states(self) -> List[JointState]:
        return []

    def get_imu_state(self) -> Optional[IMUState]:
        if self.assembly.data_plane is not None:
            imu_state = self.assembly.data_plane.get_imu_state()
            if imu_state is not None:
                return imu_state
        return None

    def capture_image(self, camera_id: Optional[str] = None) -> ImageFrame:
        image_bytes = self.assembly.capture_image_bytes()
        camera_info = self.get_camera_info(camera_id)
        assert camera_info is not None
        return ImageFrame(
            camera_id=camera_id or camera_info.camera_id,
            frame_id=camera_info.frame_id,
            width_px=camera_info.width_px,
            height_px=camera_info.height_px,
            encoding=ImageEncoding.JPEG,
            data=image_bytes,
            metadata={"size_bytes": len(image_bytes)},
        )

    def get_camera_info(self, camera_id: Optional[str] = None) -> Optional[CameraInfo]:
        camera_info = GO2_DEFAULT_CAMERA_INFO.model_copy(deep=True)
        if camera_id:
            camera_info.camera_id = camera_id
        return camera_info

    def get_sensor_topology(self) -> Optional[SensorTopology]:
        """CalibrationProvider 接口：返回 Go2 传感器标定拓扑（外参初值 + 相机内参）。

        外参为 URDF 建模值 + 光学坐标系修正的静态初值（端侧物理参数唯一真相源），
        平台几何运行时装配期取一次合并进 frame_tree 静态边。见 drivers/go2/defaults.py。
        """
        return GO2_DEFAULT_SENSOR_TOPOLOGY.model_copy(deep=True)

    def subscribe_robot_state(self, callback: Callable[[RobotState], None]) -> Optional[int]:
        return self.subscribe_localization(lambda pose, frame_tree, metadata: callback(self._build_robot_state_from_stream(pose, metadata)))

    def unsubscribe_robot_state(self, subscription_id: Optional[int]) -> None:
        self.unsubscribe_localization(subscription_id)

    def subscribe_localization(
        self,
        callback: Callable[[Pose, Optional[FrameTree], Dict[str, object]], None],
    ) -> Optional[int]:
        if self.assembly.data_plane is None:
            return None
        subscriber = getattr(self.assembly.data_plane, "subscribe_localization", None)
        if not callable(subscriber):
            return None
        return subscriber(callback)

    def unsubscribe_localization(self, subscription_id: Optional[int]) -> None:
        if self.assembly.data_plane is None or subscription_id is None:
            return
        unsubscriber = getattr(self.assembly.data_plane, "unsubscribe_localization", None)
        if callable(unsubscriber):
            unsubscriber(subscription_id)

    def _build_robot_state_from_stream(self, pose: Optional[Pose], metadata: Dict[str, object]) -> RobotState:
        return RobotState(
            robot_id=self.assembly.defaults.parameters["robot_id"],
            frame_id=self.assembly.defaults.frame_ids["base"],
            mode=_to_control_mode(self.assembly.current_mode),
            pose=pose,
            joints=self.get_joint_states(),
            imu=self.get_imu_state(),
            safety=self.get_safety_state(),
            metadata={
                "control_mode": self.assembly.current_mode,
                "source_kind": "go2_dds_stream",
                **dict(metadata or {}),
            },
        )

    def _read_current_pose_for_state(self):
        if self.assembly.data_plane is None:
            return None
        getter = getattr(self.assembly.data_plane, "get_current_pose", None)
        if not callable(getter):
            return None
        return getter()

    def get_latest_point_cloud(self) -> Optional[PointCloudFrame]:
        if self.assembly.data_plane is None:
            return None
        getter = getattr(self.assembly.data_plane, "get_latest_local_point_cloud", None)
        if getter is None:
            return None
        point_cloud = getter()
        return point_cloud.model_copy(deep=True) if point_cloud is not None else None

    def subscribe_point_cloud(self, callback: Callable[[PointCloudFrame], None]) -> Optional[int]:
        if self.assembly.data_plane is None:
            return None
        subscriber = getattr(self.assembly.data_plane, "subscribe_local_point_cloud", None)
        if not callable(subscriber):
            return None
        return subscriber(callback)

    def unsubscribe_point_cloud(self, subscription_id: Optional[int]) -> None:
        if self.assembly.data_plane is None or subscription_id is None:
            return
        unsubscriber = getattr(self.assembly.data_plane, "unsubscribe_local_point_cloud", None)
        if callable(unsubscriber):
            unsubscriber(subscription_id)

    def get_safety_state(self) -> SafetyState:
        safe_stop_state = self._get_safe_stop_state()
        is_estopped = bool(safe_stop_state.get("is_estopped", False))
        return SafetyState(
            is_estopped=is_estopped,
            motors_enabled=False,
            can_move=False,
            warning_message=(
                str(safe_stop_state.get("last_stop_reason") or "安全停止门禁处于激活状态。")
                if is_estopped
                else "设备提供器仅报告传感状态，运动授权由 motion safety guard 决定。"
            ),
        )

    def _get_safe_stop_state(self) -> Dict[str, object]:
        if self.safety_guard is None:
            return {}
        state_getter = getattr(self.safety_guard, "get_safe_stop_state", None)
        if not callable(state_getter):
            return {}
        state = state_getter()
        return dict(state) if isinstance(state, dict) else {}
