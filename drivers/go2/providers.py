from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, TYPE_CHECKING

from contracts.geometry import FrameTree, Pose, Quaternion, Twist, Vector3
from contracts.image import CameraInfo, ImageEncoding, ImageFrame
from contracts.pointcloud import PointCloudFrame
from contracts.robot_state import IMUState, JointState, RobotControlMode, RobotState, SafetyState
from drivers.common import MotionCommandDataPlane
from drivers.go2.defaults import GO2_DEFAULT_CAMERA_INFO
from providers.image import ImageProvider
from providers.localization import StreamingLocalizationProvider
from providers.motion import MotionControl
from providers.pointcloud import PointCloudProvider, StreamingPointCloudProvider
from providers.safety import SafetyProvider
from providers.state import StateProvider, StreamingStateProvider

if TYPE_CHECKING:
    from drivers.go2.assembly import Go2RobotAssembly


def _to_control_mode(mode: str) -> RobotControlMode:
    if mode == "low":
        return RobotControlMode.LOW_LEVEL
    if mode == "high":
        return RobotControlMode.HIGH_LEVEL
    return RobotControlMode.UNKNOWN


def _resolve_motion_command_data_plane(data_plane) -> Optional[MotionCommandDataPlane]:
    """返回支持底层运动命令链路的数据面。"""

    if data_plane is None or not isinstance(data_plane, MotionCommandDataPlane):
        return None
    if not data_plane.can_accept_motion_command():
        return None
    return data_plane


@dataclass
class Go2ProviderBundle(
    StateProvider,
    ImageProvider,
    PointCloudProvider,
    StreamingPointCloudProvider,
    StreamingLocalizationProvider,
    StreamingStateProvider,
    MotionControl,
    SafetyProvider,
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
        low_level_running = bool(
            self.assembly.low_level_controller is not None
            and getattr(self.assembly.low_level_controller, "is_running", False)
        )
        metadata = {
            "control_mode": self.assembly.current_mode,
            "mode_state": {
                "control_mode": self.assembly.current_mode,
                "low_level_ready": self.assembly.low_level_controller is not None,
                "low_level_running": low_level_running,
                "high_level_initialized": self.assembly.high_level_initialized,
            },
            "low_level_ready": self.assembly.low_level_controller is not None,
            "low_level_running": low_level_running,
            "motion_stop_routes": list(getattr(self.assembly, "_last_motion_stop_routes", [])),
            "motion_stop_errors": list(getattr(self.assembly, "_last_motion_stop_errors", [])),
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
            obstacle_status = data_plane_status.get("obstacle_avoidance") if isinstance(data_plane_status, dict) else None
            if isinstance(obstacle_status, dict):
                metadata["remote_command_enabled"] = bool(obstacle_status.get("remote_command_enabled", False))
                metadata["motion_command_route"] = obstacle_status.get("control_path")

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
        if self.assembly.low_level_controller is None:
            return []

        raw_state = self.assembly.low_level_controller.get_joint_state()
        joints = raw_state.get("joints", []) if isinstance(raw_state, dict) else []
        return [
            JointState(
                name=joint["name"],
                position_rad=float(joint["position"]),
                velocity_rad_s=float(joint["velocity"]),
                effort_nm=float(joint["torque"]),
            )
            for joint in joints
        ]

    def get_imu_state(self) -> Optional[IMUState]:
        if self.assembly.data_plane is not None:
            imu_state = self.assembly.data_plane.get_imu_state()
            if imu_state is not None:
                return imu_state

        if self.assembly.low_level_controller is None:
            return None

        raw_state = self.assembly.low_level_controller.get_imu_state()
        if not raw_state:
            return None

        quaternion = raw_state.get("quaternion", [0.0, 0.0, 0.0, 1.0])
        gyroscope = raw_state.get("gyroscope", [0.0, 0.0, 0.0])
        accelerometer = raw_state.get("accelerometer", [0.0, 0.0, 0.0])
        return IMUState(
            frame_id=self.assembly.defaults.frame_ids["imu"],
            orientation=Quaternion(
                x=float(quaternion[0]),
                y=float(quaternion[1]),
                z=float(quaternion[2]),
                w=float(quaternion[3]),
            ),
            angular_velocity_rad_s=Vector3(
                x=float(gyroscope[0]),
                y=float(gyroscope[1]),
                z=float(gyroscope[2]),
            ),
            linear_acceleration_m_s2=Vector3(
                x=float(accelerometer[0]),
                y=float(accelerometer[1]),
                z=float(accelerometer[2]),
            ),
        )

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

    def send_twist(self, twist: Twist) -> None:
        if self.assembly.current_mode == "low":
            raise RuntimeError("Go2 当前处于低层控制模式，不能通过高层速度链下发运动命令。")
        data_plane = _resolve_motion_command_data_plane(self.assembly.data_plane)
        if data_plane is not None:
            code = data_plane.send_motion_command(
                twist.linear.x,
                twist.linear.y,
                twist.angular.z,
            )
            if code != 0:
                raise RuntimeError(f"Go2 数据面运动命令执行失败: code={code}")
            return
        self.assembly.move(
            twist.linear.x,
            twist.linear.y,
            twist.angular.z,
        )

    def stop_motion(self) -> None:
        self.assembly.stop_all_motion(None)

    def get_safety_state(self) -> SafetyState:
        safe_stop_state = self._get_safe_stop_state()
        is_estopped = bool(safe_stop_state.get("is_estopped", False))
        return SafetyState(
            is_estopped=is_estopped,
            motors_enabled=self.assembly.high_level_initialized,
            can_move=self.assembly.high_level_initialized and not is_estopped,
            warning_message=str(safe_stop_state.get("last_stop_reason") or "安全停止门禁处于激活状态。") if is_estopped else None,
        )

    def _get_safe_stop_state(self) -> Dict[str, object]:
        if self.safety_guard is None:
            return {}
        state_getter = getattr(self.safety_guard, "get_safe_stop_state", None)
        if not callable(state_getter):
            return {}
        state = state_getter()
        return dict(state) if isinstance(state, dict) else {}

    def request_safe_stop(self, reason: Optional[str] = None) -> None:
        safe_stop_active = bool(getattr(self.safety_guard, "safe_stop_active", False))
        stop_all_motion = getattr(self.safety_guard, "stop_all_motion", None)
        if callable(stop_all_motion) and not safe_stop_active:
            stop_all_motion(reason)
            return
        self.assembly.stop_all_motion(reason)
