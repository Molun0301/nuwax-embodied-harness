"""通用机器人输入缓存；接入方发布标准数据，平台服务读取或订阅。"""

from __future__ import annotations

from datetime import datetime, timezone
import logging
from threading import RLock
from typing import Callable, Dict, List, Optional, Union

import numpy as np

from contracts.calibration import SensorTopology
from contracts.geometry import Twist, Vector3
from contracts.image import CameraInfo, ImageFrame
from contracts.pointcloud import PointCloudFrame
from contracts.robot_state import IMUState, JointState, RobotState, SafetyState
from contracts.sensors import CanonicalImuSample, CanonicalLidarFrame, SensorClockDomain, SensorSample
from providers.motion import MotionControl


LOGGER = logging.getLogger(__name__)


class CustomRobotProviderBundle:
    """无需厂商依赖的标准输入装配，适用于自定义驱动和离线回放。

    发布后数据由提供器持有，生产者不得继续修改同一帧。回调同步执行且应快速
    返回；耗时算法应由平台工作线程处理。这里不生成地图、位姿或传感器标定。
    """

    provider_name = "customrobot_standard_inputs"
    provider_version = "0.1.0"

    def __init__(self, *, motion_control: Optional[MotionControl] = None,
                 sensor_topology: Optional[SensorTopology] = None) -> None:
        self._lock = RLock()
        self._started = False
        self._motion_control = motion_control
        self._sensor_topology = sensor_topology.model_copy(deep=True) if sensor_topology else None
        self._robot_state: Optional[RobotState] = None
        self._imu: Optional[IMUState] = None
        self._point_cloud: Optional[PointCloudFrame] = None
        self._images: Dict[str, ImageFrame] = {}
        self._camera_infos: Dict[str, CameraInfo] = {}
        self._subscriptions: Dict[int, tuple] = {}
        self._next_subscription_id = 1

    def start(self) -> None:
        with self._lock:
            self._started = True

    def stop(self) -> None:
        with self._lock:
            was_started = self._started
            self._started = False
            self._robot_state = None
            self._imu = None
            self._point_cloud = None
            self._images.clear()
        if was_started and self._motion_control is not None:
            self._motion_control.stop_motion()

    def is_available(self) -> bool:
        with self._lock:
            return self._started

    def is_state_available(self) -> bool:
        with self._lock:
            return self._started and self._robot_state is not None

    def is_image_available(self) -> bool:
        with self._lock:
            return self._started and bool(self._images)

    def get_robot_state(self) -> RobotState:
        with self._lock:
            if not self._started or self._robot_state is None:
                raise RuntimeError("尚未收到机器人状态。")
            return self._robot_state.model_copy(deep=True)

    def get_joint_states(self) -> List[JointState]:
        with self._lock:
            return [joint.model_copy(deep=True) for joint in self._robot_state.joints] if self._robot_state else []

    def get_imu_state(self) -> Optional[IMUState]:
        with self._lock:
            return self._imu.model_copy(deep=True) if self._imu is not None else None

    def get_safety_state(self) -> SafetyState:
        with self._lock:
            state, motion, started = self._robot_state, self._motion_control, self._started
        if not started or motion is None or not motion.is_available():
            return SafetyState(
                motors_enabled=state.safety.motors_enabled if state is not None else None,
                can_move=False,
                warning_message="运动执行器未接入或尚未就绪。",
            )
        if state is None:
            return SafetyState(motors_enabled=None, can_move=False, warning_message="尚未收到机器人状态。")
        return state.safety.model_copy(deep=True)

    def publish_robot_state(self, state: RobotState) -> bool:
        if not isinstance(state, RobotState):
            raise TypeError("机器人状态必须是 RobotState。")
        with self._lock:
            if not self._started:
                return False
            self._robot_state = state.model_copy(deep=True)
            if state.imu is not None:
                self._imu = state.imu
        self._notify("state", state)
        return True

    def publish_imu(self, sample: Union[IMUState, CanonicalImuSample]) -> bool:
        """规范 IMU 直达高频输入流，无需设备提供定位位姿。"""

        if isinstance(sample, CanonicalImuSample):
            return self.publish_sensor_sample(sample)
        if not isinstance(sample, IMUState):
            raise TypeError("IMU 必须是 IMUState 或 CanonicalImuSample。")
        with self._lock:
            if not self._started:
                return False
            self._imu = sample
            if self._robot_state is not None:
                self._robot_state = self._robot_state.model_copy(update={"imu": sample})
        return True

    def publish_image(self, frame: ImageFrame, camera_info: Optional[CameraInfo] = None) -> bool:
        if not isinstance(frame, ImageFrame):
            raise TypeError("图像必须是 ImageFrame。")
        if camera_info is not None and (
            camera_info.camera_id != frame.camera_id or camera_info.frame_id != frame.frame_id
            or camera_info.width_px != frame.width_px or camera_info.height_px != frame.height_px
        ):
            raise ValueError("图像与相机标定的标识、坐标系及尺寸必须一致。")
        with self._lock:
            if not self._started:
                return False
            self._images[frame.camera_id] = frame
            if camera_info is not None:
                self._camera_infos[frame.camera_id] = camera_info.model_copy(deep=True)
        return True

    def capture_image(self, camera_id: Optional[str] = None) -> ImageFrame:
        with self._lock:
            if not self._started or not self._images:
                raise RuntimeError("尚未收到相机图像。")
            selected = camera_id or next(iter(self._images))
            if selected not in self._images:
                raise ValueError("未接入相机：%s" % selected)
            return self._images[selected].model_copy(deep=True)

    def get_latest_image(self, camera_id: Optional[str] = None) -> Optional[ImageFrame]:
        """只读访问已发布帧，不采集硬件，也不复制图像字节。"""

        with self._lock:
            if not self._started:
                return None
            selected = camera_id or next(iter(self._images), None)
            return self._images.get(selected)

    def status_snapshot(self) -> dict:
        """最近图像接收状态，供持续感知判定是否已有输入。"""

        with self._lock:
            available = self._started and bool(self._images)
            return {
                "available": available,
                "connected": self._started,
                "reason": "ok" if available else "image_not_received" if self._started else "stopped",
                "camera_ids": list(self._images) if self._started else [],
                "source_kind": "standard_image_provider",
            }

    def get_camera_info(self, camera_id: Optional[str] = None) -> Optional[CameraInfo]:
        with self._lock:
            selected = camera_id or next(iter(self._images), None)
            info = self._camera_infos.get(selected)
            return info.model_copy(deep=True) if info is not None else None

    def publish_point_cloud(self, frame: Union[PointCloudFrame, CanonicalLidarFrame]) -> bool:
        if isinstance(frame, CanonicalLidarFrame):
            return self.publish_sensor_sample(frame)
        if not isinstance(frame, PointCloudFrame):
            raise TypeError("点云必须是 PointCloudFrame 或 CanonicalLidarFrame。")
        with self._lock:
            if not self._started:
                return False
            self._point_cloud = frame
        self._notify("point_cloud", frame)
        return True

    def get_latest_point_cloud(self) -> Optional[PointCloudFrame]:
        with self._lock:
            return self._point_cloud if self._started else None

    def publish_sensor_sample(self, sample: SensorSample) -> bool:
        """发布带真实时间与标定引用的规范 LiDAR/IMU 数据。"""

        if not isinstance(sample, (CanonicalImuSample, CanonicalLidarFrame)):
            raise TypeError("规范输入必须是 CanonicalImuSample 或 CanonicalLidarFrame。")
        with self._lock:
            if not self._started:
                return False
        # 规范高频输入优先交给算法；显示用快照不改变源时间域，也不阻塞投递。
        self._notify("sensor_sample", sample)
        timestamp = datetime.now(timezone.utc)
        if sample.clock_domain == SensorClockDomain.UNIX_NS:
            try:
                timestamp = datetime.fromtimestamp(sample.timestamp_ns / 1_000_000_000, timezone.utc)
            except (OverflowError, OSError, ValueError):
                pass
        if isinstance(sample, CanonicalImuSample):
            # 读状态使用标准向量，高频算法仍消费原始纳秒时间与时间域。
            self.publish_imu(IMUState(
                frame_id=sample.registration_frame_id,
                timestamp=timestamp,
                angular_velocity_rad_s=Vector3(x=sample.gyro_rps[0], y=sample.gyro_rps[1], z=sample.gyro_rps[2]),
                linear_acceleration_m_s2=Vector3(x=sample.accel_mps2[0], y=sample.accel_mps2[1], z=sample.accel_mps2[2]),
            ))
        elif sample.xyz_f32_le is not None:
            self.publish_point_cloud(PointCloudFrame(
                frame_id=sample.registration_frame_id,
                timestamp=timestamp,
                point_count=sample.point_count,
                points_xyz_array=np.frombuffer(sample.xyz_f32_le, dtype="<f4").reshape((-1, 3)),
                is_dense=sample.is_dense,
                metadata={
                    "canonical_lidar_frame": sample, "source_timestamp_ns": sample.timestamp_ns,
                    "clock_domain": sample.clock_domain.value, "clock_id": sample.clock_id,
                },
            ))
        return True

    def get_sensor_topology(self) -> Optional[SensorTopology]:
        return self._sensor_topology.model_copy(deep=True) if self._sensor_topology else None

    def send_twist(self, twist: Twist) -> None:
        if self.get_safety_state().can_move is not True:
            raise RuntimeError("运动执行器不可用。")
        assert self._motion_control is not None
        self._motion_control.send_twist(twist)

    def stop_motion(self) -> None:
        if self._motion_control is not None:
            self._motion_control.stop_motion()

    def _subscribe(self, channel: str, callback: Callable) -> int:
        if not callable(callback):
            raise TypeError("订阅者必须可调用。")
        with self._lock:
            subscription_id = self._next_subscription_id
            self._next_subscription_id += 1
            self._subscriptions[subscription_id] = (channel, callback)
            return subscription_id

    def _unsubscribe(self, subscription_id: Optional[int]) -> None:
        with self._lock:
            self._subscriptions.pop(subscription_id, None)

    def _notify(self, channel: str, sample: object) -> None:
        with self._lock:
            if not self._started:
                return
            callbacks = [callback for kind, callback in self._subscriptions.values() if kind == channel]
        for callback in callbacks:
            try:
                callback(sample)
            except Exception:
                LOGGER.exception("标准输入订阅者处理失败 channel=%s", channel)

    def subscribe_robot_state(self, callback: Callable[[RobotState], None]) -> Optional[int]:
        return self._subscribe("state", callback)

    def unsubscribe_robot_state(self, subscription_id: Optional[int]) -> None:
        self._unsubscribe(subscription_id)

    def subscribe_point_cloud(self, callback: Callable[[PointCloudFrame], None]) -> Optional[int]:
        return self._subscribe("point_cloud", callback)

    def unsubscribe_point_cloud(self, subscription_id: Optional[int]) -> None:
        self._unsubscribe(subscription_id)

    def subscribe_sensor_sample(self, callback: Callable[[SensorSample], None]) -> Optional[int]:
        return self._subscribe("sensor_sample", callback)

    def unsubscribe_sensor_sample(self, subscription_id: Optional[int]) -> None:
        self._unsubscribe(subscription_id)
