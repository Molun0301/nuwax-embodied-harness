from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple, TYPE_CHECKING

from contracts.image import CameraInfo
from drivers.common.manifest import RobotDefaults

if TYPE_CHECKING:
    from settings import NuwaxRobotBridgeConfig


@dataclass(frozen=True)
class Go2ClientTimeouts:
    """Go2 客户端默认超时。"""

    sport_timeout_sec: float = 10.0
    video_timeout_sec: float = 3.0
    vui_timeout_sec: float = 3.0
    post_init_stabilize_sec: float = 1.0
    move_for_interval_sec: float = 0.5


GO2_TIMEOUTS = Go2ClientTimeouts()

GO2_DEFAULT_FRAME_IDS = {
    "base": "world/go2/base",
    "imu": "world/go2/imu",
    "camera_front": "world/go2/camera_front",
}

GO2_DEFAULT_TOPICS = {
    "robot_state": "robot.go2.state",
    "image_front": "robot.go2.image.front",
    "joint_state": "robot.go2.joint_state",
    "imu_state": "robot.go2.imu_state",
    "robot_volume": "robot.go2.audio.volume",
    "robot_odom": "/utlidar/robot_odom",
    "utlidar_imu": "/utlidar/imu",
    "utlidar_cloud": "/utlidar/cloud",
    "tf": "/tf",
    "tf_static": "/tf_static",
}

GO2_DEFAULT_SENSOR_BINDINGS = (
    "drivers/sensor/cameras/unitree/front_camera",
    "drivers/sensor/imu/unitree/body_imu",
)

GO2_DEFAULT_AUDIO_BINDINGS = (
    "drivers/audio/unitree_vui",
    "drivers/audio/unitree_webrtc",
)

GO2_DEFAULT_ADAPTER_BINDINGS: Tuple[str, ...] = ()

GO2_DEFAULT_CAMERA_INFO = CameraInfo(
    camera_id="front_camera",
    frame_id=GO2_DEFAULT_FRAME_IDS["camera_front"],
    width_px=1280,
    height_px=720,
    fx=819.553492,
    fy=820.646595,
    cx=625.284099,
    cy=336.808987,
    distortion_model="plumb_bob",
    distortion_coefficients=[0.0, 0.0, 0.0, 0.0, 0.0],
)


def build_go2_defaults(config: "NuwaxRobotBridgeConfig", iface: Optional[str] = None) -> RobotDefaults:
    """生成 Go2 默认配置。"""

    return RobotDefaults(
        frame_ids=dict(GO2_DEFAULT_FRAME_IDS),
        topics=dict(GO2_DEFAULT_TOPICS),
        parameters={
            "robot_id": "go2",
            "robot_model": "unitree_go2",
            "dds_iface": iface or config.dds.iface,
            "sdk_path": config.dds.sdk_path,
            "sport_timeout_sec": GO2_TIMEOUTS.sport_timeout_sec,
            "video_timeout_sec": GO2_TIMEOUTS.video_timeout_sec,
            "vui_timeout_sec": GO2_TIMEOUTS.vui_timeout_sec,
            "move_for_interval_sec": GO2_TIMEOUTS.move_for_interval_sec,
            "default_low_level_kp": config.low_level.default_kp,
            "default_low_level_kd": config.low_level.default_kd,
            "max_low_level_velocity": config.low_level.max_velocity,
            "max_low_level_torque": config.low_level.max_torque,
        },
    )
