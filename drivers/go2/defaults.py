from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple, TYPE_CHECKING

from contracts.calibration import SensorPair, SensorTopology
from contracts.geometry import Quaternion, Vector3
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
    "robot_odom": "/nuwax/geometry/odom",
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
    width_px=1920,
    height_px=1080,
    fx=1229.330238,
    fy=1230.969893,
    cx=937.926149,
    cy=505.213481,
    distortion_model="plumb_bob",
    distortion_coefficients=[0.0, 0.0, 0.0, 0.0, 0.0],
)
# 分辨率为实机 VideoClient.GetImageSample() 实测 1920×1080（与推流一致）。
# 内参为旧 1280×720 标定值按 1.5× 等比缩放到 1920×1080 的占位值，保证“内参分辨率
# 与帧分辨率自洽”，可用于链路连通性测试；但仍非实机精标值。
# 待 Phase-A 用 tools/calibration/intrinsic_chessboard.py 在实机棋盘格重标后写回真值。
# 这是机器人特定物理参数，按平台分层治理规范归端侧 driver 唯一真相源。

# camera↔lidar 外参初值（来源：unitree_ros go2_description.urdf 建模值，非标定值）。
# - 平移：相机光心在 base 系下的位置 xyz(0.32715, -0.00003, 0.04297)。
# - 旋转：URDF camera_link 的 rpy=(0,0,0) 是与 base 对齐的“连杆系”，并非相机
#   光学系。光学系（REP-103：+Z 前、+X 右、+Y 下）→ base 系（+X 前、+Y 左、+Z 上）
#   的旋转矩阵为 [[0,0,1],[-1,0,0],[0,-1,0]]，对应四元数 (x,y,z,w)=(-0.5,0.5,-0.5,0.5)。
#   故此处 initial_rotation 描述“相机光学系点 → base 系点”的刚体旋转，使
#   Transform(parent=base, child=camera_optical) 语义正确，点云不会被投到相机背后。
# 诚实标注：本期为静态初值，系统偏差上界 ~5-20cm（见评测报告）；在线 MI 精修移后续。
GO2_CAMERA_LIDAR_EXTRINSIC_SOURCE = "urdf_modeling_value+optical_frame_correction"

GO2_DEFAULT_SENSOR_TOPOLOGY = SensorTopology(
    robot_id="go2",
    pairs=[
        SensorPair(
            parent_frame_id=GO2_DEFAULT_FRAME_IDS["base"],
            child_frame_id=GO2_DEFAULT_CAMERA_INFO.frame_id,
            point_cloud_frame_id="utlidar_lidar",
            initial_translation=Vector3(x=0.32715, y=-0.00003, z=0.04297),
            initial_rotation=Quaternion(x=-0.5, y=0.5, z=-0.5, w=0.5),
            camera_fx=GO2_DEFAULT_CAMERA_INFO.fx,
            camera_fy=GO2_DEFAULT_CAMERA_INFO.fy,
            camera_cx=GO2_DEFAULT_CAMERA_INFO.cx,
            camera_cy=GO2_DEFAULT_CAMERA_INFO.cy,
            camera_width_px=GO2_DEFAULT_CAMERA_INFO.width_px,
            camera_height_px=GO2_DEFAULT_CAMERA_INFO.height_px,
            camera_distortion_model=GO2_DEFAULT_CAMERA_INFO.distortion_model or "plumb_bob",
            camera_distortion_coeffs=tuple(GO2_DEFAULT_CAMERA_INFO.distortion_coefficients),
        ),
    ],
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
