from __future__ import annotations

from typing import Optional, TYPE_CHECKING

from drivers.common.manifest import ComponentBinding, RobotManifest
from drivers.go2.capabilities import GO2_CAPABILITY_MATRIX
from drivers.go2.defaults import GO2_DEFAULT_ADAPTER_BINDINGS, GO2_DEFAULT_AUDIO_BINDINGS, GO2_DEFAULT_SENSOR_BINDINGS

if TYPE_CHECKING:
    from settings import NuwaxRobotBridgeConfig


def build_go2_manifest(config: Optional["NuwaxRobotBridgeConfig"] = None) -> RobotManifest:
    """构造 Go2 机器人入口清单。"""

    del config
    return RobotManifest(
        robot_name="go2",
        robot_model="unitree_go2",
        entrypoint="drivers/go2/assembly.py:create_go2_assembly",
        description="Go2 当前默认入口，聚合高层运动、前置图像、VUI 音量与低层关节控制。",
        capability_matrix=GO2_CAPABILITY_MATRIX,
        required_components=(
            ComponentBinding(
                name="sport_client",
                path="unitree_sdk2py.go2.sport.sport_client.SportClient",
                description="Go2 高层运动控制客户端。",
            ),
            ComponentBinding(
                name="video_client",
                path="unitree_sdk2py.go2.video.video_client.VideoClient",
                description="Go2 前置图像客户端。",
            ),
            ComponentBinding(
                name="vui_client",
                path="unitree_sdk2py.go2.vui.vui_client.VuiClient",
                description="Go2 机身音量与语音开关客户端。",
            ),
        ),
        optional_components=(
            ComponentBinding(
                name="low_level_controller",
                path="drivers.go2.control.low_level_controller.LowLevelController",
                description="Go2 低层关节控制器。",
                required=False,
            ),
            ComponentBinding(
                name="go2_data_plane_runtime",
                path="drivers.go2.data_plane.Go2DataPlaneRuntime",
                description="Go2 端侧定位、地图合成、官方导航执行与探索数据面运行时。",
                required=False,
            ),
            ComponentBinding(
                name="go2_data_plane_settings",
                path="drivers.go2.settings.load_go2_data_plane_config",
                description="Go2 端侧定位、地图、导航执行与探索配置加载入口。",
                required=False,
            ),
        ),
        default_sensors=GO2_DEFAULT_SENSOR_BINDINGS,
        default_audio_backends=GO2_DEFAULT_AUDIO_BINDINGS,
        default_adapters=GO2_DEFAULT_ADAPTER_BINDINGS,
    )


GO2_MANIFEST = build_go2_manifest()
