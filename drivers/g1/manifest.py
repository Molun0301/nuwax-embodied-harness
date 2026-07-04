from __future__ import annotations

from typing import Optional, TYPE_CHECKING

from drivers.common.manifest import ComponentBinding, RobotManifest, RobotProfile
from drivers.g1.capabilities import G1_CAPABILITY_MATRIX

if TYPE_CHECKING:
    from core.settings import NuwaxRobotBridgeConfig


def build_g1_manifest(config: Optional["NuwaxRobotBridgeConfig"] = None) -> RobotManifest:
    """构造 G1 入口骨架清单。"""

    del config
    return RobotManifest(
        robot_name="g1",
        robot_model="unitree_g1",
        entrypoint="drivers/g1/assembly.py:create_g1_assembly",
        description="G1 机器人入口（Unitree WebRTC，类 Go2 算力，3D SLAM）。",
        capability_matrix=G1_CAPABILITY_MATRIX,
        required_components=(
            ComponentBinding(
                name="webrtc_client",
                path="drivers.g1.webrtc_client.WebRtcClient",
                description="G1 WebRTC 客户端。",
            ),
        ),
        sensor_topology_ref="drivers.g1.defaults:G1_DEFAULT_SENSOR_TOPOLOGY",
        robot_profile=RobotProfile(
            robot_name="g1",
            tier_recommendation="full",
            required_phases=(
                "preflight", "system", "python_env", "geometry_deps",
                "build_ws", "config_env", "models", "mcp", "health",
            ),
            forbidden_phases=("data_plane",),
            required_apt_packages=(),
            forbidden_apt_packages=(),
            required_python_packages=("aiortc>=1.6", "aiohttp>=3.9"),
            external_slam_script="G1WebRtcSlamScript",
            calibration_required=True,
            slam_dimension="3d",
            memory_mode_recommendation="cloud",
            perception_mode_recommendation="hybrid",
            telemetry_interval_sec=2.0,
            telemetry_process_keywords=("gateway", "nuwax_geometry_runtime", "robot_bridge", "g1_webrtc_stream"),
        ),
    )


G1_MANIFEST = build_g1_manifest()
