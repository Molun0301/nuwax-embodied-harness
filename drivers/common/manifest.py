from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple, Any

from contracts.capabilities import CapabilityMatrix
from contracts.naming import validate_frame_id, validate_topic_name


@dataclass(frozen=True)
class ComponentBinding:
    """机器人组成部件声明。"""

    name: str
    path: str
    description: str
    required: bool = True


@dataclass(frozen=True)
class RobotProfile:
    """机器人能力画像（cross-robot-generalization-foundations G1）。

    由 driver/manifest.py 声明，installer 与运行时按画像编排 phase / 选 SLAM 后端 /
    配 telemetry 间隔 / 选视频流后端。任何新增字段必须走 OpenSpec ADDED Requirements 流程。
    """

    robot_name: str
    tier_recommendation: str  # "full" | "lite"
    required_phases: Tuple[str, ...] = ()
    forbidden_phases: Tuple[str, ...] = ()
    required_apt_packages: Tuple[str, ...] = ()
    forbidden_apt_packages: Tuple[str, ...] = ()
    required_python_packages: Tuple[str, ...] = ()
    external_slam_script: str = ""  # "Go2ShSlamScript" | "G1WebRtcSlamScript" | "RaspiSlamToolboxScript" | ""
    calibration_required: bool = True
    slam_dimension: str = "3d"  # "2d" | "3d"
    memory_mode_recommendation: str = "cloud"  # "cloud" | "local" | "disabled"
    perception_mode_recommendation: str = "hybrid"  # "local" | "hybrid" | "cloud-scene" | "disabled"
    telemetry_interval_sec: float = 2.0
    telemetry_process_keywords: Tuple[str, ...] = ("gateway", "robot_bridge", "ros2")


@dataclass(frozen=True)
class RobotDefaults:
    """机器人默认配置。"""

    frame_ids: Dict[str, str]
    topics: Dict[str, str]
    parameters: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for key, value in (self.frame_ids or {}).items():
            try:
                validate_frame_id(value)
            except ValueError as exc:
                raise ValueError(
                    f"RobotDefaults.frame_ids[{key!r}] 命名不合规: {exc}"
                ) from exc
        for key, value in (self.topics or {}).items():
            try:
                validate_topic_name(value)
            except ValueError as exc:
                raise ValueError(
                    f"RobotDefaults.topics[{key!r}] 命名不合规: {exc}"
                ) from exc


@dataclass(frozen=True)
class RobotManifest:
    """机器人入口清单。"""

    robot_name: str
    robot_model: str
    entrypoint: str
    description: str
    capability_matrix: CapabilityMatrix
    required_components: Tuple[ComponentBinding, ...] = ()
    optional_components: Tuple[ComponentBinding, ...] = ()
    default_sensors: Tuple[str, ...] = ()
    default_audio_backends: Tuple[str, ...] = ()
    default_adapters: Tuple[str, ...] = ()
    # 标定 SSOT 路径（cross-robot-generalization-foundations G4）。可 import 字符串如
    # "drivers.go2.defaults:GO2_DEFAULT_SENSOR_TOPOLOGY"；空字符串表示该机器人不需要标定。
    sensor_topology_ref: str = ""
    # 机器人能力画像（cross-robot-generalization-foundations G1）。driver 必须提供，否则 __post_init__ 抛 ValueError。
    robot_profile: Optional[RobotProfile] = None

    def __post_init__(self) -> None:
        if self.robot_profile is None:
            raise ValueError(
                f"RobotManifest({self.robot_name}) 缺 robot_profile；"
                f"请按 openspec 通用化骨架 delta 落地（cross-robot-generalization-foundations）。"
            )
        if self.robot_profile.robot_name != self.robot_name:
            raise ValueError(
                f"RobotManifest.robot_name={self.robot_name!r} 与 "
                f"robot_profile.robot_name={self.robot_profile.robot_name!r} 不一致。"
            )
