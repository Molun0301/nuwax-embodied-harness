from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple, Any

from contracts.capabilities import CapabilityMatrix


@dataclass(frozen=True)
class ComponentBinding:
    """机器人组成部件声明。"""

    name: str
    path: str
    description: str
    required: bool = True


@dataclass(frozen=True)
class RobotDefaults:
    """机器人默认配置。"""

    frame_ids: Dict[str, str]
    topics: Dict[str, str]
    parameters: Dict[str, Any] = field(default_factory=dict)


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
