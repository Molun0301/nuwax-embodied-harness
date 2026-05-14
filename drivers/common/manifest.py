from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple, Any

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
