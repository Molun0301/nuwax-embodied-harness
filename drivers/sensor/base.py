from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple


@dataclass(frozen=True)
class RegisteredSensorSpec:
    """传感器注册规格。"""

    sensor_name: str
    sensor_type: str
    entrypoint: str
    provided_streams: Tuple[str, ...] = ()
    required_config: Tuple[str, ...] = ()
    healthcheck_strategy: str = "none"
    metadata: Dict[str, object] = field(default_factory=dict)
