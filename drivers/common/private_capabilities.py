from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

from contracts.capabilities import CapabilityRiskLevel


@dataclass(frozen=True)
class PrivateCapabilitySpec:
    """机器人私有能力注册规格。"""

    name: str
    display_name: str
    description: str
    risk_level: CapabilityRiskLevel
    input_schema: Dict[str, object] = field(default_factory=dict)
    output_schema: Dict[str, object] = field(default_factory=dict)
    required_resources: Tuple[str, ...] = ()
    exposed_to_agent: bool = True
    timeout_sec: Optional[int] = None
    cancel_supported: bool = False
    tags: Tuple[str, ...] = ()


PLATFORM_COMMON_PRIVATE_CAPABILITY_SPECS: tuple[PrivateCapabilitySpec, ...] = ()
