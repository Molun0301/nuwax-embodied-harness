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
    # 通用分发约定：网关无专用 handler 时，按 dispatch 方式把本能力转发到机器人入口。
    # action_name 缺省取 name；dispatch 当前仅支持 "execute_action"（调 robot.execute_action）。
    action_name: Optional[str] = None
    dispatch: str = "execute_action"
