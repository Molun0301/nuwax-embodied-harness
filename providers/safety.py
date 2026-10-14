from __future__ import annotations

from typing import Protocol, runtime_checkable

from contracts.robot_state import SafetyState
from providers.base import BaseProvider


@runtime_checkable
class SafetyProvider(BaseProvider, Protocol):
    """只读安全状态接口。"""

    def get_safety_state(self) -> SafetyState:
        """返回当前安全状态。"""
