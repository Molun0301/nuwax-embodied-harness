"""设施控制的同步动作与实际状态；机器人运动由导航和运动守卫执行。"""
from __future__ import annotations

from typing import Optional, Protocol, runtime_checkable

from contracts.building_navigation import FacilitySnapshot
from providers.base import BaseProvider


class FacilityUnavailable(RuntimeError):
    """设施动作失败或状态无法读取。"""


@runtime_checkable
class FacilityProvider(BaseProvider, Protocol):
    supported_connection_kinds: tuple[str, ...]

    def snapshot(self, facility_id: str, *, timeout_sec: Optional[float] = None) -> FacilitySnapshot:
        """读取设施当前停层、门、运动、占用与停靠反馈。"""

    def operate(self, facility_id: str, action: str, target_floor_id: Optional[str] = None,
                timeout_sec: Optional[float] = None) -> dict:
        """同步执行reserve/call/hold_door/transport/release，失败抛FacilityUnavailable。"""

    def close(self) -> None:
        """关闭本提供器拥有的连接。"""
