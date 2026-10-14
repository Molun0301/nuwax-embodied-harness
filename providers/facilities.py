"""设施只负责预约、停层和门状态；机器人运动仍由原导航与运动守卫执行。"""
from __future__ import annotations

from typing import Protocol, runtime_checkable
from contracts.building_navigation import FacilityOperation, FacilitySnapshot
from providers.base import BaseProvider


class FacilityUnavailable(RuntimeError):
    """设施当前不可用或未取得可信状态。"""


class FacilityOutcomeUnknown(FacilityUnavailable):
    """请求可能已执行，必须按原操作标识查询，不能盲目重发。"""


@runtime_checkable
class FacilityProvider(BaseProvider, Protocol):
    """跨层设施的具幂等标识操作和只读状态接口。"""

    supported_connection_kinds: tuple[str, ...]

    def snapshot(self, facility_id: str, *, timeout_sec: float = 3.0) -> FacilitySnapshot:
        """读取设施实际状态，保留其采样时间和单调序列。"""

    def submit_operation(self, facility_id: str, operation: FacilityOperation, *, timeout_sec: float = 3.0) -> FacilityOperation:
        """以本次会话和幂等标识提交操作；不确定时抛出未决异常。"""

    def query_operation(self, facility_id: str, operation: FacilityOperation, *, timeout_sec: float = 3.0) -> FacilityOperation:
        """查询同一幂等标识；明确未发现与结果未知必须分开。"""

    def close(self) -> None:
        """关闭本提供器拥有的连接。"""


@runtime_checkable
class BoundedDockingProvider(BaseProvider, Protocol):
    """已标定的短程对接器；只使用调用方提供的原运动守卫会话。"""

    calibration_ref: str

    def execute_docking(self, connection, *, guard_session, ensure_active, remaining_sec, interlock):
        """以设备里程计/近场观察闭合有限对接，返回reached及actual_distance_m；不得另建速度输出。"""
