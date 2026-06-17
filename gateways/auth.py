from __future__ import annotations

from dataclasses import dataclass
import logging

from contracts.capabilities import CapabilityDescriptor, CapabilityRiskLevel
from gateways.errors import AccessDeniedError
from core.settings import GatewayServerConfig
from starlette.requests import Request
from starlette.websockets import WebSocket
from typing import Dict, Optional, Tuple


AUTH_LOGGER = logging.getLogger("nuwax.gateway.auth")


@dataclass(frozen=True)
class AccessContext:
    """访问上下文（个人机器狗：不鉴权，统一匿名本机身份）。

    保留 principal 字段仅供审计/`requested_by` 标注，不再承载任何权限语义。
    """

    principal: str
    source_host: Optional[str]


class GatewayAccessManager:
    """宿主机网关访问控制器。

    个人机器狗场景：不做任何用户鉴权（无令牌、无角色、无来源网段限制），
    任何能连到网关的调用方都被视为同一匿名本机身份。仅保留「能力风险门控」
    （capability risk gating）——它约束的是自主 LLM（大语言模型）能否擅自触发
    高危动作（如后空翻），与用户身份无关。
    """

    _ANONYMOUS = AccessContext(principal="local", source_host=None)

    def __init__(self, config: GatewayServerConfig) -> None:
        self._config = config
        # 高危能力白名单：仅此清单内的 HIGH 风险能力允许被（含 LLM 的）能力调用入口触发。
        self._high_risk_allowlist = frozenset(config.capability_high_risk_allowlist)

    def _anonymous(self, source_host: Optional[str]) -> AccessContext:
        if source_host is None:
            return self._ANONYMOUS
        return AccessContext(principal="local", source_host=source_host)

    def authenticate_http(self, request: Request) -> AccessContext:
        """HTTP 请求放行（不鉴权）。"""

        source_host = request.client.host if request.client is not None else None
        return self._anonymous(source_host)

    def authenticate_local_debug_page(self, request: Request) -> AccessContext:
        """本机调试页放行（不鉴权）。"""

        return self.authenticate_http(request)

    def authenticate_websocket(self, websocket: WebSocket) -> AccessContext:
        """WebSocket（网页套接字）请求放行（不鉴权）。"""

        source_host = websocket.client.host if websocket.client is not None else None
        return self._anonymous(source_host)

    def filter_capabilities(
        self,
        access: AccessContext,
        descriptors: Tuple[CapabilityDescriptor, ...],
    ) -> Tuple[CapabilityDescriptor, ...]:
        """过滤当前调用方可见的能力。"""

        return tuple(descriptor for descriptor in descriptors if self.can_access_capability(access, descriptor))

    def can_access_capability(self, access: AccessContext, descriptor: CapabilityDescriptor) -> bool:
        """判断某能力是否允许通过能力调用入口被触发。

        不再有用户角色之分；门控只取决于能力自身的风险等级与暴露标记，
        用于约束自主 LLM（大语言模型）擅自触发高危动作：
        - 未暴露给智能体的能力：拒绝（仅供内部/直连入口使用）。
        - LOW / MEDIUM：放行。
        - HIGH：仅当在高危白名单内才放行。
        """

        if not descriptor.exposed_to_agent:
            return False

        if descriptor.risk_level in {CapabilityRiskLevel.LOW, CapabilityRiskLevel.MEDIUM}:
            return True

        if descriptor.risk_level == CapabilityRiskLevel.HIGH:
            return descriptor.name in self._high_risk_allowlist

        return False

    def authorize_capability(self, access: AccessContext, descriptor: CapabilityDescriptor) -> None:
        """执行能力风险门控。"""

        if self.can_access_capability(access, descriptor):
            return
        raise AccessDeniedError(
            f"能力 {descriptor.name} 被风险门控拦截（高危能力需加入白名单）。",
            details={
                "capability_name": descriptor.name,
                "risk_level": descriptor.risk_level.value,
            },
        )

    def export_policy(self) -> Dict[str, object]:
        """导出访问策略摘要（不鉴权，仅暴露能力风险门控配置）。"""

        return {
            "auth": "disabled",
            "capability_high_risk_allowlist": sorted(self._high_risk_allowlist),
            "risk_policy": {
                "low": "放行",
                "medium": "放行",
                "high": "仅高危白名单内能力可被能力调用入口触发",
                "exposed_to_agent_false": "不经能力调用入口暴露",
            },
        }

