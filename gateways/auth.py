from __future__ import annotations

from typing import Dict, Tuple

from contracts.capabilities import CapabilityDescriptor
from core.settings import GatewayServerConfig
from gateways.errors import AccessDeniedError


class GatewayAccessManager:
    """宿主机网关能力暴露策略。

    宿主机网关不建立用户、令牌或来源网段身份层。能力调用入口只依据能力描述中的
    ``exposed_to_agent`` 标记决定是否向智能体开放；请求上下文中的固定 principal
    仅用于审计和 ``requested_by`` 标注。
    """

    def __init__(self, config: GatewayServerConfig) -> None:
        # 保留配置参数以维持网关装配接口；请求访问不读取任何鉴权配置。
        self._config = config

    def filter_capabilities(
        self,
        descriptors: Tuple[CapabilityDescriptor, ...],
    ) -> Tuple[CapabilityDescriptor, ...]:
        """只返回明确向智能体暴露的能力。"""

        return tuple(descriptor for descriptor in descriptors if descriptor.exposed_to_agent)

    def can_access_capability(self, descriptor: CapabilityDescriptor) -> bool:
        """判断能力是否明确向智能体暴露。"""

        return bool(descriptor.exposed_to_agent)

    def authorize_capability(self, descriptor: CapabilityDescriptor) -> None:
        """执行唯一的能力暴露标记检查。"""

        if self.can_access_capability(descriptor):
            return
        raise AccessDeniedError(
            f"能力 {descriptor.name} 未向智能体暴露。",
            details={
                "capability_name": descriptor.name,
                "risk_level": descriptor.risk_level.value,
            },
        )

    def export_policy(self) -> Dict[str, object]:
        """导出当前能力暴露策略摘要。"""

        return {
            "auth": "disabled",
            "capability_policy": "仅检查 exposed_to_agent",
            "risk_policy": {
                "exposed_to_agent_true": "允许通过统一能力调用入口触发",
                "exposed_to_agent_false": "不经统一能力调用入口暴露",
            },
        }
