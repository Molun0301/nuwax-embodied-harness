from __future__ import annotations

from dataclasses import dataclass
from contracts.compat import StrEnum
import ipaddress
import logging

from contracts.capabilities import CapabilityDescriptor, CapabilityRiskLevel
from gateways.errors import AccessDeniedError, AuthenticationRequiredError, GatewayError
from core.settings import GatewayServerConfig
from starlette.datastructures import Headers, QueryParams
from starlette.requests import Request
from starlette.websockets import WebSocket
from typing import Dict, Optional, Tuple


class AccessRole(StrEnum):
    """网关访问角色。"""

    AGENT = "agent"
    ADMIN = "admin"


AUTH_LOGGER = logging.getLogger("nuwax.gateway.auth")


_LOOPBACK_HOST_LITERALS = frozenset({"localhost"})


@dataclass(frozen=True)
class AccessContext:
    """已认证访问上下文。"""

    role: AccessRole
    token: str
    principal: str
    source_host: Optional[str]


class GatewayAccessManager:
    """宿主机网关访问控制器。"""

    def __init__(self, config: GatewayServerConfig) -> None:
        self._config = config
        self._allowed_networks = tuple(ipaddress.ip_network(item) for item in config.allowed_source_cidrs)
        self._agent_tokens = frozenset(token for token in config.agent_tokens if token)
        self._admin_tokens = frozenset(token for token in config.admin_tokens if token)
        self._agent_high_risk_allowlist = frozenset(config.agent_high_risk_allowlist)
        self._assert_credentials_configured()

    def _assert_credentials_configured(self) -> None:
        """无任何有效令牌时执行 fail-closed 校验。

        仅监听回环地址时降级为告警放行（便于本机开发）；对外暴露时直接拒绝启动，
        避免在缺失 .env 的部署场景下以弱默认凭据对外提供服务。
        """

        if self._agent_tokens or self._admin_tokens:
            return
        host = getattr(self._config, "host", "") or ""
        if self._is_loopback_host(host):
            AUTH_LOGGER.warning(
                "网关未配置任何 agent/admin 令牌，当前仅监听回环地址(%s)，以无鉴权降级模式启动；"
                "对外部署前必须配置 NUWAX_GATEWAY_AGENT_TOKENS / NUWAX_GATEWAY_ADMIN_TOKENS。",
                host or "<loopback>",
            )
            return
        raise GatewayError(
            "网关未配置任何 agent/admin 令牌且监听非回环地址(%s)，拒绝以弱默认凭据对外启动。"
            "请设置 NUWAX_GATEWAY_AGENT_TOKENS 与 NUWAX_GATEWAY_ADMIN_TOKENS。" % (host or "0.0.0.0"),
            error_code="gateway_misconfigured",
            http_status=500,
            details={"host": host},
        )

    @staticmethod
    def _is_loopback_host(host: str) -> bool:
        normalized = host.strip().lower()
        if normalized in _LOOPBACK_HOST_LITERALS:
            return True
        try:
            return ipaddress.ip_address(normalized).is_loopback
        except ValueError:
            return False

    def authenticate_http(self, request: Request) -> AccessContext:
        """校验 HTTP 请求。"""

        source_host = request.client.host if request.client is not None else None
        self._assert_source_allowed(source_host)
        token = self._extract_token(request.headers, request.query_params)
        return self._resolve_access_context(token, source_host)

    def authenticate_local_debug_page(self, request: Request) -> AccessContext:
        """校验本机调试页，并允许回环来源自动使用 agent 令牌。"""

        source_host = request.client.host if request.client is not None else None
        self._assert_source_allowed(source_host)
        try:
            token = self._extract_token(request.headers, request.query_params)
        except AuthenticationRequiredError:
            if not self._is_loopback_source(source_host):
                raise
            token = self.default_agent_token()
            if not token:
                raise
        return self._resolve_access_context(token, source_host)

    def authenticate_websocket(self, websocket: WebSocket) -> AccessContext:
        """校验 WebSocket 请求。"""

        source_host = websocket.client.host if websocket.client is not None else None
        self._assert_source_allowed(source_host)
        token = self._extract_token(websocket.headers, websocket.query_params)
        return self._resolve_access_context(token, source_host)

    def filter_capabilities(
        self,
        access: AccessContext,
        descriptors: Tuple[CapabilityDescriptor, ...],
    ) -> Tuple[CapabilityDescriptor, ...]:
        """过滤当前调用方可见的能力。"""

        return tuple(descriptor for descriptor in descriptors if self.can_access_capability(access, descriptor))

    def can_access_capability(self, access: AccessContext, descriptor: CapabilityDescriptor) -> bool:
        """判断访问上下文是否有权限调用某能力。"""

        if access.role == AccessRole.ADMIN:
            return True

        if not descriptor.exposed_to_agent:
            return False

        if descriptor.risk_level in {CapabilityRiskLevel.LOW, CapabilityRiskLevel.MEDIUM}:
            return True

        if descriptor.risk_level == CapabilityRiskLevel.HIGH:
            return descriptor.name in self._agent_high_risk_allowlist

        return False

    def authorize_capability(self, access: AccessContext, descriptor: CapabilityDescriptor) -> None:
        """执行能力级鉴权。"""

        if self.can_access_capability(access, descriptor):
            return
        raise AccessDeniedError(
            f"当前令牌无权调用能力 {descriptor.name}。",
            details={
                "role": access.role.value,
                "capability_name": descriptor.name,
                "risk_level": descriptor.risk_level.value,
            },
        )

    def export_policy(self) -> Dict[str, object]:
        """导出不含敏感令牌值的安全策略摘要。"""

        return {
            "roles": [role.value for role in AccessRole],
            "allowed_source_cidrs": list(self._config.allowed_source_cidrs),
            "allow_unknown_client_hosts": self._config.allow_unknown_client_hosts,
            "agent_high_risk_allowlist": sorted(self._agent_high_risk_allowlist),
            "agent_token_count": len(self._agent_tokens),
            "admin_token_count": len(self._admin_tokens),
            "token_transport": {
                "http_header": "Authorization: Bearer <token>",
                "query_param": "access_token",
            },
            "risk_policy": {
                "low": "agent/admin 可调用",
                "medium": "agent/admin 可调用",
                "high": "仅 agent allowlist 或 admin 可调用",
                "admin": "仅 admin 可调用",
            },
        }

    def _extract_token(self, headers: Headers, query_params: QueryParams) -> str:
        auth_header = headers.get("authorization", "").strip()
        if auth_header.lower().startswith("bearer "):
            token = auth_header[7:].strip()
            if token:
                return token
        query_token = query_params.get("access_token", "").strip()
        if query_token:
            return query_token
        raise AuthenticationRequiredError()

    def default_agent_token(self) -> str:
        return next((item for item in self._config.agent_tokens if item), "")

    def _resolve_access_context(self, token: str, source_host: Optional[str]) -> AccessContext:
        if token in self._admin_tokens:
            return AccessContext(role=AccessRole.ADMIN, token=token, principal="admin", source_host=source_host)
        if token in self._agent_tokens:
            return AccessContext(role=AccessRole.AGENT, token=token, principal="agent", source_host=source_host)
        raise AuthenticationRequiredError("认证令牌无效。")

    def _is_loopback_source(self, source_host: Optional[str]) -> bool:
        if not source_host:
            return self._config.allow_unknown_client_hosts
        try:
            return ipaddress.ip_address(source_host).is_loopback
        except ValueError:
            return False

    def _assert_source_allowed(self, source_host: Optional[str]) -> None:
        if not self._allowed_networks:
            return
        if not source_host:
            if self._config.allow_unknown_client_hosts:
                return
            raise AccessDeniedError("无法识别请求来源地址。")
        try:
            source_ip = ipaddress.ip_address(source_host)
        except ValueError:
            if self._config.allow_unknown_client_hosts:
                return
            raise AccessDeniedError(f"请求来源地址不合法: {source_host}")

        for network in self._allowed_networks:
            if source_ip in network:
                return
        raise AccessDeniedError(
            f"请求来源 {source_host} 不在允许网段内。",
            details={"source_host": source_host, "allowed_source_cidrs": list(self._config.allowed_source_cidrs)},
        )
