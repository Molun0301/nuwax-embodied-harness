from __future__ import annotations

from typing import Dict, Optional, Any


class GatewayError(RuntimeError):
    """平台统一业务异常。

    历史上位于 gateways 包，实为平台级业务异常（携带 HTTP 状态
    元数据），其真实消费方主体在 services 层。下沉至 core 以解开 services ⇄ gateways
    包级循环；HTTP 状态仅为协议映射元数据，不构成对 web 框架的依赖。
    """

    def __init__(
        self,
        message: str,
        *,
        error_code: str = "gateway_error",
        http_status: int = 400,
        details: Optional[Dict[str, Any]] = None,
    ) -> None:
        super().__init__(message)
        self.message = message
        self.error_code = error_code
        self.http_status = http_status
        self.details = dict(details or {})

    def to_payload(self) -> Dict[str, Any]:
        """导出标准错误载荷。"""

        return {
            "code": self.error_code,
            "message": self.message,
            "details": self.details,
        }


class MotionGuardFacadeError(RuntimeError):
    """Neutral boundary error raised by an injected motion-guard facade."""


class AuthenticationRequiredError(GatewayError):
    """缺少认证信息。"""

    def __init__(self, message: str = "请求缺少有效认证信息。") -> None:
        super().__init__(
            message,
            error_code="authentication_required",
            http_status=401,
        )


class AccessDeniedError(GatewayError):
    """访问被拒绝。"""

    def __init__(self, message: str, *, details: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(
            message,
            error_code="access_denied",
            http_status=403,
            details=details,
        )


class ArtifactNotFoundError(GatewayError):
    """制品不存在。"""

    def __init__(self, artifact_id: str) -> None:
        super().__init__(
            f"制品 {artifact_id} 不存在。",
            error_code="artifact_not_found",
            http_status=404,
            details={"artifact_id": artifact_id},
        )


def as_gateway_error(exc: Exception) -> GatewayError:
    """把未知异常转换为网关异常。"""

    if isinstance(exc, GatewayError):
        return exc
    return GatewayError(
        "工具执行失败，未完成请求。",
        error_code="internal_error",
        http_status=500,
        details={"exception_type": exc.__class__.__name__},
    )


__all__ = [
    "GatewayError",
    "AuthenticationRequiredError",
    "AccessDeniedError",
    "ArtifactNotFoundError",
    "as_gateway_error",
]
