"""网关异常过渡再导出。

GatewayError 及其子类已下沉至 `core.errors`（解开 services ⇄ gateways 包级循环）。
本模块保留为再导出 shim，供尚未迁移的网关内部引用过渡使用。新代码请直接从
`core.errors` 导入。
"""

from __future__ import annotations

from core.errors import (
    AccessDeniedError,
    ArtifactNotFoundError,
    AuthenticationRequiredError,
    GatewayError,
    as_gateway_error,
)

__all__ = [
    "GatewayError",
    "AuthenticationRequiredError",
    "AccessDeniedError",
    "ArtifactNotFoundError",
    "as_gateway_error",
]
