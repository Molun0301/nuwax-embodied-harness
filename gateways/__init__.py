"""网关层惰性导出。"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from gateways.app import create_gateway_app, create_gateway_app_from_env, create_relay_app_from_env
    from services.artifact_store import LocalArtifactStore
    from gateways.auth import GatewayAccessManager
    from gateways.runtime import GatewayRuntime, create_default_gateway_runtime

__all__ = [
    "GatewayAccessManager",
    "GatewayRuntime",
    "LocalArtifactStore",
    "create_default_gateway_runtime",
    "create_gateway_app",
    "create_gateway_app_from_env",
    "create_relay_app_from_env",
]


def __getattr__(name: str):
    """按需导出网关对象，避免包初始化循环依赖。"""

    if name == "GatewayAccessManager":
        from gateways.auth import GatewayAccessManager

        return GatewayAccessManager
    if name == "LocalArtifactStore":
        from services.artifact_store import LocalArtifactStore

        return LocalArtifactStore
    if name in {"GatewayRuntime", "create_default_gateway_runtime"}:
        from gateways.runtime import GatewayRuntime, create_default_gateway_runtime

        return {
            "GatewayRuntime": GatewayRuntime,
            "create_default_gateway_runtime": create_default_gateway_runtime,
        }[name]
    if name in {"create_gateway_app", "create_gateway_app_from_env", "create_relay_app_from_env"}:
        from gateways.app import create_gateway_app, create_gateway_app_from_env, create_relay_app_from_env

        return {
            "create_gateway_app": create_gateway_app,
            "create_gateway_app_from_env": create_gateway_app_from_env,
            "create_relay_app_from_env": create_relay_app_from_env,
        }[name]
    raise AttributeError(name)
