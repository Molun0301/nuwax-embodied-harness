"""网关公开入口；装配由 platform 模块拥有。"""
from __future__ import annotations

__all__ = ["create_gateway_app", "create_gateway_app_from_env", "create_relay_app_from_env"]


def __getattr__(name: str):
    if name in __all__:
        from gateways.app import create_gateway_app, create_gateway_app_from_env, create_relay_app_from_env
        return {"create_gateway_app": create_gateway_app,
                "create_gateway_app_from_env": create_gateway_app_from_env,
                "create_relay_app_from_env": create_relay_app_from_env}[name]
    raise AttributeError(name)
