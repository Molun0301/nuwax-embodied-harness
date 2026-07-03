"""兼容 shim：旧名 ``NavigationProxyClient`` → 新名 ``RuntimeNavigationClient``。

Phase 3 完成后，调用方应统一改为 ``from services.geometry.runtime_navigation_client import RuntimeNavigationClient``。
本文件保留作为过渡期间别名，目标态 import 路径切换在后续 lane 汇合时完成。
"""
from __future__ import annotations

from services.geometry.runtime_navigation_client import (
    NavigationError,
    NavigationFailureCode,
    NavigationStateUpdate,
    RuntimeNavigationClient,
    RuntimeNavigationRequest,
    get_default_client,
)

# 旧名别名（保留 NavigationProxyClient 给旧调用方过渡用）
NavigationProxyClient = RuntimeNavigationClient

__all__ = [
    "NavigationError",
    "NavigationFailureCode",
    "NavigationStateUpdate",
    "NavigationProxyClient",
    "RuntimeNavigationClient",
    "RuntimeNavigationRequest",
    "get_default_client",
]
