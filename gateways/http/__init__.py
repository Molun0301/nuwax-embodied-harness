"""HTTP 网关导出。"""

from gateways.http.api import build_http_router
from gateways.http.console import build_console_router

__all__ = ["build_http_router", "build_console_router"]
