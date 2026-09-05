"""机器人控制台页面与地图交互入口。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, TYPE_CHECKING

from fastapi import APIRouter, Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import HTMLResponse, JSONResponse, RedirectResponse
from core.errors import GatewayError, as_gateway_error
from gateways.serialization import to_jsonable

if TYPE_CHECKING:
    from gateways.runtime import GatewayRuntime

_CONSOLE_HTML = Path(__file__).resolve().parent / "static" / "console.html"


def render_console_page(pointcloud3d_config: Any = None) -> str:
    """页面主体与脚本分开维护，只注入运行时的点云显示设置。"""

    payload = to_jsonable(pointcloud3d_config) if pointcloud3d_config is not None else {}
    encoded = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).replace("</", "<\\/")
    return _CONSOLE_HTML.read_text(encoding="utf-8").replace('"__POINTCLOUD3D_CONFIG__"', encoded)


def build_console_router(runtime: GatewayRuntime) -> APIRouter:
    router = APIRouter()

    @router.get("/console", response_class=HTMLResponse)
    @router.get("/console/", response_class=HTMLResponse)
    async def console_page() -> HTMLResponse:
        return HTMLResponse(render_console_page(getattr(runtime.config.gateway, "pointcloud3d", None)))

    @router.get("/debug", include_in_schema=False)
    @router.get("/debug/", include_in_schema=False)
    async def previous_console_address() -> RedirectResponse:
        return RedirectResponse("/console", status_code=308)

    async def dispatch(request: Request, handler) -> JSONResponse:
        try:
            payload = await request.json()
        except ValueError as exc:
            raise GatewayError("控制参数必须是有效的 JSON 对象。", error_code="invalid_params", http_status=422) from exc
        if not isinstance(payload, dict):
            raise GatewayError("控制参数必须是对象。", error_code="invalid_params", http_status=422)
        try:
            result = await run_in_threadpool(handler, payload, requested_by="console")
        except Exception as exc:
            raise as_gateway_error(exc) from exc
        return JSONResponse(to_jsonable(result))

    @router.post("/api/console/navigation/goal")
    async def navigation_goal(request: Request) -> JSONResponse:
        return await dispatch(request, runtime.dispatch_console_navigation_goal)

    @router.post("/api/console/map/clicked_point")
    async def clicked_point(request: Request) -> JSONResponse:
        return await dispatch(request, runtime.dispatch_console_clicked_point)

    @router.post("/api/console/map/initial_pose")
    async def initial_pose(request: Request) -> JSONResponse:
        return await dispatch(request, runtime.dispatch_console_initial_pose)

    return router
