"""机器人控制台页面与地图交互入口。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, TYPE_CHECKING

from fastapi import APIRouter
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import HTMLResponse, RedirectResponse
from gateways.serialization import to_jsonable
from gateways.http.robot_model import build_robot_model_router

if TYPE_CHECKING:
    from gateways.platform import Platform

_CONSOLE_HTML = Path(__file__).resolve().parent / "static" / "console.html"
# 重构中的新控制台，P6 切换后取代 console.html。
_CONSOLE_NEXT_HTML = Path(__file__).resolve().parent / "static" / "next" / "index.html"


def render_console_page(pointcloud3d_config: Any = None, geometry_pipeline: str = "lio_3d",
                        page: Path = _CONSOLE_HTML) -> str:
    """注入当前网关配置对应的点云样式与执行链。"""

    payload = to_jsonable(pointcloud3d_config) if pointcloud3d_config is not None else {}
    encoded = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).replace("</", "<\\/")
    if geometry_pipeline not in {"lio_3d", "slam_2d"}:
        raise ValueError("未知几何执行链")
    return (page.read_text(encoding="utf-8")
            .replace('"__GEOMETRY_PIPELINE__"', json.dumps(geometry_pipeline))
            .replace('"__POINTCLOUD3D_CONFIG__"', encoded))


def build_console_router(runtime: Platform) -> APIRouter:
    router = APIRouter()
    router.include_router(build_robot_model_router(runtime))

    @router.get("/console", response_class=HTMLResponse)
    @router.get("/console/", response_class=HTMLResponse)
    async def console_page() -> HTMLResponse:
        def render() -> str:
            # 外形由模型接口动态读取，避免页面初始注入在配置变更后滞留。
            return render_console_page(getattr(runtime.config.gateway, "pointcloud3d", None),
                                       getattr(runtime.config.geometry, "pipeline", "lio_3d"))

        return HTMLResponse(await run_in_threadpool(render))

    @router.get("/console-next", response_class=HTMLResponse)
    @router.get("/console-next/", response_class=HTMLResponse)
    async def console_next_page() -> HTMLResponse:
        def render() -> str:
            return render_console_page(getattr(runtime.config.gateway, "pointcloud3d", None),
                                       getattr(runtime.config.geometry, "pipeline", "lio_3d"),
                                       _CONSOLE_NEXT_HTML)

        return HTMLResponse(await run_in_threadpool(render))

    @router.get("/debug", include_in_schema=False)
    @router.get("/debug/", include_in_schema=False)
    async def previous_console_address() -> RedirectResponse:
        return RedirectResponse("/console", status_code=308)

    return router
