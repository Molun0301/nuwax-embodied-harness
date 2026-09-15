"""机器人控制台页面与地图交互入口。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, TYPE_CHECKING

from fastapi import APIRouter, Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import HTMLResponse, JSONResponse, RedirectResponse
from core.errors import GatewayError, as_gateway_error
from core.robot_geometry import RobotGeometryError, resolve_robot_geometry
from core.config_values import runtime_value
from contracts.navigation_pipeline import navigation_support_distance
from gateways.serialization import to_jsonable

if TYPE_CHECKING:
    from gateways.runtime import GatewayRuntime

_CONSOLE_HTML = Path(__file__).resolve().parent / "static" / "console.html"


def render_console_page(pointcloud3d_config: Any = None, robot_geometry: Any = None, geometry_pipeline: str = "lio_3d", support_distance: Any = None) -> str:
    """注入当前网关配置对应的点云样式与米制机器人外形。"""

    payload = to_jsonable(pointcloud3d_config) if pointcloud3d_config is not None else {}
    encoded = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).replace("</", "<\\/")
    geometry = json.dumps(robot_geometry, ensure_ascii=False, allow_nan=False, separators=(",", ":")).replace("</", "<\\/")
    if geometry_pipeline not in {"lio_3d", "slam_2d"}:
        raise ValueError("未知几何执行链")
    try:
        support = navigation_support_distance(support_distance)
    except ValueError:
        support = None
    return (_CONSOLE_HTML.read_text(encoding="utf-8")
            .replace('"__NAV_SUPPORT_DISTANCE__"', json.dumps(support))
            .replace('"__GEOMETRY_PIPELINE__"', json.dumps(geometry_pipeline))
            .replace('"__POINTCLOUD3D_CONFIG__"', encoded)
            .replace('"__ROBOT_GEOMETRY__"', geometry))


def build_console_router(runtime: GatewayRuntime) -> APIRouter:
    router = APIRouter()

    @router.get("/console", response_class=HTMLResponse)
    @router.get("/console/", response_class=HTMLResponse)
    async def console_page() -> HTMLResponse:
        def render() -> str:
            robot = runtime.config.robot
            try:
                geometry = resolve_robot_geometry(
                    robot.geometry, robot_name=robot.active_robot,
                    base_dir=Path(__file__).resolve().parents[2],
                ).to_runtime_dict()
            except (RobotGeometryError, OSError) as exc:
                # 外形错误不妨碍打开控制台修正设置，也不伪造另一套显示尺寸。
                geometry = {"error": str(exc)}
            return render_console_page(getattr(runtime.config.gateway, "pointcloud3d", None), geometry,
                                       getattr(runtime.config.geometry, "pipeline", "lio_3d"),
                                       runtime_value("NUWAX_NAV_BODY_TO_SUPPORT_Z_M", ""))

        return HTMLResponse(await run_in_threadpool(render))

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
