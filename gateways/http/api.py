from __future__ import annotations

from typing import Dict, Optional, Any

from fastapi import APIRouter
from fastapi import Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import FileResponse
from fastapi.responses import HTMLResponse
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field

from gateways.auth import GatewayAccessManager
from gateways.errors import as_gateway_error
from gateways.runtime import GatewayRuntime
from gateways.serialization import to_jsonable
from gateways.tcp_legacy.policy import build_tcp_legacy_policy


class CapabilityInvokeRequest(BaseModel):
    """能力调用请求体。"""

    arguments: Dict[str, Any] = Field(default_factory=dict, description="调用参数。")


def _stream_migration_response(*topics: str) -> JSONResponse:
    topic_list = [topic for topic in topics if topic]
    query = ",".join(topic_list)
    stream_path = "/streams/ws" + (f"?topics={query}" if query else "")
    return JSONResponse(
        status_code=410,
        content={
            "code": "stream_required",
            "message": "实时数据 HTTP 快照入口已下线，请订阅实时流。",
            "stream": stream_path,
            "topics": topic_list,
        },
    )


def build_http_router(runtime: GatewayRuntime, access_manager: GatewayAccessManager) -> APIRouter:
    """构造 HTTP 调试与管理接口。"""

    router = APIRouter()

    def _build_layout_snapshot() -> Dict[str, Any]:
        return {
            "base_dir": runtime.config.base_dir,
            "log_dir": runtime.config.logging.log_dir,
            "log_file": runtime.config.logging.log_file,
            "artifact_dir": runtime.config.gateway.artifact_dir,
            "state_store_mode": "in_memory",
            "recommended_runtime_dirs": {
                "logs": runtime.config.logging.log_dir,
                "artifacts": runtime.config.gateway.artifact_dir,
                "state_cache": f"{runtime.config.base_dir}/var/state",
                "tmp": f"{runtime.config.base_dir}/var/tmp",
            },
        }

    def _render_ops_page() -> str:
        health = runtime.get_health_summary()
        security_policy = access_manager.export_policy()
        layout = _build_layout_snapshot()
        return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <title>nuwax_robot_bridge 运维页</title>
  <style>
    body {{ font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif; margin: 24px; background: #f4f6f8; color: #1f2937; }}
    h1, h2 {{ margin: 0 0 12px 0; }}
    .grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 16px; }}
    .card {{ background: white; border-radius: 12px; padding: 16px; box-shadow: 0 6px 18px rgba(15, 23, 42, 0.08); }}
    .mono {{ font-family: "JetBrains Mono", "Consolas", monospace; word-break: break-all; }}
    ul {{ margin: 8px 0 0 18px; padding: 0; }}
    li {{ margin: 4px 0; }}
    .muted {{ color: #6b7280; }}
  </style>
</head>
<body>
  <h1>nuwax_robot_bridge 运维页</h1>
  <p class="muted">用于快速查看宿主机运行时、目录布局和安全策略的最低可用状态页。</p>
  <div class="grid">
    <section class="card">
      <h2>运行时</h2>
      <ul>
        <li>机器人：<span class="mono">{health["robot_name"]}</span></li>
        <li>型号：<span class="mono">{health["robot_model"]}</span></li>
        <li>已启动：<span class="mono">{health["runtime_started"]}</span></li>
        <li>活动任务数：<span class="mono">{health["active_task_count"]}</span></li>
        <li>最新事件游标：<span class="mono">{health["latest_event_cursor"]}</span></li>
      </ul>
    </section>
    <section class="card">
      <h2>目录布局</h2>
      <ul>
        <li>基础目录：<span class="mono">{layout["base_dir"]}</span></li>
        <li>日志目录：<span class="mono">{layout["log_dir"]}</span></li>
        <li>日志文件：<span class="mono">{layout["log_file"]}</span></li>
        <li>制品目录：<span class="mono">{layout["artifact_dir"]}</span></li>
        <li>状态缓存：<span class="mono">{layout["state_store_mode"]}</span></li>
      </ul>
    </section>
    <section class="card">
      <h2>安全策略</h2>
      <ul>
        <li>允许网段：<span class="mono">{", ".join(security_policy["allowed_source_cidrs"]) or "未限制"}</span></li>
        <li>允许未知来源：<span class="mono">{security_policy["allow_unknown_client_hosts"]}</span></li>
        <li>Agent 令牌数：<span class="mono">{security_policy["agent_token_count"]}</span></li>
        <li>Admin 令牌数：<span class="mono">{security_policy["admin_token_count"]}</span></li>
        <li>高风险白名单：<span class="mono">{", ".join(security_policy["agent_high_risk_allowlist"]) or "空"}</span></li>
      </ul>
    </section>
    <section class="card">
      <h2>调试入口</h2>
      <ul>
        <li><span class="mono">/api/health</span></li>
        <li><span class="mono">/api/tools</span></li>
        <li><span class="mono">/api/security/policy</span></li>
        <li><span class="mono">/api/deployment/layout</span></li>
        <li><span class="mono">/api/tasks</span></li>
        <li><span class="mono">/events/stream</span></li>
      </ul>
    </section>
  </div>
</body>
</html>"""

    @router.get("/api/health")
    async def api_health(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return to_jsonable(runtime.get_health_summary())

    @router.get("/api/capabilities")
    async def api_capabilities(
        request: Request,
        exposed_only: Optional[bool] = None,
    ) -> Dict[str, Any]:
        access = access_manager.authenticate_http(request)
        descriptors = runtime.list_capabilities(exposed_only=exposed_only)
        allowed = access_manager.filter_capabilities(access, descriptors)
        return {"capabilities": [descriptor.model_dump(mode="json") for descriptor in allowed]}

    @router.get("/api/tools")
    async def api_tools(
        request: Request,
        include_unsupported: bool = False,
    ) -> Dict[str, Any]:
        access = access_manager.authenticate_http(request)
        exposed_only = None if access.role.value == "admin" else True
        views = runtime.list_tools(
            exposed_only=exposed_only,
            include_unsupported=include_unsupported,
        )
        return {"tools": to_jsonable(views)}

    @router.get("/api/security/policy")
    async def api_security_policy(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return access_manager.export_policy()

    @router.get("/api/deployment/layout")
    async def api_deployment_layout(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return _build_layout_snapshot()

    @router.get("/api/runtime/governance")
    async def api_runtime_governance(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return {"runtime_governance": to_jsonable(runtime.get_runtime_governance_summary())}

    @router.get("/ops", response_class=HTMLResponse)
    async def ops_page(request: Request) -> HTMLResponse:
        access_manager.authenticate_http(request)
        return HTMLResponse(_render_ops_page())

    def _resolve_client_host(request: Request) -> str:
        forwarded_for = request.headers.get("x-forwarded-for", "").strip()
        if forwarded_for:
            return forwarded_for.split(",", 1)[0].strip()
        if request.client is not None and request.client.host:
            return request.client.host
        return "-"

    def _build_capability_request_context(
        request: Request,
        *,
        capability_name: str,
        requested_by: str,
    ) -> Dict[str, Any]:
        state_context = getattr(request.state, "request_context", None)
        context = dict(state_context) if isinstance(state_context, dict) else {}
        request_id = context.get("request_id") or getattr(request.state, "request_id", None) or request.headers.get("x-request-id")
        if request_id:
            context["request_id"] = str(request_id).strip()
        context.update(
            {
                "capability_name": capability_name,
                "requested_by": requested_by or "unknown",
                "method": request.method,
                "path": request.url.path,
                "client_ip": context.get("client_ip") or _resolve_client_host(request),
            }
        )
        return context

    @router.post("/api/capabilities/{capability_name}/invoke")
    async def api_invoke_capability(
        capability_name: str,
        body: CapabilityInvokeRequest,
        request: Request,
    ) -> Dict[str, Any]:
        access = access_manager.authenticate_http(request)
        descriptor = runtime.capability_registry.get_descriptor(capability_name)
        access_manager.authorize_capability(access, descriptor)
        request_context = _build_capability_request_context(
            request,
            capability_name=capability_name,
            requested_by=access.principal,
        )
        try:
            result = await run_in_threadpool(
                lambda: runtime.invoke_capability(
                    capability_name,
                    body.arguments,
                    requested_by=access.principal,
                    request_context=request_context,
                )
            )
        except Exception as exc:
            raise as_gateway_error(exc) from exc
        response = to_jsonable(result)
        request_id = getattr(request.state, "request_id", None)
        if isinstance(response, dict) and request_id:
            response["request_id"] = request_id
        return response

    @router.get("/api/tasks")
    async def api_tasks(
        request: Request,
        history_limit: int = 20,
    ) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return {
            "active_tasks": to_jsonable(runtime.task_manager.list_active_tasks()),
            "history_tasks": to_jsonable(runtime.task_manager.list_history(limit=history_limit)),
        }

    @router.get("/api/tasks/{task_id}")
    async def api_task_status(task_id: str, request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        try:
            return to_jsonable(runtime.get_task_snapshot(task_id))
        except Exception as exc:
            raise as_gateway_error(exc) from exc

    @router.post("/api/tasks/{task_id}/cancel")
    async def api_cancel_task(task_id: str, request: Request) -> Dict[str, Any]:
        access = access_manager.authenticate_http(request)
        try:
            return to_jsonable(
                runtime._handle_cancel_task({"task_id": task_id}, requested_by=access.principal)
            )
        except Exception as exc:
            raise as_gateway_error(exc) from exc

    @router.get("/api/legacy/tcp_policy")
    async def api_tcp_policy(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return to_jsonable(build_tcp_legacy_policy())

    @router.get("/api/state/latest")
    async def api_state_latest(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return _stream_migration_response('robot.state')

    @router.get("/api/state/history")
    async def api_state_history(request: Request, limit: int = 20) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return _stream_migration_response('robot.state')

    @router.get("/api/state/diagnostics")
    async def api_state_diagnostics(request: Request, limit: int = 20) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return _stream_migration_response('robot.state')

    @router.get("/api/observations/latest")
    async def api_observations_latest(request: Request, camera_id: Optional[str] = None) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return _stream_migration_response('video.frame', 'pointcloud.frame')

    @router.get("/api/observations/history")
    async def api_observations_history(
        request: Request,
        camera_id: Optional[str] = None,
        limit: int = 20,
    ) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return _stream_migration_response('video.frame', 'pointcloud.frame')

    @router.get("/api/artifacts/summary")
    async def api_artifacts_summary(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return to_jsonable(runtime.artifact_service.get_summary())

    @router.get("/api/perception/latest")
    async def api_perception_latest(request: Request, camera_id: Optional[str] = None) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return _stream_migration_response('video.frame')

    @router.get("/api/perception/history")
    async def api_perception_history(
        request: Request,
        camera_id: Optional[str] = None,
        limit: int = 20,
    ) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return _stream_migration_response('video.frame')

    @router.get("/api/perception/runtime")
    async def api_perception_runtime(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return {"perception_runtime": to_jsonable(runtime.perception_video_runtime.get_status())}

    @router.post("/api/perception/runtime/start")
    async def api_perception_runtime_start(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return {"perception_runtime": to_jsonable(runtime.perception_video_runtime.start())}

    @router.post("/api/perception/runtime/stop")
    async def api_perception_runtime_stop(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return {"perception_runtime": to_jsonable(runtime.perception_video_runtime.stop())}

    @router.get("/api/localization/latest")
    async def api_localization_latest(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return _stream_migration_response('localization.pose', 'localization.frame_tree', 'localization.trajectory')

    @router.get("/api/localization/session")
    async def api_localization_session(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        try:
            return to_jsonable(
                runtime._handle_get_localization_session(
                    {},
                    requested_by="http_api",
                )
            )
        except Exception as exc:
            raise as_gateway_error(exc) from exc

    @router.post("/api/localization/relocalize")
    async def api_localization_relocalize(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        payload = await request.json()
        if not isinstance(payload, dict):
            payload = {}
        try:
            return to_jsonable(
                runtime._handle_relocalize_active_map(
                    payload,
                    requested_by="http_api",
                )
            )
        except Exception as exc:
            raise as_gateway_error(exc) from exc

    @router.get("/api/localization/history")
    async def api_localization_history(request: Request, limit: int = 20) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return _stream_migration_response('localization.pose', 'localization.trajectory')

    @router.get("/api/maps/latest")
    async def api_maps_latest(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return _stream_migration_response('map.frame')

    @router.get("/api/maps/history")
    async def api_maps_history(request: Request, limit: int = 20) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return _stream_migration_response('map.frame')

    @router.get("/api/maps/catalog")
    async def api_maps_catalog(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return {
            "maps": to_jsonable(runtime.map_workspace_service.list_maps()),
            "active_workspace": to_jsonable(runtime.map_workspace_service.get_active_workspace()),
        }

    @router.post("/api/maps/catalog")
    async def api_maps_catalog_create(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        payload = await request.json()
        if not isinstance(payload, dict):
            payload = {}
        map_name = str(payload.get("map_name", "")).strip()
        return {
            "map_asset": to_jsonable(
                runtime.map_workspace_service.create_map(
                    map_name=map_name,
                    metadata={"requested_by": "http_api"},
                )
            ),
            "maps": to_jsonable(runtime.map_workspace_service.list_maps()),
            "active_workspace": to_jsonable(runtime.map_workspace_service.get_active_workspace()),
        }

    @router.delete("/api/maps/catalog/{map_name}")
    async def api_maps_catalog_delete(
        map_name: str,
        request: Request,
        delete_memory_library: bool = False,
    ) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        delete_result = runtime.map_workspace_service.delete_map(
            map_name=map_name,
            delete_memory_library=delete_memory_library,
        )
        if bool(delete_result.get("was_active")):
            runtime._sync_perception_runtime_for_memory_state(enabled=False)
        return {
            "delete_result": to_jsonable(delete_result),
            "maps": to_jsonable(runtime.map_workspace_service.list_maps()),
            "active_workspace": to_jsonable(runtime.map_workspace_service.get_active_workspace()),
        }

    @router.post("/api/maps/activate")
    async def api_maps_activate(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        payload = await request.json()
        if not isinstance(payload, dict):
            payload = {}
        map_name = str(payload.get("map_name", "")).strip()
        active_workspace = runtime.map_workspace_service.activate_map(
            map_name=map_name,
            create_if_missing=False,
            load_memory_history=bool(payload.get("load_memory_history", True)),
            reset_memory_library=bool(payload.get("reset_memory_library", False)),
            metadata={"requested_by": "http_api"},
        )
        runtime._sync_perception_runtime_for_memory_state(enabled=True)
        return {
            "active_workspace": to_jsonable(runtime.map_workspace_service.get_active_workspace() or active_workspace),
            "maps": to_jsonable(runtime.map_workspace_service.list_maps()),
            "memory_summary": to_jsonable(runtime.memory_service.get_summary()),
        }

    @router.get("/api/maps/active")
    async def api_maps_active(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return {
            "active_workspace": to_jsonable(runtime.map_workspace_service.get_active_workspace()),
        }

    @router.get("/api/maps/catalog/{map_name}/versions")
    async def api_map_versions(
        map_name: str,
        request: Request,
        limit: int = 20,
    ) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return {
            "map_versions": to_jsonable(
                runtime.map_workspace_service.list_map_versions(
                    map_name=map_name,
                    limit=limit,
                )
            )
        }

    @router.post("/api/maps/versions/save")
    async def api_map_versions_save(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        payload = await request.json()
        if not isinstance(payload, dict):
            payload = {}
        map_version = runtime.map_workspace_service.save_map_version(
            map_name=str(payload.get("map_name", "")).strip() or None,
            reason=str(payload.get("reason", "")).strip(),
            metadata={"requested_by": "http_api"},
        )
        return {
            "map_version": to_jsonable(map_version),
            "active_workspace": to_jsonable(runtime.map_workspace_service.get_active_workspace()),
        }

    @router.post("/api/maps/versions/load")
    async def api_map_versions_load(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        payload = await request.json()
        if not isinstance(payload, dict):
            payload = {}
        map_name = str(payload.get("map_name", "")).strip()
        map_version, workspace = runtime.map_workspace_service.load_map_version(
            map_name=map_name,
            version_id=str(payload.get("version_id", "")).strip() or None,
            activate_if_needed=True,
            load_memory_history=bool(payload.get("load_memory_history", True)),
            reset_memory_library=bool(payload.get("reset_memory_library", False)),
            metadata={"requested_by": "http_api", "source": "http_api_load_map_version"},
        )
        return {
            "map_version": to_jsonable(map_version),
            "active_workspace": to_jsonable(workspace),
        }

    @router.get("/api/navigation/latest")
    async def api_navigation_latest(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return _stream_migration_response('navigation.state', 'exploration.state', 'exploration.frontier')

    @router.get("/api/navigation/semantic_context")
    async def api_navigation_semantic_context(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return {
            "navigation_semantic_context": to_jsonable(
                runtime.memory_service.get_navigation_semantic_context(
                    map_name=(
                        runtime.map_workspace_service.get_active_map_name()
                        or runtime.memory_service.get_active_library_name()
                    )
                )
            )
        }

    @router.get("/api/navigation/history")
    async def api_navigation_history(request: Request, limit: int = 20) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return _stream_migration_response('navigation.state', 'exploration.state', 'exploration.frontier')

    @router.get("/api/memory/summary")
    async def api_memory_summary(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return {"memory_summary": to_jsonable(runtime.memory_service.get_summary())}

    @router.get("/api/memory/libraries")
    async def api_memory_libraries(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return {
            "libraries": to_jsonable(runtime.memory_service.list_memory_libraries()),
            "memory_summary": to_jsonable(runtime.memory_service.get_summary()),
        }

    @router.post("/api/memory/libraries")
    async def api_create_memory_library(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        payload = await request.json()
        if not isinstance(payload, dict):
            payload = {}
        library_name = str(payload.get("library_name", "")).strip()
        return {
            "create_result": to_jsonable(runtime.memory_service.create_memory_library(library_name=library_name)),
            "libraries": to_jsonable(runtime.memory_service.list_memory_libraries()),
            "memory_summary": to_jsonable(runtime.memory_service.get_summary()),
        }

    @router.delete("/api/memory/libraries/{library_name}")
    async def api_delete_memory_library(library_name: str, request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        active_workspace = runtime.map_workspace_service.get_active_workspace()
        delete_result = runtime.memory_service.delete_memory_library(library_name=library_name)
        if active_workspace is not None and active_workspace.active_map_name == library_name:
            runtime.map_workspace_service.clear_active_map(disable_memory_library=False)
        if bool(delete_result.get("was_active")):
            runtime._sync_perception_runtime_for_memory_state(enabled=False)
        return {
            "delete_result": to_jsonable(delete_result),
            "maps": to_jsonable(runtime.map_workspace_service.list_maps()),
            "active_workspace": to_jsonable(runtime.map_workspace_service.get_active_workspace()),
            "libraries": to_jsonable(runtime.memory_service.list_memory_libraries()),
            "memory_summary": to_jsonable(runtime.memory_service.get_summary()),
        }

    @router.get("/api/memory/locations")
    async def api_memory_locations(request: Request, limit: int = 20) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return {
            "locations": to_jsonable(runtime.memory_service.list_tagged_locations(limit=limit)),
            "memory_summary": to_jsonable(runtime.memory_service.get_summary()),
        }

    @router.get("/api/memory/semantic")
    async def api_memory_semantic(request: Request, limit: int = 20) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return {
            "entries": to_jsonable(runtime.memory_service.list_semantic_memories(limit=limit)),
            "memory_summary": to_jsonable(runtime.memory_service.get_summary()),
        }

    @router.get("/api/memory/scene_spaces")
    async def api_memory_scene_spaces(
        request: Request,
        limit: int = 20,
        scene_space_id: Optional[str] = None,
        map_version_id: Optional[str] = None,
        localization_session_id: Optional[str] = None,
        review_status: Optional[str] = None,
    ) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        arguments: Dict[str, Any] = {"limit": limit}
        for key, value in {
            "scene_space_id": scene_space_id,
            "map_version_id": map_version_id,
            "localization_session_id": localization_session_id,
            "review_status": review_status,
        }.items():
            if value is not None:
                arguments[key] = value
        return to_jsonable(runtime._handle_list_scene_spaces(arguments, requested_by="http_api"))

    @router.get("/api/memory/scene_spaces/query")
    async def api_memory_scene_spaces_query(
        request: Request,
        query: str,
        limit: int = 5,
        similarity_threshold: float = 0.25,
        map_version_id: Optional[str] = None,
        localization_session_id: Optional[str] = None,
        review_status: Optional[str] = None,
        max_distance_m: Optional[float] = None,
    ) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        arguments: Dict[str, Any] = {
            "query": query,
            "limit": limit,
            "similarity_threshold": similarity_threshold,
        }
        for key, value in {
            "map_version_id": map_version_id,
            "localization_session_id": localization_session_id,
            "review_status": review_status,
            "max_distance_m": max_distance_m,
        }.items():
            if value is not None:
                arguments[key] = value
        return to_jsonable(runtime._handle_query_scene_spaces(arguments, requested_by="http_api"))

    @router.get("/api/memory/scene_spaces/{scene_space_id}")
    async def api_memory_scene_space(scene_space_id: str, request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        return to_jsonable(
            runtime._handle_get_scene_space(
                {"scene_space_id": scene_space_id},
                requested_by="http_api",
            )
        )

    @router.get("/artifacts/{artifact_id}/meta")
    async def api_artifact_meta(artifact_id: str, request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        try:
            return runtime.artifact_store.get_ref(artifact_id).model_dump(mode="json")
        except Exception as exc:
            raise as_gateway_error(exc) from exc

    @router.get("/artifacts/{artifact_id}")
    async def api_artifact_raw(artifact_id: str, request: Request) -> FileResponse:
        access_manager.authenticate_http(request)
        try:
            ref = runtime.artifact_store.get_ref(artifact_id)
            path = runtime.artifact_store.get_file_path(artifact_id)
        except Exception as exc:
            raise as_gateway_error(exc) from exc
        return FileResponse(path, media_type=ref.mime_type, filename=path.name)

    return router
