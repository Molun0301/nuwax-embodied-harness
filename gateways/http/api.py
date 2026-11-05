from __future__ import annotations

import asyncio
import json
import weakref
from contextlib import asynccontextmanager
from typing import Dict, Optional, Any

from fastapi import APIRouter, Query, BackgroundTasks
from fastapi import Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import FileResponse, Response
from fastapi.responses import HTMLResponse
from fastapi.responses import StreamingResponse

from gateways.arguments import strict_json_constant
from gateways.auth import GatewayAccessManager
from core.errors import GatewayError, as_gateway_error
from gateways.invoke import invoke
from gateways.health import summary as health_summary
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from gateways.platform import Platform
from gateways.serialization import to_jsonable, to_map_json_bytes
from gateways.ws.client_queue import ClientQueue
from core import config_io


class ArtifactFileResponse(FileResponse):
    """响应拥有读取保护；直到真正 ASGI 发送完成/取消才释放。

    保留 FileResponse 的 range/条件请求行为。响应在执行前被丢弃时由
    finalizer 释放备用句柄；正常路径均使用显式 finally，不依赖垃圾回收。
    """

    def __init__(self, store, artifact_id: str) -> None:
        hold = store.acquire_artifacts((artifact_id,), owner="artifact.http_response")
        try:
            ref = store.get_ref(artifact_id)
            path = store.get_file_path(artifact_id)
            super().__init__(path, media_type=ref.mime_type, filename=path.name)
        except BaseException:
            hold.release()
            raise
        self._release_artifact = weakref.finalize(self, hold.release)

    async def __call__(self, scope, receive, send):
        try:
            # pathsend 把路径交给服务器后就返回，服务器可能尚未真正打开文件。
            # 本响应使用标准分块/range 发送，确保保护覆盖实际文件消费。
            extensions = dict(scope.get("extensions", {}))
            extensions.pop("http.response.pathsend", None)
            await super().__call__({**scope, "extensions": extensions}, receive, send)
        finally:
            self._release_artifact()


def _settings_lifespan(original_lifespan, settings_application):
    """合成原路由生命周期；清理设置快照不掩盖更早的启停异常。"""
    @asynccontextmanager
    async def lifespan(app):
        settings_application.start()
        try:
            async with original_lifespan(app) as state:
                yield state
        except BaseException:
            try:
                settings_application.close()
            except BaseException:
                pass
            raise
        else:
            settings_application.close()
    return lifespan


def build_http_router(runtime: Platform, access_manager: GatewayAccessManager) -> APIRouter:
    """构造 HTTP（超文本传输协议）业务与管理接口。"""

    router = APIRouter()
    from gateways.settings import SettingsApplication
    from core.config_profiles import ConfigurationConflict
    settings_application = SettingsApplication(runtime)
    router.lifespan_context = _settings_lifespan(router.lifespan_context, settings_application)

    async def call_settings(function, *args, **kwargs):
        try:
            return await run_in_threadpool(function, *args, **kwargs)
        except ConfigurationConflict as error:
            raise GatewayError(str(error), error_code="configuration_conflict", http_status=409) from error
        except (ValueError, TypeError) as error:
            raise GatewayError(str(error), error_code="invalid_configuration", http_status=400) from error
        except OSError as error:
            raise GatewayError("配置文件读写失败：" + str(error), error_code="configuration_io_failed", http_status=500) from error

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
        security_policy = access_manager.export_policy()
        layout = _build_layout_snapshot()
        return f"""<!doctype html>
<html lang="zh-CN" data-theme="dark">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>NEH 运维页</title>
  <style>
    :root {{
      --bg: #f6f7f9;
      --bg-grad-a: rgba(59, 130, 246, 0.08);
      --bg-grad-b: rgba(139, 92, 246, 0.06);
      --surface: rgba(255, 255, 255, 0.95);
      --surface-strong: #ffffff;
      --panel: rgba(255, 255, 255, 0.92);
      --panel-soft: rgba(248, 250, 252, 0.85);
      --border: rgba(148, 163, 184, 0.35);
      --border-soft: rgba(148, 163, 184, 0.22);
      --border-strong: rgba(59, 130, 246, 0.55);
      --text: #0f172a;
      --muted: #475569;
      --muted-strong: #334155;
      --accent: #2563eb;
      --accent-strong: #1d4ed8;
      --shadow: 0 10px 40px rgba(0, 0, 0, 0.08);
      --shadow-soft: 0 4px 18px rgba(0, 0, 0, 0.06);
      --radius: 16px;
      --radius-sm: 12px;
    }}
    [data-theme="dark"] {{
      --bg: #0b1120;
      --bg-grad-a: rgba(34, 211, 238, 0.12);
      --bg-grad-b: rgba(124, 58, 237, 0.10);
      --surface: rgba(15, 23, 42, 0.92);
      --surface-strong: rgba(15, 23, 42, 0.96);
      --panel: rgba(15, 23, 42, 0.88);
      --panel-soft: rgba(30, 41, 59, 0.70);
      --border: rgba(148, 163, 184, 0.18);
      --border-soft: rgba(148, 163, 184, 0.12);
      --border-strong: rgba(34, 211, 238, 0.45);
      --text: #f1f5f9;
      --muted: #94a3b8;
      --muted-strong: #cbd5e1;
      --accent: #38bdf8;
      --accent-strong: #22d3ee;
      --shadow: 0 24px 70px rgba(0, 0, 0, 0.35);
      --shadow-soft: 0 14px 38px rgba(0, 0, 0, 0.22);
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      padding: 24px;
      min-height: 100vh;
      background:
        radial-gradient(circle at 8% 8%, var(--bg-grad-a), transparent 30rem),
        radial-gradient(circle at 92% 6%, var(--bg-grad-b), transparent 34rem),
        linear-gradient(135deg, var(--bg) 0%, var(--panel-soft) 100%);
      color: var(--text);
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", "Noto Sans SC", sans-serif;
      transition: background 0.3s ease, color 0.3s ease;
    }}
    .app-shell {{ max-width: 1200px; margin: 0 auto; }}
    h1 {{ margin: 0 0 6px 0; font-size: clamp(24px, 2vw, 34px); font-weight: 780; letter-spacing: -0.03em; }}
    h2 {{ margin: 0 0 12px 0; font-size: 16px; font-weight: 720; color: var(--muted-strong); }}
    .eyebrow {{ color: var(--accent-strong); font-size: 12px; font-weight: 780; letter-spacing: 0.12em; text-transform: uppercase; margin-bottom: 8px; }}
    .muted {{ color: var(--muted); }}
    .grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 16px; margin-top: 20px; }}
    .card {{
      position: relative;
      overflow: hidden;
      background: linear-gradient(180deg, var(--panel), var(--panel-soft));
      border: 1px solid var(--border);
      border-radius: var(--radius);
      padding: 18px;
      box-shadow: var(--shadow-soft);
      transition: background 0.3s ease, border-color 0.3s ease;
    }}
    .card::before {{
      content: "";
      position: absolute;
      inset: 0 0 auto;
      height: 1px;
      background: linear-gradient(90deg, transparent, var(--accent-strong), transparent);
      pointer-events: none;
      opacity: 0.7;
    }}
    .mono {{ font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace; word-break: break-all; font-size: 13px; color: var(--accent-strong); }}
    ul {{ margin: 8px 0 0 0; padding: 0; list-style: none; }}
    li {{ margin: 8px 0; padding: 8px 10px; border-radius: 10px; background: var(--panel-soft); border: 1px solid var(--border-soft); display: flex; justify-content: space-between; align-items: center; gap: 10px; flex-wrap: wrap; transition: background 0.3s ease; }}
    li span:first-child {{ color: var(--muted); font-size: 13px; }}
    .theme-toggle {{
      position: fixed;
      top: 16px;
      right: 16px;
      z-index: 100;
      width: 40px;
      height: 40px;
      border-radius: 999px;
      border: 1px solid var(--border);
      background: var(--surface-strong);
      color: var(--text);
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 18px;
      box-shadow: var(--shadow-soft);
      transition: background 0.3s ease, color 0.3s ease, border-color 0.3s ease, transform 0.2s ease;
    }}
    .theme-toggle:hover {{ transform: scale(1.08); border-color: var(--border-strong); }}
    @media (max-width: 720px) {{
      body {{ padding: 14px; }}
      .card {{ border-radius: 14px; padding: 14px; }}
      .theme-toggle {{ top: 10px; right: 10px; width: 36px; height: 36px; font-size: 16px; }}
    }}
  </style>
</head>
<body>
  <button id="themeToggle" class="theme-toggle" title="切换主题" aria-label="切换白天/黑夜主题">☀️</button>
  <div class="app-shell">
    <div class="eyebrow">运维控制台</div>
    <h1>NEH 运维页</h1>
    <p class="muted">用于快速查看宿主机运行时、目录布局和安全策略的最低可用状态页。</p>
    <div class="grid">
      <section class="card">
        <h2>运行时</h2>
        <ul>
          <li><span>机器人</span><span class="mono">{runtime.robot.manifest.robot_name}</span></li>
          <li><span>型号</span><span class="mono">{runtime.robot.manifest.robot_model}</span></li>
          <li><span>活动任务数</span><span class="mono">{len(runtime.tasks.active())}</span></li>
          <li><span>最新事件游标</span><span class="mono">{runtime.events.latest_cursor()}</span></li>
        </ul>
      </section>
      <section class="card">
        <h2>目录布局</h2>
        <ul>
          <li><span>基础目录</span><span class="mono">{layout["base_dir"]}</span></li>
          <li><span>日志目录</span><span class="mono">{layout["log_dir"]}</span></li>
          <li><span>日志文件</span><span class="mono">{layout["log_file"]}</span></li>
          <li><span>制品目录</span><span class="mono">{layout["artifact_dir"]}</span></li>
          <li><span>状态缓存</span><span class="mono">{layout["state_store_mode"]}</span></li>
        </ul>
      </section>
      <section class="card">
        <h2>安全策略</h2>
        <ul>
          <li><span>鉴权</span><span class="mono">{security_policy["auth"]}</span></li>
          <li><span>能力策略</span><span class="mono">{security_policy["capability_policy"]}</span></li>
        </ul>
      </section>
      <section class="card">
        <h2>调试入口</h2>
        <ul>
          <li><span>健康检查</span><span class="mono">/api/health</span></li>
          <li><span>工具列表</span><span class="mono">/api/tools</span></li>
          <li><span>安全策略</span><span class="mono">/api/security/policy</span></li>
          <li><span>部署布局</span><span class="mono">/api/deployment/layout</span></li>
          <li><span>任务管理</span><span class="mono">/api/tasks</span></li>
          <li><span>事件流</span><span class="mono">/events/stream</span></li>
        </ul>
      </section>
    </div>
  </div>
  <script>
    (() => {{
      const btn = document.getElementById("themeToggle");
      function initTheme() {{
        const saved = localStorage.getItem("nuwax-theme");
        const prefersDark = window.matchMedia && window.matchMedia("(prefers-color-scheme: dark)").matches;
        const theme = saved || (prefersDark ? "dark" : "light");
        document.documentElement.setAttribute("data-theme", theme);
        updateIcon(theme);
      }}
      function toggleTheme() {{
        const current = document.documentElement.getAttribute("data-theme") || "dark";
        const next = current === "dark" ? "light" : "dark";
        document.documentElement.setAttribute("data-theme", next);
        localStorage.setItem("nuwax-theme", next);
        updateIcon(next);
      }}
      function updateIcon(theme) {{
        if (btn) btn.textContent = theme === "dark" ? "🌙" : "☀️";
      }}
      initTheme();
      if (btn) btn.addEventListener("click", toggleTheme);
    }})();
  </script>
</body>
</html>"""

    @router.get("/api/health")
    async def api_health(request: Request) -> Dict[str, Any]:
        return await run_in_threadpool(lambda: to_jsonable(health_summary(runtime)))

    @router.get("/api/health/full")
    async def api_health_full(request: Request) -> Dict[str, Any]:
        """Return the authoritative runtime service-health aggregation."""

        try:
            summary = await run_in_threadpool(health_summary, runtime)
        except Exception as exc:
            raise as_gateway_error(exc) from exc
        return to_jsonable(summary)

    @router.get("/api/capabilities")
    async def api_capabilities(
        request: Request,
        exposed_only: Optional[bool] = None,
    ) -> Dict[str, Any]:
        capabilities = runtime.capabilities.for_robot(runtime.robot.manifest.robot_name)
        if exposed_only:
            capabilities = [cap for cap in capabilities if cap.agent_tool]
        return {"capabilities": [{"name": cap.name, "description": cap.description,
                                   "input_schema": cap.input_schema, "kind": cap.kind,
                                   "motion": cap.motion, "agent_tool": cap.agent_tool}
                                  for cap in capabilities]}

    @router.get("/api/tools")
    async def api_tools(request: Request) -> Dict[str, Any]:
        return {"tools": [{"name": cap.name, "description": cap.description,
                           "inputSchema": cap.input_schema, "kind": cap.kind}
                          for cap in runtime.capabilities.agent_tools(runtime.robot.manifest.robot_name)]}

    @router.get("/api/security/policy")
    async def api_security_policy(request: Request) -> Dict[str, Any]:
        return access_manager.export_policy()

    @router.get("/api/config/schema")
    async def api_config_schema(request: Request, robot: Optional[str] = None, profile: Optional[str] = None) -> Dict[str, Any]:
        """返回全量配置 schema（按板块分组，敏感项脱敏）。"""

        return await call_settings(settings_application.schema, robot, profile)

    @router.get("/api/pointcloud/dense")
    async def api_dense_pointcloud(
        request: Request,
        epoch: str = Query(default="", max_length=128),
        offset: int = Query(default=0, ge=0, le=2 ** 53 - 1),
        delta: bool = False,
    ) -> Response:
        chunk = await run_in_threadpool(runtime.geometry.get_dense_cloud_chunk,
                                        expected_epoch=epoch, offset_points=offset, allow_delta=delta)
        metadata = to_jsonable({name: getattr(chunk, name) for name in (
            "epoch", "source_timestamp_ns", "frame_id", "total_points", "offset_points",
            "bounds_min", "bounds_max", "base_epoch", "base_offset_points", "removed_ranges")})
        return Response(chunk.xyz_f32_le, media_type="application/octet-stream", headers={
            "Cache-Control": "no-store",
            "X-Nuwax-Pointcloud": json.dumps(metadata, ensure_ascii=True, separators=(",", ":")),
        })

    @router.post("/api/config/save")
    async def api_config_save(request: Request) -> Dict[str, Any]:
        """原子保存并应用支持热加载的项；重启由独立操作显式选择。"""

        payload = await request.json()
        if not isinstance(payload, dict):
            raise as_gateway_error(ValueError("请求体必须为对象 {key: value}"))
        updates = payload.get("updates", payload)
        if not isinstance(updates, dict):
            raise as_gateway_error(ValueError("updates 必须为对象 {key: value}"))
        clean = {str(k): str(v) for k, v in updates.items() if isinstance(v, (str, int, float, bool))}
        result = await call_settings(settings_application.save, clean, robot=payload.get("robot"),
                                     profile=payload.get("profile"), expected_revision=payload.get("revision"))
        result["rejected"].update({str(k): "配置值必须是单个字符串、数字或布尔值" for k in updates if str(k) not in clean})
        return result

    @router.post("/api/config/profiles")
    async def api_config_create_profile(request: Request) -> Dict[str, Any]:
        from core.config_profiles import create_profile
        payload = await request.json()
        if not isinstance(payload, dict) or not all(isinstance(payload.get(key), str) for key in ("robot", "profile", "name")):
            raise GatewayError("请指定机器人、来源方案和新方案名称", error_code="invalid_configuration", http_status=400)
        return await call_settings(create_profile, root=config_io._CORE_ENV.parent, robot=payload["robot"],
                                   source=payload["profile"], name=payload["name"], expected_revision=payload.get("revision"),
                                   revision_owner=settings_application.revision_owner)

    @router.post("/api/config/activate")
    async def api_config_activate_profile(request: Request) -> Dict[str, Any]:
        payload = await request.json()
        if not isinstance(payload, dict) or not all(isinstance(payload.get(key), str) for key in ("robot", "profile")):
            raise GatewayError("请指定要应用的机器人与配置方案", error_code="invalid_configuration", http_status=400)
        return await call_settings(settings_application.activate, robot=payload["robot"], profile=payload["profile"],
                                   expected_revision=payload.get("revision"))

    @router.post("/api/config/restart")
    async def api_config_restart(request: Request, background: BackgroundTasks) -> Dict[str, Any]:
        from core.config_values import configuration_location
        from core.config_profiles import check_revision
        from gateways.settings_restart import queue_restart, dispatch_restart
        payload = await request.json()
        if not isinstance(payload, dict):
            raise GatewayError("重启请求必须为对象", error_code="invalid_configuration", http_status=400)
        root = config_io._CORE_ENV.parent
        location = await call_settings(configuration_location, root=root, robot=payload.get("robot"), profile=payload.get("profile"))
        if location:
            if not await run_in_threadpool(settings_application._is_active, location):
                raise GatewayError("请先应用所选配置方案", error_code="configuration_not_active", http_status=409)
            await call_settings(check_revision, location, payload.get("revision"), root=root,
                                revision_owner=settings_application.revision_owner)
        result = await call_settings(queue_restart, root)
        background.add_task(dispatch_restart, root, result)
        return result

    @router.get("/api/config/status")
    async def api_config_status() -> Dict[str, Any]:
        return await call_settings(settings_application.status)


    @router.get("/api/deployment/layout")
    async def api_deployment_layout(request: Request) -> Dict[str, Any]:
        return _build_layout_snapshot()

    @router.get("/ops", response_class=HTMLResponse)
    def ops_page(request: Request) -> HTMLResponse:
        return HTMLResponse(_render_ops_page())

    def _invoke_capability(request, capability_name, arguments, *, map_response=False):
        result = invoke(runtime, capability_name, arguments, "http")
        if map_response:
            return Response(to_map_json_bytes(result), media_type="application/json")
        return to_jsonable(result)

    @router.post("/api/capabilities/{capability_name}/invoke")
    async def api_invoke_capability(
        capability_name: str,
        request: Request,
    ) -> Dict[str, Any]:
        try:
            body = json.loads(await request.body(), parse_constant=strict_json_constant)
            arguments = body.get("arguments", {})
        except (ValueError, AttributeError) as exc:
            raise GatewayError("能力调用请求必须是 JSON 对象。", error_code="invalid_params", http_status=400) from exc
        return await run_in_threadpool(_invoke_capability, request, capability_name, arguments)

    @router.get("/api/tasks")
    def api_tasks(
        request: Request,
        history_limit: int = 20,
    ) -> Dict[str, Any]:
        return {"active_tasks": to_jsonable(runtime.tasks.active()),
                "history_tasks": to_jsonable(runtime.tasks.history(history_limit)),
                "history_limit": history_limit}

    @router.get("/api/tasks/{task_id}")
    def api_task_status(task_id: str, request: Request) -> Dict[str, Any]:
        task = runtime.tasks.status(task_id)
        if task is None:
            raise GatewayError("任务不存在或已移出历史。", error_code="task_not_found", http_status=404)
        return {"task": to_jsonable(task)}

    @router.post("/api/tasks/{task_id}/cancel")
    def api_cancel_task(task_id: str, request: Request) -> Dict[str, Any]:
        return _invoke_capability(request, "cancel_task", {"task_id": task_id})






    @router.get("/api/artifacts/summary")
    def api_artifacts_summary(request: Request) -> Dict[str, Any]:
        return to_jsonable(runtime.artifacts.get_summary())



    @router.get("/api/perception/runtime")
    def api_perception_runtime(request: Request) -> Dict[str, Any]:
        return _invoke_capability(request, "get_perception_runtime_status", {})

    @router.post("/api/perception/runtime/start")
    def api_perception_runtime_start(request: Request) -> Dict[str, Any]:
        return _invoke_capability(request, "start_perception_runtime", {})

    @router.post("/api/perception/runtime/stop")
    def api_perception_runtime_stop(request: Request) -> Dict[str, Any]:
        return _invoke_capability(request, "stop_perception_runtime", {})


    @router.post("/api/localization/relocalize")
    async def api_localization_relocalize(request: Request) -> Dict[str, Any]:
        payload = await request.json()
        return await run_in_threadpool(_invoke_capability, request, "relocalize_active_map", payload)


    @router.get("/api/maps/catalog", response_model=Dict[str, Any])
    def api_maps_catalog(request: Request) -> Response:
        return _invoke_capability(request, "list_maps", {}, map_response=True)

    @router.delete("/api/maps/catalog/{map_name}", response_model=Dict[str, Any])
    def api_maps_catalog_delete(
        map_name: str,
        request: Request,
        delete_memory_library: Optional[bool] = None,
    ) -> Response:
        arguments = {"map_name": map_name}
        if delete_memory_library is not None:
            arguments["delete_memory_library"] = delete_memory_library
        return _invoke_capability(request, "delete_map", arguments, map_response=True)

    @router.post("/api/maps/load", response_model=Dict[str, Any])
    async def api_maps_activate(request: Request) -> Response:
        payload = await request.json()
        return await run_in_threadpool(_invoke_capability, request, "load_map", payload, map_response=True)


    @router.get("/api/maps/active", response_model=Dict[str, Any])
    async def api_maps_active(request: Request) -> Response:
        return await run_in_threadpool(_invoke_capability, request, "get_active_map", {}, map_response=True)

    @router.post("/api/maps/save", response_model=Dict[str, Any])
    async def api_maps_finalize(request: Request) -> Response:
        payload = await request.json()
        return await run_in_threadpool(_invoke_capability, request, "save_map", payload, map_response=True)


    @router.post("/api/maps/cleanup")
    async def api_map_cleanup_preview(request: Request):
        payload = await request.json()
        return await run_in_threadpool(_invoke_capability, request, "preview_map_cleanup", payload)

    @router.get("/api/maps/cleanup/{preview_id}")
    def api_map_cleanup_status(preview_id: str, request: Request):
        return _invoke_capability(request, "get_map_cleanup", {"preview_id": preview_id})

    @router.get("/api/maps/cleanup/{preview_id}/points/{kind}")
    def api_map_cleanup_points(preview_id: str, kind: str):
        return FileResponse(runtime.map_cleanup.points(preview_id, kind), media_type="application/octet-stream")

    @router.post("/api/maps/cleanup/{preview_id}/cancel")
    def api_map_cleanup_cancel(preview_id: str, request: Request):
        return _invoke_capability(request, "cancel_map_cleanup", {"preview_id": preview_id})

    @router.post("/api/maps/cleanup/{preview_id}/apply")
    async def api_map_cleanup_apply(preview_id: str, request: Request):
        payload = await request.json()
        return await run_in_threadpool(_invoke_capability, request, "apply_map_cleanup",
            {**payload, "preview_id": preview_id})

    @router.get("/api/navigation/forbidden-zones")
    async def api_navigation_forbidden_zones_get(
        request: Request,
        map_name: Optional[str] = None,
    ) -> Dict[str, Any]:
        arguments = {"map_name": map_name} if map_name is not None else {}
        return await run_in_threadpool(_invoke_capability, request, "list_forbidden_zones", arguments)

    @router.put("/api/navigation/forbidden-zones")
    async def api_navigation_forbidden_zones_put(
        request: Request,
    ) -> Dict[str, Any]:
        return await run_in_threadpool(_invoke_capability, request, "replace_forbidden_zones", await request.json())


    @router.post("/api/navigation/place-goal")
    async def api_navigation_place_goal(request: Request) -> Dict[str, Any]:
        """地点文本与智能体共用地图地点导航能力。"""
        payload = await request.json()
        return await run_in_threadpool(_invoke_capability, request, "navigate_to_place", payload)

    @router.get("/api/navigation/snapshot")
    def api_navigation_snapshot(request: Request):
        return _invoke_capability(request, "get_navigation_state", {})

    @router.get("/api/navigation/state")
    async def api_navigation_state(request: Request):
        async def generate():
            queue = ClientQueue(asyncio.get_running_loop(), maxsize=1)
            subscription = runtime.streams.stream_bus.subscribe(queue.put, topics=("navigation.state",))
            try:
                latest = runtime.streams.stream_bus.latest_frame("navigation.state")
                if latest is not None:
                    yield "data: " + json.dumps(to_jsonable(latest.payload), ensure_ascii=False) + "\n\n"
                while not await request.is_disconnected():
                    try:
                        frame = await asyncio.wait_for(queue.get(), timeout=runtime.config.gateway.sse_keepalive_sec)
                    except asyncio.TimeoutError:
                        yield ": keepalive\n\n"
                        continue
                    yield "data: " + json.dumps(to_jsonable(frame.payload), ensure_ascii=False) + "\n\n"
            finally:
                queue.close()
                runtime.streams.stream_bus.unsubscribe(subscription)
        return StreamingResponse(generate(), media_type="text/event-stream")

    @router.get("/api/navigation/semantic_context")
    def api_navigation_semantic_context(request: Request):
        return _invoke_capability(request, "get_navigation_semantic_context", {})

    @router.post("/api/navigation/cancel")
    async def api_navigation_cancel(request: Request):
        return await run_in_threadpool(_invoke_capability, request, "cancel_navigation", await request.json())

    @router.get("/api/memory/summary")
    def api_memory_summary(request: Request) -> Dict[str, Any]:
        return {"memory_summary": to_jsonable(runtime.memory.get_summary())}

    @router.get("/api/memory/libraries")
    def api_memory_libraries(request: Request) -> Dict[str, Any]:
        return _invoke_capability(request, "list_memory_libraries", {})

    @router.delete("/api/memory/libraries/{library_name}", response_model=Dict[str, Any])
    def api_delete_memory_library(library_name: str, request: Request) -> Response:
        return _invoke_capability(request, "delete_memory_library", {"library_name": library_name}, map_response=True)

    @router.get("/api/memory/locations")
    def api_memory_locations(request: Request, limit: int = Query(20, ge=1, le=1000)) -> Dict[str, Any]:
        # 未启用 active 库时返回空列表（避免跨库串数据），service 已默认按 active 库过滤。
        return {
            "locations": to_jsonable(
                runtime.memory.list_tagged_locations(
                    limit=limit,
                    library_name=runtime.memory.get_active_library_name(),
                )
            ),
            "memory_summary": to_jsonable(runtime.memory.get_summary()),
        }

    @router.get("/api/memory/semantic")
    def api_memory_semantic(request: Request, limit: int = Query(20, ge=1, le=1000)) -> Dict[str, Any]:
        return {
            "entries": to_jsonable(
                runtime.memory.list_semantic_memories(
                    limit=limit,
                    library_name=runtime.memory.get_active_library_name(),
                )
            ),
            "memory_summary": to_jsonable(runtime.memory.get_summary()),
        }

    @router.get("/api/memory/instances")
    def api_memory_instances(request: Request, library_name: Optional[str] = None,
                             label: Optional[str] = None) -> Dict[str, Any]:
        arguments = {}
        if library_name is not None:
            arguments["library_name"] = library_name
        if label is not None:
            arguments["label"] = label
        return _invoke_capability(request, "list_semantic_instances", arguments)

    @router.get("/api/memory/scene_spaces")
    def api_memory_scene_spaces(
        request: Request,
        limit: int = Query(20, ge=1, le=1000),
        scene_space_id: Optional[str] = None,
        map_name: Optional[str] = None,
        review_status: Optional[str] = None,
    ) -> Dict[str, Any]:
        arguments: Dict[str, Any] = {"limit": limit}
        for key, value in {
            "scene_space_id": scene_space_id,
            "map_name": map_name,
            "review_status": review_status,
        }.items():
            if value is not None:
                arguments[key] = value
        return _invoke_capability(request, "list_scene_spaces", arguments)

    @router.get("/api/memory/scene_spaces/query")
    def api_memory_scene_spaces_query(
        request: Request,
        query: str,
        limit: int = Query(5, ge=1, le=1000),
        similarity_threshold: float = 0.25,
        map_name: Optional[str] = None,
        review_status: Optional[str] = None,
        max_distance_m: Optional[float] = None,
    ) -> Dict[str, Any]:
        arguments: Dict[str, Any] = {
            "query": query,
            "limit": limit,
            "similarity_threshold": similarity_threshold,
        }
        for key, value in {
            "map_name": map_name,
            "review_status": review_status,
            "max_distance_m": max_distance_m,
        }.items():
            if value is not None:
                arguments[key] = value
        return _invoke_capability(request, "query_scene_spaces", arguments)

    @router.get("/api/memory/scene_spaces/{scene_space_id}")
    def api_memory_scene_space(scene_space_id: str, request: Request) -> Dict[str, Any]:
        return _invoke_capability(request, "get_scene_space", {"scene_space_id": scene_space_id})

    @router.get("/artifacts/{artifact_id}/meta")
    def api_artifact_meta(artifact_id: str, request: Request) -> Dict[str, Any]:
        try:
            return runtime.artifacts._store.get_ref(artifact_id).model_dump(mode="json")
        except Exception as exc:
            raise as_gateway_error(exc) from exc

    @router.get("/artifacts/{artifact_id}")
    def api_artifact_raw(artifact_id: str, request: Request) -> FileResponse:
        try:
            return ArtifactFileResponse(runtime.artifacts._store, artifact_id)
        except Exception as exc:
            raise as_gateway_error(exc) from exc

    return router
