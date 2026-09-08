from __future__ import annotations

import asyncio
import json
import math
from typing import Dict, Optional, Any

from fastapi import APIRouter, Query
from fastapi import Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import FileResponse
from fastapi.responses import HTMLResponse
from fastapi.responses import StreamingResponse
from pydantic import BaseModel, Field

from contracts.geometry import Pose, Quaternion, Vector3
from contracts.forbidden_zones import ReplaceForbiddenZonesRequest
from contracts.navigation import NavigationGoal
from gateways.auth import GatewayAccessManager
from gateways.errors import GatewayError, as_gateway_error
from gateways.feedback import build_task_summary
from gateways.runtime import GatewayRuntime
from gateways.serialization import to_jsonable
from core import config_io
from skills.handlers import localization_skills
from skills.handlers import memory_skills


class CapabilityInvokeRequest(BaseModel):
    """能力调用请求体。"""

    arguments: Dict[str, Any] = Field(default_factory=dict, description="调用参数。")


class NavigationGoalRequest(BaseModel):
    """公共导航目标请求。"""

    model_config = {"extra": "forbid"}

    goal_id: str = Field(..., min_length=1, description="客户端生成的 goal 标识。")
    pose: Dict[str, Any] = Field(default_factory=dict, description="目标位姿字典。")
    frame_id: str = Field(default="map", description="位姿所在的坐标系。")


class NavigationCancelRequest(BaseModel):
    """公共导航取消请求。"""

    model_config = {"extra": "forbid"}

    goal_id: str = Field(..., min_length=1, description="待取消的 goal 标识。")


def _build_navigation_goal(goal: NavigationGoalRequest) -> NavigationGoal:
    pose_payload = dict(goal.pose or {})
    if not pose_payload:
        raise ValueError("pose 不能为空。")

    if "position" in pose_payload or "orientation" in pose_payload:
        pose_payload.setdefault("frame_id", goal.frame_id)
        target_pose = Pose.model_validate(pose_payload)
    else:
        if "x" not in pose_payload or "y" not in pose_payload:
            raise ValueError("pose 至少需要 x 和 y。")
        yaw_rad = float(pose_payload.get("yaw_rad", pose_payload.get("yaw", 0.0)))
        half_yaw = yaw_rad / 2.0
        target_pose = Pose(
            frame_id=goal.frame_id,
            position=Vector3(
                x=float(pose_payload["x"]),
                y=float(pose_payload["y"]),
                z=float(pose_payload.get("z", 0.0)),
            ),
            orientation=Quaternion(z=math.sin(half_yaw), w=math.cos(half_yaw)),
        )

    return NavigationGoal(
        goal_id=goal.goal_id,
        target_pose=target_pose,
        metadata={"requested_by": "http_api", "source": "api_navigation_goal"},
    )


def _payload_value(payload: Any, key: str, default: Any = None) -> Any:
    if isinstance(payload, dict):
        return payload.get(key, default)
    return getattr(payload, key, default)


def _zero_ack_observed(value: Any) -> bool:
    """Only explicit actuator-zero evidence permits a public cancelled terminal."""

    context_metadata = _payload_value(value, "metadata", {}) or {}
    navigation_state = _payload_value(value, "navigation_state", value)
    state_metadata = _payload_value(navigation_state, "metadata", {}) or {}
    for metadata in (context_metadata, state_metadata):
        if not isinstance(metadata, dict):
            continue
        if metadata.get("zero_applied") is True:
            return True
        ack = metadata.get("actuator_zero_ack")
        if ack is True:
            return True
        if isinstance(ack, dict) and (
            ack.get("zero_applied") is True or ack.get("zeroed") is True
        ):
            return True
        terminal = metadata.get("nav_terminal") or metadata.get("terminal")
        if isinstance(terminal, dict) and terminal.get("zero_applied") is True:
            return True
    return False


def _navigation_state_payload(
    value: Any,
    *,
    fallback_goal_id: Optional[str] = None,
) -> Dict[str, Any]:
    stream_payload = _payload_value(value, "payload", value)
    navigation_state = _payload_value(stream_payload, "navigation_state", stream_payload)
    current_goal = _payload_value(stream_payload, "current_goal")

    raw_status = _payload_value(
        navigation_state,
        "status",
        _payload_value(navigation_state, "state", "unknown"),
    )
    raw_status = str(getattr(raw_status, "value", raw_status) or "unknown").lower()
    public_state = raw_status.upper()
    explicit_zero_ack = _zero_ack_observed(stream_payload)
    typed_cancel_terminal = raw_status in {"cancelled", "canceled"}
    zero_ack_observed = explicit_zero_ack or typed_cancel_terminal
    cancel_terminal_source = None
    if explicit_zero_ack:
        cancel_terminal_source = "actuator_zero_ack"
    elif typed_cancel_terminal:
        # The production navigation contract only emits CANCELLED after the
        # correlated motion-guard ZEROED acknowledgement has completed.
        cancel_terminal_source = "typed_navigation_terminal"
    if raw_status in {"cancelled", "canceled"}:
        public_state = "CANCELED"
    elif raw_status in {"cancelling", "cancel_pending", "cancel_requested"}:
        public_state = "CANCELLING"
    message = _payload_value(navigation_state, "message", "") or ""
    if public_state == "CANCELLING" and not zero_ack_observed:
        message = "取消请求已下发，正在等待执行器零速确认。"

    state_goal_id = _payload_value(
        navigation_state,
        "current_goal_id",
        _payload_value(navigation_state, "goal_id"),
    )
    current_goal_id = _payload_value(current_goal, "goal_id")
    goal_id = state_goal_id or current_goal_id or fallback_goal_id
    timestamp = _payload_value(
        navigation_state,
        "updated_at",
        _payload_value(navigation_state, "timestamp"),
    )
    return {
        "state": public_state,
        "status": public_state.lower(),
        "goal_id": goal_id,
        "message": message,
        "timestamp": to_jsonable(timestamp),
        "remaining_distance_m": _payload_value(navigation_state, "remaining_distance_m"),
        "goal_reached": bool(_payload_value(navigation_state, "goal_reached", False)),
        "backend_name": _payload_value(stream_payload, "backend_name"),
        "cancel_zero_ack_observed": zero_ack_observed,
        "cancel_terminal_source": cancel_terminal_source,
    }


def _full_health_payload(summary: Dict[str, Any]) -> Dict[str, Any]:
    service_health = dict(summary.get("service_health") or {})
    entries = list(service_health.get("entries") or [])
    components = {
        str(entry.get("service_id")): entry
        for entry in entries
        if isinstance(entry, dict) and entry.get("service_id")
    }
    raw_status = str(service_health.get("status") or "unknown").lower()
    if raw_status in {"available", "ready", "idle", "disabled", "unsupported"}:
        overall = "healthy"
    elif raw_status in {"degraded", "stale"}:
        overall = "degraded"
    elif raw_status in {"missing", "error"}:
        overall = "unhealthy"
    else:
        overall = "unknown"

    return {
        "overall": overall,
        "ready": overall == "healthy",
        "readiness": {
            "aggregation_version": 1,
            "authoritative": bool(service_health.get("authoritative", False)),
            "status": raw_status,
            "components": components,
            "metadata": dict(service_health.get("metadata") or {}),
        },
        "sensors": {
            "robot": summary.get("latest_robot_snapshot"),
            "localization": summary.get("latest_localization"),
            "map": summary.get("latest_map"),
            "navigation": summary.get("latest_navigation"),
        },
        "aggregation_version": 1,
        "generated_at_utc": service_health.get("generated_at"),
    }




def build_http_router(runtime: GatewayRuntime, access_manager: GatewayAccessManager) -> APIRouter:
    """构造 HTTP（超文本传输协议）业务与管理接口。"""

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
<html lang="zh-CN" data-theme="dark">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>nuwax_robot_bridge 运维页</title>
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
    <h1>nuwax_robot_bridge 运维页</h1>
    <p class="muted">用于快速查看宿主机运行时、目录布局和安全策略的最低可用状态页。</p>
    <div class="grid">
      <section class="card">
        <h2>运行时</h2>
        <ul>
          <li><span>机器人</span><span class="mono">{health["robot_name"]}</span></li>
          <li><span>型号</span><span class="mono">{health["robot_model"]}</span></li>
          <li><span>已启动</span><span class="mono">{health["runtime_started"]}</span></li>
          <li><span>活动任务数</span><span class="mono">{health["active_task_count"]}</span></li>
          <li><span>最新事件游标</span><span class="mono">{health["latest_event_cursor"]}</span></li>
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
        return await run_in_threadpool(lambda: to_jsonable(runtime.get_health_summary()))

    @router.get("/api/health/full")
    async def api_health_full(request: Request) -> Dict[str, Any]:
        """Return the authoritative runtime service-health aggregation."""

        try:
            summary = await run_in_threadpool(runtime.get_health_summary)
        except Exception as exc:
            raise as_gateway_error(exc) from exc
        return _full_health_payload(to_jsonable(summary))

    @router.get("/api/capabilities")
    async def api_capabilities(
        request: Request,
        exposed_only: Optional[bool] = None,
    ) -> Dict[str, Any]:
        descriptors = await run_in_threadpool(lambda: runtime.list_capabilities(exposed_only=exposed_only))
        allowed = access_manager.filter_capabilities(descriptors)
        return {"capabilities": [descriptor.model_dump(mode="json") for descriptor in allowed]}

    @router.get("/api/tools")
    async def api_tools(
        request: Request,
        include_unsupported: bool = False,
    ) -> Dict[str, Any]:
        views = await run_in_threadpool(
            lambda: runtime.list_tools(
                exposed_only=True,
                include_unsupported=include_unsupported,
            )
        )
        return {"tools": to_jsonable(views)}

    @router.get("/api/security/policy")
    async def api_security_policy(request: Request) -> Dict[str, Any]:
        return access_manager.export_policy()

    @router.get("/api/config/schema")
    async def api_config_schema(request: Request) -> Dict[str, Any]:
        """返回全量配置 schema（按板块分组，敏感项脱敏）。"""

        return await run_in_threadpool(config_io.build_schema)

    @router.post("/api/config/save")
    async def api_config_save(request: Request) -> Dict[str, Any]:
        """批量写回配置项到对应 env 文件（原子写 + 备份）。重启后生效。"""

        payload = await request.json()
        if not isinstance(payload, dict):
            raise as_gateway_error(ValueError("请求体必须为对象 {key: value}"))
        updates = payload.get("updates", payload)
        if not isinstance(updates, dict):
            raise as_gateway_error(ValueError("updates 必须为对象 {key: value}"))
        clean = {str(k): str(v) for k, v in updates.items()}
        result = await run_in_threadpool(config_io.save_values, clean)
        return {
            "written": result.written,
            "skipped": result.skipped,
            "rejected": result.rejected,
            "backups": {k: bool(v) for k, v in result.backups.items()},
            "restart_required": bool(result.written),
            "effect_note": "已写入受控 Geometry 配置文件；需重启网关和 Geometry Runtime 生效（非热重载）。",
        }


    @router.get("/api/deployment/layout")
    async def api_deployment_layout(request: Request) -> Dict[str, Any]:
        return _build_layout_snapshot()

    @router.get("/api/runtime/governance")
    async def api_runtime_governance(request: Request) -> Dict[str, Any]:
        return {"runtime_governance": to_jsonable(runtime.get_runtime_governance_summary())}

    @router.get("/ops", response_class=HTMLResponse)
    async def ops_page(request: Request) -> HTMLResponse:
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
        descriptor = runtime.capability_registry.get_descriptor(capability_name)
        access_manager.authorize_capability(descriptor)
        requested_by = "local"
        request_context = _build_capability_request_context(
            request,
            capability_name=capability_name,
            requested_by=requested_by,
        )
        try:
            result = await run_in_threadpool(
                lambda: runtime.invoke_capability(
                    capability_name,
                    body.arguments,
                    requested_by=requested_by,
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
        bounded_history_limit = min(max(int(history_limit), 0), 100)

        def _summary_for_status(status: Any) -> Dict[str, Any]:
            task_id = str(_payload_value(status, "task_id", "") or "").strip()
            spec = None
            error_payload = None
            if task_id:
                try:
                    spec = runtime.task_manager.get_task_spec(task_id)
                except (KeyError, AttributeError):
                    spec = None
                try:
                    error_payload = runtime.task_manager.get_task_error_payload(task_id)
                except (KeyError, AttributeError):
                    error_payload = None
            return build_task_summary(
                status,
                spec=spec,
                error_payload=error_payload,
            )

        return {
            "active_tasks": [
                _summary_for_status(status)
                for status in runtime.task_manager.list_active_tasks()
            ],
            "history_tasks": [
                _summary_for_status(status)
                for status in runtime.task_manager.list_history(limit=bounded_history_limit)
            ],
            "history_limit": bounded_history_limit,
        }

    @router.get("/api/tasks/{task_id}")
    async def api_task_status(task_id: str, request: Request) -> Dict[str, Any]:
        try:
            return to_jsonable(runtime.get_task_snapshot(task_id))
        except Exception as exc:
            raise as_gateway_error(exc) from exc

    @router.post("/api/tasks/{task_id}/cancel")
    async def api_cancel_task(task_id: str, request: Request) -> Dict[str, Any]:
        try:
            return to_jsonable(
                runtime._handle_cancel_task(
                    {"task_id": task_id},
                    requested_by="local",
                )
            )
        except Exception as exc:
            raise as_gateway_error(exc) from exc






    @router.get("/api/artifacts/summary")
    async def api_artifacts_summary(request: Request) -> Dict[str, Any]:
        return to_jsonable(runtime.artifact_service.get_summary())



    @router.get("/api/perception/runtime")
    async def api_perception_runtime(request: Request) -> Dict[str, Any]:
        return {"perception_runtime": to_jsonable(runtime.perception_video_runtime.get_status())}

    @router.post("/api/perception/runtime/start")
    async def api_perception_runtime_start(request: Request) -> Dict[str, Any]:
        return {"perception_runtime": to_jsonable(runtime._manual_perception_start())}

    @router.post("/api/perception/runtime/stop")
    async def api_perception_runtime_stop(request: Request) -> Dict[str, Any]:
        return {"perception_runtime": to_jsonable(runtime._manual_perception_stop())}


    @router.get("/api/localization/session")
    async def api_localization_session(request: Request) -> Dict[str, Any]:
        try:
            return to_jsonable(
                localization_skills.handle_get_localization_session(
                    runtime._skill_ctx,
                    {},
                    requested_by="http_api",
                )
            )
        except Exception as exc:
            raise as_gateway_error(exc) from exc

    @router.post("/api/localization/relocalize")
    async def api_localization_relocalize(request: Request) -> Dict[str, Any]:
        payload = await request.json()
        if not isinstance(payload, dict):
            payload = {}
        try:
            return to_jsonable(
                localization_skills.handle_relocalize_active_map(
                    runtime._skill_ctx,
                    payload,
                    requested_by="http_api",
                )
            )
        except Exception as exc:
            raise as_gateway_error(exc) from exc



    @router.get("/api/maps/catalog")
    async def api_maps_catalog(request: Request) -> Dict[str, Any]:
        return {
            "maps": to_jsonable(runtime.map_workspace_service.list_maps()),
            "active_workspace": to_jsonable(runtime.map_workspace_service.get_active_workspace()),
        }

    @router.post("/api/maps/catalog")
    async def api_maps_catalog_create(request: Request) -> Dict[str, Any]:
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
        delete_memory_library: Optional[bool] = None,
    ) -> Dict[str, Any]:
        delete_result = runtime.map_workspace_service.delete_map(
            map_name=map_name,
            delete_memory_library=(
                delete_memory_library
                if delete_memory_library is not None
                else runtime.config.runtime_data.map_delete_also_delete_library
            ),
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
        return await run_in_threadpool(lambda: {
            "active_workspace": to_jsonable(runtime.map_workspace_service.get_active_workspace()),
        })

    @router.post("/api/maps/finalize")
    async def api_maps_finalize(request: Request) -> Dict[str, Any]:
        payload = await request.json()
        if not isinstance(payload, dict):
            payload = {}
        result = await run_in_threadpool(
            lambda: localization_skills.handle_finalize_active_map(
                runtime._skill_ctx,
                payload,
                requested_by="http_api",
            )
        )
        return {
            "map_asset_bundle": to_jsonable(result["map_asset_bundle"]),
            "active_workspace": to_jsonable(result["active_workspace"]),
        }


    @router.get("/api/navigation/forbidden-zones")
    async def api_navigation_forbidden_zones_get(
        request: Request,
        map_name: Optional[str] = None,
    ) -> Dict[str, Any]:
        try:
            envelope = await run_in_threadpool(
                runtime.navigation_service.list_forbidden_zones,
                map_name=map_name,
            )
        except Exception as exc:
            raise as_gateway_error(exc) from exc
        return to_jsonable(envelope)

    @router.put("/api/navigation/forbidden-zones")
    async def api_navigation_forbidden_zones_put(
        request: Request,
        body: ReplaceForbiddenZonesRequest,
    ) -> Dict[str, Any]:
        try:
            envelope = await run_in_threadpool(
                runtime.navigation_service.replace_forbidden_zones,
                map_name=body.map_name,
                zones=body.zones,
                expected_generation=body.expected_generation,
            )
        except Exception as exc:
            raise as_gateway_error(exc) from exc
        return to_jsonable(envelope)

    @router.post("/api/navigation/goal")
    async def api_navigation_goal(request: Request, goal: NavigationGoalRequest) -> Dict[str, Any]:
        """Submit a goal through the singleton production navigation service."""

        try:
            navigation_goal = _build_navigation_goal(goal)

            def submit_goal():
                admission = runtime._admit_motion_request(
                    "navigate_to_pose",
                    request_id=goal.goal_id,
                )
                admitted_goal = runtime._goal_with_motion_admission(
                    navigation_goal,
                    admission,
                )
                return runtime._run_with_motion_admission(
                    admission,
                    lambda: runtime.navigation_service.set_goal(admitted_goal),
                )

            context = await run_in_threadpool(submit_goal)
        except Exception as exc:
            raise as_gateway_error(exc) from exc
        return _navigation_state_payload(context, fallback_goal_id=goal.goal_id)

    @router.post("/api/navigation/place-goal")
    async def api_navigation_place_goal(request: Request) -> Dict[str, Any]:
        """将地点文本查询接入唯一 mapping-cloud owner 与 GoalLease。"""

        payload = await request.json()
        if not isinstance(payload, dict):
            payload = {}
        query_text = str(payload.get("query_text") or "").strip()
        query_request_id = str(
            payload.get("query_request_id") or "place-{}".format(uuid.uuid4().hex)
        ).strip()
        if not query_text:
            raise as_gateway_error(ValueError("query_text 不能为空。"))
        try:
            def submit_goal():
                admission = runtime._admit_motion_request(
                    "navigate_to_pose",
                    request_id=query_request_id,
                )
                motion_admission = {
                    "guard_generation": admission.generation,
                    "guard_boot_nonce": admission.boot_nonce,
                    "motion_stop_generation": admission.guard_motion_stop_generation,
                }
                return runtime._run_with_motion_admission(
                    admission,
                    lambda: runtime.map_workspace_service.submit_mapping_cloud_goal(
                        query_request_id=query_request_id,
                        query_text=query_text,
                        motion_admission=motion_admission,
                    ),
                )

            return to_jsonable(await run_in_threadpool(submit_goal))
        except Exception as exc:
            raise as_gateway_error(exc) from exc

    @router.get("/api/navigation/state")
    async def api_navigation_state(request: Request) -> StreamingResponse:
        """Stream authoritative ``navigation.state`` frames over SSE."""

        try:
            initial_context = await run_in_threadpool(
                runtime.navigation_service.refresh_navigation
            )
        except Exception as exc:
            raise as_gateway_error(exc) from exc

        async def _event_generator():
            loop = asyncio.get_running_loop()
            queue: asyncio.Queue[Any] = asyncio.Queue(maxsize=64)

            def _enqueue(frame: Any) -> None:
                def _push() -> None:
                    if queue.full():
                        try:
                            queue.get_nowait()
                        except asyncio.QueueEmpty:
                            pass
                    queue.put_nowait(frame)

                loop.call_soon_threadsafe(_push)

            subscription_id = runtime.stream_bus.subscribe(
                _enqueue,
                topics=("navigation.state",),
            )
            try:
                payload = _navigation_state_payload(initial_context)
                yield f"data: {json.dumps(payload, ensure_ascii=False)}\n\n"
                keepalive_sec = max(
                    0.1,
                    float(getattr(runtime.config.gateway, "sse_keepalive_sec", 15.0)),
                )
                while not await request.is_disconnected():
                    try:
                        frame = await asyncio.wait_for(queue.get(), timeout=keepalive_sec)
                    except asyncio.TimeoutError:
                        yield ": keepalive\n\n"
                        continue
                    payload = _navigation_state_payload(frame)
                    yield f"data: {json.dumps(payload, ensure_ascii=False)}\n\n"
            finally:
                runtime.stream_bus.unsubscribe(subscription_id)

        return StreamingResponse(_event_generator(), media_type="text/event-stream")

    @router.get("/api/navigation/semantic_context")
    async def api_navigation_semantic_context(request: Request) -> Dict[str, Any]:
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


    @router.post("/api/navigation/cancel")
    async def api_navigation_cancel(
        request: Request, body: NavigationCancelRequest
    ) -> Dict[str, Any]:
        """Request cancellation without claiming terminal before actuator zero ack."""

        try:
            current_context = await run_in_threadpool(
                runtime.navigation_service.refresh_navigation
            )
            current = _navigation_state_payload(current_context)
            current_goal_id = current.get("goal_id")
            if not current_goal_id:
                raise GatewayError(
                    "导航后端未提供活动 goal_id，无法安全执行定向取消。",
                    error_code="navigation_goal_identity_unavailable",
                    http_status=503,
                    details={"requested_goal_id": body.goal_id},
                )
            if current_goal_id != body.goal_id:
                raise GatewayError(
                    "当前活动导航目标与取消请求不匹配。",
                    error_code="navigation_goal_mismatch",
                    http_status=409,
                    details={
                        "requested_goal_id": body.goal_id,
                        "active_goal_id": current_goal_id,
                    },
                )
            if str(current.get("status") or "").lower() not in {
                "accepted",
                "planning",
                "running",
                "cancelling",
                "cancel_requested",
            }:
                raise GatewayError(
                    "当前没有可取消的活动导航目标。",
                    error_code="navigation_goal_not_active",
                    http_status=409,
                    details={"goal_id": body.goal_id, "state": current.get("state")},
                )
            context = await run_in_threadpool(
                runtime.navigation_service.cancel_goal,
                request_id=body.goal_id,
            )
        except Exception as exc:
            raise as_gateway_error(exc) from exc
        return _navigation_state_payload(context, fallback_goal_id=body.goal_id)

    @router.get("/api/memory/summary")
    def api_memory_summary(request: Request) -> Dict[str, Any]:
        return {"memory_summary": to_jsonable(runtime.memory_service.get_summary())}

    @router.get("/api/memory/libraries")
    def api_memory_libraries(request: Request) -> Dict[str, Any]:
        return {
            "libraries": to_jsonable(runtime.memory_service.list_memory_libraries()),
            "memory_summary": to_jsonable(runtime.memory_service.get_summary()),
        }

    @router.post("/api/memory/libraries")
    async def api_create_memory_library(request: Request) -> Dict[str, Any]:
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
    def api_memory_locations(request: Request, limit: int = Query(20, ge=1, le=1000)) -> Dict[str, Any]:
        # 未启用 active 库时返回空列表（避免跨库串数据），service 已默认按 active 库过滤。
        return {
            "locations": to_jsonable(
                runtime.memory_service.list_tagged_locations(
                    limit=limit,
                    library_name=runtime.memory_service.get_active_library_name(),
                )
            ),
            "memory_summary": to_jsonable(runtime.memory_service.get_summary()),
        }

    @router.get("/api/memory/semantic")
    def api_memory_semantic(request: Request, limit: int = Query(20, ge=1, le=1000)) -> Dict[str, Any]:
        return {
            "entries": to_jsonable(
                runtime.memory_service.list_semantic_memories(
                    limit=limit,
                    library_name=runtime.memory_service.get_active_library_name(),
                )
            ),
            "memory_summary": to_jsonable(runtime.memory_service.get_summary()),
        }

    @router.get("/api/memory/instances")
    def api_memory_instances(
        request: Request,
        library_name: Optional[str] = None,
        label: Optional[str] = None,
        limit: int = Query(500, ge=1, le=1000),
    ) -> Dict[str, Any]:
        # 同步路由由网关线程池执行，读取和序列化均不占用异步事件循环。
        active_library = runtime.memory_service.get_active_library_name()
        target_library = library_name.strip() if library_name is not None else active_library
        result = runtime.invoke_capability(
            "list_semantic_instances",
            {"library_name": target_library or "", "label": label},
            requested_by="http_api",
        )
        payload = to_jsonable(result)
        if isinstance(payload, dict) and "instances" not in payload and isinstance(payload.get("result"), dict):
            payload = payload["result"]
        if runtime.memory_service.get_active_library_name() != active_library:
            raise GatewayError("查询期间记忆库已切换，请刷新。", error_code="memory_library_changed", http_status=409)
        if isinstance(payload, dict) and isinstance(payload.get("instances"), list):
            instances = payload["instances"]
            payload = {
                **payload,
                "instances": instances[:limit],
                "library_name": target_library or None,
                "active_library_name": active_library,
                "total": len(instances),
                "limit": limit,
                "truncated": len(instances) > limit,
            }
        return payload

    @router.get("/api/memory/scene_spaces")
    def api_memory_scene_spaces(
        request: Request,
        limit: int = Query(20, ge=1, le=1000),
        scene_space_id: Optional[str] = None,
        map_name: Optional[str] = None,
        localization_session_id: Optional[str] = None,
        review_status: Optional[str] = None,
    ) -> Dict[str, Any]:
        arguments: Dict[str, Any] = {"limit": limit}
        for key, value in {
            "scene_space_id": scene_space_id,
            "map_name": map_name,
            "localization_session_id": localization_session_id,
            "review_status": review_status,
        }.items():
            if value is not None:
                arguments[key] = value
        return to_jsonable(memory_skills.handle_list_scene_spaces(runtime._skill_ctx, arguments, requested_by="http_api"))

    @router.get("/api/memory/scene_spaces/query")
    def api_memory_scene_spaces_query(
        request: Request,
        query: str,
        limit: int = Query(5, ge=1, le=1000),
        similarity_threshold: float = 0.25,
        map_name: Optional[str] = None,
        localization_session_id: Optional[str] = None,
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
            "localization_session_id": localization_session_id,
            "review_status": review_status,
            "max_distance_m": max_distance_m,
        }.items():
            if value is not None:
                arguments[key] = value
        return to_jsonable(memory_skills.handle_query_scene_spaces(runtime._skill_ctx, arguments, requested_by="http_api"))

    @router.get("/api/memory/scene_spaces/{scene_space_id}")
    def api_memory_scene_space(scene_space_id: str, request: Request) -> Dict[str, Any]:
        return to_jsonable(
            memory_skills.handle_get_scene_space(
                runtime._skill_ctx,
                {"scene_space_id": scene_space_id},
                requested_by="http_api",
            )
        )

    @router.get("/artifacts/{artifact_id}/meta")
    async def api_artifact_meta(artifact_id: str, request: Request) -> Dict[str, Any]:
        try:
            return runtime.artifact_store.get_ref(artifact_id).model_dump(mode="json")
        except Exception as exc:
            raise as_gateway_error(exc) from exc

    @router.get("/artifacts/{artifact_id}")
    async def api_artifact_raw(artifact_id: str, request: Request) -> FileResponse:
        try:
            ref = runtime.artifact_store.get_ref(artifact_id)
            path = runtime.artifact_store.get_file_path(artifact_id)
        except Exception as exc:
            raise as_gateway_error(exc) from exc
        return FileResponse(path, media_type=ref.mime_type, filename=path.name)

    return router
