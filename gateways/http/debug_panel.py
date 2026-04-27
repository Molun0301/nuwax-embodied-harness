from __future__ import annotations

import base64
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
from typing import Any, Dict, List, Optional, Tuple

from fastapi import APIRouter, Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import HTMLResponse

from contracts.base import utc_now
from contracts.debug_views import (
    DebugDataStatus,
    DebugEnvelope,
    DebugError,
    DebugLocalizationView,
    DebugLogEntry,
    DebugLogsTailView,
    DebugMapView,
    DebugNavigationView,
    DebugOverviewView,
    DebugPointCloudView,
    DebugSourceSummary,
    DebugVideoView,
)
from gateways.auth import GatewayAccessManager
from gateways.runtime import GatewayRuntime
from gateways.serialization import to_jsonable
from providers import ImageProvider


_LOG_LINE_RE = re.compile(r"^(?P<timestamp>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}(?:,\d{1,6})?) \[(?P<level>[A-Z]+)\] \[(?P<source>[^\]]+)\] (?P<message>.*)$")

_STALE_THRESHOLDS_SEC = {
    "health": 3.0,
    "state": 3.0,
    "map": 2.0,
    "localization": 1.0,
    "trajectory": 2.0,
    "navigation": 3.0,
    "exploration": 3.0,
    "pointcloud": 1.0,
    "video": 1.0,
    "logs": 10.0,
}


def build_debug_panel_router(runtime: GatewayRuntime, access_manager: GatewayAccessManager) -> APIRouter:
    """构造机器人本机只读 Web 调试面板路由。"""

    router = APIRouter()

    @router.get("/debug", response_class=HTMLResponse)
    @router.get("/debug/", response_class=HTMLResponse)
    async def debug_page(request: Request) -> HTMLResponse:
        access = access_manager.authenticate_local_debug_page(request)
        return HTMLResponse(_render_debug_page(access.token))

    @router.get("/api/debug/overview")
    @router.get("/api/debug/snapshot")
    async def debug_overview(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        health_summary = runtime.get_health_summary()
        state_snapshot = runtime.robot_state_service.get_latest_snapshot()
        localization_snapshot = runtime.localization_service.get_latest_snapshot()
        map_snapshot = runtime.mapping_service.get_latest_snapshot()
        navigation_context = runtime.navigation_service.get_latest_navigation_context()
        exploration_context = runtime.navigation_service.get_latest_exploration_context()
        pointcloud = await run_in_threadpool(lambda: _get_latest_pointcloud(runtime))
        video_info = await run_in_threadpool(lambda: _get_video_summary(runtime))
        log_info = await run_in_threadpool(lambda: _get_log_file_summary(runtime))
        overview = DebugOverviewView(
            health=_summary_from_payload(
                name="health",
                source="gateway_runtime",
                timestamp=_timestamp_from_value(health_summary),
                available=True,
                message="runtime_started=%s" % bool(health_summary.get("runtime_started")),
            ),
            state=_summary_from_payload(
                name="state",
                source="robot_state_service",
                timestamp=_timestamp_from_value(state_snapshot),
                available=state_snapshot is not None,
            ),
            localization=_summary_from_payload(
                name="localization",
                source="localization_service",
                timestamp=_timestamp_from_value(localization_snapshot),
                frame_id=_frame_id_from_localization(localization_snapshot),
                available=localization_snapshot is not None and getattr(localization_snapshot, "current_pose", None) is not None,
            ),
            map=_summary_from_payload(
                name="map",
                source="mapping_service",
                timestamp=_timestamp_from_value(map_snapshot),
                frame_id=getattr(map_snapshot, "frame_id", None),
                revision=_revision_from_map(map_snapshot),
                available=map_snapshot is not None and getattr(map_snapshot, "occupancy_grid", None) is not None,
            ),
            navigation=_summary_from_payload(
                name="navigation",
                source="navigation_service",
                timestamp=_navigation_timestamp(navigation_context),
                frame_id=_frame_id_from_navigation(navigation_context),
                available=navigation_context is not None,
                message=_navigation_message(navigation_context),
            ),
            exploration=_summary_from_payload(
                name="exploration",
                source="navigation_service",
                timestamp=_exploration_timestamp(exploration_context),
                available=exploration_context is not None,
                message=_exploration_message(exploration_context),
                metadata=_exploration_metadata(exploration_context),
            ),
            pointcloud=_summary_from_payload(
                name="pointcloud",
                source=_pointcloud_source(pointcloud),
                timestamp=_timestamp_from_value(pointcloud),
                frame_id=getattr(pointcloud, "frame_id", None),
                sequence=_sequence_from_metadata(pointcloud),
                available=pointcloud is not None,
                metadata=_pointcloud_metrics(pointcloud),
            ),
            video=_summary_from_payload(
                name="video",
                source=str(video_info.get("source", "image_provider")),
                timestamp=video_info.get("timestamp"),
                frame_id=video_info.get("frame_id"),
                available=bool(video_info.get("available")),
                message=str(video_info.get("message") or ""),
                metadata=dict(video_info.get("metadata") or {}),
            ),
            logs=_summary_from_payload(
                name="logs",
                source=str(log_info.get("source", "configured_log_file")),
                timestamp=log_info.get("timestamp"),
                sequence=log_info.get("sequence"),
                available=bool(log_info.get("available")),
                message=str(log_info.get("message") or ""),
                metadata=dict(log_info.get("metadata") or {}),
            ),
        )
        return _envelope(
            payload_type="debug.overview",
            source="debug_panel",
            data=overview,
            source_timestamp=_latest_timestamp(
                _timestamp_from_value(state_snapshot),
                _timestamp_from_value(localization_snapshot),
                _timestamp_from_value(map_snapshot),
                _navigation_timestamp(navigation_context),
                _exploration_timestamp(exploration_context),
                _timestamp_from_value(pointcloud),
                video_info.get("timestamp"),
                log_info.get("timestamp"),
            ),
            status=DebugDataStatus.AVAILABLE,
        )

    @router.get("/api/debug/map")
    @router.get("/api/debug/map/frame/latest")
    async def debug_map(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        snapshot = runtime.mapping_service.get_latest_snapshot()
        if snapshot is None or getattr(snapshot, "occupancy_grid", None) is None:
            service_available = runtime.mapping_service.is_available()
            return _missing_envelope(
                payload_type="map.frame",
                source="mapping_service",
                status=DebugDataStatus.MISSING if service_available else DebugDataStatus.UNSUPPORTED,
                code="source_unavailable" if service_available else "source_unsupported",
                message="未取得平台地图缓存或占据栅格。" if service_available else "建图服务不可用或未配置。",
            )
        view = DebugMapView(
            map_snapshot=to_jsonable(snapshot),
            occupancy_grid=to_jsonable(snapshot.occupancy_grid),
            cost_map=to_jsonable(snapshot.cost_map),
            semantic_map=to_jsonable(snapshot.semantic_map),
        )
        source_timestamp = _timestamp_from_value(snapshot)
        return _envelope(
            payload_type="map.frame",
            source="mapping_service",
            data=view,
            source_timestamp=source_timestamp,
            frame_id=getattr(snapshot, "frame_id", None),
            revision=_revision_from_map(snapshot),
            status=_status_for_timestamp(source_timestamp, "map"),
        )

    @router.get("/api/debug/localization")
    @router.get("/api/debug/localization/pose/latest")
    @router.get("/api/debug/localization/trajectory")
    async def debug_localization(request: Request, limit: int = 2000) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        snapshot = runtime.localization_service.get_latest_snapshot()
        history_limit = _clamp_int(limit, default=2000, minimum=1, maximum=10000)
        history = runtime.localization_service.list_history(limit=history_limit)
        trajectory = [
            to_jsonable(item.current_pose)
            for item in history
            if getattr(item, "current_pose", None) is not None
        ]
        if snapshot is None or getattr(snapshot, "current_pose", None) is None:
            service_available = runtime.localization_service.is_available()
            return _missing_envelope(
                payload_type="localization.snapshot",
                source="localization_service",
                status=DebugDataStatus.MISSING if service_available else DebugDataStatus.UNSUPPORTED,
                code="source_unavailable" if service_available else "source_unsupported",
                message="未取得平台定位缓存位姿。" if service_available else "定位服务不可用或未配置。",
                data=DebugLocalizationView(trajectory=trajectory),
            )
        view = DebugLocalizationView(
            localization_snapshot=to_jsonable(snapshot),
            pose=to_jsonable(snapshot.current_pose),
            frame_tree=to_jsonable(snapshot.frame_tree),
            trajectory=trajectory,
        )
        source_timestamp = _timestamp_from_value(snapshot.current_pose) or _timestamp_from_value(snapshot)
        return _envelope(
            payload_type="localization.snapshot",
            source="localization_service",
            data=view,
            source_timestamp=source_timestamp,
            frame_id=getattr(snapshot.current_pose, "frame_id", None),
            status=_status_for_timestamp(source_timestamp, "localization"),
        )

    @router.get("/api/debug/navigation")
    @router.get("/api/debug/exploration/frontier/latest")
    async def debug_navigation(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        navigation_context = runtime.navigation_service.get_latest_navigation_context()
        exploration_context = runtime.navigation_service.get_latest_exploration_context()
        is_frontier_request = request.url.path.endswith("/exploration/frontier/latest")
        source_available = (
            runtime.navigation_service.is_exploration_available()
            if is_frontier_request
            else runtime.navigation_service.is_navigation_available() or runtime.navigation_service.is_exploration_available()
        )
        if is_frontier_request and exploration_context is None:
            return _missing_envelope(
                payload_type="frontier.frame",
                source="navigation_service",
                status=DebugDataStatus.MISSING if source_available else DebugDataStatus.UNSUPPORTED,
                code="source_unavailable" if source_available else "source_unsupported",
                message="未取得探索缓存上下文。" if source_available else "探索服务不可用或未配置。",
            )
        if navigation_context is None and exploration_context is None:
            return _missing_envelope(
                payload_type="navigation.snapshot",
                source="navigation_service",
                status=DebugDataStatus.MISSING if source_available else DebugDataStatus.UNSUPPORTED,
                code="source_unavailable" if source_available else "source_unsupported",
                message="未取得导航或探索缓存上下文。" if source_available else "导航或探索服务不可用或未配置。",
            )
        exploration_state = getattr(exploration_context, "exploration_state", None)
        waiting_for = None
        frontier_count = None
        if exploration_state is not None:
            frontier_count = getattr(exploration_state, "frontier_count", None)
            waiting_for = dict(getattr(exploration_state, "metadata", {}) or {}).get("waiting_for")
        frontiers = _frontiers_from_exploration_context(exploration_context)
        view = DebugNavigationView(
            navigation_context=to_jsonable(navigation_context),
            exploration_context=to_jsonable(exploration_context),
            frontier_count=frontier_count if frontier_count is not None else len(frontiers),
            waiting_for=waiting_for,
            frontiers=frontiers,
        )
        source_timestamp = _latest_timestamp(_navigation_timestamp(navigation_context), _exploration_timestamp(exploration_context))
        status = _status_for_timestamp(source_timestamp, "navigation")
        payload_type = "frontier.frame" if is_frontier_request else "navigation.snapshot"
        error = None
        if payload_type == "frontier.frame" and not frontiers:
            status = DebugDataStatus.MISSING if source_available else DebugDataStatus.UNSUPPORTED
            error = DebugError(
                code="source_unavailable" if source_available else "source_unsupported",
                message="探索上下文暂未提供 frontier 几何；请确认探索运行态已生成候选。",
                details={"frontier_count": frontier_count, "waiting_for": waiting_for},
            )
        return _envelope(
            payload_type=payload_type,
            source="navigation_service",
            data=view,
            source_timestamp=source_timestamp,
            frame_id=_frame_id_from_navigation(navigation_context),
            status=status,
            error=error,
        )

    @router.get("/api/debug/pointcloud/latest")
    @router.get("/api/debug/pointcloud/frame/latest")
    @router.get("/api/debug/pointcloud/frame/latest/meta")
    async def debug_pointcloud_latest(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        pointcloud = await run_in_threadpool(lambda: _get_latest_pointcloud(runtime))
        if pointcloud is None:
            return _missing_envelope(
                payload_type="pointcloud.frame",
                source="pointcloud_provider",
                status=DebugDataStatus.MISSING,
                code="source_unavailable",
                message="未取得 provider 或 Go2 data_plane 的最新点云帧。",
                data=DebugPointCloudView(
                    source="pointcloud_provider",
                    status=DebugDataStatus.MISSING,
                    protection=_pointcloud_protection(runtime),
                ),
            )
        source = _pointcloud_source(pointcloud)
        source_timestamp = _timestamp_from_value(pointcloud)
        status = _status_for_timestamp(source_timestamp, "pointcloud")
        points = _pointcloud_points(pointcloud)
        view = DebugPointCloudView(
            frame_id=getattr(pointcloud, "frame_id", None),
            source=source,
            status=status,
            metadata=_pointcloud_metadata(pointcloud),
            point_count=int(getattr(pointcloud, "point_count", 0) or len(points)),
            fields=to_jsonable(getattr(pointcloud, "fields", []) or []),
            points=points,
            format=_pointcloud_format(pointcloud),
            payload_bytes=_estimate_pointcloud_payload_bytes(pointcloud),
            artifact_ref=getattr(pointcloud, "artifact_id", None),
            data_uri=getattr(pointcloud, "data_uri", None),
            protection=_pointcloud_protection(runtime),
            metrics={**_pointcloud_metrics(pointcloud), "inline_points": len(points)},
        )
        return _envelope(
            payload_type="pointcloud.frame",
            source=source,
            data=view,
            source_timestamp=source_timestamp,
            frame_id=getattr(pointcloud, "frame_id", None),
            sequence=_sequence_from_metadata(pointcloud),
            status=status,
        )

    @router.get("/api/debug/video/latest")
    @router.get("/api/debug/video/frame/latest")
    async def debug_video_latest(request: Request, camera_id: Optional[str] = None) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        try:
            image_frame = await run_in_threadpool(lambda: _capture_debug_video_frame(runtime, camera_id=camera_id))
        except Exception as exc:
            return _missing_envelope(
                payload_type="video.frame",
                source="image_provider",
                status=DebugDataStatus.MISSING,
                code="source_unavailable",
                message="未取得 ImageProvider 的最新视频帧：%s" % exc,
            )
        if image_frame is None:
            return _missing_envelope(
                payload_type="video.frame",
                source="image_provider",
                status=DebugDataStatus.MISSING,
                code="source_unavailable",
                message="当前机器人入口未提供图像/视频 provider。",
            )
        source_timestamp = _timestamp_from_value(image_frame)
        payload_bytes = len(getattr(image_frame, "data", None) or b"")
        view = DebugVideoView(
            camera_id=getattr(image_frame, "camera_id", None),
            frame_id=getattr(image_frame, "frame_id", None),
            width_px=getattr(image_frame, "width_px", None),
            height_px=getattr(image_frame, "height_px", None),
            encoding=str(getattr(getattr(image_frame, "encoding", None), "value", getattr(image_frame, "encoding", "")) or ""),
            data_uri=_image_data_uri(image_frame),
            uri=getattr(image_frame, "uri", None),
            artifact_ref=getattr(image_frame, "artifact_id", None),
            payload_bytes=payload_bytes,
            metadata=to_jsonable(getattr(image_frame, "metadata", {}) or {}),
            metrics={"payload_bytes": payload_bytes},
        )
        return _envelope(
            payload_type="video.frame",
            source="image_provider",
            data=view,
            source_timestamp=source_timestamp,
            frame_id=getattr(image_frame, "frame_id", None),
            status=_status_for_timestamp(source_timestamp, "video"),
        )

    @router.get("/api/debug/logs/tail")
    async def debug_logs_tail(
        request: Request,
        cursor: int = 0,
        limit: int = 200,
        level: Optional[str] = None,
        source: Optional[str] = None,
    ) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        safe_limit = _clamp_int(limit, default=200, minimum=1, maximum=1000)
        normalized_level = str(level or "").strip().lower()
        normalized_source = str(source or "").strip()
        log_path = _resolve_debug_log_path(runtime)
        if log_path is None:
            return _missing_envelope(
                payload_type="logs.tail",
                source="configured_log_file",
                status=DebugDataStatus.MISSING,
                code="source_unavailable",
                message="未找到配置日志文件。",
                data=DebugLogsTailView(source_status=DebugDataStatus.MISSING, source_message="未找到配置日志文件。"),
            )
        entries, next_cursor, _file_size = await run_in_threadpool(
            lambda: _tail_log_file(
                log_path,
                cursor=max(0, int(cursor or 0)),
                limit=safe_limit,
                level=normalized_level,
                source=normalized_source,
            )
        )
        status = DebugDataStatus.AVAILABLE if log_path.exists() else DebugDataStatus.MISSING
        message = "读取配置日志文件 %s" % log_path
        view = DebugLogsTailView(
            next_cursor=next_cursor,
            entries=entries,
            source_status=status,
            source_message=message,
        )
        source_timestamp = _log_file_timestamp(log_path)
        return _envelope(
            payload_type="logs.tail",
            source="configured_log_file",
            data=view,
            source_timestamp=source_timestamp,
            sequence=next_cursor,
            status=status,
            error=None if status == DebugDataStatus.AVAILABLE else DebugError(code="source_unavailable", message=message),
        )


    return router


def _render_debug_page(access_token: str) -> str:
    debug_access_token = json.dumps(access_token)
    html = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>nuwax_robot_bridge 本机只读调试面板</title>
  <style>
    :root { color-scheme: dark; --bg: #0f172a; --panel: #111827; --panel-soft: #172033; --border: #334155; --text: #e5e7eb; --muted: #94a3b8; --good: #22c55e; --warn: #f59e0b; --bad: #ef4444; --info: #38bdf8; --unknown: #475569; --free: #e2e8f0; --occupied: #111827; --frontier: #d946ef; }
    * { box-sizing: border-box; }
    body { margin: 0; padding: 20px; background: var(--bg); color: var(--text); font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
    header { display: flex; flex-wrap: wrap; gap: 12px; align-items: flex-end; justify-content: space-between; margin-bottom: 16px; }
    h1, h2, h3 { margin: 0; }
    h1 { font-size: 26px; }
    h2 { font-size: 18px; }
    h3 { font-size: 15px; color: #cbd5e1; }
    code { color: #93c5fd; word-break: break-all; }
    canvas { display: block; width: 100%; border-radius: 10px; background: #020617; border: 1px solid #1f2937; }
    .muted { color: var(--muted); }
    .small { font-size: 12px; }
    .readonly { color: #bfdbfe; }
    .grid { display: grid; gap: 12px; }
    .overview-grid { grid-template-columns: repeat(auto-fit, minmax(190px, 1fr)); margin-bottom: 12px; }
    .main-grid { grid-template-columns: minmax(0, 1.4fr) minmax(320px, 0.8fr); align-items: start; }
    .side-grid { grid-template-columns: 1fr; }
    .card { background: var(--panel); border: 1px solid var(--border); border-radius: 14px; padding: 14px; box-shadow: 0 14px 30px rgba(2, 6, 23, 0.18); }
    .status-card { display: grid; gap: 8px; min-height: 146px; }
    .card-head { display: flex; align-items: center; justify-content: space-between; gap: 10px; margin-bottom: 10px; }
    .pill { display: inline-flex; align-items: center; gap: 6px; border-radius: 999px; padding: 3px 8px; font-size: 12px; border: 1px solid var(--border); background: #0b1220; color: #cbd5e1; white-space: nowrap; }
    .pill::before { content: ""; width: 7px; height: 7px; border-radius: 999px; background: var(--muted); }
    .pill.available::before { background: var(--good); }
    .pill.stale::before { background: var(--warn); }
    .pill.missing::before, .pill.unsupported::before { background: var(--unknown); }
    .pill.error::before { background: var(--bad); }
    .kv { display: grid; grid-template-columns: minmax(92px, 0.45fr) minmax(0, 1fr); gap: 5px 8px; font-size: 12px; }
    .kv div:nth-child(odd) { color: var(--muted); }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace; }
    .empty, .error-text { color: #fecaca; background: rgba(127, 29, 29, 0.22); border: 1px solid rgba(248, 113, 113, 0.35); border-radius: 10px; padding: 10px; }
    .notice { color: #fde68a; background: rgba(120, 53, 15, 0.22); border: 1px solid rgba(251, 191, 36, 0.35); border-radius: 10px; padding: 10px; }
    .panel-meta { margin-top: 8px; }
    .map-wrap, .pointcloud-wrap { position: relative; }
    #mapCanvas { height: min(62vh, 620px); min-height: 360px; image-rendering: pixelated; }
    #pointCloudCanvas { height: 300px; }
    .video-frame { display: block; width: 100%; min-height: 220px; max-height: 360px; object-fit: contain; border-radius: 10px; background: #020617; border: 1px solid #1f2937; }
    .legend { display: flex; flex-wrap: wrap; gap: 10px; margin-top: 8px; }
    .legend-item { display: inline-flex; align-items: center; gap: 6px; color: var(--muted); font-size: 12px; }
    .swatch { width: 12px; height: 12px; border: 1px solid #64748b; border-radius: 3px; }
    .pre { white-space: pre-wrap; overflow: auto; max-height: 210px; padding: 10px; background: #020617; border: 1px solid #1f2937; border-radius: 10px; font-size: 12px; }
    .logs { display: grid; gap: 8px; max-height: 420px; overflow: auto; }
    .log-line { border: 1px solid #1f2937; border-radius: 10px; padding: 8px; background: #0b1220; }
    .log-line .meta { color: #93c5fd; font-size: 12px; margin-bottom: 4px; }
    .log-line.warn .meta, .log-line.warning .meta { color: #fde68a; }
    .log-line.error .meta, .log-line.critical .meta { color: #fecaca; }
    @media (max-width: 980px) { .main-grid { grid-template-columns: 1fr; } body { padding: 12px; } }
  </style>
</head>
<body>
  <header>
    <div>
      <h1>nuwax_robot_bridge 本机只读调试面板</h1>
      <p class="muted">该页面仅发起 GET 只读轮询请求，不包含运动、导航、探索或地图修改入口。</p>
    </div>
    <div class="muted small">轮询周期约 500 ms · 前端接收：<span id="lastReceive" class="mono">--</span></div>
  </header>

  <section id="overviewGrid" class="grid overview-grid" aria-label="顶部总览"></section>

  <main class="grid main-grid">
    <section class="card">
      <div class="card-head">
        <h2>2D 地图、定位与轨迹</h2>
        <span id="mapStatus" class="pill missing">missing</span>
      </div>
      <div class="map-wrap"><canvas id="mapCanvas" width="960" height="620"></canvas></div>
      <div class="legend">
        <span class="legend-item"><span class="swatch" style="background: var(--unknown)"></span>unknown</span>
        <span class="legend-item"><span class="swatch" style="background: var(--free)"></span>free</span>
        <span class="legend-item"><span class="swatch" style="background: var(--occupied)"></span>occupied</span>
        <span class="legend-item"><span class="swatch" style="background: #38bdf8"></span>轨迹</span>
        <span class="legend-item"><span class="swatch" style="background: #f97316"></span>机器人</span>
        <span class="legend-item"><span class="swatch" style="background: var(--frontier)"></span>frontier</span>
      </div>
      <div id="mapEmpty" class="empty panel-meta">等待地图数据。</div>
      <div id="mapMeta" class="kv panel-meta"></div>
      <div class="card-head" style="margin-top: 14px;">
        <h3>定位/轨迹</h3>
        <span id="localizationStatus" class="pill missing">missing</span>
      </div>
      <div id="localizationMeta" class="kv"></div>
    </section>

    <div class="grid side-grid">
      <section class="card">
        <div class="card-head">
          <h2>navigation / frontier</h2>
          <span id="navigationStatus" class="pill missing">missing</span>
        </div>
        <div id="navigationMeta" class="kv"></div>
        <div id="frontierNotice" class="notice panel-meta">等待 frontier 候选几何。</div>
      </section>

      <section class="card">
        <div class="card-head">
          <h2>3D 点云</h2>
          <span id="pointcloudStatus" class="pill missing">missing</span>
        </div>
        <div class="pointcloud-wrap"><canvas id="pointCloudCanvas" width="620" height="300"></canvas></div>
        <div id="pointcloudEmpty" class="empty panel-meta">等待点云数据。</div>
        <div id="pointcloudMeta" class="kv panel-meta"></div>
      </section>

      <section class="card">
        <div class="card-head">
          <h2>实时视频</h2>
          <span id="videoStatus" class="pill missing">missing</span>
        </div>
        <img id="videoFrame" class="video-frame" alt="实时视频帧" hidden>
        <div id="videoEmpty" class="empty panel-meta">等待视频帧。</div>
        <div id="videoMeta" class="kv panel-meta"></div>
      </section>

      <section class="card">
        <div class="card-head">
          <h2>真实日志 tail</h2>
          <span id="logsStatus" class="pill missing">missing</span>
        </div>
        <div id="logsMeta" class="kv"></div>
        <div id="logs" class="logs panel-meta"></div>
      </section>
    </div>
  </main>

  <script>
    (() => {
      const POLL_MS = 500;
      const DEBUG_ACCESS_TOKEN = __DEBUG_ACCESS_TOKEN__;
      const urlAccessToken = new URLSearchParams(window.location.search).get("access_token") || "";
      const apiAccessToken = urlAccessToken || DEBUG_ACCESS_TOKEN;
      const overviewNames = ["health", "state", "localization", "map", "navigation", "exploration", "pointcloud", "video", "logs"];
      const overviewLabels = { health: "health", state: "state", localization: "localization", map: "map", navigation: "navigation", exploration: "exploration", pointcloud: "pointcloud", video: "video", logs: "logs" };
      const state = { mapGrid: null, mapViewport: null, pose: null, trajectory: [], frontiers: [], logs: [], logCursor: 0, polling: false, videoFrames: 0, pointCloudFrames: 0 };
      const els = {
        lastReceive: document.getElementById("lastReceive"),
        overviewGrid: document.getElementById("overviewGrid"),
        mapCanvas: document.getElementById("mapCanvas"),
        mapStatus: document.getElementById("mapStatus"),
        mapEmpty: document.getElementById("mapEmpty"),
        mapMeta: document.getElementById("mapMeta"),
        localizationStatus: document.getElementById("localizationStatus"),
        localizationMeta: document.getElementById("localizationMeta"),
        navigationStatus: document.getElementById("navigationStatus"),
        navigationMeta: document.getElementById("navigationMeta"),
        frontierNotice: document.getElementById("frontierNotice"),
        pointcloudCanvas: document.getElementById("pointCloudCanvas"),
        pointcloudStatus: document.getElementById("pointcloudStatus"),
        pointcloudEmpty: document.getElementById("pointcloudEmpty"),
        pointcloudMeta: document.getElementById("pointcloudMeta"),
        videoStatus: document.getElementById("videoStatus"),
        videoFrame: document.getElementById("videoFrame"),
        videoEmpty: document.getElementById("videoEmpty"),
        videoMeta: document.getElementById("videoMeta"),
        logsStatus: document.getElementById("logsStatus"),
        logsMeta: document.getElementById("logsMeta"),
        logs: document.getElementById("logs"),
      };

      function nowText() { return new Date().toLocaleTimeString(); }
      function shortTime(value) { return value ? new Date(value).toLocaleTimeString() : "--"; }
      function fmt(value, suffix) {
        if (value === null || value === undefined || value === "") return "--";
        if (typeof value === "number") return Number.isFinite(value) ? `${Math.round(value * 10) / 10}${suffix || ""}` : "--";
        return `${value}${suffix || ""}`;
      }
      function safeText(value) { return value === null || value === undefined || value === "" ? "--" : String(value); }
      function statusOf(envelope) { return envelope && envelope.status ? String(envelope.status) : "missing"; }
      function statusClass(status) { return ["available", "stale", "missing", "error", "unsupported"].includes(status) ? status : "missing"; }
      function setPill(el, status, label) {
        const value = statusClass(status || "missing");
        el.className = `pill ${value}`;
        el.textContent = label ? `${label}: ${value}` : value;
      }
      function setKv(el, rows) {
        el.replaceChildren();
        for (const [key, value] of rows) {
          const k = document.createElement("div");
          const v = document.createElement("div");
          v.className = "mono";
          k.textContent = key;
          v.textContent = safeText(value);
          el.append(k, v);
        }
      }
      function errorReason(envelope, fallback) {
        const err = envelope && envelope.error;
        return (err && (err.message || err.code)) || fallback || "数据不可用。";
      }
      function withAccessToken(path) {
        if (!apiAccessToken) return path;
        const url = new URL(path, window.location.origin);
        url.searchParams.set("access_token", apiAccessToken);
        return `${url.pathname}${url.search}`;
      }
      async function getJson(path) {
        const started = performance.now();
        const requestPath = withAccessToken(path);
        const res = await fetch(requestPath, { cache: "no-store" });
        const receivedAt = new Date();
        if (!res.ok) throw new Error(`${path} HTTP ${res.status}`);
        const json = await res.json();
        json.__receivedAt = receivedAt.toISOString();
        json.__frontendLatencyMs = performance.now() - started;
        return json;
      }
      function buildOverview() {
        els.overviewGrid.replaceChildren();
        for (const name of overviewNames) {
          const section = document.createElement("section");
          section.className = "card status-card";
          section.id = `overview-${name}`;
          section.innerHTML = `<div class="card-head"><h3>${overviewLabels[name]}</h3><span class="pill missing">missing</span></div><div class="kv"></div><div class="muted small"></div>`;
          els.overviewGrid.appendChild(section);
        }
      }
      function updateOverview(envelope) {
        const data = envelope && envelope.data ? envelope.data : {};
        for (const name of overviewNames) {
          const summary = data[name] || {};
          const section = document.getElementById(`overview-${name}`);
          if (!section) continue;
          setPill(section.querySelector(".pill"), summary.status || "missing");
          setKv(section.querySelector(".kv"), [
            ["source_timestamp", shortTime(summary.source_timestamp)],
            ["latency", fmt(summary.latency_ms, " ms")],
            ["stale", summary.stale === true ? "true" : "false"],
            ["source", summary.metadata && summary.metadata.source],
          ]);
          section.querySelector(".small").textContent = `前端接收 ${shortTime(envelope.__receivedAt)}`;
        }
      }
      function showPanelError(kind, message) {
        if (kind === "map") {
          setPill(els.mapStatus, "error");
          els.mapEmpty.hidden = false;
          els.mapEmpty.textContent = message;
        } else if (kind === "localization") {
          setPill(els.localizationStatus, "error");
          setKv(els.localizationMeta, [["error", message], ["前端接收", nowText()]]);
        } else if (kind === "navigation") {
          setPill(els.navigationStatus, "error");
          setKv(els.navigationMeta, [["error", message], ["前端接收", nowText()]]);
        } else if (kind === "pointcloud") {
          setPill(els.pointcloudStatus, "error");
          els.pointcloudEmpty.hidden = false;
          els.pointcloudEmpty.textContent = message;
        } else if (kind === "video") {
          setPill(els.videoStatus, "error");
          els.videoEmpty.hidden = false;
          els.videoEmpty.textContent = message;
        } else if (kind === "logs") {
          setPill(els.logsStatus, "error");
          setKv(els.logsMeta, [["error", message], ["前端接收", nowText()]]);
        }
      }
      function originPosition(grid) { return grid && grid.origin && grid.origin.position ? grid.origin.position : { x: 0, y: 0 }; }
      function originYaw(grid) { return yawFromPose(grid && grid.origin) || 0; }
      function mapToCanvas(point) {
        const grid = state.mapGrid;
        const viewport = state.mapViewport;
        if (!grid || !viewport || !point) return null;
        const origin = originPosition(grid);
        const resolution = Number(grid.resolution_m) || 1;
        const dx = Number(point.x) - Number(origin.x || 0);
        const dy = Number(point.y) - Number(origin.y || 0);
        const yaw = originYaw(grid);
        const cos = Math.cos(-yaw);
        const sin = Math.sin(-yaw);
        const localX = dx * cos - dy * sin;
        const localY = dx * sin + dy * cos;
        const col = localX / resolution;
        const row = localY / resolution;
        return { x: viewport.x + col * viewport.scale, y: viewport.y + (Number(grid.height) - 1 - row) * viewport.scale };
      }
      function pointFromPose(pose) {
        const pos = pose && pose.position;
        if (!pos) return null;
        return { x: Number(pos.x), y: Number(pos.y), z: Number(pos.z || 0) };
      }
      function yawFromPose(pose) {
        const q = pose && pose.orientation;
        if (!q) return null;
        const siny = 2 * ((Number(q.w) || 0) * (Number(q.z) || 0) + (Number(q.x) || 0) * (Number(q.y) || 0));
        const cosy = 1 - 2 * ((Number(q.y) || 0) * (Number(q.y) || 0) + (Number(q.z) || 0) * (Number(q.z) || 0));
        return Math.atan2(siny, cosy);
      }
      function drawMap() {
        const canvas = els.mapCanvas;
        const ctx = canvas.getContext("2d");
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.fillStyle = "#020617";
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        const grid = state.mapGrid;
        if (!grid || !Array.isArray(grid.data) || !grid.width || !grid.height) {
          state.mapViewport = null;
          return;
        }
        const width = Number(grid.width);
        const height = Number(grid.height);
        const scale = Math.max(1, Math.floor(Math.min(canvas.width / width, canvas.height / height)));
        const drawWidth = width * scale;
        const drawHeight = height * scale;
        const offsetX = Math.floor((canvas.width - drawWidth) / 2);
        const offsetY = Math.floor((canvas.height - drawHeight) / 2);
        state.mapViewport = { x: offsetX, y: offsetY, scale };
        const image = ctx.createImageData(width, height);
        for (let row = 0; row < height; row += 1) {
          for (let col = 0; col < width; col += 1) {
            const sourceIndex = row * width + col;
            const targetIndex = ((height - 1 - row) * width + col) * 4;
            const value = Number(grid.data[sourceIndex]);
            let color = [71, 85, 105];
            if (value >= 65) color = [17, 24, 39];
            else if (value >= 0) color = [226, 232, 240];
            image.data[targetIndex] = color[0];
            image.data[targetIndex + 1] = color[1];
            image.data[targetIndex + 2] = color[2];
            image.data[targetIndex + 3] = 255;
          }
        }
        const offscreen = document.createElement("canvas");
        offscreen.width = width;
        offscreen.height = height;
        offscreen.getContext("2d").putImageData(image, 0, 0);
        ctx.imageSmoothingEnabled = false;
        ctx.drawImage(offscreen, offsetX, offsetY, drawWidth, drawHeight);
        drawTrajectory(ctx);
        drawFrontiers(ctx);
        drawRobot(ctx);
      }
      function pointFromFrontier(frontier) {
        const geometry = frontier && frontier.geometry ? frontier.geometry : frontier;
        if (!geometry) return null;
        if (Array.isArray(geometry.coordinates) && geometry.coordinates.length >= 2) return { x: Number(geometry.coordinates[0]), y: Number(geometry.coordinates[1]), z: Number(geometry.coordinates[2] || 0) };
        if (geometry.position) return pointFromPose(geometry);
        if (geometry.pose) return pointFromPose(geometry.pose);
        if (geometry.centroid) return pointFromFrontier(geometry.centroid);
        if (geometry.x !== undefined && geometry.y !== undefined) return { x: Number(geometry.x), y: Number(geometry.y), z: Number(geometry.z || 0) };
        return null;
      }
      function drawFrontiers(ctx) {
        if (!state.frontiers || !state.frontiers.length) return;
        ctx.save();
        ctx.fillStyle = "#d946ef";
        ctx.strokeStyle = "#f5d0fe";
        ctx.lineWidth = 1;
        for (const frontier of state.frontiers) {
          const point = mapToCanvas(pointFromFrontier(frontier));
          if (!point) continue;
          ctx.beginPath();
          ctx.arc(point.x, point.y, 5, 0, Math.PI * 2);
          ctx.fill();
          ctx.stroke();
        }
        ctx.restore();
      }
      function drawTrajectory(ctx) {
        if (!state.trajectory || state.trajectory.length < 2) return;
        ctx.save();
        ctx.strokeStyle = "#38bdf8";
        ctx.lineWidth = 2;
        ctx.beginPath();
        let started = false;
        for (const pose of state.trajectory) {
          const point = mapToCanvas(pointFromPose(pose));
          if (!point) continue;
          if (!started) { ctx.moveTo(point.x, point.y); started = true; }
          else ctx.lineTo(point.x, point.y);
        }
        if (started) ctx.stroke();
        ctx.restore();
      }
      function drawRobot(ctx) {
        const point = mapToCanvas(pointFromPose(state.pose));
        if (!point) return;
        const yaw = yawFromPose(state.pose) || 0;
        ctx.save();
        ctx.translate(point.x, point.y);
        ctx.rotate(-yaw);
        ctx.fillStyle = "#f97316";
        ctx.strokeStyle = "#fed7aa";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(10, 0);
        ctx.lineTo(-7, -6);
        ctx.lineTo(-4, 0);
        ctx.lineTo(-7, 6);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        ctx.restore();
      }
      function updateMap(envelope) {
        const status = statusOf(envelope);
        const data = envelope.data || {};
        const grid = data.occupancy_grid;
        setPill(els.mapStatus, status);
        if (grid && Array.isArray(grid.data)) {
          state.mapGrid = grid;
          els.mapEmpty.hidden = status === "available";
          els.mapEmpty.textContent = status === "stale" ? "地图数据已过期，保留最后一帧。" : "";
          drawMap();
        } else {
          state.mapGrid = null;
          drawMap();
          els.mapEmpty.hidden = false;
          els.mapEmpty.textContent = errorReason(envelope, "无 occupancy_grid，占据栅格不可用。");
        }
        setKv(els.mapMeta, [
          ["frame", envelope.frame_id || (grid && grid.frame_id)],
          ["size", grid ? `${grid.width} x ${grid.height}` : "--"],
          ["resolution", grid ? `${grid.resolution_m} m` : "--"],
          ["source_timestamp", shortTime(envelope.source_timestamp)],
          ["latency", fmt(envelope.latency_ms, " ms")],
          ["stale", envelope.stale === true ? "true" : "false"],
          ["前端接收", shortTime(envelope.__receivedAt)],
        ]);
      }
      function updateLocalization(envelope) {
        const status = statusOf(envelope);
        const data = envelope.data || {};
        state.pose = data.pose || null;
        state.trajectory = Array.isArray(data.trajectory) ? data.trajectory : [];
        setPill(els.localizationStatus, status);
        const p = pointFromPose(state.pose);
        const yaw = yawFromPose(state.pose);
        setKv(els.localizationMeta, [
          ["frame", envelope.frame_id || (state.pose && state.pose.frame_id)],
          ["pose", p ? `x=${fmt(p.x)} y=${fmt(p.y)} z=${fmt(p.z)}` : errorReason(envelope, "--")],
          ["yaw", yaw === null ? "--" : fmt(yaw, " rad")],
          ["trajectory_count", state.trajectory.length],
          ["source_timestamp", shortTime(envelope.source_timestamp)],
          ["latency", fmt(envelope.latency_ms, " ms")],
          ["stale", envelope.stale === true ? "true" : "false"],
          ["前端接收", shortTime(envelope.__receivedAt)],
        ]);
        drawMap();
      }
      function summarizeContext(context, key) {
        const stateObj = context && context[key];
        if (!stateObj) return "--";
        return stateObj.status || stateObj.phase || stateObj.message || stateObj.state || "available";
      }
      function updateNavigation(envelope) {
        const status = statusOf(envelope);
        const data = envelope.data || {};
        setPill(els.navigationStatus, status);
        const frontierCount = data.frontier_count;
        const waitingFor = data.waiting_for;
        const frontiers = Array.isArray(data.frontiers) ? data.frontiers : null;
        state.frontiers = frontiers || [];
        setKv(els.navigationMeta, [
          ["navigation", summarizeContext(data.navigation_context, "navigation_state")],
          ["exploration", summarizeContext(data.exploration_context, "exploration_state")],
          ["frontier_count", frontierCount === null || frontierCount === undefined ? "--" : frontierCount],
          ["waiting_for", waitingFor],
          ["frontier_geometry", frontiers && frontiers.length ? `${frontiers.length} 个几何` : "暂无候选"],
          ["source_timestamp", shortTime(envelope.source_timestamp)],
          ["latency", fmt(envelope.latency_ms, " ms")],
          ["stale", envelope.stale === true ? "true" : "false"],
          ["前端接收", shortTime(envelope.__receivedAt)],
        ]);
        els.frontierNotice.hidden = Boolean(frontiers && frontiers.length);
        els.frontierNotice.textContent = frontiers && frontiers.length ? "" : "当前探索上下文还没有 frontier 几何，请确认探索运行态已生成候选。";
        drawMap();
      }
      function pointFromCloud(point) {
        if (Array.isArray(point)) return { x: Number(point[0]), y: Number(point[1]), z: Number(point[2] || 0) };
        if (point && point.position) return pointFromCloud(point.position);
        if (point && point.x !== undefined && point.y !== undefined) return { x: Number(point.x), y: Number(point.y), z: Number(point.z || 0) };
        return null;
      }
      function drawPointCloud(points) {
        const canvas = els.pointcloudCanvas;
        const ctx = canvas.getContext("2d");
        const started = performance.now();
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.fillStyle = "#020617";
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        if (!points.length) {
          ctx.fillStyle = "#94a3b8";
          ctx.font = "14px system-ui";
          ctx.fillText("等待真实点云 points。", 16, 36);
          return performance.now() - started;
        }
        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity, minZ = Infinity, maxZ = -Infinity;
        const parsed = [];
        for (const raw of points) {
          const p = pointFromCloud(raw);
          if (!p || !Number.isFinite(p.x) || !Number.isFinite(p.y) || !Number.isFinite(p.z)) continue;
          parsed.push(p);
          minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
          minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
          minZ = Math.min(minZ, p.z); maxZ = Math.max(maxZ, p.z);
        }
        const rangeX = Math.max(0.01, maxX - minX);
        const rangeY = Math.max(0.01, maxY - minY);
        const rangeZ = Math.max(0.01, maxZ - minZ);
        const scale = Math.min((canvas.width - 28) / rangeX, (canvas.height - 28) / rangeY);
        const offsetX = (canvas.width - rangeX * scale) / 2;
        const offsetY = (canvas.height - rangeY * scale) / 2;
        ctx.fillStyle = "#38bdf8";
        for (const p of parsed) {
          const x = offsetX + (p.x - minX) * scale;
          const y = canvas.height - (offsetY + (p.y - minY) * scale);
          const zRatio = (p.z - minZ) / rangeZ;
          ctx.fillStyle = `hsl(${190 + zRatio * 90}, 90%, ${55 + zRatio * 20}%)`;
          ctx.fillRect(x, y, 2, 2);
        }
        ctx.strokeStyle = "#334155";
        ctx.beginPath();
        ctx.moveTo(14, canvas.height - 18); ctx.lineTo(58, canvas.height - 18);
        ctx.moveTo(14, canvas.height - 18); ctx.lineTo(14, canvas.height - 62);
        ctx.stroke();
        ctx.fillStyle = "#cbd5e1";
        ctx.font = "12px system-ui";
        ctx.fillText("X/Y 俯视投影 · 颜色表示 Z", 68, canvas.height - 16);
        return performance.now() - started;
      }
      function updatePointCloud(envelope) {
        const status = statusOf(envelope);
        const data = envelope.data || {};
        const points = Array.isArray(data.points) ? data.points : [];
        setPill(els.pointcloudStatus, status);
        const renderMs = drawPointCloud(points);
        state.pointCloudFrames += points.length ? 1 : 0;
        els.pointcloudEmpty.hidden = Boolean(points.length);
        els.pointcloudEmpty.textContent = points.length
          ? ""
          : errorReason(envelope, "点云帧可读，但没有内联 points 可绘制。");
        const metrics = data.metrics || {};
        const protection = data.protection || {};
        setKv(els.pointcloudMeta, [
          ["frame", envelope.frame_id],
          ["point_count", data.point_count ?? points.length],
          ["inline_points", points.length],
          ["rendered_frames", state.pointCloudFrames],
          ["format", data.format],
          ["payload_bytes", data.payload_bytes || metrics.payload_bytes],
          ["source_topic", metrics.source_topic],
          ["downsampled", String(metrics.downsampled === true)],
          ["browser_render_ms", fmt(renderMs, " ms")],
          ["default_max_points", String(protection.default_max_points_applied_by_debug_api === true)],
          ["source_timestamp", shortTime(envelope.source_timestamp)],
          ["latency", fmt(envelope.latency_ms, " ms")],
          ["前端接收", shortTime(envelope.__receivedAt)],
        ]);
      }
      function updateVideo(envelope) {
        const status = statusOf(envelope);
        const data = envelope.data || {};
        setPill(els.videoStatus, status);
        if (data.data_uri || data.uri) {
          els.videoFrame.src = data.data_uri || data.uri;
          els.videoFrame.hidden = false;
          els.videoEmpty.hidden = true;
          state.videoFrames += 1;
        } else {
          els.videoFrame.hidden = true;
          els.videoEmpty.hidden = false;
          els.videoEmpty.textContent = errorReason(envelope, "视频 provider 未返回图像帧。");
        }
        setKv(els.videoMeta, [
          ["camera", data.camera_id],
          ["frame", envelope.frame_id || data.frame_id],
          ["size", data.width_px && data.height_px ? `${data.width_px} x ${data.height_px}` : "--"],
          ["encoding", data.encoding],
          ["payload_bytes", data.payload_bytes],
          ["rendered_frames", state.videoFrames],
          ["source_timestamp", shortTime(envelope.source_timestamp)],
          ["latency", fmt(envelope.latency_ms, " ms")],
          ["前端接收", shortTime(envelope.__receivedAt)],
        ]);
      }
      function updateLogs(envelope) {
        const status = statusOf(envelope);
        const data = envelope.data || {};
        const entries = Array.isArray(data.entries) ? data.entries : [];
        setPill(els.logsStatus, status);
        if (entries.length) {
          for (const entry of entries) {
            if (!state.logs.some((old) => old.sequence === entry.sequence && old.timestamp === entry.timestamp)) state.logs.push(entry);
          }
          state.logs = state.logs.slice(-200);
        }
        if (typeof data.next_cursor === "number") state.logCursor = data.next_cursor;
        setKv(els.logsMeta, [
          ["entries", state.logs.length],
          ["next_cursor", state.logCursor],
          ["source_status", data.source_status || status],
          ["source_message", data.source_message || (envelope.error && envelope.error.message)],
          ["前端接收", shortTime(envelope.__receivedAt)],
        ]);
        renderLogs();
      }
      function renderLogs() {
        els.logs.replaceChildren();
        if (!state.logs.length) {
          const empty = document.createElement("div");
          empty.className = "empty";
          empty.textContent = "暂无匹配日志行。";
          els.logs.appendChild(empty);
          return;
        }
        for (const entry of state.logs.slice(-120).reverse()) {
          const line = document.createElement("article");
          line.className = `log-line ${entry.level || "info"}`;
          const meta = document.createElement("div");
          meta.className = "meta mono";
          meta.textContent = `#${entry.sequence ?? "--"} ${shortTime(entry.timestamp)} ${entry.level || "info"} ${entry.source || "runtime"}`;
          const message = document.createElement("div");
          message.textContent = entry.message || entry.event_type || "runtime.event";
          line.append(meta, message);
          els.logs.appendChild(line);
        }
      }
      async function tick() {
        if (state.polling) return;
        state.polling = true;
        try {
          const [overview, map, localization, navigation, frontier, pointcloud, video, logs] = await Promise.allSettled([
            getJson("/api/debug/overview"),
            getJson("/api/debug/map"),
            getJson("/api/debug/localization"),
            getJson("/api/debug/navigation"),
            getJson("/api/debug/exploration/frontier/latest"),
            getJson("/api/debug/pointcloud/latest"),
            getJson("/api/debug/video/latest"),
            getJson(`/api/debug/logs/tail?cursor=${encodeURIComponent(state.logCursor)}&limit=200`),
          ]);
          els.lastReceive.textContent = nowText();
          if (overview.status === "fulfilled") updateOverview(overview.value);
          if (map.status === "fulfilled") updateMap(map.value); else showPanelError("map", map.reason.message);
          if (localization.status === "fulfilled") updateLocalization(localization.value); else showPanelError("localization", localization.reason.message);
          if (navigation.status === "fulfilled") updateNavigation(navigation.value); else showPanelError("navigation", navigation.reason.message);
          if (frontier.status === "fulfilled" && statusOf(frontier.value) === "available") updateNavigation(frontier.value);
          if (pointcloud.status === "fulfilled") updatePointCloud(pointcloud.value); else showPanelError("pointcloud", pointcloud.reason.message);
          if (video.status === "fulfilled") updateVideo(video.value); else showPanelError("video", video.reason.message);
          if (logs.status === "fulfilled") updateLogs(logs.value); else showPanelError("logs", logs.reason.message);
        } finally {
          state.polling = false;
        }
      }
      buildOverview();
      renderLogs();
      tick();
      window.setInterval(tick, POLL_MS);
    })();
  </script>
</body>
</html>"""
    return html.replace("__DEBUG_ACCESS_TOKEN__", debug_access_token)


def _envelope(
    *,
    payload_type: str,
    source: str,
    data: Any = None,
    source_timestamp: Optional[datetime] = None,
    frame_id: Optional[str] = None,
    sequence: Optional[int] = None,
    revision: Optional[str] = None,
    frequency_hz: Optional[float] = None,
    status: DebugDataStatus = DebugDataStatus.AVAILABLE,
    error: Optional[DebugError] = None,
) -> Dict[str, Any]:
    gateway_timestamp = utc_now()
    effective_source_timestamp = source_timestamp or gateway_timestamp
    envelope = DebugEnvelope(
        ok=status in {DebugDataStatus.AVAILABLE, DebugDataStatus.STALE},
        type=payload_type,
        source=source,
        frame_id=frame_id,
        source_timestamp=effective_source_timestamp,
        gateway_timestamp=gateway_timestamp,
        sequence=sequence,
        revision=revision,
        frequency_hz=frequency_hz,
        latency_ms=_latency_ms(effective_source_timestamp, gateway_timestamp),
        stale=status == DebugDataStatus.STALE,
        status=status,
        data=to_jsonable(data),
        error=error,
    )
    return to_jsonable(envelope)


def _missing_envelope(
    *,
    payload_type: str,
    source: str,
    status: DebugDataStatus,
    code: str,
    message: str,
    data: Any = None,
) -> Dict[str, Any]:
    return _envelope(
        payload_type=payload_type,
        source=source,
        data=data,
        status=status,
        error=DebugError(code=code, message=message),
    )


def _summary_from_payload(
    *,
    name: str,
    source: str,
    timestamp: Optional[datetime],
    available: bool,
    frame_id: Optional[str] = None,
    revision: Optional[str] = None,
    sequence: Optional[int] = None,
    message: Optional[str] = None,
    metadata: Optional[Dict[str, Any]] = None,
) -> DebugSourceSummary:
    gateway_timestamp = utc_now()
    status = DebugDataStatus.AVAILABLE if available else DebugDataStatus.MISSING
    stale = False
    latency = None
    if available and timestamp is not None:
        stale = _is_stale(timestamp, name, gateway_timestamp)
        if stale:
            status = DebugDataStatus.STALE
        latency = _latency_ms(timestamp, gateway_timestamp)
    return DebugSourceSummary(
        name=name,
        status=status,
        frame_id=frame_id,
        source_timestamp=timestamp,
        gateway_timestamp=gateway_timestamp,
        latency_ms=latency,
        stale=stale,
        revision=revision,
        sequence=sequence,
        message=message,
        metadata={"source": source, **dict(metadata or {})},
    )


def _frontiers_from_exploration_context(context: Any) -> List[Dict[str, Any]]:
    state = getattr(context, "exploration_state", None) if context is not None else None
    metadata = getattr(state, "metadata", {}) if state is not None else {}
    if not isinstance(metadata, dict):
        return []
    frontiers = metadata.get("latest_frontiers") or []
    current = metadata.get("current_frontier")
    if current:
        frontiers = [current, *list(frontiers)]
    normalized: List[Dict[str, Any]] = []
    seen = set()
    for item in frontiers:
        if not isinstance(item, dict):
            continue
        pose = item.get("pose") if isinstance(item.get("pose"), dict) else None
        geometry = item.get("geometry") if isinstance(item.get("geometry"), dict) else None
        if geometry is None and pose is not None:
            position = pose.get("position") if isinstance(pose.get("position"), dict) else {}
            geometry = {
                "type": "point",
                "coordinates": [position.get("x", 0), position.get("y", 0), position.get("z", 0)],
            }
        if geometry is None:
            continue
        key = json.dumps(geometry, sort_keys=True, default=str)
        if key in seen:
            continue
        seen.add(key)
        normalized.append({**item, "geometry": geometry})
    return to_jsonable(normalized)


def _get_image_provider(runtime: GatewayRuntime) -> Optional[ImageProvider]:
    robot = getattr(runtime, "robot", None)
    providers = getattr(robot, "providers", None) if robot is not None else None
    if providers is not None and isinstance(providers, ImageProvider):
        availability = getattr(providers, "is_available", None)
        if not callable(availability) or bool(availability()):
            return providers
    return None


def _capture_debug_video_frame(runtime: GatewayRuntime, *, camera_id: Optional[str] = None) -> Any:
    provider = _get_image_provider(runtime)
    if provider is None:
        return None
    return provider.capture_image(camera_id)


def _get_video_summary(runtime: GatewayRuntime) -> Dict[str, Any]:
    provider = _get_image_provider(runtime)
    if provider is None:
        return {"available": False, "source": "image_provider", "message": "当前机器人入口未提供图像/视频 provider。"}
    try:
        camera_info = provider.get_camera_info(None)
    except Exception as exc:
        return {"available": False, "source": "image_provider", "message": str(exc)}
    return {
        "available": camera_info is not None,
        "source": "image_provider",
        "timestamp": _timestamp_from_value(camera_info),
        "frame_id": getattr(camera_info, "frame_id", None),
        "message": "camera_info available" if camera_info is not None else "camera_info unavailable",
        "metadata": {
            "camera_id": getattr(camera_info, "camera_id", None),
            "width_px": getattr(camera_info, "width_px", None),
            "height_px": getattr(camera_info, "height_px", None),
        },
    }


def _image_data_uri(image_frame: Any) -> Optional[str]:
    data = getattr(image_frame, "data", None)
    if not data:
        return None
    encoding = str(getattr(getattr(image_frame, "encoding", None), "value", getattr(image_frame, "encoding", "jpeg")) or "jpeg").lower()
    mime = {
        "jpeg": "image/jpeg",
        "jpg": "image/jpeg",
        "png": "image/png",
    }.get(encoding, "application/octet-stream")
    return "data:%s;base64,%s" % (mime, base64.b64encode(bytes(data)).decode("ascii"))


def _pointcloud_points(pointcloud: Any) -> List[Dict[str, float]]:
    raw_points = getattr(pointcloud, "points", []) or []
    points: List[Dict[str, float]] = []
    for point in raw_points:
        if isinstance(point, dict):
            x, y, z = point.get("x"), point.get("y"), point.get("z", 0.0)
        elif isinstance(point, (list, tuple)):
            if len(point) < 2:
                continue
            x, y, z = point[0], point[1], point[2] if len(point) > 2 else 0.0
        else:
            x, y, z = getattr(point, "x", None), getattr(point, "y", None), getattr(point, "z", 0.0)
        try:
            fx, fy, fz = float(x), float(y), float(z)
        except (TypeError, ValueError):
            continue
        if math.isfinite(fx) and math.isfinite(fy) and math.isfinite(fz):
            points.append({"x": fx, "y": fy, "z": fz})
    return points


def _resolve_debug_log_path(runtime: GatewayRuntime) -> Optional[Path]:
    configured = Path(str(runtime.config.logging.log_file)).expanduser().resolve(strict=False)
    candidates = [configured]
    tts_path = str(getattr(runtime.config.log_tts, "log_path", "") or "").strip()
    if tts_path:
        tts_candidate = Path(tts_path).expanduser().resolve(strict=False)
        if tts_candidate != configured:
            candidates.append(tts_candidate)
    for candidate in candidates:
        if candidate.exists() and candidate.is_file():
            return candidate
    return configured if configured.parent.exists() else None


def _get_log_file_summary(runtime: GatewayRuntime) -> Dict[str, Any]:
    path = _resolve_debug_log_path(runtime)
    if path is None or not path.exists():
        return {"available": False, "source": "configured_log_file", "message": "配置日志文件不存在。"}
    stat = path.stat()
    return {
        "available": True,
        "source": "configured_log_file",
        "timestamp": datetime.fromtimestamp(stat.st_mtime, tz=timezone.utc),
        "sequence": stat.st_size,
        "message": "log_file=%s" % path,
        "metadata": {"path": str(path), "size_bytes": stat.st_size},
    }


def _log_file_timestamp(path: Path) -> datetime:
    try:
        return datetime.fromtimestamp(path.stat().st_mtime, tz=timezone.utc)
    except OSError:
        return utc_now()


def _parse_log_line(raw_line: str, *, sequence: int) -> DebugLogEntry:
    line = raw_line.rstrip("\r\n")
    match = _LOG_LINE_RE.match(line)
    if match:
        timestamp_text = match.group("timestamp")
        timestamp = _parse_log_timestamp(timestamp_text)
        return DebugLogEntry(
            sequence=sequence,
            timestamp=timestamp,
            level=match.group("level").lower(),
            source=match.group("source"),
            message=match.group("message"),
        )
    return DebugLogEntry(sequence=sequence, timestamp=utc_now(), level="info", source="log_file", message=line)


def _parse_log_timestamp(value: str) -> datetime:
    normalized = value.replace(",", ".")
    for fmt in ("%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S"):
        try:
            return datetime.strptime(normalized, fmt).replace(tzinfo=timezone.utc)
        except ValueError:
            continue
    return utc_now()


def _tail_log_file(path: Path, *, cursor: int, limit: int, level: str, source: str) -> Tuple[List[DebugLogEntry], int, int]:
    if not path.exists() or not path.is_file():
        return [], 0, 0
    file_size = path.stat().st_size
    if cursor <= 0 and file_size > 262144:
        start = max(0, file_size - 262144)
    else:
        start = cursor if 0 <= cursor <= file_size else 0
    entries: List[DebugLogEntry] = []
    with path.open("rb") as stream:
        stream.seek(start)
        if start > 0:
            stream.readline()
        while True:
            raw = stream.readline()
            if not raw:
                break
            line_end = stream.tell()
            text = raw.decode("utf-8", errors="replace")
            entry = _parse_log_line(text, sequence=line_end)
            if level and entry.level.lower() != level:
                continue
            if source and entry.source != source:
                continue
            entries.append(entry)
            if len(entries) > limit:
                entries = entries[-limit:]
    return entries, file_size, file_size


def _status_for_timestamp(timestamp: Optional[datetime], name: str) -> DebugDataStatus:
    if timestamp is None:
        return DebugDataStatus.MISSING
    if _is_stale(timestamp, name, utc_now()):
        return DebugDataStatus.STALE
    return DebugDataStatus.AVAILABLE


def _timestamp_from_value(value: Any) -> Optional[datetime]:
    if value is None:
        return None
    for attr in ("timestamp", "updated_at"):
        candidate = getattr(value, attr, None)
        if isinstance(candidate, datetime):
            return candidate
    if isinstance(value, dict):
        for key in ("timestamp", "updated_at", "served_at"):
            candidate = value.get(key)
            if isinstance(candidate, datetime):
                return candidate
    return None


def _latest_timestamp(*timestamps: Optional[datetime]) -> Optional[datetime]:
    candidates = [timestamp for timestamp in timestamps if timestamp is not None]
    return max(candidates) if candidates else None


def _latency_ms(source_timestamp: datetime, gateway_timestamp: datetime) -> float:
    return max(0.0, (gateway_timestamp - source_timestamp).total_seconds() * 1000.0)


def _is_stale(source_timestamp: datetime, name: str, gateway_timestamp: datetime) -> bool:
    threshold = _STALE_THRESHOLDS_SEC.get(name)
    if threshold is None:
        return False
    return (gateway_timestamp - source_timestamp).total_seconds() > threshold


def _revision_from_map(snapshot: Any) -> Optional[str]:
    if snapshot is None:
        return None
    version_id = getattr(snapshot, "version_id", None)
    revision = getattr(snapshot, "revision", None)
    if version_id is None and revision is None:
        return None
    return "%s:%s" % (version_id or "map", revision or "")


def _frame_id_from_localization(snapshot: Any) -> Optional[str]:
    pose = getattr(snapshot, "current_pose", None) if snapshot is not None else None
    return getattr(pose, "frame_id", None)


def _frame_id_from_navigation(context: Any) -> Optional[str]:
    state = getattr(context, "navigation_state", None) if context is not None else None
    pose = getattr(state, "current_pose", None) if state is not None else None
    if pose is not None:
        return getattr(pose, "frame_id", None)
    goal = getattr(context, "current_goal", None) if context is not None else None
    target_pose = getattr(goal, "target_pose", None) if goal is not None else None
    return getattr(target_pose, "frame_id", None)


def _navigation_timestamp(context: Any) -> Optional[datetime]:
    state = getattr(context, "navigation_state", None) if context is not None else None
    return _timestamp_from_value(state) or _timestamp_from_value(context)


def _exploration_timestamp(context: Any) -> Optional[datetime]:
    state = getattr(context, "exploration_state", None) if context is not None else None
    return _timestamp_from_value(state) or _timestamp_from_value(context)


def _navigation_message(context: Any) -> Optional[str]:
    state = getattr(context, "navigation_state", None) if context is not None else None
    return getattr(state, "message", None) if state is not None else None


def _exploration_message(context: Any) -> Optional[str]:
    state = getattr(context, "exploration_state", None) if context is not None else None
    return getattr(state, "message", None) if state is not None else None


def _exploration_metadata(context: Any) -> Dict[str, Any]:
    state = getattr(context, "exploration_state", None) if context is not None else None
    if state is None:
        return {}
    return {
        "frontier_count": getattr(state, "frontier_count", None),
        "waiting_for": dict(getattr(state, "metadata", {}) or {}).get("waiting_for"),
    }


def _get_latest_pointcloud(runtime: GatewayRuntime) -> Any:
    candidates = []
    robot = getattr(runtime, "robot", None)
    if robot is not None:
        candidates.append(robot)
        provider_bundle = getattr(robot, "providers", None)
        if provider_bundle is not None:
            candidates.append(provider_bundle)
        data_plane = getattr(robot, "data_plane", None)
        if data_plane is not None:
            candidates.append(data_plane)
    for candidate in candidates:
        getter = getattr(candidate, "get_latest_point_cloud", None)
        if callable(getter):
            pointcloud = getter()
            if pointcloud is not None:
                return pointcloud
        local_getter = getattr(candidate, "get_latest_local_point_cloud", None)
        if callable(local_getter):
            pointcloud = local_getter()
            if pointcloud is not None:
                return pointcloud
    return None


def _pointcloud_source(pointcloud: Any) -> str:
    if pointcloud is None:
        return "pointcloud_provider"
    metadata = getattr(pointcloud, "metadata", {}) or {}
    source_topic = metadata.get("source_topic") if isinstance(metadata, dict) else None
    if source_topic:
        return str(source_topic)
    return "pointcloud_provider"


def _pointcloud_format(pointcloud: Any) -> str:
    fields = getattr(pointcloud, "fields", []) or []
    names = [str(getattr(field, "name", "")) for field in fields]
    if not names:
        return "xyz"
    return "-".join(name for name in names if name) or "xyz"


def _estimate_pointcloud_payload_bytes(pointcloud: Any) -> Optional[int]:
    if pointcloud is None:
        return None
    metadata = getattr(pointcloud, "metadata", {}) or {}
    if isinstance(metadata, dict):
        for key in ("payload_bytes", "size_bytes", "byte_size"):
            value = metadata.get(key)
            if isinstance(value, int) and value >= 0:
                return value
    point_count = int(getattr(pointcloud, "point_count", 0) or 0)
    fields = getattr(pointcloud, "fields", []) or []
    field_count = max(3, len(fields))
    return point_count * field_count * 4


def _pointcloud_metrics(pointcloud: Any) -> Dict[str, Any]:
    if pointcloud is None:
        return {}
    metadata = getattr(pointcloud, "metadata", {}) or {}
    source_metadata = dict(metadata) if isinstance(metadata, dict) else {}
    return {
        "point_count": int(getattr(pointcloud, "point_count", 0) or 0),
        "field_count": len(getattr(pointcloud, "fields", []) or []),
        "payload_bytes": _estimate_pointcloud_payload_bytes(pointcloud),
        "source_topic": source_metadata.get("source_topic"),
        "downsampled": bool(source_metadata.get("downsampled", False)),
    }


def _pointcloud_metadata(pointcloud: Any) -> Dict[str, Any]:
    metadata = getattr(pointcloud, "metadata", {}) if pointcloud is not None else {}
    if not isinstance(metadata, dict):
        return {}
    return to_jsonable({key: value for key, value in metadata.items() if key not in {"points", "data"}})


def _pointcloud_protection(runtime: GatewayRuntime) -> Dict[str, Any]:
    robot = getattr(runtime, "robot", None)
    data_plane = getattr(robot, "data_plane", None) if robot is not None else None
    config = getattr(data_plane, "config", None)
    direct_dds = getattr(config, "direct_dds", None)
    max_points_per_scan = getattr(direct_dds, "max_points_per_scan", None)
    return {
        "default_max_points_applied_by_debug_api": False,
        "data_plane_max_points_per_scan": max_points_per_scan,
    }


def _sequence_from_metadata(value: Any) -> Optional[int]:
    metadata = getattr(value, "metadata", {}) if value is not None else {}
    if not isinstance(metadata, dict):
        return None
    for key in ("sequence", "seq", "cursor"):
        candidate = metadata.get(key)
        if isinstance(candidate, int):
            return candidate
        if isinstance(candidate, float) and math.isfinite(candidate):
            return int(candidate)
    return None


def _clamp_int(value: int, *, default: int, minimum: int, maximum: int) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        parsed = default
    return min(max(parsed, minimum), maximum)
