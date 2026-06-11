from __future__ import annotations

import base64
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import time
from typing import Any, Dict, List, Optional, Tuple

from fastapi import APIRouter, Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import HTMLResponse, JSONResponse, Response

from gateways.errors import as_gateway_error

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


_LOG_LINE_RE = re.compile(r"^(?P<timestamp>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}(?:,\d{1,6})?) \[(?P<level>[A-Z]+)\] \[(?P<source>[^\]]+)\] (?P<message>.*)$")

_STALE_THRESHOLDS_SEC = {
    "health": 3.0,
    "state": 3.0,
    "map": 30.0,
    "localization": 1.0,
    "trajectory": 2.0,
    "navigation": 3.0,
    "exploration": 3.0,
    "runtime_governance": 3.0,
    "pointcloud_map": 3.0,
    "pointcloud": 1.0,
    "video": 1.0,
    "logs": 10.0,
}


def _stream_migration_response(*topics: str) -> JSONResponse:
    topic_list = [topic for topic in topics if topic]
    query = ",".join(topic_list)
    ws_stream_path = "/streams/ws" + (f"?topics={query}" if query else "")
    sse_stream_path = "/streams/sse" + (f"?topics={query}" if query else "")
    return JSONResponse(
        status_code=410,
        content={
            "code": "stream_required",
            "message": "实时数据调试 HTTP 快照入口已下线，请订阅实时流。",
            "stream": ws_stream_path,
            "streams": {"websocket": ws_stream_path, "sse": sse_stream_path},
            "topics": topic_list,
        },
    )


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
    async def debug_overview(request: Request) -> JSONResponse:
        access_manager.authenticate_http(request)
        return _stream_migration_response(
            'robot.state',
            'map.frame',
            'map.heartbeat',
            'localization.pose',
            'navigation.state',
            'exploration.state',
            'runtime.governance',
            'pointcloud_map.meta',
            'pointcloud.frame',
            'video.frame',
            'video.detections',
            'logs.entry',
            'tasks.state',
        )

    @router.get("/api/debug/map")
    @router.get("/api/debug/map/frame/latest")
    async def debug_map(request: Request) -> JSONResponse:
        access_manager.authenticate_http(request)
        return _stream_migration_response('map.frame')

    @router.get("/api/debug/localization")
    @router.get("/api/debug/localization/pose/latest")
    @router.get("/api/debug/localization/trajectory")
    async def debug_localization(request: Request) -> JSONResponse:
        access_manager.authenticate_http(request)
        return _stream_migration_response('localization.pose', 'localization.frame_tree', 'localization.trajectory')

    @router.get("/api/debug/navigation")
    @router.get("/api/debug/exploration/frontier/latest")
    async def debug_navigation(request: Request) -> JSONResponse:
        access_manager.authenticate_http(request)
        return _stream_migration_response('navigation.state', 'exploration.state', 'exploration.frontier')

    @router.get("/api/debug/pointcloud-map/latest/meta")
    async def debug_pointcloud_map_meta(request: Request) -> JSONResponse:
        access_manager.authenticate_http(request)
        return _stream_migration_response('pointcloud_map.meta')

    @router.get("/api/debug/pointcloud-map/latest.bin")
    async def debug_pointcloud_map_binary(request: Request) -> JSONResponse:
        access_manager.authenticate_http(request)
        return _stream_migration_response('pointcloud_map.chunk', 'pointcloud_map.frame')

    @router.get("/api/debug/pointcloud/latest")
    @router.get("/api/debug/pointcloud/frame/latest")
    @router.get("/api/debug/pointcloud/frame/latest/meta")
    async def debug_pointcloud_latest(request: Request) -> JSONResponse:
        access_manager.authenticate_http(request)
        return _stream_migration_response('pointcloud.frame')

    @router.get("/api/debug/video/latest")
    @router.get("/api/debug/video/frame/latest")
    async def debug_video_latest(request: Request) -> JSONResponse:
        access_manager.authenticate_http(request)
        return _stream_migration_response('video.frame')

    @router.get("/api/debug/logs/tail")
    async def debug_logs_tail(request: Request) -> JSONResponse:
        access_manager.authenticate_http(request)
        return _stream_migration_response('logs.entry')

    @router.post("/api/debug/navigation/goal")
    async def debug_navigation_goal(request: Request) -> JSONResponse:
        """Debug 面板 2D Goal：直通 go2.sh 已拉起的 Nav2 action，不经 navigate_to_pose 任务链。"""

        access_manager.authenticate_http(request)
        body = await request.json()
        arguments = dict(body or {})
        try:
            result = await run_in_threadpool(
                lambda: runtime.dispatch_debug_nav2_goal(arguments, requested_by="debug_panel"),
            )
        except Exception as exc:
            raise as_gateway_error(exc) from exc
        return JSONResponse(to_jsonable(result))

    @router.post("/api/debug/map/clicked_point")
    async def debug_map_clicked_point(request: Request) -> JSONResponse:
        """Debug 面板 RViz Publish Point：发布 /clicked_point，不导航。"""

        access_manager.authenticate_http(request)
        body = await request.json()
        arguments = dict(body or {})
        try:
            result = await run_in_threadpool(
                lambda: runtime.dispatch_debug_clicked_point(arguments, requested_by="debug_panel"),
            )
        except Exception as exc:
            raise as_gateway_error(exc) from exc
        return JSONResponse(to_jsonable(result))

    @router.post("/api/debug/map/initial_pose")
    async def debug_map_initial_pose(request: Request) -> JSONResponse:
        """Debug 面板 RViz 2D Pose Estimate：发布 /initialpose，手动初始化位姿。"""

        access_manager.authenticate_http(request)
        body = await request.json()
        arguments = dict(body or {})
        try:
            result = await run_in_threadpool(
                lambda: runtime.dispatch_debug_initial_pose(arguments, requested_by="debug_panel"),
            )
        except Exception as exc:
            raise as_gateway_error(exc) from exc
        return JSONResponse(to_jsonable(result))

    return router


def _render_debug_page(access_token: str) -> str:
    debug_access_token = json.dumps(access_token)
    html = """<!doctype html>
<html lang="zh-CN" data-theme="dark">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>nuwax_robot_bridge 只读机器人监控驾驶舱</title>
  <style>
    :root {
      --bg: #f4f7fb;
      --bg-grad-a: rgba(37, 99, 235, 0.10);
      --bg-grad-b: rgba(14, 165, 233, 0.08);
      --surface: rgba(255, 255, 255, 0.88);
      --surface-strong: rgba(255, 255, 255, 0.96);
      --panel: rgba(255, 255, 255, 0.86);
      --panel-soft: rgba(248, 250, 252, 0.80);
      --panel-strong: #f8fafc;
      --border: rgba(148, 163, 184, 0.32);
      --border-soft: rgba(148, 163, 184, 0.18);
      --border-strong: rgba(37, 99, 235, 0.46);
      --text: #0f172a;
      --muted: #64748b;
      --muted-strong: #334155;
      --good: #16a34a;
      --warn: #ca8a04;
      --bad: #dc2626;
      --info: #2563eb;
      --accent: #2563eb;
      --accent-strong: #0ea5e9;
      --map-line: #0ea5e9;
      --nav-plan: #22ff66;
      --map-robot: #f59e0b;
      --unknown: #cbd5e1;
      --free: #dbeafe;
      --occupied: #172033;
      --frontier: #db2777;
      --shadow: 0 22px 70px rgba(15, 23, 42, 0.12);
      --shadow-soft: 0 10px 34px rgba(15, 23, 42, 0.08);
      --radius: 20px;
      --radius-sm: 14px;
    }
    [data-theme="dark"] {
      --bg: #07111f;
      --bg-grad-a: rgba(34, 211, 238, 0.14);
      --bg-grad-b: rgba(37, 99, 235, 0.12);
      --surface: rgba(15, 23, 42, 0.72);
      --surface-strong: rgba(15, 23, 42, 0.90);
      --panel: rgba(15, 23, 42, 0.74);
      --panel-soft: rgba(30, 41, 59, 0.58);
      --panel-strong: rgba(15, 23, 42, 0.92);
      --border: rgba(148, 163, 184, 0.18);
      --border-soft: rgba(148, 163, 184, 0.11);
      --border-strong: rgba(34, 211, 238, 0.42);
      --text: #f8fafc;
      --muted: #94a3b8;
      --muted-strong: #cbd5e1;
      --good: #4ade80;
      --warn: #facc15;
      --bad: #f87171;
      --info: #60a5fa;
      --accent: #38bdf8;
      --accent-strong: #22d3ee;
      --map-line: #38bdf8;
      --nav-plan: #22ff66;
      --map-robot: #fbbf24;
      --unknown: #334155;
      --free: #93c5fd;
      --occupied: #020617;
      --frontier: #f472b6;
      --shadow: 0 28px 90px rgba(0, 0, 0, 0.36);
      --shadow-soft: 0 16px 44px rgba(0, 0, 0, 0.24);
    }
    * { box-sizing: border-box; }
    html { scroll-behavior: smooth; }
    body {
      margin: 0;
      padding: 14px;
      min-height: 100vh;
      background:
        radial-gradient(circle at 10% 6%, var(--bg-grad-a), transparent 32rem),
        radial-gradient(circle at 88% 4%, var(--bg-grad-b), transparent 36rem),
        linear-gradient(135deg, var(--bg) 0%, var(--panel-strong) 100%);
      color: var(--text);
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", "Noto Sans SC", sans-serif;
      transition: background 0.3s ease, color 0.3s ease;
    }
    .app-shell { width: min(100%, 2260px); margin: 0 auto; display: grid; gap: 12px; }
    .topbar {
      display: grid;
      grid-template-columns: minmax(300px, 0.46fr) minmax(0, 1.54fr);
      gap: 12px;
      align-items: stretch;
    }
    .brand-block, .global-status, .panel, .card, .side-nav {
      position: relative;
      overflow: hidden;
      background: linear-gradient(180deg, var(--panel), var(--panel-soft));
      border: 1px solid var(--border);
      border-radius: var(--radius);
      box-shadow: var(--shadow-soft);
      backdrop-filter: blur(18px);
      transition: background 0.3s ease, border-color 0.3s ease;
    }
    .brand-block::before, .global-status::before, .panel::before, .card::before, .side-nav::before {
      content: "";
      position: absolute;
      inset: 0 0 auto;
      height: 1px;
      background: linear-gradient(90deg, transparent, var(--accent-strong), transparent);
      opacity: 0.58;
      pointer-events: none;
    }
    .brand-block { padding: 14px 16px; display: grid; align-content: center; gap: 6px; }
    .brand-row { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }
    h1, h2, h3 { margin: 0; }
    h1 { font-size: clamp(22px, 1.55vw, 32px); line-height: 1.05; font-weight: 780; letter-spacing: -0.04em; }
    h2 { font-size: 15px; line-height: 1.25; font-weight: 720; letter-spacing: -0.01em; }
    h3 { font-size: 13px; line-height: 1.35; color: var(--muted-strong); font-weight: 700; }
    code { color: var(--accent-strong); word-break: break-all; }
    canvas { display: block; width: 100%; border-radius: var(--radius-sm); background: var(--occupied); border: 1px solid var(--border-soft); box-shadow: inset 0 0 0 1px rgba(255, 255, 255, 0.04); }
    .muted { color: var(--muted); }
    .small { font-size: 12px; line-height: 1.45; }
    .eyebrow { color: var(--accent-strong); font-size: 11px; font-weight: 800; letter-spacing: 0.14em; text-transform: uppercase; }
    .readonly { color: var(--info); }
    .grid { display: grid; gap: 12px; min-width: 0; }
    .mission-layout { display: grid; gap: 12px; min-width: 0; }
    .side-nav {
      display: flex;
      gap: 6px;
      align-items: center;
      overflow-x: auto;
      padding: 8px;
      border-radius: 16px;
      scrollbar-width: thin;
    }
    .side-nav a {
      color: var(--muted-strong);
      text-decoration: none;
      border: 1px solid transparent;
      border-radius: 999px;
      padding: 8px 11px;
      font-size: 12px;
      font-weight: 700;
      white-space: nowrap;
      transition: border-color 0.16s ease, background 0.16s ease, color 0.16s ease;
    }
    .side-nav a:hover { border-color: var(--border-strong); background: rgba(37, 99, 235, 0.09); color: var(--text); }
    [data-theme="dark"] .side-nav a:hover { background: rgba(34, 211, 238, 0.10); }
    .mission-content { display: grid; gap: 12px; min-width: 0; }
    .section-band { scroll-margin-top: 16px; display: grid; gap: 12px; min-width: 0; }
    .section-title { display: flex; justify-content: space-between; align-items: center; gap: 10px; padding: 0 3px; }
    .overview-grid { grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); align-items: stretch; }
    #overview .overview-grid { max-height: min(32vh, 320px); overflow: auto; padding-right: 2px; }
    #overview .status-card { min-height: 104px; }
    .mission-grid { grid-template-columns: minmax(230px, 0.88fr) minmax(260px, 1fr) minmax(260px, 1fr); }
    .workspace {
      grid-template-columns: minmax(0, 1.95fr) minmax(330px, 0.72fr);
      align-items: stretch;
      gap: 12px;
      min-height: calc(100vh - 166px);
    }
    .right-rail { grid-template-columns: 1fr; gap: 12px; max-height: calc(100vh - 154px); overflow: auto; padding-right: 2px; }
    .diagnostics-grid { grid-template-columns: minmax(340px, 0.8fr) minmax(360px, 0.78fr) minmax(420px, 1.08fr); align-items: start; }
    .card, .panel { padding: 13px; }
    .panel.compact { padding: 12px; }
    .panel.map-panel { padding: 12px; box-shadow: var(--shadow); display: flex; flex-direction: column; min-height: calc(100vh - 166px); }
    .status-card { display: grid; gap: 8px; min-height: 136px; align-content: start; }
    .status-card .card-head { align-items: flex-start; margin-bottom: 4px; }
    .status-card .kv { grid-template-columns: minmax(92px, 0.36fr) minmax(0, 1fr); }
    .status-card .kv div { min-width: 0; }
    .status-card .mono { white-space: normal; word-break: normal; overflow-wrap: break-word; }
    .card-head { display: flex; align-items: center; justify-content: space-between; gap: 10px; margin-bottom: 10px; }
    .global-status { display: grid; grid-template-columns: repeat(8, minmax(108px, 1fr)) auto; gap: 8px; align-items: stretch; padding: 10px; }
    .metric-strip { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; justify-content: flex-end; }
    .metric {
      border: 1px solid var(--border-soft);
      border-radius: 14px;
      padding: 8px 10px;
      background: var(--panel-strong);
      color: var(--muted-strong);
      min-width: 0;
      box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.04);
      transition: background 0.3s ease, border-color 0.3s ease;
    }
    .metric b { display: block; font-size: 14px; color: var(--text); font-variant-numeric: tabular-nums; letter-spacing: -0.01em; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
    .metric span { display: block; color: var(--muted); font-size: 10.5px; margin-top: 3px; }
    .pill {
      display: inline-flex;
      align-items: center;
      gap: 7px;
      border-radius: 999px;
      padding: 5px 9px;
      font-size: 11.5px;
      font-weight: 760;
      border: 1px solid var(--border);
      background: var(--panel-strong);
      color: var(--muted-strong);
      white-space: nowrap;
      box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.04);
      transition: background 0.3s ease, border-color 0.3s ease, color 0.3s ease;
    }
    .pill::before { content: ""; width: 7px; height: 7px; border-radius: 999px; background: var(--muted); box-shadow: 0 0 0 3px rgba(148, 163, 184, 0.12); }
    .pill.available { border-color: rgba(22, 163, 74, 0.45); background: rgba(22, 163, 74, 0.10); color: #15803d; }
    [data-theme="dark"] .pill.available { border-color: rgba(74, 222, 128, 0.45); background: rgba(22, 101, 52, 0.24); color: #dcfce7; }
    .pill.available::before { background: var(--good); box-shadow: 0 0 12px rgba(22, 163, 74, 0.45); }
    [data-theme="dark"] .pill.available::before { box-shadow: 0 0 16px rgba(74, 222, 128, 0.55); }
    .pill.stale { border-color: rgba(202, 138, 4, 0.45); background: rgba(202, 138, 4, 0.10); color: #a16207; }
    [data-theme="dark"] .pill.stale { border-color: rgba(250, 204, 21, 0.45); background: rgba(113, 63, 18, 0.26); color: #fef3c7; }
    .pill.stale::before { background: var(--warn); box-shadow: 0 0 12px rgba(202, 138, 4, 0.45); }
    [data-theme="dark"] .pill.stale::before { box-shadow: 0 0 16px rgba(250, 204, 21, 0.55); }
    .pill.missing, .pill.unsupported { border-color: rgba(148, 163, 184, 0.30); background: rgba(148, 163, 184, 0.10); color: #64748b; }
    [data-theme="dark"] .pill.missing, [data-theme="dark"] .pill.unsupported { border-color: rgba(148, 163, 184, 0.28); background: rgba(51, 65, 85, 0.32); color: #cbd5e1; }
    .pill.missing::before, .pill.unsupported::before { background: var(--unknown); }
    .pill.error { border-color: rgba(220, 38, 38, 0.45); background: rgba(220, 38, 38, 0.10); color: #b91c1c; }
    [data-theme="dark"] .pill.error { border-color: rgba(248, 113, 113, 0.50); background: rgba(127, 29, 29, 0.30); color: #fee2e2; }
    .pill.error::before { background: var(--bad); box-shadow: 0 0 12px rgba(220, 38, 38, 0.45); }
    [data-theme="dark"] .pill.error::before { box-shadow: 0 0 16px rgba(248, 113, 113, 0.55); }
    .estop { align-self: center; justify-content: center; border-color: rgba(220, 38, 38, 0.55); color: #b91c1c; background: rgba(220, 38, 38, 0.10); }
    [data-theme="dark"] .estop { border-color: rgba(248, 113, 113, 0.58); color: #fee2e2; background: rgba(127, 29, 29, 0.22); }
    .kv { display: grid; grid-template-columns: minmax(108px, 0.40fr) minmax(0, 1fr); gap: 5px 9px; font-size: 11.5px; line-height: 1.42; }
    .kv div:nth-child(odd) { color: var(--muted); }
    .kv div:nth-child(even) { color: var(--muted-strong); }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace; overflow-wrap: break-word; word-break: normal; font-variant-numeric: tabular-nums; }
    .empty, .error-text { color: #b91c1c; background: rgba(220, 38, 38, 0.08); border: 1px solid rgba(220, 38, 38, 0.25); border-radius: 12px; padding: 9px 10px; }
    [data-theme="dark"] .empty, [data-theme="dark"] .error-text { color: #fee2e2; background: rgba(127, 29, 29, 0.20); border-color: rgba(252, 165, 165, 0.28); }
    .notice { color: #a16207; background: rgba(202, 138, 4, 0.08); border: 1px solid rgba(202, 138, 4, 0.25); border-radius: 12px; padding: 9px 10px; }
    [data-theme="dark"] .notice { color: #fef3c7; background: rgba(113, 63, 18, 0.22); border-color: rgba(253, 230, 138, 0.28); }
    .panel-meta { margin-top: 9px; }
    .canvas-wrap { position: relative; flex: 1 1 auto; background: var(--occupied); border-radius: var(--radius-sm); padding: 1px; border: 1px solid var(--border-soft); overflow: hidden; min-height: 480px; }
    #mapCanvas { height: 100%; min-height: 480px; image-rendering: pixelated; }
    .map-toolbar { position: absolute; z-index: 2; left: 12px; top: 12px; display: flex; gap: 7px; align-items: center; flex-wrap: wrap; }
    .map-control { min-height: 30px; border: 1px solid var(--border-strong); border-radius: 10px; background: var(--surface-strong); color: var(--accent-strong); padding: 5px 9px; font: inherit; font-size: 12px; font-weight: 740; cursor: pointer; transition: background 0.16s ease, border-color 0.16s ease; }
    .map-control:hover { border-color: var(--accent); background: var(--panel-soft); }
    .map-control.active { border-color: #34d399; background: rgba(52, 211, 153, 0.14); color: #6ee7b7; }
    .map-control.active.pose-tool { border-color: #f97316; background: rgba(249, 115, 22, 0.14); color: #fdba74; }
    #mapCanvas.goal-pick { cursor: crosshair; }
    #mapCanvas.goal-pick.pan-ready { cursor: grab; }
    #mapCanvas.goal-pick.panning { cursor: grabbing; }
    #mapCanvas.publish-point { cursor: crosshair; }
    .map-goal-chip { position: absolute; z-index: 3; left: 12px; top: 50px; max-width: min(62vw, 420px); }
    .map-goal-chip.ok { border-color: rgba(52, 211, 153, 0.45); color: #6ee7b7; }
    .map-goal-chip.pending { border-color: rgba(250, 204, 21, 0.45); color: #fde68a; }
    .map-goal-chip.error { border-color: rgba(248, 113, 113, 0.45); color: #fca5a5; }
    .map-chip { border: 1px solid var(--border-strong); background: var(--surface-strong); border-radius: 999px; padding: 6px 9px; font-size: 12px; color: var(--accent-strong); backdrop-filter: blur(10px); }
    .map-stream-chip { position: absolute; z-index: 2; right: 12px; top: 12px; max-width: min(520px, calc(100% - 24px)); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
    .map-info-grid { grid-template-columns: minmax(280px, 1fr) minmax(280px, 1fr); margin-top: 10px; max-height: 176px; overflow: auto; padding-right: 2px; }
    .video-stage { position: relative; border-radius: var(--radius-sm); overflow: hidden; border: 1px solid var(--border-soft); background: var(--occupied); box-shadow: inset 0 0 0 1px rgba(255, 255, 255, 0.04); }
    .video-frame { display: block; width: 100%; min-height: 210px; max-height: 300px; object-fit: contain; position: relative; z-index: 1; }
    .video-detection-canvas { position: absolute; inset: 0; width: 100%; height: 100%; pointer-events: none; }
    .video-overlay { position: absolute; left: 10px; right: 10px; bottom: 10px; display: flex; flex-wrap: wrap; justify-content: space-between; align-items: center; gap: 8px; pointer-events: none; }
    .overlay-chip { border: 1px solid var(--border-strong); background: var(--surface-strong); border-radius: 999px; padding: 6px 9px; font-size: 12px; color: var(--accent-strong); backdrop-filter: blur(10px); }
    .overlay-toggle { display: inline-flex; align-items: center; gap: 6px; pointer-events: auto; cursor: pointer; }
    .overlay-toggle input { accent-color: var(--accent); }
    .legend { display: flex; flex-wrap: wrap; gap: 9px; margin-top: 9px; }
    .legend-item { display: inline-flex; align-items: center; gap: 6px; color: var(--muted); font-size: 11.5px; }
    .swatch { width: 12px; height: 12px; border: 1px solid var(--border); border-radius: 4px; }
    .logs { display: grid; gap: 8px; max-height: min(38vh, 430px); overflow: auto; padding-right: 2px; }
    .log-line { border: 1px solid var(--border-soft); border-left: 3px solid var(--info); border-radius: 12px; padding: 9px 10px; background: var(--panel-strong); font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace; font-size: 12px; line-height: 1.45; transition: background 0.3s ease; }
    .log-line .meta { color: var(--info); font-size: 12px; margin-bottom: 4px; }
    .log-line.warn, .log-line.warning { border-left-color: var(--warn); }
    .log-line.warn .meta, .log-line.warning .meta { color: var(--warn); }
    .log-line.error, .log-line.critical { border-left-color: var(--bad); }
    .log-line.error .meta, .log-line.critical .meta { color: var(--bad); }
    .toolbar { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
    .field { min-height: 34px; border: 1px solid var(--border); border-radius: 10px; background: var(--panel-strong); color: var(--text); padding: 7px 10px; font: inherit; font-size: 12px; outline: none; transition: background 0.3s ease, border-color 0.3s ease, color 0.3s ease; }
    .field:focus { border-color: var(--border-strong); box-shadow: 0 0 0 3px rgba(37, 99, 235, 0.12); }
    [data-theme="dark"] .field:focus { box-shadow: 0 0 0 3px rgba(34, 211, 238, 0.12); }
    .service-list, .event-list, .task-list { display: grid; gap: 8px; max-height: min(34vh, 380px); overflow: auto; padding-right: 2px; }
    .service-row, .task-row { display: grid; grid-template-columns: minmax(120px, 1fr) auto; gap: 8px; align-items: center; border: 1px solid var(--border-soft); border-radius: 12px; padding: 8px 9px; background: var(--panel-soft); transition: background 0.3s ease; }
    .service-row .meta, .task-row .meta { color: var(--muted); font-size: 12px; }
    .health-line { height: 6px; border-radius: 999px; background: linear-gradient(90deg, var(--good), var(--accent)); opacity: 0.95; }
    .health-line.warn { background: linear-gradient(90deg, var(--warn), #fb923c); }
    .health-line.bad { background: linear-gradient(90deg, var(--bad), #ef4444); }
    .split-two { grid-template-columns: 1fr 1fr; }
    .theme-toggle {
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
    }
    .theme-toggle:hover { transform: scale(1.08); border-color: var(--border-strong); }
    @media (min-width: 1800px) {
      body { padding: 16px 20px; }
      .workspace { grid-template-columns: minmax(0, 2.25fr) minmax(360px, 0.70fr); }
      .workspace, .panel.map-panel { min-height: calc(100vh - 170px); }
      .canvas-wrap { min-height: 560px; }
      #mapCanvas { min-height: 560px; }
    }
    @media (max-width: 1500px) {
      .topbar { grid-template-columns: 1fr; }
      .global-status { grid-template-columns: repeat(4, minmax(130px, 1fr)); }
      .workspace { grid-template-columns: minmax(0, 1.5fr) minmax(300px, 0.75fr); }
      .overview-grid { grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); }
      .diagnostics-grid { grid-template-columns: 1fr 1fr; }
    }
    @media (max-width: 1120px) {
      .workspace, .mission-grid, .diagnostics-grid { grid-template-columns: 1fr; }
      .right-rail { grid-template-columns: repeat(2, minmax(260px, 1fr)); max-height: none; overflow: visible; padding-right: 0; }
      #runtime-settings { grid-column: span 2; }
      .workspace, .panel.map-panel { min-height: auto; }
      .canvas-wrap { min-height: 460px; }
      #mapCanvas { height: min(68vh, 760px); min-height: 460px; }
    }
    @media (max-width: 720px) {
      body { padding: 10px; }
      .brand-block, .global-status, .panel, .card, .side-nav { border-radius: 14px; padding: 11px; }
      .side-nav { padding: 7px; }
      .split-two, .right-rail, .overview-grid, .map-info-grid { grid-template-columns: 1fr; }
      #runtime-settings { grid-column: auto; }
      .global-status { grid-template-columns: 1fr 1fr; }
      .kv { grid-template-columns: 1fr; }
      .canvas-wrap { min-height: 340px; }
      #mapCanvas { height: 58vh; min-height: 340px; }
      .map-stream-chip { left: 12px; top: 50px; right: auto; max-width: calc(100% - 24px); }
      .theme-toggle { top: 10px; right: 10px; width: 36px; height: 36px; font-size: 16px; }
    }

    html, body { height: 100%; overflow: hidden; }
    body { padding: 10px; }
    .app-shell { height: calc(100vh - 20px); max-height: calc(100vh - 20px); width: min(100%, 2260px); grid-template-rows: auto minmax(0, 1fr); overflow: hidden; }
    .topbar { grid-template-columns: minmax(250px, 0.36fr) minmax(0, 1.64fr); gap: 10px; min-height: 78px; }
    .brand-block { padding: 10px 14px; gap: 4px; }
    .brand-block .small { display: none; }
    h1 { font-size: clamp(20px, 1.25vw, 28px); }
    .global-status { grid-template-columns: repeat(8, minmax(92px, 1fr)) auto; gap: 6px; padding: 8px; }
    .metric { padding: 7px 9px; }
    .metric b { font-size: 13px; }
    .metric span { font-size: 10px; }
    .mission-layout, .mission-content { height: 100%; min-height: 0; overflow: hidden; }
    .workspace { height: 100%; min-height: 0; grid-template-columns: minmax(0, 1.55fr) minmax(410px, 0.82fr); align-items: stretch; }
    .panel.map-panel { min-height: 0; height: 100%; display: grid; grid-template-rows: auto minmax(0, 1fr) auto auto auto; overflow: hidden; }
    .map-panel .card-head { margin-bottom: 8px; }
    .canvas-wrap { min-height: 0; }
    #mapCanvas { height: 100%; min-height: 0; }
    .legend { margin-top: 6px; gap: 7px; }
    #mapEmpty { margin-top: 6px; padding: 7px 9px; }
    .map-info-grid { max-height: 142px; grid-template-columns: minmax(0, 1fr) minmax(0, 1fr); margin-top: 6px; overflow: hidden; }
    .map-info-grid .panel { min-height: 0; padding: 9px; }
    .map-info-grid .card-head { margin-bottom: 6px; }
    .map-info-grid .kv { max-height: 86px; overflow: auto; }
    .right-rail { height: 100%; max-height: none; overflow: hidden; padding-right: 0; grid-template-rows: minmax(0, 1.08fr) minmax(0, 0.92fr); }
    .tab-panel { display: grid; grid-template-rows: auto auto minmax(0, 1fr); min-height: 0; overflow: hidden; }
    .tab-title { display: flex; align-items: center; justify-content: space-between; gap: 10px; margin-bottom: 8px; }
    .tab-menu { display: grid; grid-auto-flow: column; grid-auto-columns: 1fr; gap: 6px; margin-bottom: 8px; }
    .tab-button { border: 1px solid var(--border-soft); border-radius: 999px; background: var(--panel-strong); color: var(--muted-strong); min-height: 30px; padding: 6px 8px; font: inherit; font-size: 11.5px; font-weight: 760; cursor: pointer; transition: background 0.16s ease, border-color 0.16s ease, color 0.16s ease; }
    .tab-button:hover, .tab-button.active { border-color: var(--border-strong); background: rgba(37, 99, 235, 0.11); color: var(--text); }
    [data-theme="dark"] .tab-button:hover, [data-theme="dark"] .tab-button.active { background: rgba(34, 211, 238, 0.12); }
    .tab-content { min-height: 0; overflow: auto; padding-right: 2px; }
    .tab-pane { display: none; min-height: 0; }
    .tab-pane.active { display: block; }
    .tab-pane > .panel, .tab-pane > section.panel { box-shadow: none; background: rgba(255, 255, 255, 0.04); }
    [data-theme="dark"] .tab-pane > .panel, [data-theme="dark"] .tab-pane > section.panel { background: rgba(15, 23, 42, 0.28); }
    .overview-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 8px; }
    #overview .overview-grid { max-height: none; overflow: visible; padding-right: 0; }
    #overview .status-card, .status-card { min-height: 92px; padding: 10px; }
    .status-card .kv { gap: 4px 7px; }
    .task-list, .service-list { max-height: none; }
    .logs { max-height: none; }
    .video-frame { min-height: 170px; max-height: 230px; }
    .diagnostics-grid, .side-nav { display: none; }
    @media (max-width: 1500px) {
      .topbar { grid-template-columns: 1fr; min-height: 126px; }
      .global-status { grid-template-columns: repeat(4, minmax(120px, 1fr)); }
      .workspace { grid-template-columns: minmax(0, 1.35fr) minmax(360px, 0.85fr); }
    }
    @media (max-width: 1120px) {
      html, body { overflow: auto; }
      body { padding: 10px; }
      .app-shell { height: auto; max-height: none; overflow: visible; }
      .workspace { height: auto; grid-template-columns: 1fr; }
      .panel.map-panel { height: auto; min-height: 620px; }
      .canvas-wrap { min-height: 420px; }
      #mapCanvas { height: 52vh; min-height: 420px; }
      .right-rail { height: auto; grid-template-rows: none; }
      .diagnostics-grid { display: none; }
    }
  </style>
</head>
<body>
  <button id="themeToggle" class="theme-toggle" title="切换主题" aria-label="切换白天/黑夜主题">☀️</button>
  <div class="app-shell">
    <header class="topbar">
      <div class="brand-block">
        <div class="eyebrow">只读机器人观测</div>
        <div class="brand-row">
          <h1 id="robotName">nuwax_robot_bridge</h1>
          <span id="robotModel" class="pill missing">型号：--</span>
        </div>
        <div class="muted small">单屏实时地图驾驶舱 · 关键态势优先</div>
      </div>
      <div class="global-status">
        <div class="metric"><b id="connectionText">connecting</b><span>网关连接</span></div>
        <div class="metric"><b id="runtimeMode">--</b><span>运行模式</span></div>
        <div class="metric"><b id="batteryMetric">--</b><span>电池状态</span></div>
        <div class="metric"><b id="networkLatency" class="mono">--</b><span>HTTP 往返</span></div>
        <div class="metric"><b id="resourceMetric">--</b><span>资源/任务</span></div>
        <div class="metric"><b id="clockNow" class="mono">--</b><span>当前时间</span></div>
        <div class="metric"><b id="lastReceive" class="mono">--</b><span>数据接收</span></div>
        <span id="estopStatus" class="pill estop">急停状态：只读展示</span>
      </div>
    </header>

    <div class="mission-layout">
      <div class="mission-content">
        <main class="grid workspace">
          <section id="live-map" class="panel map-panel">
            <div class="card-head">
              <div>
                <div class="eyebrow">Live Spatial Canvas</div>
                <h2>空间态势 · 2D 地图 / 定位 / 轨迹 / 导航目标</h2>
              </div>
              <span id="mapStatus" class="pill missing">missing</span>
            </div>
            <div class="canvas-wrap">
              <div class="map-toolbar" aria-label="地图缩放与导航控制">
                <button id="mapZoomOut" class="map-control" type="button">-</button>
                <span id="mapZoomValue" class="map-chip">100%</span>
                <button id="mapZoomIn" class="map-control" type="button">+</button>
                <button id="mapZoomReset" class="map-control" type="button">reset</button>
                <button id="mapCenterRobot" class="map-control" type="button" title="视图居中到机器人（RViz 风格）" hidden>居中</button>
                <button id="mapInitialPoseToggle" class="map-control pose-tool" type="button" title="RViz 2D Pose Estimate：左键拖拽设初始位姿，发布 /initialpose">2D Pose</button>
                <button id="mapGoalToggle" class="map-control" type="button" title="RViz 风格 2D Goal：北向 map、左键设目标、中/右键平移、滚轮缩放">2D Goal</button>
                <button id="mapPublishPointToggle" class="map-control" type="button" title="RViz Publish Point：左键发布 /clicked_point，不导航">选点</button>
                <button id="mapModeToggle" class="map-control" type="button" style="font-size:11px;padding:2px 8px;">3D</button>
              </div>
              <span id="mapGoalStatus" class="map-chip map-goal-chip" hidden></span>
              <span id="mapStreamStatus" class="map-chip map-stream-chip">等待地图首帧</span>
              <canvas id="mapCanvas" width="1180" height="680"></canvas>
              <div id="map3dContainer" style="width:100%;height:100%;min-height:480px;position:absolute;inset:0;display:none;"></div>
            </div>
            <div class="legend">
              <span class="legend-item"><span class="swatch" style="background: var(--unknown)"></span>未知</span>
              <span class="legend-item"><span class="swatch" style="background: var(--free)"></span>可通行</span>
              <span class="legend-item"><span class="swatch" style="background: var(--occupied)"></span>占据</span>
              <span class="legend-item"><span class="swatch" style="background: var(--map-line)"></span>轨迹</span>
              <span class="legend-item"><span class="swatch" style="background: var(--nav-plan)"></span>规划路径</span>
              <span class="legend-item"><span class="swatch" style="background: var(--map-robot)"></span>机器人</span>
              <span class="legend-item"><span class="swatch" style="background: var(--frontier)"></span>探索前沿</span>
              <span class="legend-item"><span class="swatch" style="background: #f97316"></span>初始位姿</span>
              <span class="legend-item"><span class="swatch" style="background: #34d399"></span>导航目标</span>
              <span class="legend-item"><span class="swatch" style="background: #38bdf8"></span>选点</span>
              <span id="mapRvizHint" class="legend-item" hidden>2D Goal：左键拖拽设目标 · 中/右键平移 · 滚轮缩放</span>
            </div>
            <div id="mapEmpty" class="empty panel-meta">等待地图数据。</div>
            <div class="grid map-info-grid">
              <section class="panel compact">
                <div class="card-head"><h3>地图信息</h3><span class="muted small">occupancy grid</span></div>
                <div id="mapMeta" class="kv"></div>
              </section>
              <section class="panel compact">
                <div class="card-head">
                  <h3>定位/轨迹</h3>
                  <span id="localizationStatus" class="pill missing">missing</span>
                </div>
                <div id="localizationMeta" class="kv"></div>
              </section>
            </div>
          </section>

          <aside class="grid right-rail" aria-label="态势与诊断面板">
            <section class="panel tab-panel" data-tab-group="situation">
              <div class="tab-title">
                <div>
                  <div class="eyebrow">Situation</div>
                  <h2>状态总览</h2>
                </div>
                <span id="missionStatus" class="pill missing">mission: warming</span>
              </div>
              <div class="tab-menu" role="tablist" aria-label="状态总览二级菜单">
                <button class="tab-button active" type="button" data-tab-target="navigation-pane">导航</button>
                <button class="tab-button" type="button" data-tab-target="overview-pane">总览</button>
                <button class="tab-button" type="button" data-tab-target="task-pane">任务</button>
                <button class="tab-button" type="button" data-tab-target="sensor-pane">传感器</button>
              </div>
              <div class="tab-content">
                <div id="navigation-pane" class="tab-pane active">
                  <section id="navigation" class="panel compact">
                    <div class="card-head">
                      <h3>导航态势 / frontier（探索前沿）</h3>
                      <span id="navigationStatus" class="pill missing">missing</span>
                    </div>
                    <div id="navigationMeta" class="kv"></div>
                    <div id="frontierNotice" class="notice panel-meta">等待探索前沿候选几何。</div>
                  </section>
                </div>
                <div id="overview-pane" class="tab-pane">
                  <section id="overview" class="panel compact">
                    <div class="card-head"><h3>运行态总览</h3><span class="muted small">核心链路</span></div>
                    <section id="overviewGrid" class="grid overview-grid" aria-label="运行态总览"></section>
                  </section>
                </div>
                <div id="task-pane" class="tab-pane">
                  <section class="panel compact">
                    <div class="card-head"><h3>当前任务</h3><span id="taskStatus" class="pill missing">missing</span></div>
                    <div id="taskSummary" class="task-list"></div>
                  </section>
                </div>
                <div id="sensor-pane" class="tab-pane">
                  <section class="panel compact">
                    <div class="card-head"><h3>位姿与运动</h3><span id="poseStatus" class="pill missing">missing</span></div>
                    <div id="missionPoseMeta" class="kv"></div>
                  </section>
                  <section class="panel compact panel-meta">
                    <div class="card-head"><h3>传感器链路</h3><span id="sensorStatus" class="pill missing">missing</span></div>
                    <div id="sensorSummary" class="kv"></div>
                  </section>
                </div>
              </div>
            </section>

            <section class="panel tab-panel" data-tab-group="diagnostics">
              <div class="tab-title">
                <div>
                  <div class="eyebrow">Diagnostics</div>
                  <h2>运行诊断</h2>
                </div>
                <span id="governanceStatus" class="pill missing">missing</span>
              </div>
              <div class="tab-menu" role="tablist" aria-label="运行诊断二级菜单">
                <button class="tab-button active" type="button" data-tab-target="governance-pane">治理</button>
                <button class="tab-button" type="button" data-tab-target="video-pane">视频</button>
                <button class="tab-button" type="button" data-tab-target="service-pane">服务</button>
                <button class="tab-button" type="button" data-tab-target="logs-pane">日志</button>
              </div>
              <div class="tab-content">
                <div id="governance-pane" class="tab-pane active">
                  <section id="runtime-settings" class="panel compact">
                    <div class="card-head"><h3>RTAB-Map 运行治理</h3><span class="muted small">只读</span></div>
                    <div id="governanceMeta" class="kv"></div>
                    <div class="muted small panel-meta">展示 profile、几何资产、重定位质量和导航门控摘要。</div>
                  </section>
                </div>
                <div id="video-pane" class="tab-pane">
                  <section id="video-perception" class="panel compact">
                    <div class="card-head">
                      <h3>端侧推流视频感知</h3>
                      <span id="videoStatus" class="pill missing">missing</span>
                    </div>
                    <div class="video-stage">
                      <img id="videoFrame" class="video-frame" alt="Go2 端侧推流视频帧" hidden>
                      <canvas id="videoDetections" class="video-detection-canvas" hidden></canvas>
                      <div class="video-overlay">
                        <span id="visionModelStatus" class="overlay-chip">本地模型：--</span>
                        <span id="videoFpsStatus" class="overlay-chip">帧率：--</span>
                        <label class="overlay-chip overlay-toggle"><input id="showDetections" type="checkbox">显示检测框</label>
                      </div>
                    </div>
                    <div id="videoEmpty" class="empty panel-meta">等待 Go2 端侧视频推流。</div>
                    <div id="videoMeta" class="kv panel-meta"></div>
                  </section>
                </div>
                <div id="service-pane" class="tab-pane">
                  <section id="services" class="panel compact">
                    <div class="card-head">
                      <h3>服务矩阵</h3>
                      <span id="servicesStatus" class="pill missing">missing</span>
                    </div>
                    <div id="serviceMatrix" class="service-list"></div>
                  </section>
                </div>
                <div id="logs-pane" class="tab-pane">
                  <section id="logs-section" class="panel compact">
                    <div class="card-head">
                      <h3>运行日志尾部</h3>
                      <span id="logsStatus" class="pill missing">missing</span>
                    </div>
                    <div class="toolbar">
                      <select id="logLevelFilter" class="field" aria-label="日志等级过滤">
                        <option value="">ALL</option>
                        <option value="debug">DEBUG</option>
                        <option value="info">INFO</option>
                        <option value="warn">WARN</option>
                        <option value="error">ERROR</option>
                        <option value="critical">CRITICAL</option>
                      </select>
                      <input id="logSearch" class="field" type="search" placeholder="搜索日志、节点或关键词" aria-label="搜索日志">
                    </div>
                    <div id="logsMeta" class="kv"></div>
                    <div id="logs" class="logs panel-meta"></div>
                  </section>
                </div>
              </div>
            </section>
          </aside>
        </main>
      </div>
    </div>
  </div>

  <script>
    (() => {
      const DEBUG_ACCESS_TOKEN = __DEBUG_ACCESS_TOKEN__;
      const urlAccessToken = new URLSearchParams(window.location.search).get("access_token") || "";
      const apiAccessToken = urlAccessToken || DEBUG_ACCESS_TOKEN;
      const CANVAS_THEME = {
        mapBackground: "#020617",
        mapUnknown: [51, 65, 85],
        mapFree: [219, 234, 254],
        mapOccupied: [2, 6, 23],
        trajectory: "#22d3ee",
        frontier: "#f472b6",
        frontierStroke: "#fbcfe8",
        robot: "#fbbf24",
        robotStroke: "#fef3c7",
        goal: "#a78bfa",
        goalStroke: "#ddd6fe",
        goalPick: "#34d399",
        goalPickStroke: "#6ee7b7",
        clickedPoint: "#38bdf8",
        clickedPointStroke: "#7dd3fc",
        initialPosePick: "#f97316",
        initialPosePickStroke: "#fdba74",
        navPath: "#22ff66",
      };
      const overviewNames = ["health", "state", "localization", "map", "navigation", "exploration", "runtime_governance", "pointcloud_map", "pointcloud", "video", "logs"];
      const overviewLabels = { health: "健康", state: "状态", localization: "定位", map: "地图", navigation: "导航", exploration: "探索", runtime_governance: "RTAB-Map治理", pointcloud_map: "3D点云地图", pointcloud: "扫描点云", video: "视频", logs: "日志" };
      const OVERVIEW_RENDER_INTERVAL_MS = 500;
      const STREAM_FLUSH_ORDER = ["robot.state", "map.frame", "map.heartbeat", "localization.trajectory", "localization.pose", "navigation.state", "exploration.frontier", "exploration.state", "runtime.governance", "pointcloud_map.meta", "pointcloud.frame", "video.frame", "video.detections", "tasks.state"];
      const VIDEO_STREAM_INITIAL_RECONNECT_MS = 500;
      const VIDEO_STREAM_MAX_RECONNECT_MS = 5000;
      const VIDEO_JPEG_DECODE_TIMEOUT_MS = 3000;
      const MJPEG_MAX_BUFFER_BYTES = 16 * 1024 * 1024;
      const MAP_DISPLAY_FRESH_MS = 20000;
      const MAP_DISPLAY_IDLE_MESSAGE = "未启动建图/导航/探索，隐藏缓存地图。";
      const LOCALIZATION_DISPLAY_IDLE_MESSAGE = "无活跃地图/建图/导航/探索，隐藏定位。";
      const ACTIVE_SPATIAL_STATUSES = new Set(["accepted", "planning", "running"]);
      const ACTIVE_SPATIAL_CAPABILITIES = new Set([
        "explore_area",
        "explore_and_find_target",
        "navigate_to_pose",
        "navigate_to_named_location",
        "find_target",
        "inspect_target",
        "relocalize_active_map",
        "load_map_version",
        "activate_map",
      ]);
      const ACTIVE_LOCALIZATION_STATUSES = new Set([
        "accepted",
        "running",
        "localizing",
        "relocalizing",
        "ready",
      ]);
      const state = {
        mapGrid: null,
        mapViewport: null,
        mapZoom: 1,
        mapRvizView: { center: null, zoom: 1 },
        mapPanDrag: null,
        mapStream: null,
        pose: null,
        trajectory: [],
        frontiers: [],
        goalPose: null,
        goalPickMode: false,
        publishPointMode: false,
        initialPosePickMode: false,
        goalPickDrag: null,
        initialPosePickDrag: null,
        publishPointClick: null,
        clickedPoints: [],
        initialPoses: [],
        pendingNavGoal: null,
        navigationPath: [],
        logs: [],
        logCursor: 0,
        streamSocket: null,
        streamReconnectTimer: null,
        latestStreamCursor: 0,
        latestStreamFrames: {},
        pendingStreamFrames: {},
        pendingLogFrames: [],
        streamFlushFrame: null,
        videoFrames: 0,
        pendingVideoEnvelope: null,
        videoRenderFrame: null,
        lastVideoFrameKey: null,
        videoStreamSrc: null,
        videoStreamController: null,
        videoStreamGeneration: 0,
        videoStreamReconnectTimer: null,
        videoStreamReconnectDelayMs: VIDEO_STREAM_INITIAL_RECONNECT_MS,
        videoStreamStatus: "idle",
        videoStreamBytes: 0,
        videoDecodedBytes: 0,
        videoDecodeErrors: 0,
        videoLastError: "",
        videoLastFrameAt: null,
        videoCurrentObjectUrl: null,
        showDetections: false,
        videoDetections: [],
        videoSourceSize: null,
        videoDetectionDrawFrame: null,
        latestOverview: null,
        overviewRenderTimer: null,
        overviewLastRenderAt: 0,
        overviewReceivedAt: null,
        latestHealth: null,
        latestGovernance: null,
        latestTasks: { active_tasks: [], history_tasks: [] },
        taskActiveById: new Map(),
        taskHistoryById: new Map(),
        lastRobotStateRevision: null,
        lastRobotStateHash: null,
        logFilter: "",
        logSearch: "",
        mapDrawFrame: null,
        mapBaseCache: null,
        mapGridContentKey: "",
        lastMapEnvelope: null,
        lastLocalizationEnvelope: null,
        lastMapHeartbeatEnvelope: null,
        latestFrontierStream: null,
      };
      const els = {
        robotName: document.getElementById("robotName"),
        robotModel: document.getElementById("robotModel"),
        connectionText: document.getElementById("connectionText"),
        runtimeMode: document.getElementById("runtimeMode"),
        batteryMetric: document.getElementById("batteryMetric"),
        networkLatency: document.getElementById("networkLatency"),
        resourceMetric: document.getElementById("resourceMetric"),
        clockNow: document.getElementById("clockNow"),
        estopStatus: document.getElementById("estopStatus"),
        missionStatus: document.getElementById("missionStatus"),
        taskStatus: document.getElementById("taskStatus"),
        taskSummary: document.getElementById("taskSummary"),
        poseStatus: document.getElementById("poseStatus"),
        missionPoseMeta: document.getElementById("missionPoseMeta"),
        sensorStatus: document.getElementById("sensorStatus"),
        sensorSummary: document.getElementById("sensorSummary"),
        lastReceive: document.getElementById("lastReceive"),
        overviewGrid: document.getElementById("overviewGrid"),
        mapCanvas: document.getElementById("mapCanvas"),
        mapZoomIn: document.getElementById("mapZoomIn"),
        mapZoomOut: document.getElementById("mapZoomOut"),
        mapZoomReset: document.getElementById("mapZoomReset"),
        mapCenterRobot: document.getElementById("mapCenterRobot"),
        mapZoomValue: document.getElementById("mapZoomValue"),
        mapRvizHint: document.getElementById("mapRvizHint"),
        mapInitialPoseToggle: document.getElementById("mapInitialPoseToggle"),
        mapGoalToggle: document.getElementById("mapGoalToggle"),
        mapPublishPointToggle: document.getElementById("mapPublishPointToggle"),
        mapGoalStatus: document.getElementById("mapGoalStatus"),
        mapStreamStatus: document.getElementById("mapStreamStatus"),
        mapStatus: document.getElementById("mapStatus"),
        mapEmpty: document.getElementById("mapEmpty"),
        mapMeta: document.getElementById("mapMeta"),
        localizationStatus: document.getElementById("localizationStatus"),
        localizationMeta: document.getElementById("localizationMeta"),
        navigationStatus: document.getElementById("navigationStatus"),
        navigationMeta: document.getElementById("navigationMeta"),
        frontierNotice: document.getElementById("frontierNotice"),
        governanceStatus: document.getElementById("governanceStatus"),
        governanceMeta: document.getElementById("governanceMeta"),
        videoStatus: document.getElementById("videoStatus"),
        videoFrame: document.getElementById("videoFrame"),
        videoDetections: document.getElementById("videoDetections"),
        showDetections: document.getElementById("showDetections"),
        videoEmpty: document.getElementById("videoEmpty"),
        videoMeta: document.getElementById("videoMeta"),
        visionModelStatus: document.getElementById("visionModelStatus"),
        videoFpsStatus: document.getElementById("videoFpsStatus"),
        servicesStatus: document.getElementById("servicesStatus"),
        serviceMatrix: document.getElementById("serviceMatrix"),
        logsStatus: document.getElementById("logsStatus"),
        logsMeta: document.getElementById("logsMeta"),
        logs: document.getElementById("logs"),
        logLevelFilter: document.getElementById("logLevelFilter"),
        logSearch: document.getElementById("logSearch"),
        themeToggle: document.getElementById("themeToggle"),
      };

      function initTheme() {
        const saved = localStorage.getItem("nuwax-theme");
        const prefersDark = window.matchMedia && window.matchMedia("(prefers-color-scheme: dark)").matches;
        const theme = saved || (prefersDark ? "dark" : "light");
        document.documentElement.setAttribute("data-theme", theme);
        updateThemeIcon(theme);
      }
      function toggleTheme() {
        const current = document.documentElement.getAttribute("data-theme") || "dark";
        const next = current === "dark" ? "light" : "dark";
        document.documentElement.setAttribute("data-theme", next);
        localStorage.setItem("nuwax-theme", next);
        updateThemeIcon(next);
      }
      function updateThemeIcon(theme) {
        if (els.themeToggle) els.themeToggle.textContent = theme === "dark" ? "🌙" : "☀️";
      }
      initTheme();
      if (els.themeToggle) els.themeToggle.addEventListener("click", toggleTheme);
      function initTabPanels() {
        for (const panel of document.querySelectorAll(".tab-panel")) {
          const buttons = Array.from(panel.querySelectorAll(".tab-button"));
          const panes = Array.from(panel.querySelectorAll(".tab-pane"));
          for (const button of buttons) {
            button.addEventListener("click", () => {
              const targetId = button.dataset.tabTarget;
              for (const item of buttons) item.classList.toggle("active", item === button);
              for (const pane of panes) pane.classList.toggle("active", pane.id === targetId);
              redrawMapAfterResize();
            });
          }
        }
      }
      initTabPanels();

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
        const scrollContainer = el.closest(".map-info-grid, .tab-content, .kv");
        const savedScroll = scrollContainer ? scrollContainer.scrollTop : 0;
        el.replaceChildren();
        for (const [key, value] of rows) {
          const k = document.createElement("div");
          const v = document.createElement("div");
          v.className = "mono";
          k.textContent = key;
          v.textContent = safeText(value);
          el.append(k, v);
        }
        if (scrollContainer && savedScroll > 0) {
          scrollContainer.scrollTop = savedScroll;
        }
      }
      function setText(el, value) {
        if (!el) return;
        el.textContent = safeText(value);
      }
      function syncMapCanvasSize() {
        const canvas = els.mapCanvas;
        if (!canvas) return false;
        const rect = canvas.getBoundingClientRect();
        const width = Math.max(360, Math.round(rect.width));
        const height = Math.max(320, Math.round(rect.height));
        if (canvas.width === width && canvas.height === height) return false;
        canvas.width = width;
        canvas.height = height;
        return true;
      }
      function redrawMapAfterResize() {
        requestMapDraw(false);
      }
      function statusSeverity(status) {
        const value = statusClass(status || "missing");
        if (value === "available") return 0;
        if (value === "stale") return 1;
        if (value === "missing" || value === "unsupported") return 2;
        return 3;
      }
      function worstStatus(items) {
        let result = "available";
        for (const item of items) {
          const status = item && item.status ? item.status : "missing";
          if (statusSeverity(status) > statusSeverity(result)) result = status;
        }
        return result;
      }
      function updateClock() {
        setText(els.clockNow, nowText());
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
      function summarizeSource(source) {
        const value = String(source || "");
        if (!value) return "--";
        const parts = value.split(/[/.]/).filter(Boolean);
        return parts.length ? parts[parts.length - 1].slice(0, 28) : value.slice(0, 28);
      }
      function updateOverview(envelope) {
        const data = envelope && envelope.data ? envelope.data : {};
        state.latestOverview = data;
        if (!state.streamSocket || state.streamSocket.readyState !== WebSocket.OPEN) setText(els.connectionText, "online");
        setText(els.networkLatency, fmt(envelope.__frontendLatencyMs, " ms"));
        for (const name of overviewNames) {
          const summary = data[name] || {};
          const section = document.getElementById(`overview-${name}`);
          if (!section) continue;
          setPill(section.querySelector(".pill"), summary.status || "missing");
          setKv(section.querySelector(".kv"), [
            ["时间", shortTime(summary.source_timestamp)],
            ["延迟", fmt(summary.latency_ms, " ms")],
            ["过期", summary.stale === true ? "是" : "否"],
            ["来源", summarizeSource(summary.metadata && summary.metadata.source)],
          ]);
          section.querySelector(".small").textContent = `前端接收 ${shortTime(envelope.__receivedAt)}`;
        }
        const mission = worstStatus([data.health, data.state, data.localization, data.map, data.navigation, data.video, data.logs]);
        setPill(els.missionStatus, mission, "mission");
        updateSensorSummary(data);
        renderServiceMatrix(data);
      }
      function updateSensorSummary(data) {
        const map = data.map || {};
        const localization = data.localization || {};
        const video = data.video || {};
        const pointcloudMap = data.pointcloud_map || {};
        const pointcloud = data.pointcloud || {};
        const sensorStatus = worstStatus([map, localization, pointcloudMap, pointcloud, video]);
        setPill(els.sensorStatus, sensorStatus);
        setKv(els.sensorSummary, [
          ["2D_map", `${safeText(map.status)} latency=${fmt(map.latency_ms, " ms")}`],
          ["localization", `${safeText(localization.status)} frame=${safeText(localization.frame_id)}`],
          ["pointcloud_map", `${safeText(pointcloudMap.status)} frame=${safeText(pointcloudMap.frame_id)} rev=${safeText(pointcloudMap.revision)} points=${safeText(pointcloudMap.metadata && pointcloudMap.metadata.point_count)} source=${safeText(pointcloudMap.metadata && pointcloudMap.metadata.source_kind)}`],
          ["pointcloud", `${safeText(pointcloud.status)} points=${safeText(pointcloud.metadata && pointcloud.metadata.point_count)} source=${safeText(pointcloud.metadata && pointcloud.metadata.source)}`],
          ["video", `${safeText(video.status)} source=${safeText(video.metadata && video.metadata.source)}`],
        ]);
      }
      function renderServiceMatrix(data) {
        const rows = [
          ["gateway", data.health],
          ["robot_state", data.state],
          ["mapping", data.map],
          ["localization", data.localization],
          ["navigation", data.navigation],
          ["exploration", data.exploration],
          ["rtabmap_governance", data.runtime_governance],
          ["pointcloud_map", data.pointcloud_map],
          ["pointcloud", data.pointcloud],
          ["video", data.video],
          ["logs", data.logs],
        ];
        els.serviceMatrix.replaceChildren();
        const serviceStatus = worstStatus(rows.map((item) => item[1]));
        setPill(els.servicesStatus, serviceStatus);
        for (const [name, summary] of rows) {
          const status = summary && summary.status ? summary.status : "missing";
          const row = document.createElement("article");
          row.className = "service-row";
          const left = document.createElement("div");
          const title = document.createElement("div");
          title.textContent = name;
          title.className = "mono";
          const meta = document.createElement("div");
          meta.className = "meta";
          meta.textContent = `${safeText(summary && summary.metadata && summary.metadata.source)} · latency ${fmt(summary && summary.latency_ms, " ms")} · ${shortTime(summary && summary.source_timestamp)}`;
          const line = document.createElement("div");
          line.className = `health-line ${statusSeverity(status) >= 3 ? "bad" : statusSeverity(status) >= 1 ? "warn" : ""}`;
          left.append(title, meta, line);
          const pill = document.createElement("span");
          pill.className = `pill ${statusClass(status)}`;
          pill.textContent = status;
          row.append(left, pill);
          els.serviceMatrix.appendChild(row);
        }
      }
      function updateHealth(payload) {
        const health = payload || {};
        state.latestHealth = health;
        setText(els.robotName, health.robot_name || "nuwax_robot_bridge");
        setPill(els.robotModel, health.runtime_started ? "available" : "missing", `型号：${safeText(health.robot_model)}`);
        const robot = health.latest_robot_snapshot || {};
        const status = health.assembly_status || {};
        const safety = robot.safety || {};
        const battery = robot.battery || {};
        setText(els.runtimeMode, status.control_mode || robot.mode || "--");
        const rawBattery = Number(battery.percentage);
        const batteryValue = battery.percentage !== undefined
          ? `${Math.round(rawBattery <= 1 ? rawBattery * 100 : rawBattery)}%`
          : (battery.level !== undefined ? `${battery.level}%` : "--");
        setText(els.batteryMetric, batteryValue);
        setText(els.resourceMetric, `tasks ${health.active_task_count ?? 0} · events ${health.latest_event_cursor ?? 0}`);
        const estopped = safety.is_estopped === true || safety.estopped === true;
        setPill(els.estopStatus, estopped ? "error" : "available", estopped ? "急停状态：触发" : "急停状态：清除");
        els.estopStatus.classList.add("estop");
      }
      function normalizeTaskState(value) {
        const raw = value && typeof value === "object" && "value" in value ? value.value : value;
        const stateValue = String(raw || "missing").toLowerCase();
        if (stateValue.startsWith("task.")) return stateValue.slice(5);
        return stateValue;
      }
      function isTerminalTaskState(value) {
        return ["succeeded", "failed", "cancelled", "timeout"].includes(normalizeTaskState(value));
      }
      function taskUpdatedMs(task) {
        const value = task && (task.updated_at || task.completed_at || task.started_at || task.timestamp || task.created_at);
        const parsed = value ? Date.parse(value) : 0;
        return Number.isFinite(parsed) ? parsed : 0;
      }
      function taskListFromMap(map, limit) {
        const items = Array.from(map.values()).sort((a, b) => taskUpdatedMs(b) - taskUpdatedMs(a));
        return typeof limit === "number" ? items.slice(0, limit) : items;
      }
      function refreshTaskStore() {
        const active = taskListFromMap(state.taskActiveById);
        const history = taskListFromMap(state.taskHistoryById, 20);
        state.latestTasks = { active_tasks: active, history_tasks: history };
        updateTasks(state.latestTasks);
        updateHealth({ ...(state.latestHealth || {}), active_task_count: active.length });
        reevaluateCachedMapDisplay();
      }
      function replaceTaskStore(payload) {
        state.taskActiveById.clear();
        state.taskHistoryById.clear();
        for (const task of Array.isArray(payload && payload.active_tasks) ? payload.active_tasks : []) {
          if (task && task.task_id) state.taskActiveById.set(String(task.task_id), task);
        }
        for (const task of Array.isArray(payload && payload.history_tasks) ? payload.history_tasks : []) {
          if (task && task.task_id) state.taskHistoryById.set(String(task.task_id), task);
        }
        refreshTaskStore();
      }
      function updateTasks(payload) {
        const active = Array.isArray(payload && payload.active_tasks) ? payload.active_tasks : [];
        const history = Array.isArray(payload && payload.history_tasks) ? payload.history_tasks : [];
        state.latestTasks = payload || {};
        els.taskSummary.replaceChildren();
        setPill(els.taskStatus, "available", active.length ? `${active.length} active` : "idle");
        const items = active.length ? active.slice(0, 4) : history.slice(0, 3);
        if (!items.length) {
          const row = document.createElement("article");
          row.className = "task-row";
          row.innerHTML = '<div><div class="mono">idle</div><div class="meta">当前没有活动任务</div></div><span class="pill missing">idle</span>';
          els.taskSummary.appendChild(row);
          return;
        }
        for (const task of items) {
          const row = document.createElement("article");
          row.className = "task-row";
          const left = document.createElement("div");
          const title = document.createElement("div");
          title.className = "mono";
          title.textContent = task.task_id || task.capability_name || "task";
          const meta = document.createElement("div");
          meta.className = "meta";
          meta.textContent = `${safeText(task.capability_name)} · progress ${fmt(task.progress, "")} · ${safeText(task.stage || task.state)}`;
          left.append(title, meta);
          const pill = document.createElement("span");
          const taskState = normalizeTaskState(task.state || task.status);
          pill.className = `pill ${taskState === "failed" || taskState === "timeout" ? "error" : taskState === "running" || taskState === "accepted" ? "available" : taskState === "succeeded" ? "available" : "stale"}`;
          pill.textContent = taskState;
          row.append(left, pill);
          els.taskSummary.appendChild(row);
        }
      }
      async function loadInitialTasks() {
        try {
          const response = await fetch(withAccessToken("/api/tasks?history_limit=20"));
          if (!response.ok) throw new Error(`HTTP ${response.status}`);
          replaceTaskStore(await response.json());
        } catch (err) {
          showPanelError("logs", `任务列表初始化失败：${err.message || err}`);
        }
      }
      function startTaskPolling() {
        setInterval(async () => {
          try {
            const response = await fetch(withAccessToken("/api/tasks?history_limit=20"));
            if (!response.ok) return;
            replaceTaskStore(await response.json());
          } catch (err) {
            // 静默忽略轮询失败
          }
        }, 3000);
      }
      function showPanelError(kind, message) {
        if (kind === "governance") {
          setPill(els.governanceStatus, "error");
          setKv(els.governanceMeta, [["error", message], ["前端接收", nowText()]]);
        } else if (kind === "map") {
          setPill(els.mapStatus, "error");
          els.mapEmpty.hidden = false;
          els.mapEmpty.textContent = message;
        } else if (kind === "localization") {
          setPill(els.localizationStatus, "error");
          setKv(els.localizationMeta, [["error", message], ["前端接收", nowText()]]);
        } else if (kind === "navigation") {
          setPill(els.navigationStatus, "error");
          setKv(els.navigationMeta, [["error", message], ["前端接收", nowText()]]);
        } else if (kind === "video") {
          setPill(els.videoStatus, "error");
          els.videoEmpty.hidden = false;
          els.videoEmpty.textContent = message;
        } else if (kind === "logs") {
          setPill(els.logsStatus, "error");
          setKv(els.logsMeta, [["error", message], ["前端接收", nowText()]]);
        }
      }
      function normalizeFrameId(value, fallback) {
        const raw = String(value || fallback || "map").trim().replace(/^\/+/, "");
        return raw || "map";
      }
      function gridResolution(grid) {
        if (!grid) return 1;
        const candidates = [grid.resolution_m, grid.resolution, grid.metadata && grid.metadata.resolution];
        for (const value of candidates) {
          const number = Number(value);
          if (Number.isFinite(number) && number > 0) return number;
        }
        return 1;
      }
      function originPosition(grid) {
        const origin = grid && grid.origin;
        if (!origin) return { x: 0, y: 0, z: 0 };
        if (origin.position) return origin.position;
        if (origin.x !== undefined && origin.y !== undefined) {
          return { x: Number(origin.x), y: Number(origin.y), z: Number(origin.z || 0) };
        }
        return { x: 0, y: 0, z: 0 };
      }
      function originYaw(grid) { return planarYawFromPose(grid && grid.origin) || 0; }
      function pointFromPose(pose) {
        const pos = pose && pose.position;
        if (!pos) return null;
        const x = Number(pos.x);
        const y = Number(pos.y);
        if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
        return { x, y, z: Number(pos.z || 0) };
      }
      function yawFromPose(pose) {
        const q = pose && pose.orientation;
        if (!q) return null;
        const siny = 2 * ((Number(q.w) || 0) * (Number(q.z) || 0) + (Number(q.x) || 0) * (Number(q.y) || 0));
        const cosy = 1 - 2 * ((Number(q.y) || 0) * (Number(q.y) || 0) + (Number(q.z) || 0) * (Number(q.z) || 0));
        return Math.atan2(siny, cosy);
      }
      function planarYawFromPose(pose) {
        const q = pose && pose.orientation;
        if (!q) return yawFromPose(pose);
        const x = Number(q.x) || 0;
        const y = Number(q.y) || 0;
        const z = Number(q.z) || 0;
        const w = Number(q.w) || 1;
        const forwardX = 1 - 2 * (y * y + z * z);
        const forwardY = 2 * (x * y + w * z);
        if (Math.hypot(forwardX, forwardY) < 1e-6) return yawFromPose(pose);
        return Math.atan2(forwardY, forwardX);
      }
      function mapCorners(grid) {
        if (!grid || !grid.width || !grid.height) return [];
        const origin = originPosition(grid);
        const yaw = originYaw(grid);
        const cos = Math.cos(yaw);
        const sin = Math.sin(yaw);
        const resolution = gridResolution(grid);
        const width = Number(grid.width) * resolution;
        const height = Number(grid.height) * resolution;
        return [[0, 0], [width, 0], [width, height], [0, height]].map(([x, y]) => ({
          x: Number(origin.x || 0) + x * cos - y * sin,
          y: Number(origin.y || 0) + x * sin + y * cos,
        }));
      }
      function isMapToolActive() {
        return Boolean(state.goalPickMode || state.publishPointMode || state.initialPosePickMode);
      }
      function usesRvizMapView() {
        return Boolean(
          isMapToolActive()
          || state.goalPickDrag
          || state.initialPosePickDrag
          || state.mapPanDrag,
        );
      }
      function mapViewZoom() {
        return usesRvizMapView() ? (state.mapRvizView.zoom || 1) : state.mapZoom;
      }
      function defaultMapViewCenter() {
        const posePoint = pointFromPose(state.pose);
        if (posePoint) return { x: posePoint.x, y: posePoint.y };
        const corners = mapCorners(state.mapGrid);
        if (!corners.length) return { x: 0, y: 0 };
        return corners.reduce((acc, item) => ({ x: acc.x + item.x / corners.length, y: acc.y + item.y / corners.length }), { x: 0, y: 0 });
      }
      function getMapRvizViewCenter() {
        const center = state.mapRvizView && state.mapRvizView.center;
        if (center && Number.isFinite(Number(center.x)) && Number.isFinite(Number(center.y))) {
          return { x: Number(center.x), y: Number(center.y) };
        }
        return defaultMapViewCenter();
      }
      function resetRvizMapView(focusPose) {
        const posePoint = focusPose || pointFromPose(state.pose);
        state.mapRvizView = {
          center: posePoint ? { x: posePoint.x, y: posePoint.y } : defaultMapViewCenter(),
          zoom: 1,
        };
      }
      function centerRvizMapOnRobot() {
        const posePoint = pointFromPose(state.pose);
        if (!posePoint) return;
        if (!state.mapRvizView) state.mapRvizView = { center: null, zoom: 1 };
        state.mapRvizView.center = { x: posePoint.x, y: posePoint.y };
        requestMapDraw(false);
      }
      function cameraAnchor() {
        if (usesRvizMapView()) {
          const center = getMapRvizViewCenter();
          return { point: { x: center.x, y: center.y, z: 0 }, yaw: 0, locked: false, mapFixed: true };
        }
        const posePoint = pointFromPose(state.pose);
        if (posePoint) return { point: posePoint, yaw: planarYawFromPose(state.pose) || 0, locked: true, mapFixed: false };
        const fallback = defaultMapViewCenter();
        return { point: { x: fallback.x, y: fallback.y, z: 0 }, yaw: 0, locked: false, mapFixed: false };
      }
      function basePixelsPerMeterRviz() {
        const canvas = els.mapCanvas;
        const corners = mapCorners(state.mapGrid);
        if (!corners.length) return 60;
        let minX = Infinity;
        let maxX = -Infinity;
        let minY = Infinity;
        let maxY = -Infinity;
        for (const corner of corners) {
          minX = Math.min(minX, Number(corner.x));
          maxX = Math.max(maxX, Number(corner.x));
          minY = Math.min(minY, Number(corner.y));
          maxY = Math.max(maxY, Number(corner.y));
        }
        const spanX = Math.max(maxX - minX, 1);
        const spanY = Math.max(maxY - minY, 1);
        return Math.max(4, Math.min(canvas.width / (spanX * 1.12), canvas.height / (spanY * 1.12)));
      }
      function basePixelsPerMeter(anchor) {
        if (anchor && anchor.mapFixed) return basePixelsPerMeterRviz();
        const canvas = els.mapCanvas;
        const corners = mapCorners(state.mapGrid);
        if (!corners.length) return 60;
        let maxX = 1;
        let maxY = 1;
        const yaw = anchor.yaw || 0;
        const cos = Math.cos(yaw);
        const sin = Math.sin(yaw);
        for (const corner of corners) {
          const dx = Number(corner.x) - anchor.point.x;
          const dy = Number(corner.y) - anchor.point.y;
          const right = sin * dx - cos * dy;
          const up = cos * dx + sin * dy;
          maxX = Math.max(maxX, Math.abs(right));
          maxY = Math.max(maxY, Math.abs(up));
        }
        return Math.max(4, Math.min(canvas.width / (maxX * 2.16), canvas.height / (maxY * 2.16)));
      }
      function cameraScale(anchor) {
        return basePixelsPerMeter(anchor) * mapViewZoom();
      }
      function applyCameraTransform(ctx, anchor, scale) {
        const centerX = els.mapCanvas.width / 2;
        const centerY = els.mapCanvas.height / 2;
        if (anchor && anchor.mapFixed) {
          const ax = Number(anchor.point.x) || 0;
          const ay = Number(anchor.point.y) || 0;
          ctx.setTransform(scale, 0, 0, -scale, centerX - scale * ax, centerY + scale * ay);
          return;
        }
        const yaw = anchor.yaw || 0;
        const sin = Math.sin(yaw);
        const cos = Math.cos(yaw);
        const a = scale * sin;
        const b = -scale * cos;
        const c = -scale * cos;
        const d = -scale * sin;
        const e = centerX - a * anchor.point.x - c * anchor.point.y;
        const f = centerY - b * anchor.point.x - d * anchor.point.y;
        ctx.setTransform(a, b, c, d, e, f);
      }
      function mapToCanvas(point, anchorOverride, scaleOverride) {
        if (!point) return null;
        const anchor = anchorOverride || cameraAnchor();
        const scale = scaleOverride || cameraScale(anchor);
        const centerX = els.mapCanvas.width / 2;
        const centerY = els.mapCanvas.height / 2;
        const dx = Number(point.x) - Number(anchor.point.x);
        const dy = Number(point.y) - Number(anchor.point.y);
        if (anchor.mapFixed) {
          return { x: centerX + dx * scale, y: centerY - dy * scale };
        }
        const yaw = anchor.yaw || 0;
        return {
          x: centerX + (Math.sin(yaw) * dx - Math.cos(yaw) * dy) * scale,
          y: centerY - (Math.cos(yaw) * dx + Math.sin(yaw) * dy) * scale,
        };
      }
      function worldToCanvasMatrix(anchor, scale) {
        const centerX = els.mapCanvas.width / 2;
        const centerY = els.mapCanvas.height / 2;
        if (anchor && anchor.mapFixed) {
          const ax = Number(anchor.point.x) || 0;
          const ay = Number(anchor.point.y) || 0;
          return { a: scale, b: 0, c: 0, d: -scale, e: centerX - scale * ax, f: centerY + scale * ay };
        }
        const yaw = anchor.yaw || 0;
        const sin = Math.sin(yaw);
        const cos = Math.cos(yaw);
        const a = scale * sin;
        const b = -scale * cos;
        const c = -scale * cos;
        const d = -scale * sin;
        const e = centerX - a * anchor.point.x - c * anchor.point.y;
        const f = centerY - b * anchor.point.x - d * anchor.point.y;
        return { a, b, c, d, e, f };
      }
      function canvasToMap(px, py, anchorOverride) {
        const anchor = anchorOverride || cameraAnchor();
        const scale = cameraScale(anchor);
        if (!Number.isFinite(scale) || scale <= 0) {
          return { x: Number(anchor.point.x) || 0, y: Number(anchor.point.y) || 0 };
        }
        const centerX = els.mapCanvas.width / 2;
        const centerY = els.mapCanvas.height / 2;
        if (anchor.mapFixed) {
          return {
            x: Number(anchor.point.x) + (px - centerX) / scale,
            y: Number(anchor.point.y) - (py - centerY) / scale,
          };
        }
        const yaw = anchor.yaw || 0;
        const { a, b, c, d, e, f } = worldToCanvasMatrix(anchor, scale);
        const det = a * d - b * c;
        if (!Number.isFinite(det) || Math.abs(det) < 1e-12) {
          return { x: Number(anchor.point.x) || 0, y: Number(anchor.point.y) || 0 };
        }
        const sx = px - e;
        const sy = py - f;
        return {
          x: (d * sx - c * sy) / det,
          y: (-b * sx + a * sy) / det,
        };
      }
      function zoomRvizMapAt(pointer, factor) {
        const anchor = cameraAnchor();
        const scaleBefore = cameraScale(anchor);
        if (!Number.isFinite(scaleBefore) || scaleBefore <= 0) return;
        const world = canvasToMap(pointer.x, pointer.y, anchor);
        const nextZoom = Math.min(8, Math.max(0.25, mapViewZoom() * factor));
        if (!state.mapRvizView) state.mapRvizView = { center: null, zoom: 1 };
        state.mapRvizView.zoom = nextZoom;
        const anchorAfter = cameraAnchor();
        const scaleAfter = cameraScale(anchorAfter);
        const centerX = els.mapCanvas.width / 2;
        const centerY = els.mapCanvas.height / 2;
        state.mapRvizView.center = {
          x: world.x - (pointer.x - centerX) / scaleAfter,
          y: world.y + (pointer.y - centerY) / scaleAfter,
        };
      }
      function formatMapPoint(point) {
        if (!point) return "--";
        return `(${Number(point.x).toFixed(2)}, ${Number(point.y).toFixed(2)})`;
      }
      function planarDistanceMeters(a, b) {
        if (!a || !b) return null;
        const dx = Number(a.x) - Number(b.x);
        const dy = Number(a.y) - Number(b.y);
        if (!Number.isFinite(dx) || !Number.isFinite(dy)) return null;
        return Math.hypot(dx, dy);
      }
      function navGoalMatchesPending(targetPose, pending) {
        if (!targetPose || !pending) return false;
        const point = pointFromPose(targetPose);
        if (!point) return false;
        const dist = planarDistanceMeters(point, pending);
        return dist !== null && dist < 0.35;
      }
      function syncDebugNavGoalFromStream(navPayload) {
        const pending = state.pendingNavGoal;
        if (!pending || pending.status === "sending") return;
        const navState = getNestedState(navPayload, "navigation_state");
        if (!navState) return;
        const status = String(navState.status || "").toLowerCase();
        const metadata = navState.metadata || {};
        const targetPose = metadata.target_pose || null;
        const matches = navGoalMatchesPending(targetPose, pending);
        if (status === "running" || status === "planning") {
          if (!matches) return;
          const remain = navState.remaining_distance_m;
          const remainLabel = remain === null || remain === undefined ? "" : ` · 剩余 ${Number(remain).toFixed(2)}m`;
          updateGoalPickStatus(`Nav2 执行中 → ${formatMapPoint(pending)}${remainLabel}`, "ok");
          pending.status = "accepted";
          return;
        }
        if (status === "succeeded") {
          if (!matches) return;
          updateGoalPickStatus(`已到达 ${formatMapPoint(pending)}`, "ok");
          state.pendingNavGoal = null;
          requestMapDraw(false);
          return;
        }
        if (status === "failed" || status === "cancelled") {
          if (!matches && pending.status !== "pending" && pending.status !== "accepted") return;
          const message = navState.message || (status === "cancelled" ? "导航已取消" : "导航失败");
          const failureReason = String(metadata.failure_reason || "");
          if (
            failureReason === "nav2_not_ready"
            || message.includes("Nav2 未 ready")
            || message.includes("lifecycle 未全部 active")
            || message.includes("costmap 尚未首帧")
          ) {
            return;
          }
          pending.status = "error";
          pending.message = message;
          updateGoalPickStatus(`${message}（目标 ${formatMapPoint(pending)}）`, "error");
          requestMapDraw(false);
        }
      }
      function mapGoalUsesOfflineGrid() {
        const diagnostics = state.mapStream && state.mapStream.diagnostics ? state.mapStream.diagnostics : {};
        const sourceKind = String(diagnostics.source_kind || "").toLowerCase();
        const snapshotKind = String(diagnostics.map_snapshot_kind || "").toLowerCase();
        return sourceKind === "platform_map_version" || snapshotKind === "offline_checkpoint";
      }
      function canvasPointer(event) {
        const rect = els.mapCanvas.getBoundingClientRect();
        const scaleX = els.mapCanvas.width / Math.max(rect.width, 1);
        const scaleY = els.mapCanvas.height / Math.max(rect.height, 1);
        return {
          x: (event.clientX - rect.left) * scaleX,
          y: (event.clientY - rect.top) * scaleY,
        };
      }
      function activeMapName() {
        const diagnostics = state.mapStream && state.mapStream.diagnostics ? state.mapStream.diagnostics : {};
        const governance = state.latestGovernance || {};
        const overview = state.latestOverview || {};
        const overviewGovernance = overview.runtime_governance || {};
        return diagnostics.active_map_name
          || diagnostics.map_name
          || governance.active_map_name
          || overviewGovernance.active_map_name
          || null;
      }
      function navigationFrameId() {
        const poseFrame = state.pose && state.pose.frame_id;
        const gridFrame = state.mapGrid && state.mapGrid.frame_id;
        return normalizeFrameId(poseFrame || gridFrame, "map");
      }
      function updateGoalPickStatus(text, tone) {
        if (!els.mapGoalStatus) return;
        const value = String(text || "").trim();
        if (!value) {
          els.mapGoalStatus.hidden = true;
          els.mapGoalStatus.textContent = "";
          els.mapGoalStatus.className = "map-chip map-goal-chip";
          return;
        }
        els.mapGoalStatus.hidden = false;
        els.mapGoalStatus.textContent = value;
        els.mapGoalStatus.className = `map-chip map-goal-chip${tone ? ` ${tone}` : ""}`;
      }
      function syncMapToolUi() {
        const goalActive = Boolean(state.goalPickMode);
        const pointActive = Boolean(state.publishPointMode);
        const initialPoseActive = Boolean(state.initialPosePickMode);
        const toolActive = goalActive || pointActive || initialPoseActive;
        if (els.mapInitialPoseToggle) els.mapInitialPoseToggle.classList.toggle("active", initialPoseActive);
        if (els.mapGoalToggle) els.mapGoalToggle.classList.toggle("active", goalActive);
        if (els.mapPublishPointToggle) els.mapPublishPointToggle.classList.toggle("active", pointActive);
        if (els.mapCanvas) {
          els.mapCanvas.classList.toggle("goal-pick", toolActive);
          els.mapCanvas.classList.toggle("publish-point", pointActive && !goalActive && !initialPoseActive);
          els.mapCanvas.classList.toggle("initial-pose", initialPoseActive && !goalActive && !pointActive);
          els.mapCanvas.classList.toggle("pan-ready", toolActive && !state.mapPanDrag);
          els.mapCanvas.classList.toggle("panning", Boolean(state.mapPanDrag));
        }
        if (els.mapCenterRobot) els.mapCenterRobot.hidden = !toolActive;
        if (els.mapRvizHint) {
          els.mapRvizHint.hidden = !toolActive;
          if (initialPoseActive) {
            els.mapRvizHint.textContent = "2D Pose：左键拖拽设初始位姿 · 中/右键平移 · 滚轮缩放";
          } else if (goalActive) {
            els.mapRvizHint.textContent = "2D Goal：左键拖拽设目标 · 中/右键平移 · 滚轮缩放";
          } else if (pointActive) {
            els.mapRvizHint.textContent = "选点：左键发布 /clicked_point · 中/右键平移 · 滚轮缩放";
          }
        }
      }
      const syncGoalPickUi = syncMapToolUi;
      function goalPickYawFromDrag(drag) {
        const pointerDx = drag.pointerCurrent.x - drag.pointerStart.x;
        const pointerDy = drag.pointerCurrent.y - drag.pointerStart.y;
        if (Math.hypot(pointerDx, pointerDy) < 8) {
          return planarYawFromPose(state.pose) || 0;
        }
        const mapDx = drag.current.x - drag.start.x;
        const mapDy = drag.current.y - drag.start.y;
        return Math.atan2(mapDy, mapDx);
      }
      function formatNavGoalError(payload, status) {
        const code = String((payload && payload.code) || "").trim();
        const message = String((payload && payload.message) || "").trim();
        if (code === "localization_unavailable") {
          return "平台定位会话未就绪（与 go2.sh 模式无关）。mapping / explore / localization 任一模式均可导航，但需先 activate_map，且定位 session 进入 ready。";
        }
        if (code === "map_not_activated") {
          return "未激活地图。请先调用 activate_map 选择地图。";
        }
        if (code === "access_denied") {
          return "当前令牌无权调用 navigate_to_pose。";
        }
        if (code === "nav2_not_ready") {
          const waitingFor = payload && payload.details && payload.details.waiting_for;
          return waitingFor
            ? `Nav2 未就绪（等待 ${waitingFor}）。${message || ""}`
            : (message || "Nav2 未就绪，无法导航。");
        }
        if (code === "nav2_goal_rejected") {
          const details = payload && payload.details ? payload.details : {};
          const actionName = details.action_name ? ` action=${details.action_name}` : "";
          return `${message || "Nav2 拒绝目标。"}${actionName}`;
        }
        if (code === "nav2_provider_unavailable") {
          return message || "Nav2 action provider 未初始化。";
        }
        return message || `HTTP ${status}`;
      }
      async function dispatchNavGoal(mapPoint, yawRad) {
        if (!state.mapGrid) {
          updateGoalPickStatus("地图未显示，无法下发导航目标", "error");
          return;
        }
        if (!state.pose || !pointFromPose(state.pose)) {
          updateGoalPickStatus("定位位姿不可用，无法将点击坐标变换到 map 系", "error");
          return;
        }
        if (mapGoalUsesOfflineGrid()) {
          updateGoalPickStatus("当前为离线/缓存地图，与 Nav2 实时 costmap 可能不一致，建议等待实时流后再发目标", "error");
          return;
        }
        if (state.pendingNavGoal && state.pendingNavGoal.status === "sending") {
          updateGoalPickStatus("上一条 Nav2 目标仍在下发，请稍候", "pending");
          return;
        }
        const frameId = navigationFrameId();
        const mapName = activeMapName();
        const robotPoint = pointFromPose(state.pose);
        const distanceM = planarDistanceMeters(mapPoint, robotPoint);
        const body = {
          frame_id: frameId,
          x: Number(mapPoint.x),
          y: Number(mapPoint.y),
          z: 0,
          yaw_rad: Number(yawRad),
          accept_timeout_sec: 8,
        };
        if (mapName) body.map_name = mapName;
        state.pendingNavGoal = {
          x: body.x,
          y: body.y,
          yaw_rad: body.yaw_rad,
          frame_id: frameId,
          status: "sending",
        };
        const robotLabel = robotPoint ? formatMapPoint(robotPoint) : "--";
        const distanceLabel = distanceM === null ? "" : ` · 距离 ${distanceM.toFixed(2)}m`;
        updateGoalPickStatus(`下发 map 目标 ${formatMapPoint(mapPoint)} · 机器人 ${robotLabel}${distanceLabel} · frame=${frameId}`, "pending");
        requestMapDraw(false);
        const controller = new AbortController();
        const requestTimeout = window.setTimeout(() => controller.abort(), 12000);
        try {
          const response = await fetch(withAccessToken("/api/debug/navigation/goal"), {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(body),
            signal: controller.signal,
          });
          const payload = await response.json().catch(() => ({}));
          if (!response.ok) {
            throw new Error(formatNavGoalError(payload, response.status));
          }
          const accepted = payload.accepted === true;
          const pending = payload.pending === true;
          const apiDistance = payload.distance_m;
          const distanceSuffix = Number.isFinite(Number(apiDistance))
            ? ` · 距离 ${Number(apiDistance).toFixed(2)}m`
            : distanceLabel;
          state.pendingNavGoal = {
            x: body.x,
            y: body.y,
            yaw_rad: body.yaw_rad,
            frame_id: frameId,
            status: accepted ? "accepted" : (pending ? "pending" : "error"),
            goalId: payload.goal_id || null,
            actionName: payload.action_name || null,
          };
          const actionLabel = payload.action_name ? ` · ${payload.action_name}` : "";
          const goalLabel = `${formatMapPoint(mapPoint)}${distanceSuffix} · ${frameId}`;
          if (accepted) {
            updateGoalPickStatus(`Nav2 已接受 ${goalLabel}${actionLabel}`, "ok");
          } else if (pending) {
            updateGoalPickStatus(`Nav2 已发送 ${goalLabel}，等待接受（坐标与地图一致；若未移动请看导航面板失败原因）${actionLabel}`, "pending");
          } else {
            updateGoalPickStatus(payload.message || "Nav2 未接受目标", "error");
          }
        } catch (error) {
          const message = error && error.name === "AbortError"
            ? "Nav2 下发超时（12s），请确认 go2.sh / Nav2 action server 已启动。"
            : (error && error.message ? error.message : String(error));
          state.pendingNavGoal = {
            x: body.x,
            y: body.y,
            yaw_rad: body.yaw_rad,
            frame_id: frameId,
            status: "error",
            message,
          };
          updateGoalPickStatus(`下发失败：${message}`, "error");
        } finally {
          window.clearTimeout(requestTimeout);
        }
        requestMapDraw(false);
      }
      async function dispatchClickedPoint(mapPoint) {
        if (!state.mapGrid) {
          updateGoalPickStatus("地图未显示，无法发布选点", "error");
          return;
        }
        if (!state.pose || !pointFromPose(state.pose)) {
          updateGoalPickStatus("定位位姿不可用，无法将点击坐标变换到 map 系", "error");
          return;
        }
        const frameId = navigationFrameId();
        const robotPoint = pointFromPose(state.pose);
        const distanceM = planarDistanceMeters(mapPoint, robotPoint);
        const body = {
          frame_id: frameId,
          x: Number(mapPoint.x),
          y: Number(mapPoint.y),
          z: 0,
        };
        const distanceLabel = distanceM === null ? "" : ` · 距机器人 ${distanceM.toFixed(2)}m`;
        updateGoalPickStatus(`发布 /clicked_point ${formatMapPoint(mapPoint)}${distanceLabel} · ${frameId}`, "pending");
        try {
          const response = await fetch(withAccessToken("/api/debug/map/clicked_point"), {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(body),
          });
          const payload = await response.json().catch(() => ({}));
          if (!response.ok) {
            const message = String((payload && payload.message) || `HTTP ${response.status}`);
            throw new Error(message);
          }
          state.clickedPoints = [
            {
              x: body.x,
              y: body.y,
              z: body.z,
              frame_id: frameId,
              at: Date.now(),
            },
            ...(state.clickedPoints || []),
          ].slice(0, 30);
          const topic = payload.topic || "/clicked_point";
          updateGoalPickStatus(`已发布 ${topic} ${formatMapPoint(mapPoint)}${distanceLabel} · ${frameId}`, "ok");
        } catch (error) {
          const message = error && error.message ? error.message : String(error);
          updateGoalPickStatus(`选点发布失败：${message}`, "error");
        }
        requestMapDraw(false);
      }
      async function dispatchInitialPose(mapPoint, yawRad) {
        if (!state.mapGrid) {
          updateGoalPickStatus("地图未显示，无法发布初始位姿", "error");
          return;
        }
        const frameId = navigationFrameId();
        const robotPoint = pointFromPose(state.pose);
        const distanceM = planarDistanceMeters(mapPoint, robotPoint);
        const body = {
          frame_id: frameId,
          x: Number(mapPoint.x),
          y: Number(mapPoint.y),
          z: 0,
          yaw_rad: Number(yawRad),
        };
        const distanceLabel = distanceM === null ? "" : ` · 距当前位姿 ${distanceM.toFixed(2)}m`;
        updateGoalPickStatus(`发布 /initialpose ${formatMapPoint(mapPoint)} yaw=${(body.yaw_rad * 180 / Math.PI).toFixed(0)}°${distanceLabel} · ${frameId}`, "pending");
        try {
          const response = await fetch(withAccessToken("/api/debug/map/initial_pose"), {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(body),
          });
          const payload = await response.json().catch(() => ({}));
          if (!response.ok) {
            const message = String((payload && payload.message) || `HTTP ${response.status}`);
            throw new Error(message);
          }
          state.initialPoses = [
            {
              x: body.x,
              y: body.y,
              z: body.z,
              yaw_rad: body.yaw_rad,
              frame_id: frameId,
              at: Date.now(),
            },
            ...(state.initialPoses || []),
          ].slice(0, 20);
          const topic = payload.topic || "/initialpose";
          updateGoalPickStatus(`已发布 ${topic} ${formatMapPoint(mapPoint)} yaw=${(body.yaw_rad * 180 / Math.PI).toFixed(0)}°${distanceLabel} · ${frameId}`, "ok");
        } catch (error) {
          const message = error && error.message ? error.message : String(error);
          updateGoalPickStatus(`初始位姿发布失败：${message}`, "error");
        }
        requestMapDraw(false);
      }
      function drawOrientedGoal(ctx, anchor, scale, point, yawRad, colors, pixels) {
        if (!point || !Number.isFinite(Number(point.x)) || !Number.isFinite(Number(point.y))) return;
        ctx.save();
        applyCameraTransform(ctx, anchor, scale);
        ctx.translate(Number(point.x), Number(point.y));
        ctx.rotate(Number(yawRad) || 0);
        const shaft = Math.max(0.35, 14 / scale);
        const head = Math.max(0.18, 7 / scale);
        ctx.strokeStyle = colors.stroke;
        ctx.fillStyle = colors.fill;
        ctx.lineWidth = Math.max(1 / scale, pixels / scale);
        ctx.lineCap = "round";
        ctx.lineJoin = "round";
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(shaft, 0);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(shaft, 0);
        ctx.lineTo(shaft - head, head * 0.55);
        ctx.lineTo(shaft - head, -head * 0.55);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(0, 0, Math.max(0.08, 4 / scale), 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
      }
      function setWorldLineStyle(ctx, scale, color, pixels) {
        ctx.strokeStyle = color;
        ctx.lineWidth = Math.max(1 / scale, pixels / scale);
        ctx.lineCap = "round";
        ctx.lineJoin = "round";
      }
      function requestMapDraw(baseDirty) {
        if (baseDirty) state.mapBaseCache = null;
        if (state.mapDrawFrame) return;
        state.mapDrawFrame = window.requestAnimationFrame(() => {
          state.mapDrawFrame = null;
          drawMap();
        });
      }
      function mapGridDataSignature(grid) {
        const data = grid && Array.isArray(grid.data) ? grid.data : [];
        if (!data.length) return "empty";
        let hash = 2166136261;
        for (let index = 0; index < data.length; index += 1) {
          hash ^= (Number(data[index]) + 129) & 255;
          hash = Math.imul(hash, 16777619) >>> 0;
        }
        return `${data.length}:${hash.toString(16)}`;
      }
      function mapGridContentKey(grid, envelope, diagnostics) {
        if (!grid) return "missing";
        const origin = originPosition(grid);
        const revision = (diagnostics && (diagnostics.occupancy_content_hash || diagnostics.content_hash || diagnostics.grid_hash || diagnostics.occupancy_content_revision || diagnostics.content_revision || diagnostics.revision)) || (grid.metadata && (grid.metadata.occupancy_content_hash || grid.metadata.content_hash || grid.metadata.grid_hash || grid.metadata.occupancy_content_revision || grid.metadata.content_revision || grid.metadata.revision)) || envelope.revision || grid.revision || mapGridDataSignature(grid);
        return [grid.frame_id || envelope.frame_id || "", grid.width, grid.height, gridResolution(grid), Number(origin.x || 0), Number(origin.y || 0), originYaw(grid), revision].join("|");
      }
      function normalizeStatusValue(value) {
        const raw = value && typeof value === "object" && "value" in value ? value.value : value;
        return String(raw || "").trim().toLowerCase().replace(/^(task|navigation|exploration)\./, "");
      }
      function isActiveSpatialStatus(value) {
        return ACTIVE_SPATIAL_STATUSES.has(normalizeStatusValue(value));
      }
      function isTerminalSpatialStatus(value) {
        return ["succeeded", "failed", "cancelled", "timeout", "idle"].includes(normalizeStatusValue(value));
      }
      function getNestedState(payload, key) {
        if (!payload) return {};
        if (payload[key] && typeof payload[key] === "object") return payload[key];
        return payload;
      }
      function localizationPoseStreamReason() {
        const envelope = state.lastLocalizationEnvelope;
        if (!envelope || statusOf(envelope) !== "available") return null;
        return pointFromPose(state.pose) ? "localization_pose_stream" : null;
      }
      function goalInteractionReason() {
        if (state.goalPickMode || state.publishPointMode || state.initialPosePickMode || state.pendingNavGoal) {
          return "goal_pick";
        }
        return null;
      }
      function activeSpatialTaskReason() {
        for (const task of state.taskActiveById.values()) {
          const capability = String(task.capability_name || task.name || "").trim();
          const taskState = normalizeStatusValue(task.state || task.status);
          if (!capability || isTerminalSpatialStatus(taskState)) continue;
          if (ACTIVE_SPATIAL_CAPABILITIES.has(capability) || capability.startsWith("navigate_") || capability.includes("explore")) {
            return `task:${capability}`;
          }
        }
        return null;
      }
      function backendSpatialProcessReason() {
        const geoRuntime = state.latestGovernance && state.latestGovernance.metadata && state.latestGovernance.metadata.geometry_runtime;
        const processes = geoRuntime && geoRuntime.processes;
        const external = processes && processes.external_slam_nav2;
        if (!external || external.running !== true) return null;
        const cmd = String(external.command_display || (Array.isArray(external.command) ? external.command.join(" ") : external.command) || "").toLowerCase();
        if (cmd.includes("--explore") || cmd.includes("--exploration")) return "exploration";
        if (cmd.includes("--localization") || cmd.includes("--loc")) return "backend_localization";
        if (cmd.includes("--mapping") || cmd.includes("--new")) return "mapping";
        return "backend_geometry";
      }
      function activeSpatialContextReason() {
        const taskReason = activeSpatialTaskReason();
        if (taskReason) return taskReason;
        const navPayload = state.latestStreamFrames["navigation.state"] && state.latestStreamFrames["navigation.state"].payload;
        const navState = getNestedState(navPayload, "navigation_state");
        if (isActiveSpatialStatus(navState.status) || (navState.current_goal_id && !isTerminalSpatialStatus(navState.status))) return "navigation";
        const explorationPayload = state.latestStreamFrames["exploration.state"] && state.latestStreamFrames["exploration.state"].payload;
        const explorationState = getNestedState(explorationPayload, "exploration_state");
        if (isActiveSpatialStatus(explorationState.status) || (explorationState.current_request_id && !isTerminalSpatialStatus(explorationState.status))) return "exploration";
        const backendReason = backendSpatialProcessReason();
        if (backendReason) return backendReason;
        const localizationReason = activeLocalizationSessionReason();
        if (localizationReason) return localizationReason;
        const poseStreamReason = localizationPoseStreamReason();
        if (poseStreamReason) return poseStreamReason;
        const goalReason = goalInteractionReason();
        if (goalReason) return goalReason;
        return null;
      }
      function activeLocalizationSessionReason() {
        const session = state.latestGovernance && state.latestGovernance.localization_session;
        if (!session || session.available !== true) return null;
        const status = normalizeStatusValue(session.status);
        if (!status || !ACTIVE_LOCALIZATION_STATUSES.has(status)) return null;
        return "localization";
      }
      function loadedMapVersionShouldDisplay(diagnostics) {
        if (!diagnostics) return false;
        const mapName = String(diagnostics.active_map_name || diagnostics.map_name || "").trim();
        if (!mapName) return false;
        const versionId = String(
          diagnostics.platform_map_version_id ||
          diagnostics.seed_map_version_id ||
          diagnostics.map_version_id ||
          ""
        ).trim();
        const snapshotKind = String(diagnostics.map_snapshot_kind || "").toLowerCase();
        const sourceKind = String(diagnostics.source_kind || "").toLowerCase();
        return Boolean(
          versionId ||
          snapshotKind === "offline_checkpoint" ||
          sourceKind === "platform_map_version" ||
          diagnostics.checkpoint_ready === true ||
          diagnostics.geometry_ready === true
        );
      }
      function firstFiniteNumber(...values) {
        for (const value of values) {
          if (value === undefined || value === null || value === "") continue;
          const number = Number(value);
          if (Number.isFinite(number)) return number;
        }
        return null;
      }
      function timestampIsFresh(...values) {
        const now = Date.now();
        for (const value of values) {
          if (!value) continue;
          const parsed = Date.parse(value);
          if (!Number.isFinite(parsed)) continue;
          const age = now - parsed;
          if (age >= -5000 && age <= MAP_DISPLAY_FRESH_MS) return true;
        }
        return false;
      }
      function diagnosticsAgeIsFresh(envelope, diagnostics) {
        const streamAgeMs = firstFiniteNumber(
          diagnostics && diagnostics.stream_age_ms,
          diagnostics && diagnostics.cache_age !== undefined ? Number(diagnostics.cache_age) * 1000 : null,
          diagnostics && diagnostics.source_age !== undefined ? Number(diagnostics.source_age) * 1000 : null
        );
        if (streamAgeMs !== null && streamAgeMs <= MAP_DISPLAY_FRESH_MS) return true;
        return timestampIsFresh(
          envelope && envelope.source_timestamp,
          diagnostics && diagnostics.source_timestamp,
          diagnostics && diagnostics.last_message_at,
          diagnostics && diagnostics.latest_occupancy_received_at,
          diagnostics && diagnostics.latest_occupancy_timestamp
        );
      }
      function mapSnapshotIsOffline(diagnostics) {
        const sourceKind = String(diagnostics && diagnostics.source_kind || "").toLowerCase();
        const snapshotKind = String(diagnostics && diagnostics.map_snapshot_kind || "").toLowerCase();
        return sourceKind === "platform_map_version" || snapshotKind === "offline_checkpoint";
      }
      function hasLiveMapFlag(diagnostics, streamBridge) {
        if (!diagnostics) return false;
        if (mapSnapshotIsOffline(diagnostics)) return false;
        const sourceKind = String(diagnostics.source_kind || "").toLowerCase();
        const snapshotKind = String(diagnostics.map_snapshot_kind || "").toLowerCase();
        const bridgeMode = String((streamBridge && (streamBridge.status || streamBridge.state || streamBridge.mode)) || "").toLowerCase();
        return Boolean(
          sourceKind === "platform_live_map_provider" ||
          snapshotKind === "live_streaming" ||
          diagnostics.source_alive === true ||
          diagnostics.streaming === true ||
          diagnostics.live_mapping_ready === true ||
          diagnostics.rtabmap_live_streaming_ready === true ||
          bridgeMode === "streaming" ||
          (streamBridge && streamBridge.streaming === true)
        );
      }
      function hasPositiveMapCount(diagnostics) {
        const count = firstFiniteNumber(
          diagnostics && diagnostics.message_count,
          diagnostics && diagnostics.occupancy_message_count,
          diagnostics && diagnostics.occupancy_sequence,
          diagnostics && diagnostics.update_sequence,
          diagnostics && diagnostics.sequence
        );
        return count === null || count > 0;
      }
      function shouldDisplayMapFrame(envelope, diagnostics, streamBridge, grid, status, cleared) {
        if (goalInteractionReason()) {
          if (grid && Array.isArray(grid.data) && grid.width && grid.height) {
            return { display: true, reason: "goal_pick" };
          }
        }
        if (cleared) return { display: false, status: "cleared", message: mapClearedMessage(diagnostics) };
        if (!grid || !Array.isArray(grid.data) || !grid.width || !grid.height) {
          return { display: false, status: status === "error" ? "error" : "missing", message: errorReason(envelope, "无 occupancy_grid，占据栅格不可用。") };
        }
        const activeReason = activeSpatialContextReason();
        if (activeReason) return { display: true, reason: activeReason };
        if (status === "available" && loadedMapVersionShouldDisplay(diagnostics)) {
          return { display: true, reason: "loaded_map_version" };
        }
        if (status !== "available") {
          return { display: false, status: status || "stale", message: status === "stale" ? "地图数据已过期，已隐藏缓存帧。" : errorReason(envelope, MAP_DISPLAY_IDLE_MESSAGE) };
        }
        if (hasLiveMapFlag(diagnostics, streamBridge) && hasPositiveMapCount(diagnostics) && diagnosticsAgeIsFresh(envelope, diagnostics || {})) {
          return { display: true, reason: "live_map_stream" };
        }
        return { display: false, status: "missing", message: MAP_DISPLAY_IDLE_MESSAGE };
      }
      function computeMapDisplayDecision(envelope) {
        if (!envelope) return { display: false, status: "missing", message: MAP_DISPLAY_IDLE_MESSAGE };
        const data = envelope.data || {};
        const grid = data.occupancy_grid || (data.map_snapshot && data.map_snapshot.occupancy_grid) || null;
        const diagnostics = data.diagnostics || data.metadata || (data.map_snapshot && data.map_snapshot.metadata) || {};
        const streamBridge = diagnostics.stream_bridge || {};
        const status = statusOf(envelope);
        const cleared = isMapClearedEnvelope(envelope, diagnostics, status);
        return shouldDisplayMapFrame(envelope, diagnostics, streamBridge, grid, status, cleared);
      }
      function shouldDisplaySpatialPose() {
        if (activeSpatialContextReason()) return true;
        return computeMapDisplayDecision(state.lastMapEnvelope).display === true;
      }
      function clearLocalizationDisplay(message, status) {
        state.pose = null;
        state.trajectory = [];
        setPill(els.localizationStatus, status || "missing");
        setKv(els.localizationMeta, [
          ["显示状态", "hidden"],
          ["reason", message || LOCALIZATION_DISPLAY_IDLE_MESSAGE],
          ["frame", "--"],
          ["pose", "--"],
          ["yaw", "--"],
          ["trajectory_count", 0],
        ]);
        setPill(els.poseStatus, status || "missing");
        setKv(els.missionPoseMeta, [
          ["frame", "--"],
          ["position", "--"],
          ["yaw", "--"],
          ["trajectory", 0],
        ]);
      }
      function clearMapOverlayState() {
        state.frontiers = [];
        state.goalPose = null;
        state.navigationPath = [];
      }
      function clearMapDisplay(message, status, diagnostics, streamBridge, envelope, grid) {
        if (state.goalPickMode || state.publishPointMode || state.initialPosePickMode || state.pendingNavGoal) {
          els.mapEmpty.hidden = false;
          els.mapEmpty.textContent = message || MAP_DISPLAY_IDLE_MESSAGE;
          return;
        }
        state.mapGrid = null;
        state.mapGridContentKey = "";
        state.mapBaseCache = null;
        state.mapViewport = null;
        if (!activeSpatialContextReason()) clearMapOverlayState();
        setPill(els.mapStatus, status || "missing");
        els.mapEmpty.hidden = false;
        els.mapEmpty.textContent = message || MAP_DISPLAY_IDLE_MESSAGE;
        requestMapDraw(true);
        const displayDiagnostics = { ...(diagnostics || {}), display_suppressed_reason: message || MAP_DISPLAY_IDLE_MESSAGE };
        setKv(els.mapMeta, [
          ["地图名称", displayDiagnostics.active_map_name || displayDiagnostics.map_name || (envelope && envelope.frame_id) || (grid && grid.frame_id) || "--"],
          ["总栅格数", grid ? `${grid.width * grid.height}` : "--"],
          ["显示状态", "hidden"],
          ["reason", message || MAP_DISPLAY_IDLE_MESSAGE],
          ["source_timestamp", shortTime(envelope && envelope.source_timestamp)],
          ["revision", (envelope && envelope.revision) || displayDiagnostics.content_revision],
          ["sequence", (envelope && envelope.sequence) || displayDiagnostics.message_count],
          ["streaming", mapStreamLabel(status || "missing", displayDiagnostics, streamBridge || {})],
          ["latency", fmt(envelope && envelope.latency_ms, " ms")],
          ["前端接收", shortTime(envelope && envelope.__receivedAt)],
        ]);
        updateMapStreamChip(status || "missing", displayDiagnostics, streamBridge || {}, envelope && envelope.source_timestamp);
        reevaluateCachedLocalizationDisplay();
      }
      function ensureMapBaseCanvas(ctx, grid) {
        const width = Number(grid.width);
        const height = Number(grid.height);
        if (state.mapBaseCache && state.mapBaseCache.key === state.mapGridContentKey && state.mapBaseCache.width === width && state.mapBaseCache.height === height) return state.mapBaseCache.canvas;
        const image = ctx.createImageData(width, height);
        for (let row = 0; row < height; row += 1) {
          for (let col = 0; col < width; col += 1) {
            const sourceIndex = row * width + col;
            const targetIndex = ((height - 1 - row) * width + col) * 4;
            const value = Number(grid.data[sourceIndex]);
            let color = CANVAS_THEME.mapUnknown;
            if (value >= 65) color = CANVAS_THEME.mapOccupied;
            else if (value >= 0) color = CANVAS_THEME.mapFree;
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
        state.mapBaseCache = { key: state.mapGridContentKey, width, height, canvas: offscreen };
        return offscreen;
      }
      function drawMap() {
        syncMapCanvasSize();
        const canvas = els.mapCanvas;
        const ctx = canvas.getContext("2d");
        ctx.setTransform(1, 0, 0, 1, 0, 0);
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.fillStyle = CANVAS_THEME.mapBackground;
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        const grid = state.mapGrid;
        const anchor = cameraAnchor();
        const scale = cameraScale(anchor);
        state.mapViewport = { centerX: canvas.width / 2, centerY: canvas.height / 2, scale, zoom: state.mapZoom, robotLocked: anchor.locked };
        if (els.mapZoomValue) els.mapZoomValue.textContent = `${Math.round(mapViewZoom() * 100)}%`;
        if (!grid || !Array.isArray(grid.data) || !grid.width || !grid.height) return;
        const width = Number(grid.width);
        const height = Number(grid.height);
        const offscreen = ensureMapBaseCanvas(ctx, grid);
        ctx.save();
        applyCameraTransform(ctx, anchor, scale);
        const origin = originPosition(grid);
        const resolution = gridResolution(grid);
        ctx.translate(Number(origin.x || 0), Number(origin.y || 0));
        ctx.rotate(originYaw(grid));
        ctx.scale(resolution, -resolution);
        ctx.imageSmoothingEnabled = false;
        ctx.drawImage(offscreen, 0, -height, width, height);
        ctx.restore();
        drawTrajectory(ctx, anchor, scale);
        drawNavigationPath(ctx, anchor, scale);
        drawGoal(ctx, anchor, scale);
        drawGoalPickPreview(ctx, anchor, scale);
        drawInitialPosePickPreview(ctx, anchor, scale);
        drawInitialPoses(ctx, anchor, scale);
        drawClickedPoints(ctx, anchor, scale);
        drawFrontiers(ctx, anchor, scale);
        drawRobot(ctx, anchor);
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
      function drawWorldPoints(ctx, anchor, scale, points, color, pixels) {
        if (!points || points.length < 2) return;
        ctx.save();
        applyCameraTransform(ctx, anchor, scale);
        setWorldLineStyle(ctx, scale, color, pixels);
        ctx.beginPath();
        let started = false;
        for (const point of points) {
          if (!point) continue;
          if (!started) { ctx.moveTo(point.x, point.y); started = true; }
          else ctx.lineTo(point.x, point.y);
        }
        if (started) ctx.stroke();
        ctx.restore();
      }
      function drawFrontiers(ctx, anchor, scale) {
        if (!state.frontiers || !state.frontiers.length) return;
        ctx.save();
        applyCameraTransform(ctx, anchor, scale);
        ctx.fillStyle = CANVAS_THEME.frontier;
        ctx.strokeStyle = CANVAS_THEME.frontierStroke;
        ctx.lineWidth = Math.max(1 / scale, 1.4 / scale);
        for (const frontier of state.frontiers) {
          const point = pointFromFrontier(frontier);
          if (!point) continue;
          ctx.beginPath();
          ctx.arc(point.x, point.y, Math.max(0.08, 5 / scale), 0, Math.PI * 2);
          ctx.fill();
          ctx.stroke();
        }
        ctx.restore();
      }
      function drawTrajectory(ctx, anchor, scale) {
        const points = (state.trajectory || []).map(pointFromPose).filter(Boolean);
        drawWorldPoints(ctx, anchor, scale, points, CANVAS_THEME.trajectory, 2.5);
      }
      function drawNavigationPath(ctx, anchor, scale) {
        drawWorldPoints(ctx, anchor, scale, state.navigationPath, CANVAS_THEME.navPath, 3);
      }
      function drawGoal(ctx, anchor, scale) {
        const pending = state.pendingNavGoal;
        if (pending && pending.status !== "error") {
          drawOrientedGoal(
            ctx,
            anchor,
            scale,
            { x: pending.x, y: pending.y },
            pending.yaw_rad,
            { fill: CANVAS_THEME.goalPick, stroke: CANVAS_THEME.goalPickStroke },
            2.8,
          );
        }
        const point = pointFromPose(state.goalPose);
        if (!point) return;
        const yaw = yawFromPose(state.goalPose);
        if (Number.isFinite(yaw)) {
          drawOrientedGoal(
            ctx,
            anchor,
            scale,
            point,
            yaw,
            { fill: CANVAS_THEME.goal, stroke: CANVAS_THEME.goalStroke },
            2.4,
          );
          return;
        }
        ctx.save();
        applyCameraTransform(ctx, anchor, scale);
        ctx.strokeStyle = CANVAS_THEME.goalStroke;
        ctx.fillStyle = CANVAS_THEME.goal;
        ctx.lineWidth = Math.max(1 / scale, 2 / scale);
        const radius = Math.max(0.12, 7 / scale);
        ctx.beginPath();
        ctx.arc(point.x, point.y, radius, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        ctx.restore();
      }
      function drawClickedPoints(ctx, anchor, scale) {
        const points = state.clickedPoints || [];
        if (!points.length) return;
        ctx.save();
        applyCameraTransform(ctx, anchor, scale);
        ctx.fillStyle = CANVAS_THEME.clickedPoint;
        ctx.strokeStyle = CANVAS_THEME.clickedPointStroke;
        for (const point of points) {
          if (!point || !Number.isFinite(Number(point.x)) || !Number.isFinite(Number(point.y))) continue;
          const radius = Math.max(0.1, 6 / scale);
          ctx.lineWidth = Math.max(1 / scale, 2 / scale);
          ctx.beginPath();
          ctx.arc(Number(point.x), Number(point.y), radius, 0, Math.PI * 2);
          ctx.fill();
          ctx.stroke();
        }
        ctx.restore();
      }
      function drawGoalPickPreview(ctx, anchor, scale) {
        const drag = state.goalPickDrag;
        if (!drag || !drag.start) return;
        const previewAnchor = drag.anchor || anchor;
        const previewScale = drag.anchor ? cameraScale(previewAnchor) : scale;
        const yaw = goalPickYawFromDrag(drag);
        drawOrientedGoal(
          ctx,
          previewAnchor,
          previewScale,
          drag.start,
          yaw,
          { fill: CANVAS_THEME.goalPick, stroke: CANVAS_THEME.goalPickStroke },
          3,
        );
      }
      function drawInitialPosePickPreview(ctx, anchor, scale) {
        const drag = state.initialPosePickDrag;
        if (!drag || !drag.start) return;
        const previewAnchor = drag.anchor || anchor;
        const previewScale = drag.anchor ? cameraScale(previewAnchor) : scale;
        const yaw = goalPickYawFromDrag(drag);
        drawOrientedGoal(
          ctx,
          previewAnchor,
          previewScale,
          drag.start,
          yaw,
          { fill: CANVAS_THEME.initialPosePick, stroke: CANVAS_THEME.initialPosePickStroke },
          3,
        );
      }
      function drawInitialPoses(ctx, anchor, scale) {
        const poses = state.initialPoses || [];
        if (!poses.length) return;
        for (const pose of poses) {
          if (!pose || !Number.isFinite(Number(pose.x)) || !Number.isFinite(Number(pose.y))) continue;
          drawOrientedGoal(
            ctx,
            anchor,
            scale,
            { x: pose.x, y: pose.y },
            Number(pose.yaw_rad) || 0,
            { fill: CANVAS_THEME.initialPosePick, stroke: CANVAS_THEME.initialPosePickStroke },
            2.4,
          );
        }
      }
      function drawRobot(ctx, anchor) {
        const posePoint = pointFromPose(state.pose);
        if (!posePoint) return;
        const locked = anchor && anchor.locked === true;
        ctx.save();
        ctx.setTransform(1, 0, 0, 1, 0, 0);
        if (locked) {
          ctx.translate(els.mapCanvas.width / 2, els.mapCanvas.height / 2);
        } else {
          const canvasPoint = mapToCanvas(posePoint);
          if (!canvasPoint) {
            ctx.restore();
            return;
          }
          ctx.translate(canvasPoint.x, canvasPoint.y);
          const yaw = planarYawFromPose(state.pose) || 0;
          // 北向 map 视图：屏幕 Y 向上对应 world +Y，三角形默认指屏幕上方，需用 π/2 - yaw（不是 yaw + π/2）
          ctx.rotate(Math.PI / 2 - yaw);
        }
        ctx.fillStyle = CANVAS_THEME.robot;
        ctx.strokeStyle = CANVAS_THEME.robotStroke;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(0, -14);
        ctx.lineTo(-9, 9);
        ctx.lineTo(0, 5);
        ctx.lineTo(9, 9);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        ctx.restore();
      }
      function updateMap(envelope) {
        state.lastMapEnvelope = envelope;
        const status = statusOf(envelope);
        const data = envelope.data || {};
        const grid = data.occupancy_grid || (data.map_snapshot && data.map_snapshot.occupancy_grid) || null;
        const diagnostics = data.diagnostics || data.metadata || (data.map_snapshot && data.map_snapshot.metadata) || {};
        const streamBridge = diagnostics.stream_bridge || {};
        const cleared = isMapClearedEnvelope(envelope, diagnostics, status);
        state.mapStream = { diagnostics, streamBridge, revision: envelope.revision, sequence: envelope.sequence, timestamp: envelope.source_timestamp, cleared };
        const displayDecision = shouldDisplayMapFrame(envelope, diagnostics, streamBridge, grid, status, cleared);
        if (!displayDecision.display) {
          clearMapDisplay(displayDecision.message, displayDecision.status || (cleared ? "cleared" : status), diagnostics, streamBridge, envelope, grid);
          return;
        }
        setPill(els.mapStatus, status);
        if (grid && Array.isArray(grid.data)) {
          const nextContentKey = mapGridContentKey(grid, envelope, diagnostics);
          const baseDirty = state.mapGridContentKey !== nextContentKey;
          state.mapGrid = grid;
          state.mapGridContentKey = nextContentKey;
          els.mapEmpty.hidden = status === "available";
          els.mapEmpty.textContent = status === "stale" ? "地图数据已过期，保留最后一帧。" : "";
          requestMapDraw(baseDirty);
        }
        setKv(els.mapMeta, [
          ["地图名称", diagnostics.active_map_name || diagnostics.map_name || envelope.frame_id || (grid && grid.frame_id) || "--"],
          ["总栅格数", grid ? `${grid.width * grid.height}` : "--"],
          ["source_timestamp", shortTime(envelope.source_timestamp)],
          ["revision", envelope.revision || diagnostics.content_revision],
          ["sequence", envelope.sequence || diagnostics.message_count],
          ["streaming", cleared ? "已清空" : mapStreamLabel(status, diagnostics, streamBridge)],
          ["latency", fmt(envelope.latency_ms, " ms")],
          ["前端接收", shortTime(envelope.__receivedAt)],
        ]);
        updateMapStreamChip(cleared ? "cleared" : status, diagnostics, streamBridge, envelope.source_timestamp);
      }
      function isMapClearedEnvelope(envelope, diagnostics, status) {
        if (status === "cleared") return true;
        const eventType = diagnostics && (diagnostics.event_type || diagnostics.suppressed_live_map_reason);
        if (typeof eventType === "string" && (eventType === "catalog_cleared" || eventType === "workspace_cleared")) return true;
        return false;
      }
      function mapClearedMessage(diagnostics) {
        const cleared = diagnostics && (diagnostics.cleared_map_name || diagnostics.previous_active_map_name);
        const reason = diagnostics && diagnostics.reason;
        if (cleared && reason) return `地图 ${cleared} 已清空（${reason}）。`;
        if (cleared) return `地图 ${cleared} 已清空。`;
        if (reason) return `地图已清空（${reason}）。`;
        return "地图已清空，等待新工作区激活。";
      }
      function mapStreamLabel(status, diagnostics, streamBridge) {
        if (diagnostics && diagnostics.display_suppressed_reason) return "缓存地图已隐藏";
        if (loadedMapVersionShouldDisplay(diagnostics)) return "已载入地图版本";
        const messageCount = diagnostics && (diagnostics.message_count || diagnostics.update_sequence || diagnostics.sequence);
        const revision = diagnostics && (diagnostics.content_revision || diagnostics.occupancy_content_revision || diagnostics.revision);
        const delta = diagnostics && diagnostics.message_count_delta;
        const bridgeStatus = streamBridge && (streamBridge.status || streamBridge.state || streamBridge.mode);
        if (!messageCount) return status === "available" ? "首帧已缓存" : "等待首帧";
        const live = status === "available" && (delta === undefined || delta === null || Number(delta) > 0 || diagnostics.live_mapping_ready === true || diagnostics.source_alive === true);
        return `${live ? "实时流" : "离线资产/缓存"} rev=${safeText(revision)} seq=${safeText(messageCount)} bridge=${safeText(bridgeStatus)}`;
      }
      function updateMapStreamChip(status, diagnostics, streamBridge, timestamp) {
        if (!els.mapStreamStatus) return;
        els.mapStreamStatus.textContent = `${mapStreamLabel(status, diagnostics, streamBridge)} · ${shortTime(timestamp)}`;
      }
      function updateMapHeartbeat(envelope) {
        state.lastMapHeartbeatEnvelope = envelope;
        const status = statusOf(envelope);
        const heartbeat = envelope.data || {};
        const diagnostics = { ...((state.mapStream && state.mapStream.diagnostics) || {}), ...(envelope.metadata || {}), ...heartbeat };
        const streamBridge = diagnostics.stream_bridge || {};
        const cleared = isMapClearedEnvelope(envelope, diagnostics, status);
        const grid = cleared ? null : state.mapGrid;
        state.mapStream = { diagnostics, streamBridge, revision: envelope.revision || diagnostics.content_revision, sequence: envelope.sequence || diagnostics.update_sequence, timestamp: envelope.source_timestamp, cleared };
        const displayDecision = shouldDisplayMapFrame(envelope, diagnostics, streamBridge, grid, status, cleared);
        if (!displayDecision.display) {
          clearMapDisplay(displayDecision.message, displayDecision.status || (cleared ? "cleared" : status), diagnostics, streamBridge, envelope, grid);
          return;
        }
        setPill(els.mapStatus, status);
        if (grid && Array.isArray(grid.data)) {
          els.mapEmpty.hidden = true;
          els.mapEmpty.textContent = "";
        } else {
          els.mapEmpty.hidden = false;
          els.mapEmpty.textContent = "地图源在线，等待内容更新帧。";
        }
        setKv(els.mapMeta, [
          ["frame", envelope.frame_id || diagnostics.frame_id || (grid && grid.frame_id)],
          ["size", grid ? `${grid.width} x ${grid.height}` : "--"],
          ["resolution", grid ? `${grid.resolution_m} m` : "--"],
          ["source_timestamp", shortTime(envelope.source_timestamp)],
          ["revision", envelope.revision || diagnostics.content_revision],
          ["sequence", envelope.sequence || diagnostics.update_sequence || diagnostics.message_count],
          ["streaming", cleared ? "已清空" : mapStreamLabel(status, diagnostics, streamBridge)],
          ["latency", fmt(envelope.latency_ms, " ms")],
          ["stale", envelope.stale === true ? "true" : "false"],
          ["前端接收", shortTime(envelope.__receivedAt)],
        ]);
        updateMapStreamChip(status, diagnostics, streamBridge, envelope.source_timestamp);
      }
      function reevaluateCachedMapDisplay() {
        if (!state.lastMapEnvelope) {
          reevaluateCachedLocalizationDisplay();
          return;
        }
        const envelope = state.lastMapEnvelope;
        const data = envelope.data || {};
        const grid = data.occupancy_grid || (data.map_snapshot && data.map_snapshot.occupancy_grid) || null;
        const diagnostics = data.diagnostics || data.metadata || (data.map_snapshot && data.map_snapshot.metadata) || {};
        const streamBridge = diagnostics.stream_bridge || {};
        const status = statusOf(envelope);
        const cleared = isMapClearedEnvelope(envelope, diagnostics, status);
        const decision = shouldDisplayMapFrame(envelope, diagnostics, streamBridge, grid, status, cleared);
        if (decision.display || state.mapGrid) updateMap(envelope);
        else if (state.mapGrid) {
          clearMapDisplay(decision.message, decision.status || status, diagnostics, streamBridge, envelope, grid);
        }
        reevaluateCachedLocalizationDisplay();
      }
      function reevaluateCachedLocalizationDisplay() {
        if (!state.lastLocalizationEnvelope) return;
        applyLocalizationDisplay(state.lastLocalizationEnvelope, { reevaluateMap: false });
      }
      function applyLocalizationDisplay(envelope, options) {
        const reevaluateMap = !options || options.reevaluateMap !== false;
        state.lastLocalizationEnvelope = envelope;
        if (!shouldDisplaySpatialPose()) {
          clearLocalizationDisplay(LOCALIZATION_DISPLAY_IDLE_MESSAGE, "missing");
          requestMapDraw(false);
          return;
        }
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
        setPill(els.poseStatus, status);
        setKv(els.missionPoseMeta, [
          ["frame", envelope.frame_id || (state.pose && state.pose.frame_id)],
          ["position", p ? `x=${fmt(p.x)} y=${fmt(p.y)} z=${fmt(p.z)}` : "--"],
          ["yaw", yaw === null ? "--" : fmt(yaw, " rad")],
          ["trajectory", state.trajectory.length],
          ["latency", fmt(envelope.latency_ms, " ms")],
        ]);
        if (reevaluateMap) reevaluateCachedMapDisplay();
        else requestMapDraw(false);
      }
      function updateLocalization(envelope) {
        applyLocalizationDisplay(envelope, { reevaluateMap: true });
      }
      function firstArrayValue(...values) {
        for (const value of values) {
          if (Array.isArray(value) && value.length) return value;
        }
        return [];
      }
      function pathPointFromValue(value) {
        if (!value) return null;
        if (Array.isArray(value) && value.length >= 2) return { x: Number(value[0]), y: Number(value[1]), z: Number(value[2] || 0) };
        if (value.pose) return pointFromPose(value.pose);
        if (value.position) return pointFromPose(value);
        if (value.target_pose) return pointFromPose(value.target_pose);
        if (value.x !== undefined && value.y !== undefined) return { x: Number(value.x), y: Number(value.y), z: Number(value.z || 0) };
        return null;
      }
      function explorationStateMetadata(context) {
        if (!context) return {};
        const stateObj = context.exploration_state || context;
        return (stateObj && stateObj.metadata) || context.metadata || {};
      }
      function extractNavigationPath(context, explorationContext) {
        const metadata = context && context.metadata ? context.metadata : {};
        const stateObj = context && context.navigation_state ? context.navigation_state : {};
        const stateMetadata = stateObj && stateObj.metadata ? stateObj.metadata : {};
        const explorationMetadata = explorationStateMetadata(explorationContext);
        const raw = firstArrayValue(
          context && context.planned_path,
          context && context.current_path,
          context && context.remaining_path,
          context && context.nav_path,
          context && context.waypoints,
          metadata.planned_path,
          metadata.current_path,
          metadata.remaining_path,
          metadata.nav_path,
          metadata.waypoints,
          stateMetadata.planned_path,
          stateMetadata.current_path,
          stateMetadata.remaining_path,
          stateMetadata.nav_path,
          stateMetadata.waypoints,
          explorationContext && explorationContext.planned_path,
          explorationMetadata.planned_path,
          explorationMetadata.nav_path
        );
        return raw.map(pathPointFromValue).filter(Boolean);
      }
      function extractNavigationGoal(context, explorationContext) {
        const goal = context && context.current_goal ? context.current_goal : null;
        if (goal && goal.target_pose) return goal.target_pose;
        const explorationMetadata = explorationStateMetadata(explorationContext);
        const explorationGoal = explorationContext && explorationContext.current_goal;
        if (explorationGoal && explorationGoal.target_pose) return explorationGoal.target_pose;
        const frontier = explorationMetadata.current_frontier;
        if (frontier && frontier.pose) return frontier.pose;
        return null;
      }
      function normalizeExplorerStatusText(statusText) {
        const text = safeText(statusText);
        if (!text) return "";
        return text.replace(/^\[explorer\]\s*/i, "").trim();
      }
      function isTerminalExplorerStatus(statusText) {
        const upper = normalizeExplorerStatusText(statusText).toUpperCase();
        return upper.includes("DONE") || upper.includes("EXPLORATION FINISHED") || upper.includes("COMPLETE");
      }
      function summarizeContext(context, key) {
        const stateObj = context && context[key];
        if (!stateObj) return "--";
        const metadata = stateObj.metadata || {};
        const explorerStatus = metadata.status_text || metadata.explorer_status_text;
        if (metadata.exploration_terminal || metadata.terminal) {
          const terminal = metadata.terminal_status || stateObj.status || "completed";
          if (explorerStatus && isTerminalExplorerStatus(explorerStatus)) {
            return `${terminal} · ${normalizeExplorerStatusText(explorerStatus)}`;
          }
          return String(terminal);
        }
        if (explorerStatus) {
          const base = stateObj.status || stateObj.phase || stateObj.message || stateObj.state || "available";
          return `${base} · ${normalizeExplorerStatusText(explorerStatus)}`;
        }
        return stateObj.status || stateObj.phase || stateObj.message || stateObj.state || "available";
      }
      function updateGovernance(envelope) {
        const data = envelope.runtime_governance || (envelope.data && envelope.data.runtime_governance) || envelope.data || {};
        state.latestGovernance = data;
        const status = envelope.status ? statusOf(envelope) : (data && data.rtabmap_profile ? "available" : "missing");
        const profile = data.rtabmap_profile || {};
        const asset = data.geometry_asset || {};
        const session = data.localization_session || {};
        const relocalization = data.last_relocalization || {};
        const policy = data.navigation_control_policy || {};
        const frontAlign = policy.front_align || {};
        const rotate = policy.rotate_relocalization || {};
        setPill(els.governanceStatus, status);
        setKv(els.governanceMeta, [
          ["profile", `${safeText(profile.name)} / ${safeText(profile.status)} / route=${safeText(profile.route)}`],
          ["geometry_asset", asset.available ? `${safeText(asset.geometry_stack)} db=${safeText(asset.database_present)} pcd=${safeText(asset.pcd_present)}` : "missing"],
          ["asset_frame", asset.frame_id],
          ["localization", session.available ? `${safeText(session.status)} pose=${safeText(session.pose_available)}` : "missing"],
          ["last_relocalization", relocalization.available ? `fitness=${safeText(relocalization.fitness)} inlier=${safeText(relocalization.inlier_ratio)}` : safeText(relocalization.status)],
          ["relocalization_failure", relocalization.failure_reason || session.last_error],
          ["front_align", `${safeText(frontAlign.enabled)} triggered=${safeText(frontAlign.triggered)} reason=${safeText(frontAlign.reason)}`],
          ["rotate_reloc", `${safeText(rotate.enabled)} triggered=${safeText(rotate.triggered)} reason=${safeText(rotate.reason)}`],
          ["source", (data.metadata && data.metadata.source) || "platform_service_contracts"],
          ["前端接收", shortTime(envelope.__receivedAt)],
        ]);
        reevaluateCachedMapDisplay();
      }
      function mergeLatestFrontierStream(data) {
        const fr = state.latestFrontierStream;
        if (!fr) return data;
        const merged = { ...data };
        if (!merged.explorer_status_text && fr.status_text) merged.explorer_status_text = fr.status_text;
        if ((merged.frontier_count === null || merged.frontier_count === undefined) && fr.frontier_count != null) {
          merged.frontier_count = fr.frontier_count;
        }
        if ((!merged.frontiers || !merged.frontiers.length) && Array.isArray(fr.latest_frontiers) && fr.latest_frontiers.length) {
          merged.frontiers = fr.latest_frontiers;
        }
        if ((!merged.planned_path || !merged.planned_path.length) && Array.isArray(fr.planned_path) && fr.planned_path.length) {
          merged.planned_path = fr.planned_path;
        }
        if (!merged.current_goal && fr.current_goal) merged.current_goal = fr.current_goal;
        if (merged.exploration_terminal === undefined || merged.exploration_terminal === null) {
          merged.exploration_terminal = fr.exploration_terminal;
        }
        return merged;
      }
      function updateNavigation(envelope) {
        const status = statusOf(envelope);
        const data = mergeLatestFrontierStream(envelope.data || {});
        setPill(els.navigationStatus, status);
        const frontierCount = data.frontier_count;
        const waitingFor = data.waiting_for;
        const explorationState = data.exploration_context && data.exploration_context.exploration_state;
        const explorationMetadata = explorationState && explorationState.metadata ? explorationState.metadata : {};
        let explorerStatus = data.explorer_status_text
          || explorationMetadata.status_text
          || explorationMetadata.explorer_status_text;
        const explorationTerminal = data.exploration_terminal
          || explorationMetadata.exploration_terminal
          || (explorationState && ["succeeded", "failed", "cancelled"].includes(String(explorationState.status || "")));
        if (explorationTerminal && explorerStatus && !isTerminalExplorerStatus(explorerStatus)) {
          explorerStatus = explorationMetadata.status_text && isTerminalExplorerStatus(explorationMetadata.status_text)
            ? explorationMetadata.status_text
            : "DONE (exploration finished)";
        }
        const mappingDiag = data.mapping_chain_diagnostics || {};
        const liveMap = mappingDiag.live_map_provider || {};
        const cloudPub = mappingDiag.point_cloud_publisher || {};
        const cloudSource = mappingDiag.point_cloud_source || {};
        const frontiers = Array.isArray(data.frontiers) ? data.frontiers : null;
        state.frontiers = frontiers || [];
        state.goalPose = extractNavigationGoal(data.navigation_context, data.exploration_context);
        state.navigationPath = extractNavigationPath(data.navigation_context, data.exploration_context);
        if (!state.navigationPath.length && Array.isArray(data.planned_path)) {
          state.navigationPath = data.planned_path.map(pathPointFromValue).filter(Boolean);
        }
        if (!state.goalPose && data.current_goal && data.current_goal.target_pose) {
          state.goalPose = data.current_goal.target_pose;
        }
        syncDebugNavGoalFromStream(data.navigation_context || data);
        if (state.pendingNavGoal && state.pendingNavGoal.status === "accepted" && state.goalPose) {
          state.pendingNavGoal = null;
        }
        setKv(els.navigationMeta, [
          ["navigation", summarizeContext(data.navigation_context, "navigation_state")],
          ["exploration", summarizeContext(data.exploration_context, "exploration_state")],
          ["explorer_status", explorerStatus ? normalizeExplorerStatusText(explorerStatus) : "--"],
          ["exploration_terminal", explorationTerminal === true ? "true" : (explorationTerminal === false ? "false" : "--")],
          ["frontier_count", frontierCount === null || frontierCount === undefined ? "--" : frontierCount],
          ["waiting_for", waitingFor],
          ["mapping_detail", mappingDiag.message],
          ["occupancy", `${safeText(liveMap.occupancy_topic)} ready=${safeText(liveMap.occupancy_grid_ready)} count=${safeText(liveMap.occupancy_message_count)}`],
          ["pointcloud_bridge", `${safeText(cloudPub.topic || cloudPub.point_cloud_topic)} published=${safeText(cloudPub.published_count)} source=${safeText(cloudPub.source || cloudPub.last_source_topic || cloudSource.source_topic)}`],
          ["frontier_geometry", frontiers && frontiers.length ? `${frontiers.length} 个几何` : "暂无候选"],
          ["goal", state.goalPose ? "target_pose" : "--"],
          ["nav_path", state.navigationPath.length ? `${state.navigationPath.length} 点` : "--"],
          ["source_timestamp", shortTime(envelope.source_timestamp)],
          ["latency", fmt(envelope.latency_ms, " ms")],
          ["stale", envelope.stale === true ? "true" : "false"],
          ["前端接收", shortTime(envelope.__receivedAt)],
        ]);
        els.frontierNotice.hidden = Boolean(explorationTerminal || (frontiers && frontiers.length));
        els.frontierNotice.textContent = explorationTerminal
          ? "探索器已报告完成（DONE）。"
          : (frontiers && frontiers.length ? "" : "当前探索上下文还没有 frontier 几何，请确认探索运行态已生成候选。");
        reevaluateCachedMapDisplay();
        requestMapDraw(false);
      }
      function scheduleVideoUpdate(envelope) {
        state.pendingVideoEnvelope = envelope;
        if (state.videoRenderFrame) return;
        state.videoRenderFrame = window.requestAnimationFrame(() => {
          state.videoRenderFrame = null;
          const next = state.pendingVideoEnvelope;
          state.pendingVideoEnvelope = null;
          if (next) updateVideo(next);
        });
      }
      function buildVideoFrameSrc(data) {
        if (data.stream_url) {
          const url = new URL(data.stream_url, window.location.origin);
          if (!url.searchParams.get("fps")) url.searchParams.set("fps", "15");
          if (apiAccessToken && !url.searchParams.get("access_token")) url.searchParams.set("access_token", apiAccessToken);
          return url.toString();
        }
        return data.data_uri || data.uri || "";
      }
      function revokeVideoObjectUrl() {
        if (!state.videoCurrentObjectUrl) return;
        try {
          URL.revokeObjectURL(state.videoCurrentObjectUrl);
        } catch (err) {
          // 忽略浏览器释放失败；下一帧会覆盖显示。
        }
        state.videoCurrentObjectUrl = null;
      }
      function setVideoStreamStatus(status, message) {
        state.videoStreamStatus = status;
        if (message) {
          els.videoEmpty.hidden = state.videoFrames > 0;
          if (!els.videoEmpty.hidden) els.videoEmpty.textContent = message;
        }
        setText(
          els.videoFpsStatus,
          `解码帧 ${state.videoFrames} · 检测 ${state.videoDetections.length} · 流 ${status}`
        );
      }
      function scheduleVideoStreamReconnect(reason) {
        if (!state.videoStreamSrc) return;
        if (state.videoStreamReconnectTimer) return;
        const delayMs = state.videoStreamReconnectDelayMs;
        const nextDelayMs = Math.min(VIDEO_STREAM_MAX_RECONNECT_MS, Math.max(VIDEO_STREAM_INITIAL_RECONNECT_MS, delayMs * 1.6));
        setVideoStreamStatus("reconnecting", `${reason || "MJPEG 视频流中断"}，${Math.round(delayMs)} ms 后重连。`);
        state.videoStreamReconnectTimer = window.setTimeout(() => {
          state.videoStreamReconnectTimer = null;
          state.videoStreamReconnectDelayMs = nextDelayMs;
          startVideoStream({ force: true, reason: "reconnect" });
        }, delayMs);
      }
      function stopVideoStream(reason) {
        state.videoStreamGeneration += 1;
        if (state.videoStreamController) {
          try {
            state.videoStreamController.abort();
          } catch (err) {
            // AbortController 可能已结束。
          }
          state.videoStreamController = null;
        }
        if (state.videoStreamReconnectTimer) {
          window.clearTimeout(state.videoStreamReconnectTimer);
          state.videoStreamReconnectTimer = null;
        }
        setVideoStreamStatus("stopped", reason || "视频流已停止。");
      }
      function clearVideoDisplay(reason) {
        stopVideoStream(reason);
        revokeVideoObjectUrl();
        els.videoFrame.removeAttribute("src");
        els.videoFrame.hidden = true;
        state.videoStreamSrc = null;
        state.lastVideoFrameKey = null;
      }
      function startVideoStream(options) {
        const opts = options || {};
        const videoSrc = opts.streamUrl
          ? new URL(opts.streamUrl, window.location.origin).toString()
          : buildVideoFrameSrc({ stream_url: DEFAULT_VIDEO_STREAM_URL });
        if (!videoSrc) return;
        if (!opts.force && state.videoStreamSrc === videoSrc && state.videoStreamController) return;
        const reconnectDelay = opts.reason === "reconnect" ? state.videoStreamReconnectDelayMs : VIDEO_STREAM_INITIAL_RECONNECT_MS;
        if (state.videoStreamController) {
          try {
            state.videoStreamController.abort();
          } catch (err) {
            // 旧连接可能已结束。
          }
          state.videoStreamController = null;
        }
        if (state.videoStreamReconnectTimer) {
          window.clearTimeout(state.videoStreamReconnectTimer);
          state.videoStreamReconnectTimer = null;
        }
        state.videoStreamReconnectDelayMs = reconnectDelay;
        state.videoStreamGeneration += 1;
        state.videoStreamSrc = videoSrc;
        const generation = state.videoStreamGeneration;
        const controller = new AbortController();
        state.videoStreamController = controller;
        state.videoStreamBytes = 0;
        state.videoDecodedBytes = 0;
        state.videoLastError = "";
        setVideoStreamStatus("connecting", "正在连接平台转发的 Go2 端侧 MJPEG 视频流。");
        consumeMjpegStream(videoSrc, controller.signal, generation).catch((err) => {
          if (generation !== state.videoStreamGeneration || controller.signal.aborted) return;
          const message = err && err.message ? err.message : String(err || "未知错误");
          state.videoLastError = message;
          state.videoDecodeErrors += 1;
          state.videoStreamController = null;
          scheduleVideoStreamReconnect(`MJPEG 解码流失败：${message}`);
        });
      }
      function resetVideoStreamFromMetadata(videoSrc) {
        if (!videoSrc) return false;
        const latestRenderedAgeMs = state.videoLastFrameAt ? Date.now() - state.videoLastFrameAt : Infinity;
        if (
          state.videoStreamSrc === videoSrc
          && !state.videoStreamController
          && !state.videoStreamReconnectTimer
          && latestRenderedAgeMs > VIDEO_STREAM_MAX_RECONNECT_MS
        ) {
          startVideoStream({ streamUrl: videoSrc, force: true, reason: "metadata_watchdog" });
          return true;
        }
        return false;
      }
      async function consumeMjpegStream(videoSrc, signal, generation) {
        const response = await fetch(videoSrc, { method: "GET", cache: "no-store", signal });
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`);
        }
        if (!response.body || !response.body.getReader) {
          throw new Error("当前浏览器不支持 ReadableStream MJPEG 解码。");
        }
        state.videoStreamReconnectDelayMs = VIDEO_STREAM_INITIAL_RECONNECT_MS;
        setVideoStreamStatus("streaming", "已连接 MJPEG 字节流，等待首帧 JPEG 解码。");
        const reader = response.body.getReader();
        let buffer = new Uint8Array(0);
        try {
          while (generation === state.videoStreamGeneration) {
            const result = await reader.read();
            if (result.done) break;
            if (!result.value || !result.value.byteLength) continue;
            state.videoStreamBytes += result.value.byteLength;
            buffer = appendBytes(buffer, result.value);
            if (buffer.byteLength > MJPEG_MAX_BUFFER_BYTES) {
              const soi = findMarker(buffer, 0xff, 0xd8, Math.max(0, buffer.byteLength - 1024 * 1024));
              buffer = soi >= 0 ? buffer.slice(soi) : new Uint8Array(0);
              if (!buffer.byteLength) throw new Error("MJPEG 缓冲区超过上限且未找到 JPEG 起始标记。");
            }
            let parsedAny = false;
            let latestFrameBytes = null;
            while (true) {
              const parsed = extractNextJpeg(buffer);
              if (!parsed) break;
              if (!parsed.frame) {
                buffer = parsed.remaining;
                break;
              }
              parsedAny = true;
              latestFrameBytes = parsed.frame;
              buffer = parsed.remaining;
            }
            if (latestFrameBytes) {
              try {
                await renderDecodedVideoFrame(latestFrameBytes, generation);
              } catch (err) {
                const message = err && err.message ? err.message : String(err || "未知错误");
                state.videoLastError = message;
                state.videoDecodeErrors += 1;
                state.videoStreamController = null;
                scheduleVideoStreamReconnect(`JPEG 帧解码失败：${message}`);
                return;
              }
            }
            if (!parsedAny && state.videoFrames === 0) {
              setVideoStreamStatus("streaming", `已接收 ${state.videoStreamBytes} 字节，等待完整 JPEG 帧。`);
            }
          }
        } finally {
          try {
            reader.releaseLock();
          } catch (err) {
            // 读取器可能已经释放。
          }
        }
        if (generation === state.videoStreamGeneration) {
          state.videoStreamController = null;
          scheduleVideoStreamReconnect("MJPEG 字节流已结束");
        }
      }
      function appendBytes(left, right) {
        if (!left || !left.byteLength) return right;
        const merged = new Uint8Array(left.byteLength + right.byteLength);
        merged.set(left, 0);
        merged.set(right, left.byteLength);
        return merged;
      }
      function findMarker(bytes, a, b, start) {
        const begin = Math.max(0, Number(start || 0));
        for (let i = begin; i < bytes.byteLength - 1; i += 1) {
          if (bytes[i] === a && bytes[i + 1] === b) return i;
        }
        return -1;
      }
      function extractNextJpeg(buffer) {
        const soi = findMarker(buffer, 0xff, 0xd8, 0);
        if (soi < 0) {
          return {
            frame: null,
            remaining: buffer.byteLength > 2 ? buffer.slice(buffer.byteLength - 2) : buffer,
          };
        }
        const eoi = findMarker(buffer, 0xff, 0xd9, soi + 2);
        if (eoi < 0) {
          return {
            frame: null,
            remaining: soi > 0 ? buffer.slice(soi) : buffer,
          };
        }
        const frame = buffer.slice(soi, eoi + 2);
        const remaining = buffer.slice(eoi + 2);
        if (!frame.byteLength) return null;
        return { frame, remaining };
      }
      async function decodeJpegBlob(blob) {
        let probeUrl = null;
        if (typeof createImageBitmap === "function") {
          const timeout = new Promise((_, reject) => {
            window.setTimeout(() => reject(new Error("JPEG 解码超时")), VIDEO_JPEG_DECODE_TIMEOUT_MS);
          });
          const bitmap = await Promise.race([createImageBitmap(blob), timeout]);
          if (bitmap && typeof bitmap.close === "function") bitmap.close();
        } else {
          try {
            await new Promise((resolve, reject) => {
              const probe = new Image();
              let settled = false;
              const timer = window.setTimeout(() => {
                if (settled) return;
                settled = true;
                reject(new Error("JPEG 解码超时"));
              }, VIDEO_JPEG_DECODE_TIMEOUT_MS);
              probe.onload = () => {
                if (settled) return;
                settled = true;
                window.clearTimeout(timer);
                resolve();
              };
              probe.onerror = () => {
                if (settled) return;
                settled = true;
                window.clearTimeout(timer);
                reject(new Error("JPEG 解码失败"));
              };
              probeUrl = URL.createObjectURL(blob);
              probe.src = probeUrl;
            });
          } finally {
            if (probeUrl) URL.revokeObjectURL(probeUrl);
          }
        }
      }
      async function renderDecodedVideoFrame(frameBytes, generation) {
        if (!frameBytes || !frameBytes.byteLength) return;
        const blob = new Blob([frameBytes], { type: "image/jpeg" });
        await decodeJpegBlob(blob);
        if (generation !== state.videoStreamGeneration) return;
        const nextUrl = await displayVideoBlob(blob, generation);
        if (generation !== state.videoStreamGeneration) {
          try {
            URL.revokeObjectURL(nextUrl);
          } catch (err) {
            // 过期帧释放失败不影响下一轮。
          }
          return;
        }
        state.videoCurrentObjectUrl = nextUrl;
        state.videoDecodedBytes += frameBytes.byteLength;
        state.videoFrames += 1;
        state.videoLastFrameAt = Date.now();
        state.videoStreamStatus = "rendering";
        els.videoFrame.hidden = false;
        els.videoEmpty.hidden = true;
        void els.videoFrame.offsetHeight;
        scheduleVideoDetectionDraw();
      }
      async function displayVideoBlob(blob, generation) {
        const nextUrl = URL.createObjectURL(blob);
        const previousUrl = state.videoCurrentObjectUrl;
        try {
          await new Promise((resolve, reject) => {
            let settled = false;
            const timer = window.setTimeout(() => {
              if (settled) return;
              settled = true;
              cleanup();
              reject(new Error("JPEG 显示超时"));
            }, VIDEO_JPEG_DECODE_TIMEOUT_MS);
            function cleanup() {
              window.clearTimeout(timer);
              els.videoFrame.removeEventListener("load", onLoad);
              els.videoFrame.removeEventListener("error", onError);
            }
            function onLoad() {
              if (settled) return;
              settled = true;
              cleanup();
              resolve();
            }
            function onError() {
              if (settled) return;
              settled = true;
              cleanup();
              reject(new Error("JPEG 显示失败"));
            }
            els.videoFrame.addEventListener("load", onLoad, { once: true });
            els.videoFrame.addEventListener("error", onError, { once: true });
            if (generation !== state.videoStreamGeneration) {
              settled = true;
              cleanup();
              resolve();
              return;
            }
            els.videoFrame.src = nextUrl;
          });
        } catch (err) {
          try {
            URL.revokeObjectURL(nextUrl);
          } catch (revokeErr) {
            // 忽略失败。
          }
          throw err;
        }
        if (generation !== state.videoStreamGeneration) return nextUrl;
        if (previousUrl) {
          window.setTimeout(() => {
            try {
              URL.revokeObjectURL(previousUrl);
            } catch (err) {
              // 旧帧 URL 释放失败不影响后续实时帧。
            }
          }, 250);
        }
        return nextUrl;
      }
      function updateVideo(envelope) {
        const status = statusOf(envelope);
        const data = envelope.data || {};
        const detections = Array.isArray(data.detections_2d) ? data.detections_2d : null;
        if (detections) {
          updateVideoDetections(data, envelope);
          return;
        }
        setPill(els.videoStatus, status);
        const videoSrc = buildVideoFrameSrc(data);
        if (videoSrc) {
          if (data.stream_url) {
            if (state.videoStreamSrc !== videoSrc || (!state.videoStreamController && !state.videoStreamReconnectTimer)) {
              startVideoStream({ streamUrl: videoSrc, force: state.videoStreamSrc !== videoSrc, reason: "metadata" });
            } else {
              resetVideoStreamFromMetadata(videoSrc);
            }
          } else if (state.lastVideoFrameKey !== videoSrc) {
            stopVideoStream("切换到单帧图像显示。");
            revokeVideoObjectUrl();
            els.videoFrame.setAttribute("src", videoSrc);
            state.videoStreamSrc = videoSrc;
            state.lastVideoFrameKey = videoSrc;
            state.videoFrames += 1;
            state.videoLastFrameAt = Date.now();
          }
          if (data.width_px && data.height_px) state.videoSourceSize = { width: Number(data.width_px), height: Number(data.height_px) };
          if (state.videoFrames > 0 || !data.stream_url) {
            els.videoFrame.hidden = false;
            els.videoEmpty.hidden = state.videoFrames > 0;
          } else {
            els.videoFrame.hidden = true;
            els.videoEmpty.hidden = false;
          }
          scheduleVideoDetectionDraw();
        } else if (!state.lastVideoFrameKey) {
          clearVideoDisplay(errorReason(envelope, "尚未收到 Go2 端侧主动推送的视频帧。"));
          els.videoEmpty.textContent = errorReason(envelope, "尚未收到 Go2 端侧主动推送的视频帧。");
        }
        setKv(els.videoMeta, [
          ["camera", data.camera_id],
          ["frame", envelope.frame_id || data.frame_id],
          ["size", data.width_px && data.height_px ? `${data.width_px} x ${data.height_px}` : "--"],
          ["encoding", data.encoding],
          ["transport", data.transport],
          ["ingest_connected", data.ingest_status && data.ingest_status.connected],
          ["ingest_fps", data.ingest_status && fmt(data.ingest_status.fps_estimate, " fps")],
          ["ingest_last_client", data.ingest_status && data.ingest_status.last_client_id],
          ["stream_url", data.stream_url],
          ["payload_bytes", data.payload_bytes],
          ["browser_stream", state.videoStreamStatus],
          ["browser_bytes", state.videoStreamBytes],
          ["decoded_bytes", state.videoDecodedBytes],
          ["decode_errors", state.videoDecodeErrors],
          ["last_decode_error", state.videoLastError],
          ["detections", state.videoDetections.length],
          ["rendered_frames", state.videoFrames],
          ["last_rendered_at", state.videoLastFrameAt ? new Date(state.videoLastFrameAt).toLocaleTimeString() : "--"],
          ["source_timestamp", shortTime(envelope.source_timestamp)],
          ["latency", fmt(envelope.latency_ms, " ms")],
          ["前端接收", shortTime(envelope.__receivedAt)],
        ]);
        setText(els.visionModelStatus, `本地模型：${status}`);
        setText(els.videoFpsStatus, `解码帧 ${state.videoFrames} · 检测 ${state.videoDetections.length} · 流 ${state.videoStreamStatus} · 延迟 ${fmt(envelope.latency_ms, " ms")}`);
      }
      function updateVideoDetections(data, envelope) {
        state.videoDetections = Array.isArray(data.detections_2d) ? data.detections_2d : [];
        if (data.width_px && data.height_px) state.videoSourceSize = { width: Number(data.width_px), height: Number(data.height_px) };
        setPill(els.videoStatus, statusOf(envelope), `${state.videoDetections.length} detections`);
        setText(els.visionModelStatus, `本地模型：${data.detector_backend || envelope.source || "perception"}`);
        setText(els.videoFpsStatus, `解码帧 ${state.videoFrames} · 检测 ${state.videoDetections.length} · 流 ${state.videoStreamStatus}`);
        scheduleVideoDetectionDraw();
      }
      function scheduleVideoDetectionDraw() {
        if (state.videoDetectionDrawFrame) return;
        state.videoDetectionDrawFrame = window.requestAnimationFrame(() => {
          state.videoDetectionDrawFrame = null;
          drawVideoDetections();
        });
      }
      function drawVideoDetections() {
        const canvas = els.videoDetections;
        const image = els.videoFrame;
        if (!canvas || !image || !state.showDetections || image.hidden || !state.videoDetections.length) {
          if (canvas) {
            canvas.hidden = true;
            const context = canvas.getContext("2d");
            if (context) context.clearRect(0, 0, canvas.width, canvas.height);
          }
          return;
        }
        const stageRect = canvas.parentElement.getBoundingClientRect();
        const imageRect = image.getBoundingClientRect();
        const width = Math.max(1, Math.round(stageRect.width));
        const height = Math.max(1, Math.round(stageRect.height));
        if (canvas.width !== width || canvas.height !== height) {
          canvas.width = width;
          canvas.height = height;
        }
        const sourceWidth = Number((state.videoSourceSize && state.videoSourceSize.width) || image.naturalWidth || 0);
        const sourceHeight = Number((state.videoSourceSize && state.videoSourceSize.height) || image.naturalHeight || 0);
        if (!(sourceWidth > 0 && sourceHeight > 0)) {
          canvas.hidden = true;
          return;
        }
        const fitScale = Math.min(imageRect.width / sourceWidth, imageRect.height / sourceHeight);
        const drawnWidth = sourceWidth * fitScale;
        const drawnHeight = sourceHeight * fitScale;
        const scaleX = fitScale;
        const scaleY = fitScale;
        const offsetX = imageRect.left - stageRect.left + (imageRect.width - drawnWidth) / 2;
        const offsetY = imageRect.top - stageRect.top + (imageRect.height - drawnHeight) / 2;
        const context = canvas.getContext("2d");
        context.clearRect(0, 0, canvas.width, canvas.height);
        context.lineWidth = 2;
        context.font = "12px ui-monospace, SFMono-Regular, Menlo, monospace";
        for (const detection of state.videoDetections) {
          const bbox = detection && detection.bbox;
          if (!bbox) continue;
          const x = offsetX + Number(bbox.x_px || 0) * scaleX;
          const y = offsetY + Number(bbox.y_px || 0) * scaleY;
          const boxWidth = Number(bbox.width_px || 0) * scaleX;
          const boxHeight = Number(bbox.height_px || 0) * scaleY;
          if (!(boxWidth > 0 && boxHeight > 0)) continue;
          const label = `${detection.label || "object"} ${detection.score !== undefined ? Math.round(Number(detection.score) * 100) + "%" : ""}`.trim();
          context.strokeStyle = "#22d3ee";
          context.fillStyle = "rgba(34, 211, 238, 0.16)";
          context.strokeRect(x, y, boxWidth, boxHeight);
          context.fillRect(x, y, boxWidth, boxHeight);
          const labelWidth = Math.min(context.measureText(label).width + 10, Math.max(50, boxWidth));
          context.fillStyle = "rgba(2, 6, 23, 0.82)";
          context.fillRect(x, Math.max(0, y - 20), labelWidth, 18);
          context.fillStyle = "#e0f2fe";
          context.fillText(label, x + 5, Math.max(12, y - 7));
        }
        canvas.hidden = false;
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
        const metrics = data.metrics || {};
        setKv(els.logsMeta, [
          ["entries", state.logs.length],
          ["next_cursor", state.logCursor],
          ["source_status", data.source_status || status],
          ["source_message", data.source_message || (envelope.error && envelope.error.message)],
          ["server_compute_ms", fmt(metrics.server_compute_ms, " ms")],
          ["file_read_ms", fmt(metrics.file_read_ms, " ms")],
          ["bytes_read", metrics.bytes_read],
          ["前端接收", shortTime(envelope.__receivedAt)],
        ]);
        renderLogs();
      }
      function renderLogs() {
        els.logs.replaceChildren();
        const normalizedLevel = String(state.logFilter || "").toLowerCase();
        const query = String(state.logSearch || "").trim().toLowerCase();
        const filtered = state.logs.filter((entry) => {
          const level = String(entry.level || "").toLowerCase();
          const haystack = `${entry.timestamp || ""} ${entry.level || ""} ${entry.source || ""} ${entry.message || ""} ${entry.event_type || ""}`.toLowerCase();
          if (normalizedLevel && level !== normalizedLevel && !(normalizedLevel === "warn" && level === "warning") && !(normalizedLevel === "warning" && level === "warn")) return false;
          if (query && !haystack.includes(query)) return false;
          return true;
        });
        if (!filtered.length) {
          const empty = document.createElement("div");
          empty.className = "empty";
          empty.textContent = "暂无匹配日志行。";
          els.logs.appendChild(empty);
          return;
        }
        for (const entry of filtered.slice(-120).reverse()) {
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
      const DEFAULT_VIDEO_STREAM_URL = "/streams/video.mjpeg";
      const STREAM_TOPICS = [
        "robot.state",
        "map.frame",
        "map.heartbeat",
        "localization.pose",
        "localization.frame_tree",
        "localization.trajectory",
        "navigation.state",
        "exploration.state",
        "exploration.frontier",
        "runtime.governance",
        "pointcloud_map.meta",
        "pointcloud_map.live_chunk",
        "pointcloud.frame",
        "video.frame",
        "video.detections",
        "logs.entry",
        "tasks.state"
      ];
      function envelopeFromFrame(frame, payloadOverride) {
        const receivedAt = new Date().toISOString();
        return {
          ok: frame.status !== "error" && frame.status !== "missing",
          type: frame.topic,
          source: frame.source,
          frame_id: frame.frame_id || (frame.payload && frame.payload.frame_id),
          source_timestamp: frame.timestamp || (frame.payload && frame.payload.timestamp),
          gateway_timestamp: frame.timestamp,
          sequence: frame.sequence ?? frame.cursor,
          revision: frame.revision,
          status: frame.status || "available",
          stale: frame.status === "stale",
          data: payloadOverride === undefined ? frame.payload : payloadOverride,
          metadata: frame.metadata || {},
          __receivedAt: receivedAt,
          __frontendLatencyMs: 0,
        };
      }
      function buildStreamUrl() {
        const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
        const url = new URL(`${protocol}//${window.location.host}/streams/ws`);
        url.searchParams.set("topics", STREAM_TOPICS.join(","));
        if (state.latestStreamCursor) url.searchParams.set("after_cursor", String(state.latestStreamCursor));
        if (apiAccessToken) url.searchParams.set("access_token", apiAccessToken);
        return url.toString();
      }
      function connectStreams() {
        if (state.streamSocket && state.streamSocket.readyState === WebSocket.OPEN) return;
        const socket = new WebSocket(buildStreamUrl());
        socket.binaryType = "arraybuffer";
        state.streamSocket = socket;
        socket.addEventListener("open", () => {
          setText(els.connectionText, "streaming");
          setText(els.networkLatency, "stream");
          if (state.streamReconnectTimer) {
            window.clearTimeout(state.streamReconnectTimer);
            state.streamReconnectTimer = null;
          }
        });
        socket.addEventListener("message", (event) => {
          if (event.data instanceof ArrayBuffer) {
            window.dispatchEvent(new CustomEvent("stream:pointcloud_map.binary", { detail: event.data }));
            return;
          }
          const message = JSON.parse(event.data);
          if (message.type === "keepalive") {
            setText(els.connectionText, "streaming");
            if (typeof message.cursor === "number") state.latestStreamCursor = message.cursor;
            return;
          }
          if (message.type !== "stream.frame" || !message.frame) return;
          handleStreamFrame(message.frame);
        });
        socket.addEventListener("close", () => scheduleStreamReconnect());
        socket.addEventListener("error", () => {
          setText(els.connectionText, "stream_error");
          showPanelError("logs", "实时数据流连接异常，正在重连。");
        });
      }
      function scheduleStreamReconnect() {
        setText(els.connectionText, "reconnecting");
        if (state.streamReconnectTimer) return;
        state.streamReconnectTimer = window.setTimeout(() => {
          state.streamReconnectTimer = null;
          connectStreams();
        }, 1000);
      }
      function handleStreamFrame(frame) {
        state.latestStreamCursor = Math.max(state.latestStreamCursor || 0, Number(frame.cursor || 0));
        state.latestStreamFrames[frame.topic] = frame;
        if (frame.topic === "logs.entry") {
          state.pendingLogFrames.push(frame);
          if (state.pendingLogFrames.length > 200) state.pendingLogFrames.splice(0, state.pendingLogFrames.length - 200);
        } else {
          state.pendingStreamFrames[frame.topic] = frame;
        }
        els.lastReceive.textContent = nowText();
        setText(els.connectionText, "streaming");
        scheduleStreamFlush();
      }
      function scheduleStreamFlush() {
        if (state.streamFlushFrame) return;
        state.streamFlushFrame = window.requestAnimationFrame(flushStreamFrames);
      }
      function flushStreamFrames() {
        state.streamFlushFrame = null;
        const pending = state.pendingStreamFrames;
        const logFrames = state.pendingLogFrames;
        state.pendingStreamFrames = {};
        state.pendingLogFrames = [];
        for (const topic of STREAM_FLUSH_ORDER) {
          const frame = pending[topic];
          if (!frame) continue;
          delete pending[topic];
          processStreamFrame(frame);
        }
        for (const topic of Object.keys(pending)) processStreamFrame(pending[topic]);
        if (logFrames.length) updateLogFrames(logFrames);
      }
      function processStreamFrame(frame) {
        let updateOverviewForFrame = true;
        if (frame.topic === "robot.state") {
          updateOverviewForFrame = updateRobotStateFrame(frame);
        } else if (frame.topic === "map.frame") {
          updateMap(envelopeFromFrame(frame));
        } else if (frame.topic === "map.heartbeat") {
          updateMapHeartbeat(envelopeFromFrame(frame));
        } else if (frame.topic === "localization.pose") {
          updateLocalization(envelopeFromFrame(frame, { pose: frame.payload, trajectory: state.trajectory }));
        } else if (frame.topic === "localization.trajectory") {
          const trajectory = frame.payload && Array.isArray(frame.payload.trajectory) ? frame.payload.trajectory : [];
          updateLocalization(envelopeFromFrame(frame, { pose: state.pose, trajectory }));
        } else if (frame.topic === "navigation.state") {
          updateNavigation(envelopeFromFrame(frame, { navigation_context: frame.payload, exploration_context: state.latestStreamFrames["exploration.state"] && state.latestStreamFrames["exploration.state"].payload }));
        } else if (frame.topic === "exploration.state") {
          updateNavigation(envelopeFromFrame(frame, { navigation_context: state.latestStreamFrames["navigation.state"] && state.latestStreamFrames["navigation.state"].payload, exploration_context: frame.payload }));
        } else if (frame.topic === "exploration.frontier") {
          updateFrontierFrame(frame);
        } else if (frame.topic === "runtime.governance") {
          updateGovernance(envelopeFromFrame(frame));
        } else if (frame.topic === "pointcloud_map.meta") {
          updatePointcloudMapFrame(frame);
        } else if (frame.topic === "pointcloud.frame") {
          updatePointcloudFrame(frame);
        } else if (frame.topic === "video.frame") {
          scheduleVideoUpdate(envelopeFromFrame(frame));
        } else if (frame.topic === "video.detections") {
          updateVideoDetections(frame.payload || {}, envelopeFromFrame(frame));
        } else if (frame.topic === "logs.entry") {
          updateLogFrame(frame);
        } else if (frame.topic === "tasks.state") {
          updateTaskFrame(frame);
        } else {
          updateOverviewForFrame = false;
        }
        if (updateOverviewForFrame) updateStreamOverview(frame);
      }
      function robotStateChangeKey(frame) {
        const metadata = frame.metadata || {};
        return metadata.content_hash || metadata.signature || frame.revision || null;
      }
      function updateRobotStateFrame(frame) {
        const changeKey = robotStateChangeKey(frame);
        if (changeKey && changeKey === state.lastRobotStateHash) return false;
        state.lastRobotStateHash = changeKey;
        state.lastRobotStateRevision = frame.revision || (frame.metadata && frame.metadata.revision) || state.lastRobotStateRevision;
        const payload = frame.payload || {};
        updateHealth({
          runtime_started: true,
          robot_name: payload.robot_name,
          robot_model: payload.robot_model,
          assembly_status: payload.assembly_status || {},
          latest_robot_snapshot: payload.robot_state || {},
          active_task_count: state.latestTasks && Array.isArray(state.latestTasks.active_tasks) ? state.latestTasks.active_tasks.length : 0,
          latest_event_cursor: frame.cursor || 0,
        });
        return true;
      }
      function updateFrontierFrame(frame) {
        const payload = frame.payload || {};
        state.latestFrontierStream = {
          status_text: payload.status_text,
          frontier_count: payload.frontier_count,
          latest_frontiers: Array.isArray(payload.latest_frontiers) ? payload.latest_frontiers : [],
          planned_path: Array.isArray(payload.planned_path) ? payload.planned_path : [],
          current_goal: payload.current_goal || null,
          exploration_terminal: payload.exploration_terminal,
          sequence: frame.sequence,
          received_at: frame.timestamp,
        };
        const frontiers = Array.isArray(payload.latest_frontiers) ? payload.latest_frontiers : [];
        state.frontiers = frontiers;
        updateNavigation(envelopeFromFrame(frame, {
          navigation_context: state.latestStreamFrames["navigation.state"] && state.latestStreamFrames["navigation.state"].payload,
          exploration_context: state.latestStreamFrames["exploration.state"] && state.latestStreamFrames["exploration.state"].payload,
          frontiers,
          frontier_count: payload.frontier_count !== undefined && payload.frontier_count !== null ? payload.frontier_count : frontiers.length,
          explorer_status_text: payload.status_text,
          exploration_terminal: payload.exploration_terminal,
          planned_path: payload.planned_path,
          current_goal: payload.current_goal,
        }));
      }
      function updatePointcloudMapFrame(frame) {
        const summary = frame.payload || {};
        const envelope = envelopeFromFrame(frame, summary);
        const data = { pointcloud_map: { status: envelope.status, frame_id: envelope.frame_id, revision: envelope.revision, latency_ms: envelope.latency_ms, source_timestamp: envelope.source_timestamp, metadata: { source: envelope.source, source_kind: summary.source_kind || (frame.metadata && frame.metadata.source_kind), point_count: summary.point_count ?? (frame.metadata && frame.metadata.point_count), pcd_path: summary.pcd_path || (frame.metadata && frame.metadata.pcd_path) } } };
        updateSensorSummary({ ...(state.latestOverview || {}), ...data });
      }
      function updatePointcloudFrame(frame) {
        const payload = frame.payload || {};
        const envelope = envelopeFromFrame(frame, payload);
        const data = { pointcloud: { status: envelope.status, frame_id: envelope.frame_id, revision: envelope.revision, latency_ms: envelope.latency_ms, source_timestamp: envelope.source_timestamp, metadata: { source: envelope.source, point_count: payload.point_count || (frame.metadata && frame.metadata.point_count) } } };
        updateSensorSummary({ ...(state.latestOverview || {}), ...data });
      }
      function logEntryFromFrame(frame) {
        const payload = frame.payload || {};
        return {
          sequence: frame.sequence ?? frame.cursor,
          timestamp: payload.timestamp || frame.timestamp,
          level: String(payload.severity || payload.level || frame.status || "info").toLowerCase(),
          source: payload.source || frame.source,
          event_type: payload.event_type,
          message: payload.message || payload.event_type || "runtime.event",
        };
      }
      function updateLogFrame(frame) {
        updateLogFrames([frame]);
      }
      function updateLogFrames(frames) {
        const latest = frames[frames.length - 1];
        const entries = frames.map(logEntryFromFrame);
        updateLogs(envelopeFromFrame(latest, { entries, next_cursor: latest.cursor, source_status: "streaming", metrics: {} }));
        updateStreamOverview(latest);
      }
      function updateTaskFrame(frame) {
        const framePayload = frame.payload || {};
        if (Array.isArray(framePayload.active_tasks) || Array.isArray(framePayload.history_tasks)) {
          replaceTaskStore(framePayload);
          return;
        }
        const event = framePayload.event || framePayload;
        const payload = event.payload || {};
        const requestContext = payload.request_context || {};
        const taskId = event.task_id || payload.task_id || requestContext.task_id;
        if (!taskId) return;
        const existing = state.taskActiveById.get(String(taskId)) || state.taskHistoryById.get(String(taskId)) || {};
        const task = {
          ...existing,
          task_id: taskId,
          capability_name: payload.capability_name || requestContext.capability_name || existing.capability_name,
          state: payload.state || event.state || existing.state || event.event_type,
          stage: payload.stage || event.message || existing.stage || event.event_type,
          message: payload.message || event.message || existing.message,
          progress: payload.progress !== undefined ? payload.progress : existing.progress,
          updated_at: event.timestamp || payload.updated_at || frame.timestamp || existing.updated_at,
          completed_at: payload.completed_at || existing.completed_at,
        };
        if (isTerminalTaskState(task.state)) {
          state.taskActiveById.delete(String(taskId));
          state.taskHistoryById.set(String(taskId), task);
        } else {
          state.taskHistoryById.delete(String(taskId));
          state.taskActiveById.set(String(taskId), task);
        }
        refreshTaskStore();
      }
      function updateStreamOverview(frame) {
        const keyByTopic = {
          "robot.state": "state",
          "map.frame": "map",
          "map.heartbeat": "map",
          "localization.pose": "localization",
          "navigation.state": "navigation",
          "exploration.state": "exploration",
          "runtime.governance": "runtime_governance",
          "pointcloud_map.meta": "pointcloud_map",
          "pointcloud.frame": "pointcloud",
          "video.frame": "video",
          "logs.entry": "logs",
        };
        const key = keyByTopic[frame.topic];
        if (!key) return;
        const payload = frame.payload || {};
        const pointcloudMeta = { source: frame.source, point_count: payload.point_count ?? (frame.metadata && frame.metadata.point_count) };
        const metadata = frame.topic === "pointcloud_map.meta"
          ? { ...pointcloudMeta, source_kind: payload.source_kind || (frame.metadata && frame.metadata.source_kind), pcd_path: payload.pcd_path || (frame.metadata && frame.metadata.pcd_path) }
          : frame.topic === "pointcloud.frame" ? pointcloudMeta
          : frame.topic === "map.heartbeat" ? { source: frame.source, event_type: payload.event_type || (frame.metadata && frame.metadata.event_type), message_count: payload.message_count, update_sequence: payload.update_sequence }
          : { source: frame.source };
        const overview = { ...(state.latestOverview || {}) };
        overview[key] = {
          status: frame.status || "available",
          source_timestamp: frame.timestamp,
          latency_ms: 0,
          stale: frame.status === "stale",
          frame_id: frame.frame_id,
          revision: frame.revision,
          metadata,
        };
        state.latestOverview = overview;
        state.overviewReceivedAt = new Date().toISOString();
        scheduleOverviewRender(false);
      }
      function scheduleOverviewRender(immediate) {
        if (!state.latestOverview) return;
        const render = () => {
          state.overviewRenderTimer = null;
          state.overviewLastRenderAt = Date.now();
          updateOverview({ data: state.latestOverview, __receivedAt: state.overviewReceivedAt || new Date().toISOString(), __frontendLatencyMs: 0 });
        };
        if (immediate) {
          if (state.overviewRenderTimer) window.clearTimeout(state.overviewRenderTimer);
          render();
          return;
        }
        if (state.overviewRenderTimer) return;
        const elapsed = Date.now() - state.overviewLastRenderAt;
        state.overviewRenderTimer = window.setTimeout(render, Math.max(0, OVERVIEW_RENDER_INTERVAL_MS - elapsed));
      }
      buildOverview();
      renderLogs();
      updateClock();
      loadInitialTasks();
      startTaskPolling();
      syncMapCanvasSize();
      if (window.ResizeObserver) {
        new ResizeObserver(redrawMapAfterResize).observe(els.mapCanvas);
      } else {
        window.addEventListener("resize", redrawMapAfterResize);
      }
      window.setInterval(updateClock, 1000);
      els.showDetections.addEventListener("change", () => {
        state.showDetections = Boolean(els.showDetections.checked);
        scheduleVideoDetectionDraw();
      });
      els.videoFrame.addEventListener("load", () => {
        els.videoFrame.hidden = false;
        els.videoEmpty.hidden = true;
        scheduleVideoDetectionDraw();
      });
      els.videoFrame.addEventListener("error", () => {
        els.videoFrame.hidden = true;
        els.videoEmpty.hidden = false;
        state.videoLastError = "浏览器无法显示已解码 JPEG 帧。";
        state.videoDecodeErrors += 1;
        els.videoEmpty.textContent = "已收到 MJPEG 字节流，但浏览器显示解码后的 JPEG 帧失败。";
      });
      window.addEventListener("resize", scheduleVideoDetectionDraw);
      startVideoStream();
      function setMapZoom(nextZoom) {
        const zoom = Math.min(8, Math.max(0.25, nextZoom));
        if (usesRvizMapView() || isMapToolActive()) {
          if (!state.mapRvizView) state.mapRvizView = { center: null, zoom: 1 };
          state.mapRvizView.zoom = zoom;
        } else {
          state.mapZoom = zoom;
        }
        requestMapDraw(false);
      }
      els.mapZoomIn.addEventListener("click", () => setMapZoom(mapViewZoom() * 1.25));
      els.mapZoomOut.addEventListener("click", () => setMapZoom(mapViewZoom() / 1.25));
      els.mapZoomReset.addEventListener("click", () => {
        if (isMapToolActive()) resetRvizMapView();
        else setMapZoom(1);
        requestMapDraw(false);
      });
      if (els.mapCenterRobot) {
        els.mapCenterRobot.addEventListener("click", () => centerRvizMapOnRobot());
      }
      function isMap3dVisible() {
        const container = document.getElementById("map3dContainer");
        return Boolean(container && container.style.display !== "none");
      }
      function finishGoalPick(event) {
        if (!state.goalPickDrag) return;
        if (state.pendingNavGoal && state.pendingNavGoal.status === "sending") {
          state.goalPickDrag = null;
          return;
        }
        const drag = state.goalPickDrag;
        state.goalPickDrag = null;
        const yaw = goalPickYawFromDrag(drag);
        void dispatchNavGoal(drag.start, yaw);
        if (event && event.pointerId !== undefined && els.mapCanvas.releasePointerCapture) {
          try { els.mapCanvas.releasePointerCapture(event.pointerId); } catch (_) {}
        }
        requestMapDraw(false);
      }
      function disableMapTools(clearStatus) {
        state.goalPickMode = false;
        state.publishPointMode = false;
        state.initialPosePickMode = false;
        state.goalPickDrag = null;
        state.initialPosePickDrag = null;
        state.publishPointClick = null;
        state.mapPanDrag = null;
        syncMapToolUi();
        if (clearStatus) updateGoalPickStatus("", "");
        requestMapDraw(false);
      }
      function finishInitialPosePick(event) {
        if (!state.initialPosePickDrag) return;
        const drag = state.initialPosePickDrag;
        state.initialPosePickDrag = null;
        const yaw = goalPickYawFromDrag(drag);
        void dispatchInitialPose(drag.start, yaw);
        if (event && event.pointerId !== undefined && els.mapCanvas.releasePointerCapture) {
          try { els.mapCanvas.releasePointerCapture(event.pointerId); } catch (_) {}
        }
        requestMapDraw(false);
      }
      if (els.mapInitialPoseToggle) {
        els.mapInitialPoseToggle.addEventListener("click", () => {
          const next = !state.initialPosePickMode;
          disableMapTools(false);
          state.initialPosePickMode = next;
          syncMapToolUi();
          if (state.initialPosePickMode) {
            resetRvizMapView();
            reevaluateCachedMapDisplay();
            updateGoalPickStatus(
              state.mapGrid
                ? "2D Pose 模式：左键拖拽设初始位姿 · 中/右键平移 · 滚轮缩放"
                : "2D Pose 已开启，但地图未显示。请先 activate_map 并启动 go2.sh。",
              state.mapGrid ? "pending" : "error",
            );
          } else {
            updateGoalPickStatus("", "");
          }
          requestMapDraw(false);
        });
      }
      if (els.mapGoalToggle) {
        els.mapGoalToggle.addEventListener("click", () => {
          const next = !state.goalPickMode;
          disableMapTools(false);
          state.goalPickMode = next;
          syncMapToolUi();
          if (state.goalPickMode) {
            resetRvizMapView();
            reevaluateCachedMapDisplay();
            updateGoalPickStatus(
              state.mapGrid
                ? "RViz 模式：左键设目标 · 中/右键平移 · 滚轮缩放"
                : "2D Goal 已开启，但地图未显示。请先 activate_map 并启动 go2.sh（mapping / explore / localization 均可）。",
              state.mapGrid ? "pending" : "error",
            );
          } else {
            updateGoalPickStatus("", "");
          }
          requestMapDraw(false);
        });
      }
      if (els.mapPublishPointToggle) {
        els.mapPublishPointToggle.addEventListener("click", () => {
          const next = !state.publishPointMode;
          disableMapTools(false);
          state.publishPointMode = next;
          syncMapToolUi();
          if (state.publishPointMode) {
            resetRvizMapView();
            reevaluateCachedMapDisplay();
            updateGoalPickStatus(
              state.mapGrid
                ? "选点模式：左键发布 /clicked_point（不导航）"
                : "选点已开启，但地图未显示。请先 activate_map 并启动 go2.sh。",
              state.mapGrid ? "pending" : "error",
            );
          } else {
            updateGoalPickStatus("", "");
          }
          requestMapDraw(false);
        });
      }
      function finishMapPan(event) {
        if (!state.mapPanDrag) return;
        state.mapPanDrag = null;
        syncGoalPickUi();
        if (event && event.pointerId !== undefined && els.mapCanvas.releasePointerCapture) {
          try { els.mapCanvas.releasePointerCapture(event.pointerId); } catch (_) {}
        }
      }
      function finishPublishPoint(event) {
        if (!state.publishPointClick) return;
        const click = state.publishPointClick;
        state.publishPointClick = null;
        if (event && event.pointerId !== undefined && els.mapCanvas.releasePointerCapture) {
          try { els.mapCanvas.releasePointerCapture(event.pointerId); } catch (_) {}
        }
        void dispatchClickedPoint(click.mapPoint);
      }
      els.mapCanvas.addEventListener("contextmenu", (event) => {
        if (isMapToolActive()) event.preventDefault();
      });
      els.mapCanvas.addEventListener("pointerdown", (event) => {
        if (!isMapToolActive() || isMap3dVisible()) return;
        const pointer = canvasPointer(event);
        if (event.button === 1 || event.button === 2) {
          event.preventDefault();
          state.mapPanDrag = {
            pointerStart: pointer,
            centerStart: getMapRvizViewCenter(),
          };
          syncMapToolUi();
          if (els.mapCanvas.setPointerCapture) els.mapCanvas.setPointerCapture(event.pointerId);
          return;
        }
        if (event.button !== 0) return;
        event.preventDefault();
        const anchor = cameraAnchor();
        const mapPoint = canvasToMap(pointer.x, pointer.y, anchor);
        if (state.publishPointMode) {
          state.publishPointClick = { pointer, anchor, mapPoint };
          if (els.mapCanvas.setPointerCapture) els.mapCanvas.setPointerCapture(event.pointerId);
          return;
        }
        if (state.initialPosePickMode) {
          state.initialPosePickDrag = {
            pointerStart: pointer,
            pointerCurrent: pointer,
            anchor,
            start: mapPoint,
            current: mapPoint,
          };
          if (els.mapCanvas.setPointerCapture) els.mapCanvas.setPointerCapture(event.pointerId);
          requestMapDraw(false);
          return;
        }
        state.goalPickDrag = {
          pointerStart: pointer,
          pointerCurrent: pointer,
          anchor,
          start: mapPoint,
          current: mapPoint,
        };
        if (els.mapCanvas.setPointerCapture) els.mapCanvas.setPointerCapture(event.pointerId);
        requestMapDraw(false);
      });
      els.mapCanvas.addEventListener("pointermove", (event) => {
        if (state.mapPanDrag) {
          const pointer = canvasPointer(event);
          const anchor = cameraAnchor();
          const scale = cameraScale(anchor);
          if (!Number.isFinite(scale) || scale <= 0) return;
          const dx = pointer.x - state.mapPanDrag.pointerStart.x;
          const dy = pointer.y - state.mapPanDrag.pointerStart.y;
          if (!state.mapRvizView) state.mapRvizView = { center: null, zoom: 1 };
          state.mapRvizView.center = {
            x: state.mapPanDrag.centerStart.x - dx / scale,
            y: state.mapPanDrag.centerStart.y + dy / scale,
          };
          requestMapDraw(false);
          return;
        }
        if (state.initialPosePickDrag) {
          const pointer = canvasPointer(event);
          state.initialPosePickDrag.pointerCurrent = pointer;
          state.initialPosePickDrag.current = canvasToMap(pointer.x, pointer.y, state.initialPosePickDrag.anchor);
          requestMapDraw(false);
          return;
        }
        if (!state.goalPickDrag) return;
        const pointer = canvasPointer(event);
        state.goalPickDrag.pointerCurrent = pointer;
        state.goalPickDrag.current = canvasToMap(pointer.x, pointer.y, state.goalPickDrag.anchor);
        requestMapDraw(false);
      });
      els.mapCanvas.addEventListener("pointerup", (event) => {
        if (state.mapPanDrag) {
          finishMapPan(event);
          return;
        }
        if (state.publishPointClick) {
          finishPublishPoint(event);
          return;
        }
        if (state.initialPosePickDrag) {
          finishInitialPosePick(event);
          return;
        }
        finishGoalPick(event);
      });
      els.mapCanvas.addEventListener("pointercancel", (event) => {
        if (state.mapPanDrag) {
          finishMapPan(event);
          return;
        }
        if (state.publishPointClick) {
          finishPublishPoint(event);
          return;
        }
        if (state.initialPosePickDrag) {
          finishInitialPosePick(event);
          return;
        }
        finishGoalPick(event);
      });
      els.mapCanvas.addEventListener("wheel", (event) => {
        event.preventDefault();
        if (isMapToolActive()) {
          const pointer = canvasPointer(event);
          zoomRvizMapAt(pointer, event.deltaY < 0 ? 1.12 : 1 / 1.12);
          requestMapDraw(false);
          return;
        }
        setMapZoom(state.mapZoom * (event.deltaY < 0 ? 1.12 : 1 / 1.12));
      }, { passive: false });
      els.logLevelFilter.addEventListener("change", () => {
        state.logFilter = els.logLevelFilter.value;
        renderLogs();
      });
      els.logSearch.addEventListener("input", () => {
        state.logSearch = els.logSearch.value;
        renderLogs();
      });
      connectStreams();
      document.getElementById('mapModeToggle').addEventListener('click', () => {
        const c = document.getElementById('mapCanvas');
        const d = document.getElementById('map3dContainer');
        const btn = document.getElementById('mapModeToggle');
        const is3d = d.style.display !== 'none';
        c.style.display = is3d ? '' : 'none';
        d.style.display = is3d ? 'none' : '';
        btn.textContent = is3d ? '3D' : '2D';
        if (!is3d) {
          disableMapTools(true);
          if (window.pointcloud3d) window.pointcloud3d.init(d);
        }
      });
    })();
  </script>
  <script type="module" src="/static/js/pointcloud3d.js"></script>
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


def _build_map_diagnostics(snapshot: Any, *, refresh_error: Optional[str] = None) -> Dict[str, Any]:
    metadata = dict(getattr(snapshot, "metadata", {}) or {})
    live_mapping = metadata.get("live_mapping_diagnostics") if isinstance(metadata.get("live_mapping_diagnostics"), dict) else {}
    provider_status = metadata.get("provider_status") if isinstance(metadata.get("provider_status"), dict) else {}
    live_provider = live_mapping.get("live_map_provider") if isinstance(live_mapping.get("live_map_provider"), dict) else {}
    map_stream = live_mapping.get("rtabmap_readiness", {}).get("map_stream") if isinstance(live_mapping.get("rtabmap_readiness"), dict) else {}
    if not isinstance(map_stream, dict):
        map_stream = {}
    status = {**dict(live_provider), **dict(provider_status)}
    occupancy_grid = getattr(snapshot, "occupancy_grid", None)
    cost_map = getattr(snapshot, "cost_map", None)
    source_timestamp = _map_source_timestamp(snapshot)
    last_message_at = _parse_debug_timestamp(status.get("last_message_at") or status.get("latest_occupancy_received_at"))
    gateway_timestamp = utc_now()
    cache_age = max(0.0, (gateway_timestamp - last_message_at).total_seconds()) if last_message_at is not None else None
    source_age = max(0.0, (gateway_timestamp - source_timestamp).total_seconds()) if source_timestamp is not None else None
    expected_frame_id = status.get("expected_frame_id") or _map_expected_frame_id(snapshot)
    frame_id = getattr(snapshot, "frame_id", None)
    frame_mismatch = bool(status.get("frame_mismatch")) or bool(
        expected_frame_id and frame_id and str(expected_frame_id) != str(frame_id)
    )
    message_count = _int_or_none(status.get("occupancy_message_count") or status.get("message_count"))
    cost_message_count = _int_or_none(status.get("cost_map_message_count"))
    content_revision = _int_or_none(
        map_stream.get("content_revision") or status.get("occupancy_revision") or _metric_value(status, "occupancy_metrics", "content_revision")
    )
    costmap_revision = _int_or_none(
        map_stream.get("costmap_revision") or status.get("cost_map_revision") or _metric_value(status, "cost_map_metrics", "content_revision")
    )
    occupancy_content_hash = metadata.get("occupancy_content_hash") or map_stream.get("content_hash") or _metric_value(status, "occupancy_metrics", "content_hash")
    cost_map_content_hash = metadata.get("cost_map_content_hash") or map_stream.get("costmap_hash") or _metric_value(status, "cost_map_metrics", "content_hash")
    diagnostics = {
        "source_timestamp": source_timestamp,
        "gateway_timestamp": gateway_timestamp,
        "time_skew": status.get("time_skew_sec"),
        "frame_id": frame_id,
        "expected_frame_id": expected_frame_id,
        "sequence": message_count,
        "revision": getattr(snapshot, "revision", None),
        "message_count": message_count,
        "provider_status": status.get("provider_status") or ("available" if status.get("started") and status.get("ros_ready") else "unknown"),
        "frame_mismatch": frame_mismatch,
        "cache_age": cache_age,
        "source_age": source_age,
        "last_message_at": last_message_at,
        "message_count_delta": map_stream.get("message_count_delta"),
        "stream_age_ms": map_stream.get("stream_age_ms") if map_stream.get("stream_age_ms") is not None else (cache_age * 1000.0 if cache_age is not None else None),
        "content_revision": content_revision,
        "occupancy_content_hash": occupancy_content_hash,
        "content_hash": occupancy_content_hash,
        "known_cell_delta": map_stream.get("known_cell_delta") if map_stream.get("known_cell_delta") is not None else _metric_value(status, "occupancy_metrics", "known_cell_delta"),
        "known_cell_count": map_stream.get("known_cell_count") if map_stream.get("known_cell_count") is not None else _metric_value(status, "occupancy_metrics", "known_cell_count"),
        "costmap_delta": map_stream.get("costmap_delta") if map_stream.get("costmap_delta") is not None else _metric_value(status, "cost_map_metrics", "costmap_delta"),
        "costmap_revision": costmap_revision,
        "cost_map_content_hash": cost_map_content_hash,
        "costmap_hash": cost_map_content_hash,
        "cost_map_message_count": cost_message_count,
        "occupancy_grid_ready": occupancy_grid is not None,
        "cost_map_ready": cost_map is not None,
        "live_mapping_ready": live_mapping.get("ready"),
        "waiting_for": live_mapping.get("waiting_for"),
        "waiting_message": live_mapping.get("message"),
        "refresh_error": refresh_error,
        "diagnostic_roles": {
            "pointcloud_map": "health_check_input",
            "pointcloud": "side_channel_not_map_source",
            "map_source": "rtabmap_occupancy_grid",
        },
    }
    if message_count is not None and message_count <= 0:
        diagnostics["message_count_warning"] = "/map message_count 未增长或尚未收到首帧。"
    return diagnostics


def _map_source_timestamp(snapshot: Any) -> Optional[datetime]:
    for layer_name in ("occupancy_grid", "cost_map", "semantic_map"):
        timestamp = _timestamp_from_value(getattr(snapshot, layer_name, None))
        if timestamp is not None:
            return timestamp
    return _timestamp_from_value(snapshot)


def _map_expected_frame_id(snapshot: Any) -> Optional[str]:
    metadata = dict(getattr(snapshot, "metadata", {}) or {})
    input_contract = metadata.get("input_contract") if isinstance(metadata.get("input_contract"), dict) else {}
    tf_contract = input_contract.get("tf") if isinstance(input_contract.get("tf"), dict) else {}
    return tf_contract.get("map_frame") or metadata.get("frame_id")


def _status_for_map_diagnostics(diagnostics: Dict[str, Any], cache_timestamp: Optional[datetime]) -> DebugDataStatus:
    if diagnostics.get("refresh_error") and not diagnostics.get("occupancy_grid_ready"):
        return DebugDataStatus.ERROR
    if not diagnostics.get("occupancy_grid_ready"):
        return DebugDataStatus.MISSING
    if diagnostics.get("frame_mismatch"):
        return DebugDataStatus.STALE
    if cache_timestamp is not None and _is_stale(cache_timestamp, "map", utc_now()):
        return DebugDataStatus.STALE
    return DebugDataStatus.AVAILABLE


def _parse_debug_timestamp(value: Any) -> Optional[datetime]:
    if isinstance(value, datetime):
        return value if value.tzinfo is not None else value.replace(tzinfo=timezone.utc)
    if not value:
        return None
    try:
        parsed = datetime.fromisoformat(str(value).replace("Z", "+00:00"))
    except ValueError:
        return None
    return parsed if parsed.tzinfo is not None else parsed.replace(tzinfo=timezone.utc)


def _metric_value(status: Dict[str, Any], key: str, metric: str) -> Any:
    value = status.get(key)
    return value.get(metric) if isinstance(value, dict) else None


def _int_or_none(value: Any) -> Optional[int]:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


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



def _runtime_governance_message(summary: Any) -> str:
    profile = dict(getattr(summary, "rtabmap_profile", {}) or {})
    asset = dict(getattr(summary, "geometry_asset", {}) or {})
    session = dict(getattr(summary, "localization_session", {}) or {})
    return "route=%s profile=%s asset=%s localization=%s" % (
        profile.get("route") or "unknown",
        profile.get("status") or "unknown",
        "available" if asset.get("available") else "missing",
        session.get("status") or "missing",
    )


def _runtime_governance_metadata(summary: Any) -> Dict[str, Any]:
    profile = dict(getattr(summary, "rtabmap_profile", {}) or {})
    asset = dict(getattr(summary, "geometry_asset", {}) or {})
    session = dict(getattr(summary, "localization_session", {}) or {})
    relocalization = dict(getattr(summary, "last_relocalization", {}) or {})
    policy = dict(getattr(summary, "navigation_control_policy", {}) or {})
    front_align = policy.get("front_align") if isinstance(policy.get("front_align"), dict) else {}
    rotate = policy.get("rotate_relocalization") if isinstance(policy.get("rotate_relocalization"), dict) else {}
    return {
        "profile_name": profile.get("name"),
        "profile_status": profile.get("status"),
        "route": profile.get("route") or "unknown",
        "geometry_asset_available": bool(asset.get("available")),
        "database_present": asset.get("database_present"),
        "pcd_present": asset.get("pcd_present"),
        "localization_status": session.get("status"),
        "last_relocalization_available": bool(relocalization.get("available")),
        "last_relocalization_failure": relocalization.get("failure_reason"),
        "front_align_triggered": bool(front_align.get("triggered", False)),
        "rotate_relocalization_triggered": bool(rotate.get("triggered", False)),
    }


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


def _tail_log_file(path: Path, *, cursor: int, limit: int, level: str, source: str) -> Tuple[List[DebugLogEntry], int, int, Dict[str, Any]]:
    started_at = time.perf_counter()
    stat_started = time.perf_counter()
    if not path.exists() or not path.is_file():
        return [], 0, 0, {
            "file_stat_ms": (time.perf_counter() - stat_started) * 1000.0,
            "file_read_ms": 0.0,
            "bytes_read": 0,
            "raw_lines": 0,
            "tail_compute_ms": (time.perf_counter() - started_at) * 1000.0,
        }
    file_size = path.stat().st_size
    stat_ms = (time.perf_counter() - stat_started) * 1000.0
    if cursor <= 0 and file_size > 262144:
        start = max(0, file_size - 262144)
    else:
        start = cursor if 0 <= cursor <= file_size else 0
    entries: List[DebugLogEntry] = []
    raw_lines = 0
    bytes_read = 0
    read_started = time.perf_counter()
    with path.open("rb") as stream:
        stream.seek(start)
        if start > 0:
            skipped = stream.readline()
            bytes_read += len(skipped)
        while True:
            raw = stream.readline()
            if not raw:
                break
            raw_lines += 1
            bytes_read += len(raw)
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
    read_ms = (time.perf_counter() - read_started) * 1000.0
    return entries, file_size, file_size, {
        "file_stat_ms": stat_ms,
        "file_read_ms": read_ms,
        "bytes_read": bytes_read,
        "raw_lines": raw_lines,
        "tail_start_cursor": start,
        "tail_end_cursor": file_size,
        "tail_compute_ms": (time.perf_counter() - started_at) * 1000.0,
    }


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
    metadata = dict(getattr(state, "metadata", {}) or {})
    return {
        "frontier_count": getattr(state, "frontier_count", None),
        "waiting_for": metadata.get("waiting_for"),
        "failure_code": metadata.get("failure_code"),
        "frontier_freshness": metadata.get("frontier_freshness"),
        "map_revision": metadata.get("map_revision"),
        "costmap_revision": metadata.get("costmap_revision"),
        "nav2_ready": metadata.get("nav2_ready"),
        "nav2_readiness": metadata.get("nav2_readiness"),
    }



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
        "not_a_pointcloud_map": True,
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
