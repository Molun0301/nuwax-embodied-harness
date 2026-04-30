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
    stream_path = "/streams/ws" + (f"?topics={query}" if query else "")
    return JSONResponse(
        status_code=410,
        content={
            "code": "stream_required",
            "message": "实时数据调试 HTTP 快照入口已下线，请订阅实时流。",
            "stream": stream_path,
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
        return _stream_migration_response('robot.state', 'map.frame', 'localization.pose', 'navigation.state', 'exploration.state', 'logs.entry', 'tasks.state')

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


    return router


def _render_debug_page(access_token: str) -> str:
    debug_access_token = json.dumps(access_token)
    html = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>nuwax_robot_bridge 只读机器人监控驾驶舱</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #07111f;
      --bg-radial-a: rgba(34, 211, 238, 0.18);
      --bg-radial-b: rgba(124, 58, 237, 0.14);
      --surface: rgba(11, 18, 32, 0.92);
      --surface-strong: rgba(15, 23, 42, 0.96);
      --panel: rgba(15, 23, 42, 0.92);
      --panel-soft: rgba(30, 41, 59, 0.74);
      --panel-strong: rgba(30, 41, 59, 0.96);
      --border: rgba(148, 163, 184, 0.22);
      --border-soft: rgba(148, 163, 184, 0.14);
      --border-strong: rgba(34, 211, 238, 0.42);
      --text: #f8fafc;
      --muted: #94a3b8;
      --muted-strong: #cbd5e1;
      --good: #86efac;
      --warn: #fde68a;
      --bad: #fca5a5;
      --info: #93c5fd;
      --accent: #22d3ee;
      --accent-strong: #67e8f9;
      --map-line: #22d3ee;
      --map-robot: #fbbf24;
      --unknown: #334155;
      --free: #dbeafe;
      --occupied: #020617;
      --frontier: #f472b6;
      --shadow: 0 24px 70px rgba(0, 0, 0, 0.34);
      --shadow-soft: 0 14px 38px rgba(0, 0, 0, 0.24);
    }
    * { box-sizing: border-box; }
    html { scroll-behavior: smooth; }
    body {
      margin: 0;
      padding: 20px;
      min-height: 100vh;
      background:
        radial-gradient(circle at 8% 8%, var(--bg-radial-a), transparent 32rem),
        radial-gradient(circle at 92% 6%, var(--bg-radial-b), transparent 36rem),
        linear-gradient(135deg, #020617 0%, var(--bg) 46%, #0f172a 100%);
      color: var(--text);
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", "Noto Sans SC", sans-serif;
      letter-spacing: 0;
    }
    .app-shell { max-width: 1780px; margin: 0 auto; }
    .topbar { display: grid; grid-template-columns: minmax(300px, 0.62fr) minmax(560px, 1.38fr); gap: 16px; align-items: stretch; margin-bottom: 16px; }
    .brand-block, .global-status {
      position: relative;
      overflow: hidden;
      background: linear-gradient(145deg, rgba(15, 23, 42, 0.94), rgba(15, 23, 42, 0.72));
      border: 1px solid var(--border);
      border-radius: 20px;
      padding: 18px;
      box-shadow: var(--shadow-soft);
      backdrop-filter: blur(18px);
    }
    .brand-block::before, .global-status::before, .panel::before, .card::before {
      content: "";
      position: absolute;
      inset: 0 0 auto;
      height: 1px;
      background: linear-gradient(90deg, transparent, rgba(103, 232, 249, 0.7), transparent);
      pointer-events: none;
    }
    .brand-row { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; margin-top: 8px; }
    h1, h2, h3 { margin: 0; }
    h1 { font-size: clamp(26px, 2.1vw, 38px); line-height: 1.08; font-weight: 780; letter-spacing: -0.04em; }
    h2 { font-size: 17px; line-height: 1.3; font-weight: 720; letter-spacing: -0.01em; }
    h3 { font-size: 14px; line-height: 1.35; color: var(--muted-strong); font-weight: 680; }
    code { color: #a5f3fc; word-break: break-all; }
    canvas { display: block; width: 100%; border-radius: 18px; background: #020617; border: 1px solid rgba(148, 163, 184, 0.18); box-shadow: inset 0 0 0 1px rgba(15, 23, 42, 0.92); }
    .muted { color: var(--muted); }
    .small { font-size: 12px; line-height: 1.55; }
    .eyebrow { color: var(--accent-strong); font-size: 12px; font-weight: 780; letter-spacing: 0.12em; text-transform: uppercase; }
    .readonly { color: #bae6fd; }
    .grid { display: grid; gap: 14px; }
    .mission-layout { display: grid; grid-template-columns: 190px minmax(0, 1fr); gap: 16px; align-items: start; }
    .side-nav {
      position: sticky;
      top: 18px;
      display: grid;
      gap: 7px;
      padding: 10px;
      background: rgba(15, 23, 42, 0.78);
      border: 1px solid var(--border);
      border-radius: 18px;
      box-shadow: var(--shadow-soft);
      backdrop-filter: blur(14px);
    }
    .side-nav a {
      color: var(--muted-strong);
      text-decoration: none;
      border: 1px solid transparent;
      border-radius: 13px;
      padding: 10px 11px;
      font-size: 13px;
      font-weight: 650;
      transition: border-color 0.16s ease, background 0.16s ease, color 0.16s ease, transform 0.16s ease;
    }
    .side-nav a:hover { border-color: var(--border-strong); background: rgba(34, 211, 238, 0.1); color: var(--text); transform: translateX(2px); }
    .mission-content { display: grid; gap: 14px; min-width: 0; }
    .section-band { scroll-margin-top: 18px; display: grid; gap: 14px; }
    .section-title { display: flex; justify-content: space-between; align-items: center; gap: 12px; padding: 0 3px; }
    .overview-grid { grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); align-items: stretch; }
    .mission-grid { grid-template-columns: minmax(260px, 0.8fr) minmax(260px, 0.8fr) minmax(320px, 1fr); }
    .workspace { grid-template-columns: minmax(680px, 1.28fr) minmax(440px, 0.72fr); align-items: start; }
    .right-rail { grid-template-columns: 1fr; }
    .card, .panel {
      position: relative;
      overflow: hidden;
      background: linear-gradient(180deg, var(--panel), rgba(15, 23, 42, 0.82));
      border: 1px solid var(--border);
      border-radius: 20px;
      padding: 15px;
      box-shadow: var(--shadow-soft);
    }
    .panel.compact { padding: 13px; }
    .status-card { display: grid; gap: 10px; min-height: 176px; align-content: start; }
    .status-card .card-head { align-items: flex-start; }
    .status-card .kv { grid-template-columns: minmax(98px, 0.38fr) minmax(0, 1fr); }
    .status-card .kv div { min-width: 0; }
    .status-card .mono { white-space: normal; word-break: normal; overflow-wrap: break-word; }
    .card-head { display: flex; align-items: center; justify-content: space-between; gap: 12px; margin-bottom: 12px; }
    .global-status { display: grid; grid-template-columns: repeat(4, minmax(126px, 1fr)); gap: 10px; align-items: stretch; }
    .metric-strip { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; justify-content: flex-end; }
    .metric {
      border: 1px solid var(--border-soft);
      border-radius: 16px;
      padding: 10px 11px;
      background: rgba(15, 23, 42, 0.72);
      color: var(--muted-strong);
      min-width: 126px;
      box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.035);
    }
    .metric b { display: block; font-size: 15px; color: var(--text); font-variant-numeric: tabular-nums; letter-spacing: -0.01em; }
    .metric span { display: block; color: var(--muted); font-size: 11px; margin-top: 3px; }
    .pill {
      display: inline-flex;
      align-items: center;
      gap: 7px;
      border-radius: 999px;
      padding: 5px 10px;
      font-size: 12px;
      font-weight: 700;
      border: 1px solid var(--border);
      background: rgba(15, 23, 42, 0.78);
      color: var(--muted-strong);
      white-space: nowrap;
      box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.035);
    }
    .pill::before { content: ""; width: 8px; height: 8px; border-radius: 999px; background: var(--muted); box-shadow: 0 0 0 3px rgba(148, 163, 184, 0.12); }
    .pill.available { border-color: rgba(134, 239, 172, 0.44); background: rgba(22, 101, 52, 0.24); color: #dcfce7; }
    .pill.available::before { background: var(--good); box-shadow: 0 0 16px rgba(134, 239, 172, 0.62); }
    .pill.stale { border-color: rgba(253, 230, 138, 0.46); background: rgba(113, 63, 18, 0.26); color: #fef3c7; }
    .pill.stale::before { background: var(--warn); box-shadow: 0 0 16px rgba(253, 230, 138, 0.55); }
    .pill.missing, .pill.unsupported { border-color: rgba(148, 163, 184, 0.28); background: rgba(51, 65, 85, 0.32); color: #cbd5e1; }
    .pill.missing::before, .pill.unsupported::before { background: var(--unknown); }
    .pill.error { border-color: rgba(252, 165, 165, 0.5); background: rgba(127, 29, 29, 0.3); color: #fee2e2; }
    .pill.error::before { background: var(--bad); box-shadow: 0 0 16px rgba(252, 165, 165, 0.6); }
    .estop { border-color: rgba(252, 165, 165, 0.58); color: #fee2e2; background: rgba(127, 29, 29, 0.22); }
    .kv { display: grid; grid-template-columns: minmax(112px, 0.42fr) minmax(0, 1fr); gap: 6px 10px; font-size: 12px; line-height: 1.45; }
    .kv div:nth-child(odd) { color: var(--muted); }
    .kv div:nth-child(even) { color: var(--muted-strong); }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace; overflow-wrap: break-word; word-break: normal; font-variant-numeric: tabular-nums; }
    .empty, .error-text { color: #fee2e2; background: rgba(127, 29, 29, 0.2); border: 1px solid rgba(252, 165, 165, 0.28); border-radius: 14px; padding: 10px 11px; }
    .notice { color: #fef3c7; background: rgba(113, 63, 18, 0.22); border: 1px solid rgba(253, 230, 138, 0.28); border-radius: 14px; padding: 10px 11px; }
    .panel-meta { margin-top: 10px; }
    .canvas-wrap { position: relative; background: #020617; border-radius: 18px; padding: 1px; border: 1px solid rgba(148, 163, 184, 0.16); overflow: hidden; }
    #mapCanvas { height: min(64vh, 700px); min-height: 450px; image-rendering: pixelated; }
    .map-toolbar { position: absolute; z-index: 2; left: 12px; top: 12px; display: flex; gap: 7px; align-items: center; flex-wrap: wrap; }
    .map-control { min-height: 30px; border: 1px solid rgba(34, 211, 238, 0.34); border-radius: 11px; background: rgba(2, 6, 23, 0.78); color: #cffafe; padding: 5px 9px; font: inherit; font-size: 12px; font-weight: 720; cursor: pointer; }
    .map-control:hover { border-color: var(--accent-strong); background: rgba(8, 47, 73, 0.72); }
    .map-chip { border: 1px solid rgba(34, 211, 238, 0.3); background: rgba(2, 6, 23, 0.74); border-radius: 999px; padding: 6px 9px; font-size: 12px; color: #cffafe; backdrop-filter: blur(10px); }
    .map-stream-chip { position: absolute; z-index: 2; right: 12px; top: 12px; max-width: min(430px, calc(100% - 24px)); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
    .video-stage { position: relative; border-radius: 18px; overflow: hidden; border: 1px solid rgba(148, 163, 184, 0.18); background: #020617; box-shadow: inset 0 0 0 1px rgba(15, 23, 42, 0.9); }
    .video-frame { display: block; width: 100%; min-height: 230px; max-height: 360px; object-fit: contain; background: #020617; }
    .video-overlay { position: absolute; left: 12px; right: 12px; bottom: 12px; display: flex; justify-content: space-between; gap: 8px; pointer-events: none; }
    .overlay-chip { border: 1px solid rgba(34, 211, 238, 0.36); background: rgba(2, 6, 23, 0.78); border-radius: 999px; padding: 6px 9px; font-size: 12px; color: #cffafe; backdrop-filter: blur(10px); }
    .legend { display: flex; flex-wrap: wrap; gap: 10px; margin-top: 10px; }
    .legend-item { display: inline-flex; align-items: center; gap: 6px; color: var(--muted); font-size: 12px; }
    .swatch { width: 13px; height: 13px; border: 1px solid rgba(226, 232, 240, 0.42); border-radius: 4px; }
    .logs { display: grid; gap: 8px; max-height: 360px; overflow: auto; padding-right: 2px; }
    .log-line { border: 1px solid var(--border-soft); border-left: 3px solid rgba(147, 197, 253, 0.64); border-radius: 13px; padding: 9px 10px; background: rgba(2, 6, 23, 0.46); font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace; font-size: 12px; line-height: 1.45; }
    .log-line .meta { color: #93c5fd; font-size: 12px; margin-bottom: 4px; }
    .log-line.warn, .log-line.warning { border-left-color: rgba(253, 230, 138, 0.82); }
    .log-line.warn .meta, .log-line.warning .meta { color: #fef3c7; }
    .log-line.error, .log-line.critical { border-left-color: rgba(252, 165, 165, 0.88); }
    .log-line.error .meta, .log-line.critical .meta { color: #fecaca; }
    .toolbar { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
    .field { min-height: 34px; border: 1px solid var(--border); border-radius: 12px; background: rgba(2, 6, 23, 0.5); color: var(--text); padding: 7px 10px; font: inherit; font-size: 12px; outline: none; }
    .field:focus { border-color: var(--border-strong); box-shadow: 0 0 0 3px rgba(34, 211, 238, 0.12); }
    .service-list, .event-list, .task-list { display: grid; gap: 8px; }
    .service-row, .task-row { display: grid; grid-template-columns: minmax(120px, 1fr) auto; gap: 8px; align-items: center; border: 1px solid var(--border-soft); border-radius: 13px; padding: 9px 10px; background: rgba(15, 23, 42, 0.62); }
    .service-row .meta, .task-row .meta { color: var(--muted); font-size: 12px; }
    .health-line { height: 6px; border-radius: 999px; background: linear-gradient(90deg, var(--good), var(--accent)); opacity: 0.95; }
    .health-line.warn { background: linear-gradient(90deg, var(--warn), #fb923c); }
    .health-line.bad { background: linear-gradient(90deg, var(--bad), #ef4444); }
    .split-two { grid-template-columns: 1fr 1fr; }
    @media (max-width: 1280px) {
      .topbar, .mission-layout, .workspace, .mission-grid { grid-template-columns: 1fr; }
      .side-nav { position: static; grid-template-columns: repeat(auto-fit, minmax(128px, 1fr)); }
      .side-nav a:hover { transform: none; }
      .global-status { grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); }
      .overview-grid { grid-template-columns: repeat(auto-fit, minmax(240px, 1fr)); }
    }
    @media (max-width: 720px) {
      body { padding: 10px; }
      .brand-block, .global-status, .panel, .card { border-radius: 16px; padding: 12px; }
      .split-two { grid-template-columns: 1fr; }
      .metric-strip { justify-content: flex-start; }
      .global-status { grid-template-columns: 1fr 1fr; }
      .kv { grid-template-columns: 1fr; }
      .overview-grid { grid-template-columns: 1fr; }
      #mapCanvas { min-height: 320px; }
    }
  </style>
</head>
<body>
  <div class="app-shell">
  <header class="topbar">
    <div class="brand-block">
      <div class="eyebrow">只读机器人观测</div>
      <div class="brand-row">
        <h1 id="robotName">nuwax_robot_bridge</h1>
        <span id="robotModel" class="pill missing">型号：--</span>
      </div>
      <div class="muted small panel-meta">机器人监控驾驶舱 · 只读观察、诊断和空间态势汇总</div>
    </div>
    <div class="global-status">
      <div class="metric"><b id="connectionText">connecting</b><span>网关连接</span></div>
      <div class="metric"><b id="runtimeMode">--</b><span>运行模式</span></div>
      <div class="metric"><b id="batteryMetric">--</b><span>电池状态</span></div>
      <div class="metric"><b id="networkLatency" class="mono">--</b><span>HTTP（超文本传输协议）往返</span></div>
      <div class="metric"><b id="resourceMetric">--</b><span>资源/任务</span></div>
      <div class="metric"><b id="clockNow" class="mono">--</b><span>当前时间</span></div>
      <div class="metric"><b id="lastReceive" class="mono">--</b><span>数据接收</span></div>
      <div class="metric"><b>&lt;500 ms</b><span>P95（第 95 百分位）目标</span></div>
      <span id="estopStatus" class="pill estop">急停状态：只读展示</span>
    </div>
  </header>

  <div class="mission-layout">
    <aside class="side-nav" aria-label="页面导航">
      <a href="#overview">运行总览</a>
      <a href="#live-map">2D 地图</a>
      <a href="#video-perception">视频感知</a>
      <a href="#navigation">导航态势</a>
      <a href="#services">服务矩阵</a>
      <a href="#logs-section">日志</a>
      <a href="#runtime-settings">运行治理</a>
    </aside>

    <div class="mission-content">
  <section id="overview" class="section-band">
    <div class="section-title">
      <h2>运行态总览</h2>
      <span id="missionStatus" class="pill missing">mission: warming</span>
    </div>
    <section id="overviewGrid" class="grid overview-grid" aria-label="运行态总览"></section>
    <div class="grid mission-grid">
      <section class="panel compact">
        <div class="card-head"><h3>当前任务</h3><span id="taskStatus" class="pill missing">missing</span></div>
        <div id="taskSummary" class="task-list"></div>
      </section>
      <section class="panel compact">
        <div class="card-head"><h3>位姿与运动</h3><span id="poseStatus" class="pill missing">missing</span></div>
        <div id="missionPoseMeta" class="kv"></div>
      </section>
      <section class="panel compact">
        <div class="card-head"><h3>传感器链路</h3><span id="sensorStatus" class="pill missing">missing</span></div>
        <div id="sensorSummary" class="kv"></div>
      </section>
    </div>
  </section>

  <main class="grid workspace">
    <section id="live-map" class="panel">
      <div class="card-head">
        <h2>空间态势 · 2D 地图 / 定位 / 轨迹</h2>
        <span id="mapStatus" class="pill missing">missing</span>
      </div>
      <div class="canvas-wrap">
        <div class="map-toolbar" aria-label="地图缩放控制">
          <button id="mapZoomOut" class="map-control" type="button">-</button>
          <span id="mapZoomValue" class="map-chip">100%</span>
          <button id="mapZoomIn" class="map-control" type="button">+</button>
          <button id="mapZoomReset" class="map-control" type="button">reset</button>
        </div>
        <span id="mapStreamStatus" class="map-chip map-stream-chip">等待地图首帧</span>
        <canvas id="mapCanvas" width="960" height="620"></canvas>
      </div>
      <div class="legend">
        <span class="legend-item"><span class="swatch" style="background: var(--unknown)"></span>未知</span>
        <span class="legend-item"><span class="swatch" style="background: var(--free)"></span>可通行</span>
        <span class="legend-item"><span class="swatch" style="background: var(--occupied)"></span>占据</span>
        <span class="legend-item"><span class="swatch" style="background: var(--map-line)"></span>轨迹</span>
        <span class="legend-item"><span class="swatch" style="background: var(--map-robot)"></span>机器人</span>
        <span class="legend-item"><span class="swatch" style="background: var(--frontier)"></span>探索前沿</span>
      </div>
      <div id="mapEmpty" class="empty panel-meta">等待地图数据。</div>
      <div id="mapMeta" class="kv panel-meta"></div>
      <div class="card-head" style="margin-top: 14px;">
        <h3>定位/轨迹</h3>
        <span id="localizationStatus" class="pill missing">missing</span>
      </div>
      <div id="localizationMeta" class="kv"></div>
    </section>

    <div class="grid right-rail">
      <section id="navigation" class="panel">
        <div class="card-head">
          <h2>导航态势 / frontier（探索前沿）</h2>
          <span id="navigationStatus" class="pill missing">missing</span>
        </div>
        <div id="navigationMeta" class="kv"></div>
        <div id="frontierNotice" class="notice panel-meta">等待探索前沿候选几何。</div>
      </section>

      <section id="runtime-settings" class="panel">
        <div class="card-head">
          <h2>RTAB-Map 运行治理</h2>
          <span id="governanceStatus" class="pill missing">missing</span>
        </div>
        <div id="governanceMeta" class="kv"></div>
        <div class="muted small panel-meta">只读展示 profile、几何资产、重定位质量和导航门控摘要。</div>
      </section>

      <section id="video-perception" class="panel">
        <div class="card-head">
          <h2>视频感知</h2>
          <span id="videoStatus" class="pill missing">missing</span>
        </div>
        <div class="video-stage">
          <img id="videoFrame" class="video-frame" alt="实时视频帧" hidden>
          <div class="video-overlay">
            <span id="visionModelStatus" class="overlay-chip">本地模型：--</span>
            <span id="videoFpsStatus" class="overlay-chip">帧率：--</span>
          </div>
        </div>
        <div id="videoEmpty" class="empty panel-meta">等待视频帧。</div>
        <div id="videoMeta" class="kv panel-meta"></div>
      </section>

      <section id="services" class="panel">
        <div class="card-head">
          <h2>服务矩阵</h2>
          <span id="servicesStatus" class="pill missing">missing</span>
        </div>
        <div id="serviceMatrix" class="service-list"></div>
      </section>

      <section id="logs-section" class="panel">
        <div class="card-head">
          <h2>运行日志尾部</h2>
          <span id="logsStatus" class="pill missing">missing</span>
        </div>
        <div class="toolbar">
          <select id="logLevelFilter" class="field" aria-label="日志等级过滤">
            <option value="">ALL</option>
            <option value="debug">DEBUG</option>
            <option value="info">INFO</option>
            <option value="warn">WARN</option>
            <option value="warning">WARNING</option>
            <option value="error">ERROR</option>
            <option value="critical">CRITICAL</option>
          </select>
          <input id="logSearch" class="field" type="search" placeholder="搜索日志、节点或关键词" aria-label="搜索日志">
        </div>
        <div id="logsMeta" class="kv"></div>
        <div id="logs" class="logs panel-meta"></div>
      </section>
    </div>
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
        navPath: "#38bdf8",
      };
      const overviewNames = ["health", "state", "localization", "map", "navigation", "exploration", "runtime_governance", "pointcloud_map", "pointcloud", "video", "logs"];
      const overviewLabels = { health: "健康", state: "状态", localization: "定位", map: "地图", navigation: "导航", exploration: "探索", runtime_governance: "RTAB-Map治理", pointcloud_map: "3D点云地图", pointcloud: "扫描点云", video: "视频", logs: "日志" };
      const state = {
        mapGrid: null,
        mapViewport: null,
        mapZoom: 1,
        mapStream: null,
        pose: null,
        trajectory: [],
        frontiers: [],
        goalPose: null,
        navigationPath: [],
        logs: [],
        logCursor: 0,
        streamSocket: null,
        streamReconnectTimer: null,
        latestStreamCursor: 0,
        latestStreamFrames: {},
        videoFrames: 0,
        latestOverview: null,
        latestHealth: null,
        latestTasks: null,
        logFilter: "",
        logSearch: "",
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
        mapZoomValue: document.getElementById("mapZoomValue"),
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
      function setText(el, value) {
        if (!el) return;
        el.textContent = safeText(value);
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
        setText(els.connectionText, "online");
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
          ["pointcloud_map", `${safeText(pointcloudMap.status)} frame=${safeText(pointcloudMap.frame_id)} rev=${safeText(pointcloudMap.revision)}`],
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
          const taskState = String(task.state || task.status || "missing").toLowerCase();
          pill.className = `pill ${taskState === "failed" || taskState === "timeout" ? "error" : taskState === "running" ? "available" : "stale"}`;
          pill.textContent = taskState;
          row.append(left, pill);
          els.taskSummary.appendChild(row);
        }
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
      function originPosition(grid) { return grid && grid.origin && grid.origin.position ? grid.origin.position : { x: 0, y: 0 }; }
      function originYaw(grid) { return yawFromPose(grid && grid.origin) || 0; }
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
      function mapCorners(grid) {
        if (!grid || !grid.width || !grid.height) return [];
        const origin = originPosition(grid);
        const yaw = originYaw(grid);
        const cos = Math.cos(yaw);
        const sin = Math.sin(yaw);
        const resolution = Number(grid.resolution_m) || 1;
        const width = Number(grid.width) * resolution;
        const height = Number(grid.height) * resolution;
        return [[0, 0], [width, 0], [width, height], [0, height]].map(([x, y]) => ({
          x: Number(origin.x || 0) + x * cos - y * sin,
          y: Number(origin.y || 0) + x * sin + y * cos,
        }));
      }
      function cameraAnchor() {
        const posePoint = pointFromPose(state.pose);
        if (posePoint) return { point: posePoint, yaw: yawFromPose(state.pose) || 0, locked: true };
        const corners = mapCorners(state.mapGrid);
        if (!corners.length) return { point: { x: 0, y: 0 }, yaw: 0, locked: false };
        const point = corners.reduce((acc, item) => ({ x: acc.x + item.x / corners.length, y: acc.y + item.y / corners.length }), { x: 0, y: 0 });
        return { point, yaw: 0, locked: false };
      }
      function basePixelsPerMeter(anchor) {
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
        return basePixelsPerMeter(anchor) * state.mapZoom;
      }
      function applyCameraTransform(ctx, anchor, scale) {
        const yaw = anchor.yaw || 0;
        const sin = Math.sin(yaw);
        const cos = Math.cos(yaw);
        const centerX = els.mapCanvas.width / 2;
        const centerY = els.mapCanvas.height / 2;
        const a = scale * sin;
        const b = -scale * cos;
        const c = -scale * cos;
        const d = -scale * sin;
        const e = centerX - a * anchor.point.x - c * anchor.point.y;
        const f = centerY - b * anchor.point.x - d * anchor.point.y;
        ctx.setTransform(a, b, c, d, e, f);
      }
      function mapToCanvas(point) {
        if (!point) return null;
        const anchor = cameraAnchor();
        const scale = cameraScale(anchor);
        const yaw = anchor.yaw || 0;
        const dx = Number(point.x) - anchor.point.x;
        const dy = Number(point.y) - anchor.point.y;
        return {
          x: els.mapCanvas.width / 2 + (Math.sin(yaw) * dx - Math.cos(yaw) * dy) * scale,
          y: els.mapCanvas.height / 2 - (Math.cos(yaw) * dx + Math.sin(yaw) * dy) * scale,
        };
      }
      function setWorldLineStyle(ctx, scale, color, pixels) {
        ctx.strokeStyle = color;
        ctx.lineWidth = Math.max(1 / scale, pixels / scale);
        ctx.lineCap = "round";
        ctx.lineJoin = "round";
      }
      function drawMap() {
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
        if (els.mapZoomValue) els.mapZoomValue.textContent = `${Math.round(state.mapZoom * 100)}%`;
        if (!grid || !Array.isArray(grid.data) || !grid.width || !grid.height) return;
        const width = Number(grid.width);
        const height = Number(grid.height);
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
        ctx.save();
        applyCameraTransform(ctx, anchor, scale);
        const origin = originPosition(grid);
        const resolution = Number(grid.resolution_m) || 1;
        ctx.translate(Number(origin.x || 0), Number(origin.y || 0));
        ctx.rotate(originYaw(grid));
        ctx.scale(resolution, -resolution);
        ctx.imageSmoothingEnabled = false;
        ctx.drawImage(offscreen, 0, -height, width, height);
        ctx.restore();
        drawTrajectory(ctx, anchor, scale);
        drawNavigationPath(ctx, anchor, scale);
        drawGoal(ctx, anchor, scale);
        drawFrontiers(ctx, anchor, scale);
        drawRobot(ctx, anchor.locked);
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
        const point = pointFromPose(state.goalPose);
        if (!point) return;
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
        ctx.beginPath();
        ctx.moveTo(point.x - radius * 1.45, point.y);
        ctx.lineTo(point.x + radius * 1.45, point.y);
        ctx.moveTo(point.x, point.y - radius * 1.45);
        ctx.lineTo(point.x, point.y + radius * 1.45);
        ctx.stroke();
        ctx.restore();
      }
      function drawRobot(ctx, locked) {
        if (!locked) return;
        ctx.save();
        ctx.setTransform(1, 0, 0, 1, 0, 0);
        ctx.translate(els.mapCanvas.width / 2, els.mapCanvas.height / 2);
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
        const status = statusOf(envelope);
        const data = envelope.data || {};
        const grid = data.occupancy_grid;
        const diagnostics = data.diagnostics || {};
        const streamBridge = diagnostics.stream_bridge || {};
        state.mapStream = { diagnostics, streamBridge, revision: envelope.revision, sequence: envelope.sequence, timestamp: envelope.source_timestamp };
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
          ["revision", envelope.revision || diagnostics.content_revision],
          ["sequence", envelope.sequence || diagnostics.message_count],
          ["streaming", mapStreamLabel(status, diagnostics, streamBridge)],
          ["latency", fmt(envelope.latency_ms, " ms")],
          ["stale", envelope.stale === true ? "true" : "false"],
          ["前端接收", shortTime(envelope.__receivedAt)],
        ]);
        updateMapStreamChip(status, diagnostics, streamBridge, envelope.source_timestamp);
      }
      function mapStreamLabel(status, diagnostics, streamBridge) {
        const messageCount = diagnostics && (diagnostics.message_count || diagnostics.sequence);
        const revision = diagnostics && (diagnostics.content_revision || diagnostics.revision);
        const delta = diagnostics && diagnostics.message_count_delta;
        const bridgeStatus = streamBridge && (streamBridge.status || streamBridge.state || streamBridge.mode);
        if (!messageCount) return status === "available" ? "首帧已缓存" : "等待首帧";
        const live = status === "available" && (delta === undefined || delta === null || Number(delta) > 0 || diagnostics.live_mapping_ready === true);
        return `${live ? "实时流" : "离线资产/缓存"} rev=${safeText(revision)} seq=${safeText(messageCount)} bridge=${safeText(bridgeStatus)}`;
      }
      function updateMapStreamChip(status, diagnostics, streamBridge, timestamp) {
        if (!els.mapStreamStatus) return;
        els.mapStreamStatus.textContent = `${mapStreamLabel(status, diagnostics, streamBridge)} · ${shortTime(timestamp)}`;
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
        setPill(els.poseStatus, status);
        setKv(els.missionPoseMeta, [
          ["frame", envelope.frame_id || (state.pose && state.pose.frame_id)],
          ["position", p ? `x=${fmt(p.x)} y=${fmt(p.y)} z=${fmt(p.z)}` : "--"],
          ["yaw", yaw === null ? "--" : fmt(yaw, " rad")],
          ["trajectory", state.trajectory.length],
          ["latency", fmt(envelope.latency_ms, " ms")],
        ]);
        drawMap();
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
      function extractNavigationPath(context) {
        const metadata = context && context.metadata ? context.metadata : {};
        const stateObj = context && context.navigation_state ? context.navigation_state : {};
        const stateMetadata = stateObj && stateObj.metadata ? stateObj.metadata : {};
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
          stateMetadata.waypoints
        );
        return raw.map(pathPointFromValue).filter(Boolean);
      }
      function extractNavigationGoal(context) {
        const goal = context && context.current_goal ? context.current_goal : null;
        return goal && goal.target_pose ? goal.target_pose : null;
      }
      function summarizeContext(context, key) {
        const stateObj = context && context[key];
        if (!stateObj) return "--";
        return stateObj.status || stateObj.phase || stateObj.message || stateObj.state || "available";
      }
      function updateGovernance(envelope) {
        const data = envelope.runtime_governance || (envelope.data && envelope.data.runtime_governance) || envelope.data || {};
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
      }
      function updateNavigation(envelope) {
        const status = statusOf(envelope);
        const data = envelope.data || {};
        setPill(els.navigationStatus, status);
        const frontierCount = data.frontier_count;
        const waitingFor = data.waiting_for;
        const mappingDiag = data.mapping_chain_diagnostics || {};
        const liveMap = mappingDiag.live_map_provider || {};
        const cloudPub = mappingDiag.point_cloud_publisher || {};
        const cloudSource = mappingDiag.point_cloud_source || {};
        const frontiers = Array.isArray(data.frontiers) ? data.frontiers : null;
        state.frontiers = frontiers || [];
        state.goalPose = extractNavigationGoal(data.navigation_context);
        state.navigationPath = extractNavigationPath(data.navigation_context);
        setKv(els.navigationMeta, [
          ["navigation", summarizeContext(data.navigation_context, "navigation_state")],
          ["exploration", summarizeContext(data.exploration_context, "exploration_state")],
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
        els.frontierNotice.hidden = Boolean(frontiers && frontiers.length);
        els.frontierNotice.textContent = frontiers && frontiers.length ? "" : "当前探索上下文还没有 frontier 几何，请确认探索运行态已生成候选。";
        drawMap();
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
        setText(els.visionModelStatus, `本地模型：${status}`);
        setText(els.videoFpsStatus, `帧 ${state.videoFrames} · 延迟 ${fmt(envelope.latency_ms, " ms")}`);
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
          if (normalizedLevel && level !== normalizedLevel) return false;
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
      const STREAM_TOPICS = [
        "robot.state",
        "map.frame",
        "localization.pose",
        "localization.frame_tree",
        "localization.trajectory",
        "navigation.state",
        "exploration.state",
        "exploration.frontier",
        "runtime.governance",
        "pointcloud_map.meta",
        "pointcloud.frame",
        "video.frame",
        "logs.entry",
        "tasks.state"
      ];
      function envelopeFromFrame(frame, payloadOverride) {
        const receivedAt = new Date().toISOString();
        return {
          ok: frame.status !== "error" && frame.status !== "missing",
          type: frame.topic,
          source: frame.source,
          frame_id: frame.frame_id,
          source_timestamp: frame.timestamp,
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
        els.lastReceive.textContent = nowText();
        setText(els.connectionText, "streaming");
        if (frame.topic === "robot.state") {
          updateRobotStateFrame(frame);
        } else if (frame.topic === "map.frame") {
          updateMap(envelopeFromFrame(frame));
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
          updateVideo(envelopeFromFrame(frame));
        } else if (frame.topic === "logs.entry") {
          updateLogFrame(frame);
        } else if (frame.topic === "tasks.state") {
          updateTaskFrame(frame);
        }
        updateStreamOverview(frame);
      }
      function updateRobotStateFrame(frame) {
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
      }
      function updateFrontierFrame(frame) {
        const payload = frame.payload || {};
        const frontiers = Array.isArray(payload.latest_frontiers) ? payload.latest_frontiers : [];
        state.frontiers = frontiers;
        updateNavigation(envelopeFromFrame(frame, {
          navigation_context: state.latestStreamFrames["navigation.state"] && state.latestStreamFrames["navigation.state"].payload,
          exploration_context: state.latestStreamFrames["exploration.state"] && state.latestStreamFrames["exploration.state"].payload,
          frontiers,
          frontier_count: frontiers.length,
        }));
      }
      function updatePointcloudMapFrame(frame) {
        const summary = frame.payload || {};
        const envelope = envelopeFromFrame(frame, summary);
        const data = { pointcloud_map: { status: envelope.status, frame_id: envelope.frame_id, revision: envelope.revision, latency_ms: envelope.latency_ms, source_timestamp: envelope.source_timestamp, metadata: { source: envelope.source } } };
        updateSensorSummary({ ...(state.latestOverview || {}), ...data });
      }
      function updatePointcloudFrame(frame) {
        const payload = frame.payload || {};
        const envelope = envelopeFromFrame(frame, payload);
        const data = { pointcloud: { status: envelope.status, frame_id: envelope.frame_id, revision: envelope.revision, latency_ms: envelope.latency_ms, source_timestamp: envelope.source_timestamp, metadata: { source: envelope.source, point_count: payload.point_count || (frame.metadata && frame.metadata.point_count) } } };
        updateSensorSummary({ ...(state.latestOverview || {}), ...data });
      }
      function updateLogFrame(frame) {
        const payload = frame.payload || {};
        const entry = {
          sequence: frame.sequence ?? frame.cursor,
          timestamp: payload.timestamp || frame.timestamp,
          level: String(payload.severity || frame.status || "info").toLowerCase(),
          source: payload.source || frame.source,
          event_type: payload.event_type,
          message: payload.message || payload.event_type || "runtime.event",
        };
        updateLogs(envelopeFromFrame(frame, { entries: [entry], next_cursor: frame.cursor, source_status: "streaming", metrics: {} }));
      }
      function updateTaskFrame(frame) {
        const event = frame.payload || {};
        const payload = event.payload || {};
        const task = {
          task_id: event.task_id || payload.task_id,
          capability_name: payload.capability_name || (payload.request_context && payload.request_context.capability_name),
          state: payload.state || event.state || event.event_type,
          stage: event.message || event.event_type,
          progress: payload.progress,
        };
        updateTasks({ active_tasks: task.task_id ? [task] : [], history_tasks: [] });
      }
      function updateStreamOverview(frame) {
        const keyByTopic = {
          "robot.state": "state",
          "map.frame": "map",
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
        const overview = { ...(state.latestOverview || {}) };
        overview[key] = {
          status: frame.status || "available",
          source_timestamp: frame.timestamp,
          latency_ms: 0,
          stale: frame.status === "stale",
          frame_id: frame.frame_id,
          revision: frame.revision,
          metadata: { source: frame.source },
        };
        state.latestOverview = overview;
        updateOverview({ data: overview, __receivedAt: new Date().toISOString(), __frontendLatencyMs: 0 });
      }
      buildOverview();
      renderLogs();
      updateClock();
      window.setInterval(updateClock, 1000);
      function setMapZoom(nextZoom) {
        state.mapZoom = Math.min(8, Math.max(0.25, nextZoom));
        drawMap();
      }
      els.mapZoomIn.addEventListener("click", () => setMapZoom(state.mapZoom * 1.25));
      els.mapZoomOut.addEventListener("click", () => setMapZoom(state.mapZoom / 1.25));
      els.mapZoomReset.addEventListener("click", () => setMapZoom(1));
      els.mapCanvas.addEventListener("wheel", (event) => {
        event.preventDefault();
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
        "content_hash": map_stream.get("content_hash") or _metric_value(status, "occupancy_metrics", "content_hash"),
        "known_cell_delta": map_stream.get("known_cell_delta") if map_stream.get("known_cell_delta") is not None else _metric_value(status, "occupancy_metrics", "known_cell_delta"),
        "known_cell_count": map_stream.get("known_cell_count") if map_stream.get("known_cell_count") is not None else _metric_value(status, "occupancy_metrics", "known_cell_count"),
        "costmap_delta": map_stream.get("costmap_delta") if map_stream.get("costmap_delta") is not None else _metric_value(status, "cost_map_metrics", "costmap_delta"),
        "costmap_revision": costmap_revision,
        "costmap_hash": map_stream.get("costmap_hash") or _metric_value(status, "cost_map_metrics", "content_hash"),
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
        profile.get("route") or "rtabmap",
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
        "route": profile.get("route") or "rtabmap",
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
