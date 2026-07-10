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


# Standalone debug-panel front-end (single source of truth; preview == production).
_DEBUG_PANEL_HTML_PATH = Path(__file__).resolve().parent / "static" / "debug_panel.html"

_LOG_LINE_RE = re.compile(r"^(?P<timestamp>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}(?:,\d{1,6})?) \[(?P<level>[A-Z]+)\] \[(?P<source>[^\]]+)\] (?P<message>.*)$")
_LOG_EXTRA_KEY_RE = re.compile(r"(?:(?<=^)|(?<=\s))(?P<key>[A-Za-z_][A-Za-z0-9_.]*)=")
_STRUCTURED_LOG_KEYS = frozenset(
    {
        "event",
        "request_id",
        "task_id",
        "capability_name",
        "method",
        "path",
        "status_code",
        "duration_ms",
        "error_category",
        "category",
        "runtime_event_type",
        "runtime_message",
        "payload_preview",
        "metadata_preview",
        "source",
        "cursor",
        "subject_id",
        "severity",
        "suppressed_count",
        "rate_limit_window_sec",
        "request_id_generated",
        "request_id_replaced",
    }
)

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
        access_manager.authenticate_local_debug_page(request)
        # 个人机器狗：网关不鉴权，前端无需注入访问令牌（保留空串占位，沿用现有 fetch 逻辑）。
        return HTMLResponse(_render_debug_page(""))

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
        """Debug 面板 2D Goal：RViz SetGoal 直通，发布 /goal_pose → bt_navigator。"""

        access_manager.authenticate_http(request)
        body = await request.json()
        arguments = dict(body or {})
        try:
            result = await run_in_threadpool(
                lambda: runtime.dispatch_debug_navigation_goal(arguments, requested_by="debug_panel"),
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
        """Debug 面板 RViz 2D Pose Estimate 直通：发布 /initialpose → RTAB-Map 重定位。"""

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
    """Serve the debug panel from the standalone static HTML file.

    The page markup/styles/script live in ``static/debug_panel.html`` so the
    file is the single source of truth: open it directly in a browser to
    preview exactly what production serves. The only server-side step is
    injecting the access token into the quoted ``"__DEBUG_ACCESS_TOKEN__"``
    placeholder (kept quoted so the raw file is valid JS when previewed).
    """
    debug_access_token = json.dumps(access_token)
    html = _DEBUG_PANEL_HTML_PATH.read_text(encoding="utf-8")
    return html.replace('"__DEBUG_ACCESS_TOKEN__"', debug_access_token)


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
    # R-N14 v4 Phase 2:metadata key 读新 key + fallback 老 key（LEGACY_VERSIONJSON_VALUES 计数由平台 runtime 接管）。
    map_stream = (live_mapping.get("mapping_readiness") or live_mapping.get("rtabmap_readiness") or {}).get("map_stream") if isinstance((live_mapping.get("mapping_readiness") or live_mapping.get("rtabmap_readiness") or {}), dict) else {}
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
    if configured.exists() and configured.is_file():
        return configured
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
        message, extra = _parse_log_message_extra(match.group("message"))
        return DebugLogEntry(
            sequence=sequence,
            timestamp=timestamp,
            level=match.group("level").lower(),
            source=match.group("source"),
            message=message,
            event_type=_optional_str(extra.get("event")),
            runtime_event_type=_optional_str(extra.get("runtime_event_type")),
            runtime_message=_optional_str(extra.get("runtime_message")),
            display_message=_optional_str(extra.get("runtime_message")) or message,
            request_id=_optional_str(extra.get("request_id")),
            task_id=_optional_str(extra.get("task_id")),
            capability_name=_optional_str(extra.get("capability_name")),
            duration_ms=_optional_float(extra.get("duration_ms")),
            error_category=_optional_str(extra.get("error_category")),
            payload_preview=extra.get("payload_preview"),
            metadata_preview=extra.get("metadata_preview"),
            details_compact={key: value for key, value in extra.items() if value is not None and value != "-"},
            suppressed_count=_optional_int(extra.get("suppressed_count")),
            payload=extra,
        )
    return DebugLogEntry(sequence=sequence, timestamp=utc_now(), level="info", source="log_file", message=line)


def _parse_log_message_extra(message: str) -> Tuple[str, Dict[str, Any]]:
    """Conservatively parse ContextAwareFormatter key=value extras from a log message."""

    for match in _LOG_EXTRA_KEY_RE.finditer(message):
        key = match.group("key")
        if key not in _STRUCTURED_LOG_KEYS:
            continue
        prefix = message[: match.start()].rstrip()
        suffix = message[match.start() :].strip()
        parsed = _parse_log_extra_tokens(suffix)
        if parsed:
            return prefix or message, parsed
    return message, {}


def _parse_log_extra_tokens(text: str) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    index = 0
    length = len(text)
    while index < length:
        while index < length and text[index].isspace():
            index += 1
        if index >= length:
            break
        key_match = re.match(r"([A-Za-z_][A-Za-z0-9_.]*)=", text[index:])
        if not key_match:
            break
        key = key_match.group(1)
        index += len(key) + 1
        value_text, index = _consume_log_extra_value(text, index)
        result[key] = _decode_log_extra_value(value_text)
    return result


def _consume_log_extra_value(text: str, index: int) -> Tuple[str, int]:
    if index >= len(text):
        return "", index
    if text[index] == '"':
        start = index
        index += 1
        escaped = False
        while index < len(text):
            char = text[index]
            index += 1
            if escaped:
                escaped = False
                continue
            if char == "\\":
                escaped = True
                continue
            if char == '"':
                break
        return text[start:index], index
    start = index
    while index < len(text) and not text[index].isspace():
        index += 1
    return text[start:index], index


def _decode_log_extra_value(value_text: str) -> Any:
    text = str(value_text or "")
    if not text:
        return ""
    if text.startswith('"') and text.endswith('"'):
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            return text.strip('"')
    if text in {"true", "false"}:
        return text == "true"
    if text and text[0] in "[{":
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            return text
    if re.fullmatch(r"-?\d+", text):
        try:
            return int(text)
        except ValueError:
            return text
    if re.fullmatch(r"-?(?:\d+\.\d*|\d*\.\d+)", text):
        try:
            return float(text)
        except ValueError:
            return text
    return text


def _optional_str(value: Any) -> Optional[str]:
    if value is None or value == "":
        return None
    return str(value)


def _optional_float(value: Any) -> Optional[float]:
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _optional_int(value: Any) -> Optional[int]:
    if value is None or value == "":
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


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
