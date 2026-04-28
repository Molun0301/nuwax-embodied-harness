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
    DebugPointCloudMapView,
    DebugPointCloudView,
    DebugSourceSummary,
    DebugVideoView,
)
from gateways.auth import GatewayAccessManager
from gateways.runtime import GatewayRuntime
from gateways.serialization import to_jsonable
from providers import ImageProvider
from services.geometry.pointcloud_map import (
    PointCloudMap,
    PointCloudMapCache,
    PointCloudMapError,
    UnsupportedPointCloudMapError,
)


_LOG_LINE_RE = re.compile(r"^(?P<timestamp>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}(?:,\d{1,6})?) \[(?P<level>[A-Z]+)\] \[(?P<source>[^\]]+)\] (?P<message>.*)$")

_STALE_THRESHOLDS_SEC = {
    "health": 3.0,
    "state": 3.0,
    "map": 2.0,
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

_POINTCLOUD_MAP_CACHE = PointCloudMapCache()


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
        runtime_governance = runtime.get_runtime_governance_summary()
        state_snapshot = runtime.robot_state_service.get_latest_snapshot()
        localization_snapshot = runtime.localization_service.get_latest_snapshot()
        map_snapshot = runtime.mapping_service.get_latest_snapshot()
        navigation_context = runtime.navigation_service.get_latest_navigation_context()
        exploration_context = runtime.navigation_service.get_latest_exploration_context()
        pointcloud_map_info = await run_in_threadpool(lambda: _get_pointcloud_map_summary(runtime))
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
            runtime_governance=_summary_from_payload(
                name="runtime_governance",
                source="gateway_runtime",
                timestamp=_timestamp_from_value(runtime_governance),
                available=True,
                message=_runtime_governance_message(runtime_governance),
                metadata=_runtime_governance_metadata(runtime_governance),
            ),
            pointcloud_map=_summary_from_payload(
                name="pointcloud_map",
                source=str(pointcloud_map_info.get("source", "pointcloud_map")),
                timestamp=pointcloud_map_info.get("timestamp"),
                frame_id=pointcloud_map_info.get("frame_id"),
                revision=pointcloud_map_info.get("revision"),
                available=bool(pointcloud_map_info.get("available")),
                message=str(pointcloud_map_info.get("message") or ""),
                metadata=dict(pointcloud_map_info.get("metadata") or {}),
            ),
            pointcloud=_summary_from_payload(
                name="pointcloud",
                source=_pointcloud_source(pointcloud),
                timestamp=_timestamp_from_value(pointcloud),
                frame_id=getattr(pointcloud, "frame_id", None),
                sequence=_sequence_from_metadata(pointcloud),
                available=pointcloud is not None,
                metadata={
                    **_pointcloud_metrics(pointcloud),
                    "diagnostic_role": "side_channel",
                    "not_pointcloud_map": True,
                    "map_source": "pointcloud_map",
                },
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
                _timestamp_from_value(runtime_governance),
                pointcloud_map_info.get("timestamp"),
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

    @router.get("/api/debug/pointcloud-map/latest/meta")
    async def debug_pointcloud_map_meta(request: Request) -> Dict[str, Any]:
        access_manager.authenticate_http(request)
        started_at = time.perf_counter()
        descriptor = _resolve_pointcloud_map_descriptor(runtime)
        pcd_path = str(descriptor.get("pcd_path") or "").strip()
        if not pcd_path:
            return _missing_envelope(
                payload_type="pointcloud_map.meta",
                source="pointcloud_map",
                status=DebugDataStatus.MISSING,
                code="source_unavailable",
                message="未找到当前激活地图版本的 PCD 点云地图资产；不会回退到最新扫描帧。",
                data=_missing_pointcloud_map_view(descriptor, started_at),
            )
        try:
            pointcloud_map = await run_in_threadpool(
                lambda: _POINTCLOUD_MAP_CACHE.load_pcd(
                    Path(pcd_path),
                    frame_id=str(descriptor.get("frame_id") or "map"),
                    source_kind=str(descriptor.get("source_kind") or "map_version_pcd"),
                )
            )
        except UnsupportedPointCloudMapError as exc:
            return _pointcloud_map_error_envelope(descriptor, str(exc), started_at, code="unsupported_pcd_format")
        except PointCloudMapError as exc:
            return _pointcloud_map_error_envelope(descriptor, str(exc), started_at, code="pcd_parse_failed")
        view = _pointcloud_map_view(pointcloud_map, descriptor, started_at)
        return _envelope(
            payload_type="pointcloud_map.meta",
            source=str(descriptor.get("source") or "pointcloud_map"),
            data=view,
            source_timestamp=utc_now(),
            frame_id=pointcloud_map.frame_id,
            revision=pointcloud_map.revision,
            status=DebugDataStatus.AVAILABLE,
        )

    @router.get("/api/debug/pointcloud-map/latest.bin")
    async def debug_pointcloud_map_binary(
        request: Request,
        lod: str = "preview",
        revision: Optional[str] = None,
    ) -> Response:
        started_at = time.perf_counter()
        access_manager.authenticate_http(request)
        descriptor = _resolve_pointcloud_map_descriptor(runtime)
        pcd_path = str(descriptor.get("pcd_path") or "").strip()
        if not pcd_path:
            return JSONResponse(
                status_code=404,
                content=_missing_envelope(
                    payload_type="pointcloud_map.bin",
                    source="pointcloud_map",
                    status=DebugDataStatus.MISSING,
                    code="source_unavailable",
                    message="未找到 PCD 点云地图资产；不会回退到最新扫描帧。",
                    data=_missing_pointcloud_map_view(descriptor, started_at),
                ),
            )
        try:
            pointcloud_map = await run_in_threadpool(
                lambda: _POINTCLOUD_MAP_CACHE.load_pcd(
                    Path(pcd_path),
                    frame_id=str(descriptor.get("frame_id") or "map"),
                    source_kind=str(descriptor.get("source_kind") or "map_version_pcd"),
                )
            )
        except UnsupportedPointCloudMapError as exc:
            return JSONResponse(
                status_code=422,
                content=_pointcloud_map_error_envelope(
                    descriptor,
                    str(exc),
                    started_at,
                    code="unsupported_pcd_format",
                ),
            )
        except PointCloudMapError as exc:
            return JSONResponse(
                status_code=422,
                content=_pointcloud_map_error_envelope(
                    descriptor,
                    str(exc),
                    started_at,
                    code="pcd_parse_failed",
                ),
            )
        if revision and revision != pointcloud_map.revision:
            return JSONResponse(
                status_code=409,
                content=_envelope(
                    payload_type="pointcloud_map.bin",
                    source=str(descriptor.get("source") or "pointcloud_map"),
                    status=DebugDataStatus.STALE,
                    frame_id=pointcloud_map.frame_id,
                    revision=pointcloud_map.revision,
                    error=DebugError(
                        code="revision_changed",
                        message="点云地图 revision 已变化，请先重新读取 meta。",
                        details={"requested_revision": revision, "current_revision": pointcloud_map.revision},
                    ),
                    data=_pointcloud_map_view(pointcloud_map, descriptor, started_at),
                ),
            )
        normalized_lod = str(lod or "preview").strip().lower() or "preview"
        points = pointcloud_map.lod_points(normalized_lod)
        payload = pointcloud_map.binary_xyz(normalized_lod)
        headers = {
            "X-PointCloud-Revision": pointcloud_map.revision,
            "X-PointCloud-Frame-Id": pointcloud_map.frame_id,
            "X-PointCloud-Point-Count": str(int(points.shape[0])),
            "X-PointCloud-Lod": normalized_lod,
            "X-PointCloud-Format": "float32-xyz",
            "X-PointCloud-Source-Kind": pointcloud_map.source_kind,
            "X-PointCloud-Payload-Bytes": str(len(payload)),
            "X-Server-Compute-Ms": "%.3f" % ((time.perf_counter() - started_at) * 1000.0),
            "Cache-Control": "no-store",
        }
        return Response(content=payload, media_type="application/octet-stream", headers=headers)

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
                    source="go2_dds_scan_diagnostic",
                    status=DebugDataStatus.MISSING,
                    metadata={"diagnostic_role": "side_channel", "not_pointcloud_map": True},
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
            metadata={
                **_pointcloud_metadata(pointcloud),
                "diagnostic_role": "side_channel",
                "not_pointcloud_map": True,
                "map_source_endpoint": "/api/debug/pointcloud-map/latest/meta",
            },
            point_count=int(getattr(pointcloud, "point_count", 0) or len(points)),
            fields=to_jsonable(getattr(pointcloud, "fields", []) or []),
            points=points,
            format=_pointcloud_format(pointcloud),
            payload_bytes=_estimate_pointcloud_payload_bytes(pointcloud),
            artifact_ref=getattr(pointcloud, "artifact_id", None),
            data_uri=getattr(pointcloud, "data_uri", None),
            protection=_pointcloud_protection(runtime),
            metrics={
                **_pointcloud_metrics(pointcloud),
                "inline_points": len(points),
                "diagnostic_role": "side_channel",
                "not_pointcloud_map": True,
            },
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
        started_at = time.perf_counter()
        access_manager.authenticate_http(request)
        auth_ms = (time.perf_counter() - started_at) * 1000.0
        safe_limit = _clamp_int(limit, default=200, minimum=1, maximum=1000)
        normalized_level = str(level or "").strip().lower()
        normalized_source = str(source or "").strip()
        resolve_started = time.perf_counter()
        log_path = _resolve_debug_log_path(runtime)
        path_resolve_ms = (time.perf_counter() - resolve_started) * 1000.0
        base_metrics = {
            "auth_ms": auth_ms,
            "path_resolve_ms": path_resolve_ms,
            "requested_cursor": max(0, int(cursor or 0)),
            "requested_limit": safe_limit,
        }
        if log_path is None:
            return _missing_envelope(
                payload_type="logs.tail",
                source="configured_log_file",
                status=DebugDataStatus.MISSING,
                code="source_unavailable",
                message="未找到配置日志文件。",
                data=DebugLogsTailView(
                    source_status=DebugDataStatus.MISSING,
                    source_message="未找到配置日志文件。",
                    metrics={**base_metrics, "server_compute_ms": (time.perf_counter() - started_at) * 1000.0},
                ),
            )
        entries, next_cursor, _file_size, tail_metrics = await run_in_threadpool(
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
            metrics={
                **base_metrics,
                **tail_metrics,
                "returned_entries": len(entries),
                "server_compute_ms": (time.perf_counter() - started_at) * 1000.0,
            },
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
    :root { color-scheme: dark; --bg: #0d0f0e; --surface: #121614; --panel: #171c1a; --panel-soft: #1d2421; --panel-strong: #222b27; --border: #34413b; --border-soft: #26302c; --text: #eef3ed; --muted: #9aa79f; --good: #6fda8c; --warn: #e7b44a; --bad: #ef6b72; --info: #62c8c4; --accent: #8ab4f8; --map-line: #8ab4f8; --unknown: #4d5650; --free: #dfe6dc; --occupied: #141716; --frontier: #d783ff; }
    * { box-sizing: border-box; }
    html { scroll-behavior: smooth; }
    body { margin: 0; padding: 16px; background: var(--bg); color: var(--text); font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; letter-spacing: 0; }
    .app-shell { max-width: 1760px; margin: 0 auto; }
    .topbar { display: grid; grid-template-columns: minmax(260px, 0.55fr) minmax(520px, 1.45fr); gap: 14px; align-items: stretch; margin-bottom: 14px; }
    .brand-block, .global-status { background: rgba(18, 22, 20, 0.92); border: 1px solid var(--border); border-radius: 8px; padding: 14px; box-shadow: 0 14px 34px rgba(0, 0, 0, 0.26); }
    .brand-row { display: flex; align-items: baseline; gap: 10px; flex-wrap: wrap; }
    h1, h2, h3 { margin: 0; }
    h1 { font-size: 24px; font-weight: 720; }
    h2 { font-size: 16px; font-weight: 650; }
    h3 { font-size: 13px; color: #d7d2c8; font-weight: 620; }
    code { color: #9bd7d3; word-break: break-all; }
    canvas { display: block; width: 100%; border-radius: 6px; background: #0b0b0a; border: 1px solid #2d2e29; }
    .muted { color: var(--muted); }
    .small { font-size: 12px; }
    .eyebrow { color: var(--info); font-size: 12px; font-weight: 650; text-transform: uppercase; }
    .readonly { color: #b6ede8; }
    .grid { display: grid; gap: 12px; }
    .mission-layout { display: grid; grid-template-columns: 176px minmax(0, 1fr); gap: 14px; align-items: start; }
    .side-nav { position: sticky; top: 16px; display: grid; gap: 6px; padding: 10px; background: rgba(18, 22, 20, 0.88); border: 1px solid var(--border); border-radius: 8px; }
    .side-nav a { color: #cbd8d1; text-decoration: none; border: 1px solid transparent; border-radius: 6px; padding: 8px 9px; font-size: 13px; }
    .side-nav a:hover { border-color: var(--border); background: var(--panel-soft); color: var(--text); }
    .mission-content { display: grid; gap: 12px; min-width: 0; }
    .section-band { scroll-margin-top: 16px; display: grid; gap: 12px; }
    .section-title { display: flex; justify-content: space-between; align-items: center; gap: 10px; padding: 0 2px; }
    .overview-grid { grid-template-columns: repeat(auto-fit, minmax(148px, 1fr)); }
    .mission-grid { grid-template-columns: minmax(260px, 0.8fr) minmax(260px, 0.8fr) minmax(320px, 1fr); }
    .workspace { grid-template-columns: minmax(660px, 1.25fr) minmax(430px, 0.75fr); align-items: start; }
    .right-rail { grid-template-columns: 1fr; }
    .card, .panel { background: rgba(23, 28, 26, 0.94); border: 1px solid var(--border); border-radius: 8px; padding: 12px; box-shadow: 0 10px 24px rgba(0, 0, 0, 0.22); }
    .panel.compact { padding: 10px; }
    .status-card { display: grid; gap: 7px; min-height: 112px; }
    .card-head { display: flex; align-items: center; justify-content: space-between; gap: 10px; margin-bottom: 10px; }
    .global-status { display: grid; grid-template-columns: repeat(6, minmax(108px, 1fr)); gap: 8px; align-items: stretch; }
    .metric-strip { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; justify-content: flex-end; }
    .metric { border: 1px solid var(--border); border-radius: 8px; padding: 7px 9px; background: var(--panel-soft); color: #d9d3c8; min-width: 112px; }
    .metric b { display: block; font-size: 14px; color: var(--text); }
    .metric span { display: block; color: var(--muted); font-size: 11px; }
    .pill { display: inline-flex; align-items: center; gap: 6px; border-radius: 999px; padding: 3px 8px; font-size: 12px; border: 1px solid var(--border); background: #101412; color: #d8d3ca; white-space: nowrap; }
    .pill::before { content: ""; width: 7px; height: 7px; border-radius: 999px; background: var(--muted); }
    .pill.available::before { background: var(--good); }
    .pill.stale::before { background: var(--warn); }
    .pill.missing::before, .pill.unsupported::before { background: var(--unknown); }
    .pill.error::before { background: var(--bad); }
    .estop { border-color: rgba(239, 107, 114, 0.55); color: #ffd5d7; background: rgba(95, 20, 25, 0.3); }
    .kv { display: grid; grid-template-columns: minmax(104px, 0.42fr) minmax(0, 1fr); gap: 5px 8px; font-size: 12px; }
    .kv div:nth-child(odd) { color: var(--muted); }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace; overflow-wrap: anywhere; }
    .empty, .error-text { color: #ffd0d0; background: rgba(116, 28, 33, 0.22); border: 1px solid rgba(244, 109, 117, 0.35); border-radius: 8px; padding: 9px; }
    .notice { color: #ffe1a8; background: rgba(115, 78, 21, 0.22); border: 1px solid rgba(244, 185, 66, 0.35); border-radius: 8px; padding: 9px; }
    .panel-meta { margin-top: 8px; }
    .canvas-wrap { position: relative; background: #0b0b0a; border-radius: 6px; }
    #mapCanvas { height: min(62vh, 680px); min-height: 430px; image-rendering: pixelated; }
    #pointCloudMapCanvas { height: 410px; }
    #pointCloudCanvas { height: 190px; }
    .video-stage { position: relative; border-radius: 6px; overflow: hidden; border: 1px solid #2d2e29; background: #080a09; }
    .video-frame { display: block; width: 100%; min-height: 220px; max-height: 340px; object-fit: contain; background: #0b0b0a; }
    .video-overlay { position: absolute; left: 10px; right: 10px; bottom: 10px; display: flex; justify-content: space-between; gap: 8px; pointer-events: none; }
    .overlay-chip { border: 1px solid rgba(98, 200, 196, 0.35); background: rgba(8, 12, 11, 0.72); border-radius: 6px; padding: 5px 7px; font-size: 12px; color: #d7eee9; }
    .legend { display: flex; flex-wrap: wrap; gap: 10px; margin-top: 8px; }
    .legend-item { display: inline-flex; align-items: center; gap: 6px; color: var(--muted); font-size: 12px; }
    .swatch { width: 12px; height: 12px; border: 1px solid #777268; border-radius: 3px; }
    .logs { display: grid; gap: 7px; max-height: 340px; overflow: auto; }
    .log-line { border: 1px solid #2d2e29; border-radius: 8px; padding: 8px; background: #121210; }
    .log-line .meta { color: #8fd8d3; font-size: 12px; margin-bottom: 4px; }
    .log-line.warn .meta, .log-line.warning .meta { color: #ffe1a8; }
    .log-line.error .meta, .log-line.critical .meta { color: #ffd0d0; }
    .toolbar { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
    .field { min-height: 30px; border: 1px solid var(--border); border-radius: 6px; background: #101412; color: var(--text); padding: 5px 8px; font: inherit; font-size: 12px; }
    .service-list, .event-list, .task-list { display: grid; gap: 7px; }
    .service-row, .task-row { display: grid; grid-template-columns: minmax(120px, 1fr) auto; gap: 8px; align-items: center; border: 1px solid var(--border-soft); border-radius: 7px; padding: 8px; background: #121715; }
    .service-row .meta, .task-row .meta { color: var(--muted); font-size: 12px; }
    .health-line { height: 6px; border-radius: 999px; background: linear-gradient(90deg, var(--good), var(--info)); opacity: 0.9; }
    .health-line.warn { background: linear-gradient(90deg, var(--warn), #f97316); }
    .health-line.bad { background: linear-gradient(90deg, var(--bad), #a61f2b); }
    .split-two { grid-template-columns: 1fr 1fr; }
    @media (max-width: 1280px) { .topbar, .mission-layout, .workspace, .mission-grid { grid-template-columns: 1fr; } .side-nav { position: static; grid-template-columns: repeat(auto-fit, minmax(120px, 1fr)); } .global-status { grid-template-columns: repeat(auto-fit, minmax(118px, 1fr)); } }
    @media (max-width: 720px) { body { padding: 10px; } .split-two { grid-template-columns: 1fr; } .metric-strip { justify-content: flex-start; } #mapCanvas, #pointCloudMapCanvas { min-height: 320px; } }
  </style>
</head>
<body>
  <div class="app-shell">
  <header class="topbar">
    <div class="brand-block">
      <div class="eyebrow">只读局域网监控</div>
      <div class="brand-row">
        <h1 id="robotName">nuwax_robot_bridge</h1>
        <span id="robotModel" class="pill missing">model: --</span>
      </div>
      <div class="muted small panel-meta">Robotics mission control dashboard · 只读观察面</div>
    </div>
    <div class="global-status">
      <div class="metric"><b id="connectionText">connecting</b><span>连接状态</span></div>
      <div class="metric"><b id="runtimeMode">--</b><span>运行模式</span></div>
      <div class="metric"><b id="batteryMetric">--</b><span>电池</span></div>
      <div class="metric"><b id="networkLatency" class="mono">--</b><span>HTTP 往返</span></div>
      <div class="metric"><b id="resourceMetric">--</b><span>CPU/GPU/RAM</span></div>
      <div class="metric"><b id="clockNow" class="mono">--</b><span>当前时间</span></div>
      <div class="metric"><b id="lastReceive" class="mono">--</b><span>前端接收</span></div>
      <div class="metric"><b>&lt;500 ms</b><span>轻量链路目标 P95</span></div>
      <span id="estopStatus" class="pill estop">Emergency Stop: read-only</span>
    </div>
  </header>

  <div class="mission-layout">
    <aside class="side-nav" aria-label="页面导航">
      <a href="#overview">Overview</a>
      <a href="#live-map">Live Map</a>
      <a href="#pointcloud-map">3D Map</a>
      <a href="#video-perception">Video</a>
      <a href="#navigation">Navigation</a>
      <a href="#services">Services</a>
      <a href="#logs-section">Logs</a>
      <a href="#runtime-settings">Settings</a>
    </aside>

    <div class="mission-content">
  <section id="overview" class="section-band">
    <div class="section-title">
      <h2>Overview</h2>
      <span id="missionStatus" class="pill missing">mission: warming</span>
    </div>
    <section id="overviewGrid" class="grid overview-grid" aria-label="顶部总览"></section>
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
        <h2>2D 地图、定位与轨迹</h2>
        <span id="mapStatus" class="pill missing">missing</span>
      </div>
      <div class="canvas-wrap"><canvas id="mapCanvas" width="960" height="620"></canvas></div>
      <div class="legend">
        <span class="legend-item"><span class="swatch" style="background: var(--unknown)"></span>unknown</span>
        <span class="legend-item"><span class="swatch" style="background: var(--free)"></span>free</span>
        <span class="legend-item"><span class="swatch" style="background: var(--occupied)"></span>occupied</span>
        <span class="legend-item"><span class="swatch" style="background: var(--map-line)"></span>轨迹</span>
        <span class="legend-item"><span class="swatch" style="background: #f2a65a"></span>机器人</span>
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

    <div class="grid right-rail">
      <section id="pointcloud-map" class="panel">
        <div class="card-head">
          <h2>3D 点云地图</h2>
          <span id="pointcloudMapStatus" class="pill missing">missing</span>
        </div>
        <div class="canvas-wrap"><canvas id="pointCloudMapCanvas" width="760" height="360"></canvas></div>
        <div id="pointcloudMapEmpty" class="empty panel-meta">等待 RTAB-Map/PCD 累计点云地图。</div>
        <div id="pointcloudMapMeta" class="kv panel-meta"></div>
      </section>

      <section id="scan-diagnostics" class="panel">
        <div class="card-head">
          <h2>实时扫描帧诊断</h2>
          <span id="pointcloudStatus" class="pill missing">missing</span>
        </div>
        <div class="canvas-wrap"><canvas id="pointCloudCanvas" width="620" height="190"></canvas></div>
        <div id="pointcloudEmpty" class="empty panel-meta">等待最新扫描帧。</div>
        <div id="pointcloudMeta" class="kv panel-meta"></div>
      </section>

      <section id="navigation" class="panel">
        <div class="card-head">
          <h2>navigation / frontier</h2>
          <span id="navigationStatus" class="pill missing">missing</span>
        </div>
        <div id="navigationMeta" class="kv"></div>
        <div id="frontierNotice" class="notice panel-meta">等待 frontier 候选几何。</div>
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
          <h2>视频与感知</h2>
          <span id="videoStatus" class="pill missing">missing</span>
        </div>
        <div class="video-stage">
          <img id="videoFrame" class="video-frame" alt="实时视频帧" hidden>
          <div class="video-overlay">
            <span id="visionModelStatus" class="overlay-chip">local model: --</span>
            <span id="videoFpsStatus" class="overlay-chip">fps: --</span>
          </div>
        </div>
        <div id="videoEmpty" class="empty panel-meta">等待视频帧。</div>
        <div id="videoMeta" class="kv panel-meta"></div>
      </section>

      <section id="services" class="panel">
        <div class="card-head">
          <h2>服务状态</h2>
          <span id="servicesStatus" class="pill missing">missing</span>
        </div>
        <div id="serviceMatrix" class="service-list"></div>
      </section>

      <section id="logs-section" class="panel">
        <div class="card-head">
          <h2>真实日志 tail</h2>
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
          <input id="logSearch" class="field" type="search" placeholder="搜索日志" aria-label="搜索日志">
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
      const FAST_POLL_MS = 500;
      const SENSOR_POLL_MS = 1000;
      const SLOW_POLL_MS = 1500;
      const DEBUG_ACCESS_TOKEN = __DEBUG_ACCESS_TOKEN__;
      const urlAccessToken = new URLSearchParams(window.location.search).get("access_token") || "";
      const apiAccessToken = urlAccessToken || DEBUG_ACCESS_TOKEN;
      const overviewNames = ["health", "state", "localization", "map", "navigation", "exploration", "runtime_governance", "pointcloud_map", "pointcloud", "video", "logs"];
      const overviewLabels = { health: "health", state: "state", localization: "localization", map: "map", navigation: "navigation", exploration: "exploration", runtime_governance: "RTAB-Map治理", pointcloud_map: "点云地图", pointcloud: "扫描帧", video: "video", logs: "logs" };
      const state = {
        mapGrid: null,
        mapViewport: null,
        pose: null,
        trajectory: [],
        frontiers: [],
        logs: [],
        logCursor: 0,
        fastPolling: false,
        sensorPolling: false,
        slowPolling: false,
        videoFrames: 0,
        pointCloudFrames: 0,
        pointCloudMapRevision: "",
        pointCloudMapFrames: 0,
        pointCloudMapPointCount: 0,
        pointCloudMapFetchMs: null,
        pointCloudMapParseMs: null,
        pointCloudMapRenderMs: null,
        pointCloudMapBounds: null,
        pointCloudMapLod: "",
        pointCloudMapPoints: null,
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
        pointcloudMapCanvas: document.getElementById("pointCloudMapCanvas"),
        pointcloudMapStatus: document.getElementById("pointcloudMapStatus"),
        pointcloudMapEmpty: document.getElementById("pointcloudMapEmpty"),
        pointcloudMapMeta: document.getElementById("pointcloudMapMeta"),
        pointcloudCanvas: document.getElementById("pointCloudCanvas"),
        pointcloudStatus: document.getElementById("pointcloudStatus"),
        pointcloudEmpty: document.getElementById("pointcloudEmpty"),
        pointcloudMeta: document.getElementById("pointcloudMeta"),
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
      async function getArrayBuffer(path) {
        const started = performance.now();
        const requestPath = withAccessToken(path);
        const res = await fetch(requestPath, { cache: "no-store" });
        const receivedAt = new Date();
        if (!res.ok) throw new Error(`${path} HTTP ${res.status}`);
        const buffer = await res.arrayBuffer();
        return { buffer, receivedAt: receivedAt.toISOString(), frontendLatencyMs: performance.now() - started, headers: res.headers };
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
        state.latestOverview = data;
        setText(els.connectionText, "online");
        setText(els.networkLatency, fmt(envelope.__frontendLatencyMs, " ms"));
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
        const mission = worstStatus([data.health, data.state, data.localization, data.map, data.navigation, data.pointcloud_map, data.video, data.logs]);
        setPill(els.missionStatus, mission, "mission");
        updateSensorSummary(data);
        renderServiceMatrix(data);
      }
      function updateSensorSummary(data) {
        const map = data.map || {};
        const localization = data.localization || {};
        const cloudMap = data.pointcloud_map || {};
        const scan = data.pointcloud || {};
        const video = data.video || {};
        const sensorStatus = worstStatus([map, localization, cloudMap, scan, video]);
        setPill(els.sensorStatus, sensorStatus);
        setKv(els.sensorSummary, [
          ["2D_map", `${safeText(map.status)} latency=${fmt(map.latency_ms, " ms")}`],
          ["localization", `${safeText(localization.status)} frame=${safeText(localization.frame_id)}`],
          ["pointcloud_map", `${safeText(cloudMap.status)} rev=${safeText(cloudMap.revision)}`],
          ["scan_frame", `${safeText(scan.status)} frame=${safeText(scan.frame_id)}`],
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
          ["sensor_scan", data.pointcloud],
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
        setPill(els.robotModel, health.runtime_started ? "available" : "missing", `model: ${safeText(health.robot_model)}`);
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
        setPill(els.estopStatus, estopped ? "error" : "available", estopped ? "Emergency Stop: active" : "Emergency Stop: clear");
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
        } else if (kind === "pointcloud_map") {
          setPill(els.pointcloudMapStatus, "error");
          els.pointcloudMapEmpty.hidden = false;
          els.pointcloudMapEmpty.textContent = message;
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
      function normalizeBounds(bounds, points) {
        const rawMin = bounds && Array.isArray(bounds.min) ? bounds.min : null;
        const rawMax = bounds && Array.isArray(bounds.max) ? bounds.max : null;
        if (rawMin && rawMax && rawMin.length >= 3 && rawMax.length >= 3) {
          return {
            min: [Number(rawMin[0]), Number(rawMin[1]), Number(rawMin[2])],
            max: [Number(rawMax[0]), Number(rawMax[1]), Number(rawMax[2])],
          };
        }
        if (!points || points.length < 3) return null;
        const min = [Infinity, Infinity, Infinity];
        const max = [-Infinity, -Infinity, -Infinity];
        for (let index = 0; index + 2 < points.length; index += 3) {
          const x = points[index], y = points[index + 1], z = points[index + 2];
          if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
          min[0] = Math.min(min[0], x); min[1] = Math.min(min[1], y); min[2] = Math.min(min[2], z);
          max[0] = Math.max(max[0], x); max[1] = Math.max(max[1], y); max[2] = Math.max(max[2], z);
        }
        return Number.isFinite(min[0]) ? { min, max } : null;
      }
      function clearPointCloudMapCanvas(message) {
        const canvas = els.pointcloudMapCanvas;
        const ctx = canvas.getContext("2d");
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.fillStyle = "#080807";
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        ctx.strokeStyle = "#2d2e29";
        ctx.lineWidth = 1;
        for (let x = 24; x < canvas.width; x += 44) {
          ctx.beginPath();
          ctx.moveTo(x, 0);
          ctx.lineTo(x, canvas.height);
          ctx.stroke();
        }
        for (let y = 24; y < canvas.height; y += 44) {
          ctx.beginPath();
          ctx.moveTo(0, y);
          ctx.lineTo(canvas.width, y);
          ctx.stroke();
        }
        ctx.fillStyle = "#a7a29a";
        ctx.font = "14px system-ui";
        ctx.fillText(message || "等待累计点云地图。", 16, 32);
      }
      function drawPointCloudMap(points, bounds) {
        const canvas = els.pointcloudMapCanvas;
        const ctx = canvas.getContext("2d");
        const started = performance.now();
        if (!points || points.length < 3) {
          clearPointCloudMapCanvas("累计点云地图暂无可绘制点。");
          return performance.now() - started;
        }
        const normalizedBounds = normalizeBounds(bounds, points);
        if (!normalizedBounds) {
          clearPointCloudMapCanvas("累计点云地图 bounds 不可用。");
          return performance.now() - started;
        }
        const min = normalizedBounds.min;
        const max = normalizedBounds.max;
        const center = [(min[0] + max[0]) / 2, (min[1] + max[1]) / 2, (min[2] + max[2]) / 2];
        const span = [
          Math.max(0.01, max[0] - min[0]),
          Math.max(0.01, max[1] - min[1]),
          Math.max(0.01, max[2] - min[2]),
        ];
        const yaw = -Math.PI / 5;
        const cos = Math.cos(yaw);
        const sin = Math.sin(yaw);
        const projectedSpan = Math.max(span[0], span[1], span[2] * 1.8, 0.01);
        const scale = Math.min(canvas.width, canvas.height) * 0.78 / projectedSpan;
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        const gradient = ctx.createLinearGradient(0, 0, canvas.width, canvas.height);
        gradient.addColorStop(0, "#0b1113");
        gradient.addColorStop(1, "#0a0a09");
        ctx.fillStyle = gradient;
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        ctx.strokeStyle = "rgba(93, 199, 194, 0.12)";
        ctx.lineWidth = 1;
        for (let i = -5; i <= 5; i += 1) {
          const a = projectPoint([min[0], center[1] + i * span[1] / 10, min[2]], center, cos, sin, scale, canvas);
          const b = projectPoint([max[0], center[1] + i * span[1] / 10, min[2]], center, cos, sin, scale, canvas);
          const c = projectPoint([center[0] + i * span[0] / 10, min[1], min[2]], center, cos, sin, scale, canvas);
          const d = projectPoint([center[0] + i * span[0] / 10, max[1], min[2]], center, cos, sin, scale, canvas);
          ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.moveTo(c.x, c.y); ctx.lineTo(d.x, d.y); ctx.stroke();
        }
        const pointCount = Math.floor(points.length / 3);
        const stride = pointCount > 70000 ? Math.ceil(pointCount / 70000) : 1;
        for (let index = 0; index + 2 < points.length; index += 3 * stride) {
          const x = points[index], y = points[index + 1], z = points[index + 2];
          if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
          const p = projectPoint([x, y, z], center, cos, sin, scale, canvas);
          const zRatio = Math.max(0, Math.min(1, (z - min[2]) / span[2]));
          ctx.fillStyle = `hsl(${176 + zRatio * 88}, 72%, ${52 + zRatio * 18}%)`;
          ctx.fillRect(p.x, p.y, 1.45, 1.45);
        }
        drawPointCloudMapAxes(ctx, center, cos, sin, scale, canvas, span);
        return performance.now() - started;
      }
      function projectPoint(point, center, cos, sin, scale, canvas) {
        const x = point[0] - center[0];
        const y = point[1] - center[1];
        const z = point[2] - center[2];
        const rx = x * cos - y * sin;
        const ry = x * sin + y * cos;
        return {
          x: canvas.width / 2 + rx * scale,
          y: canvas.height / 2 + (ry * 0.45 - z * 0.85) * scale,
        };
      }
      function drawPointCloudMapAxes(ctx, center, cos, sin, scale, canvas, span) {
        const origin = projectPoint(center, center, cos, sin, scale, canvas);
        const axes = [
          { label: "X", color: "#f97316", end: [center[0] + span[0] * 0.18, center[1], center[2]] },
          { label: "Y", color: "#5dc7c2", end: [center[0], center[1] + span[1] * 0.18, center[2]] },
          { label: "Z", color: "#d783ff", end: [center[0], center[1], center[2] + span[2] * 0.28] },
        ];
        ctx.save();
        ctx.font = "12px system-ui";
        for (const axis of axes) {
          const end = projectPoint(axis.end, center, cos, sin, scale, canvas);
          ctx.strokeStyle = axis.color;
          ctx.fillStyle = axis.color;
          ctx.lineWidth = 2;
          ctx.beginPath();
          ctx.moveTo(origin.x, origin.y);
          ctx.lineTo(end.x, end.y);
          ctx.stroke();
          ctx.fillText(axis.label, end.x + 5, end.y + 4);
        }
        ctx.restore();
      }
      function selectPointCloudMapLod(data) {
        const lods = Array.isArray(data.lods) ? data.lods : [];
        return lods.find((item) => item.level === "preview") || lods[0] || null;
      }
      async function loadPointCloudMapBinary(data) {
        const lod = selectPointCloudMapLod(data);
        if (!lod || !lod.bin_url) return;
        const fetched = await getArrayBuffer(lod.bin_url);
        const parseStarted = performance.now();
        const points = new Float32Array(fetched.buffer);
        state.pointCloudMapParseMs = performance.now() - parseStarted;
        state.pointCloudMapFetchMs = fetched.frontendLatencyMs;
        state.pointCloudMapLod = lod.level || fetched.headers.get("X-PointCloud-Lod") || "preview";
        state.pointCloudMapPoints = points;
        state.pointCloudMapPointCount = Number(fetched.headers.get("X-PointCloud-Point-Count")) || Math.floor(points.length / 3);
        state.pointCloudMapBounds = normalizeBounds(data.bounds, points);
        state.pointCloudMapRenderMs = drawPointCloudMap(points, state.pointCloudMapBounds);
        state.pointCloudMapFrames += 1;
      }
      async function updatePointCloudMap(envelope) {
        const status = statusOf(envelope);
        const data = envelope.data || {};
        setPill(els.pointcloudMapStatus, status);
        if (status !== "available" || !data.revision) {
          state.pointCloudMapRevision = "";
          state.pointCloudMapPoints = null;
          clearPointCloudMapCanvas(errorReason(envelope, "累计点云地图不可用。"));
          els.pointcloudMapEmpty.hidden = false;
          els.pointcloudMapEmpty.textContent = errorReason(envelope, "累计点云地图不可用；不会回退到实时扫描帧。");
        } else {
          els.pointcloudMapEmpty.hidden = true;
          if (state.pointCloudMapRevision !== data.revision) {
            try {
              await loadPointCloudMapBinary(data);
              state.pointCloudMapRevision = data.revision;
            } catch (error) {
              showPanelError("pointcloud_map", error.message);
            }
          } else if (state.pointCloudMapPoints) {
            state.pointCloudMapRenderMs = drawPointCloudMap(state.pointCloudMapPoints, state.pointCloudMapBounds || data.bounds);
          }
        }
        const metrics = data.metrics || {};
        const metadata = data.metadata || {};
        setKv(els.pointcloudMapMeta, [
          ["map", data.map_name || "--"],
          ["version", data.map_version_id || "--"],
          ["revision", data.revision || envelope.revision],
          ["frame", envelope.frame_id || data.frame_id],
          ["source_kind", data.source_kind],
          ["point_count", data.point_count || state.pointCloudMapPointCount],
          ["lod", state.pointCloudMapLod || "--"],
          ["payload_bytes", data.payload_bytes],
          ["pcd_data", metadata.pcd_data],
          ["pcd_path", data.pcd_path],
          ["server_compute_ms", fmt(metrics.server_compute_ms, " ms")],
          ["frontend_fetch_ms", fmt(state.pointCloudMapFetchMs, " ms")],
          ["browser_parse_ms", fmt(state.pointCloudMapParseMs, " ms")],
          ["browser_render_ms", fmt(state.pointCloudMapRenderMs, " ms")],
          ["rendered_frames", state.pointCloudMapFrames],
          ["source_timestamp", shortTime(envelope.source_timestamp)],
          ["前端接收", shortTime(envelope.__receivedAt)],
        ]);
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
        const metadata = data.metadata || {};
        const protection = data.protection || {};
        setKv(els.pointcloudMeta, [
          ["frame", envelope.frame_id],
          ["point_count", data.point_count ?? points.length],
          ["inline_points", points.length],
          ["rendered_frames", state.pointCloudFrames],
          ["format", data.format],
          ["payload_bytes", data.payload_bytes || metrics.payload_bytes],
          ["scan_role", metrics.diagnostic_role || (metadata.diagnostic_role || "side_channel")],
          ["not_pointcloud_map", String(metrics.not_pointcloud_map === true || metadata.not_pointcloud_map === true)],
          ["map_source_endpoint", metadata.map_source_endpoint || "/api/debug/pointcloud-map/latest/meta"],
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
        setText(els.visionModelStatus, `local model: ${status}`);
        setText(els.videoFpsStatus, `frames ${state.videoFrames} · latency ${fmt(envelope.latency_ms, " ms")}`);
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
      async function tickFast() {
        if (state.fastPolling) return;
        state.fastPolling = true;
        try {
          const [overview, map, localization, navigation, frontier, governance, pointcloudMap] = await Promise.allSettled([
            getJson("/api/debug/overview"),
            getJson("/api/debug/map"),
            getJson("/api/debug/localization"),
            getJson("/api/debug/navigation"),
            getJson("/api/debug/exploration/frontier/latest"),
            getJson("/api/runtime/governance"),
            getJson("/api/debug/pointcloud-map/latest/meta"),
          ]);
          els.lastReceive.textContent = nowText();
          if (overview.status === "fulfilled") updateOverview(overview.value);
          else {
            setText(els.connectionText, "degraded");
            setText(els.networkLatency, "--");
          }
          if (map.status === "fulfilled") updateMap(map.value); else showPanelError("map", map.reason.message);
          if (localization.status === "fulfilled") updateLocalization(localization.value); else showPanelError("localization", localization.reason.message);
          if (navigation.status === "fulfilled") updateNavigation(navigation.value); else showPanelError("navigation", navigation.reason.message);
          if (frontier.status === "fulfilled" && statusOf(frontier.value) === "available") updateNavigation(frontier.value);
          if (governance.status === "fulfilled") updateGovernance(governance.value); else showPanelError("governance", governance.reason.message);
          if (pointcloudMap.status === "fulfilled") await updatePointCloudMap(pointcloudMap.value); else showPanelError("pointcloud_map", pointcloudMap.reason.message);
        } finally {
          state.fastPolling = false;
        }
      }
      async function tickSensor() {
        if (state.sensorPolling) return;
        state.sensorPolling = true;
        try {
          const [pointcloud, video] = await Promise.allSettled([
            getJson("/api/debug/pointcloud/latest"),
            getJson("/api/debug/video/latest"),
          ]);
          if (pointcloud.status === "fulfilled") updatePointCloud(pointcloud.value); else showPanelError("pointcloud", pointcloud.reason.message);
          if (video.status === "fulfilled") updateVideo(video.value); else showPanelError("video", video.reason.message);
        } finally {
          state.sensorPolling = false;
        }
      }
      async function tickSlow() {
        if (state.slowPolling) return;
        state.slowPolling = true;
        try {
          const [logs, health, tasks] = await Promise.allSettled([
            getJson(`/api/debug/logs/tail?cursor=${encodeURIComponent(state.logCursor)}&limit=200`),
            getJson("/api/health"),
            getJson("/api/tasks?history_limit=20"),
          ]);
          if (logs.status === "fulfilled") updateLogs(logs.value); else showPanelError("logs", logs.reason.message);
          if (health.status === "fulfilled") updateHealth(health.value);
          if (tasks.status === "fulfilled") updateTasks(tasks.value);
        } catch (error) {
          showPanelError("logs", error.message);
        } finally {
          state.slowPolling = false;
        }
      }
      buildOverview();
      renderLogs();
      updateClock();
      window.setInterval(updateClock, 1000);
      els.logLevelFilter.addEventListener("change", () => {
        state.logFilter = els.logLevelFilter.value;
        renderLogs();
      });
      els.logSearch.addEventListener("input", () => {
        state.logSearch = els.logSearch.value;
        renderLogs();
      });
      clearPointCloudMapCanvas("等待 RTAB-Map/PCD 累计点云地图。");
      tickFast();
      tickSensor();
      tickSlow();
      window.setInterval(tickFast, FAST_POLL_MS);
      window.setInterval(tickSensor, SENSOR_POLL_MS);
      window.setInterval(tickSlow, SLOW_POLL_MS);
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


def _get_pointcloud_map_summary(runtime: GatewayRuntime) -> Dict[str, Any]:
    descriptor = _resolve_pointcloud_map_descriptor(runtime)
    pcd_path = str(descriptor.get("pcd_path") or "").strip()
    if not pcd_path:
        return {
            "available": False,
            "source": "pointcloud_map",
            "frame_id": descriptor.get("frame_id"),
            "message": "未找到 PCD 点云地图资产。",
            "metadata": descriptor,
        }
    path = Path(pcd_path).expanduser()
    if not path.exists() or not path.is_file():
        return {
            "available": False,
            "source": str(descriptor.get("source") or "pointcloud_map"),
            "frame_id": descriptor.get("frame_id"),
            "message": "PCD 点云地图资产不存在：%s" % pcd_path,
            "metadata": descriptor,
        }
    stat = path.stat()
    return {
        "available": True,
        "source": str(descriptor.get("source") or "pointcloud_map"),
        "timestamp": utc_now(),
        "frame_id": descriptor.get("frame_id"),
        "revision": "pcd:%d:%d" % (int(stat.st_mtime_ns), int(stat.st_size)),
        "message": "pointcloud_map=%s" % path,
        "metadata": {
            **descriptor,
            "asset_mtime": datetime.fromtimestamp(stat.st_mtime, tz=timezone.utc),
            "asset_size_bytes": stat.st_size,
        },
    }


def _resolve_pointcloud_map_descriptor(runtime: GatewayRuntime) -> Dict[str, Any]:
    runtime_governance = runtime.get_runtime_governance_summary()
    governance_asset = dict(getattr(runtime_governance, "geometry_asset", {}) or {})
    governance_profile = dict(getattr(runtime_governance, "rtabmap_profile", {}) or {})
    snapshot = runtime.mapping_service.get_latest_snapshot()
    snapshot_metadata = dict(getattr(snapshot, "metadata", {}) or {}) if snapshot is not None else {}
    geometry_summary = snapshot_metadata.get("geometry_asset_summary")
    if not isinstance(geometry_summary, dict):
        geometry_summary = {}
    pcd_path = _first_non_empty(
        governance_asset.get("pcd_path"),
        snapshot_metadata.get("pcd_path"),
        geometry_summary.get("pcd_path"),
        _pcd_path_from_asset_refs(snapshot_metadata.get("geometry_assets")),
        _pcd_path_from_asset_refs(geometry_summary.get("assets")),
    )
    source_kind = "runtime_governance_pcd" if governance_asset.get("pcd_path") else "map_version_pcd" if pcd_path else "unavailable"
    geometry_runtime = getattr(runtime, "platform_geometry_runtime", None)
    geometry_config = getattr(geometry_runtime, "config", None)
    frames = getattr(geometry_config, "frames", None)
    rtabmap = getattr(geometry_config, "rtabmap", None)
    outputs = getattr(rtabmap, "outputs", None)
    return {
        "source": "rtabmap_pointcloud_map",
        "source_kind": source_kind if pcd_path else "unavailable",
        "map_name": _first_non_empty(snapshot_metadata.get("map_name"), geometry_summary.get("map_name")),
        "map_version_id": _first_non_empty(
            getattr(snapshot, "version_id", None),
            snapshot_metadata.get("platform_map_version_id"),
            geometry_summary.get("version_id"),
        ),
        "revision": _revision_from_map(snapshot),
        "frame_id": _first_non_empty(
            governance_asset.get("frame_id"),
            getattr(snapshot, "frame_id", None),
            snapshot_metadata.get("frame_id"),
            geometry_summary.get("frame_id"),
            getattr(frames, "map_frame", None),
            "map",
        ),
        "geometry_stack": _first_non_empty(
            governance_asset.get("geometry_stack"),
            snapshot_metadata.get("geometry_stack"),
            geometry_summary.get("geometry_stack"),
            getattr(geometry_config, "stack", None),
            "rtabmap_nav2",
        ),
        "rtabmap_db_path": _first_non_empty(
            governance_asset.get("rtabmap_db_path"),
            snapshot_metadata.get("rtabmap_db_path"),
            geometry_summary.get("rtabmap_db_path"),
        ),
        "pcd_path": str(pcd_path or ""),
        "quality_status": governance_asset.get("quality_status"),
        "asset_summary": governance_asset,
        "expected_pcd_candidates": [str(path) for path in _configured_pcd_candidates(runtime)],
        "rtabmap_profile": {
            "name": _first_non_empty(governance_profile.get("name"), getattr(rtabmap, "profile_name", None)),
            "enabled": governance_profile.get("enabled", getattr(rtabmap, "profile_enabled", None)),
            "pcd_root": getattr(rtabmap, "pcd_root", None),
            "pcd_asset_name": getattr(outputs, "pcd_asset_name", None),
        },
    }


def _pointcloud_map_view(
    pointcloud_map: PointCloudMap,
    descriptor: Dict[str, Any],
    started_at: float,
) -> DebugPointCloudMapView:
    payload_bytes = int(pointcloud_map.lod_points("preview").shape[0]) * 3 * 4
    lods = []
    for item in pointcloud_map.lods():
        level = str(item["level"])
        lods.append(
            {
                **item,
                "format": "float32-xyz",
                "payload_bytes": int(item["point_count"]) * 3 * 4,
                "bin_url": "/api/debug/pointcloud-map/latest.bin?lod=%s&revision=%s" % (level, pointcloud_map.revision),
            }
        )
    return DebugPointCloudMapView(
        map_name=descriptor.get("map_name"),
        map_version_id=descriptor.get("map_version_id"),
        revision=pointcloud_map.revision,
        frame_id=pointcloud_map.frame_id,
        source=str(descriptor.get("source") or "rtabmap_pointcloud_map"),
        source_kind=pointcloud_map.source_kind,
        geometry_stack=descriptor.get("geometry_stack"),
        rtabmap_db_path=descriptor.get("rtabmap_db_path"),
        pcd_path=pointcloud_map.path,
        point_count=pointcloud_map.point_count,
        fields=[{"name": "x", "datatype": "float32"}, {"name": "y", "datatype": "float32"}, {"name": "z", "datatype": "float32"}],
        bounds={"min": list(pointcloud_map.bounds_min), "max": list(pointcloud_map.bounds_max)},
        lods=lods,
        payload_bytes=payload_bytes,
        metadata={
            "asset_mtime": pointcloud_map.source_timestamp.isoformat(),
            "loaded_at": pointcloud_map.loaded_at.isoformat(),
            "pcd_data": pointcloud_map.header.data,
            "pcd_width": pointcloud_map.header.width,
            "pcd_height": pointcloud_map.header.height,
            "pcd_header_fields": list(pointcloud_map.header.fields),
            "not_sensor_scan": True,
        },
        metrics={
            "server_compute_ms": (time.perf_counter() - started_at) * 1000.0,
            "pcd_load_ms": pointcloud_map.load_ms,
            "preview_payload_bytes": payload_bytes,
        },
    )


def _missing_pointcloud_map_view(descriptor: Dict[str, Any], started_at: float) -> DebugPointCloudMapView:
    return DebugPointCloudMapView(
        map_name=descriptor.get("map_name"),
        map_version_id=descriptor.get("map_version_id"),
        revision=descriptor.get("revision"),
        frame_id=descriptor.get("frame_id"),
        source=str(descriptor.get("source") or "pointcloud_map"),
        source_kind=str(descriptor.get("source_kind") or "unavailable"),
        geometry_stack=descriptor.get("geometry_stack"),
        rtabmap_db_path=descriptor.get("rtabmap_db_path"),
        pcd_path=descriptor.get("pcd_path"),
        metadata={
            "expected_pcd_candidates": descriptor.get("expected_pcd_candidates", []),
            "rtabmap_profile": descriptor.get("rtabmap_profile", {}),
            "asset_summary": descriptor.get("asset_summary", {}),
            "quality_status": descriptor.get("quality_status"),
            "not_fallback_to_sensor_scan": True,
        },
        metrics={"server_compute_ms": (time.perf_counter() - started_at) * 1000.0},
    )


def _pointcloud_map_error_envelope(
    descriptor: Dict[str, Any],
    message: str,
    started_at: float,
    *,
    code: str,
) -> Dict[str, Any]:
    return _envelope(
        payload_type="pointcloud_map.meta",
        source=str(descriptor.get("source") or "pointcloud_map"),
        data=_missing_pointcloud_map_view(descriptor, started_at),
        status=DebugDataStatus.ERROR,
        frame_id=descriptor.get("frame_id"),
        error=DebugError(
            code=code,
            message=message,
            details={
                "pcd_path": descriptor.get("pcd_path"),
                "expected_pcd_candidates": descriptor.get("expected_pcd_candidates", []),
            },
        ),
    )


def _configured_pcd_candidates(runtime: GatewayRuntime) -> List[Path]:
    geometry_runtime = getattr(runtime, "platform_geometry_runtime", None)
    config = getattr(geometry_runtime, "config", None)
    rtabmap = getattr(config, "rtabmap", None)
    outputs = getattr(rtabmap, "outputs", None)
    root_text = str(getattr(rtabmap, "pcd_root", "") or "").strip()
    if not root_text:
        return []
    root = Path(root_text).expanduser()
    if root.suffix == ".pcd":
        return [root]
    asset_name = str(getattr(outputs, "pcd_asset_name", "") or "cloud_map.pcd")
    return [root / asset_name, root / "cloud_map.pcd", root / "map.pcd", root / "current.pcd"]


def _pcd_path_from_asset_refs(value: Any) -> str:
    if not isinstance(value, list):
        return ""
    for item in value:
        if not isinstance(item, dict):
            continue
        asset_type = str(item.get("asset_type") or "").lower()
        if asset_type == "pcd":
            return str(item.get("path") or "")
    return ""


def _first_non_empty(*values: Any) -> str:
    for value in values:
        if value is None:
            continue
        text = str(value).strip()
        if text:
            return text
    return ""


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
