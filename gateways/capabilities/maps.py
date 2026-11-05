"""地图会话、元数据与禁行区能力。"""
from dataclasses import asdict
from datetime import datetime, timezone
import math
from contracts.forbidden_zones import ForbiddenZone
from contracts.map_workspace import (
    MAP_GEOREFERENCE_SCHEMA_VERSION, MAP_GEOREFERENCE_SOURCE_CRS, MAP_GEOREFERENCE_PROJECTED_CRS,
    MAP_GEOREFERENCE_AXIS_CONVENTION, MAP_GEOREFERENCE_MAX_LATITUDE_DEG, MAP_GEOREFERENCE_MAX_COORDINATE_M,
    MAP_GEOREFERENCE_ALTITUDE_POLICY,
)

from core import components
from core.capability import Capability
from core.errors import GatewayError
from gateways.capabilities.common import rpc_errors, schema


def get_active_map(platform, args):
    snap = platform.geometry_state.snapshot()
    return {"running": False, "message": "Geometry 未运行（尚未使用地图功能）"} if snap is None else {"running": platform.geometry_state.connected, **asdict(snap)}


def list_maps(platform, args):
    snap = platform.geometry_state.snapshot()
    return {"maps": [asdict(item) for item in platform.maps.list()],
            "phase": None if snap is None else snap.phase.value, "map_name": "" if snap is None else snap.map_name}


@rpc_errors
def start_mapping(platform, args):
    snap = platform.geometry.start_mapping(args["map_name"], continue_saved=args.get("continue_saved", False))
    if not args.get("continue_saved", False):
        meta = platform.maps.read_meta(args["map_name"])
        now = datetime.now(timezone.utc).isoformat()
        platform.maps.write_meta(args["map_name"], {**meta, "display_name": meta.get("display_name", args["map_name"]),
                                  "created_at": meta.get("created_at", now), "updated_at": now})
        platform.memory.ensure_memory_library(library_name=args["map_name"])
    return asdict(snap)


@rpc_errors
def end_map_session(platform, args):
    if components.status().get("geometry", {}).get("state") != "running":
        return {"phase": "idle", "map_name": ""}
    return asdict(platform.geometry.stop_session())


@rpc_errors
def save_map(platform, ctx, args):
    snap = platform.geometry_state.snapshot()
    source_name = snap.map_name if snap is not None else ""
    source = platform.maps.get(source_name) if source_name else None
    meta = platform.maps.read_meta(source_name) if source_name else {}
    zones = platform.maps.read_forbidden_zones(source_name) if source_name else []
    name = args.get("map_name", "") or source_name
    ctx.progress(0.1, "saving", f"正在保存地图 {name}")
    saved = platform.geometry.save_map(name, timeout_sec=ctx.remaining())
    ctx.check()
    renamed = bool(source_name and source_name != saved.map_name)
    if renamed:
        platform.maps.write_forbidden_zones(saved.map_name, zones)
        if source is not None and not source.saved:
            platform.memory.rename_memory_library(library_name=source_name, new_library_name=saved.map_name)
            platform.maps.delete(source_name)
        else:
            platform.memory.ensure_memory_library(library_name=saved.map_name)
    now = datetime.now(timezone.utc).isoformat()
    meta.update(display_name=meta.get("display_name", saved.map_name),
                created_at=meta.get("created_at", now), updated_at=now)
    if "georeference" in args:
        meta["georeference"] = args["georeference"]
    platform.maps.write_meta(saved.map_name, meta)
    return asdict(saved)


@rpc_errors
def relocalize(platform, ctx, args):
    seed = args.get("planar_seed")
    result = platform.geometry.relocalize(None if seed is None else tuple(seed),
                                          timeout_sec=ctx.remaining())
    ctx.check()
    if not result.succeeded:
        raise GatewayError(result.message, error_code="geometry_relocalization_failed", http_status=409)
    return asdict(result)


@rpc_errors
def load_map(platform, ctx, args):
    platform.geometry.load_map(args["map_name"], timeout_sec=ctx.remaining())
    ctx.check()
    result = relocalize(platform, ctx, args)
    platform.memory.activate_memory_library(library_name=args["map_name"],
                    load_history=args.get("load_memory_history", True), reset_library=args.get("reset_memory_library", False))
    return result


@rpc_errors
def delete_map(platform, args):
    name = args["map_name"]
    snap = platform.geometry_state.snapshot()
    if snap is not None and snap.map_name == name and components.status().get("geometry", {}).get("state") == "running":
        platform.geometry.stop_session()
    platform.maps.delete(name)
    if args.get("delete_memory_library", False):
        platform.memory.delete_memory_library(library_name=name)
    return {"map_name": name, "deleted": True}


def replace_forbidden_zones(platform, args):
    platform.maps.write_forbidden_zones(args["map_name"], args["zones"])
    return {"map_name": args["map_name"], "zones": args["zones"]}


def georeference_schema():
    return schema(
        {
            "schema_version": {
                "type": "integer",
                "const": MAP_GEOREFERENCE_SCHEMA_VERSION,
            },
            "map_name": {"type": "string", "minLength": 1},
            "map_frame_id": {"type": "string", "minLength": 1},
            "source_crs": {"type": "string", "const": MAP_GEOREFERENCE_SOURCE_CRS},
            "projected_crs": {
                "type": "string",
                "const": MAP_GEOREFERENCE_PROJECTED_CRS,
            },
            "axis_convention": {
                "type": "string",
                "const": MAP_GEOREFERENCE_AXIS_CONVENTION,
            },
            "anchor_lat_deg": {
                "type": "number",
                "minimum": -MAP_GEOREFERENCE_MAX_LATITUDE_DEG,
                "maximum": MAP_GEOREFERENCE_MAX_LATITUDE_DEG,
            },
            "anchor_lng_deg": {"type": "number", "minimum": -180.0, "maximum": 180.0},
            "anchor_easting_m": {"type": "number"},
            "anchor_northing_m": {"type": "number"},
            "map_origin_x_m": {
                "type": "number",
                "minimum": -MAP_GEOREFERENCE_MAX_COORDINATE_M,
                "maximum": MAP_GEOREFERENCE_MAX_COORDINATE_M,
            },
            "map_origin_y_m": {
                "type": "number",
                "minimum": -MAP_GEOREFERENCE_MAX_COORDINATE_M,
                "maximum": MAP_GEOREFERENCE_MAX_COORDINATE_M,
            },
            "map_from_projected_yaw_rad": {
                "type": "number",
                "minimum": -math.pi,
                "exclusiveMaximum": math.pi,
            },
            "altitude_policy": {
                "type": "string",
                "const": MAP_GEOREFERENCE_ALTITUDE_POLICY,
            },
            "map_plane_z_m": {
                "type": "number",
                "minimum": -MAP_GEOREFERENCE_MAX_COORDINATE_M,
                "maximum": MAP_GEOREFERENCE_MAX_COORDINATE_M,
            },
        },
        required=(
            "schema_version",
            "map_name",
            "map_frame_id",
            "source_crs",
            "projected_crs",
            "axis_convention",
            "anchor_lat_deg",
            "anchor_lng_deg",
            "anchor_easting_m",
            "anchor_northing_m",
            "map_origin_x_m",
            "map_origin_y_m",
            "map_from_projected_yaw_rad",
            "altitude_policy",
            "map_plane_z_m",
        ),
    )


def forbidden_schema():
    zone = ForbiddenZone.model_json_schema()
    definitions = zone.pop("$defs", {})
    result = schema({"map_name": TEXT, "zones": {"type": "array", "items": zone}}, ("map_name", "zones"))
    result["$defs"] = definitions
    return result


TEXT = {"type": "string"}
TIMEOUT = {"type": "number", "exclusiveMinimum": 0}
SEED = {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3}
CAPABILITIES = [
    Capability("list_maps", "列出地图目录和当前会话。", schema(), "query", list_maps),
    Capability("get_active_map", "读取地图会话、位姿、导航、探索与重定位状态。", schema(), "query", get_active_map),
    Capability("start_mapping", "开始新图或同名续建。",
               schema({"map_name": TEXT, "continue_saved": {"type": "boolean"}}, ("map_name",)),
               "action", start_mapping, needs=("geometry",)),
    Capability("end_map_session", "结束地图会话并返回空闲。", schema({"reason": TEXT}), "action", end_map_session),
    Capability("save_map", "保存当前地图或另存为新名字。",
               schema({"map_name": TEXT, "reason": TEXT, "georeference": georeference_schema()}),
               "task", save_map, needs=("geometry",), timeout_sec=120),
    Capability("load_map", "加载地图并执行重定位。",
               schema({"map_name": TEXT, "load_memory_history": {"type": "boolean"},
                       "reset_memory_library": {"type": "boolean"}, "timeout_sec": TIMEOUT}, ("map_name",)),
               "task", load_map, needs=("geometry",), timeout_sec=120),
    Capability("relocalize_active_map", "在已加载地图上重定位。",
               schema({"planar_seed": SEED, "timeout_sec": TIMEOUT}),
               "task", relocalize, needs=("geometry",), timeout_sec=120),
    Capability("delete_map", "删除地图并按参数删除同名记忆库。",
               schema({"map_name": TEXT, "delete_memory_library": {"type": "boolean"}}, ("map_name",)), "action", delete_map),
    Capability("list_forbidden_zones", "读取地图禁行区。", schema({"map_name": TEXT}, ("map_name",)), "query",
               lambda p, a: {"map_name": a["map_name"], "zones": p.maps.read_forbidden_zones(a["map_name"])}),
    Capability("replace_forbidden_zones", "整体替换禁行区，下次规划生效。",
               forbidden_schema(),
               "action", replace_forbidden_zones),
]
