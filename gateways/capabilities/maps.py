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
    return asdict(start_mapping_session(platform, args["map_name"],
        continue_saved=args.get("continue_saved", False), timeout_sec=args.get("timeout_sec")))


def start_mapping_session(platform, name, *, continue_saved=False, timeout_sec=None):
    """普通建图、探索与新楼层共用同一会话及元数据流程。"""
    platform.input_bridge.start()
    snap = platform.geometry.start_mapping(name, continue_saved=continue_saved, timeout_sec=timeout_sec)
    if not continue_saved:
        meta = platform.maps.read_meta(name)
        now = datetime.now(timezone.utc).isoformat()
        platform.maps.write_meta(name, {**meta, "display_name": meta.get("display_name", name),
                                  "created_at": meta.get("created_at", now), "updated_at": now})
    return snap


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
    target = platform.maps.root / name
    if name != source_name and (target.exists() or target.is_symlink()):
        raise GatewayError("名称已被占用：" + name, error_code="map_name_occupied", http_status=409)
    ctx.progress(0.1, "saving", f"正在保存地图 {name}")
    migrating = bool(source_name and name != source_name and not (source is not None and source.saved))
    if migrating:
        platform.memory.expect_rename(source_name, name)
    try:
        ctx.check()
        # 发出保存后完成全部收尾，不把任务取消或期限传给这段提交过程。
        saved = platform.geometry.save_map(name)
    except BaseException:
        if migrating:
            platform.memory.clear_expected_rename()
        raise
    renamed = bool(source_name and source_name != saved.map_name)
    if renamed:
        platform.maps.write_forbidden_zones(saved.map_name, zones)
    now = datetime.now(timezone.utc).isoformat()
    meta.update(display_name=saved.map_name if renamed else meta.get("display_name", saved.map_name),
                created_at=now if renamed else meta.get("created_at", now), updated_at=now)
    if "georeference" in args:
        meta["georeference"] = args["georeference"]
    if isinstance(meta.get("georeference"), dict):
        meta["georeference"] = {**meta["georeference"], "map_name": saved.map_name}
    platform.maps.write_meta(saved.map_name, meta)
    if migrating:
        platform.memory.wait_for_library(saved.map_name)
        platform.maps.delete(source_name)
    return asdict(saved)


@rpc_errors
def relocalize(platform, ctx, args):
    seed = args.get("planar_seed")
    result = platform.geometry.relocalize(None if seed is None else tuple(seed),
                                          timeout_sec=ctx.remaining())
    ctx.check()
    if not result.succeeded:
        raise GatewayError(result.message, error_code="geometry_relocalization_failed", http_status=409,
                           details={"fitness": result.fitness, "frames_used": result.frames_used})
    return asdict(result)


@rpc_errors
def load_map(platform, ctx, args):
    loaded = platform.geometry.load_map(args["map_name"], timeout_sec=ctx.remaining())
    ctx.check()
    if loaded.view_only:
        return {"map_name": loaded.map_name, "localized": False, "reason": loaded.reason}
    return relocalize(platform, ctx, args)


@rpc_errors
def delete_map(platform, args):
    name = args["map_name"]
    snap = platform.geometry_state.snapshot()
    if snap is not None and snap.map_name == name and components.status().get("geometry", {}).get("state") == "running":
        # 控制应答已证明会话结束，不等待订阅线程消费新状态。
        platform.geometry.stop_session(timeout_sec=args.get("timeout_sec"))
    platform.memory.delete_library(name, recreate_active=False)
    platform.maps.delete(name)
    return {"map_name": name, "deleted": True}


def replace_forbidden_zones(platform, args):
    platform.maps.write_forbidden_zones(args["map_name"], args["zones"])
    return {"map_name": args["map_name"], "zones": args["zones"]}


def list_forbidden_zones(platform, args):
    snapshot = platform.geometry_state.snapshot()
    name = args.get("map_name") or (snapshot.map_name if snapshot is not None else "")
    return {"map_name": name, "zones": platform.maps.read_forbidden_zones(name)}


def georeference_schema():
    return schema(
        {
            "schema_version": {
                "type": "integer",
                "const": MAP_GEOREFERENCE_SCHEMA_VERSION,
                "description": "地理参考的格式版本，固定为 1。",
            },
            "map_name": {"type": "string", "minLength": 1, "description": "地理参考所属的地图名称。"},
            "map_frame_id": {"type": "string", "minLength": 1, "description": "右手系且 Z 轴向上的地图坐标系标识。"},
            "source_crs": {"type": "string", "const": MAP_GEOREFERENCE_SOURCE_CRS,
                           "description": "经纬度坐标参考系，固定为 EPSG:4326。"},
            "projected_crs": {
                "type": "string",
                "const": MAP_GEOREFERENCE_PROJECTED_CRS,
                "description": "平面投影坐标参考系，固定为 EPSG:3857。",
            },
            "axis_convention": {
                "type": "string",
                "const": MAP_GEOREFERENCE_AXIS_CONVENTION,
                "description": "投影坐标的轴向约定，X 向东、Y 向北、Z 向上。",
            },
            "anchor_lat_deg": {
                "type": "number",
                "minimum": -MAP_GEOREFERENCE_MAX_LATITUDE_DEG,
                "maximum": MAP_GEOREFERENCE_MAX_LATITUDE_DEG,
                "description": "地理参考锚点的纬度（度）。",
            },
            "anchor_lng_deg": {"type": "number", "minimum": -180.0, "maximum": 180.0,
                               "description": "地理参考锚点的经度（度）。"},
            "anchor_easting_m": {"type": "number", "description": "锚点的投影东向坐标（米）。"},
            "anchor_northing_m": {"type": "number", "description": "锚点的投影北向坐标（米）。"},
            "map_origin_x_m": {
                "type": "number",
                "minimum": -MAP_GEOREFERENCE_MAX_COORDINATE_M,
                "maximum": MAP_GEOREFERENCE_MAX_COORDINATE_M,
                "description": "锚点在地图坐标系中的 X 坐标（米）。",
            },
            "map_origin_y_m": {
                "type": "number",
                "minimum": -MAP_GEOREFERENCE_MAX_COORDINATE_M,
                "maximum": MAP_GEOREFERENCE_MAX_COORDINATE_M,
                "description": "锚点在地图坐标系中的 Y 坐标（米）。",
            },
            "map_from_projected_yaw_rad": {
                "type": "number",
                "minimum": -math.pi,
                "exclusiveMaximum": math.pi,
                "description": "投影坐标系到地图坐标系的逆时针偏航角（弧度）。",
            },
            "altitude_policy": {
                "type": "string",
                "const": MAP_GEOREFERENCE_ALTITUDE_POLICY,
                "description": "高度策略，固定使用地图平面高度。",
            },
            "map_plane_z_m": {
                "type": "number",
                "minimum": -MAP_GEOREFERENCE_MAX_COORDINATE_M,
                "maximum": MAP_GEOREFERENCE_MAX_COORDINATE_M,
                "description": "地名导航目标使用的地图平面 Z 坐标（米）。",
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
    result = schema({"map_name": MAP_NAME, "zones": {"type": "array", "items": zone,
                    "description": "要保存的完整禁行区列表。传空列表会清空该地图的禁行区。"}}, ("map_name", "zones"))
    result["$defs"] = definitions
    return result


MAP_NAME = {"type": "string", "description": "地图目录中的地图名称。"}
TIMEOUT = {"type": "number", "exclusiveMinimum": 0, "description": "任务总时限（秒）。留空时为 900 秒。"}
SEED = {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3,
        "description": "重定位的初始位姿猜测，依次为 X、Y（米）和偏航角（弧度）。留空时不提供初始位姿。"}
CAPABILITIES = [
    Capability("list_maps", "列出地图目录和当前会话。", schema(), "query", list_maps),
    Capability("get_active_map", "读取地图会话、位姿、导航、探索与重定位状态。", schema(), "query", get_active_map),
    Capability("start_mapping", "开始新图或同名续建。",
               schema({"map_name": {"type": "string", "description": "要新建或续建的地图名称。"},
                       "continue_saved": {"type": "boolean", "description": "是否续建同名已保存地图。留空时开始新图。"}}, ("map_name",)),
               "action", start_mapping, needs=("geometry",), normalize_names=("map_name",)),
    Capability("end_map_session", "结束地图会话并返回空闲。", schema(), "action", end_map_session),
    Capability("save_map", "保存当前地图或另存为新名字。",
               schema({"map_name": {"type": "string", "description": "保存后的地图名称。留空时使用当前会话名称。"},
                       "georeference": {**georeference_schema(), "description": "经纬度与地图坐标间的转换定义。留空时沿用现有地理参考。"},
                       "timeout_sec": {**TIMEOUT, "description": "发出保存请求前的任务时限（秒）。留空时为 900 秒，已发出的保存会完成收尾。"}}),
               "task", save_map, needs=("geometry",), timeout_sec=900, normalize_names=("map_name",)),
    Capability("load_map", "加载地图并执行重定位。",
               schema({"map_name": MAP_NAME, "timeout_sec": TIMEOUT}, ("map_name",)),
               "task", load_map, needs=("geometry",), timeout_sec=900, normalize_names=("map_name",)),
    Capability("relocalize_active_map", "在已加载地图上重定位。",
               schema({"planar_seed": SEED, "timeout_sec": TIMEOUT}),
               "task", relocalize, needs=("geometry",), timeout_sec=900),
    Capability("delete_map", "删除地图及同目录的记忆库。",
               schema({"map_name": MAP_NAME, "timeout_sec": {**TIMEOUT,
                       "description": "删除当前地图前结束会话的调用时限（秒）。留空时不指定该时限。"}}, ("map_name",)), "action", delete_map,
               normalize_names=("map_name",)),
    Capability("list_forbidden_zones", "读取地图禁行区。",
               schema({"map_name": {"type": "string", "description": "要读取禁行区的地图名称。留空时使用当前会话地图。"}}), "query",
               list_forbidden_zones, normalize_names=("map_name",)),
    Capability("replace_forbidden_zones", "整体替换禁行区，下次规划生效。",
               forbidden_schema(),
               "action", replace_forbidden_zones, normalize_names=("map_name",)),
]
