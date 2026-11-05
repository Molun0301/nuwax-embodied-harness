"""同一任务线程中顺序执行跨层导航，不保存中断进度。"""
from __future__ import annotations

import logging

from contracts.building_navigation import FloorPose
from core.errors import GatewayError
from providers.facilities import FacilityUnavailable
from services.navigation.building_catalog import route_connections

LOGGER = logging.getLogger(__name__)


def building(platform, building_id):
    topology = platform.buildings.get(building_id)
    if topology is None:
        raise GatewayError("楼栋目录不存在。", error_code="building_not_found", http_status=404)
    return topology


def floor(topology, floor_id):
    for value in topology.floors:
        if value.floor_id == floor_id and value.enabled:
            return value
    raise GatewayError("目标楼层未登记或已停用。", error_code="floor_not_found", http_status=404)


def current_floor(platform, topology):
    snapshot = platform.geometry_state.snapshot()
    name = snapshot.map_name if snapshot is not None else ""
    for value in topology.floors:
        if value.map_name == name and value.enabled:
            return value
    raise GatewayError("当前地图不属于该楼栋的可用楼层。", error_code="current_floor_unknown", http_status=409)


def _navigate(platform, ctx, pose: FloorPose, motion_generation, observed_pose=None):
    from gateways.capabilities.navigation import navigate_to_pose
    ctx.check()
    args = {"x": pose.x, "y": pose.y, "yaw_rad": pose.yaw_rad, "motion_generation": motion_generation}
    if observed_pose is not None:
        args["z"] = observed_pose.z
    return navigate_to_pose(platform, ctx, args)


def load_floor(platform, ctx, record, seed=None):
    """加载与重定位都由Geometry给出真实结果，不读取地图内部文件。"""
    ctx.check()
    snapshot = platform.geometry.load_map(record.map_name, timeout_sec=ctx.remaining())
    ctx.check()
    planar_seed = ((seed.x, seed.y, seed.yaw_rad)
                   if seed is not None and snapshot.info.pipeline == "slam_2d" else None)
    localized = platform.geometry.relocalize(planar_seed, timeout_sec=ctx.remaining())
    ctx.check()
    if not localized.succeeded:
        raise GatewayError(localized.message, error_code="geometry_relocalization_failed", http_status=409)
    return localized


def _operate(provider, connection, action, target_floor_id, ctx):
    try:
        return provider.operate(connection.facility_id, action, target_floor_id=target_floor_id,
                                timeout_sec=ctx.remaining())
    except FacilityUnavailable as error:
        raise GatewayError(str(error), error_code="facility_unavailable", http_status=503) from error


def _await_docked(provider, connection, floor_id, ctx):
    while True:
        ctx.check()
        try:
            snapshot = provider.snapshot(connection.facility_id, timeout_sec=ctx.remaining())
        except FacilityUnavailable as error:
            raise GatewayError(str(error), error_code="facility_unavailable", http_status=503) from error
        if not snapshot.available or snapshot.fault or snapshot.occupied_by_other:
            raise GatewayError(snapshot.fault or "设施不可用或被其他使用者占用。",
                               error_code="facility_unavailable", http_status=409)
        if (snapshot.floor_id == floor_id and snapshot.door == "open" and not snapshot.moving
                and snapshot.docking_verified and snapshot.docking_calibration_ref == connection.docking_calibration_ref):
            return snapshot
        ctx.progress(0.0, "waiting_facility", "等待设施停层、开门与停靠反馈。", floor_id=floor_id)
        ctx.wait(0.1)


def transfer(platform, ctx, topology, target, motion_generation, *, create_target=False):
    """跨每条连接后加载对应楼层；未知目标楼层由探索处理器开始新图。"""
    source = current_floor(platform, topology)
    route = route_connections(topology, source.floor_id, target.floor_id, platform.buildings.providers)
    completed = []
    for connection in route:
        ctx.check()
        destination = floor(topology, connection.to_floor_id)
        provider = platform.buildings.providers[connection.provider_id]
        ctx.progress(0.0, "navigating_waiting", "前往设施等待位置。", floor_id=source.floor_id,
                     connection_id=connection.connection_id)
        _navigate(platform, ctx, connection.waiting_pose, motion_generation)
        ctx.check()
        _operate(provider, connection, "reserve", None, ctx)
        exited = False
        try:
            ctx.check()
            _operate(provider, connection, "call", source.floor_id, ctx)
            _await_docked(provider, connection, source.floor_id, ctx)
            _operate(provider, connection, "hold_door", source.floor_id, ctx)
            _navigate(platform, ctx, connection.boarding_pose, motion_generation)
            ctx.check()
            platform.geometry.stop_session(timeout_sec=ctx.remaining())
            ctx.check()
            ctx.progress(0.0, "transporting", "设施正在跨层转移。", floor_id=destination.floor_id)
            _operate(provider, connection, "transport", destination.floor_id, ctx)
            _await_docked(provider, connection, destination.floor_id, ctx)
            _operate(provider, connection, "hold_door", destination.floor_id, ctx)
            if create_target and destination.floor_id == target.floor_id:
                # 首次建图的原点由Geometry在接收处建立，不伪造目标地图或定位结果。
                observed = platform.geometry.start_mapping(destination.map_name, timeout_sec=ctx.remaining())
            else:
                ctx.progress(0.0, "localizing", "加载目标楼层地图并重定位。", map_name=destination.map_name)
                observed = load_floor(platform, ctx, destination, connection.arrival_pose)
            ctx.check()
            _navigate(platform, ctx, connection.exit_pose, motion_generation, observed.pose)
            completed.append(connection.connection_id)
            source = destination
            exited = True
        finally:
            if exited:
                _operate(provider, connection, "release", None, ctx)
            else:
                try:
                    _operate(provider, connection, "release", None, ctx)
                except GatewayError:
                    LOGGER.exception("跨层任务中断后释放设施失败。")
    return tuple(completed)


def navigate_to_floor(platform, ctx, args):
    topology = building(platform, args["building_id"])
    source = current_floor(platform, topology)
    target = floor(topology, args.get("floor_id", source.floor_id))
    route = transfer(platform, ctx, topology, target, args["motion_generation"])
    localized = load_floor(platform, ctx, target) if not route else None
    ctx.progress(0.0, "navigating_final", "前往目标楼层内的目标位姿。", floor_id=target.floor_id)
    result = _navigate(platform, ctx, FloorPose(args["x"], args["y"], args.get("yaw_rad", 0.0)),
                       args["motion_generation"], localized.pose if localized is not None else None)
    return {"building_id": topology.building_id, "floor_id": target.floor_id,
            "map_name": target.map_name, "route": list(route), "navigation": result}
