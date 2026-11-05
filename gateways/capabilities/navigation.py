"""下发一个目标，并用 Geometry 状态结束同一个任务。"""
from __future__ import annotations

from dataclasses import asdict
import logging
import math
import time
from itertools import chain

from core import components
from core.capability import Capability
from core.errors import GatewayError
from core.tasks import Cancelled
from gateways.capabilities.common import rpc_errors, schema
from services.geometry.client import GeometryError, NavigationGoal
from services.geometry.state import GoalStatus, Pose
from services.navigation.places import map_coordinates

LOGGER = logging.getLogger(__name__)


@rpc_errors
def navigate_to_pose(platform, ctx, args, *, region=None):
    ctx.check()
    snapshot = platform.geometry_state.snapshot()
    if "z" in args:
        z = args["z"]
    elif snapshot.info.pipeline == "slam_2d":
        z = 0.0
    else:
        z = platform.geometry_state.wait_for_pose(ctx).z
    yaw = args.get("yaw_rad", 0.0)
    pose = Pose(args["x"], args["y"], z, 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2),
                args.get("frame_id", snapshot.info.map_frame_id), time.time_ns())
    try:
        goal_id = platform.geometry.set_goal(NavigationGoal(pose, args["motion_generation"],
            allow_reverse=args.get("allow_reverse"), position_tolerance_m=args.get("tolerance_position_m"),
            yaw_tolerance_rad=args.get("tolerance_yaw_rad")), timeout_sec=ctx.remaining())
    except GeometryError as error:
        if error.code == "FAILED_PRECONDITION":
            raise GatewayError(error.message, error_code="not_localized", http_status=409) from error
        raise
    finished = False
    try:
        # 控制应答可先于订阅到达；先接到自己的目标，避免消费尚在途的加载/定位状态。
        first = platform.geometry_state.wait_for(lambda state: (
            state.navigation is not None and state.navigation.goal_id == goal_id
            or state.previous_navigation is not None and state.previous_navigation.goal_id == goal_id), ctx)
        initial = first.navigation if first.navigation is not None and first.navigation.goal_id == goal_id else first.previous_navigation
        last = None
        for navigation in chain((initial,), platform.geometry_state.follow_goal(goal_id, ctx)):
            if navigation is last:
                continue
            last = navigation
            ctx.progress(0.0, navigation.status.value, navigation.message,
                         goal_id=goal_id, remaining_distance_m=navigation.remaining_distance_m)
            if navigation.status == GoalStatus.SUCCEEDED:
                finished = True
                return {"goal_id": goal_id, "arrival": "target"}
            if navigation.status in {GoalStatus.CANCELLED, GoalStatus.REPLACED, GoalStatus.STOPPED}:
                finished = True
                raise Cancelled(navigation.message or "导航已被取消或更新的命令停止")
            if (region is not None and navigation.no_feasible_path
                    and platform.navigation_memory.contains(region, platform.geometry_state.pose())):
                finished = True
                platform.geometry.cancel_goal(goal_id, timeout_sec=ctx.remaining())
                return {"goal_id": goal_id, "arrival": "region"}
            if navigation.status == GoalStatus.FAILED:
                finished = True
                raise GatewayError(navigation.message, error_code="navigation_failed", http_status=409)
    finally:
        if not finished:
            try:
                platform.geometry.cancel_goal(goal_id)
            except GeometryError:
                LOGGER.exception("取消导航目标失败：%s", goal_id)


@rpc_errors
def navigate_to_named_location(platform, ctx, args):
    target = platform.navigation_memory.resolve(args["target_name"], camera_id=args.get("camera_id"))
    pose = target.pose
    result = navigate_to_pose(platform, ctx, {"x": pose.x, "y": pose.y, "z": pose.z,
        "yaw_rad": pose.yaw, "frame_id": pose.frame_id, "motion_generation": args["motion_generation"]},
        region=target.region)
    result["target_name"] = target.target_name
    if args.get("verify_on_arrival", True):
        result["verification"] = platform.navigation_memory.verify(target, camera_id=args.get("camera_id"),
            similarity_threshold=args.get("verify_similarity_threshold", 0.55), goal_id=result["goal_id"])
    return result


@rpc_errors
def navigate_to_place(platform, ctx, args):
    state = platform.geometry_state.snapshot()
    reference = platform.maps.read_meta(state.map_name).get("georeference")
    if reference is None:
        raise GatewayError("当前地图没有地理参考。", error_code="map_georeference_missing", http_status=409)
    place = platform.places.search(args["query_text"], timeout_sec=ctx.remaining())
    ctx.check()
    x, y, z = map_coordinates(place["lat"], place["lon"], reference)
    result = navigate_to_pose(platform, ctx, {"x": x, "y": y, "z": z,
        "frame_id": reference["map_frame_id"], "motion_generation": args["motion_generation"]})
    return {**result, "place": place}


@rpc_errors
def cancel_navigation(platform, args):
    if components.status().get("geometry", {}).get("state") != "running":
        return {"running": False, "navigation": None, "message": "没有活动目标"}
    platform.geometry.cancel_goal(args.get("goal_id", ""))
    return navigation_state(platform, {})


def navigation_state(platform, args):
    state = platform.geometry_state.snapshot()
    return {"running": platform.geometry_state.connected,
        "navigation": None if state is None or state.navigation is None else asdict(state.navigation),
        "exploration": None if state is None or state.exploration is None else asdict(state.exploration)}


TIMEOUT = {"type": "number", "exclusiveMinimum": 0}
CAPABILITIES = [
    Capability("navigate_to_pose", "导航到当前地图坐标中的位姿。",
        schema({"x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"},
                "yaw_rad": {"type": "number"}, "frame_id": {"type": "string"},
                "tolerance_position_m": {"type": "number", "minimum": 0},
                "tolerance_yaw_rad": {"type": "number", "minimum": 0},
                "allow_reverse": {"type": "boolean"}, "timeout_sec": TIMEOUT}, ("x", "y")),
        "task", navigate_to_pose, needs=("nav_stack", "geometry"), motion=True, timeout_sec=600),
    Capability("navigate_to_place", "通过开放街道地图地名查找并导航。",
        schema({"query_text": {"type": "string", "minLength": 1}, "timeout_sec": TIMEOUT}, ("query_text",)),
        "task", navigate_to_place, needs=("nav_stack", "geometry"), motion=True, timeout_sec=600),
    Capability("navigate_to_named_location", "导航到记忆中的命名地点或区域，可在到达后核对。",
        schema({"target_name": {"type": "string"}, "camera_id": {"type": "string"},
                "verify_on_arrival": {"type": "boolean"},
                "verify_similarity_threshold": {"type": "number", "minimum": 0, "maximum": 1},
                "timeout_sec": TIMEOUT}, ("target_name",)),
        "task", navigate_to_named_location, needs=("nav_stack", "geometry"), motion=True, timeout_sec=600),
    Capability("cancel_navigation", "取消指定目标或当前目标，不等待停稳。",
        schema({"goal_id": {"type": "string"}}), "action", cancel_navigation),
    Capability("get_navigation_state", "读取 Geometry 导航与探索状态。", schema(), "query", navigation_state),
]
