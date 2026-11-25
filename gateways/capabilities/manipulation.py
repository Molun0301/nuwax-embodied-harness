"""同一任务中前往操作站位、执行设备操作并更新记忆。"""
from math import atan2

from contracts.base import utc_now
from core.capability import Capability
from core.config import value
from core.errors import GatewayError
from gateways.capabilities.common import rpc_errors
from gateways.capabilities.memory import remember_sample
from gateways.capabilities.navigation import navigate_to_pose
from services.manipulation.http_backend import DeviceError
from services.memory.semantic_map.room_segmentation import pose_in_scene_space


def _motion_options(_platform, args):
    if "target_instance_id" in args:
        return {"motion": True, "needs": ("nav_stack", "geometry")}


class _ApproachContext:
    def __init__(self, context):
        self.context = context

    def __getattr__(self, name):
        return getattr(self.context, name)

    def progress(self, value, stage, message, **data):
        self.context.progress(value, "approaching", message, navigation_stage=stage, **data)


def _map_cloud_chunks(platform, ctx):
    if platform.config.geometry.pipeline != "lio_3d":
        return
    page_points = 65_536
    chunk = platform.geometry.get_dense_cloud_chunk(max_points=page_points, timeout_sec=ctx.remaining())
    count = chunk.total_points
    while chunk.offset_points < count:
        yield chunk.xyz_f32_le
        offset = chunk.offset_points + len(chunk.xyz_f32_le) // 12
        if offset >= count:
            break
        ctx.check()
        chunk = platform.geometry.get_dense_cloud_chunk(expected_epoch=chunk.epoch, offset_points=offset,
                                                         max_points=page_points,
                                                         timeout_sec=ctx.remaining())


@rpc_errors
def _approach(platform, ctx, args, mode):
    instance = platform.memory.get_semantic_instance(args["target_instance_id"])
    if instance is None:
        raise GatewayError("当前记忆库中没有目标实例。", http_status=404)
    current = platform.geometry_state.wait_for_pose(ctx)
    config = platform.config.manipulation
    place_distance = value("manipulation.place_standoff_distance_m") if mode == "place" else ""
    distance = config.standoff_distance_m if place_distance == "" else place_distance
    pose = platform.memory.plan_manipulation_pose(instance, current, place=mode == "place",
        distance_m=distance, cloud_chunks=_map_cloud_chunks(platform, ctx))
    if pose is None:
        raise GatewayError("目标周围没有可达操作站位。", error_code="navigation_failed", http_status=409)
    q = pose.orientation
    yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    if mode == "pick":
        yaw += value("manipulation.pick_yaw_offset_rad")
    navigate_to_pose(platform, _ApproachContext(ctx), {"x": pose.position.x, "y": pose.position.y,
        "z": pose.position.z, "yaw_rad": yaw, "frame_id": pose.frame_id,
        "tolerance_position_m": config.approach_position_tolerance_m,
        "tolerance_yaw_rad": config.approach_yaw_tolerance_rad,
        # 离开或贴近台沿需要先后退；速度仍受机型后退上限约束。
        "allow_reverse": True, "motion_generation": args["motion_generation"]})
    return instance


def _remember_placement(platform, ctx, held, target):
    runtime = platform.perception.video_runtime
    reason = f"manipulation_place:{ctx.task_id}"
    labels = (held["label"],) if held and held["label"] else ()
    runtime.acquire(reason, open_vocabulary_classes=labels)
    try:
        sample, = runtime.run_burst(ctx, reason=reason, frame_count=1)
        metadata = {"source": "manipulation_place", "task_id": ctx.task_id}
        if target is not None and labels:
            spaces = platform.scene_mapping.capture_partition()["spaces"]
            room = next((space for space in spaces if space.primary_label and sample.pose is not None
                         and pose_in_scene_space(sample.pose, space)), None)
            surface = target.display_name or target.label
            location = f"{room.primary_label}的{surface}" if room is not None else surface
            entry = remember_sample(platform, sample, title=f"物品位置：{labels[0]}",
                summary=f"{labels[0]} 已放在{location}上。", tags=[*labels, location],
                metadata={**metadata, "placement_target_instance_id": target.instance_id})
            platform.memory.index_scene_embedding(entry)
        else:
            remember_sample(platform, sample, metadata=metadata)
    finally:
        runtime.release(reason)


def execute_manipulation_task(platform, ctx, args):
    service = platform.manipulation
    mode = args.get("goal_mode", "transfer")
    try:
        instance = None
        if "target_instance_id" in args:
            ctx.progress(0.0, "approaching", "正在前往操作站位。")
            instance = _approach(platform, ctx, args, mode)
        ctx.progress(0.5, "manipulating", "正在向设备提交操作指令。")
        binding = service.bind()
        result = service.run(ctx, binding, {"instruction": args["instruction"].strip(), "goal_mode": mode,
            "timeout_sec": args.get("timeout_sec", 120), "max_steps": args.get("max_steps", 1440)})
        if mode == "pick":
            picked_at = utc_now()
            service.held_object = {"instance_id": instance.instance_id if instance else None,
                "label": instance.label if instance else None, "started_at": picked_at.isoformat()}
            if instance is not None:
                platform.memory.mark_instance_moved(instance, task_id=ctx.task_id, timestamp=picked_at)
        elif mode == "place":
            held = service.held_object
            service.held_object = None
            _remember_placement(platform, ctx, held, instance)
        return {**result, "held_object": service.held_object}
    except DeviceError as error:
        raise GatewayError(str(error), error_code=error.code,
            http_status=503 if error.code == "manipulation_unavailable" else 409) from error


CAPABILITIES = [Capability("execute_manipulation_task", "执行抓取、放置或完整搬运。给定目标实例时，先前往面向目标的操作站位。",
    {"type": "object", "properties": {
        "instruction": {"type": "string", "minLength": 1, "maxLength": 2000, "pattern": r"\S"},
        "timeout_sec": {"type": "number", "minimum": 1, "maximum": 1800, "default": 120},
        "max_steps": {"type": "integer", "minimum": 1, "maximum": 10000, "default": 1440},
        "goal_mode": {"type": "string", "enum": ["pick", "place", "transfer"], "default": "transfer"},
        "target_instance_id": {"type": "string", "description": "抓取目标实例或放置台面实例。省略时原地操作。"}},
     "required": ["instruction"], "additionalProperties": False}, "task", execute_manipulation_task,
     timeout_sec=120, before_submit=_motion_options)]
