"""跟随能力：统一感知给目标，原生跟随进程给速度，守卫执行。"""
from dataclasses import asdict
import time

from core.capability import Capability
from core.errors import GatewayError
from core.target_query import normalize_query
from gateways.capabilities.common import rpc_errors, schema
from gateways.person_follow import (
    PERSON_FOLLOW_LOST, PERSON_FOLLOW_TRACKED, PERSON_FOLLOW_TRACKING,
    PersonFollowBridgeError, PersonFollowBridgeUnavailable, _source_timestamp_ns, build_person_follow_target,
)
from services.navigation.motion_client import MotionSource


def _best_detection(query, context):
    label = normalize_query(query)
    return max((item for item in context.observation.detections_2d if normalize_query(item.label) == label),
               key=lambda item: item.score, default=None)


def _calibration(platform, ctx, camera_frame):
    transforms = platform.geometry.get_frame_tree(timeout_sec=ctx.remaining())
    matches = [item for item in transforms if item.child_frame == camera_frame and item.timestamp_ns == 0]
    if len(matches) != 1:
        raise GatewayError("跟随相机缺少唯一直接静态标定边。", error_code="person_follow_calibration_unavailable", http_status=503)
    return matches[0]


def _cloud_revision(platform):
    snapshot = platform.streams.get_pointcloud_map_snapshot()
    if snapshot is None or snapshot.point_count == 0 or snapshot.revision_ordinal <= 0:
        raise GatewayError("跟随需要已发布的非空点云地图。", error_code="person_follow_cloud_unavailable", http_status=503)
    return snapshot.revision_ordinal


def _run_follow_target(platform, ctx, args):
    query = args["query"].strip()
    camera_id, backend = args.get("camera_id"), args.get("detector_backend")
    duration, interval, lost_timeout = args.get("duration_sec", 2.0), args.get("interval_sec", 0.1), args.get("lost_timeout_sec", 0.8)
    runtime, bridge = platform.perception.video_runtime, platform.follow
    reason = "follow:" + ctx.task_id
    bridge.start()
    started = time.monotonic()
    last_stamp = None
    last_detection = last_target = calibration = last_command = None
    lost_since = None
    cloud_revision = 0
    target_seq = 0
    tracking_started = False
    runtime.acquire(reason, interval_sec=interval, camera_id=camera_id, detector_backend_name=backend,
                    open_vocabulary_classes=(query,))
    try:
        with platform.motion.lease(MotionSource.GATEWAY_FOLLOW, args["motion_generation"]) as lease:
            bridge.bind(ctx.task_id, lease, check=ctx.check)
            try:
                while True:
                    ctx.check()
                    last_command = bridge.latest_command(ctx.task_id)
                    if last_command is not None and last_command.state == PERSON_FOLLOW_TRACKING:
                        tracking_started = True
                    now = time.monotonic()
                    elapsed = now - started
                    if elapsed >= duration:
                        if last_detection is not None and not tracking_started:
                            raise GatewayError("已发现目标，但原生跟随未能开始。", error_code="person_follow_not_started", http_status=409)
                        break
                    sample = runtime.latest(camera_id=camera_id, detector_backend_name=backend)
                    context = sample.context if sample is not None else None
                    stamp = _source_timestamp_ns(context) if context is not None else None
                    fresh = stamp is not None and (last_stamp is None or stamp > last_stamp)
                    detection = _best_detection(query, context) if fresh else None
                    if fresh:
                        last_stamp = stamp
                        if detection is not None:
                            if context.camera_info is None:
                                raise GatewayError("跟随观察缺少相机内参。", error_code="person_follow_calibration_unavailable", http_status=503)
                            camera_frame = context.camera_info.frame_id
                            if calibration is None:
                                calibration = _calibration(platform, ctx, camera_frame)
                                cloud_revision = _cloud_revision(platform)
                            elif camera_frame != calibration.child_frame:
                                raise GatewayError("跟随相机坐标系在任务中发生变化。", error_code="person_follow_calibration_unavailable", http_status=503)
                            last_detection = detection
                            lost_since = None
                        elif last_target is not None and lost_since is None:
                            lost_since = now
                        if last_target is not None or detection is not None:
                            target_seq += 1
                            last_target = build_person_follow_target(
                                session_id=ctx.task_id, query=query, target_seq=target_seq,
                                required_cloud_revision=cloud_revision, camera_parent_frame=calibration.parent_frame,
                                camera_translation=(calibration.x, calibration.y, calibration.z),
                                camera_rotation=(calibration.qx, calibration.qy, calibration.qz, calibration.qw),
                                perception_context=context, detection=detection,
                                tracking_state=PERSON_FOLLOW_TRACKED if detection is not None else PERSON_FOLLOW_LOST,
                            )
                            bridge.publish(last_target, check=ctx.check)
                    elif last_target is not None and lost_since is None:
                        lost_since = now
                    if last_target is None:
                        # 尚无实际目标，当前租约明确保持零速。
                        lease.velocity(0.0, 0.0, 0.0)
                    if lost_since is not None and now - lost_since >= lost_timeout:
                        raise GatewayError(f"目标 {query} 已丢失。", error_code="person_follow_target_lost", http_status=409)
                    stage = "follow_tracking" if last_command is not None and last_command.state == PERSON_FOLLOW_TRACKING else "follow_searching"
                    ctx.progress(min(0.99, elapsed / duration), stage,
                                 f"正在跟随目标 {query}。" if stage == "follow_tracking" else f"正在等待目标 {query} 的跟随结果。",
                                 last_detection=last_detection, last_native_command=asdict(last_command) if last_command is not None else None)
                    ctx.wait(min(interval, duration - elapsed))
            finally:
                bridge.end(ctx.task_id)
    finally:
        runtime.release(reason)
    return {"query": query, "duration_sec": duration, "last_detection": last_detection,
            "tracking_started": tracking_started, "last_native_command": asdict(last_command) if last_command is not None else None}


@rpc_errors
def follow_target(platform, ctx, args):
    try:
        return _run_follow_target(platform, ctx, args)
    except PersonFollowBridgeUnavailable as error:
        raise GatewayError(str(error), error_code="person_follow_unavailable", http_status=503) from error
    except PersonFollowBridgeError as error:
        raise GatewayError(str(error), error_code="person_follow_failed", http_status=409) from error


CAPABILITIES = [
    Capability("follow_target", "在指定时段消费统一感知结果跟随目标；目标丢失超时结束。",
        schema({"query": {"type": "string", "minLength": 1}, "camera_id": {"type": "string"},
                "detector_backend": {"type": "string"},
                "duration_sec": {"type": "number", "exclusiveMinimum": 0, "default": 2.0},
                "interval_sec": {"type": "number", "exclusiveMinimum": 0, "default": 0.1},
                "lost_timeout_sec": {"type": "number", "exclusiveMinimum": 0, "default": 0.8},
                "timeout_sec": {"type": "number", "exclusiveMinimum": 0}}, ("query",)),
        "task", follow_target, needs=("geometry", "person_follow"), motion=True, timeout_sec=900.0),
]
