"""感知能力与手动连续识别原因。"""
from contracts.events import RuntimeEventCategory
from core.capability import Capability
from core.errors import GatewayError
from gateways.capabilities.common import schema
from services.perception.image_utils import encode_image_frame_bytes


def capture_image(platform, args):
    context = platform.observation.capture_observation(camera_id=args.get("camera_id"), requested_by="invoke")
    artifact = context.image_artifact
    platform.events.publish(platform.events.build_event("artifact.created",
        category=RuntimeEventCategory.PERCEPTION, source="perception", subject_id=artifact.artifact_id,
        message="图像制品已写入宿主机。", payload={"artifact_id": artifact.artifact_id, "camera_id": context.camera_id}))
    return {"artifact": artifact, "observation_context": context, "camera_id": context.camera_id,
            "frame_id": context.observation.frame_id, "encoding": context.observation.metadata.get("encoding")}


def get_latest_observation(platform, args):
    context = platform.observation.get_latest_observation(args.get("camera_id"))
    if context is None:
        raise GatewayError("当前还没有可用的观察缓存。")
    return {"observation_context": context}


def _observe(platform, args, *, refresh=True):
    snapshot = platform.geometry_state.snapshot()
    name = snapshot.map_name if snapshot is not None else ("default" if platform.memory.current_library == "default" else "")
    context = platform.perception.describe_current_scene(camera_id=args.get("camera_id"),
        detector_backend_name=args.get("detector_backend"), refresh=refresh, requested_by="invoke")
    inputs = platform.perception.get_full_observation_input(context)
    if inputs is not None:
        image, camera_info, pose, tree = inputs
        image_bytes, _ = encode_image_frame_bytes(image)
        platform.perception.video_runtime.publish_sample(context, map_name=name, pose=pose,
            frame_tree=tree, image_bytes=image_bytes, inputs=(image, camera_info), complete=True)
    return context


def perceive_current_scene(platform, args):
    return {"perception_context": _observe(platform, args)}


def describe_current_scene(platform, args):
    context = _observe(platform, args, refresh=args.get("refresh", True))
    return {"summary": context.scene_summary.headline, "scene_summary": context.scene_summary,
            "perception_context": context}


def get_latest_perception(platform, args):
    context = platform.perception.get_latest_perception(args.get("camera_id"))
    if context is None:
        raise GatewayError("当前还没有可用的感知缓存。")
    return {"perception_context": context}


def get_perception_runtime_status(platform, args):
    return {"perception_runtime": platform.perception.video_runtime.get_status()}


def start_perception_runtime(platform, args):
    return {"perception_runtime": platform.perception.video_runtime.acquire("manual")}


def stop_perception_runtime(platform, args):
    return {"perception_runtime": platform.perception.video_runtime.release("manual")}


CAMERA = {"camera_id": {"type": "string"}}
DETECTION = {**CAMERA, "detector_backend": {"type": "string"}}
CAPABILITIES = [
    Capability("capture_image", "抓取单帧图像并生成观察与图像制品。", schema(CAMERA), "action", capture_image),
    Capability("get_latest_observation", "读取缓存的最新观察。", schema(CAMERA), "query", get_latest_observation),
    Capability("perceive_current_scene", "识别当前画面并生成场景摘要。", schema(DETECTION), "action", perceive_current_scene),
    Capability("describe_current_scene", "返回当前场景摘要与完整感知结果。",
        schema({**DETECTION, "refresh": {"type": "boolean", "default": True}}), "action", describe_current_scene),
    Capability("get_latest_perception", "读取缓存的最新识别结果。", schema(CAMERA), "query", get_latest_perception),
    Capability("get_perception_runtime_status", "读取连续识别状态及使用原因。", schema({}), "query", get_perception_runtime_status),
    Capability("start_perception_runtime", "申请手动连续识别原因。", schema({}), "action", start_perception_runtime),
    Capability("stop_perception_runtime", "释放手动连续识别原因。", schema({}), "action", stop_perception_runtime),
]
