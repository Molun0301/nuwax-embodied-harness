"""用统一感知循环发布的同帧数据写入记忆。"""
from core.errors import GatewayError
from contracts.geometry import Pose


def _observation_inputs(platform, observation):
    value = observation.metadata["observation_pose"]
    pose = Pose.model_validate(value) if value is not None else None
    image = observation.image_artifact
    return {"map_name": observation.metadata["map_name"], "observation_pose": pose,
            "observation_context": observation,
            "image_bytes": platform.artifacts.read_bytes(image.artifact_id) if image is not None else None}


def remember_sample(platform, sample, *, title=None, summary=None, tags=None, metadata=None):
    """显式传递观察时的地图、位姿、坐标树和图片，不重新采集。"""
    sample = platform.perception.video_runtime.complete_sample(sample)
    return platform.memory.remember_current_scene(map_name=sample.map_name,
        title=title, camera_id=sample.context.camera_id, summary_override=summary,
        tags=tags, metadata=metadata, perception_context=sample.context,
        observation_pose=sample.pose, frame_tree=sample.frame_tree, image_bytes=sample.image_bytes)


def save_sample_photo(platform, sample):
    """复核可读取这一帧图片；业务观察在复核后写入。"""
    artifact = sample.context.image_artifact
    if artifact is not None and sample.image_bytes is not None:
        return platform.memory.save_photo(artifact.artifact_id, sample.image_bytes,
                                         map_name=sample.map_name, mime_type=artifact.mime_type)
    return None


def remember_current_scene(platform, args):
    sample = platform.perception.video_runtime.latest(camera_id=args.get("camera_id"))
    if sample is None:
        observation = platform.observation.get_latest_observation(args.get("camera_id"))
        if observation is None:
            raise GatewayError("当前没有可写入记忆的观察或感知结果", error_code="observation_unavailable", http_status=409)
        return {"semantic_memory": platform.memory.remember_current_scene(
            **_observation_inputs(platform, observation), title=args.get("title"), camera_id=observation.camera_id,
            summary_override=args.get("summary"), metadata={"source": "remember_current_scene"})}
    return {"semantic_memory": remember_sample(platform, sample, title=args.get("title"),
        summary=args.get("summary"), metadata={"source": "remember_current_scene"})}


def tag_location(platform, args):
    sample = platform.perception.video_runtime.latest(camera_id=args.get("camera_id"))
    if sample is None:
        observation = platform.observation.get_latest_observation(args.get("camera_id"))
        if observation is not None:
            inputs = _observation_inputs(platform, observation)
        else:
            state = platform.geometry_state.snapshot()
            inputs = {"map_name": state.map_name if state is not None else "",
                      "observation_pose": platform.geometry_state.pose(), "attach_latest_observation": False}
        values = platform.memory.tag_location(args["name"], **inputs, aliases=args.get("aliases"),
            description=args.get("description"), camera_id=args.get("camera_id"),
            auto_create_memory=args.get("auto_create_memory", True), attach_latest_perception=False,
            metadata={"source": "tag_location"})
    else:
        sample = platform.perception.video_runtime.complete_sample(sample)
        values = platform.memory.tag_location(args["name"], map_name=sample.map_name,
            aliases=args.get("aliases"), description=args.get("description"), camera_id=sample.context.camera_id,
            auto_create_memory=args.get("auto_create_memory", True), perception_context=sample.context,
            observation_pose=sample.pose, frame_tree=sample.frame_tree, image_bytes=sample.image_bytes,
            metadata={"source": "tag_location"})
    return {"tagged_location": values[0], "semantic_memory": values[1]}
