"""手动记忆采集当前画面；组合任务写入其实际识别帧。"""


def _current_observation_inputs(platform, args):
    observation = platform.observation.capture_observation(
        camera_id=args.get("camera_id"), requested_by="memory_command")
    image = observation.image_artifact
    return {"map_name": observation.metadata["map_name"], "observation_pose": platform.geometry_state.pose(),
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
    inputs = _current_observation_inputs(platform, args)
    return {"semantic_memory": platform.memory.remember_current_scene(
        **inputs, title=args.get("title"), camera_id=inputs["observation_context"].camera_id,
        summary_override=args.get("summary"), metadata={"source": "remember_current_scene"})}


def tag_location(platform, args):
    inputs = _current_observation_inputs(platform, args)
    values = platform.memory.tag_location(args["name"], **inputs, aliases=args.get("aliases"),
        description=args.get("description"), camera_id=inputs["observation_context"].camera_id,
        auto_create_memory=args.get("auto_create_memory", True), attach_latest_perception=False,
        attach_latest_observation=False, metadata={"source": "tag_location"})
    return {"tagged_location": values[0], "semantic_memory": values[1]}
