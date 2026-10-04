"""单层探索暂停和恢复接线；由原探索任务持有资源并完成存图或定位。"""
from __future__ import annotations

import json
import math
import uuid

from services.navigation.exploration_checkpoint import ExplorationCheckpoint, ExplorationCheckpointStore


def store(runtime):
    return ExplorationCheckpointStore(runtime.config.runtime_data.map_catalog_root)


def request_pause(runtime, request_id):
    """只请求原任务中断；存图结果由原任务元数据单独报告。"""
    for status in runtime.task_manager.list_active_tasks():
        spec = runtime.task_manager.get_task_spec(status.task_id)
        if spec.capability_name not in {"explore_area", "explore_area_v2"} or not runtime._task_matches_exploration_request(status.task_id, request_id):
            continue
        checkpoint_id = spec.metadata.get("pause_checkpoint_id") or "exploration-" + uuid.uuid4().hex
        runtime.task_manager.update_task_metadata(status.task_id, {"pause_checkpoint_id": checkpoint_id, "exploration_checkpoint": {"status": "pending", "checkpoint_id": checkpoint_id}})
        if not runtime.task_manager.cancel_task(status.task_id, "保存检查点并暂停探索"):
            runtime.task_manager.update_task_metadata(status.task_id, {"exploration_checkpoint": {"status": "rejected", "checkpoint_id": checkpoint_id, "message": "任务已进入不可中断终态"}})
            raise ValueError("探索已经进入不可中断终态，未受理暂停")
        return {"task_id": status.task_id, "request_id": request_id, "checkpoint_id": checkpoint_id, "checkpoint_pending": True}
    raise ValueError("没有匹配的活动单层探索任务")


def save_after_stop(runtime, context, request):
    """调用者已沿原停车屏障确认完成，仍持有原任务资源。"""
    spec = runtime.task_manager.get_task_spec(context.task_id)
    checkpoint_id = spec.metadata.get("pause_checkpoint_id")
    if not checkpoint_id:
        return
    payload = runtime.platform_geometry_runtime.get_exploration_progress(request.request_id)
    before = json.loads(payload)
    seed = runtime.platform_geometry_runtime.providers.get_exploration_pose()
    result = runtime._finalize_workspace_assets_after_task(map_name=request.map_name, trigger="explore_pause_checkpoint",
        exploration_request_id=request.request_id, task_id=context.task_id)
    bundle = result.get("finalized_map_asset")
    if result.get("map_asset_finalize_error") or bundle is None:
        raise ValueError("探索已停稳，但本次完整地图保存未成功，不能发布检查点")
    after = json.loads(runtime.platform_geometry_runtime.get_exploration_progress(request.request_id))
    version = before.get("schema_version")
    if type(version) is not int or version not in (1, 2) or type(after.get("schema_version")) is not int:
        raise ValueError("探索进度版本无效，检查点未发布")
    identity = ("schema_version", "request_id", "map_name", "map_frame_id", "origin_session_id", "origin_session_generation", "origin_frame_generation")
    # New progress compares actual settings; v1 values remain opaque legacy metadata.
    settings = ("configuration", "navigation_configuration") if version == 2 else ("configuration_sha256", "navigation_configuration_sha256")
    if any(before[key] != after[key] for key in identity + settings):
        raise ValueError("完整地图保存期间探索坐标或配置改变，检查点未发布")
    frozen = request.model_copy(update={"metadata": {k: v for k, v in request.metadata.items() if k != "motion_admission"}, "resume_progress_json": None}, deep=True)
    value = ExplorationCheckpoint(checkpoint_id=checkpoint_id, pipeline=runtime.config.geometry.pipeline, request=frozen,
        asset_manifest_sha256=bundle.manifest_sha256, geometry_progress_json=payload, localization_seed=seed)
    store(runtime).save(value)
    runtime.task_manager.update_task_metadata(context.task_id, {"exploration_checkpoint": {"status": "succeeded", "checkpoint_id": checkpoint_id,
        "map_name": request.map_name, "asset_manifest_sha256": bundle.manifest_sha256}})


def prepare_resume(runtime, context, request, checkpoint_id, requested_by):
    """激活准确资产并等待真实定位，再由原探索流程续建和派发新子目标。"""
    catalog = store(runtime)
    saved = catalog.load(checkpoint_id)
    if saved.pipeline != runtime.config.geometry.pipeline or saved.request.map_name != request.map_name or saved.geometry_progress_json != request.resume_progress_json:
        raise ValueError("探索恢复请求与检查点或当前链路不一致")
    context.ensure_active()
    context.update(stage="resume_exploration_localization", message="激活检查点地图并核对实际定位。")
    runtime.map_workspace_service.activate_map(map_name=request.map_name, create_if_missing=False,
        metadata={"source": "resume_exploration", "requested_by": requested_by or "unknown"})
    context.ensure_active()
    metadata = {"source": "resume_exploration", "request_id": "resume-localize-" + uuid.uuid4().hex,
        "logical_timeout_ms": int(min(30.0, context.timeout_remaining_sec() or 30.0) * 1000)}
    if saved.pipeline == "slam_2d":
        pose = saved.localization_seed
        if pose is None:
            raise ValueError("二维探索检查点缺少实际定位初始提示")
        q = pose.orientation
        metadata["planar_seed"] = [pose.position.x, pose.position.y, math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))]
    session, workspace = runtime.map_workspace_service.relocalize_active_map(map_name=request.map_name, metadata=metadata)
    context.ensure_active()
    catalog.validate_asset(saved)
    status = getattr(session.status, "value", session.status)
    if status != "ready" or workspace.active_map_name != request.map_name or workspace.map_asset.asset_manifest_sha256 != saved.asset_manifest_sha256:
        raise ValueError("探索检查点尚未获得准确资产上的真实定位")
