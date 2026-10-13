"""单层探索暂停和恢复接线；由原探索任务持有资源并完成存图或定位。"""
from __future__ import annotations

import json
import math
import uuid

from contracts.map_relocalization import AssignedRelocalizationRequest
from contracts.runtime_views import LocalizationSessionStatus
from services.geometry.mapping_backend_checkpoint_store import _identity_document
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
    frozen = request.model_copy(update={"metadata": {k: v for k, v in request.metadata.items() if k != "motion_admission"}, "resume_progress_json": None}, deep=True)
    catalog = store(runtime)
    try:
        retained = catalog.load(checkpoint_id, validate_asset=False)
    except FileNotFoundError:
        retained = None
    if retained is not None:
        if (retained.pipeline != runtime.config.geometry.pipeline or retained.request != frozen
                or retained.geometry_progress_json != payload):
            raise ValueError("原暂停检查点的请求或进度已改变，不能重写")
        catalog.save(retained)  # 原 created_at/字节和身份保持不变，并重新确认目录持久化。
        _publish_saved(runtime, context, retained)
        return
    seed = (runtime.platform_geometry_runtime.providers.get_exploration_pose()
            if runtime.config.geometry.pipeline == "slam_2d" else None)
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
    assigned = getattr(bundle, "identity_schema_version", 1) == 2
    binding = ({"schema_version": 2, "asset_identity": bundle.asset_identity,
                "mapping_checkpoint_id": bundle.geometry_asset.metadata["mapping_backend_checkpoint"]["checkpoint_id"]}
               if assigned else {"asset_manifest_sha256": bundle.manifest_sha256})
    value = ExplorationCheckpoint(checkpoint_id=checkpoint_id, pipeline=runtime.config.geometry.pipeline, request=frozen,
        geometry_progress_json=payload, localization_seed=seed, **binding)
    catalog.save(value)
    _publish_saved(runtime, context, value)


def _publish_saved(runtime, context, value):
    binding = ({"schema_version": 2, "asset_identity": value.asset_identity.model_dump(),
                "mapping_checkpoint_id": value.mapping_checkpoint_id}
               if value.schema_version == 2 else {"asset_manifest_sha256": value.asset_manifest_sha256})
    runtime.task_manager.update_task_metadata(context.task_id, {"exploration_checkpoint": {"status": "succeeded", "checkpoint_id": value.checkpoint_id,
        "map_name": value.request.map_name, **binding}})


def prepare_resume(runtime, context, request, checkpoint_id, requested_by):
    """激活准确资产并等待真实定位，再由原探索流程续建和派发新子目标。"""
    catalog = store(runtime)
    saved = catalog.load(checkpoint_id, validate_asset=False)
    if saved.pipeline != runtime.config.geometry.pipeline or saved.request.map_name != request.map_name or saved.geometry_progress_json != request.resume_progress_json:
        raise ValueError("探索恢复请求与检查点或当前链路不一致")
    snapshot = catalog.validate_asset(saved)
    if saved.schema_version == 2:
        return _prepare_assigned_resume(runtime, context, request, saved, snapshot, catalog, requested_by)
    context.ensure_active()
    context.update(stage="resume_exploration_localization", message="激活检查点地图并核对实际定位。")
    runtime.map_workspace_service.activate_map(map_name=request.map_name, create_if_missing=False,
        metadata={"source": "resume_exploration", "requested_by": requested_by or "unknown"})
    context.ensure_active()
    metadata = {"source": "resume_exploration", "business_request_id": "resume-localize-" + uuid.uuid4().hex,
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


def _pending_request(catalog, saved):
    pending = catalog.maps.get_relocalization_intent()
    if pending is None:
        return None, None
    if pending.get("schema_version") != 2:
        raise ValueError("另一个旧协议重定位请求仍待对账")
    request = AssignedRelocalizationRequest.from_dict(pending["assigned_request_v2"])
    if (request.asset_identity != saved.asset_identity or request.activation_receipt.map_name != saved.request.map_name
            or request.activation_receipt.to_dict()["request"]["checkpoint_id"] != saved.mapping_checkpoint_id):
        raise ValueError("未决重定位不属于该探索检查点")
    return request, pending["phase"]


def _resume_request(runtime, context, saved, pending):
    spec = runtime.task_manager.get_task_spec(context.task_id)
    retained = spec.metadata.get("resume_relocalization")
    binding = {"checkpoint_id": saved.checkpoint_id, "asset_identity": saved.asset_identity.model_dump(),
               "mapping_checkpoint_id": saved.mapping_checkpoint_id}
    if retained is None:
        remaining = context.timeout_remaining_sec()
        if pending is None and remaining is not None and remaining < 0.25:
            raise ValueError("剩余任务时间不足以提交原生重定位")
        retained = {**binding, "request_id": pending.request_id if pending is not None else "resume-localize-" + uuid.uuid4().hex,
            "logical_timeout_ms": pending.logical_timeout_ms if pending is not None
                else int(min(30.0, remaining if remaining is not None else 30.0) * 1000)}
        runtime.task_manager.update_task_metadata(context.task_id, {"resume_relocalization": retained})
    if (not isinstance(retained, dict) or set(retained) != set(binding) | {"request_id", "logical_timeout_ms"}
            or any(retained[key] != value for key, value in binding.items())
            or not isinstance(retained["request_id"], str) or not retained["request_id"]
            or type(retained["logical_timeout_ms"]) is not int or not 250 <= retained["logical_timeout_ms"] <= 30_000
            or pending is not None and (retained["request_id"] != pending.request_id
                or retained["logical_timeout_ms"] != pending.logical_timeout_ms)):
        raise ValueError("探索恢复的原重定位请求绑定已改变")
    if runtime.task_manager.get_task_spec(context.task_id).metadata.get("resume_relocalization") != retained:
        raise ValueError("探索恢复请求在任务元数据发布期间已改变")
    return retained


def _require_assigned_target(saved, snapshot, target):
    receipt = target.activation_receipt
    if (receipt.asset_identity != saved.asset_identity or receipt.map_name != saved.request.map_name
            or receipt.to_dict()["request"]["checkpoint_id"] != saved.mapping_checkpoint_id
            or snapshot.assigned_source_member is None
            or receipt.to_dict()["target"]["source_cloud_member"] != _identity_document(snapshot.assigned_source_member)):
        raise ValueError("当前安装的资产或源检查点与探索记录不一致")


def _prepare_assigned_resume(runtime, context, request, saved, snapshot, catalog, requested_by):
    service, geometry = runtime.map_workspace_service, runtime.platform_geometry_runtime
    context.ensure_active()
    context.update(stage="resume_exploration_localization", message="恢复准确 assigned 地图并核对真实定位。")
    pending, pending_phase = _pending_request(catalog, saved)
    retained = _resume_request(runtime, context, saved, pending)
    context.ensure_active()
    if pending is not None and pending_phase != "prepared":
        # 查询原操作，不能先激活另一个拥有方或换 UUID/timeout 重新执行。
        recovered = service.recover_relocalization_receipt()
        if recovered is None:
            raise ValueError("原重定位拥有方已退役，不能继续旧探索恢复")
        workspace = service.get_active_workspace(refresh_localization=False)
        session = service.get_active_localization_session()
    elif pending is None:
        service.activate_map(map_name=request.map_name, create_if_missing=False,
            metadata={"source": "resume_exploration", "requested_by": requested_by or "unknown"})
        workspace = session = None
    else:
        workspace = session = None
    context.ensure_active()
    target = geometry.capture_assigned_map_activation_target(map_name=request.map_name, asset_identity=saved.asset_identity)
    _require_assigned_target(saved, snapshot, target)
    if pending is not None and not target.activation_receipt.exact_replay_of(pending.activation_receipt):
        raise ValueError("已恢复重定位与当前安装不是同一拥有方")
    if pending is None or pending_phase == "prepared":
        session, workspace = service.relocalize_active_map(map_name=request.map_name,
            metadata={"source": "resume_exploration", "business_request_id": retained["request_id"],
                      "logical_timeout_ms": retained["logical_timeout_ms"]})
    context.ensure_active()
    token = service._localization_service.get_active_session_token()
    if (session is None or workspace is None or token[0] is not session
            or session.identity_schema_version != 2 or session.status != LocalizationSessionStatus.READY
            or not session.pose_available or session.asset_identity != saved.asset_identity
            or not session.activation_receipt_v2.exact_replay_of(target.activation_receipt)
            or session.relocalization_terminal_v2 is None or session.relocalization_terminal_v2.disposition not in {1, 2}
            or session.relocalization_terminal_v2.request.request_id != retained["request_id"]
            or session.relocalization_terminal_v2.request.logical_timeout_ms != retained["logical_timeout_ms"]
            or pending is not None and not session.relocalization_terminal_v2.request.exact_replay_of(pending)
            or not session.relocalization_terminal_v2.request.activation_receipt.exact_replay_of(target.activation_receipt)
            or workspace.active_map_name != request.map_name or not workspace.localization_ready
            or workspace.active_localization_session is not session or workspace.map_asset is None
            or workspace.map_asset.identity_schema_version != 2 or workspace.map_asset.asset_identity != saved.asset_identity):
        raise ValueError("探索恢复尚未获得同一安装上的真实 READY 会话")
    # 重定位之后只核对小型目录/文件观察，不重复读取整图。
    catalog._validate_catalog_identity(saved)
    if not catalog.assets.map_asset_bundle_snapshot_is_current(request.map_name, snapshot):
        raise ValueError("重定位期间探索检查点资产已被替换")
    pose = service.get_relocalized_pose(request.map_name)
    if pose is None or pose.frame_id != snapshot.bundle.frame_id:
        raise ValueError("探索恢复缺少当前原生拥有方位姿")
    context.ensure_active()
    with geometry.assigned_map_installation_guard(expected_target=target):
        actual, epoch = service._localization_service.get_active_session_token()
        if actual is not token[0] or epoch != token[1]:
            raise ValueError("探索恢复期间定位会话已被后继修改")
