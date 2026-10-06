"""普通点云地图保存：先完成内容，再分配一次不可变资产身份。"""

import logging
import os
import uuid
from datetime import datetime, timezone

from contracts.map_workspace import (
    AssetIdentityV2,
    MapAssetBundle,
    MapAssetStatus,
    MapGeoreferenceBindingV1,
    MapGeoreferenceV1,
)
from contracts.runtime_views import MapSnapshot
from core import StateNamespace
from core.errors import GatewayError

LOGGER = logging.getLogger(__name__)


def _problem(message, code="map_asset_finalize_failed"):
    return GatewayError(message, error_code=code, http_status=409, jsonrpc_code=-32000)


def catalog_owner(owner, asset):
    """名称不是创建身份；同时保留目录 inode 和原元数据的不可变拥有方。"""
    try:
        status = owner._asset_repository.current_directory(asset.map_name).parent.stat()
    except OSError as exc:
        raise _problem("地图目录创建拥有方已不可用。", "map_asset_owner_changed") from exc
    return (status.st_dev, status.st_ino, asset.created_at, asset.identity_schema_version,
            asset.asset_identity, asset.asset_manifest_sha256, asset.finalized)


def require_catalog_owner(owner, asset, captured):
    current = owner._repository.get_map_asset(asset.map_name)
    if current is None or catalog_owner(owner, current) != captured:
        raise _problem("地图目录拥有方已改变，不能采用旧保存结果。", "map_asset_owner_changed")
    return current


def reconcile_assigned_current(owner, asset, bundle, *, captured, reason=None, metadata=None,
                               confirm_durable=False, expected_previous_identity=None):
    """重启读取只采用已经持久的精确 current；不会重导出或重新分配 ID。"""
    if (bundle.identity_schema_version != 2 or bundle.asset_state != "finalized"
            or bundle.manifest_sha256 is not None):
        raise _problem("保存结果没有分配式 finalized 身份。")
    if confirm_durable or asset.identity_schema_version != 2:
        # 前次 rename 已可见但目录 fsync 失败时，显式保存重试需完成持久化。
        descriptor = os.open(owner._asset_repository.current_directory(asset.map_name).parent,
            os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    with owner._repository.map_activation_guard():
        current = require_catalog_owner(owner, asset, captured)
        if current.identity_schema_version == 2 and current.asset_identity != bundle.asset_identity:
            proof = owner._asset_repository.get_assigned_map_asset_exchange(asset.map_name)
            if (expected_previous_identity != current.asset_identity or proof is None or not proof.committed
                    or proof.previous_identity != current.asset_identity or proof.candidate_identity != bundle.asset_identity
                    or proof.catalog_created_at != current.created_at.isoformat()
                    or proof.map_directory != captured[:2]):
                raise _problem("分配式目录与 current 身份不一致。", "map_asset_bundle_invalid")
        if (current.identity_schema_version == 2 and current.finalized
                and current.asset_identity == bundle.asset_identity
                and current.frame_id == bundle.frame_id and current.finalized_at is not None):
            if confirm_durable:
                owner._repository.save_map_asset(current)
            return current
        now = datetime.now(timezone.utc)
        is_active = owner._repository.get_active_map_name() == asset.map_name
        updated = current.model_copy(update={
            "identity_schema_version": 2, "asset_identity": bundle.asset_identity,
            "asset_manifest_sha256": None, "finalized": True,
            "finalized_at": (current.finalized_at or bundle.timestamp)
                if current.asset_identity == bundle.asset_identity else bundle.timestamp,
            "frame_id": bundle.frame_id,
            "active": is_active, "localization_ready": False,
            "status": MapAssetStatus.MAPPING if is_active else MapAssetStatus.READY,
            "updated_at": now,
            "metadata": {**dict(current.metadata),
                **({"finalize_reason": str(reason or "").strip() or None} if reason is not None else {}),
                **(owner._sanitize_map_metadata(metadata) or {})},
        })
        owner._repository.save_map_asset(updated)
        published_owner = catalog_owner(owner, updated)
    # 持久化已完成。扩展 write/event 回调不可占用目录或运行态守卫。
    key = f"asset:{asset.map_name}"
    try:
        owner._state_store.write(StateNamespace.MAP, key, updated,
            source="map_workspace_service", metadata={"kind": "map_asset", "map_name": asset.map_name})
        require_catalog_owner(owner, updated, published_owner)
    except BaseException:
        owner._state_store.discard_if_value(StateNamespace.MAP, key, updated)
        raise
    return updated


def finalize_assigned_cloud_map(owner, asset, *, context, reason, metadata, georeference,
                                save_operation_id=None, expected_previous_identity=None):
    name = asset.map_name
    metadata = dict(owner._sanitize_map_metadata(metadata) or {})
    metadata.pop("save_operation_id", None)  # 仅私有逻辑保存入口可写入该关联字段。
    captured = catalog_owner(owner, asset)
    if asset.finalized and expected_previous_identity != asset.asset_identity:
        raise _problem("地图元数据标记为 finalized，但 current 资产包不存在。", "map_asset_bundle_missing")
    if context.get("active") is not True or context.get("map_name") != name:
        raise _problem("分配式保存要求目标地图的活动原生建图会话。")
    if georeference is not None and not isinstance(georeference, MapGeoreferenceV1):
        raise GatewayError("地图地理参考必须使用 MapGeoreferenceV1 强类型合同。",
            error_code="invalid_params", http_status=422, jsonrpc_code=-32602)
    coherent = owner._read_coherent_active_map_snapshot()
    if coherent is None or coherent[0] != name:
        raise _problem("当前没有属于激活地图的可发布地图快照。", "map_snapshot_unavailable")
    snapshot = coherent[1]
    frame = owner._resolve_map_frame_id(snapshot)
    if not frame:
        raise _problem("当前可发布地图快照缺少地图坐标系。", "map_snapshot_frame_unavailable")
    if (georeference is not None and (georeference.map_name != name
            or georeference.map_frame_id != frame
            or asset.frame_id is not None and asset.frame_id != frame)):
        raise _problem("地图地理参考与待发布地图身份不一致。", "map_georeference_identity_mismatch")
    snapshot_metadata = owner._sanitize_map_metadata(snapshot.metadata)
    snapshot_metadata["map_name"] = name
    values = {key: getattr(snapshot, key) for key in type(snapshot).model_fields}
    values["metadata"] = snapshot_metadata
    draft = MapAssetBundle(map_name=name, source_name=snapshot.source_name, frame_id=frame,
        snapshot=MapSnapshot(**values),
        georeference=MapGeoreferenceBindingV1(contract=georeference) if georeference is not None else None,
        metadata={"reason": str(reason or "").strip(),
            "mapping_session_id": snapshot_metadata.get("mapping_session_id") or snapshot_metadata.get("session_id"),
            "session_generation": snapshot_metadata.get("session_generation"),
            "content_revision": snapshot_metadata.get("content_revision"),
            **metadata,
            **({"save_operation_id": save_operation_id} if save_operation_id is not None else {})})
    staging = owner._asset_repository.create_staging_directory(name)
    try:
        prepared = owner._geometry_runtime.prepare_map_asset_bundle(
            draft, bundle_root=staging, checkpoint_schema_version=2)
        with owner._repository.map_activation_guard():
            require_catalog_owner(owner, asset, captured)
            owner._require_no_map_activation_recovery("finalize_active_map")
            if owner._repository.get_active_map_name() != name:
                raise _problem("保存准备期间活动地图已改变。", "map_asset_owner_changed")
        # 所有内容更新先完成；分配后的候选不可修改或重新生成身份。
        identity = AssetIdentityV2(asset_id=uuid.uuid4().hex)
        values = {key: getattr(prepared, key) for key in type(prepared).model_fields}
        values.update(identity_schema_version=2, asset_identity=identity,
                      asset_state="candidate", manifest_sha256=None)
        candidate = MapAssetBundle(**values)
        owner._asset_repository.finalize_assigned_map_asset_bundle(
            candidate, staging_dir=staging, asset_identity=identity,
            expected_previous_identity=expected_previous_identity)
        finalized = owner._asset_repository.get_map_asset_bundle(name, migrate_legacy=False,
            expected_asset_identity=identity)
        if finalized is None:
            raise _problem("提交后的精确 current 资产不可读取。")
        committed = reconcile_assigned_current(owner, asset, finalized, captured=captured,
            reason=reason, metadata=metadata, confirm_durable=True,
            expected_previous_identity=expected_previous_identity)
        if expected_previous_identity is not None:
            owner._asset_repository.settle_assigned_map_asset_exchange(name,
                expected_previous_identity=expected_previous_identity, asset_identity=identity)
    except BaseException as exc:
        preserve = getattr(exc, "preserve_staging", False)
        if expected_previous_identity is not None:
            try:
                preserve = preserve or owner._asset_repository.get_assigned_map_asset_exchange(name) is not None
            except Exception:
                preserve = True  # 无法证明交换位置时，资源继续归原事务所有。
        if (staging.exists() or staging.is_symlink()) and not preserve:
            try:
                owner._asset_repository.discard_staging_directory(name, staging)
            except BaseException:
                LOGGER.exception("无法清理原保存候选；保留原错误与替代目录。")
        if isinstance(exc, GatewayError) or not isinstance(exc, Exception):
            raise
        raise _problem(f"地图 {name} 资产包发布失败：{exc}") from exc
    committed_owner = catalog_owner(owner, committed)
    try:
        owner._publish_event("map.asset_finalized", f"地图 {name} 已发布 current 资产包。",
            payload={"map_name": name, "identity_schema_version": 2,
                     "asset_identity": identity.model_dump(), "checkpoint_id":
                         finalized.geometry_asset.metadata["mapping_backend_checkpoint"]["checkpoint_id"]})
        require_catalog_owner(owner, committed, committed_owner)
    except BaseException:
        owner._state_store.discard_if_value(StateNamespace.MAP, f"asset:{name}", committed)
        raise
    return finalized
