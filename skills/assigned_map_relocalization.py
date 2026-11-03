"""分配式重定位编排：原请求、真实终态与当前原生位姿共同准入。"""

import uuid
from functools import wraps

from contracts.map_relocalization import AssignedRelocalizationRequest
from contracts.map_workspace import MapAssetStatus
from contracts.runtime_views import LocalizationSessionStatus
from core.errors import GatewayError
from skills.assigned_map_activation import build_assigned_workspace


def _problem(message, request=None, code="relocalization_failed"):
    details = ({"request_id": request.request_id, "map_name": request.activation_receipt.map_name,
        "target_asset_identity": request.asset_identity.model_dump()} if request is not None else {})
    return GatewayError(message, error_code=code, http_status=409, jsonrpc_code=-32000, details=details)


def _public_boundary(method):
    @wraps(method)
    def call(*args, **kwargs):
        try:
            return method(*args, **kwargs)
        except GatewayError:
            raise
        except Exception as exc:
            raise _problem(f"分配式重定位尚未完成一致采用：{exc}") from exc
    return call


def _capture_target(owner, request):
    name = request.activation_receipt.map_name
    asset = owner._repository.get_map_asset(name)
    if (asset is None or asset.identity_schema_version != 2 or asset.asset_identity != request.asset_identity
            or owner._repository.get_active_map_name() != name):
        raise _problem("分配式重定位的目录拥有方已改变。", request, "relocalization_owner_changed")
    target = owner._geometry_runtime.capture_assigned_map_activation_target(
        map_name=name, asset_identity=request.asset_identity)
    owner._geometry_runtime.get_validated_assigned_map_activation_receipt(expected_target=target)
    if not target.activation_receipt.exact_replay_of(request.activation_receipt):
        raise _problem("当前安装并非原重定位激活拥有方。", request, "relocalization_owner_changed")
    return target


def _expected_session(owner, target):
    session, epoch = owner._localization_service.get_active_session_token()
    document = target.activation_receipt.to_dict()
    source_key = "source_graph_member" if document["activation"]["backend_kind"] == "slam_toolbox" else "source_cloud_member"
    frame = document["target"][source_key]["frame_id"]
    if (session is None or session.identity_schema_version != 2 or session.frame_id != frame
            or not session.activation_receipt_v2.exact_replay_of(target.activation_receipt)):
        raise _problem("当前定位会话不属于原重定位安装。", code="relocalization_owner_changed")
    return session, epoch


def _set_pending_session(owner, target, request, publication):
    service = owner._localization_service
    expected, epoch = _expected_session(owner, target)
    if (expected is not None and expected.identity_schema_version == 2
            and expected.activation_receipt_v2.exact_replay_of(target.activation_receipt)
            and expected.status == LocalizationSessionStatus.LOCALIZING
            and expected.relocalization_terminal_v2 is None):
        return expected, epoch
    prepared = service.prepare_assigned_session_update(expected_session=expected, expected_mutation_epoch=epoch,
        activation_receipt_v2=target.activation_receipt, frame_id=expected.frame_id if expected else None,
        geometry_asset=expected.geometry_asset if expected else None, refresh=False)
    with owner._repository.map_activation_guard():
        with owner._geometry_runtime.assigned_map_installation_guard(expected_target=target) as receipt:
            committed = service._compare_and_enqueue_prepared_assigned_session(prepared,
                guarded_activation_receipt_v2=receipt, guarded_relocalization_terminal_v2=None,
                publication_scope=publication)
            if not committed.accepted:
                raise _problem("重定位准备期间定位会话已改变。")
    current, current_epoch = service.get_active_session_token()
    if current is not committed.session or current_epoch != committed.committed_epoch:
        raise _problem("重定位准备发布已被后继修改。", code="relocalization_owner_changed")
    return committed.session, committed.committed_epoch


def _finish(owner, request, terminal, target, *, publication, metadata=None, expected_token=None):
    runtime, repository, service = owner._geometry_runtime, owner._repository, owner._localization_service
    # 必须在原生读取之前捕获 CAS token，不能在 I/O 后换成较新的会话来强行提交。
    expected, epoch = _expected_session(owner, target)
    if expected_token is not None and (expected is not expected_token[0] or epoch != expected_token[1]):
        raise _problem("原生重定位执行期间定位会话已改变。", request, "relocalization_owner_changed")
    sample = runtime.capture_assigned_localization_sample(assigned_request_v2=request,
        expected_target=target, expected_terminal=terminal)
    prepared = service.prepare_assigned_localization_update(sample=sample,
        expected_session=expected, expected_mutation_epoch=epoch, metadata=metadata,
        geometry_asset=expected.geometry_asset if expected else None, force_relocalize=True)
    with repository.map_activation_guard():
        # 磁盘写入在运行状态与会话守卫之外完成。
        with runtime._assigned_relocalization_result_guard(terminal, assigned_request_v2=request,
                expected_target=target, localization_sample=sample):
            committed = service._compare_and_enqueue_prepared_assigned_session(prepared,
                guarded_activation_receipt_v2=target.activation_receipt,
                guarded_relocalization_terminal_v2=sample.terminal,
                guarded_localization_sample=sample, publication_scope=publication)
            if not committed.accepted:
                raise _problem("位姿读取期间定位会话已被后继修改。", request, "relocalization_owner_changed")
        session = committed.session
        if session.status != LocalizationSessionStatus.READY or not session.pose_available:
            raise _problem("真实原生位姿尚未满足定位就绪条件。", request)
        asset = repository.get_map_asset(target.map_name)
        if asset is None or asset.asset_identity != target.asset_identity:
            raise _problem("定位采用期间目录拥有方已改变。", request, "relocalization_owner_changed")
        repository.save_map_asset(asset.model_copy(update={"status": MapAssetStatus.READY,
            "localization_ready": True}))
    return session, committed.committed_epoch


@_public_boundary
def relocalize_assigned_workspace(owner, *, map_name, metadata=None):
    repository, runtime, service = owner._repository, owner._geometry_runtime, owner._localization_service
    metadata = owner._sanitize_relocalization_request_metadata(owner._sanitize_map_metadata(metadata) or {})
    with service.assigned_session_publication_scope() as publication, repository.map_activation_executor_guard():
        asset = repository.get_map_asset(map_name)
        if asset is None or asset.identity_schema_version != 2 or repository.get_active_map_name() != map_name:
            raise _problem("重定位前必须加载当前分配式地图。")
        target = runtime.capture_assigned_map_activation_target(map_name=map_name, asset_identity=asset.asset_identity)
        activation = target.activation_receipt.to_dict()
        planar = activation["activation"]["backend_kind"] == "slam_toolbox"
        body = {"target": activation["target"], "policy": target.relocalization_policy,
                "logical_timeout_ms": metadata.get("logical_timeout_ms", 30_000)}
        if planar:
            seed = runtime.resolve_planar_relocalization_seed(metadata.get("planar_seed"))
            body["seed"] = dict(zip(("x", "y", "yaw"), seed))
        elif metadata.get("planar_seed") is not None:
            raise _problem("分配式点云重定位不接受二维初始位置。", code="invalid_params")
        request = AssignedRelocalizationRequest.admit({"schema_version": 2,
            "request_id": uuid.uuid4().hex, "assigned_planar_v2" if planar else "assigned_cloud_v2": body})
        expected_token = _set_pending_session(owner, target, request, publication)
        terminal = runtime.relocalize_assigned_active_map(assigned_request_v2=request, expected_target=target)
        if terminal.disposition not in {1, 2}:
            raise _problem("原生重定位未应用。", request, "relocalization_rejected")
        session, committed_epoch = _finish(owner, request, terminal, target, publication=publication,
            metadata=metadata, expected_token=expected_token)
    current, current_epoch = service.get_active_session_token()
    if current is not session or current_epoch != committed_epoch:
        raise _problem("定位发布回调已替换当前会话。", request, "relocalization_owner_changed")
    workspace = build_assigned_workspace(owner, repository.get_map_asset(map_name))
    current, current_epoch = service.get_active_session_token()
    if (workspace is None or not workspace.localization_ready or current is not session
            or current_epoch != committed_epoch):
        raise _problem("定位后的工作区已改变或尚未一致。", request, "relocalization_owner_changed")
    return session, workspace


def get_assigned_relocalized_pose(owner, map_name):
    session, epoch = owner._localization_service.get_active_session_token()
    if (session is None or session.status != LocalizationSessionStatus.READY
            or session.relocalization_terminal_v2 is None):
        return None
    terminal = session.relocalization_terminal_v2
    request = terminal.request
    if request.activation_receipt.map_name != map_name:
        return None
    try:
        target = _capture_target(owner, request)
        sample = owner._geometry_runtime.capture_assigned_localization_sample(
            assigned_request_v2=request, expected_target=target, expected_terminal=terminal)
        with owner._geometry_runtime._assigned_relocalization_result_guard(terminal,
                assigned_request_v2=request, expected_target=target, localization_sample=sample):
            current, current_epoch = owner._localization_service.get_active_session_token()
            if current is not session or current_epoch != epoch:
                return None
        return sample.current_pose
    except (ValueError, RuntimeError, GatewayError):
        return None
