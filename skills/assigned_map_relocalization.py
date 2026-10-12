"""分配式重定位编排：原请求、真实终态与当前原生位姿共同准入。"""

import logging
import uuid
from contextlib import contextmanager
from functools import wraps

from contracts.map_activation import AssignedActivationReceipt
from contracts.map_relocalization import AssignedRelocalizationRequest, AssignedRelocalizationTerminal
from contracts.map_workspace import MapAssetStatus
from contracts.runtime_views import LocalizationSessionStatus
from core.errors import GatewayError
from core.relocalization_request_id import relocalization_request_identity
from skills.assigned_map_activation import activate_assigned_workspace, build_assigned_workspace

LOGGER = logging.getLogger(__name__)


def _problem(message, request=None, code="relocalization_recovery_required"):
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


@contextmanager
def _transaction(owner):
    with owner._repository.map_activation_executor_guard(), owner._repository.map_activation_guard():
        yield


def _request_from_intent(pending):
    if pending.get("schema_version") != 2:
        raise _problem("旧重定位请求必须沿原协议完成对账。")
    return AssignedRelocalizationRequest.from_dict(pending["assigned_request_v2"])


def _terminal_from_receipt(receipt):
    if receipt is None or receipt.get("schema_version") != 2:
        return None
    return AssignedRelocalizationTerminal.from_dict(receipt["assigned_terminal_v2"])


def _capture_target(owner, request, *, restore):
    name = request.activation_receipt.map_name
    asset = owner._repository.get_map_asset(name)
    if (asset is None or asset.identity_schema_version != 2 or asset.asset_identity != request.asset_identity
            or owner._repository.get_active_map_name() != name):
        raise _problem("分配式重定位的目录拥有方已改变。", request, "relocalization_owner_changed")
    try:
        target = owner._geometry_runtime.capture_assigned_map_activation_target(
            map_name=name, asset_identity=request.asset_identity)
        owner._geometry_runtime.get_validated_assigned_map_activation_receipt(expected_target=target)
    except (ValueError, RuntimeError):
        if not restore:
            raise
        bundle = owner.get_map_asset_bundle(map_name=name)
        if bundle is None:
            raise _problem("恢复时找不到原 assigned current 资产。", request)
        # 只读当前原生封存证据；此入口不发送 StopMapping/ActivateMap。
        activate_assigned_workspace(owner, bundle, load_memory_history=True, allow_dispatch=False)
        target = owner._geometry_runtime.capture_assigned_map_activation_target(
            map_name=name, asset_identity=request.asset_identity)
        owner._geometry_runtime.get_validated_assigned_map_activation_receipt(expected_target=target)
    if not target.activation_receipt.exact_replay_of(request.activation_receipt):
        raise _problem("当前安装并非原重定位激活拥有方。", request, "relocalization_owner_changed")
    return target


def _owner_arguments(target):
    return {"activation_receipt": target.activation_receipt, "target_asset_identity": target.asset_identity}


def _expected_session(owner, target):
    session, epoch = owner._localization_service.get_active_session_token()
    document = target.activation_receipt.to_dict()
    source_key = "source_graph_member" if document["activation"]["backend_kind"] == "slam_toolbox" else "source_cloud_member"
    frame = document["target"][source_key]["frame_id"]
    if (session is None or session.identity_schema_version != 2 or session.frame_id != frame
            or not session.activation_receipt_v2.exact_replay_of(target.activation_receipt)):
        raise _problem("当前定位会话不属于原重定位安装。", code="relocalization_owner_changed")
    return session, epoch


def _require_replay_metadata(request, metadata):
    if "logical_timeout_ms" in metadata:
        candidate = request.to_dict()
        candidate[request.body_key]["logical_timeout_ms"] = metadata["logical_timeout_ms"]
        if not request.exact_replay_of(AssignedRelocalizationRequest.admit_retained(candidate)):
            raise _problem("相同重定位请求不能更换逻辑超时。", request, "relocalization_request_conflict")

    if metadata.get("planar_seed") is not None:
        seed = metadata["planar_seed"]
        if request.backend_kind != "slam_toolbox" or not isinstance(seed, (list, tuple)) or len(seed) != 3:
            raise _problem("二维初始位置与原重定位后端不符。", request, "relocalization_request_conflict")
        candidate = request.to_dict()
        candidate[request.body_key]["seed"] = dict(zip(("x", "y", "yaw"), seed))
        if not request.exact_replay_of(AssignedRelocalizationRequest.admit_retained(candidate)):
            raise _problem("相同重定位请求不能更换二维初始位置。", request, "relocalization_request_conflict")


def _set_pending_session(owner, target, request):
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
    with service.assigned_session_publication_scope() as publication, _transaction(owner):
        pending = owner._repository._require_assigned_relocalization_intent(request)
        if pending["phase"] != "prepared":
            raise _problem("原重定位准备阶段已由另一个执行者推进。", request)
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


def _finish(owner, request, terminal, target, *, metadata=None, expected_token=None):
    runtime, repository, service = owner._geometry_runtime, owner._repository, owner._localization_service
    with _transaction(owner):
        repository.reconcile_assigned_relocalization_known_terminal(
            relocalization_terminal=terminal, **_owner_arguments(target))
    # 必须在原生读取之前捕获 CAS token，不能在 I/O 后换成较新的会话来强行提交。
    expected, epoch = _expected_session(owner, target)
    if expected_token is not None and (expected is not expected_token[0] or epoch != expected_token[1]):
        raise _problem("原生重定位执行期间定位会话已改变。", request, "relocalization_owner_changed")
    sample = runtime.capture_assigned_localization_sample(assigned_request_v2=request,
        expected_target=target, expected_terminal=terminal)
    prepared = service.prepare_assigned_localization_update(sample=sample,
        expected_session=expected, expected_mutation_epoch=epoch, metadata=metadata,
        geometry_asset=expected.geometry_asset if expected else None, force_relocalize=True)
    with service.assigned_session_publication_scope() as publication, _transaction(owner):
        # Disk work remains outside R/provider/S/P; only the small session CAS is fenced there.
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
        repository.settle_assigned_relocalization_durable(
            assigned_request=request, **_owner_arguments(target))
    current, current_epoch = service.get_active_session_token()
    if current is not session or current_epoch != committed.committed_epoch:
        raise _problem("定位发布回调已替换当前会话。", request, "relocalization_owner_changed")
    asset = repository.get_map_asset(target.map_name)
    workspace = build_assigned_workspace(owner, asset) if asset is not None else None
    current, current_epoch = service.get_active_session_token()
    if (workspace is None or not workspace.localization_ready or current is not session
            or current_epoch != committed.committed_epoch):
        raise _problem("定位后的工作区已改变或尚未一致。", request, "relocalization_owner_changed")
    return session, workspace


def _query_terminal(owner, request):
    result = owner._geometry_runtime.get_assigned_relocalization_terminal(assigned_request_v2=request)
    if result.status != "terminal" or result.terminal is None:
        raise _problem("原生原请求终态尚未确定，不能重新执行。", request)
    if result.terminal.disposition not in {1, 2}:
        owner._repository.settle_assigned_relocalization_query(query=result.query, response=result.to_dict())
        raise _problem("原生原请求已拒绝或取消。", request, "relocalization_rejected")
    return result.terminal


@_public_boundary
def recover_assigned_relocalization(owner):
    pending = owner._repository.get_assigned_relocalization_intent()
    if pending is not None:
        request = _request_from_intent(pending)
        if pending["phase"] == "prepared":
            raise _problem("原请求尚未跨越 dispatch；恢复不会执行新的副作用。", request)
        terminal = _terminal_from_receipt(pending.get("terminal_receipt"))
        terminal = terminal or _query_terminal(owner, request)
    else:
        terminal = _terminal_from_receipt(owner._repository.get_last_successful_relocalization_receipt())
        if terminal is None:
            return None
        request = terminal.request
        active = owner._repository.get_last_successful_map_activation_receipt()
        if (active is None or active.get("schema_version") != 3
                or not AssignedActivationReceipt.admit(active["activation_receipt_v2"])
                    .exact_replay_of(request.activation_receipt)):
            return None  # 已完成的旧拥有方只是历史；不向新的安装重新应用。
    try:
        target = _capture_target(owner, request, restore=True)
    except GatewayError as exc:
        if pending is None or exc.error_code != "relocalization_owner_changed":
            raise
        active = owner._repository.get_last_successful_map_activation_receipt()
        if active is None or active.get("schema_version") != 3 or active.get("trusted_identity") is not True:
            raise
        replacement = AssignedActivationReceipt.admit(active["activation_receipt_v2"])
        if replacement.exact_replay_of(request.activation_receipt):
            raise
        query = owner._geometry_runtime.get_assigned_relocalization_terminal(assigned_request_v2=request)
        owner._repository.settle_assigned_relocalization_query(query=query.query,
            response=query.to_dict(), replacement_activation=replacement)
        return None
    return _finish(owner, request, terminal, target)


@_public_boundary
def relocalize_assigned_workspace(owner, *, map_name, metadata=None):
    repository, runtime = owner._repository, owner._geometry_runtime
    metadata = owner._sanitize_relocalization_request_metadata(owner._sanitize_map_metadata(metadata) or {})
    supplied_id, recovery_only = relocalization_request_identity(metadata)
    pending = repository.get_assigned_relocalization_intent()
    if pending is not None:
        request = _request_from_intent(pending)
        _require_replay_metadata(request, metadata)
        if (request.activation_receipt.map_name != map_name
                or supplied_id is not None and supplied_id != request.request_id):
            raise _problem("另一个原重定位请求仍待对账。", request)
        if pending["phase"] != "prepared":
            recovered = recover_assigned_relocalization(owner)
            if recovered is None:
                raise _problem("原重定位拥有方已被真实封存的后继取代。", request, "relocalization_owner_changed")
            return recovered
        if recovery_only:
            raise _problem("原请求尚未 dispatch；旧 request_id 恢复入口不会开始执行。", request)
        target = _capture_target(owner, request, restore=False)
    else:
        asset = repository.get_map_asset(map_name)
        if asset is None or asset.identity_schema_version != 2 or repository.get_active_map_name() != map_name:
            raise _problem("重定位前必须加载当前分配式地图。")
        target = runtime.capture_assigned_map_activation_target(map_name=map_name, asset_identity=asset.asset_identity)
        previous = _terminal_from_receipt(repository.get_last_successful_relocalization_receipt())
        if previous is not None and supplied_id == previous.request_id:
            request = previous.request
            _require_replay_metadata(request, metadata)
            if not request.activation_receipt.exact_replay_of(target.activation_receipt):
                raise _problem("保留请求不属于当前安装。", request, "relocalization_owner_changed")
        else:
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
                "request_id": supplied_id if supplied_id is not None else uuid.uuid4().hex,
                "assigned_planar_v2" if planar else "assigned_cloud_v2": body})
        current_receipt = previous is not None and supplied_id == previous.request_id
        if recovery_only and current_receipt:
            return _finish(owner, request, previous, target, metadata=metadata)
        if supplied_id is not None and not current_receipt:
            # An explicit ID may name an older operation even when the catalog
            # only retains a newer receipt. Query before preparing any state.
            result = runtime.get_assigned_relocalization_terminal(assigned_request_v2=request)
            if result.status == "terminal" and result.terminal is not None:
                raise GatewayError("原请求已有历史终态；仅返回查询结果，不重新采用定位。",
                    error_code="relocalization_historical_terminal", http_status=409, jsonrpc_code=-32000,
                    details={"request_id": request.request_id, "terminal": result.terminal.to_dict()})
            if result.status != "not_found" or recovery_only:
                code = ("relocalization_request_conflict" if result.status == "identity_conflict"
                        else "relocalization_request_not_found" if result.status == "not_found"
                        else "relocalization_recovery_required")
                raise _problem("未确认此编号可新建；保留现状，不创建或重新执行请求。", request, code)
        with _transaction(owner):
            pending = repository.begin_assigned_relocalization_durable(
                assigned_request=request, **_owner_arguments(target))
        if pending["phase"] != "prepared":
            return recover_assigned_relocalization(owner)
    expected_token = _set_pending_session(owner, target, request)
    with _transaction(owner):
        repository.mark_assigned_relocalization_backend_dispatching(
            assigned_request=request, **_owner_arguments(target))
    try:
        terminal = runtime.relocalize_assigned_active_map(assigned_request_v2=request, expected_target=target)
    except BaseException as exc:
        try:
            repository.mark_assigned_relocalization_outcome_unknown(
                assigned_request=request, **_owner_arguments(target))
        except BaseException:
            LOGGER.exception("原重定位 dispatch 日志仍保留，无法追加 outcome_unknown。")
        if not isinstance(exc, Exception):
            raise
        raise _problem("原生重定位回复未知；保留原请求等待查询。", request, "relocalization_outcome_unknown") from exc
    if terminal.disposition not in {1, 2}:
        repository.abort_assigned_relocalization_before_apply(assigned_request=request,
            terminal=terminal, **_owner_arguments(target))
        raise _problem("原生重定位未应用。", request, "relocalization_rejected")
    return _finish(owner, request, terminal, target, metadata=metadata, expected_token=expected_token)


def get_assigned_relocalized_pose(owner, map_name):
    if owner._repository.get_relocalization_intent() is not None:
        return None
    terminal = _terminal_from_receipt(owner._repository.get_last_successful_relocalization_receipt())
    if terminal is None or terminal.request.activation_receipt.map_name != map_name:
        return None
    request = terminal.request
    try:
        target = _capture_target(owner, request, restore=False)
        expected, epoch = owner._localization_service.get_active_session_token()
        if (expected is None or expected.status != LocalizationSessionStatus.READY
                or expected.relocalization_terminal_v2 is None
                or not expected.relocalization_terminal_v2.exact_replay_of(terminal)):
            return None
        sample = owner._geometry_runtime.capture_assigned_localization_sample(
            assigned_request_v2=request, expected_target=target, expected_terminal=terminal)
        if owner._repository.get_relocalization_intent() is not None:
            return None
        with owner._geometry_runtime._assigned_relocalization_result_guard(terminal,
                assigned_request_v2=request, expected_target=target, localization_sample=sample):
            current, current_epoch = owner._localization_service.get_active_session_token()
            if current is not expected or current_epoch != epoch:
                return None
        return sample.current_pose
    except (ValueError, RuntimeError, GatewayError):
        return None
