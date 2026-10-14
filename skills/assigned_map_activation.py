"""分配式工作区激活编排：只消费原请求和真实原生回执。"""
import logging
import uuid
from contextlib import contextmanager
from datetime import datetime, timezone

from contracts.map_activation import AssignedActivationReceipt, _request
from contracts.map_workspace import MapAssetStatus, MapWorkspace
from contracts.runtime_views import LocalizationSessionStatus
from core import StateNamespace
from core.errors import GatewayError

LOGGER = logging.getLogger(__name__)

_PREDECESSOR_FIELDS = ('session_id', 'map_name', 'session_generation', 'runtime_generation', 'mapping_backend_kind')


def _problem(message, journal=None):
    return GatewayError(message, error_code='map_activation_recovery_required', http_status=409,
        jsonrpc_code=-32000, details={**(journal or {}), 'recommended_next_tool': 'activate_map'})


@contextmanager
def _transaction(owner):
    # 只包围短日志/元数据提交；不包含 RPC、地图读取和发布回调。
    with owner._repository.map_activation_executor_guard(), owner._repository.map_activation_guard():
        yield


def _identity(prepared, request):
    return {'target_map_name': prepared.map_name, 'target_asset_identity': prepared.asset_identity,
                'target_checkpoint_id': prepared.checkpoint_id,
                'activation_request_id': request['activation_request_id'], 'activation_request': request}


def _require_journal(owner, prepared, request):
    journal = owner._repository.get_map_activation()
    if (journal is None or journal.get('schema_version') != 3
            or journal.get('activation_request') != request
            or journal.get('target_asset_identity') != prepared.asset_identity.model_dump()
            or journal.get('target_checkpoint_id') != prepared.checkpoint_id):
        raise _problem('分配式地图激活事务已改变，不能采用历史结果。', journal)
    return journal


def _advance(owner, prepared, request, phase, expected):
    with _transaction(owner):
        # 同阶段也重新确认目录 fsync：上次 rename 可见不代表失败后的持久化已完成。
        return owner._repository.update_assigned_map_activation_phase(
            phase, **_identity(prepared, request), expected_phases=tuple(set(expected) | {phase}))


def _target_request(state, prepared):
    if (state.get('activation_receipt_schema_version') != 2
            or state.get('activation_receipt_disposition') not in (2, 3)):
        return None
    receipt = AssignedActivationReceipt.admit(state.get('activation_receipt_v2'))
    if (receipt.asset_identity != prepared.asset_identity or receipt.map_name != prepared.map_name
            or receipt.checkpoint_id != prepared.checkpoint_id):
        return None
    return receipt.to_dict()['request']


def _mapping_matches(snapshot, target):
    if snapshot is None or snapshot.map_name != target.map_name:
        return False
    metadata = snapshot.metadata
    if (metadata.get('identity_schema_version') != 2
            or metadata.get('asset_identity') != target.asset_identity.model_dump()
            or metadata.get('map_activation_generation') != target.installation_generation):
        return False
    try:
        receipt = AssignedActivationReceipt.admit(metadata.get('activation_receipt_v2'))
    except (ValueError, TypeError):
        return False
    return receipt.exact_replay_of(target.activation_receipt)


def _session_matches(session, target):
    return (session is not None and session.identity_schema_version == 2
            and session.map_name == target.map_name and session.asset_identity == target.asset_identity
            and session.activation_receipt_v2.exact_replay_of(target.activation_receipt))


def current_components(owner, asset, *, allow_pending=False):
    """只读标量/已拥有对象；不刷新 provider，不执行旧 session CAS。"""
    if not asset.active or asset.identity_schema_version != 2:
        return None
    pending = owner._repository.get_map_activation()
    if not allow_pending and (pending is not None or owner._geometry_runtime.is_map_asset_activation_recovery_pending()):
        return None
    try:
        target = owner._geometry_runtime.capture_assigned_map_activation_target(
            map_name=asset.map_name, asset_identity=asset.asset_identity)
        snapshot = owner._mapping_service.get_latest_snapshot()
        if not _mapping_matches(snapshot, target):
            return None
        with owner._geometry_runtime.assigned_map_installation_guard(expected_target=target):
            session, epoch = owner._localization_service.get_active_session_token()
            if not _session_matches(session, target):
                return None
        return target, snapshot, session, epoch
    except (ValueError, RuntimeError):
        return None


def assigned_asset_status(owner, asset, *, is_active):
    pending = owner._repository.get_map_activation()
    recovering = pending is not None and pending.get('target_map_name') == asset.map_name
    values = current_components(owner, asset.model_copy(update={'active': is_active}), allow_pending=False)
    ready = bool(values and values[2].status == LocalizationSessionStatus.READY
                 and values[2].relocalization_terminal_v2 is not None
                 and owner._repository.get_relocalization_intent() is None)
    metadata = dict(asset.metadata)
    metadata.pop('activation_target_manifest_sha256', None)
    metadata.update(activation_recovery_pending=recovering)
    if recovering:
        metadata.update(activation_phase=pending['phase'],
                        activation_target_asset_identity=pending.get('target_asset_identity'))
    return asset.model_copy(update={'active': is_active, 'localization_ready': ready,
        'status': MapAssetStatus.ACTIVATING if recovering else MapAssetStatus.READY if ready or not is_active
                  else MapAssetStatus.LOCALIZING,
        'metadata': metadata})


def build_assigned_workspace(owner, asset, *, publish=True):
    values = current_components(owner, asset)
    if values is None:
        return None
    target, snapshot, session, epoch = values
    ready = (session.status == LocalizationSessionStatus.READY and session.relocalization_terminal_v2 is not None
             and owner._repository.get_relocalization_intent() is None)
    workspace = MapWorkspace(active_map_name=asset.map_name,
        active_memory_library_name=owner._memory_service.get_active_library_name(),
        map_asset=assigned_asset_status(owner, asset, is_active=True), map_loaded=True,
        localization_ready=ready, active_localization_session=session, mapping_runtime_enabled=True,
        latest_map_snapshot=snapshot,
        latest_localization_snapshot=owner._localization_service.get_latest_snapshot() if ready else None,
        latest_memory_summary=owner._memory_service.get_summary(),
        navigation_context=owner._navigation_service.get_latest_navigation_context(),
        exploration_context=owner._navigation_service.get_latest_exploration_context(),
        metadata={'catalog_root': owner.catalog_root, 'identity_schema_version': 2,
                  'asset_identity': target.asset_identity.model_dump()})
    def still_current():
        if (owner._mapping_service.get_latest_snapshot() is not snapshot
                or owner._repository.get_map_activation() is not None
                or owner._repository.get_active_map_name() != asset.map_name):
            return False
        try:
            with owner._geometry_runtime.assigned_map_installation_guard(expected_target=target):
                actual, actual_epoch = owner._localization_service.get_active_session_token()
                return actual is session and actual_epoch == epoch
        except (ValueError, RuntimeError):
            return False

    # 所有扩展回调前后都核对实际拥有方；只撤销自己刚写入的历史对象。
    if not still_current():
        return None
    if publish:
        try:
            owner._state_store.write(StateNamespace.MAP, 'active_workspace', workspace,
                source='map_workspace_service', metadata={'kind': 'active_workspace', 'map_name': asset.map_name})
            valid = still_current()
        except BaseException:
            try:
                if not still_current():
                    owner._state_store.discard_if_value(StateNamespace.MAP, 'active_workspace', workspace)
            except BaseException:
                LOGGER.exception('Workspace stale-publication cleanup failed after interruption.')
            raise
        if not valid:
            owner._state_store.discard_if_value(StateNamespace.MAP, 'active_workspace', workspace)
            return None
    return workspace


def _save_selected_asset(owner, asset, bundle, metadata):
    asset = owner._repository.get_map_asset(bundle.map_name)
    if asset is None or asset.identity_schema_version != 2 or asset.asset_identity != bundle.asset_identity:
        raise _problem('目录资产拥有方在提交前已改变。')
    now = datetime.now(asset.updated_at.tzinfo or timezone.utc)
    values = dict(asset.metadata)
    values.pop('activation_target_manifest_sha256', None)
    values.update(metadata or {})
    values.update(activation_recovery_pending=False, activation_phase='active')
    selected = asset.model_copy(update={'identity_schema_version': 2, 'asset_identity': bundle.asset_identity,
        'asset_manifest_sha256': None, 'finalized': True, 'finalized_at': asset.finalized_at or bundle.timestamp,
        'active': True, 'localization_ready': False, 'status': MapAssetStatus.LOCALIZING,
        'frame_id': bundle.frame_id, 'last_activated_at': now, 'updated_at': now, 'metadata': values})
    owner._repository.set_active_map_name(bundle.map_name)
    # 不调用会触发 StateStore/事件回调的 _save_asset。
    owner._repository.save_map_asset(selected)
    # 完整目标已持久化；对账只需更新其它地图，避免先写一次目标 active 标志。
    owner._reconcile_catalog_active_flags(bundle.map_name)
    return selected


def _activation_evidence(owner, prepared, request, journal, initial_state, allow_dispatch):
    runtime = owner._geometry_runtime
    phase = journal['phase']
    if phase in ('backend_applied', 'reconciling', 'recovering') or not allow_dispatch:
        state = runtime.capture_assigned_mapping_state(prepared)
        if _target_request(state, prepared) != request:
            raise _problem('当前原生状态尚未证明原激活请求已封存。', journal)
        if phase == 'prepared':
            _advance(owner, prepared, request, 'backend_dispatching', ('prepared',))
        return state
    if phase == 'backend_dispatching':
        # 这是调用方再次显式提交；先完成原 dispatch 意图的持久化，再交给 provider。
        _advance(owner, prepared, request, 'backend_dispatching', ('backend_dispatching',))
        return runtime.dispatch_prepared_assigned_map(prepared, request)
    state = initial_state if initial_state is not None else runtime.capture_assigned_mapping_state(prepared)
    if _target_request(state, prepared) == request:
        _advance(owner, prepared, request, 'backend_dispatching', (phase,))
        return state
    if state['runtime_generation'] != request['expected_runtime_generation']:
        raise _problem('原请求的运行时代次已改变；不能替换请求代次。', journal)
    predecessor = journal['predecessor']
    if predecessor is not None:
        if {key: state.get(key) for key in _PREDECESSOR_FIELDS} != predecessor:
            raise _problem('当前原生会话与保留的前驱拥有方不一致。', journal)
        if state['active']:
            _advance(owner, prepared, request, 'predecessor_stop_dispatching', ('prepared',))
            runtime.stop_assigned_mapping_predecessor(prepared, predecessor)
        elif state['phase'] != 'STOPPED':
            raise _problem('尚无匹配的前驱 STOPPED 回执。', journal)
        _advance(owner, prepared, request, 'predecessor_stopped', ('prepared', 'predecessor_stop_dispatching'))
        phase = 'predecessor_stopped'
    elif state['active']:
        raise _problem('新的活跃拥有方未包含在原激活事务中。', journal)
    _advance(owner, prepared, request, 'backend_dispatching', (phase,))
    return runtime.dispatch_prepared_assigned_map(prepared, request)


def activate_assigned_workspace(owner, bundle, *, load_memory_history, reset_memory_library=False,
                                metadata=None, allow_dispatch=True, _activation_read=None):
    """一次显式事务；所有未知结果保留日志，绝不在异常中重新调用后端。"""
    if reset_memory_library:
        raise _problem('finalized 地图激活不支持重置记忆库。')
    state = owner._map_activation_execution_state
    if getattr(state, 'assigned_active', False):
        raise _problem('当前激活回调不能重入同一个同步激活操作。')
    state.assigned_active = True
    try:
        with owner._localization_service.assigned_session_publication_scope() as publication:
            try:
                _activate(owner, bundle, load_memory_history, metadata, allow_dispatch, publication, _activation_read)
            finally:
                # 定位发布回调在所有 E/C/R 守卫外运行，允许后继操作真正替换当前拥有方。
                state.assigned_active = False
        # 发布回调可能已经替换当前地图/会话，不能返回之前捕获的成功快照。
        asset = owner._repository.get_map_asset(bundle.map_name)
        workspace = build_assigned_workspace(owner, asset) if asset is not None else None
        if workspace is None:
            raise _problem('激活后的工作区已被后继操作替换或尚未一致。', owner._repository.get_map_activation())
        return workspace
    except BaseException as error:
        state.assigned_active = False
        journal = None
        try:
            journal = owner._repository.get_map_activation()
            if journal is not None and journal.get('activation_request') is not None:
                owner._arm_map_activation_fence(journal, reason='assigned_reconciliation_incomplete')
        except BaseException:
            LOGGER.exception('Assigned activation fence restoration failed after interruption.')
        if isinstance(error, Exception) and not isinstance(error, GatewayError) and journal is not None:
            raise _problem(f'地图激活尚待按原请求对账：{error}', journal) from error
        raise


def _activate(owner, bundle, load_memory_history, metadata, allow_dispatch, publication, activation_read):
    runtime, repository = owner._geometry_runtime, owner._repository
    root = owner._asset_repository.current_directory(bundle.map_name)
    if activation_read is None:
        prepared = runtime.prepare_assigned_map_activation(bundle, bundle_root=root)
    else:
        from services.mapping.asset_store import _AssignedActivationRead
        if type(activation_read) is not _AssignedActivationRead or activation_read.bundle is not bundle:
            raise TypeError("activation read must retain the exact admitted bundle")
        prepared = runtime.prepare_retained_assigned_map_activation(activation_read,
            repository=owner._asset_repository, bundle_root=root)
    with _transaction(owner):
        journal = repository.get_map_activation()
        asset = repository.get_map_asset(bundle.map_name)
        if asset is None:
            raise _problem('分配式 finalized 地图缺少目录记录。')
        if journal is not None and (journal.get('schema_version') != 3
                or journal.get('target_map_name') != bundle.map_name
                or journal.get('target_asset_identity') != prepared.asset_identity.model_dump()
                or journal.get('target_checkpoint_id') != prepared.checkpoint_id):
            raise _problem('另一个地图激活事务尚待对账。', journal)
    initial_state = None
    if journal is None:
        initial_state = runtime.capture_assigned_mapping_state(prepared)
        request = _target_request(initial_state, prepared)
        if request is None:
            if not allow_dispatch:
                raise _problem('启动恢复没有当前原生封存证据；请显式激活原地图。')
            request_id = 'mapload:' + uuid.uuid4().hex
            request = _request({'activation_request_id': request_id, 'session_id': request_id,
                'map_name': bundle.map_name, 'asset_identity': prepared.asset_identity.model_dump(),
                'checkpoint_id': prepared.checkpoint_id, 'expected_runtime_generation': initial_state['runtime_generation']})
        else:
            # 已完成的同一真实安装，不产生第二个安装/会话或重新导入地图。
            runtime.validate_prepared_assigned_map_evidence(prepared,
                activation_request=request, activation_evidence=initial_state)
            existing = build_assigned_workspace(owner, asset, publish=False)
            if existing is not None and existing.active_localization_session.activation_receipt_v2.to_dict()['request'] == request:
                return existing
        predecessor = ({key: initial_state[key] for key in _PREDECESSOR_FIELDS}
                       if initial_state['active'] and initial_state['session_id'] != request['session_id'] else None)
        with _transaction(owner):
            journal = repository.begin_assigned_map_activation(
                previous_map_name=repository.get_active_map_name(), target_map_name=bundle.map_name,
                target_asset_identity=prepared.asset_identity, target_checkpoint_id=prepared.checkpoint_id,
                load_memory_history=bool(load_memory_history), reuse_completed_request_id=False,
                activation_request=request, predecessor=predecessor)
    elif 'activation_request' not in journal:
        raise _problem('旧分配式日志缺少原 session/runtime 字段，不能推测后重执行。', journal)
    request = _request(journal['activation_request'])
    if not runtime.arm_assigned_map_activation_fence(activation_request=request, reason='assigned_workspace_activation'):
        raise _problem('运行态已有不同的激活拥有方。', journal)
    evidence = _activation_evidence(owner, prepared, request, journal, initial_state, allow_dispatch)
    receipt = runtime.validate_prepared_assigned_map_evidence(prepared,
        activation_request=request, activation_evidence=evidence)
    with _transaction(owner):
        journal = _require_journal(owner, prepared, request)
        journal = repository.record_assigned_map_activation_evidence(
            **_identity(prepared, request), activation_receipt=receipt,
            relocalization_policy_v2=evidence['relocalization_policy_v2'],
            relocalization_target_available=evidence['relocalization_target_available'],
            activation_receipt_disposition=evidence['activation_receipt_disposition'],
            activation_receipt_disposition_name=evidence['activation_receipt_disposition_name'],
            expected_phases=('backend_dispatching', 'backend_applied', 'reconciling', 'recovering'))
    target, snapshot = runtime.install_prepared_assigned_map(prepared,
        activation_request=request, activation_evidence=evidence)
    _advance(owner, prepared, request, 'reconciling', ('backend_applied', 'recovering'))
    current_map = owner._mapping_service.get_latest_snapshot()
    if not _mapping_matches(current_map, target):
        owner._mapping_service.ingest_external_snapshot(map_name=bundle.map_name,
            source_name=snapshot.source_name, source_epoch=snapshot.source_epoch,
            occupancy_grid=snapshot.occupancy_grid, cost_map=snapshot.cost_map, semantic_map=snapshot.semantic_map,
            metadata=snapshot.metadata, update_sequence_override=snapshot.update_sequence)
    runtime_snapshot = owner._mapping_service.get_latest_snapshot()
    if not _mapping_matches(runtime_snapshot, target):
        raise _problem('地图发布回调已替换安装快照。', journal)
    owner._navigation_service.load_forbidden_zones_for_map(map_name=bundle.map_name)
    owner._memory_service.ensure_memory_library(library_name=bundle.map_name)
    if owner._memory_service.get_active_library_name() != bundle.map_name:
        owner._memory_service.activate_memory_library(library_name=bundle.map_name,
            load_history=journal['load_memory_history'], reset_library=False, expected_library_name=bundle.map_name)
    expected, epoch = owner._localization_service.get_active_session_token()
    same_session = _session_matches(expected, target) and expected.frame_id == bundle.frame_id
    prepared_session = None if same_session else owner._localization_service.prepare_assigned_session_update(
        expected_session=expected, expected_mutation_epoch=epoch, activation_receipt_v2=receipt,
        relocalization_terminal_v2=None, frame_id=bundle.frame_id, refresh=False,
        metadata={'map_name': bundle.map_name, 'source_kind': 'assigned_map_asset'},
        geometry_asset={'geometry_stack': bundle.geometry_asset.geometry_stack, 'frame_id': bundle.frame_id,
                        'checkpoint_ready': True, 'live_streaming_ready': False})
    with _transaction(owner):
        _require_journal(owner, prepared, request)
        selected = _save_selected_asset(owner, asset, bundle, metadata)
        # 持久回执先写，但日志仍保留。磁盘 I/O 不进入 runtime/session 守卫。
        repository.complete_assigned_map_activation_durable(
            **_identity(prepared, request), activation_receipt=receipt, expected_phases=('reconciling',),
            idempotent_replay=evidence['activation_receipt_disposition'] == 3,
            relocalization_target_available=evidence['relocalization_target_available'], retain_journal=True)
        with runtime.assigned_map_installation_guard(expected_target=target) as guarded:
            if prepared_session is not None:
                commit = owner._localization_service._compare_and_enqueue_prepared_assigned_session(
                    prepared_session, guarded_activation_receipt_v2=guarded,
                    guarded_relocalization_terminal_v2=None, publication_scope=publication)
                if not commit.accepted:
                    raise _problem('定位会话在采用期间已改变。', journal)
                expected, epoch = commit.session, commit.committed_epoch
            else:
                current, current_epoch = owner._localization_service.get_active_session_token()
                if current is not expected or current_epoch != epoch:
                    raise _problem('同一定位会话已被后继修改。', journal)
        if owner._mapping_service.get_latest_snapshot() is not runtime_snapshot:
            raise _problem('最终提交前地图快照已改变。', journal)
        if not runtime.complete_assigned_map_activation(activation_request=request, expected_target=target):
            raise _problem('持久回执已写入，但运行态安装已改变，不能宣布成功。', journal)
        repository.clear_assigned_map_activation(**_identity(prepared, request), expected_phases=('reconciling',))
    return selected
