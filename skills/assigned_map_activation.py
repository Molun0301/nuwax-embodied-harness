"""分配式工作区激活编排：整体执行激活并消费真实原生回执。"""
import logging
import uuid
from datetime import datetime, timezone

from contracts.map_activation import _request
from contracts.map_workspace import MapAssetStatus, MapWorkspace
from contracts.runtime_views import LocalizationSessionStatus
from core import StateNamespace
from core.errors import GatewayError

LOGGER = logging.getLogger(__name__)

_PREDECESSOR_FIELDS = ('session_id', 'map_name', 'session_generation', 'runtime_generation', 'mapping_backend_kind')


def _problem(message):
    return GatewayError(message, error_code='map_activation_failed', http_status=409,
        jsonrpc_code=-32000)


def _mapping_matches(snapshot, target):
    if snapshot is None or snapshot.map_name != target.map_name:
        return False
    metadata = snapshot.metadata
    if (metadata.get('identity_schema_version') != 2
            or metadata.get('asset_identity') != target.asset_identity.model_dump()
            or metadata.get('map_activation_generation') != target.installation_generation):
        return False
    return metadata.get('activation_receipt_v2') == target.activation_receipt.to_dict()


def _session_matches(session, target):
    return (session is not None and session.identity_schema_version == 2
            and session.map_name == target.map_name and session.asset_identity == target.asset_identity
            and session.activation_receipt_v2.exact_replay_of(target.activation_receipt))


def current_components(owner, asset):
    """只读标量/已拥有对象；不刷新 provider，不执行旧 session CAS。"""
    if not asset.active or asset.identity_schema_version != 2:
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
    values = current_components(owner, asset.model_copy(update={'active': is_active}))
    ready = bool(values and values[2].status == LocalizationSessionStatus.READY
                 and values[2].relocalization_terminal_v2 is not None)
    return asset.model_copy(update={'active': is_active, 'localization_ready': ready,
        'status': MapAssetStatus.READY if ready or not is_active else MapAssetStatus.LOCALIZING})


def build_assigned_workspace(owner, asset, *, publish=True):
    values = current_components(owner, asset)
    if values is None:
        return None
    target, snapshot, session, epoch = values
    ready = (session.status == LocalizationSessionStatus.READY and session.relocalization_terminal_v2 is not None)
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
    values.update(metadata or {})
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


def activate_assigned_workspace(owner, bundle, *, publication, load_memory_history,
                                metadata=None, activation_read=None):
    """调用方持有定位发布范围与执行锁；失败后下次从头激活。"""
    state = owner._map_activation_execution_state
    if getattr(state, 'assigned_active', False):
        raise _problem('当前激活回调不能重入同一个同步激活操作。')
    state.assigned_active = True
    try:
        runtime, repository = owner._geometry_runtime, owner._repository
        asset = repository.get_map_asset(bundle.map_name)
        if asset is None:
            raise _problem('分配式 finalized 地图缺少目录记录。')
        if current_components(owner, asset) is not None:
            return
        runtime.clear_active_map_asset(reason='activate_map')
        owner._mapping_service.clear_latest_snapshot(
            cleared_map_name=repository.get_active_map_name(), reason='activate_map')
        owner._localization_service.clear_latest_snapshot()
        owner._localization_service.clear_active_session(reason='activate_map')
        owner._navigation_service.clear_latest_exploration_context(reason='activate_map')
        with repository.map_activation_guard():
            repository.set_active_map_name(bundle.map_name)
            owner._reconcile_catalog_active_flags(bundle.map_name)
        root = owner._asset_repository.current_directory(bundle.map_name)
        if activation_read is None:
            prepared = runtime.prepare_assigned_map_activation(bundle, bundle_root=root)
        else:
            from services.mapping.asset_store import _AssignedActivationRead
            if type(activation_read) is not _AssignedActivationRead or activation_read.bundle is not bundle:
                raise TypeError("activation read must retain the exact admitted bundle")
            prepared = runtime.prepare_retained_assigned_map_activation(activation_read,
                repository=owner._asset_repository, bundle_root=root)
        current_state = runtime.capture_assigned_mapping_state(prepared)
        if current_state['active']:
            runtime.stop_assigned_mapping_predecessor(prepared,
                {key: current_state[key] for key in _PREDECESSOR_FIELDS})
        request_id = 'mapload:' + uuid.uuid4().hex
        request = _request({'activation_request_id': request_id, 'session_id': request_id,
            'map_name': bundle.map_name, 'asset_identity': prepared.asset_identity.model_dump(),
            'checkpoint_id': prepared.checkpoint_id,
            'expected_runtime_generation': current_state['runtime_generation']})
        evidence = runtime.dispatch_prepared_assigned_map(prepared, request)
        receipt = runtime.validate_prepared_assigned_map_evidence(prepared, activation_evidence=evidence)
        target, snapshot = runtime.install_prepared_assigned_map(prepared, activation_evidence=evidence)
        if not _mapping_matches(owner._mapping_service.get_latest_snapshot(), target):
            owner._mapping_service.ingest_external_snapshot(map_name=bundle.map_name,
                source_name=snapshot.source_name, source_epoch=snapshot.source_epoch,
                occupancy_grid=snapshot.occupancy_grid, cost_map=snapshot.cost_map, semantic_map=snapshot.semantic_map,
                metadata=snapshot.metadata, update_sequence_override=snapshot.update_sequence)
        runtime_snapshot = owner._mapping_service.get_latest_snapshot()
        if not _mapping_matches(runtime_snapshot, target):
            raise _problem('地图发布回调已替换安装快照。')
        owner._navigation_service.load_forbidden_zones_for_map(map_name=bundle.map_name)
        owner._memory_service.ensure_memory_library(library_name=bundle.map_name)
        if owner._memory_service.get_active_library_name() != bundle.map_name:
            owner._memory_service.activate_memory_library(library_name=bundle.map_name,
                load_history=load_memory_history, reset_library=False, expected_library_name=bundle.map_name)
        expected, epoch = owner._localization_service.get_active_session_token()
        prepared_session = owner._localization_service.prepare_assigned_session_update(
            expected_session=expected, expected_mutation_epoch=epoch, activation_receipt_v2=receipt,
            relocalization_terminal_v2=None, frame_id=bundle.frame_id, refresh=False,
            metadata={'map_name': bundle.map_name, 'source_kind': 'assigned_map_asset'},
            geometry_asset={'geometry_stack': bundle.geometry_asset.geometry_stack, 'frame_id': bundle.frame_id,
                            'checkpoint_ready': True, 'live_streaming_ready': False})
        with repository.map_activation_guard():
            _save_selected_asset(owner, asset, bundle, metadata)
            with runtime.assigned_map_installation_guard(expected_target=target) as guarded:
                commit = owner._localization_service._compare_and_enqueue_prepared_assigned_session(
                    prepared_session, guarded_activation_receipt_v2=guarded,
                    guarded_relocalization_terminal_v2=None, publication_scope=publication)
                if not commit.accepted:
                    raise _problem('定位会话在采用期间已改变。')
            if owner._mapping_service.get_latest_snapshot() is not runtime_snapshot:
                raise _problem('最终提交前地图快照已改变。')
    finally:
        state.assigned_active = False
