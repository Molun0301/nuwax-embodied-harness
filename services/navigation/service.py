from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from services.navigation.motion_guard_client import StopAllMotionConfirmation

from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
import logging
import threading
import time

import grpc

from contracts.events import RuntimeEventCategory
from contracts.geometry_runtime import DEFAULT_MAP_FRAME
from contracts.forbidden_zones import ForbiddenZone, ForbiddenZonesEnvelope
from contracts.map_workspace import MapAssetStatus
from contracts.runtime_views import LocalizationSessionStatus
from contracts.navigation import ExplorationStatus, ExploreAreaRequest, NavigationGoal, NavigationState, NavigationStatus
from contracts.runtime_views import ExplorationContext, NavigationContext
from core import EventBus, StateNamespace, StateStore, StreamBus
from core.errors import GatewayError
from core.logging_utils import LogRateLimiter
from providers import ExplorationProvider, NavigationProvider
from services.localization import LocalizationService
from services.mapping.service import MappingService
from services.geometry.geometry_runtime_provider import (
    GeometryRuntimeRpcError,
)
from typing import Deque, Dict, Optional, Sequence, Tuple

from .forbidden_zones import ForbiddenZoneStore
from .state_machine import (
    NavigationStateMachine,
    NavigationDiagnostics,
    create_navigation_state_machine,
)

LOGGER = logging.getLogger("nuwax_robot_bridge.navigation.service")

_EXPLORATION_MAP_IDENTITY_KEYS = {
    "map_version_id",
    "map_revision",
    "stop_on_map_revision_change",
}


def _exploration_deadline_remaining(
    deadline_monotonic: Optional[float],
    rpc_name: str,
) -> Optional[float]:
    if deadline_monotonic is None:
        return None
    remaining = float(deadline_monotonic) - time.monotonic()
    if remaining <= 0:
        raise GeometryRuntimeRpcError(
            rpc_name=rpc_name,
            grpc_code=grpc.StatusCode.DEADLINE_EXCEEDED,
            grpc_details="cooperative exploration deadline exhausted",
        )
    return remaining


@contextmanager
def _exploration_deadline_lock(lock, deadline_monotonic, rpc_name):
    remaining = _exploration_deadline_remaining(deadline_monotonic, rpc_name)
    if remaining is None:
        with lock:
            yield
        return
    if not lock.acquire(timeout=remaining):
        raise GeometryRuntimeRpcError(
            rpc_name=rpc_name,
            grpc_code=grpc.StatusCode.DEADLINE_EXCEEDED,
            grpc_details="cooperative exploration lock deadline exhausted",
        )
    try:
        yield
    finally:
        lock.release()


def _exploration_owner_lock(service):
    return getattr(service, "_exploration_ownership_lock", service._lock)


@dataclass(frozen=True)
class _ForbiddenZoneRuntimeAck:
    map_name: str
    generation: int
    signature: str
    applied_revision: int
    planner_revision: int

    def confirms(
        self,
        *,
        map_name: str,
        generation: int,
        signature: str,
    ) -> bool:
        return bool(
            self.map_name == map_name
            and self.generation == generation
            and self.signature == signature
            and signature
            and self.applied_revision > 0
            and self.planner_revision >= self.applied_revision
        )


def _is_complete_exploration_terminal(
    exploration_state,
    *,
    request_id: str,
) -> bool:
    expected_id = str(request_id or "").strip()
    metadata = dict(getattr(exploration_state, "metadata", {}) or {})
    return bool(
        expected_id
        and str(
            getattr(exploration_state, "current_request_id", "") or ""
        ).strip()
        == expected_id
        and getattr(exploration_state, "status", None)
        in {
            ExplorationStatus.SUCCEEDED,
            ExplorationStatus.FAILED,
            ExplorationStatus.CANCELLED,
        }
        and metadata.get("exploration_terminal") is True
        and metadata.get("parking_pending") is not True
        and not str(metadata.get("child_task_id") or "").strip()
        and metadata.get("child_lease") is None
    )


def _without_exploration_map_identity(
    request: Optional[ExploreAreaRequest],
) -> Optional[ExploreAreaRequest]:
    if request is None:
        return None
    metadata = dict(request.metadata)
    stop_conditions = dict(request.stop_conditions)
    for key in _EXPLORATION_MAP_IDENTITY_KEYS:
        metadata.pop(key, None)
        stop_conditions.pop(key, None)
    if metadata == dict(request.metadata) and stop_conditions == dict(
        request.stop_conditions
    ):
        return request
    return request.model_copy(
        update={
            "metadata": metadata,
            "stop_conditions": stop_conditions,
        },
        deep=False,
    )


class NavigationService:
    """统一导航与探索服务。

    平台导航运行时核心：
    - 维护平台导航状态机，管理 NavigationGoal -> 当前导航后端 -> NavigationContext 的完整链路
    - 原生导航运行时是唯一正式执行器，目标生成、状态推进、失败语义和取消语义都沉淀在平台
    - 已废弃的 platform_grid 自定义执行链不得作为正式后端或回退路径重新接入
    - 导航完成与否由平台状态机和到点验证共同决定
    """

    def __init__(
        self,
        *,
        provider_owner,
        localization_service: LocalizationService,
        mapping_service: MappingService,
        state_store: StateStore,
        event_bus: Optional[EventBus] = None,
        stream_bus: Optional[StreamBus] = None,
        history_limit: int = 100,
        navigation_state_machine: Optional[NavigationStateMachine] = None,
        forbidden_zone_store: Optional[ForbiddenZoneStore] = None,
    ) -> None:
        self._provider_owner = provider_owner
        self._localization_service = localization_service
        self._mapping_service = mapping_service
        self._state_store = state_store
        self._event_bus = event_bus
        self._stream_bus = stream_bus
        self._forbidden_zone_store = forbidden_zone_store or ForbiddenZoneStore()
        self._navigation_history: Deque[NavigationContext] = deque(maxlen=max(1, history_limit))
        self._exploration_history: Deque[ExplorationContext] = deque(maxlen=max(1, history_limit))
        self._latest_navigation_context: Optional[NavigationContext] = None
        self._latest_exploration_context: Optional[ExplorationContext] = None
        self._current_goal: Optional[NavigationGoal] = None
        self._current_explore_request: Optional[ExploreAreaRequest] = None
        self._pending_explore_request_id: Optional[str] = None
        self._forbidden_zone_update_lock = threading.Lock()
        self._forbidden_zone_update_active = False
        self._forbidden_zone_required_generation: Optional[int] = None
        self._forbidden_zone_required_signature: Optional[str] = None
        self._forbidden_zone_required_map_name: Optional[str] = None
        self._forbidden_zone_cancel_wait_timeout_sec = 5.0
        self._forbidden_zone_cancel_poll_interval_sec = 0.05
        self._forbidden_zone_revision_wait_timeout_sec = 2.0
        self._forbidden_zone_revision_poll_interval_sec = 0.05
        # 平台导航状态机
        self._state_machine: NavigationStateMachine = navigation_state_machine or create_navigation_state_machine()
        self._last_navigation_diagnostics: Optional[NavigationDiagnostics] = None
        self._lock = threading.RLock()
        self._provider_update_log_limiter = LogRateLimiter()
        self._exploration_ownership_lock = threading.RLock()
        self._navigation_update_subscription_id: Optional[int] = None
        self._exploration_update_subscription_id: Optional[int] = None
        # 定位会话刷新节流：避免 *_until_complete 高频轮询每轮触发完整定位刷新副作用链（P0-3）
        self._localization_refresh_min_interval_sec: float = 1.0
        self._last_localization_refresh_monotonic: Optional[float] = None
        self._subscribe_provider_navigation_updates()
        self._subscribe_provider_exploration_updates()

    def list_forbidden_zones(
        self,
        map_name: Optional[str] = None,
    ) -> ForbiddenZonesEnvelope:
        """返回指定地图的禁行区；省略地图名时解析当前活动地图。"""

        resolved_map_name = self._resolve_forbidden_zone_map_name(map_name)
        return self._forbidden_zone_store.list_for_map(resolved_map_name)

    def load_forbidden_zones_for_map(
        self,
        *,
        map_name: str,
    ) -> ForbiddenZonesEnvelope:
        """在地图载入后恢复其禁行区快照并同步前端与运行时栅栏。"""

        resolved_map_name = self._resolve_forbidden_zone_map_name(map_name)
        envelope = self._forbidden_zone_store.list_for_map(resolved_map_name)
        entry = envelope.maps[resolved_map_name]
        observed_ack = self._read_forbidden_zone_runtime_ack()
        with self._forbidden_zone_state_lock():
            if (
                entry.generation > 0
                and entry.signature
                and not (
                    observed_ack is not None
                    and observed_ack.confirms(
                        map_name=resolved_map_name,
                        generation=entry.generation,
                        signature=entry.signature,
                    )
                )
            ):
                self._forbidden_zone_required_generation = entry.generation
                self._forbidden_zone_required_signature = entry.signature
                self._forbidden_zone_required_map_name = resolved_map_name
            else:
                self._forbidden_zone_required_generation = None
                self._forbidden_zone_required_signature = None
                self._forbidden_zone_required_map_name = None
        if self._stream_bus is not None:
            self._stream_bus.publish(
                "navigation.forbidden_zones",
                source="navigation_service",
                payload=envelope,
                frame_id=DEFAULT_MAP_FRAME,
                sequence=entry.generation,
                revision=str(entry.generation),
                status="available",
                metadata={
                    "map_name": resolved_map_name,
                    "generation": entry.generation,
                    "zone_count": len(entry.zones),
                    "lifecycle_action": "map_loaded",
                },
            )
        return envelope

    def delete_forbidden_zones_for_map(
        self,
        *,
        map_name: str,
    ) -> ForbiddenZonesEnvelope:
        """删除地图资产时同步删除其禁行区记录。"""

        tombstones = self._forbidden_zone_store.delete_for_map(map_name)
        if not tombstones.maps:
            return tombstones
        entry = next(iter(tombstones.maps.values()))
        with self._forbidden_zone_state_lock():
            if (
                self._forbidden_zone_required_map_name
                == map_name
            ):
                self._forbidden_zone_required_generation = None
                self._forbidden_zone_required_signature = None
                self._forbidden_zone_required_map_name = None
        if self._stream_bus is not None:
            self._stream_bus.publish(
                "navigation.forbidden_zones",
                source="navigation_service",
                payload=tombstones,
                frame_id=DEFAULT_MAP_FRAME,
                sequence=entry.generation,
                revision=str(entry.generation),
                status="cleared",
                metadata={
                    "map_name": map_name,
                    "generation": entry.generation,
                    "zone_count": 0,
                    "lifecycle_action": "map_deleted",
                },
            )
        return tombstones

    def replace_forbidden_zones(
        self,
        *,
        map_name: Optional[str] = None,
        zones: Sequence[ForbiddenZone],
        expected_generation: Optional[int] = None,
    ) -> ForbiddenZonesEnvelope:
        """原子替换禁行区；活动地图更新前先停止旧 revision 上的运动任务。"""

        resolved_map_name = self._resolve_forbidden_zone_map_name(map_name)
        zone_snapshot = tuple(zones)
        update_lock = self._forbidden_zone_update_lock
        with update_lock:
            affects_active_map = self._forbidden_zone_update_affects_active_map(
                resolved_map_name
            )
            self._preflight_forbidden_zone_generation(
                map_name=resolved_map_name,
                expected_generation=expected_generation,
            )
            if affects_active_map:
                envelope = self._replace_active_map_forbidden_zones(
                    map_name=resolved_map_name,
                    zones=zone_snapshot,
                    expected_generation=expected_generation,
                )
            else:
                envelope = self._replace_inactive_map_forbidden_zones(
                    map_name=resolved_map_name,
                    zones=zone_snapshot,
                    expected_generation=expected_generation,
                )
            entry = envelope.maps[resolved_map_name]
            if self._stream_bus is not None:
                self._stream_bus.publish(
                    "navigation.forbidden_zones",
                    source="navigation_service",
                    payload=envelope,
                    frame_id=DEFAULT_MAP_FRAME,
                    sequence=entry.generation,
                    revision=str(entry.generation),
                    status="available",
                    metadata={
                        "map_name": resolved_map_name,
                        "generation": entry.generation,
                        "zone_count": len(entry.zones),
                    },
                )
            return envelope

    def _preflight_forbidden_zone_generation(
        self,
        *,
        map_name: str,
        expected_generation: Optional[int],
    ) -> int:
        """在任何取消动作前验证 store 可读且 CAS generation 匹配。"""

        envelope = self._forbidden_zone_store.list_for_map(map_name)
        current_generation = envelope.maps[map_name].generation
        if (
            expected_generation is not None
            and expected_generation != current_generation
        ):
            raise GatewayError(
                "禁行区已被其他编辑器更新，请重新加载后再保存。",
                error_code="forbidden_zones_generation_conflict",
                http_status=409,
                details={
                    "map_name": map_name,
                    "expected_generation": expected_generation,
                    "current_generation": current_generation,
                },
            )
        return current_generation

    def _replace_inactive_map_forbidden_zones(
        self,
        *,
        map_name: str,
        zones: Sequence[ForbiddenZone],
        expected_generation: Optional[int],
    ) -> ForbiddenZonesEnvelope:
        state_lock = self._forbidden_zone_state_lock()
        with state_lock:
            self._forbidden_zone_update_active = True
        try:
            return self._forbidden_zone_store.replace(
                map_name=map_name,
                zones=zones,
                expected_generation=expected_generation,
            )
        finally:
            with state_lock:
                self._forbidden_zone_update_active = False

    def _forbidden_zone_update_affects_active_map(
        self,
        map_name: str,
    ) -> bool:
        """仅为会被当前 Geometry Runtime 消费的地图建立运动栅栏。"""

        active_map_names = set()
        active_map_lookup_failed = False
        localization_service = self._localization_service
        if localization_service is not None:
            try:
                active_session = localization_service.get_active_session()
            except Exception:
                active_session = None
                active_map_lookup_failed = True
            active_map_name = str(
                getattr(active_session, "map_name", "") or ""
            ).strip()
            if active_map_name:
                active_map_names.add(active_map_name)

        mapping_service = self._mapping_service
        if mapping_service is not None:
            try:
                map_snapshot = mapping_service.get_latest_snapshot()
            except Exception:
                map_snapshot = None
                active_map_lookup_failed = True
            snapshot_metadata = getattr(map_snapshot, "metadata", None)
            if isinstance(snapshot_metadata, dict):
                for key in (
                    "active_map_name",
                    "map_name",
                    "seed_map_name",
                ):
                    active_map_name = str(
                        snapshot_metadata.get(key) or ""
                    ).strip()
                    if active_map_name:
                        active_map_names.add(active_map_name)

        with self._forbidden_zone_state_lock():
            metadata_sources = [
                getattr(self._current_goal, "metadata", None),
                getattr(
                    self._current_explore_request,
                    "metadata",
                    None,
                ),
                getattr(
                    getattr(
                        self._latest_navigation_context,
                        "navigation_state",
                        None,
                    ),
                    "metadata",
                    None,
                ),
                getattr(
                    getattr(
                        self._latest_exploration_context,
                        "exploration_state",
                        None,
                    ),
                    "metadata",
                    None,
                ),
            ]
        for metadata_source in metadata_sources:
            if not isinstance(metadata_source, dict):
                continue
            for key in (
                "map_name",
                "active_map_name",
                "seed_map_name",
            ):
                active_map_name = str(
                    metadata_source.get(key) or ""
                ).strip()
                if active_map_name:
                    active_map_names.add(active_map_name)

        # 无法证明更新与活动地图无关时 fail closed。
        return bool(
            map_name in active_map_names
            or active_map_lookup_failed
            or not active_map_names
        )

    def _replace_active_map_forbidden_zones(
        self,
        *,
        map_name: str,
        zones: Sequence[ForbiddenZone],
        expected_generation: Optional[int],
    ) -> ForbiddenZonesEnvelope:
        state_lock = self._forbidden_zone_state_lock()
        with state_lock:
            pending_request_id = str(
                self._pending_explore_request_id or ""
            ).strip()
            current_request_id = str(
                getattr(
                    self._current_explore_request,
                    "request_id",
                    "",
                )
                or ""
            ).strip()
            if pending_request_id and pending_request_id != current_request_id:
                raise GatewayError(
                    "探索任务正在启动，无法安全替换活动地图禁行区。",
                    error_code="forbidden_zone_update_conflict",
                    http_status=409,
                    details={"pending_exploration_request_id": pending_request_id},
                )
            goal_id = self._active_navigation_goal_id_locked()
            exploration_request_id = (
                current_request_id or self._active_exploration_request_id_locked()
            )
            self._forbidden_zone_update_active = True

        try:
            exploration_request_id = (
                self._authoritative_active_exploration_request_id(
                    exploration_request_id
                )
            )
            goal_id = self._authoritative_active_navigation_goal_id(
                goal_id,
                exploration_request_id=exploration_request_id,
            )
            if exploration_request_id:
                exploration_context = self.stop_exploration(
                    request_id=exploration_request_id
                )
                self._wait_for_forbidden_zone_exploration_stop(
                    exploration_request_id,
                    initial_state=getattr(
                        exploration_context,
                        "exploration_state",
                        None,
                    ),
                )
            if goal_id:
                navigation_context = self.cancel_goal(request_id=goal_id)
                self._wait_for_forbidden_zone_navigation_stop(
                    goal_id,
                    initial_state=getattr(
                        navigation_context,
                        "navigation_state",
                        None,
                    ),
                )

            envelope = self._forbidden_zone_store.replace(
                map_name=map_name,
                zones=zones,
                expected_generation=expected_generation,
            )
            entry = envelope.maps[map_name]
            required_generation = entry.generation
            required_signature = entry.signature
            with state_lock:
                self._forbidden_zone_required_generation = required_generation
                self._forbidden_zone_required_signature = required_signature
                self._forbidden_zone_required_map_name = map_name

            observed_ack = self._wait_for_forbidden_zone_runtime_ack(
                map_name=map_name,
                generation=required_generation,
                signature=required_signature,
            )
            with state_lock:
                if (
                    observed_ack is not None
                    and observed_ack.confirms(
                        map_name=map_name,
                        generation=required_generation,
                        signature=required_signature,
                    )
                    and self._forbidden_zone_required_generation
                    == required_generation
                    and self._forbidden_zone_required_signature
                    == required_signature
                    and self._forbidden_zone_required_map_name == map_name
                ):
                    self._forbidden_zone_required_generation = None
                    self._forbidden_zone_required_signature = None
                    self._forbidden_zone_required_map_name = None
            if observed_ack is None or not observed_ack.confirms(
                map_name=map_name,
                generation=required_generation,
                signature=required_signature,
            ):
                LOGGER.warning(
                    "禁行区已保存，但 Geometry Runtime/planner 尚未确认消费该 generation；"
                    "map_name=%s generation=%s observed_ack=%s",
                    map_name,
                    required_generation,
                    observed_ack,
                )
            return envelope
        finally:
            with state_lock:
                self._forbidden_zone_update_active = False

    def _forbidden_zone_state_lock(self):
        state_lock = self._lock
        return state_lock

    def _active_navigation_goal_id_locked(self) -> Optional[str]:
        goal_id = str(
            getattr(self._current_goal, "goal_id", "") or ""
        ).strip()
        if goal_id:
            return goal_id
        context = self._latest_navigation_context
        state = getattr(context, "navigation_state", None)
        if getattr(state, "status", None) not in {
            NavigationStatus.UNCONFIRMED,
            NavigationStatus.ACCEPTED,
            NavigationStatus.PLANNING,
            NavigationStatus.RUNNING,
        }:
            return None
        return str(getattr(state, "current_goal_id", "") or "").strip() or None

    def _active_exploration_request_id_locked(self) -> Optional[str]:
        context = self._latest_exploration_context
        state = getattr(context, "exploration_state", None)
        if getattr(state, "status", None) not in {
            ExplorationStatus.ACCEPTED,
            ExplorationStatus.RUNNING,
        }:
            return None
        return str(getattr(state, "current_request_id", "") or "").strip() or None

    @staticmethod
    def _status_value(state) -> str:
        raw_status = getattr(state, "status", None)
        return str(getattr(raw_status, "value", raw_status) or "").strip().lower()

    def _authoritative_active_exploration_request_id(
        self,
        fallback_request_id: Optional[str],
    ) -> Optional[str]:
        owner = self._provider_owner
        provider = getattr(owner, "providers", None)
        state_reader = getattr(provider, "get_exploration_state", None)
        if not callable(state_reader):
            return fallback_request_id
        try:
            state = state_reader()
        except Exception as exc:
            raise GatewayError(
                "禁行区更新前无法确认当前探索 exact request_id。",
                error_code="exploration_request_identity_unavailable",
                http_status=503,
            ) from exc
        status = self._status_value(state)
        if not status:
            return fallback_request_id
        backend_request_id = str(
            getattr(state, "current_request_id", "") or ""
        ).strip()
        metadata = dict(getattr(state, "metadata", {}) or {})
        is_running = status in {
            ExplorationStatus.ACCEPTED.value,
            ExplorationStatus.RUNNING.value,
        }
        complete_terminal = _is_complete_exploration_terminal(
            state,
            request_id=backend_request_id,
        )
        retains_child = bool(str(metadata.get("child_task_id") or "").strip())
        requires_stop = bool(
            is_running
            or retains_child
            or (
                backend_request_id
                and status != ExplorationStatus.IDLE.value
                and not complete_terminal
            )
        )
        if requires_stop and not backend_request_id:
            raise GatewayError(
                "探索后端处于活动状态，但未提供 exact request_id。",
                error_code="exploration_request_identity_unavailable",
                http_status=409,
                details={"active_status": status},
            )
        if (
            fallback_request_id
            and complete_terminal
            and backend_request_id == fallback_request_id
        ):
            return None
        if fallback_request_id and (
            not requires_stop or backend_request_id != fallback_request_id
        ):
            raise GatewayError(
                "本地探索 request_id 与后端活动会话不一致。",
                error_code="exploration_request_mismatch",
                http_status=409,
                details={
                    "local_request_id": fallback_request_id,
                    "active_request_id": backend_request_id or None,
                    "active_status": status,
                },
            )
        return backend_request_id if requires_stop else None

    def _authoritative_active_navigation_goal_id(
        self,
        fallback_goal_id: Optional[str],
        *,
        exploration_request_id: Optional[str],
    ) -> Optional[str]:
        provider = self._forbidden_zone_navigation_provider()
        if provider is None:
            return fallback_goal_id
        try:
            state = provider.get_navigation_state()
        except Exception as exc:
            raise GatewayError(
                "禁行区更新前无法确认当前导航 exact goal_id。",
                error_code="navigation_goal_identity_unavailable",
                http_status=503,
            ) from exc
        status = self._status_value(state)
        if not status:
            return fallback_goal_id
        backend_goal_id = str(
            getattr(state, "current_goal_id", "") or ""
        ).strip()
        is_active = status in {
            NavigationStatus.UNCONFIRMED.value,
            NavigationStatus.ACCEPTED.value,
            NavigationStatus.PLANNING.value,
            NavigationStatus.RUNNING.value,
        }
        if is_active and not backend_goal_id:
            raise GatewayError(
                "导航后端处于活动状态，但未提供 exact goal_id。",
                error_code="navigation_goal_identity_unavailable",
                http_status=409,
                details={"active_status": status},
            )
        metadata = dict(getattr(state, "metadata", {}) or {})
        cancel_phase = str(metadata.get("cancel_phase") or "").strip().upper()
        if backend_goal_id and (
            cancel_phase in {"FAILED", "TIMED_OUT"}
            or (
                status == NavigationStatus.CANCELLED.value
                and not self._navigation_has_complete_zero_proof(
                    state,
                    goal_id=backend_goal_id,
                )
            )
        ):
            raise GatewayError(
                "当前导航会话缺少 actuator ZEROED 证明，禁止更新禁行区。",
                error_code="navigation_cancel_not_zeroed",
                http_status=409,
                details={
                    "goal_id": backend_goal_id,
                    "active_status": status,
                    "cancel_phase": cancel_phase or None,
                },
            )
        parent_request_id = str(
            metadata.get("parent_request_id") or ""
        ).strip()
        owner_kind = metadata.get("owner_kind")
        exploration_child = bool(
            exploration_request_id
            and is_active
            and (
                parent_request_id == exploration_request_id
                or owner_kind in {2, "2", "exploration_child"}
            )
        )
        if exploration_child and not fallback_goal_id:
            return None
        if (
            fallback_goal_id
            and backend_goal_id == fallback_goal_id
            and self._navigation_has_complete_zero_proof(
                state,
                goal_id=backend_goal_id,
            )
        ):
            return None
        if fallback_goal_id and (
            not is_active or backend_goal_id != fallback_goal_id
        ):
            raise GatewayError(
                "本地导航 goal_id 与后端活动会话不一致。",
                error_code="navigation_goal_mismatch",
                http_status=409,
                details={
                    "local_goal_id": fallback_goal_id,
                    "active_goal_id": backend_goal_id or None,
                    "active_status": status,
                },
            )
        return backend_goal_id if is_active else None

    @staticmethod
    def _navigation_has_complete_zero_proof(state, *, goal_id: str) -> bool:
        metadata = dict(getattr(state, "metadata", {}) or {})
        try:
            zero_applied_monotonic_ns = int(
                metadata.get("zero_applied_monotonic_ns") or 0
            )
        except (TypeError, ValueError):
            zero_applied_monotonic_ns = 0
        return bool(
            NavigationService._status_value(state)
            == NavigationStatus.CANCELLED.value
            and str(getattr(state, "current_goal_id", "") or "").strip()
            == goal_id
            and metadata.get("actuator_zeroed") is True
            and str(metadata.get("zero_ack_id") or "").strip()
            and zero_applied_monotonic_ns > 0
        )

    def _forbidden_zone_cancel_wait_settings(self) -> Tuple[float, float]:
        timeout_sec = max(
            0.0,
            float(
                self._forbidden_zone_cancel_wait_timeout_sec
            ),
        )
        poll_interval_sec = max(
            0.01,
            float(
                self._forbidden_zone_cancel_poll_interval_sec
            ),
        )
        return timeout_sec, poll_interval_sec

    def _wait_for_forbidden_zone_navigation_stop(
        self,
        goal_id: str,
        *,
        initial_state=None,
    ) -> None:
        provider = self._forbidden_zone_navigation_provider()
        if initial_state is not None and self._navigation_has_complete_zero_proof(
            initial_state,
            goal_id=goal_id,
        ):
            return
        if provider is None:
            raise GatewayError(
                "导航取消已下发，但无法验证 actuator ZEROED 证明。",
                error_code="navigation_cancel_confirmation_unavailable",
                http_status=503,
                details={"goal_id": goal_id},
            )
        timeout_sec, poll_interval_sec = (
            self._forbidden_zone_cancel_wait_settings()
        )
        deadline = time.monotonic() + timeout_sec
        last_state = initial_state
        while True:
            try:
                last_state = provider.get_navigation_state()
            except Exception as exc:
                raise GatewayError(
                    "导航取消后无法读取 actuator ZEROED 证明。",
                    error_code="navigation_cancel_confirmation_unavailable",
                    http_status=503,
                    details={"goal_id": goal_id},
                ) from exc
            if self._navigation_has_complete_zero_proof(
                last_state,
                goal_id=goal_id,
            ):
                return
            metadata = dict(getattr(last_state, "metadata", {}) or {})
            cancel_phase = str(metadata.get("cancel_phase") or "").strip().upper()
            if (
                self._status_value(last_state) == NavigationStatus.FAILED.value
                or cancel_phase in {"FAILED", "TIMED_OUT"}
            ):
                raise GatewayError(
                    "导航取消未获得 actuator ZEROED 证明。",
                    error_code="navigation_cancel_not_zeroed",
                    http_status=409,
                    details={
                        "goal_id": goal_id,
                        "cancel_phase": cancel_phase or None,
                    },
                )
            if time.monotonic() >= deadline:
                raise GatewayError(
                    "等待导航 actuator ZEROED 证明超时，禁行区未写入。",
                    error_code="navigation_cancel_confirmation_timeout",
                    http_status=503,
                    details={
                        "goal_id": goal_id,
                        "active_status": self._status_value(last_state) or None,
                    },
                )
            time.sleep(poll_interval_sec)

    def _wait_for_forbidden_zone_exploration_stop(
        self,
        request_id: str,
        *,
        initial_state=None,
    ) -> None:
        if initial_state is not None and _is_complete_exploration_terminal(
            initial_state,
            request_id=request_id,
        ):
            return
        owner = self._provider_owner
        provider = getattr(owner, "providers", None)
        state_reader = getattr(provider, "get_exploration_state", None)
        if not callable(state_reader):
            raise GatewayError(
                "探索停止已下发，但无法验证 child navigation 已清空。",
                error_code="exploration_stop_confirmation_unavailable",
                http_status=503,
                details={"request_id": request_id},
            )
        timeout_sec, poll_interval_sec = (
            self._forbidden_zone_cancel_wait_settings()
        )
        deadline = time.monotonic() + timeout_sec
        last_state = initial_state
        while True:
            try:
                last_state = state_reader()
            except Exception as exc:
                raise GatewayError(
                    "探索停止后无法读取 child navigation 清理状态。",
                    error_code="exploration_stop_confirmation_unavailable",
                    http_status=503,
                    details={"request_id": request_id},
                ) from exc
            if _is_complete_exploration_terminal(
                last_state,
                request_id=request_id,
            ):
                return
            observed_request_id = str(
                getattr(last_state, "current_request_id", "") or ""
            ).strip()
            if (
                observed_request_id
                and observed_request_id != request_id
                and self._status_value(last_state)
                in {
                    ExplorationStatus.ACCEPTED.value,
                    ExplorationStatus.RUNNING.value,
                }
            ):
                raise GatewayError(
                    "探索停止确认时检测到其它活动 request_id。",
                    error_code="exploration_request_mismatch",
                    http_status=409,
                    details={
                        "request_id": request_id,
                        "active_request_id": observed_request_id,
                    },
                )
            if time.monotonic() >= deadline:
                metadata = dict(getattr(last_state, "metadata", {}) or {})
                raise GatewayError(
                    "等待探索 child navigation 清空超时，禁行区未写入。",
                    error_code="exploration_stop_confirmation_timeout",
                    http_status=503,
                    details={
                        "request_id": request_id,
                        "active_status": self._status_value(last_state) or None,
                        "child_task_id": metadata.get("child_task_id"),
                    },
                )
            time.sleep(poll_interval_sec)

    def _forbidden_zone_navigation_provider(self):
        owner = self._provider_owner
        provider = getattr(owner, "providers", None)
        if not callable(getattr(provider, "get_navigation_state", None)):
            return None
        return provider

    def _read_forbidden_zone_runtime_ack(
        self,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> Optional[_ForbiddenZoneRuntimeAck]:
        owner = self._provider_owner
        provider = getattr(owner, "providers", None)
        readiness_reader = getattr(provider, "get_navigation_readiness", None)
        if not callable(readiness_reader):
            return None
        try:
            if deadline_monotonic is None:
                readiness = readiness_reader()
            else:
                readiness = readiness_reader(
                    deadline_monotonic=deadline_monotonic,
                )
        except GeometryRuntimeRpcError:
            raise
        except Exception:
            LOGGER.exception("读取禁行区 runtime acknowledgement 失败。")
            return None
        if not isinstance(readiness, dict):
            return None

        def _nonnegative_int(value) -> int:
            if value is None or isinstance(value, bool):
                return 0
            try:
                parsed = int(value)
            except (TypeError, ValueError):
                return 0
            return parsed if parsed >= 0 else 0

        return _ForbiddenZoneRuntimeAck(
            map_name=str(
                readiness.get("forbidden_zone_map_name") or ""
            ).strip(),
            generation=_nonnegative_int(
                readiness.get("forbidden_zone_generation")
            ),
            signature=str(
                readiness.get("forbidden_zone_signature") or ""
            ).strip(),
            applied_revision=_nonnegative_int(
                readiness.get("forbidden_zone_applied_revision")
            ),
            planner_revision=_nonnegative_int(
                readiness.get("costmap_revision")
            ),
        )

    def _wait_for_forbidden_zone_runtime_ack(
        self,
        *,
        map_name: str,
        generation: int,
        signature: str,
    ) -> Optional[_ForbiddenZoneRuntimeAck]:
        timeout_sec = max(
            0.0,
            float(
                self._forbidden_zone_revision_wait_timeout_sec
            ),
        )
        poll_interval_sec = max(
            0.01,
            float(
                self._forbidden_zone_revision_poll_interval_sec
            ),
        )
        deadline = time.monotonic() + timeout_sec
        observed_ack: Optional[_ForbiddenZoneRuntimeAck] = None
        while True:
            observed_ack = self._read_forbidden_zone_runtime_ack()
            if observed_ack is not None and observed_ack.confirms(
                map_name=map_name,
                generation=generation,
                signature=signature,
            ):
                return observed_ack
            if time.monotonic() >= deadline:
                return observed_ack
            time.sleep(poll_interval_sec)

    def _ensure_forbidden_zone_motion_start_allowed(
        self,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> None:
        state_lock = self._forbidden_zone_state_lock()
        with _exploration_deadline_lock(
            state_lock,
            deadline_monotonic,
            "ForbiddenZoneMotionAdmission",
        ):
            if self._forbidden_zone_update_active:
                self._raise_forbidden_zone_motion_start_blocked_locked()
            required_generation = self._forbidden_zone_required_generation
            required_map_name = self._forbidden_zone_required_map_name
        if required_generation is None or not required_map_name:
            return

        observed_ack = self._read_forbidden_zone_runtime_ack(
            deadline_monotonic=deadline_monotonic,
        )
        with _exploration_deadline_lock(
            state_lock,
            deadline_monotonic,
            "ForbiddenZoneMotionAdmission",
        ):
            if self._forbidden_zone_update_active:
                self._raise_forbidden_zone_motion_start_blocked_locked()
            current_required_generation = self._forbidden_zone_required_generation
            current_required_map_name = self._forbidden_zone_required_map_name
            current_required_signature = self._forbidden_zone_required_signature
            if current_required_generation is None:
                return
            if (
                observed_ack is not None
                and observed_ack.confirms(
                    map_name=current_required_map_name,
                    generation=current_required_generation,
                    signature=current_required_signature or "",
                )
            ):
                self._forbidden_zone_required_generation = None
                self._forbidden_zone_required_signature = None
                self._forbidden_zone_required_map_name = None
                return
            self._raise_forbidden_zone_motion_start_blocked_locked(
                observed_ack=observed_ack
            )

    def _raise_forbidden_zone_motion_start_blocked_locked(
        self,
        *,
        observed_ack: Optional[_ForbiddenZoneRuntimeAck] = None,
    ) -> None:
        if self._forbidden_zone_update_active:
            raise GatewayError(
                "活动地图禁行区正在更新，请稍后重试。",
                error_code="forbidden_zone_update_in_progress",
                http_status=409,
            )
        required_generation = self._forbidden_zone_required_generation
        if required_generation is not None:
            raise GatewayError(
                "planner 尚未消费最新禁行区 costmap，暂不允许启动运动任务。",
                error_code="forbidden_zone_revision_pending",
                http_status=503,
                details={
                    "map_name": self._forbidden_zone_required_map_name,
                    "required_generation": required_generation,
                    "required_signature": self._forbidden_zone_required_signature,
                    "observed_map_name": (
                        observed_ack.map_name if observed_ack else None
                    ),
                    "observed_generation": (
                        observed_ack.generation if observed_ack else None
                    ),
                    "observed_signature": (
                        observed_ack.signature if observed_ack else None
                    ),
                    "forbidden_zone_applied_revision": (
                        observed_ack.applied_revision if observed_ack else None
                    ),
                    "planner_costmap_revision": (
                        observed_ack.planner_revision if observed_ack else None
                    ),
                },
            )

    def _resolve_forbidden_zone_map_name(
        self,
        map_name: Optional[str],
    ) -> str:
        resolved = str(map_name or "").strip()
        if resolved:
            return resolved

        try:
            active_session = self._localization_service.get_active_session()
        except Exception:
            active_session = None
        resolved = str(getattr(active_session, "map_name", "") or "").strip()
        if resolved:
            return resolved

        try:
            map_snapshot = self._mapping_service.get_latest_snapshot()
        except Exception:
            map_snapshot = None
        resolved = str(getattr(map_snapshot, "map_name", "") or "").strip()
        if resolved:
            return resolved
        snapshot_metadata = getattr(map_snapshot, "metadata", None)
        metadata = snapshot_metadata if isinstance(snapshot_metadata, dict) else {}
        for key in ("active_map_name", "map_name", "seed_map_name"):
            resolved = str(metadata.get(key) or "").strip()
            if resolved:
                return resolved

        for context in (
            self._current_goal,
            self._current_explore_request,
        ):
            resolved = str(getattr(context, "map_name", "") or "").strip()
            if resolved:
                return resolved
            context_metadata = getattr(context, "metadata", None)
            if not isinstance(context_metadata, dict):
                continue
            for key in ("map_name", "active_map_name", "seed_map_name"):
                resolved = str(context_metadata.get(key) or "").strip()
                if resolved:
                    return resolved

        raise GatewayError(
            "未提供 map_name，且当前没有可解析的活动地图。",
            error_code="map_name_required",
            http_status=400,
        )

    def is_navigation_available(self) -> bool:
        """判断导航后端是否可用。"""

        provider = self._find_navigation_provider()
        if provider is None or not provider.is_available():
            return False
        availability_checker = getattr(provider, "is_navigation_available", None)
        if callable(availability_checker):
            return bool(availability_checker())
        return True

    def is_exploration_available(self) -> bool:
        """判断探索后端是否可用。"""

        provider = self._find_exploration_provider()
        if provider is None or not provider.is_available():
            return False
        availability_checker = getattr(provider, "is_exploration_available", None)
        if callable(availability_checker):
            return bool(availability_checker())
        return True

    def get_navigation_control_context(self, *, deadline_monotonic: Optional[float] = None,
                                       include_readiness: bool = False) -> NavigationContext:
        """读取本次控制状态；任务捕获可同时取得规划器观测，不复用展示终态。"""
        provider = self._find_navigation_provider()
        if provider is None:
            raise GatewayError("当前平台几何运行时未提供导航后端。")
        getter = (provider.get_navigation_state if include_readiness
                  else getattr(provider, "get_navigation_control_state", provider.get_navigation_state))
        state = getter() if deadline_monotonic is None else getter(deadline_monotonic=deadline_monotonic)
        return NavigationContext(navigation_state=state, backend_name=provider.provider_name,
            goal_reached=bool(state.goal_reached), metadata={"provider_version":provider.provider_version})

    def refresh_navigation(
        self,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> NavigationContext:
        """刷新导航上下文，使用平台导航状态机管理状态转移。

        主链：NavigationGoal -> 平台当前导航后端 -> 平台 NavigationContext
        后端只负责执行反馈，不负责平台任务定义。
        """

        _exploration_deadline_remaining(deadline_monotonic, "GetNavigationState")
        provider = self._find_navigation_provider()
        if provider is None:
            raise GatewayError("当前平台几何运行时未提供导航后端。")
        if deadline_monotonic is None:
            navigation_state = provider.get_navigation_state()
        else:
            navigation_state = provider.get_navigation_state(
                deadline_monotonic=deadline_monotonic,
            )
        return self._ingest_navigation_state(
            provider,
            navigation_state,
            lightweight=deadline_monotonic is not None,
            deadline_monotonic=deadline_monotonic,
        )

    def _handle_provider_navigation_update(self, navigation_state) -> None:
        provider = self._find_navigation_provider()
        if provider is None:
            return
        try:
            self._ingest_navigation_state(provider, navigation_state)
        except Exception:
            self._provider_update_log_limiter.log(
                LOGGER, logging.ERROR, "导航提供器更新失败。",
                key="_handle_provider_navigation_update", interval_sec=5.0, exc_info=True,
            )
            return

    def _ingest_navigation_state(
        self,
        provider: NavigationProvider,
        navigation_state,
        *,
        lightweight: bool = False,
        deadline_monotonic: Optional[float] = None,
    ) -> NavigationContext:
        with _exploration_deadline_lock(
            self._lock,
            deadline_monotonic,
            "GetNavigationState",
        ):
            previous_context = self._latest_navigation_context
            current_goal = self._current_goal
            # 同步查询开始后可能已有新推送到达；旧查询不能回退新目标或抹掉停车证据。
            if previous_context is not None:
                previous = previous_context.navigation_state
                previous_metadata = dict(previous.metadata or {})
                incoming_obligation = dict(navigation_state.metadata or {}).get("dispatch_obligation") or {}
                retired_obligation = previous_metadata.get("dispatch_obligation") or {}
                if (previous_metadata.get("authority_current") is False
                        and incoming_obligation and retired_obligation
                        and incoming_obligation.get("connection_epoch") == retired_obligation.get("connection_epoch")
                        and incoming_obligation.get("identity") == retired_obligation.get("identity")):
                    # 仅丢弃失效连接原命令的迟到回包；当前流和后继目标没有这份旧责任。
                    return previous_context
                def identity(state):
                    metadata = dict(state.metadata or {})
                    try:
                        producer = int(metadata.get("producer_runtime_generation") or 0)
                        generation = int(metadata.get("goal_generation") or 0)
                    except (ValueError, TypeError):
                        return None
                    return (producer, generation) if producer > 0 and generation > 0 else None
                old_identity, incoming_identity = identity(previous), identity(navigation_state)
                if old_identity is not None and incoming_identity is not None and incoming_identity < old_identity:
                    return previous_context
                same_goal = bool(previous.current_goal_id and previous.current_goal_id == navigation_state.current_goal_id)
                same_identity = same_goal and (
                    old_identity == incoming_identity if old_identity is not None or incoming_identity is not None
                    else current_goal is None or current_goal is previous_context.current_goal)
                metadata = dict(previous.metadata or {})
                try:
                    zero_stamp = int(metadata.get("zero_applied_monotonic_ns") or 0)
                except (TypeError, ValueError):
                    zero_stamp = 0
                previous_fenced = (previous.status in {NavigationStatus.SUCCEEDED, NavigationStatus.FAILED, NavigationStatus.CANCELLED}
                    and metadata.get("actuator_zeroed") is True and bool(str(metadata.get("zero_ack_id") or "").strip()) and zero_stamp > 0)
                if same_identity:
                    if previous_fenced and navigation_state.status not in {NavigationStatus.SUCCEEDED, NavigationStatus.FAILED, NavigationStatus.CANCELLED}:
                        return previous_context
                    incoming_metadata = dict(navigation_state.metadata or {})
                    old_without_source = metadata.get("command_projection_without_source") is True
                    incoming_without_source = incoming_metadata.get("command_projection_without_source") is True
                    def full_identity(state):
                        values = dict(state.metadata or {})
                        return (state.current_goal_id, values.get("owner_kind"),
                                values.get("parent_request_id"), values.get("goal_generation"),
                                values.get("producer_runtime_generation"))
                    exact_identity = (old_identity is not None
                        and full_identity(previous) == full_identity(navigation_state))
                    if exact_identity and (old_without_source or incoming_without_source):
                        old_path = int(metadata.get("path_revision") or 0)
                        incoming_path = int(incoming_metadata.get("path_revision") or 0)
                        old_planner = int(metadata.get("planner_runtime_generation") or 0)
                        incoming_planner = int(incoming_metadata.get("planner_runtime_generation") or 0)
                        if (incoming_path < old_path
                                or (incoming_path == old_path and incoming_planner != old_planner)):
                            return previous_context
                    incoming_fenced = bool(
                        navigation_state.status in {NavigationStatus.SUCCEEDED, NavigationStatus.FAILED, NavigationStatus.CANCELLED}
                        and incoming_metadata.get("actuator_zeroed") is True
                        and str(incoming_metadata.get("zero_ack_id") or "").strip()
                        and int(incoming_metadata.get("zero_applied_monotonic_ns") or 0) > 0)
                    if previous_fenced and not incoming_fenced:
                        return previous_context
                    obligation = metadata.get("dispatch_obligation") or {}
                    if (obligation.get("terminal_observed")
                            and navigation_state.status in {
                                NavigationStatus.ACCEPTED, NavigationStatus.PLANNING, NavigationStatus.RUNNING,
                            }):
                        return previous_context
                    incoming_obligation = incoming_metadata.get("dispatch_obligation") or {}
                    stronger_terminal_observation = bool(
                        exact_identity and incoming_without_source
                        and incoming_metadata.get("ownership_retained") is True
                        and incoming_metadata.get("authority_current") is True
                        and incoming_obligation.get("terminal_observed")
                        and incoming_metadata.get("reason_code") == "GOAL_TERMINAL_PROOF_PENDING"
                        and not obligation.get("terminal_observed")
                        and not previous_fenced
                        and int(incoming_metadata.get("path_revision") or 0) >= int(metadata.get("path_revision") or 0)
                        and (int(incoming_metadata.get("path_revision") or 0) > int(metadata.get("path_revision") or 0)
                             or incoming_metadata.get("planner_runtime_generation") == metadata.get("planner_runtime_generation")))
                    # A newly observed terminal is stronger than Running. Other late
                    # source-less commands cannot roll back a sourced execution frame.
                    if (exact_identity and incoming_without_source and not old_without_source
                            and not stronger_terminal_observation):
                        return previous_context
                    if (navigation_state.updated_at < previous.updated_at
                            and not stronger_terminal_observation
                            and not (exact_identity and old_without_source and not incoming_without_source)):
                        return previous_context
                if previous_fenced and current_goal is None and navigation_state.status == NavigationStatus.IDLE:
                    return previous_context
                if (
                    same_identity and old_identity is not None
                    and current_goal is not previous_context.current_goal
                ):
                    # 同一目标 ID 可用于新尝试；旧代次仍属于旧上下文，不能挂到新提交对象。
                    current_goal = previous_context.current_goal
            reuse_terminal_state = (
                previous_context is not None
                and previous_context.navigation_state.status
                in {NavigationStatus.SUCCEEDED, NavigationStatus.FAILED, NavigationStatus.CANCELLED}
                and navigation_state.status == NavigationStatus.IDLE
                and previous_context.navigation_state.current_goal_id is not None
                and (current_goal is None or current_goal is previous_context.current_goal)
            )
            if reuse_terminal_state:
                navigation_state = previous_context.navigation_state

            # 帧中的目标与提交中的目标可不同；旧帧只能携带自己的目标，不能结束新目标。
            incoming_goal_id = navigation_state.current_goal_id
            if current_goal is not None and current_goal.goal_id != incoming_goal_id:
                current_goal = None
            if current_goal is None and previous_context is not None:
                previous_goal = previous_context.current_goal
                if previous_goal is not None and previous_goal.goal_id == incoming_goal_id:
                    current_goal = previous_goal
            if (
                self._current_goal is None
                and current_goal is not None
                and navigation_state.status in {
                    NavigationStatus.UNCONFIRMED, NavigationStatus.ACCEPTED,
                    NavigationStatus.PLANNING, NavigationStatus.RUNNING,
                }
            ):
                # 同目标的新尝试可在旧终态之后到达，复用其目标引用，不重建目标或准入凭据。
                self._current_goal = current_goal

            # 状态机只投影已通过代次/时间检查的权威帧；轻量读取与推送采用同一条路径。
            # 到达证据仍属于这一帧，不用本地旧状态覆盖后端的运行、重规划或终态。
            goal_reached = bool(navigation_state.goal_reached)
            self._state_machine.update_from_backend_state(
                backend_name=provider.provider_name,
                backend_status=navigation_state.status.value,
                message=navigation_state.message,
                goal_reached=goal_reached,
                goal_id=incoming_goal_id,
            )
            if lightweight:
                context = NavigationContext(
                    current_goal=current_goal,
                    navigation_state=navigation_state,
                    backend_name=provider.provider_name,
                    goal_reached=goal_reached,
                    metadata={"provider_version": provider.provider_version},
                )
                self._latest_navigation_context = context
                self._navigation_history.append(context)
                if (
                    navigation_state.status in {
                        NavigationStatus.SUCCEEDED, NavigationStatus.FAILED, NavigationStatus.CANCELLED,
                    }
                    and self._current_goal is current_goal
                ):
                    self._current_goal = None
                return context

            diagnostics = self._build_navigation_diagnostics(provider, navigation_state)
            self._last_navigation_diagnostics = diagnostics

            runtime_metadata = self._build_map_runtime_metadata(
                # 已结束的旧尝试可与新地图并存，只让当前拥有方约束活动地图。
                map_name=(current_goal.map_name
                    if current_goal is not None and self._current_goal is current_goal else None),
                existing_waiting_for=navigation_state.metadata.get("waiting_for"),
                refresh_localization=False,
            )
            runtime_metadata["navigation_state_machine"] = self._state_machine.get_state_summary()

            navigation_state = navigation_state.model_copy(
                update={
                    "metadata": {
                        **dict(navigation_state.metadata),
                        **runtime_metadata,
                    }
                },
                deep=False,
            )
            context = NavigationContext(
                current_goal=current_goal,
                navigation_state=navigation_state,
                backend_name=provider.provider_name,
                goal_reached=goal_reached,
                metadata={
                    "provider_version": provider.provider_version,
                    "diagnostics": diagnostics.model_dump(mode="json"),
                    **runtime_metadata,
                },
            )
            self._latest_navigation_context = context
            self._navigation_history.append(context)
            self._state_store.write(
                StateNamespace.NAVIGATION,
                "navigation",
                context,
                source=provider.provider_name,
                metadata={"kind": "latest_navigation"},
            )
            if self._stream_bus is not None:
                navigation_status = str(getattr(context.navigation_state.status, "value", context.navigation_state.status))
                self._stream_bus.publish(
                    "navigation.state",
                    source="navigation_service",
                    payload=context,
                    frame_id=getattr(getattr(context.navigation_state, "current_pose", None), "frame_id", None),
                    status="available",
                    metadata={"backend_name": provider.provider_name, "navigation_status": navigation_status},
                )
            if (
                context.navigation_state.status in {
                    NavigationStatus.SUCCEEDED, NavigationStatus.FAILED, NavigationStatus.CANCELLED,
                }
                and self._current_goal is current_goal
            ):
                self._current_goal = None
            return context

    @staticmethod
    def _resolved_navigation_command_context(provider, goal, receipt, *, lightweight):
        """Return this command's proof without publishing or restoring old authority."""
        receipt = receipt.model_copy(update={"metadata": {
            **dict(receipt.metadata), "authority_current": False,
            "ownership_retained": False, "command_receipt_only": True,
        }}, deep=False)
        context_metadata = {"provider_version": provider.provider_version,
                            "command_receipt_only": True}
        if not lightweight:
            # Use a private projection, never the service's possibly newer state machine.
            machine = NavigationStateMachine()
            machine.update_from_backend_state(provider.provider_name, receipt.status.value,
                receipt.message, receipt.goal_reached, goal.goal_id)
            summary = machine.get_state_summary()
            diagnostics = NavigationDiagnostics(
                backend_name=provider.provider_name, backend_status=receipt.status.value,
                platform_status=receipt.status, goal_id=goal.goal_id,
                remaining_distance_m=receipt.remaining_distance_m,
                remaining_yaw_rad=receipt.remaining_yaw_rad,
                last_error=receipt.message if receipt.status == NavigationStatus.FAILED else None,
                metadata={"is_running": False, "is_terminal": True, "command_receipt_only": True},
            )
            context_metadata.update(diagnostics=diagnostics.model_dump(mode="json"),
                                    navigation_state_machine=summary, map_name=goal.map_name)
            receipt = receipt.model_copy(update={"metadata": {
                **dict(receipt.metadata), "navigation_state_machine": summary,
            }}, deep=False)
        return NavigationContext(
            timestamp=receipt.timestamp, current_goal=goal, navigation_state=receipt,
            backend_name=provider.provider_name, goal_reached=receipt.goal_reached,
            metadata=context_metadata,
        )

    def set_goal(
        self,
        goal: NavigationGoal,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> NavigationContext:
        """提交导航目标，使用平台状态机管理目标接受流程。

        主链：NavigationGoal -> 平台状态机 ACCEPTED -> 当前导航后端执行 -> 平台 NavigationContext
        """

        if deadline_monotonic is None:
            self._ensure_forbidden_zone_motion_start_allowed()
        else:
            self._ensure_forbidden_zone_motion_start_allowed(
                deadline_monotonic=deadline_monotonic,
            )
        if deadline_monotonic is None:
            provider = self._get_navigation_provider()
        else:
            provider = self._get_navigation_provider(
                deadline_monotonic=deadline_monotonic,
            )
        if goal.target_pose is not None:
            LOGGER.info(
                "导航服务接收目标 goal_id=%s provider=%s target=(%.3f, %.3f, %.3f) frame=%s map_name=%s target_name=%s metadata=%s",
                goal.goal_id,
                provider.provider_name,
                float(goal.target_pose.position.x),
                float(goal.target_pose.position.y),
                float(goal.target_pose.position.z),
                goal.target_pose.frame_id,
                goal.map_name,
                goal.target_name,
                dict(goal.metadata),
            )
        else:
            LOGGER.info(
                "导航服务接收目标 goal_id=%s provider=%s target_pose=None map_name=%s target_name=%s metadata=%s",
                goal.goal_id,
                provider.provider_name,
                goal.map_name,
                goal.target_name,
                dict(goal.metadata),
            )
        with _exploration_deadline_lock(
            self._lock,
            deadline_monotonic,
            "SetNavigationGoal",
        ):
            self._raise_forbidden_zone_motion_start_blocked_locked()
            previous_goal = self._current_goal
            previous_context = self._latest_navigation_context
            self._current_goal = goal
            self._state_machine.set_goal(goal.goal_id, message="平台导航目标已接受。")
        LOGGER.debug("导航服务转发目标 goal_id=%s provider=%s", goal.goal_id, provider.provider_name)
        if deadline_monotonic is None:
            accepted = provider.set_goal(goal)
        else:
            accepted = provider.set_goal(
                goal,
                deadline_monotonic=deadline_monotonic,
            )
        take_result = getattr(provider, "take_goal_submission_result", None)
        submission = take_result(goal.goal_id) if callable(take_result) else None
        # Confirmed submissions use the original refresh path below. Their
        # trusted responsibility metadata is already committed in the provider cache.
        if (isinstance(submission, dict) and submission.get("goal_id") == goal.goal_id
                and submission.get("disposition") in {"owned_unconfirmed", "resolved"}
                and isinstance(submission.get("navigation"), dict)):
            wire = dict(submission["navigation"])
            source_ns = int(wire.get("timestamp_ns") or 0)
            timestamp = (datetime.fromtimestamp(source_ns / 1_000_000_000, tz=timezone.utc)
                         if source_ns > 0 else datetime.now(timezone.utc))
            raw_submission_status = str(wire.get("status") or "idle").strip().lower()
            status = NavigationStatus("accepted" if raw_submission_status == "pending" else raw_submission_status)
            metadata = {key: value for key, value in wire.items()
                        if key not in {"status", "goal_id", "message", "timestamp_ns", "distance_remaining"}}
            metadata.update(raw_status=wire.get("status", ""), observed_goal_id=wire.get("goal_id", ""),
                            source_timestamp_ns=source_ns,
                            ownership_retained=bool(submission.get("ownership_retained")),
                            authority_current=bool(submission.get("authority_current")))
            submitted_state = NavigationState(
                current_goal_id=wire.get("goal_id") or goal.goal_id, status=status,
                timestamp=timestamp, updated_at=timestamp, message=wire.get("message"),
                remaining_distance_m=wire.get("distance_remaining"),
                goal_reached=status == NavigationStatus.SUCCEEDED, metadata=metadata,
            )
            context = self._ingest_navigation_state(provider, submitted_state,
                lightweight=deadline_monotonic is not None, deadline_monotonic=deadline_monotonic)
            selected_state = context.navigation_state
            selected_metadata = dict(selected_state.metadata or {})
            selected_exact = bool(selected_state.current_goal_id == wire.get("goal_id")
                and all(selected_metadata.get(key) == wire.get(key) for key in (
                    "owner_kind", "parent_request_id", "goal_generation", "producer_runtime_generation")))
            selected_complete = bool(selected_exact
                and selected_state.status in {NavigationStatus.SUCCEEDED, NavigationStatus.FAILED, NavigationStatus.CANCELLED}
                and selected_metadata.get("actuator_zeroed") is True
                and str(selected_metadata.get("zero_ack_id") or "").strip()
                and int(selected_metadata.get("zero_applied_monotonic_ns") or 0) > 0)
            # A stream can win after provider commit but before this command is consumed.
            # Consume the selected exact proof/real acceptance, not the obsolete unary label.
            if selected_complete:
                return context
            if submission["disposition"] == "resolved":
                # A newer owner/epoch may have won the shared context while the unary
                # result was being consumed. This original receipt remains the command
                # result; returning a foreign context would lose its exact proof.
                return self._resolved_navigation_command_context(provider, goal, submitted_state,
                    lightweight=deadline_monotonic is not None)
            if submission["disposition"] == "owned_unconfirmed":
                selected_accepted = bool(selected_exact
                    and selected_metadata.get("authority_current") is True
                    and not selected_metadata.get("command_projection_without_source")
                    and selected_state.status in {
                        NavigationStatus.ACCEPTED, NavigationStatus.PLANNING, NavigationStatus.RUNNING,
                    })
                if selected_accepted:
                    accepted = True
                else:
                    selected_wire = dict(wire)
                    if selected_state.current_goal_id == goal.goal_id:
                        for key in tuple(selected_wire):
                            if key in selected_metadata:
                                selected_wire[key] = selected_metadata[key]
                        selected_wire.update(status=selected_state.status.value,
                            goal_id=selected_state.current_goal_id, message=selected_state.message or "",
                            timestamp_ns=int(selected_metadata.get("source_timestamp_ns") or 0),
                            distance_remaining=selected_state.remaining_distance_m)
                        for key in ("dispatch_obligation", "observed_status", "historical_navigation_identity", "historical_navigation"):
                            if key in selected_metadata:
                                selected_wire[key] = selected_metadata[key]
                    raise GatewayError(selected_wire.get("message") or "导航目标发布尚未确认，原责任仍未结算。",
                        error_code="navigation_dispatch_unconfirmed", http_status=409,
                        details={"goal_id": goal.goal_id, "navigation": selected_wire,
                                 "reason_code": selected_wire.get("reason_code") or submission.get("reason_code"),
                                 "ownership_retained": True,
                                 "authority_current": bool(selected_wire.get("authority_current")),
                                 "retryable": False})
        if not accepted:
            message = "导航后端拒绝目标 %s。" % goal.goal_id
            details = {"goal_id": goal.goal_id}
            if submission and submission.get("goal_id") == goal.goal_id and not submission.get("accepted"):
                message = submission.get("message") or message
                details["backend_message"] = message
                # 只解析本次事务回执中的机器代码，不从共享的最后状态猜测。
                import re
                match = re.search(r"SET_GOAL_REJECTED\[([A-Z0-9_]+)\]", message)
                if match:
                    details["backend_code"] = match.group(1)
            with _exploration_deadline_lock(
                self._lock, deadline_monotonic, "SetNavigationGoal",
            ):
                if self._current_goal is goal:
                    if submission and submission.get("reason_code") == "BUSY":
                        self._current_goal = previous_goal
                        self._latest_navigation_context = previous_context
                        if previous_context is not None:
                            old = previous_context.navigation_state
                            self._state_machine.update_from_backend_state(provider.provider_name,
                                old.status.value, old.message, old.goal_reached, old.current_goal_id)
                    else:
                        self._current_goal = None
                        self._state_machine.fail(message)
            raise GatewayError(message, error_code="navigation_goal_rejected", details=details)
        LOGGER.info("导航服务目标已被后端接收 goal_id=%s provider=%s", goal.goal_id, provider.provider_name)
        if deadline_monotonic is None:
            context = self.refresh_navigation()
        else:
            context = self.refresh_navigation(
                deadline_monotonic=deadline_monotonic,
            )
        if context.navigation_state.status == NavigationStatus.IDLE:
            context = context.model_copy(
                update={
                    "navigation_state": context.navigation_state.model_copy(
                        update={
                            "current_goal_id": goal.goal_id,
                            "status": NavigationStatus.ACCEPTED,
                            "message": "导航目标已接受。",
                        },
                        deep=False,
                    )
                },
                deep=False,
            )
            self._latest_navigation_context = context
        if deadline_monotonic is None:
            self._publish_navigation_event("navigation.goal_set", f"导航目标 {goal.goal_id} 已提交。", goal_id=goal.goal_id)
        return context

    def cancel_goal(
        self,
        request_id: Optional[str] = None,
        *,
        deadline_monotonic: Optional[float] = None,
        global_stop_confirmation: Optional[StopAllMotionConfirmation] = None,
    ) -> NavigationContext:
        """按精确目标标识直接请求后端取消，再读取其控制结果。"""

        exact_goal_id = str(request_id or "").strip()
        if not exact_goal_id:
            raise GatewayError(
                "导航取消缺少 exact goal_id。",
                error_code="navigation_goal_identity_unavailable",
                http_status=409,
            )
        provider = self._find_navigation_provider()
        if provider is None:
            raise GatewayError("当前平台几何运行时未提供导航后端。")
        cancel_options = {}
        if deadline_monotonic is not None:
            cancel_options["deadline_monotonic"] = deadline_monotonic
        if global_stop_confirmation is not None:
            cancel_options["global_stop_confirmation"] = global_stop_confirmation
        accepted = provider.cancel_goal(request_id=exact_goal_id, **cancel_options)
        if not accepted:
            raise GatewayError(
                "导航后端拒绝取消当前目标。",
                error_code="navigation_cancel_rejected",
                http_status=409,
            )
        if deadline_monotonic is None:
            context = self.get_navigation_control_context()
        else:
            context = self.get_navigation_control_context(
                deadline_monotonic=deadline_monotonic,
            )
        if context.navigation_state.status == NavigationStatus.IDLE and self._current_goal is not None:
            context = context.model_copy(
                update={
                    "navigation_state": context.navigation_state.model_copy(
                        update={
                            "current_goal_id": self._current_goal.goal_id,
                            "status": NavigationStatus.CANCELLED,
                            "message": "导航目标已取消。",
                        },
                        deep=False,
                    )
                },
                deep=False,
            )
            self._latest_navigation_context = context
        if deadline_monotonic is None:
            self._publish_navigation_event(
                "navigation.goal_cancelled",
                "导航目标已取消。",
                goal_id=exact_goal_id,
            )
        return context

    def get_navigation_diagnostics(self) -> Optional[NavigationDiagnostics]:
        """返回当前平台导航诊断信息。"""

        return self._last_navigation_diagnostics

    def is_goal_reached(self) -> bool:
        """判断当前目标是否已到达。"""

        if self._latest_navigation_context is not None:
            return self._latest_navigation_context.goal_reached
        provider = self._get_navigation_provider()
        return provider.is_goal_reached()

    def navigate_until_complete(
        self,
        goal: NavigationGoal,
        *,
        poll_interval_sec: float = 0.2,
        timeout_sec: Optional[float] = None,
        on_progress=None,
    ) -> NavigationContext:
        """同步等待导航目标达到终态。

        Args:
            timeout_sec: 任务级超时上限（秒）。超时后下发 cancel_goal 并抛 GatewayError，
                避免后端永驻 RUNNING 导致调用方线程永久阻塞。None 表示不限时。
        """

        self.set_goal(goal)
        deadline = None if timeout_sec is None or timeout_sec <= 0 else time.monotonic() + float(timeout_sec)
        while True:
            context = self.refresh_navigation()
            if callable(on_progress):
                on_progress(context)
            state = context.navigation_state.status
            if context.goal_reached:
                return context
            if state == NavigationStatus.SUCCEEDED:
                raise GatewayError(context.navigation_state.message or "导航后端报告完成，但平台未确认已到达目标。")
            if state == NavigationStatus.FAILED:
                raise GatewayError(context.navigation_state.message or "导航执行失败。")
            if state == NavigationStatus.CANCELLED:
                raise GatewayError(context.navigation_state.message or "导航已取消。")
            if deadline is not None and time.monotonic() >= deadline:
                try:
                    self.cancel_goal(request_id=goal.goal_id)
                except Exception:
                    LOGGER.exception("导航超时取消失败 goal_id=%s", goal.goal_id)
                raise GatewayError(
                    "导航在 %.1fs 内未达终态，已下发取消。" % float(timeout_sec),
                    error_code="navigation_timeout",
                )
            time.sleep(max(0.01, poll_interval_sec))

    def resolve_named_goal(self, target_name: str) -> NavigationGoal:
        """把命名位置解析成标准导航目标。"""

        map_snapshot = self._mapping_service.get_latest_snapshot()
        if map_snapshot is None or map_snapshot.semantic_map is None:
            raise GatewayError("当前没有可用于命名导航的语义地图。")

        normalized_target = target_name.strip().lower()
        for region in map_snapshot.semantic_map.regions:
            aliases = [str(region.attributes.get("alias", ""))]
            aliases.extend(str(item) for item in region.attributes.get("aliases", []) if item)
            candidates = {region.region_id.strip().lower(), region.label.strip().lower(), *(item.strip().lower() for item in aliases)}
            if normalized_target not in candidates:
                continue
            if region.centroid is None:
                raise GatewayError(f"语义区域 {target_name} 没有可导航的中心位姿。")
            try:
                resolved_map_name = self._resolve_forbidden_zone_map_name(None)
            except GatewayError:
                resolved_map_name = None
            if (
                resolved_map_name is not None
                and map_snapshot.map_name != resolved_map_name
            ):
                raise GatewayError(
                    "当前语义地图与激活的 map_name 不一致。",
                    error_code="map_snapshot_name_mismatch",
                )
            goal_metadata = {
                "resolved_region_id": region.region_id,
                "map_name": map_snapshot.map_name,
                "map_snapshot_identity": {
                    "schema_version": map_snapshot.schema_version,
                    "map_name": map_snapshot.map_name,
                    "source_name": map_snapshot.source_name,
                    "source_epoch": map_snapshot.source_epoch,
                    "frame_id": map_snapshot.frame_id,
                    "update_sequence": str(map_snapshot.update_sequence),
                },
            }
            return NavigationGoal(
                goal_id=f"nav_named_{self._normalize_name(target_name)}",
                target_pose=region.centroid,
                target_name=target_name,
                metadata=goal_metadata,
            )
        raise GatewayError(f"未在语义地图中找到命名位置：{target_name}")

    def refresh_exploration(
        self,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> ExplorationContext:
        """刷新探索上下文。"""

        _exploration_deadline_remaining(
            deadline_monotonic,
            "GetExplorationState",
        )
        if deadline_monotonic is None:
            provider = self._get_exploration_provider()
        else:
            provider = self._get_exploration_provider(
                deadline_monotonic=deadline_monotonic,
            )
        if deadline_monotonic is None:
            exploration_state = provider.get_exploration_state()
        else:
            exploration_state = provider.get_exploration_state(
                deadline_monotonic=deadline_monotonic,
            )
        return self._ingest_exploration_state(
            provider,
            exploration_state,
            lightweight=deadline_monotonic is not None,
            deadline_monotonic=deadline_monotonic,
        )

    def _handle_provider_exploration_update(self, exploration_state) -> None:
        provider = self._find_exploration_provider()
        if provider is None:
            return
        try:
            exploration_metadata = dict(getattr(exploration_state, "metadata", {}) or {})
            status_value = getattr(exploration_state.status, "value", exploration_state.status)
            lightweight = bool(
                exploration_metadata.get("native_go2_frontier_explorer")
                and status_value == ExplorationStatus.RUNNING.value
            )
            self._ingest_exploration_state(provider, exploration_state, lightweight=lightweight)
        except Exception:
            self._provider_update_log_limiter.log(
                LOGGER, logging.ERROR, "探索提供器更新失败。",
                key="_handle_provider_exploration_update", interval_sec=5.0, exc_info=True,
            )
            return

    def _ingest_exploration_state(
        self,
        provider: ExplorationProvider,
        exploration_state,
        *,
        lightweight: bool = False,
        deadline_monotonic: Optional[float] = None,
    ) -> ExplorationContext:
        with _exploration_deadline_lock(
            self._lock,
            deadline_monotonic,
            "GetExplorationState",
        ):
            previous_context = self._latest_exploration_context
            with _exploration_deadline_lock(
                _exploration_owner_lock(self),
                deadline_monotonic,
                "GetExplorationState",
            ):
                current_request = _without_exploration_map_identity(
                    self._current_explore_request
                )
            idle_without_request = bool(
                exploration_state.status == ExplorationStatus.IDLE
                and current_request is None
            )
            previous_request_id = str(
                getattr(
                    getattr(previous_context, "current_request", None),
                    "request_id",
                    "",
                )
                or getattr(
                    getattr(previous_context, "exploration_state", None),
                    "current_request_id",
                    "",
                )
                or ""
            ).strip()
            if (
                previous_context is not None
                and _is_complete_exploration_terminal(
                    previous_context.exploration_state,
                    request_id=previous_request_id,
                )
                and exploration_state.status == ExplorationStatus.IDLE
                and previous_context.exploration_state.current_request_id is not None
            ):
                exploration_state = previous_context.exploration_state.model_copy(deep=True)
                current_request = current_request or previous_context.current_request
                idle_without_request = False
            if lightweight and previous_context is not None:
                runtime_metadata = dict(previous_context.metadata or {})
            elif lightweight:
                runtime_metadata = {}
            else:
                runtime_metadata = self._build_map_runtime_metadata(
                    map_name=current_request.map_name if current_request is not None else None,
                    memory_library_name=current_request.map_name if current_request is not None else None,
                    existing_waiting_for=None if idle_without_request else exploration_state.metadata.get("waiting_for"),
                    include_waiting_for=not idle_without_request,
                )
            merged_metadata = {
                **dict(exploration_state.metadata),
                **runtime_metadata,
            }
            runtime_metadata.pop("map_version_id", None)
            runtime_metadata.pop("map_revision", None)
            runtime_metadata.pop("stop_on_map_revision_change", None)
            merged_metadata.pop("map_version_id", None)
            merged_metadata.pop("map_revision", None)
            merged_metadata.pop("stop_on_map_revision_change", None)
            if idle_without_request:
                merged_metadata.pop("waiting_for", None)
                merged_metadata.pop("failure_code", None)
            exploration_state = exploration_state.model_copy(
                update={
                    "metadata": merged_metadata
                },
                deep=False,
            )
            context = ExplorationContext(
                current_request=current_request,
                exploration_state=exploration_state,
                backend_name=provider.provider_name,
                metadata={
                    "provider_version": provider.provider_version,
                    **runtime_metadata,
                },
            )
            self._latest_exploration_context = context
            self._exploration_history.append(context)
            context_request_id = str(
                getattr(context.current_request, "request_id", "") or ""
            ).strip()
            if _is_complete_exploration_terminal(
                context.exploration_state,
                request_id=context_request_id,
            ):
                with _exploration_deadline_lock(
                    _exploration_owner_lock(self),
                    deadline_monotonic,
                    "GetExplorationState",
                ):
                    self._current_explore_request = None
                    if self._pending_explore_request_id == context_request_id:
                        self._pending_explore_request_id = None
            if lightweight:
                return context
            self._state_store.write(
                StateNamespace.NAVIGATION,
                "exploration",
                context,
                source=provider.provider_name,
                metadata={"kind": "latest_exploration"},
            )
            if self._stream_bus is not None:
                exploration_status = str(getattr(context.exploration_state.status, "value", context.exploration_state.status))
                self._stream_bus.publish(
                    "exploration.state",
                    source="navigation_service",
                    payload=context,
                    status="available",
                    metadata={"backend_name": provider.provider_name, "exploration_status": exploration_status},
                )
                exploration_metadata = dict(getattr(context.exploration_state, "metadata", {}) or {})
                frontiers = exploration_metadata.get("latest_frontiers")
                current_frontier = exploration_metadata.get("current_frontier")
                observer_meta = exploration_metadata.get("external_frontier_debug_observer")
                geometry_session_mode = str(
                    exploration_metadata.get("geometry_session_mode") or "",
                ).strip().lower()
                should_publish_frontier = geometry_session_mode == "explore" and (
                    frontiers is not None
                    or current_frontier is not None
                    or bool(exploration_metadata.get("exploration_terminal"))
                    or bool(exploration_metadata.get("status_text"))
                    or isinstance(observer_meta, dict)
                )
                if should_publish_frontier:
                    frontier_payload = {
                        "latest_frontiers": frontiers if isinstance(frontiers, list) else [],
                        "current_frontier": current_frontier if isinstance(current_frontier, dict) else None,
                        "frontier_count": context.exploration_state.frontier_count,
                        "revision": exploration_metadata.get("revision"),
                        "selected_source": exploration_metadata.get("selected_source"),
                        "last_stop_reason": exploration_metadata.get("last_stop_reason"),
                        "child_task_id": exploration_metadata.get("child_task_id"),
                        "state_source": exploration_metadata.get("state_source"),
                        "geometry_session_mode": geometry_session_mode or "explore",
                        "status_text": exploration_metadata.get("status_text"),
                        "exploration_terminal": bool(exploration_metadata.get("exploration_terminal")),
                        "terminal_status": exploration_metadata.get("terminal_status"),
                        "planned_path": exploration_metadata.get("planned_path") if isinstance(exploration_metadata.get("planned_path"), list) else [],
                        # current_goal 已在源头解析为 NavigationGoal|None，直接透传。
                        "current_goal": exploration_metadata.get("current_goal") if isinstance(exploration_metadata.get("current_goal"), NavigationGoal) else None,
                        "external_frontier_debug_observer": observer_meta,
                    }
                    frontier_sequence = None
                    if isinstance(observer_meta, dict):
                        try:
                            frontier_sequence = int(observer_meta.get("sequence"))
                        except (TypeError, ValueError):
                            frontier_sequence = None
                    self._stream_bus.publish(
                        "exploration.frontier",
                        source="navigation_service",
                        payload=frontier_payload,
                        sequence=frontier_sequence,
                        status="available",
                        metadata={
                            "backend_name": provider.provider_name,
                            "exploration_status": exploration_status,
                            "exploration_terminal": bool(exploration_metadata.get("exploration_terminal")),
                            "revision": exploration_metadata.get("revision"),
                        },
                    )
                elif geometry_session_mode != "explore" and self._frontier_stream_has_active_geometry():
                    self._publish_cleared_exploration_frontier(
                        geometry_session_mode=geometry_session_mode,
                        reason="geometry_session_not_explore",
                    )
                self._publish_external_exploration_navigation_overlay(context)
            return context

    def start_exploration(
        self,
        request: ExploreAreaRequest,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> ExplorationContext:
        """启动探索任务。"""

        _exploration_deadline_remaining(
            deadline_monotonic,
            "StartExploration",
        )
        request = _without_exploration_map_identity(request) or request
        if deadline_monotonic is None:
            self._ensure_forbidden_zone_motion_start_allowed()
        else:
            self._ensure_forbidden_zone_motion_start_allowed(
                deadline_monotonic=deadline_monotonic,
            )
        if deadline_monotonic is None:
            provider = self._get_exploration_provider()
        else:
            provider = self._get_exploration_provider(
                deadline_monotonic=deadline_monotonic,
            )
        request_id = str(request.request_id or "").strip()
        registered_new_owner = False
        with _exploration_deadline_lock(
            self._lock,
            deadline_monotonic,
            "StartExploration",
        ):
            with _exploration_deadline_lock(
                _exploration_owner_lock(self),
                deadline_monotonic,
                "StartExploration",
            ):
                self._raise_forbidden_zone_motion_start_blocked_locked()
                current_id = str(
                    getattr(self._current_explore_request, "request_id", "") or ""
                ).strip()
                if (current_id and current_id != request_id) or (
                    self._pending_explore_request_id
                    and self._pending_explore_request_id != request_id
                ):
                    raise GatewayError("已有其它探索请求正在启动或运行。")
                registered_new_owner = not bool(current_id)
                self._pending_explore_request_id = request_id
                self._current_explore_request = request
        try:
            _exploration_deadline_remaining(
                deadline_monotonic,
                "StartExploration",
            )
            if deadline_monotonic is None:
                accepted = provider.start_exploration(request)
            else:
                accepted = provider.start_exploration(
                    request,
                    deadline_monotonic=deadline_monotonic,
                )
        except Exception as exc:
            if getattr(exc, "exploration_rpc_dispatched", True) is False:
                with _exploration_deadline_lock(
                    _exploration_owner_lock(self),
                    deadline_monotonic,
                    "StartExploration",
                ):
                    if self._pending_explore_request_id == request_id:
                        self._pending_explore_request_id = None
                    current_id = str(
                        getattr(
                            self._current_explore_request,
                            "request_id",
                            "",
                        )
                        or ""
                    ).strip()
                    if registered_new_owner and current_id == request_id:
                        self._current_explore_request = None
            raise
        if not accepted:
            message = f"探索后端拒绝请求 {request.request_id}。"
            try:
                if deadline_monotonic is None:
                    exploration_state = provider.get_exploration_state()
                else:
                    exploration_state = provider.get_exploration_state(
                        deadline_monotonic=deadline_monotonic,
                    )
            except Exception:
                exploration_state = None
            if exploration_state is not None and exploration_state.message:
                message = exploration_state.message
            with _exploration_deadline_lock(
                _exploration_owner_lock(self),
                deadline_monotonic,
                "StartExploration",
            ):
                if self._pending_explore_request_id == request_id:
                    self._pending_explore_request_id = None
                current_id = str(
                    getattr(self._current_explore_request, "request_id", "") or ""
                ).strip()
                if registered_new_owner and current_id == request_id:
                    self._current_explore_request = None
            _exploration_deadline_remaining(
                deadline_monotonic,
                "StartExploration",
            )
            raise GatewayError(message)
        with _exploration_deadline_lock(
            self._lock,
            deadline_monotonic,
            "StartExploration",
        ):
            with _exploration_deadline_lock(
                _exploration_owner_lock(self),
                deadline_monotonic,
                "StartExploration",
            ):
                if self._pending_explore_request_id != request_id:
                    raise GatewayError("探索启动确认与本地 pending request 不匹配。")
                current_id = str(
                    getattr(self._current_explore_request, "request_id", "") or ""
                ).strip()
                if current_id != request_id:
                    raise GatewayError("探索启动确认与本地 current request 不匹配。")
                self._pending_explore_request_id = None
        if deadline_monotonic is None:
            context = self.refresh_exploration()
        else:
            context = self.refresh_exploration(
                deadline_monotonic=deadline_monotonic,
            )
        if context.exploration_state.status == ExplorationStatus.IDLE:
            context = context.model_copy(
                update={
                    "exploration_state": context.exploration_state.model_copy(
                        update={
                            "current_request_id": request.request_id,
                            "status": ExplorationStatus.ACCEPTED,
                            "strategy": request.strategy,
                            "message": "探索任务已接受。",
                        },
                        deep=False,
                    )
                },
                deep=False,
            )
            self._latest_exploration_context = context
        return context

    def stop_exploration(
        self,
        request_id: Optional[str] = None,
        *,
        deadline_monotonic: Optional[float] = None,
        global_stop_confirmation: Optional[StopAllMotionConfirmation] = None,
        expected_exploration_generation: Optional[int] = None,
    ) -> ExplorationContext:
        """停止当前探索任务。"""

        _exploration_deadline_remaining(
            deadline_monotonic,
            "StopExploration",
        )
        correlated_request_id = str(request_id or "").strip()
        with _exploration_deadline_lock(
            self._lock,
            deadline_monotonic,
            "StopExploration",
        ):
            with _exploration_deadline_lock(
                _exploration_owner_lock(self),
                deadline_monotonic,
                "StopExploration",
            ):
                previous_request = self._current_explore_request
                if previous_request is None and self._latest_exploration_context is not None:
                    previous_request = self._latest_exploration_context.current_request
        if deadline_monotonic is None:
            provider = self._get_exploration_provider()
        else:
            provider = self._get_exploration_provider(
                deadline_monotonic=deadline_monotonic,
            )
        if (
            correlated_request_id
            and previous_request is not None
            and previous_request.request_id != correlated_request_id
        ):
            raise GatewayError(
                "停止探索 request_id 与当前会话不匹配。",
                error_code="exploration_request_mismatch",
                http_status=409,
                details={
                    "request_id": correlated_request_id,
                    "active_request_id": previous_request.request_id,
                },
            )
        stop_options = {}
        if global_stop_confirmation is not None:
            stop_options["global_stop_confirmation"] = global_stop_confirmation
        if expected_exploration_generation is not None:
            stop_options["expected_exploration_generation"] = expected_exploration_generation
        if correlated_request_id:
            try:
                accepted = provider.stop_exploration(
                    **stop_options,
                    request_id=correlated_request_id,
                    **(
                        {"deadline_monotonic": deadline_monotonic}
                        if deadline_monotonic is not None
                        else {}
                    ),
                )
            except TypeError as exc:
                raise GatewayError(
                    "探索后端不支持 correlated StopExploration request_id。",
                    error_code="stop_exploration_request_id_unsupported",
                    http_status=501,
                    details={"request_id": correlated_request_id},
                ) from exc
        else:
            if deadline_monotonic is None:
                accepted = provider.stop_exploration(**stop_options)
            else:
                accepted = provider.stop_exploration(
                    **stop_options,
                    deadline_monotonic=deadline_monotonic,
                )
        if not accepted:
            raise GatewayError("探索后端拒绝停止当前任务。")
        if deadline_monotonic is None:
            context = self.refresh_exploration()
        else:
            context = self.refresh_exploration(
                deadline_monotonic=deadline_monotonic,
            )

        if previous_request is not None and context.current_request is None:
            state_updates = {}
            if context.exploration_state.current_request_id is None:
                state_updates["current_request_id"] = previous_request.request_id
            if context.exploration_state.strategy is None:
                state_updates["strategy"] = previous_request.strategy
            exploration_state = context.exploration_state
            if state_updates:
                exploration_state = exploration_state.model_copy(
                    update=state_updates,
                    deep=False,
                )
            context = context.model_copy(
                update={
                    "current_request": previous_request,
                    "exploration_state": exploration_state,
                },
                deep=False,
            )
            self._latest_exploration_context = context

        if previous_request is not None and _is_complete_exploration_terminal(
            context.exploration_state,
            request_id=previous_request.request_id,
        ):
            self._current_explore_request = None
        return context




    def sync_exploration_after_task(self) -> ExplorationContext:
        """探索任务自然结束后的轻量同步：不杀 go2.sh，不标 CANCELLED。"""

        provider = self._get_exploration_provider()
        sync_hook = getattr(provider, "sync_exploration_after_task", None)
        if callable(sync_hook):
            sync_hook()
        else:
            provider.stop_exploration()
        return self.refresh_exploration()

    def explore_until_complete(
        self,
        request: ExploreAreaRequest,
        *,
        poll_interval_sec: float = 0.2,
        timeout_sec: Optional[float] = None,
        on_progress=None,
    ) -> ExplorationContext:
        """同步等待探索任务达到终态。

        Args:
            timeout_sec: 任务级超时上限（秒）。超时后下发 stop_exploration 并抛 GatewayError，
                避免探索后端永驻 RUNNING 导致调用方线程永久阻塞。None 表示不限时。
        """

        self.start_exploration(request)
        deadline = None if timeout_sec is None or timeout_sec <= 0 else time.monotonic() + float(timeout_sec)
        while True:
            context = self.refresh_exploration()
            if callable(on_progress):
                on_progress(context)
            state = context.exploration_state.status
            terminal = _is_complete_exploration_terminal(
                context.exploration_state,
                request_id=request.request_id,
            )
            if terminal:
                if state == ExplorationStatus.SUCCEEDED:
                    return context
                if state == ExplorationStatus.FAILED:
                    raise GatewayError(
                        context.exploration_state.message or "探索执行失败。"
                    )
                if state == ExplorationStatus.CANCELLED:
                    raise GatewayError(
                        context.exploration_state.message or "探索已取消。"
                    )
            if deadline is not None and time.monotonic() >= deadline:
                try:
                    self.stop_exploration()
                except Exception:
                    LOGGER.exception("探索超时停止失败 request_id=%s", request.request_id)
                raise GatewayError(
                    "探索在 %.1fs 内未达终态，已下发停止。" % float(timeout_sec),
                    error_code="exploration_timeout",
                )
            time.sleep(max(0.01, poll_interval_sec))

    def _frontier_stream_has_active_geometry(self) -> bool:
        """判断 exploration.frontier 最新帧是否仍携带可展示的前沿几何。"""

        if self._stream_bus is None:
            return False
        frame = self._stream_bus.latest_frame("exploration.frontier")
        if frame is None:
            return False
        payload = frame.payload if isinstance(getattr(frame, "payload", None), dict) else {}
        if payload.get("frontier_cleared"):
            return False
        frame_metadata = frame.metadata if isinstance(getattr(frame, "metadata", None), dict) else {}
        if frame_metadata.get("frontier_cleared"):
            return False
        frontiers = payload.get("latest_frontiers")
        if isinstance(frontiers, list) and frontiers:
            return True
        try:
            return int(payload.get("frontier_count") or 0) > 0
        except (TypeError, ValueError):
            return False

    def _publish_cleared_exploration_frontier(
        self,
        *,
        geometry_session_mode: str = "",
        reason: str = "",
    ) -> None:
        """发布空前沿帧，顶掉 StreamBus / Debug 面板中滞留的探索前沿缓存。"""

        if self._stream_bus is None:
            return
        cleared_payload = {
            "latest_frontiers": [],
            "current_frontier": None,
            "frontier_count": 0,
            "geometry_session_mode": geometry_session_mode or "idle",
            "status_text": None,
            "exploration_terminal": True,
            "terminal_status": "cleared",
            "planned_path": [],
            "current_goal": None,
            "external_frontier_debug_observer": None,
            "frontier_cleared": True,
            "clear_reason": str(reason or "").strip() or None,
        }
        self._stream_bus.publish(
            "exploration.frontier",
            source="navigation_service",
            payload=cleared_payload,
            status="available",
            metadata={
                "exploration_status": "idle",
                "exploration_terminal": True,
                "frontier_cleared": True,
                "clear_reason": str(reason or "").strip() or None,
            },
        )

    def _publish_external_exploration_navigation_overlay(self, exploration_context: ExplorationContext) -> None:
        if self._stream_bus is None:
            return
        exploration_state = exploration_context.exploration_state
        metadata = dict(getattr(exploration_state, "metadata", {}) or {})
        if metadata.get("exploration_mode") != "external_geometry_script":
            return
        if exploration_state.status not in {
            ExplorationStatus.ACCEPTED,
            ExplorationStatus.RUNNING,
            ExplorationStatus.SUCCEEDED,
            ExplorationStatus.FAILED,
            ExplorationStatus.CANCELLED,
        }:
            return

        status_text = str(metadata.get("status_text") or "")
        upper = status_text.upper()
        terminal = bool(metadata.get("exploration_terminal"))
        expl_status = exploration_state.status

        if terminal or expl_status in {
            ExplorationStatus.SUCCEEDED,
            ExplorationStatus.FAILED,
            ExplorationStatus.CANCELLED,
        }:
            if expl_status == ExplorationStatus.SUCCEEDED:
                nav_status = NavigationStatus.SUCCEEDED
            elif expl_status == ExplorationStatus.CANCELLED:
                nav_status = NavigationStatus.CANCELLED
            else:
                nav_status = NavigationStatus.FAILED
            message = exploration_state.message or "外部前沿探索已结束。"
        elif any(token in upper for token in ("NAVIGATING", "FOLLOWING")):
            nav_status = NavigationStatus.RUNNING
            message = "外部前沿探索导航中。"
        elif "DISPATCHING" in upper:
            nav_status = NavigationStatus.PLANNING
            message = "外部探索器正在派发前沿目标。"
        elif "WARMING UP" in upper:
            nav_status = NavigationStatus.PLANNING
            message = "探索器预热中，等待派发前沿目标。"
        elif "WAITING" in upper:
            nav_status = NavigationStatus.ACCEPTED
            message = "探索器等待可达前沿点。"
        elif expl_status == ExplorationStatus.RUNNING:
            nav_status = NavigationStatus.RUNNING
            message = exploration_state.message or "外部前沿探索运行中。"
        else:
            return

        # current_goal 已在生产端（external_debug_observer）解析为 NavigationGoal|None，直接取用。
        # goal_id 在此补充为含 request_id 的标识，保持原有语义。
        goal_id = "external_frontier_%s" % (exploration_state.current_request_id or "active")
        _raw_goal = metadata.get("current_goal")
        current_goal: Optional[NavigationGoal] = None
        if isinstance(_raw_goal, NavigationGoal):
            # 用 model_copy 覆盖 goal_id，保持其余字段不变。
            current_goal = _raw_goal.model_copy(update={"goal_id": goal_id})

        navigation_state = NavigationState(
            current_goal_id=goal_id if current_goal is not None else None,
            status=nav_status,
            message=message,
            goal_reached=nav_status == NavigationStatus.SUCCEEDED,
            metadata={
                "synthetic": True,
                "source": "external_frontier_explorer",
                "explorer_status_text": status_text or None,
                "exploration_request_id": exploration_state.current_request_id,
                "exploration_terminal": terminal,
            },
        )
        navigation_context = NavigationContext(
            current_goal=current_goal,
            navigation_state=navigation_state,
            backend_name=exploration_context.backend_name,
            goal_reached=navigation_state.goal_reached,
            metadata={"synthetic_external_exploration_navigation": True},
        )
        navigation_status = str(getattr(navigation_state.status, "value", navigation_state.status))
        self._stream_bus.publish(
            "navigation.state",
            source="navigation_service",
            payload=navigation_context,
            status="available",
            metadata={
                "backend_name": exploration_context.backend_name,
                "navigation_status": navigation_status,
                "synthetic_external_exploration_navigation": True,
            },
        )

    def get_latest_navigation_context(self) -> Optional[NavigationContext]:
        """返回最新导航上下文。"""

        return self._latest_navigation_context

    def get_latest_exploration_context(self) -> Optional[ExplorationContext]:
        """返回最新探索上下文。"""

        return self._latest_exploration_context

    def clear_latest_exploration_context(self, *, reason: str = "") -> None:
        """清掉已结束探索任务缓存，避免地图激活响应继续携带旧探索状态。"""

        with self._lock:
            self._latest_exploration_context = None
            self._current_explore_request = None
        if self._frontier_stream_has_active_geometry():
            self._publish_cleared_exploration_frontier(
                reason=reason or "clear_latest_exploration_context",
            )

    def list_navigation_history(self, *, limit: Optional[int] = None) -> Tuple[NavigationContext, ...]:
        """返回导航历史。"""

        items = list(self._navigation_history)
        if limit is not None:
            items = items[-max(0, limit) :]
        return tuple(items)

    def list_exploration_history(self, *, limit: Optional[int] = None) -> Tuple[ExplorationContext, ...]:
        """返回探索历史。"""

        items = list(self._exploration_history)
        if limit is not None:
            items = items[-max(0, limit) :]
        return tuple(items)

    def _should_refresh_localization(self) -> bool:
        """定位会话刷新节流。

        高频轮询（navigate/explore_until_complete）每轮都会触发 _build_map_runtime_metadata，
        进而触发 LocalizationService 完整刷新副作用链（pose 捕获 + 诊断 + 三条流发布 + StateStore 写）。
        此处按最小刷新间隔节流：未达间隔时返回 False，复用 get_active_session() 缓存。

        调用方位于 _ingest_navigation_state/_ingest_exploration_state 的 self._lock 临界区内，
        故节流时间戳读写受锁保护。
        """

        now = time.monotonic()
        last = self._last_localization_refresh_monotonic
        if last is not None and (now - last) < self._localization_refresh_min_interval_sec:
            return False
        self._last_localization_refresh_monotonic = now
        return True

    def _build_map_runtime_metadata(
        self,
        *,
        map_name: Optional[str],
        memory_library_name: Optional[str] = None,
        existing_waiting_for: Optional[object] = None,
        include_waiting_for: bool = True,
        refresh_localization: bool = True,
    ) -> dict:
        map_snapshot = self._mapping_service.get_latest_snapshot()
        localization_session = self._localization_service.get_active_session()
        if refresh_localization and localization_session is not None and self._should_refresh_localization():
            try:
                refreshed_session = self._localization_service.refresh_current_session(
                    metadata={"source": "navigation_runtime_metadata"}
                )
                if refreshed_session is not None:
                    localization_session = refreshed_session
            except GatewayError:
                pass
        localization_ready = bool(
            localization_session is not None
            and localization_session.status == LocalizationSessionStatus.READY
            and (map_name is None or localization_session.map_name == map_name)
        )
        requested_map_name = str(map_name or "").strip() or None
        active_map_name = (
            str(localization_session.map_name or "").strip() or None
            if localization_session is not None
            else None
        )
        expected_map_name = requested_map_name or active_map_name
        has_map = map_snapshot is not None
        if (
            map_snapshot is not None
            and expected_map_name is not None
            and map_snapshot.map_name != expected_map_name
        ):
            raise GatewayError(
                "运行态地图快照与导航请求的 map_name 不一致。",
                error_code="map_snapshot_name_mismatch",
            )
        if has_map and localization_ready:
            map_status = MapAssetStatus.READY.value
        elif has_map:
            map_status = MapAssetStatus.LOCALIZING.value
        elif map_name:
            map_status = MapAssetStatus.MAPPING.value
        else:
            map_status = MapAssetStatus.CREATED.value
        waiting_for = str(existing_waiting_for or "").strip() or None
        if include_waiting_for and waiting_for is None:
            if not has_map:
                waiting_for = "mapping_ready"
            elif not localization_ready:
                waiting_for = "localization_ready"
        elif not include_waiting_for:
            waiting_for = None
        return {
            "map_name": (
                requested_map_name
                or (map_snapshot.map_name if map_snapshot is not None else None)
                or active_map_name
            ),
            "map_status": map_status,
            "map_snapshot_identity": (
                {
                    "schema_version": map_snapshot.schema_version,
                    "map_name": map_snapshot.map_name,
                    "source_name": map_snapshot.source_name,
                    "source_epoch": map_snapshot.source_epoch,
                    "frame_id": map_snapshot.frame_id,
                    "update_sequence": str(map_snapshot.update_sequence),
                }
                if map_snapshot is not None
                else None
            ),
            "memory_library_name": memory_library_name or map_name,
            "localization_ready": localization_ready,
            "localization_session_status": (
                localization_session.status.value
                if localization_session is not None and (map_name is None or localization_session.map_name == map_name)
                else None
            ),
            "localization_session_id": (
                localization_session.session_id
                if localization_session is not None and (map_name is None or localization_session.map_name == map_name)
                else None
            ),
            "asset_identity": (
                localization_session.asset_identity.model_dump()
                if localization_session is not None and localization_session.asset_identity is not None
                and (map_name is None or localization_session.map_name == map_name)
                else None
            ),
            "waiting_for": waiting_for,
        }

    def _publish_navigation_event(self, event_type: str, message: str, **payload) -> None:
        if self._event_bus is None:
            return
        self._event_bus.publish(
            self._event_bus.build_event(
                event_type,
                category=RuntimeEventCategory.TASK,
                source="navigation_service",
                message=message,
                payload=payload,
            )
        )

    def _get_navigation_provider(
        self,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> NavigationProvider:
        _exploration_deadline_remaining(
            deadline_monotonic,
            "NavigationProviderResolution",
        )
        provider = self._find_navigation_provider()
        if provider is None:
            raise GatewayError("当前平台几何运行时未提供导航后端。")
        if deadline_monotonic is not None:
            return provider
        if not provider.is_available():
            raise GatewayError(self._navigation_provider_error_message(provider, default="当前导航后端暂不可用。"))
        availability_checker = getattr(provider, "is_navigation_available", None)
        if callable(availability_checker) and not availability_checker():
            raise GatewayError(self._navigation_provider_error_message(provider, default="当前导航后端暂不可用。"))
        return provider

    def _find_navigation_provider(self) -> Optional[NavigationProvider]:
        providers = getattr(self._provider_owner, "providers", None)
        if providers is None or not isinstance(providers, NavigationProvider):
            return None
        return providers

    def _navigation_provider_error_message(self, provider: NavigationProvider, *, default: str) -> str:
        try:
            state = provider.get_navigation_state()
        except Exception as exc:
            LOGGER.exception("读取导航后端故障状态失败。")
            return f"{default}（读取后端状态失败：{exc}）"
        message = str(getattr(state, "message", "") or "").strip()
        metadata = dict(getattr(state, "metadata", {}) or {})
        readiness = metadata.get("readiness") or metadata.get("navigation_readiness")
        reasons = []
        if isinstance(readiness, dict):
            errors = readiness.get("errors")
            if isinstance(errors, dict):
                reasons.extend(str(value) for value in errors.values() if value)
            waiting_for = str(readiness.get("waiting_for") or "").strip()
            if waiting_for:
                reasons.append("waiting_for=%s" % waiting_for)
        if reasons:
            suffix = "；".join(dict.fromkeys(reasons))
            return "%s（%s）" % (message or default, suffix)
        return message or default

    def _subscribe_provider_navigation_updates(self) -> None:
        provider = self._find_navigation_provider()
        if provider is None:
            return
        subscriber = getattr(provider, "subscribe_navigation_updates", None)
        if not callable(subscriber):
            return
        subscription_id = subscriber(self._handle_provider_navigation_update)
        if subscription_id is not None:
            try:
                self._navigation_update_subscription_id = int(subscription_id)
            except (TypeError, ValueError):
                self._navigation_update_subscription_id = None

    def _subscribe_provider_exploration_updates(self) -> None:
        provider = self._find_exploration_provider()
        if provider is None:
            return
        subscriber = getattr(provider, "subscribe_exploration_updates", None)
        if not callable(subscriber):
            return
        subscription_id = subscriber(self._handle_provider_exploration_update)
        if subscription_id is not None:
            try:
                self._exploration_update_subscription_id = int(subscription_id)
            except (TypeError, ValueError):
                self._exploration_update_subscription_id = None

    def _get_exploration_provider(
        self,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> ExplorationProvider:
        # v1.1 改造：GatewayError 追加 details={"capability", ...} hint，
        # 让前端 / debug_panel / start_gateway.sh log scrubber 能直接拿到
        # runtime_status（或 provider_available / exploration_available）而无需
        # 再翻 runtime context；上游错误体字段兼容（details dict 可增 key）。
        _exploration_deadline_remaining(
            deadline_monotonic,
            "ExplorationProviderResolution",
        )
        provider = self._find_exploration_provider()
        if deadline_monotonic is not None:
            if provider is None:
                raise GatewayError(
                    "当前平台几何运行时未提供探索后端。",
                    details={"capability": "exploration"},
                )
            return provider

        runtime_status: Dict[str, object] = {}
        owner = self._provider_owner
        if owner is not None:
            getter = getattr(owner, "get_runtime_status", None)
            if callable(getter):
                try:
                    runtime_status = dict(getter() or {})
                except Exception:
                    # runtime_status 取不到不影响主错误抛出；降级为空 dict 即可
                    runtime_status = {}
        if provider is None:
            raise GatewayError(
                "当前平台几何运行时未提供探索后端（runtime phase=%s, message=%s）。"
                % (
                    runtime_status.get("phase") or "unknown",
                    runtime_status.get("message") or "<none>",
                ),
                details={"capability": "exploration", "runtime_status": runtime_status},
            )
        if not provider.is_available():
            raise GatewayError(
                "当前探索后端暂不可用。",
                details={"capability": "exploration", "provider_available": False},
            )
        availability_checker = getattr(provider, "is_exploration_available", None)
        if callable(availability_checker) and not availability_checker():
            raise GatewayError(
                "当前探索后端暂不可用。",
                details={"capability": "exploration", "exploration_available": False},
            )
        return provider

    def _find_exploration_provider(self) -> Optional[ExplorationProvider]:
        providers = getattr(self._provider_owner, "providers", None)
        if providers is None or not isinstance(providers, ExplorationProvider):
            return None
        return providers

    def _build_navigation_diagnostics(
        self,
        provider: NavigationProvider,
        navigation_state,
    ) -> NavigationDiagnostics:
        """构建平台导航诊断信息。"""

        state_summary = self._state_machine.get_state_summary()
        recovery_attempts = int(state_summary.get("recovery_attempts") or 0)
        return NavigationDiagnostics(
            backend_name=provider.provider_name,
            backend_status=navigation_state.status.value,
            platform_status=self._state_machine.current_status,
            goal_id=self._state_machine.current_goal_id,
            remaining_distance_m=navigation_state.remaining_distance_m,
            remaining_yaw_rad=navigation_state.remaining_yaw_rad,
            recovery_in_progress=recovery_attempts > 0,
            last_error=state_summary.get("last_error"),
            metadata={
                "is_running": self._state_machine.is_running,
                "is_terminal": self._state_machine.is_terminal,
                "elapsed_time_sec": self._state_machine.elapsed_time_sec,
                "recovery_attempts": recovery_attempts,
            },
        )

    def _normalize_name(self, value: str) -> str:
        normalized = []
        for char in value.lower():
            if char.isalnum():
                normalized.append(char)
            else:
                normalized.append("_")
        return "".join(normalized).strip("_") or "location"
