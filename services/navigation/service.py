from __future__ import annotations

from collections import deque
import logging
import threading
import time

from contracts.events import RuntimeEventCategory
from contracts.map_workspace import MapAssetStatus
from contracts.runtime_views import LocalizationSessionStatus
from contracts.navigation import ExplorationStatus, ExploreAreaRequest, NavigationGoal, NavigationStatus
from contracts.runtime_views import ExplorationContext, NavigationContext
from core import EventBus, StateNamespace, StateStore, StreamBus
from gateways.errors import GatewayError
from providers import ExplorationProvider, NavigationProvider
from services.localization import LocalizationService
from services.mapping.service import MappingService
from typing import Deque, Dict, Optional, Tuple

from .state_machine import (
    NavigationStateMachine,
    NavigationDiagnostics,
    create_navigation_state_machine,
    map_nav2_status_to_platform,
)

LOGGER = logging.getLogger("nuwax_robot_bridge.navigation.service")


class NavigationService:
    """统一导航与探索服务。

    平台导航运行时核心：
    - 维护平台导航状态机，管理 NavigationGoal -> 当前导航后端 -> NavigationContext 的完整链路
    - Nav2 action 是唯一正式导航执行器，目标生成、状态推进、失败语义、取消语义都沉淀在平台
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
    ) -> None:
        self._provider_owner = provider_owner
        self._localization_service = localization_service
        self._mapping_service = mapping_service
        self._state_store = state_store
        self._event_bus = event_bus
        self._stream_bus = stream_bus
        self._navigation_history: Deque[NavigationContext] = deque(maxlen=max(1, history_limit))
        self._exploration_history: Deque[ExplorationContext] = deque(maxlen=max(1, history_limit))
        self._latest_navigation_context: Optional[NavigationContext] = None
        self._latest_exploration_context: Optional[ExplorationContext] = None
        self._current_goal: Optional[NavigationGoal] = None
        self._current_explore_request: Optional[ExploreAreaRequest] = None
        # 平台导航状态机
        self._state_machine: NavigationStateMachine = navigation_state_machine or create_navigation_state_machine()
        self._last_navigation_diagnostics: Optional[NavigationDiagnostics] = None
        self._lock = threading.RLock()
        self._navigation_update_subscription_id: Optional[int] = None
        self._exploration_update_subscription_id: Optional[int] = None
        self._subscribe_provider_navigation_updates()
        self._subscribe_provider_exploration_updates()

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

    def refresh_navigation(self) -> NavigationContext:
        """刷新导航上下文，使用平台导航状态机管理状态转移。

        主链：NavigationGoal -> 平台当前导航后端 -> 平台 NavigationContext
        Nav2 action 只负责执行反馈，不负责平台任务定义。
        """

        provider = self._find_navigation_provider()
        if provider is None:
            raise GatewayError("当前平台几何运行时未提供导航后端。")
        navigation_state = provider.get_navigation_state()
        return self._ingest_navigation_state(provider, navigation_state)

    def _handle_provider_navigation_update(self, navigation_state) -> None:
        provider = self._find_navigation_provider()
        if provider is None:
            return
        try:
            self._ingest_navigation_state(provider, navigation_state)
        except Exception:
            return

    def _ingest_navigation_state(self, provider: NavigationProvider, navigation_state) -> NavigationContext:
        with self._lock:
            previous_context = self._latest_navigation_context
            current_goal = self._current_goal
            reuse_terminal_state = (
                previous_context is not None
                and previous_context.navigation_state.status
                in {NavigationStatus.SUCCEEDED, NavigationStatus.FAILED, NavigationStatus.CANCELLED}
                and navigation_state.status == NavigationStatus.IDLE
                and previous_context.navigation_state.current_goal_id is not None
            )

            diagnostics = self._build_navigation_diagnostics(provider, navigation_state)
            self._last_navigation_diagnostics = diagnostics

            goal_reached = bool(provider.is_goal_reached() or navigation_state.goal_reached)
            if reuse_terminal_state:
                navigation_state = previous_context.navigation_state.model_copy(deep=True)
                current_goal = current_goal or previous_context.current_goal
                goal_reached = previous_context.goal_reached
            else:
                self._state_machine.update_from_backend_state(
                    backend_name=provider.provider_name,
                    backend_status=navigation_state.status.value,
                    message=navigation_state.message,
                    goal_reached=goal_reached,
                )
                current_machine_state = self._state_machine.current_status
                if current_machine_state != navigation_state.status:
                    navigation_state = navigation_state.model_copy(
                        update={"status": current_machine_state, "message": self._state_machine._status_message},
                        deep=True,
                    )
                if current_goal is not None and navigation_state.current_goal_id is None:
                    navigation_state = navigation_state.model_copy(
                        update={"current_goal_id": current_goal.goal_id},
                        deep=True,
                    )
                if goal_reached and current_machine_state not in {
                    NavigationStatus.SUCCEEDED,
                    NavigationStatus.FAILED,
                    NavigationStatus.CANCELLED,
                }:
                    navigation_state = navigation_state.model_copy(
                        update={
                            "status": NavigationStatus.SUCCEEDED,
                            "goal_reached": True,
                            "message": navigation_state.message or "目标已到达。",
                        },
                        deep=True,
                    )

            runtime_metadata = self._build_map_runtime_metadata(
                map_name=current_goal.map_name if current_goal is not None else None,
                existing_waiting_for=navigation_state.metadata.get("waiting_for"),
            )
            runtime_metadata["navigation_state_machine"] = self._state_machine.get_state_summary()

            navigation_state = navigation_state.model_copy(
                update={
                    "metadata": {
                        **dict(navigation_state.metadata),
                        **runtime_metadata,
                    }
                },
                deep=True,
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
            if context.navigation_state.status in {
                NavigationStatus.SUCCEEDED,
                NavigationStatus.FAILED,
                NavigationStatus.CANCELLED,
            }:
                self._current_goal = None
            return context

    def set_goal(self, goal: NavigationGoal) -> NavigationContext:
        """提交导航目标，使用平台状态机管理目标接受流程。

        主链：NavigationGoal -> 平台状态机 ACCEPTED -> 当前导航后端执行 -> 平台 NavigationContext
        """

        provider = self._get_navigation_provider()
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
        with self._lock:
            self._current_goal = goal
            self._state_machine.set_goal(goal.goal_id, message="平台导航目标已接受。")
        LOGGER.info("导航服务转发目标 goal_id=%s provider=%s", goal.goal_id, provider.provider_name)
        accepted = provider.set_goal(goal)
        if not accepted:
            message = self._navigation_provider_error_message(provider, default="导航后端拒绝目标 %s。" % goal.goal_id)
            with self._lock:
                self._current_goal = None
                self._state_machine.fail(message)
            raise GatewayError(message)
        LOGGER.info("导航服务目标已被后端接收 goal_id=%s provider=%s", goal.goal_id, provider.provider_name)
        context = self.refresh_navigation()
        if context.navigation_state.status == NavigationStatus.IDLE:
            context = context.model_copy(
                update={
                    "navigation_state": context.navigation_state.model_copy(
                        update={
                            "current_goal_id": goal.goal_id,
                            "status": NavigationStatus.ACCEPTED,
                            "message": "导航目标已接受。",
                        },
                        deep=True,
                    )
                },
                deep=True,
            )
            self._latest_navigation_context = context
        self._publish_navigation_event("navigation.goal_set", f"导航目标 {goal.goal_id} 已提交。", goal_id=goal.goal_id)
        return context

    def cancel_goal(self) -> NavigationContext:
        """取消当前导航目标，使用平台状态机管理取消流程。"""

        provider = self._get_navigation_provider()
        with self._lock:
            self._state_machine.cancel("平台导航目标已取消。")
        provider.cancel_goal()
        context = self.refresh_navigation()
        if context.navigation_state.status == NavigationStatus.IDLE and self._current_goal is not None:
            context = context.model_copy(
                update={
                    "navigation_state": context.navigation_state.model_copy(
                        update={
                            "current_goal_id": self._current_goal.goal_id,
                            "status": NavigationStatus.CANCELLED,
                            "message": "导航目标已取消。",
                        },
                        deep=True,
                    )
                },
                deep=True,
            )
            self._latest_navigation_context = context
        self._publish_navigation_event("navigation.goal_cancelled", "导航目标已取消。")
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
        poll_interval_sec: float = 0.05,
        on_progress=None,
    ) -> NavigationContext:
        """同步等待导航目标达到终态。"""

        self.set_goal(goal)
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
            return NavigationGoal(
                goal_id=f"nav_named_{self._normalize_name(target_name)}",
                target_pose=region.centroid,
                target_name=target_name,
                metadata={"resolved_region_id": region.region_id, "map_version_id": map_snapshot.version_id},
            )
        raise GatewayError(f"未在语义地图中找到命名位置：{target_name}")

    def refresh_exploration(self) -> ExplorationContext:
        """刷新探索上下文。"""

        provider = self._get_exploration_provider()
        exploration_state = provider.get_exploration_state()
        return self._ingest_exploration_state(provider, exploration_state)

    def _handle_provider_exploration_update(self, exploration_state) -> None:
        provider = self._find_exploration_provider()
        if provider is None:
            return
        try:
            self._ingest_exploration_state(provider, exploration_state)
        except Exception:
            return

    def _ingest_exploration_state(self, provider: ExplorationProvider, exploration_state) -> ExplorationContext:
        with self._lock:
            previous_context = self._latest_exploration_context
            current_request = self._current_explore_request
            if (
                previous_context is not None
                and previous_context.exploration_state.status
                in {ExplorationStatus.SUCCEEDED, ExplorationStatus.FAILED, ExplorationStatus.CANCELLED}
                and exploration_state.status == ExplorationStatus.IDLE
                and previous_context.exploration_state.current_request_id is not None
            ):
                exploration_state = previous_context.exploration_state.model_copy(deep=True)
                current_request = current_request or previous_context.current_request
            runtime_metadata = self._build_map_runtime_metadata(
                map_name=current_request.map_name if current_request is not None else None,
                memory_library_name=current_request.map_name if current_request is not None else None,
                existing_waiting_for=exploration_state.metadata.get("waiting_for"),
            )
            exploration_state = exploration_state.model_copy(
                update={
                    "metadata": {
                        **dict(exploration_state.metadata),
                        **runtime_metadata,
                    }
                },
                deep=True,
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
                frontiers = dict(getattr(context.exploration_state, "metadata", {}) or {}).get("latest_frontiers")
                current_frontier = dict(getattr(context.exploration_state, "metadata", {}) or {}).get("current_frontier")
                if frontiers is not None or current_frontier is not None:
                    self._stream_bus.publish(
                        "exploration.frontier",
                        source="navigation_service",
                        payload={"latest_frontiers": frontiers, "current_frontier": current_frontier},
                        status="available",
                        metadata={"backend_name": provider.provider_name},
                    )
            if context.exploration_state.status in {
                ExplorationStatus.SUCCEEDED,
                ExplorationStatus.FAILED,
                ExplorationStatus.CANCELLED,
            }:
                self._current_explore_request = None
            return context

    def start_exploration(self, request: ExploreAreaRequest) -> ExplorationContext:
        """启动探索任务。"""

        provider = self._get_exploration_provider()
        with self._lock:
            self._current_explore_request = request
        accepted = provider.start_exploration(request)
        if not accepted:
            message = f"探索后端拒绝请求 {request.request_id}。"
            try:
                exploration_state = provider.get_exploration_state()
            except Exception:
                exploration_state = None
            if exploration_state is not None and exploration_state.message:
                message = exploration_state.message
            with self._lock:
                self._current_explore_request = None
            raise GatewayError(message)
        context = self.refresh_exploration()
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
                        deep=True,
                    )
                },
                deep=True,
            )
            self._latest_exploration_context = context
        return context

    def stop_exploration(self) -> ExplorationContext:
        """停止当前探索任务。"""

        provider = self._get_exploration_provider()
        previous_request = self._current_explore_request
        if previous_request is None and self._latest_exploration_context is not None:
            previous_request = self._latest_exploration_context.current_request
        provider.stop_exploration()
        context = self.refresh_exploration()

        terminal_statuses = {
            ExplorationStatus.SUCCEEDED,
            ExplorationStatus.FAILED,
            ExplorationStatus.CANCELLED,
        }
        if context.exploration_state.status in terminal_statuses:
            if previous_request is not None and context.current_request is None:
                state_updates = {}
                if context.exploration_state.current_request_id is None:
                    state_updates["current_request_id"] = previous_request.request_id
                if context.exploration_state.strategy is None:
                    state_updates["strategy"] = previous_request.strategy
                exploration_state = context.exploration_state
                if state_updates:
                    exploration_state = exploration_state.model_copy(update=state_updates, deep=True)
                context = context.model_copy(
                    update={
                        "current_request": previous_request,
                        "exploration_state": exploration_state,
                    },
                    deep=True,
                )
                self._latest_exploration_context = context
            self._current_explore_request = None
            return context

        if previous_request is not None:
            context = context.model_copy(
                update={
                    "current_request": previous_request,
                    "exploration_state": context.exploration_state.model_copy(
                        update={
                            "current_request_id": previous_request.request_id,
                            "status": ExplorationStatus.CANCELLED,
                            "message": "探索任务已取消。",
                        },
                        deep=True,
                    ),
                },
                deep=True,
            )
            self._latest_exploration_context = context
            self._current_explore_request = None
        return context

    def explore_until_complete(
        self,
        request: ExploreAreaRequest,
        *,
        poll_interval_sec: float = 0.05,
        on_progress=None,
    ) -> ExplorationContext:
        """同步等待探索任务达到终态。"""

        self.start_exploration(request)
        while True:
            context = self.refresh_exploration()
            if callable(on_progress):
                on_progress(context)
            state = context.exploration_state.status
            if state == ExplorationStatus.SUCCEEDED:
                return context
            if state == ExplorationStatus.FAILED:
                raise GatewayError(context.exploration_state.message or "探索执行失败。")
            if state == ExplorationStatus.CANCELLED:
                raise GatewayError(context.exploration_state.message or "探索已取消。")
            time.sleep(max(0.01, poll_interval_sec))

    def get_latest_navigation_context(self) -> Optional[NavigationContext]:
        """返回最新导航上下文。"""

        return self._latest_navigation_context

    def get_latest_exploration_context(self) -> Optional[ExplorationContext]:
        """返回最新探索上下文。"""

        return self._latest_exploration_context

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

    def _build_map_runtime_metadata(
        self,
        *,
        map_name: Optional[str],
        memory_library_name: Optional[str] = None,
        existing_waiting_for: Optional[object] = None,
    ) -> dict:
        map_snapshot = self._mapping_service.get_latest_snapshot()
        localization_session = self._localization_service.get_active_session()
        localization_ready = bool(
            localization_session is not None
            and localization_session.status == LocalizationSessionStatus.READY
            and (map_name is None or localization_session.map_name == map_name)
        )
        has_map = map_snapshot is not None
        if has_map and localization_ready:
            map_status = MapAssetStatus.READY.value
        elif has_map:
            map_status = MapAssetStatus.LOCALIZING.value
        elif map_name:
            map_status = MapAssetStatus.MAPPING.value
        else:
            map_status = MapAssetStatus.CREATED.value
        waiting_for = str(existing_waiting_for or "").strip() or None
        if waiting_for is None:
            if not has_map:
                waiting_for = "mapping_ready"
            elif not localization_ready:
                waiting_for = "localization_ready"
        return {
            "map_name": map_name,
            "map_status": map_status,
            "map_version_id": map_snapshot.version_id if map_snapshot is not None else None,
            "latest_map_version_id": map_snapshot.version_id if map_snapshot is not None else None,
            "memory_library_name": memory_library_name or map_name,
            "localization_ready": localization_ready,
            "localization_session_status": (
                localization_session.status.value
                if localization_session is not None and (map_name is None or localization_session.map_name == map_name)
                else None
            ),
            "localization_session_map_version_id": (
                localization_session.map_version_id
                if localization_session is not None and (map_name is None or localization_session.map_name == map_name)
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

    def _get_navigation_provider(self) -> NavigationProvider:
        provider = self._find_navigation_provider()
        if provider is None:
            raise GatewayError("当前平台几何运行时未提供导航后端。")
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
        except Exception:
            return default
        message = str(getattr(state, "message", "") or "").strip()
        metadata = dict(getattr(state, "metadata", {}) or {})
        readiness = metadata.get("readiness") or metadata.get("nav2_readiness")
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
        try:
            subscription_id = subscriber(self._handle_provider_navigation_update)
        except Exception:
            return
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
        try:
            subscription_id = subscriber(self._handle_provider_exploration_update)
        except Exception:
            return
        if subscription_id is not None:
            try:
                self._exploration_update_subscription_id = int(subscription_id)
            except (TypeError, ValueError):
                self._exploration_update_subscription_id = None

    def _get_exploration_provider(self) -> ExplorationProvider:
        provider = self._find_exploration_provider()
        if provider is None:
            raise GatewayError("当前平台几何运行时未提供探索后端。")
        if not provider.is_available():
            raise GatewayError("当前探索后端暂不可用。")
        availability_checker = getattr(provider, "is_exploration_available", None)
        if callable(availability_checker) and not availability_checker():
            raise GatewayError("当前探索后端暂不可用。")
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

        return NavigationDiagnostics(
            backend_name=provider.provider_name,
            backend_status=navigation_state.status.value,
            platform_status=self._state_machine.current_status,
            goal_id=self._state_machine.current_goal_id,
            remaining_distance_m=navigation_state.remaining_distance_m,
            remaining_yaw_rad=navigation_state.remaining_yaw_rad,
            recovery_in_progress=self._state_machine._recovery_attempts > 0,
            last_error=self._state_machine._last_error,
            metadata={
                "is_running": self._state_machine.is_running,
                "is_terminal": self._state_machine.is_terminal,
                "elapsed_time_sec": self._state_machine.elapsed_time_sec,
                "recovery_attempts": self._state_machine._recovery_attempts,
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
