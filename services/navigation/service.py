from __future__ import annotations

from collections import deque
import time

from contracts.events import RuntimeEventCategory
from contracts.map_workspace import MapAssetStatus
from contracts.runtime_views import LocalizationSessionStatus
from contracts.navigation import ExplorationStatus, ExploreAreaRequest, NavigationGoal, NavigationStatus
from contracts.runtime_views import ExplorationContext, NavigationContext
from core import EventBus, StateNamespace, StateStore
from gateways.errors import GatewayError
from providers import ExplorationProvider, NavigationProvider
from services.localization import LocalizationService
from services.mapping.service import MappingService
from typing import Deque, Optional, Tuple


class NavigationService:
    """统一导航与探索服务。"""

    def __init__(
        self,
        *,
        provider_owner,
        localization_service: LocalizationService,
        mapping_service: MappingService,
        state_store: StateStore,
        event_bus: Optional[EventBus] = None,
        history_limit: int = 100,
    ) -> None:
        self._provider_owner = provider_owner
        self._localization_service = localization_service
        self._mapping_service = mapping_service
        self._state_store = state_store
        self._event_bus = event_bus
        self._navigation_history: Deque[NavigationContext] = deque(maxlen=max(1, history_limit))
        self._exploration_history: Deque[ExplorationContext] = deque(maxlen=max(1, history_limit))
        self._latest_navigation_context: Optional[NavigationContext] = None
        self._latest_exploration_context: Optional[ExplorationContext] = None
        self._current_goal: Optional[NavigationGoal] = None
        self._current_explore_request: Optional[ExploreAreaRequest] = None

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
        """刷新导航上下文。"""

        provider = self._get_navigation_provider()
        navigation_state = provider.get_navigation_state()
        goal_reached = bool(provider.is_goal_reached() or navigation_state.goal_reached)
        if self._current_goal is not None and navigation_state.current_goal_id is None:
            navigation_state = navigation_state.model_copy(
                update={"current_goal_id": self._current_goal.goal_id},
                deep=True,
            )
        if goal_reached and navigation_state.status not in {NavigationStatus.SUCCEEDED, NavigationStatus.FAILED, NavigationStatus.CANCELLED}:
            navigation_state = navigation_state.model_copy(
                update={
                    "status": NavigationStatus.SUCCEEDED,
                    "goal_reached": True,
                    "message": navigation_state.message or "目标已到达。",
                },
                deep=True,
            )
        runtime_metadata = self._build_map_runtime_metadata(
            map_name=self._current_goal.map_name if self._current_goal is not None else None,
            existing_waiting_for=navigation_state.metadata.get("waiting_for"),
        )
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
            current_goal=self._current_goal,
            navigation_state=navigation_state,
            backend_name=provider.provider_name,
            goal_reached=goal_reached,
            metadata={
                "provider_version": provider.provider_version,
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
        return context

    def set_goal(self, goal: NavigationGoal) -> NavigationContext:
        """提交导航目标。"""

        provider = self._get_navigation_provider()
        accepted = provider.set_goal(goal)
        if not accepted:
            raise GatewayError(f"导航后端拒绝目标 {goal.goal_id}。")
        self._current_goal = goal
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
        """取消当前导航目标。"""

        provider = self._get_navigation_provider()
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
            if context.goal_reached or state == NavigationStatus.SUCCEEDED:
                return context
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
        previous_context = self._latest_exploration_context
        if (
            previous_context is not None
            and previous_context.exploration_state.status
            in {ExplorationStatus.SUCCEEDED, ExplorationStatus.FAILED, ExplorationStatus.CANCELLED}
            and exploration_state.status == ExplorationStatus.IDLE
            and previous_context.exploration_state.current_request_id is not None
        ):
            exploration_state = previous_context.exploration_state.model_copy(deep=True)
        runtime_metadata = self._build_map_runtime_metadata(
            map_name=self._current_explore_request.map_name if self._current_explore_request is not None else None,
            memory_library_name=self._current_explore_request.map_name if self._current_explore_request is not None else None,
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
            current_request=self._current_explore_request,
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
        return context

    def start_exploration(self, request: ExploreAreaRequest) -> ExplorationContext:
        """启动探索任务。"""

        provider = self._get_exploration_provider()
        accepted = provider.start_exploration(request)
        if not accepted:
            message = f"探索后端拒绝请求 {request.request_id}。"
            try:
                exploration_state = provider.get_exploration_state()
            except Exception:
                exploration_state = None
            if exploration_state is not None and exploration_state.message:
                message = exploration_state.message
            raise GatewayError(message)
        self._current_explore_request = request
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
        provider.stop_exploration()
        context = self.refresh_exploration()
        if context.exploration_state.status == ExplorationStatus.IDLE and self._current_explore_request is not None:
            context = context.model_copy(
                update={
                    "exploration_state": context.exploration_state.model_copy(
                        update={
                            "current_request_id": self._current_explore_request.request_id,
                            "status": ExplorationStatus.CANCELLED,
                            "message": "探索任务已取消。",
                        },
                        deep=True,
                    )
                },
                deep=True,
            )
            self._latest_exploration_context = context
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
            raise GatewayError("当前导航后端暂不可用。")
        return provider

    def _find_navigation_provider(self) -> Optional[NavigationProvider]:
        providers = getattr(self._provider_owner, "providers", None)
        if providers is None or not isinstance(providers, NavigationProvider):
            return None
        return providers

    def _get_exploration_provider(self) -> ExplorationProvider:
        provider = self._find_exploration_provider()
        if provider is None:
            raise GatewayError("当前平台几何运行时未提供探索后端。")
        if not provider.is_available():
            raise GatewayError("当前探索后端暂不可用。")
        return provider

    def _find_exploration_provider(self) -> Optional[ExplorationProvider]:
        providers = getattr(self._provider_owner, "providers", None)
        if providers is None or not isinstance(providers, ExplorationProvider):
            return None
        return providers

    def _normalize_name(self, value: str) -> str:
        normalized = []
        for char in value.lower():
            if char.isalnum():
                normalized.append(char)
            else:
                normalized.append("_")
        return "".join(normalized).strip("_") or "location"
