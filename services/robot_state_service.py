from __future__ import annotations

from collections import deque
from dataclasses import asdict, is_dataclass

from contracts.runtime_views import DiagnosticItem, DiagnosticLevel, DiagnosticSnapshot, RobotStatusSnapshot
from contracts.events import RuntimeEventCategory
from core import EventBus, StateNamespace, StateStore
from drivers.common.assembly import RobotAssemblyBase
from gateways.errors import GatewayError
from providers import SafetyProvider, StateProvider
from typing import Deque, List, Optional, Tuple


class RobotStateService:
    """机器人状态服务。"""

    def __init__(
        self,
        robot: RobotAssemblyBase,
        *,
        state_store: StateStore,
        event_bus: Optional[EventBus] = None,
        history_limit: int = 200,
        diagnostic_history_limit: int = 200,
    ) -> None:
        self._robot = robot
        self._state_store = state_store
        self._event_bus = event_bus
        self._history: Deque[RobotStatusSnapshot] = deque(maxlen=max(1, history_limit))
        self._diagnostic_history: Deque[DiagnosticSnapshot] = deque(maxlen=max(1, diagnostic_history_limit))
        self._latest_snapshot: Optional[RobotStatusSnapshot] = None

    def refresh(self) -> RobotStatusSnapshot:
        """采样并写入最新机器人状态。"""

        state_provider = self._get_state_provider()
        robot_state = state_provider.get_robot_state()
        safety_state = self._get_safety_state(robot_state)
        diagnostics = self._build_diagnostics(robot_state, safety_state)
        snapshot = RobotStatusSnapshot(
            robot_name=self._robot.manifest.robot_name,
            robot_model=self._robot.manifest.robot_model,
            assembly_status=self._serialize(self._robot.get_status()),
            robot_state=robot_state,
            safety_state=safety_state,
            diagnostics=diagnostics,
            metadata={
                "provider_name": state_provider.provider_name,
                "adapter_count": len(self._robot.get_adapter_health_statuses()),
            },
        )
        diagnostic_snapshot = DiagnosticSnapshot(
            robot_name=self._robot.manifest.robot_name,
            items=diagnostics,
            metadata={"robot_model": self._robot.manifest.robot_model},
        )

        self._latest_snapshot = snapshot
        self._history.append(snapshot)
        self._diagnostic_history.append(diagnostic_snapshot)

        self._state_store.write_from_provider(
            state_provider.provider_name,
            StateNamespace.ROBOT_STATE,
            self._robot.manifest.robot_name,
            robot_state,
            metadata={"kind": "robot_state"},
        )
        self._state_store.write(
            StateNamespace.SAFETY,
            self._robot.manifest.robot_name,
            safety_state,
            source=state_provider.provider_name,
        )
        self._state_store.write(
            StateNamespace.SYSTEM,
            f"{self._robot.manifest.robot_name}.status_snapshot",
            snapshot,
            source="robot_state_service",
        )
        self._state_store.write(
            StateNamespace.SYSTEM,
            f"{self._robot.manifest.robot_name}.diagnostics",
            diagnostic_snapshot,
            source="robot_state_service",
        )

        if self._event_bus is not None:
            self._event_bus.publish(
                self._event_bus.build_event(
                    "robot.state_refreshed",
                    category=RuntimeEventCategory.ROBOT,
                    source="robot_state_service",
                    subject_id=self._robot.manifest.robot_name,
                    message="机器人状态已刷新。",
                    payload={
                        "mode": robot_state.mode.value,
                        "diagnostic_count": len(diagnostics),
                    },
                )
            )
        return snapshot

    def get_latest_snapshot(self) -> Optional[RobotStatusSnapshot]:
        """返回最近一次状态快照。"""

        return self._latest_snapshot

    def list_history(self, *, limit: Optional[int] = None) -> Tuple[RobotStatusSnapshot, ...]:
        """返回状态历史。"""

        items = list(self._history)
        if limit is not None:
            items = items[-max(0, limit) :]
        return tuple(items)

    def list_diagnostics(self, *, limit: Optional[int] = None) -> Tuple[DiagnosticSnapshot, ...]:
        """返回诊断历史。"""

        items = list(self._diagnostic_history)
        if limit is not None:
            items = items[-max(0, limit) :]
        return tuple(items)

    def _get_state_provider(self) -> StateProvider:
        providers = getattr(self._robot, "providers", None)
        if providers is None or not isinstance(providers, StateProvider):
            raise GatewayError("当前机器人入口未提供状态提供器。")
        if not providers.is_available():
            raise GatewayError("当前状态提供器暂不可用。")
        return providers

    def _get_safety_state(self, robot_state) -> object:
        providers = getattr(self._robot, "providers", None)
        if providers is not None and isinstance(providers, SafetyProvider) and providers.is_available():
            return providers.get_safety_state()
        return robot_state.safety

    def _build_diagnostics(self, robot_state, safety_state) -> List[DiagnosticItem]:
        diagnostics: List[DiagnosticItem] = [
            DiagnosticItem(
                component="control_mode",
                level=DiagnosticLevel.INFO,
                message=f"当前控制模式: {robot_state.mode.value}",
            )
        ]
        adapter_statuses = self._robot.get_adapter_health_statuses()
        if adapter_statuses:
            healthy_count = sum(1 for item in adapter_statuses if item.is_healthy)
            diagnostics.append(
                DiagnosticItem(
                    component="adapter_runtime",
                    level=DiagnosticLevel.INFO if healthy_count == len(adapter_statuses) else DiagnosticLevel.WARNING,
                    message=f"适配器健康数 {healthy_count}/{len(adapter_statuses)}",
                    metadata={"healthy_adapter_count": healthy_count, "adapter_count": len(adapter_statuses)},
                )
            )
            for item in adapter_statuses:
                if item.is_healthy:
                    continue
                diagnostics.append(
                    DiagnosticItem(
                        component=f"adapter.{item.adapter_name}",
                        level=DiagnosticLevel.WARNING,
                        message=item.message or "适配器健康检查失败。",
                    )
                )
        if not safety_state.can_move:
            diagnostics.append(
                DiagnosticItem(
                    component="safety",
                    level=DiagnosticLevel.WARNING,
                    message=safety_state.warning_message or "当前机器人不允许移动。",
                )
            )
        if safety_state.is_estopped:
            diagnostics.append(
                DiagnosticItem(
                    component="safety",
                    level=DiagnosticLevel.ERROR,
                    message="机器人处于急停状态。",
                )
            )
        return diagnostics

    def _serialize(self, value):
        if is_dataclass(value):
            return asdict(value)
        return value
