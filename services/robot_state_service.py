from __future__ import annotations

from collections import deque
from dataclasses import asdict, is_dataclass
import json
from uuid import uuid4

from contracts.runtime_views import DiagnosticItem, DiagnosticLevel, DiagnosticSnapshot, RobotStatusSnapshot
from contracts.events import RuntimeEventCategory
from core import EventBus, StateNamespace, StateStore, StreamBus
from core.errors import GatewayError
from providers import SafetyProvider, StateProvider
from providers.data_plane import RobotAssemblyProtocol
from typing import Deque, List, Optional, Tuple


class RobotStateService:
    """机器人状态服务。"""

    def __init__(
        self,
        robot: RobotAssemblyProtocol,
        *,
        state_store: StateStore,
        event_bus: Optional[EventBus] = None,
        stream_bus: Optional[StreamBus] = None,
        history_limit: int = 200,
        diagnostic_history_limit: int = 200,
    ) -> None:
        self._robot = robot
        self._state_store = state_store
        self._event_bus = event_bus
        self._stream_bus = stream_bus
        self._history: Deque[RobotStatusSnapshot] = deque(maxlen=max(1, history_limit))
        self._diagnostic_history: Deque[DiagnosticSnapshot] = deque(maxlen=max(1, diagnostic_history_limit))
        self._latest_snapshot: Optional[RobotStatusSnapshot] = None
        self._last_web_state_signature: Optional[str] = None
        self._web_state_source_epoch = uuid4().hex
        self._web_state_revision = 0

    def refresh(self) -> RobotStatusSnapshot:
        """采样并写入最新机器人状态。"""

        state_provider = self._get_state_provider()
        robot_state = state_provider.get_robot_state()
        return self.ingest_robot_state(
            robot_state,
            source_name=state_provider.provider_name,
            metadata={"source_kind": "provider_refresh"},
        )

    def ingest_robot_state(
        self,
        robot_state,
        *,
        source_name: str,
        safety_state=None,
        metadata: Optional[dict] = None,
    ) -> RobotStatusSnapshot:
        """写入源头推送的机器人状态。"""

        safety_state = safety_state or self._get_safety_state(robot_state)
        diagnostics = self._build_diagnostics(robot_state, safety_state)
        snapshot = RobotStatusSnapshot(
            robot_name=self._robot.manifest.robot_name,
            robot_model=self._robot.manifest.robot_model,
            assembly_status=self._serialize(self._robot.get_status()),
            robot_state=robot_state,
            safety_state=safety_state,
            diagnostics=diagnostics,
            metadata={
                "provider_name": source_name,
                "adapter_count": len(self._robot.get_adapter_health_statuses()),
                **dict(metadata or {}),
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
            source_name,
            StateNamespace.ROBOT_STATE,
            self._robot.manifest.robot_name,
            robot_state,
            metadata={"kind": "robot_state"},
        )
        self._state_store.write(
            StateNamespace.SAFETY,
            self._robot.manifest.robot_name,
            safety_state,
            source=source_name,
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

        web_state_signature = self._build_web_state_signature(snapshot)
        if web_state_signature == self._last_web_state_signature:
            return snapshot
        self._last_web_state_signature = web_state_signature
        self._web_state_revision += 1
        web_state_metadata = {
            "robot_name": self._robot.manifest.robot_name,
            "mode": getattr(getattr(robot_state, "mode", None), "value", None),
            "diagnostic_count": len(diagnostics),
            "revision": self._web_state_revision,
            "source_epoch": self._web_state_source_epoch,
            "change_policy": "web_semantic_fields",
        }

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
                        "revision": self._web_state_revision,
                        "source_epoch": self._web_state_source_epoch,
                    },
                )
            )
        if self._stream_bus is not None:
            self._stream_bus.publish(
                "robot.state",
                source="robot_state_service",
                payload=snapshot,
                frame_id=getattr(robot_state, "frame_id", None),
                # 旧控制台只读取信封 revision，也必须识别服务重启后的新生命周期。
                revision=f"robot-state:{self._web_state_source_epoch}:{self._web_state_revision}",
                status="available",
                metadata=web_state_metadata,
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

    def _build_web_state_signature(self, snapshot: RobotStatusSnapshot) -> str:
        payload = {
            "robot_name": snapshot.robot_name,
            "robot_model": snapshot.robot_model,
            "assembly_status": self._stable_value(snapshot.assembly_status),
            "robot_state": self._web_robot_state(snapshot.robot_state),
            "safety_state": self._stable_value(snapshot.safety_state),
            "diagnostics": [self._web_diagnostic(item) for item in snapshot.diagnostics],
            "provider_name": snapshot.metadata.get("provider_name"),
            "adapter_count": snapshot.metadata.get("adapter_count"),
            "source_kind": snapshot.metadata.get("source_kind"),
        }
        return json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))

    def _web_robot_state(self, robot_state) -> dict:
        return {
            "robot_id": getattr(robot_state, "robot_id", None),
            "frame_id": getattr(robot_state, "frame_id", None),
            "mode": getattr(getattr(robot_state, "mode", None), "value", getattr(robot_state, "mode", None)),
            "battery": self._stable_value(getattr(robot_state, "battery", None)),
            "safety": self._stable_value(getattr(robot_state, "safety", None)),
            "metadata": self._stable_value(self._filter_web_metadata(getattr(robot_state, "metadata", {}) or {})),
        }

    def _web_diagnostic(self, item: DiagnosticItem) -> dict:
        return {
            "component": item.component,
            "level": getattr(item.level, "value", item.level),
            "message": item.message,
            "metadata": self._stable_value(item.metadata),
        }

    def _filter_web_metadata(self, metadata: dict) -> dict:
        volatile_keys = {
            "pose",
            "twist",
            "joints",
            "imu",
            # 高频惯性载荷与姿态派生数值只更新完整快照，不驱动网页语义修订。
            "canonical_imu_sample",
            "orientation_yaw_rate_rps",
            "yaw_stabilization_offset_rad",
            "timestamp",
            "updated_at",
            "source_timestamp",
            "sequence",
            "seq",
            "cursor",
            "frame_count",
            "latest_point_cloud_points",
            "dds_point_cloud_reader_drop_count",
            "dds_point_cloud_dispatch_drop_count",
        }
        return self._drop_volatile_keys(dict(metadata), volatile_keys)

    def _drop_volatile_keys(self, value, volatile_keys: set):
        if isinstance(value, dict):
            return {
                key: self._drop_volatile_keys(item, volatile_keys)
                for key, item in value.items()
                if key not in volatile_keys
            }
        if isinstance(value, (list, tuple)):
            return [self._drop_volatile_keys(item, volatile_keys) for item in value]
        return value

    def _stable_value(self, value):
        if is_dataclass(value):
            return self._stable_value(asdict(value))
        model_dump = getattr(value, "model_dump", None)
        if callable(model_dump):
            return self._stable_value(model_dump(mode="json"))
        if isinstance(value, dict):
            return {str(key): self._stable_value(value[key]) for key in sorted(value)}
        if isinstance(value, (list, tuple)):
            return [self._stable_value(item) for item in value]
        return value

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
        if safety_state.can_move is not True:
            diagnostics.append(
                DiagnosticItem(
                    component="safety",
                    level=DiagnosticLevel.WARNING,
                    message=safety_state.warning_message or (
                        "当前机器人运动状态尚未确认。"
                        if safety_state.can_move is None
                        else "当前机器人运动执行链不可用。"
                    ),
                )
            )
        return diagnostics

    def _serialize(self, value):
        if is_dataclass(value):
            return asdict(value)
        return value
