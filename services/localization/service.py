from __future__ import annotations

from collections import deque
import time

from contracts.base import utc_now
from contracts.events import RuntimeEventCategory
from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.runtime_views import LocalizationSession, LocalizationSessionStatus, LocalizationSnapshot
from core import EventBus, StateNamespace, StateStore
from gateways.errors import GatewayError
from providers import LocalizationProvider
from typing import Deque, Dict, Optional, Tuple

from .state_machine import (
    LocalizationStateMachine,
    LocalizationDiagnostics,
    create_localization_state_machine,
)


class LocalizationService:
    """统一定位与 TF 快照服务。

    平台定位运行时核心，管理 LocalizationSession 生命周期：
    - activate_map -> ensure_active_session -> relocalize -> READY -> navigation available
    - 支持 STALE（跟踪丢失可恢复）和 FAILED（定位失败）状态
    - 数据源来自平台几何运行时，不直接依赖 Go2 数据面
    """

    def __init__(
        self,
        *,
        provider_owner,
        state_store: StateStore,
        event_bus: Optional[EventBus] = None,
        history_limit: int = 100,
        state_machine: Optional[LocalizationStateMachine] = None,
    ) -> None:
        self._provider_owner = provider_owner
        self._state_store = state_store
        self._event_bus = event_bus
        self._history: Deque[LocalizationSnapshot] = deque(maxlen=max(1, history_limit))
        self._session_history: Deque[LocalizationSession] = deque(maxlen=max(1, history_limit))
        self._latest_snapshot: Optional[LocalizationSnapshot] = None
        self._active_session: Optional[LocalizationSession] = None
        # 平台定位状态机
        self._state_machine: LocalizationStateMachine = state_machine or create_localization_state_machine()
        self._last_diagnostics: Optional[LocalizationDiagnostics] = None

    def refresh(self) -> LocalizationSnapshot:
        """从本地定位提供器刷新定位快照。"""

        provider = self._get_localization_provider()
        current_pose, frame_tree = self._capture_provider_data(provider)
        return self.ingest_external_snapshot(
            source_name=provider.provider_name,
            current_pose=current_pose,
            frame_tree=frame_tree,
            metadata={"source_kind": "provider", "provider_version": provider.provider_version},
        )

    def ensure_active_session(
        self,
        *,
        map_name: str,
        map_version_id: str,
        frame_id: Optional[str] = None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> LocalizationSession:
        """确保当前存在绑定到指定地图版本的平台定位会话。"""

        existing = self._active_session
        if (
            existing is not None
            and existing.map_name == str(map_name or "").strip()
            and existing.map_version_id == str(map_version_id or "").strip()
            and (
                existing.frame_id == (str(frame_id or "").strip() or None)
                or (
                    existing.frame_id is not None
                    and str(frame_id or "").strip()
                    and frame_ids_semantically_equal(existing.frame_id, str(frame_id or "").strip())
                )
            )
        ):
            return existing
        session = LocalizationSession(
            session_id=self._build_session_id(map_name=map_name, map_version_id=map_version_id),
            map_name=str(map_name or "").strip(),
            map_version_id=str(map_version_id or "").strip(),
            frame_id=str(frame_id or "").strip() or None,
            status=LocalizationSessionStatus.LOCALIZING,
            source_name="platform_localization_runtime",
            pose_available=False,
            metadata=dict(metadata or {}),
        )
        self._latest_snapshot = None
        self._set_active_session(session)
        return session

    def refresh_active_session(
        self,
        *,
        map_name: str,
        map_version_id: str,
        frame_id: Optional[str] = None,
        metadata: Optional[Dict[str, object]] = None,
        force_relocalize: bool = False,
    ) -> LocalizationSession:
        """刷新当前平台定位会话，使用平台状态机管理定位生命周期。

        主链：
        1. 从平台几何运行时获取位姿和诊断数据（不直接依赖 Go2 数据面）
        2. 更新平台定位状态机
        3. 根据状态机状态更新 LocalizationSession
        4. 生成平台级 last_error 语义

        Args:
            map_name: 绑定地图名称
            map_version_id: 绑定地图版本
            frame_id: 目标坐标系
            metadata: 附加元数据
            force_relocalize: 是否强制触发重定位
        """

        session = self.ensure_active_session(
            map_name=map_name,
            map_version_id=map_version_id,
            frame_id=frame_id,
            metadata=metadata,
        )

        provider = self._get_localization_provider()
        provider_name = provider.provider_name

        # 从平台几何运行时获取位姿数据（数据源可能是 RTAB-Map 或 Go2 SDK，但通过平台 provider 层抽象）
        current_pose, frame_tree = self._capture_provider_data(provider)

        # 构建定位诊断信息
        diagnostics = self._build_localization_diagnostics(
            provider=provider,
            current_pose=current_pose,
            session_frame_id=session.frame_id,
        )
        self._last_diagnostics = diagnostics

        # 新会话首次刷新或显式重定位时，都应进入 LOCALIZING 周期。
        if force_relocalize or self._state_machine.current_state == "idle":
            self._state_machine.start()

        # 更新状态机并检查超时
        self._state_machine.check_timeouts()

        if current_pose is not None:
            # 位姿可用，根据跟踪质量更新状态机
            pose_available = True
            tracking_quality = diagnostics.tracking_quality
            backend_state = diagnostics.backend_state

            # 检查坐标系匹配
            if session.frame_id and not frame_ids_semantically_equal(current_pose.frame_id, session.frame_id):
                pose_available = False
                diagnostics.last_error = "坐标系不匹配：%s vs %s" % (current_pose.frame_id, session.frame_id)
                tracking_quality = 0.0

            self._state_machine.pose_received(
                pose_available=pose_available,
                tracking_quality=tracking_quality,
                backend_state=backend_state,
            )
        else:
            # 无位姿
            self._state_machine.pose_received(
                pose_available=False,
                tracking_quality=0.0,
                backend_state=diagnostics.backend_state,
            )

        # 再次检查超时
        self._state_machine.check_timeouts()

        # 获取最终状态机状态
        machine_state = self._state_machine.current_state

        # 映射到 LocalizationSessionStatus
        session_status = self._map_machine_state_to_session_status(machine_state)

        # 构建 last_error
        last_error = self._build_session_last_error(
            diagnostics=diagnostics,
            session_status=session_status,
        )

        # 构建快照
        usable_pose = current_pose if session_status == LocalizationSessionStatus.READY else None
        snapshot = self.ingest_external_snapshot(
            source_name="platform_localization:%s" % session.map_name,
            current_pose=usable_pose,
            frame_tree=frame_tree if usable_pose is not None else None,
            metadata={
                "source_kind": "platform_localization_session",
                "map_name": session.map_name,
                "platform_map_version_id": session.map_version_id,
                "localization_session_id": session.session_id,
                "provider_name": provider_name,
                "machine_state": machine_state,
                "diagnostics": diagnostics.model_dump(mode="json"),
                "force_relocalize": force_relocalize,
                **dict(metadata or {}),
            },
        )

        # 更新会话
        updated = session.model_copy(
            update={
                "status": session_status,
                "source_name": provider_name,
                "pose_available": session_status == LocalizationSessionStatus.READY,
                "last_localized_at": utc_now() if session_status == LocalizationSessionStatus.READY else session.last_localized_at,
                "last_error": last_error,
                "metadata": {
                    **dict(session.metadata),
                    "machine_state": machine_state,
                    **dict(snapshot.metadata),
                },
            },
            deep=True,
        )
        self._set_active_session(updated)
        return updated

    def relocalize_active_session(
        self,
        *,
        map_name: str,
        map_version_id: str,
        frame_id: Optional[str] = None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> LocalizationSession:
        """对当前激活会话执行平台级重定位。

        正式主链：
        1. 重置状态机到 LOCALIZING 状态
        2. 尝试获取新位姿
        3. 等待定位就绪或失败

        Returns:
            更新后的 LocalizationSession
        """

        # 触发状态机重定位
        self._state_machine.relocalize()

        return self.refresh_active_session(
            map_name=map_name,
            map_version_id=map_version_id,
            frame_id=frame_id,
            metadata={
                "relocalize": True,
                **dict(metadata or {}),
            },
            force_relocalize=False,  # 状态机已处理重定位
        )

    def get_localization_diagnostics(self) -> Optional[LocalizationDiagnostics]:
        """返回当前平台定位诊断信息。"""

        return self._last_diagnostics

    def _build_localization_diagnostics(
        self,
        provider: LocalizationProvider,
        current_pose: Optional[object],
        session_frame_id: Optional[str],
    ) -> LocalizationDiagnostics:
        """构建平台定位诊断信息。"""

        from adapters.localization.geometry_state_adapter import (
            BackendLocalizationState,
            compute_tracking_quality,
        )

        backend_name = provider.provider_name

        # 尝试获取后端的原始状态
        backend_state = self._capture_backend_state(provider)

        # 坐标系匹配检查
        frame_id_match = True
        if current_pose is not None and session_frame_id is not None:
            frame_id_match = frame_ids_semantically_equal(current_pose.frame_id, session_frame_id)

        # 构建后端状态对象
        state = BackendLocalizationState(
            backend_name=backend_name,
            backend_type=self._resolve_backend_type(provider),
            pose_available=current_pose is not None,
            tracking_state=backend_state.get("tracking_state", "unknown"),
            map_loaded=backend_state.get("map_loaded", False),
            frame_id_match=frame_id_match,
            quality_score=backend_state.get("quality_score", 0.0),
            loop_closure_detected=backend_state.get("loop_closure_detected", False),
            relocalization_in_progress=backend_state.get("relocalization_in_progress", False),
            last_error=backend_state.get("last_error"),
            metadata=backend_state,
        )

        # 计算跟踪质量
        tracking_quality = compute_tracking_quality(state)

        return LocalizationDiagnostics(
            backend_name=backend_name,
            pose_available=current_pose is not None,
            tracking_quality=tracking_quality,
            time_since_last_pose_sec=0.0,  # 由状态机计算
            backend_state=state.tracking_state,
            tracking_lost_count=0,  # 由状态机管理
            relocalization_attempts=self._state_machine._relocalization_attempts if hasattr(self._state_machine, '_relocalization_attempts') else 0,
            last_error=state.last_error,
            metadata={
                "backend_type": state.backend_type,
                "loop_closure": state.loop_closure_detected,
                "relocalizing": state.relocalization_in_progress,
            },
        )

    def _capture_backend_state(
        self,
        provider: LocalizationProvider,
    ) -> Dict[str, object]:
        """捕获后端定位状态（RTAB-Map 或 Go2 SDK）。"""

        # 尝试获取后端状态方法
        get_state = getattr(provider, "get_localization_state", None)
        if callable(get_state):
            try:
                state = get_state()
                if isinstance(state, dict):
                    return state
            except Exception:
                pass

        # 尝试获取原始诊断信息
        get_diagnostics = getattr(provider, "get_localization_diagnostics", None)
        if callable(get_diagnostics):
            try:
                diag = get_diagnostics()
                if hasattr(diag, "model_dump"):
                    return diag.model_dump(mode="json")
                if isinstance(diag, dict):
                    return diag
            except Exception:
                pass

        # 返回默认状态
        return {
            "tracking_state": "unknown",
            "map_loaded": False,
            "quality_score": 0.0,
            "last_error": None,
        }

    def _resolve_backend_type(self, provider: LocalizationProvider) -> str:
        """解析后端类型。"""

        backend_type = getattr(provider, "backend_type", None)
        if backend_type:
            return str(backend_type)

        provider_name = getattr(provider, "provider_name", "")
        if "rtabmap" in provider_name.lower():
            return "rtabmap"
        if "go2" in provider_name.lower():
            return "go2_sdk"
        if "nav2" in provider_name.lower():
            return "nav2"

        return "unknown"

    def _map_machine_state_to_session_status(
        self,
        machine_state: str,
    ) -> LocalizationSessionStatus:
        """将状态机状态映射到 LocalizationSessionStatus。"""

        status_map = {
            "idle": LocalizationSessionStatus.IDLE,
            "localizing": LocalizationSessionStatus.LOCALIZING,
            "ready": LocalizationSessionStatus.READY,
            "stale": LocalizationSessionStatus.STALE,
            "failed": LocalizationSessionStatus.FAILED,
        }
        return status_map.get(machine_state, LocalizationSessionStatus.LOCALIZING)

    def _build_session_last_error(
        self,
        diagnostics: LocalizationDiagnostics,
        session_status: LocalizationSessionStatus,
    ) -> Optional[str]:
        """构建会话级别 last_error 语义。"""

        if diagnostics.last_error:
            return diagnostics.last_error

        if session_status == LocalizationSessionStatus.STALE:
            return "跟踪丢失，尝试恢复中..."

        if session_status == LocalizationSessionStatus.FAILED:
            return "定位失败，需要重新激活地图或重定位。"

        if session_status == LocalizationSessionStatus.LOCALIZING:
            return "定位中..."

        return None

    def refresh_current_session(
        self,
        *,
        metadata: Optional[Dict[str, object]] = None,
    ) -> Optional[LocalizationSession]:
        """刷新当前已存在的平台定位会话。"""

        session = self._active_session
        if session is None:
            return None
        return self.refresh_active_session(
            map_name=session.map_name,
            map_version_id=session.map_version_id,
            frame_id=session.frame_id,
            metadata=metadata,
        )

    def get_active_session(self) -> Optional[LocalizationSession]:
        """返回当前激活的平台定位会话。"""

        return self._active_session

    def clear_active_session(self, *, reason: str = "") -> None:
        """清空当前激活的平台定位会话。"""

        self._active_session = None
        self._state_machine.reset()
        self._last_diagnostics = None
        self._state_store.write(
            StateNamespace.LOCALIZATION,
            "active_session",
            None,
            source="localization_service",
            metadata={"kind": "localization_session", "reason": str(reason or "").strip() or None},
        )

    def ingest_external_snapshot(
        self,
        *,
        source_name: str,
        current_pose=None,
        frame_tree=None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> LocalizationSnapshot:
        """写入来自适配器或外部系统的定位快照。"""

        snapshot = LocalizationSnapshot(
            source_name=source_name,
            current_pose=current_pose,
            frame_tree=frame_tree,
            metadata=dict(metadata or {}),
        )
        self._latest_snapshot = snapshot
        self._history.append(snapshot)
        self._state_store.write(
            StateNamespace.LOCALIZATION,
            source_name,
            snapshot,
            source=source_name,
            metadata={"kind": "latest_localization"},
        )

        if self._event_bus is not None:
            self._event_bus.publish(
                self._event_bus.build_event(
                    "localization.snapshot_ready",
                    category=RuntimeEventCategory.ROBOT,
                    source="localization_service",
                    message="定位快照已更新。",
                    payload={"source_name": source_name},
                )
            )
        return snapshot

    def get_latest_snapshot(self) -> Optional[LocalizationSnapshot]:
        """返回最新定位快照。"""

        return self._latest_snapshot

    def clear_latest_snapshot(self) -> None:
        """清空当前最新定位快照。"""

        self._latest_snapshot = None

    def list_session_history(self, *, limit: Optional[int] = None) -> Tuple[LocalizationSession, ...]:
        """返回定位会话历史。"""

        items = list(self._session_history)
        if limit is not None:
            items = items[-max(0, limit) :]
        return tuple(items)

    def list_history(self, *, limit: Optional[int] = None) -> Tuple[LocalizationSnapshot, ...]:
        """返回定位历史。"""

        items = list(self._history)
        if limit is not None:
            items = items[-max(0, limit) :]
        return tuple(items)

    def is_available(self) -> bool:
        """判断定位服务当前是否可用。"""

        if self._latest_snapshot is not None:
            return True
        provider = self._find_localization_provider()
        if provider is None or not provider.is_available():
            return False
        availability_checker = getattr(provider, "is_localization_available", None)
        if callable(availability_checker):
            return bool(availability_checker())
        return True

    def _get_localization_provider(self) -> LocalizationProvider:
        provider = self._find_localization_provider()
        if provider is None:
            raise GatewayError("当前平台几何运行时未提供定位提供器。")
        if not provider.is_available():
            raise GatewayError("当前定位提供器暂不可用。")
        return provider

    def _capture_provider_data(self, provider: LocalizationProvider):
        return provider.get_current_pose(), provider.get_frame_tree()

    def _set_active_session(self, session: LocalizationSession) -> LocalizationSession:
        self._active_session = session
        self._session_history.append(session)
        self._state_store.write(
            StateNamespace.LOCALIZATION,
            "active_session",
            session,
            source="localization_service",
            metadata={
                "kind": "localization_session",
                "map_name": session.map_name,
                "map_version_id": session.map_version_id,
                "status": session.status.value,
            },
        )
        if self._event_bus is not None:
            self._event_bus.publish(
                self._event_bus.build_event(
                    "localization.session_updated",
                    category=RuntimeEventCategory.ROBOT,
                    source="localization_service",
                    message="平台定位会话已更新。",
                    payload={
                        "map_name": session.map_name,
                        "map_version_id": session.map_version_id,
                        "status": session.status.value,
                    },
                )
            )
        return session

    def _build_session_id(self, *, map_name: str, map_version_id: str) -> str:
        timestamp = int(utc_now().timestamp() * 1000)
        return "locsess_%s_%s_%d" % (
            str(map_name or "").strip(),
            str(map_version_id or "").strip(),
            timestamp,
        )

    def _find_localization_provider(self) -> Optional[LocalizationProvider]:
        providers = getattr(self._provider_owner, "providers", None)
        if providers is None or not isinstance(providers, LocalizationProvider):
            return None
        return providers
