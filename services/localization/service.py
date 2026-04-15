from __future__ import annotations

from collections import deque

from contracts.base import utc_now
from contracts.events import RuntimeEventCategory
from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.runtime_views import LocalizationSession, LocalizationSessionStatus, LocalizationSnapshot
from core import EventBus, StateNamespace, StateStore
from gateways.errors import GatewayError
from providers import LocalizationProvider
from typing import Deque, Dict, Optional, Tuple


class LocalizationService:
    """统一定位与 TF 快照服务。"""

    def __init__(
        self,
        *,
        provider_owner,
        state_store: StateStore,
        event_bus: Optional[EventBus] = None,
        history_limit: int = 100,
    ) -> None:
        self._provider_owner = provider_owner
        self._state_store = state_store
        self._event_bus = event_bus
        self._history: Deque[LocalizationSnapshot] = deque(maxlen=max(1, history_limit))
        self._session_history: Deque[LocalizationSession] = deque(maxlen=max(1, history_limit))
        self._latest_snapshot: Optional[LocalizationSnapshot] = None
        self._active_session: Optional[LocalizationSession] = None

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
    ) -> LocalizationSession:
        """刷新当前平台定位会话，并把结果绑定到指定地图版本。"""

        session = self.ensure_active_session(
            map_name=map_name,
            map_version_id=map_version_id,
            frame_id=frame_id,
            metadata=metadata,
        )
        provider = self._get_localization_provider()
        current_pose, frame_tree = self._capture_provider_data(provider)
        usable_pose = current_pose
        last_error = None
        if current_pose is None:
            usable_pose = None
            last_error = "当前定位提供器还没有可用位姿。"
        elif session.frame_id and not frame_ids_semantically_equal(current_pose.frame_id, session.frame_id):
            usable_pose = None
            last_error = "定位提供器位姿坐标系 %s 与目标地图坐标系 %s 不一致。" % (
                current_pose.frame_id,
                session.frame_id,
            )
        localized = usable_pose is not None
        snapshot = self.ingest_external_snapshot(
            source_name="platform_localization:%s" % session.map_name,
            current_pose=usable_pose,
            frame_tree=frame_tree if localized else None,
            metadata={
                "source_kind": "platform_localization_session",
                "map_name": session.map_name,
                "platform_map_version_id": session.map_version_id,
                "localization_session_id": session.session_id,
                "provider_name": provider.provider_name,
                "provider_version": provider.provider_version,
                "provider_pose_available": current_pose is not None,
                "provider_pose_frame_id": current_pose.frame_id if current_pose is not None else None,
                **dict(metadata or {}),
            },
        )
        updated = session.model_copy(
            update={
                "status": LocalizationSessionStatus.READY if localized else LocalizationSessionStatus.LOCALIZING,
                "source_name": provider.provider_name,
                "pose_available": localized,
                "last_localized_at": utc_now() if localized else session.last_localized_at,
                "last_error": last_error,
                "metadata": {
                    **dict(session.metadata),
                    **dict(snapshot.metadata),
                },
            },
            deep=True,
        )
        self._set_active_session(updated)
        return updated

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
