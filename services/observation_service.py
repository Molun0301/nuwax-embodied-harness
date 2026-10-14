from __future__ import annotations

import threading
import time
from collections import deque
from contextlib import contextmanager
from typing import Deque, Dict, Optional, Tuple

from contracts.events import RuntimeEventCategory
from contracts.naming import build_observation_id
from contracts.perception import Observation
from contracts.runtime_views import ObservationContext
from core import EventBus, StateNamespace, StateStore
from core.errors import GatewayError
from providers import ImageProvider
from services.artifact_service import ArtifactService


class ObservationService:
    """观察服务。"""

    def __init__(
        self,
        *,
        provider_owner,
        artifact_service: ArtifactService,
        state_store: StateStore,
        event_bus: Optional[EventBus] = None,
        history_limit: int = 100,
    ) -> None:
        self._provider_owner = provider_owner
        self._artifact_service = artifact_service
        self._state_store = state_store
        self._event_bus = event_bus
        self._history: Deque[ObservationContext] = deque(maxlen=max(1, history_limit))
        self._latest_by_camera: Dict[str, ObservationContext] = {}
        self._context_condition = threading.Condition(threading.RLock())
        self._latest_holds = {}
        self._history_holds = deque()
        self._active_operations = 0
        self._closing = False
        self._closed = False

    def capture_observation(
        self,
        *,
        camera_id: Optional[str] = None,
        requested_by: Optional[str] = None,
    ) -> ObservationContext:
        """抓取当前观察结果。"""

        with self._operation(), self._artifact_service.publication_scope(owner="observation.capture") as hold:
            return self._capture_observation(camera_id=camera_id, requested_by=requested_by, hold=hold)

    def _capture_observation(self, *, camera_id, requested_by, hold):
        image_provider = self._get_image_provider()
        image_frame = image_provider.capture_image(camera_id)
        resolved_camera_id = image_frame.camera_id
        camera_info = image_provider.get_camera_info(resolved_camera_id)
        artifact = self._artifact_service.save_image_frame(
            image_frame,
            metadata={"requested_by": requested_by or "unknown"},
        )

        summary = f"已获取来自 {resolved_camera_id} 的 {image_frame.encoding.value} 图像，分辨率 {image_frame.width_px}x{image_frame.height_px}。"
        latest_state_entry = self._state_store.read_latest(StateNamespace.ROBOT_STATE)
        if latest_state_entry is not None:
            latest_state = latest_state_entry.value
            summary = f"{summary} 当前机器人模式为 {latest_state.mode.value}。"

        observation = Observation(
            observation_id=build_observation_id(self._normalize_camera_id(resolved_camera_id)),
            frame_id=image_frame.frame_id,
            summary=summary,
            artifact_ids=[artifact.artifact_id],
            metadata={
                "camera_id": resolved_camera_id,
                "encoding": image_frame.encoding.value,
                "width_px": image_frame.width_px,
                "height_px": image_frame.height_px,
                "requested_by": requested_by or "unknown",
                "frame_timestamp": image_frame.timestamp.isoformat(),
                "frame_timestamp_origin": "image_frame.timestamp",
                "source_timestamp_ns": image_frame.metadata.get("source_timestamp_ns"),
                "clock_domain": image_frame.metadata.get("clock_domain"),
            },
        )
        context = ObservationContext(
            camera_id=resolved_camera_id,
            observation=observation,
            image_artifact=artifact,
            camera_info=camera_info,
            metadata={
                "frame_id": image_frame.frame_id,
                "size_bytes": len(image_frame.data or b""),
            },
        )
        with self._context_condition:
            self._ensure_open()
            hold.validate_introduced()
            latest_hold = history_hold = None
            try:
                latest_hold = hold.retain(owner="observation.latest")
                history_hold = hold.retain(owner="observation.history")
                self._state_store.write(
                    StateNamespace.OBSERVATION, resolved_camera_id, context,
                    source=image_provider.provider_name, metadata={"kind": "latest_observation"},
                )
            except BaseException:
                for new_hold in (latest_hold, history_hold):
                    if new_hold is not None:
                        new_hold.release()
                raise
            old_latest = self._latest_holds.get(resolved_camera_id)
            old_history = self._history_holds.popleft() if len(self._history) == self._history.maxlen else None
            self._latest_by_camera[resolved_camera_id] = context
            self._latest_holds[resolved_camera_id] = latest_hold
            self._history.append(context)
            self._history_holds.append(history_hold)
            for old in (old_latest, old_history):
                if old is not None:
                    old.release()

        if self._event_bus is not None:
            self._event_bus.publish(
                self._event_bus.build_event(
                    "perception.observation_ready",
                    category=RuntimeEventCategory.PERCEPTION,
                    source="observation_service",
                    subject_id=observation.observation_id,
                    message=summary,
                    payload={
                        "camera_id": resolved_camera_id,
                        "artifact_id": artifact.artifact_id,
                    },
                )
            )
        return context

    def get_latest_observation(self, camera_id: Optional[str] = None) -> Optional[ObservationContext]:
        """返回最新观察。"""

        with self._context_condition:
            return self._select_latest(camera_id)

    def _select_latest(self, camera_id):
        if camera_id is not None:
            return self._latest_by_camera.get(camera_id)
        return max(self._latest_by_camera.values(), key=lambda item: item.timestamp, default=None)

    def acquire_latest_observation(self, camera_id=None, *, owner):
        """在槽位替换锁内同时选择上下文并取得独立读取拥有方。"""
        with self._context_condition:
            self._ensure_open()
            context = self._select_latest(camera_id)
            hold = self._latest_holds.get(context.camera_id) if context is not None else None
            if hold is not None:
                hold.validate_introduced()
            return context, hold.retain(owner=owner) if hold is not None else None

    def list_history(
        self,
        *,
        camera_id: Optional[str] = None,
        limit: Optional[int] = None,
    ) -> Tuple[ObservationContext, ...]:
        """返回观察历史。"""

        with self._context_condition:
            items = list(self._history)
        if camera_id is not None:
            items = [item for item in items if item.camera_id == camera_id]
        if limit is not None:
            items = items[-max(0, limit) :]
        return tuple(items)

    def list_latest_contexts(self) -> Tuple[ObservationContext, ...]:
        """返回各相机当前最新观察。"""

        with self._context_condition:
            items = sorted(self._latest_by_camera.values(), key=lambda item: (item.camera_id, item.timestamp))
        return tuple(items)

    @staticmethod
    def _context_artifact_ids(context):
        observation = getattr(context, "observation", None)
        ids = set(getattr(observation, "artifact_ids", ()) or ())
        image_artifact = getattr(context, "image_artifact", None)
        if image_artifact is not None:
            ids.add(image_artifact.artifact_id)
        return ids

    def acquire_context_artifacts(self, context, *, owner):
        """副本不隐式复制拥有权；短内存证明转交，外部上下文在锁外查验。"""
        ids = self._context_artifact_ids(context)
        if not ids:
            return None
        with self._operation():
            with self._context_condition:
                for item, hold in list(zip(self._history, self._history_holds)) + [
                    (self._latest_by_camera.get(context.camera_id), self._latest_holds.get(context.camera_id))
                ]:
                    if item is context and hold is not None and hold.artifact_ids == ids:
                        hold.validate_introduced()
                        return hold.retain(owner=owner)
            hold = self._artifact_service.acquire_artifacts(ids, owner=owner)
            try:
                hold.validate_introduced()
                return hold
            except BaseException:
                hold.release()
                raise

    def _ensure_open(self):
        if self._closing:
            raise GatewayError("观察服务正在关闭。", error_code="observation_closed", http_status=409)

    @contextmanager
    def _operation(self):
        with self._context_condition:
            self._ensure_open()
            self._active_operations += 1
        try:
            yield
        finally:
            with self._context_condition:
                self._active_operations -= 1
                self._context_condition.notify_all()

    def get_lifecycle_status(self):
        with self._context_condition:
            return {"closing": self._closing, "closed": self._closed,
                    "active_operations": self._active_operations,
                    "latest_holders": len(self._latest_holds), "history_holders": len(self._history_holds)}

    def close(self, timeout=5.0):
        """终止准入；超时保留仍运行操作和可见槽位的拥有权。"""
        deadline = time.monotonic() + max(0.0, float(timeout))
        with self._context_condition:
            self._closing = True
            while self._active_operations:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._context_condition.wait(remaining)
            if self._closed:
                return True
            for camera_id in self._latest_by_camera:
                self._state_store.write(StateNamespace.OBSERVATION, camera_id, None,
                                        source="observation_service", metadata={"kind": "closed"})
            holds = tuple(self._latest_holds.values()) + tuple(self._history_holds)
            self._latest_by_camera.clear()
            self._history.clear()
            self._latest_holds.clear()
            self._history_holds.clear()
            for hold in holds:
                if hold is not None:
                    hold.release()
            self._closed = True
            return True

    def _get_image_provider(self) -> ImageProvider:
        providers = getattr(self._provider_owner, "providers", None)
        if providers is None or not isinstance(providers, ImageProvider):
            raise GatewayError("当前机器人入口未提供图像提供器。")
        if not providers.is_available():
            raise GatewayError("当前图像提供器暂不可用。")
        return providers

    def _normalize_camera_id(self, camera_id: str) -> str:
        normalized = []
        for char in camera_id.lower():
            if char.isalnum():
                normalized.append(char)
            else:
                normalized.append("_")
        value = "".join(normalized).strip("_")
        return value or "camera"
