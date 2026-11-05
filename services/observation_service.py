from __future__ import annotations

import threading
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

        with self._operation():
            return self._capture_observation(camera_id=camera_id, requested_by=requested_by)

    def _capture_observation(self, *, camera_id, requested_by):
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
            self._state_store.write(
                StateNamespace.OBSERVATION, resolved_camera_id, context,
                source=image_provider.provider_name, metadata={"kind": "latest_observation"},
            )
            self._latest_by_camera[resolved_camera_id] = context
            self._history.append(context)

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
                    "active_operations": self._active_operations}

    def close(self) -> None:
        """结束新抓图准入，等待在途抓图退出后清空可见快照。"""
        with self._context_condition:
            self._closing = True
            while self._active_operations:
                self._context_condition.wait()
            if self._closed:
                return
            for camera_id in self._latest_by_camera:
                self._state_store.write(StateNamespace.OBSERVATION, camera_id, None,
                                        source="observation_service", metadata={"kind": "closed"})
            self._latest_by_camera.clear()
            self._history.clear()
            self._closed = True

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
