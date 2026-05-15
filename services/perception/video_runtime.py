from __future__ import annotations

import logging
import threading
import time
from typing import Dict, Iterable, Optional, Tuple
from datetime import datetime

from contracts.base import utc_now
from contracts.geometry import Pose
from contracts.runtime_views import PerceptionRuntimeStatus
from core import EventBus, StateNamespace, StateStore
from services.memory.semantic_map_builder import SemanticMapBuildResult, SemanticMapBuilder
from services.perception.keyframe import PerceptionKeyframeSelector
from services.perception.service import PerceptionService


PERCEPTION_RUNTIME_LOGGER = logging.getLogger("nuwax_robot_bridge.perception.runtime")


class PerceptionVideoRuntime:
    """持续视频感知运行时。

    该运行时只从平台已接收的端侧视频推流 latest/keyframe 缓冲抽帧，
    再复用 `PerceptionService` 完成检测、跟踪与场景摘要；它不触发平台到
    机器人侧的图像采集。
    """

    def __init__(
        self,
        *,
        perception_service: PerceptionService,
        state_store: StateStore,
        localization_service=None,
        mapping_service=None,
        memory_service=None,
        event_bus: Optional[EventBus] = None,
        enabled: bool,
        auto_start: bool,
        camera_id: str,
        interval_sec: float,
        detector_backend_name: str = "",
        store_artifact: bool = False,
        failure_backoff_sec: float = 2.0,
        keyframe_min_interval_sec: float = 1.0,
        keyframe_max_interval_sec: float = 5.0,
        keyframe_translation_threshold_m: float = 0.75,
        keyframe_yaw_threshold_deg: float = 20.0,
        remember_keyframes: bool = True,
        remember_min_interval_sec: float = 15.0,
        burst_frame_count: int = 8,
        burst_interval_sec: float = 0.15,
    ) -> None:
        self._perception_service = perception_service
        self._state_store = state_store
        self._localization_service = localization_service
        self._mapping_service = mapping_service
        self._memory_service = memory_service
        self._event_bus = event_bus
        self._enabled = bool(enabled)
        self._auto_start = bool(auto_start)
        self._camera_id = str(camera_id or "front_camera")
        self._interval_sec = max(0.05, float(interval_sec))
        self._detector_backend_name = str(detector_backend_name or "").strip()
        self._store_artifact = bool(store_artifact)
        self._failure_backoff_sec = max(0.1, float(failure_backoff_sec))
        self._remember_keyframes = bool(remember_keyframes)
        self._remember_min_interval_sec = max(0.0, float(remember_min_interval_sec))
        self._burst_frame_count = max(1, int(burst_frame_count))
        self._burst_interval_sec = max(0.05, float(burst_interval_sec))
        self._semantic_map_builder = SemanticMapBuilder()
        self._keyframe_selector = PerceptionKeyframeSelector(
            min_interval_sec=keyframe_min_interval_sec,
            max_interval_sec=keyframe_max_interval_sec,
            translation_threshold_m=keyframe_translation_threshold_m,
            yaw_threshold_deg=keyframe_yaw_threshold_deg,
        )
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread = None
        self._processed_frames = 0
        self._failure_count = 0
        self._last_keyframe_reason = None
        self._last_memory_write_at = None
        self._last_success_message = None
        self._last_error = None
        self._last_status_metadata = {}
        self._write_status(running=False)

    def is_enabled(self) -> bool:
        """返回功能是否启用。"""

        return self._enabled

    def should_auto_start(self) -> bool:
        """返回是否应在宿主机运行时启动时自动拉起。"""

        if not (self._enabled and self._auto_start):
            return False
        if self._memory_service is None:
            return True
        if hasattr(self._memory_service, "is_enabled"):
            try:
                return bool(self._memory_service.is_enabled())
            except Exception:
                return False
        return True

    def bind_memory_service(self, memory_service) -> None:
        """显式注入 memory_service，避免外部直接赋值私有字段。"""

        self._memory_service = memory_service

    def is_running(self) -> bool:
        """返回后台线程是否处于运行状态。"""

        thread = self._thread
        return bool(thread is not None and thread.is_alive())

    def get_status(self) -> PerceptionRuntimeStatus:
        """返回当前持续感知状态。"""

        with self._lock:
            status = PerceptionRuntimeStatus(
                enabled=self._enabled,
                auto_start=self._auto_start,
                running=self.is_running(),
                camera_id=self._camera_id,
                interval_sec=self._interval_sec,
                detector_backend=self._detector_backend_name or self._perception_service.detector_pipeline.default_backend_name,
                source_name=str(self._last_status_metadata.get("source_name") or ""),
                processed_frames=self._processed_frames,
                failure_count=self._failure_count,
                last_success_message=self._last_success_message,
                last_error=self._last_error,
                metadata=dict(self._last_status_metadata),
            )
        return status

    def start(self) -> PerceptionRuntimeStatus:
        """启动后台持续感知线程。"""

        if not self._enabled:
            return self.get_status()
        with self._lock:
            if self.is_running():
                return self.get_status()
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._run_loop,
                name="nuwax_perception_video_runtime",
                daemon=True,
            )
            self._thread.start()
            status = self.get_status()
        PERCEPTION_RUNTIME_LOGGER.info(
            "持续视频感知运行时已启动 camera=%s interval=%.3fs detector=%s",
            status.camera_id,
            status.interval_sec,
            status.detector_backend or "default",
        )
        self._write_status(running=True)
        return status

    def stop(self) -> PerceptionRuntimeStatus:
        """停止后台持续感知线程。"""

        thread = None
        with self._lock:
            thread = self._thread
            self._stop_event.set()
        if thread is not None and thread.is_alive():
            thread.join(timeout=max(1.0, self._interval_sec + self._failure_backoff_sec + 0.5))
        with self._lock:
            self._thread = None
            status = self.get_status()
        self._write_status(running=False)
        PERCEPTION_RUNTIME_LOGGER.info("持续视频感知运行时已停止 camera=%s", status.camera_id)
        return status

    def run_once(
        self,
        *,
        force_keyframe: bool = False,
        reason: str = "manual",
        allow_memory_write: bool = True,
        detection_metadata: Optional[Dict[str, object]] = None,
        open_vocabulary_classes: Optional[Iterable[str]] = None,
    ) -> Optional[str]:
        """手动执行一次抓帧与感知，返回摘要标题。"""

        if not self._enabled:
            self._last_error = "持续视频感知功能未启用。"
            self._write_status(running=self.is_running())
            return None

        now = utc_now()
        current_pose, map_context = self._resolve_runtime_context()
        decision = self._keyframe_selector.decide(
            now=now,
            current_pose=current_pose,
            map_context=map_context,
        )
        video_status = self._perception_service.video_frame_source_status()
        if not bool(video_status.get("available")):
            self._perception_service.publish_current_video_frame(
                camera_id=self._camera_id,
                requested_by="perception_video_runtime",
            )
            with self._lock:
                self._last_error = "端侧视频推流未就绪或最近帧已过期。"
                self._last_status_metadata = {
                    **dict(self._last_status_metadata),
                    "camera_id": self._camera_id,
                    "capture_mode": "video_stream",
                    "last_skip_reason": "video_ingest_not_ready",
                    "video_ingest_status": dict(video_status),
                    **dict(decision.metadata),
                }
            self._write_status(running=self.is_running())
            return None
        effective_reason = reason
        if not force_keyframe:
            if not decision.should_process:
                self._perception_service.publish_current_video_frame(
                    camera_id=self._camera_id,
                    requested_by="perception_video_runtime",
                )
                with self._lock:
                    self._last_error = None
                    self._last_status_metadata = {
                        **dict(self._last_status_metadata),
                        "camera_id": self._camera_id,
                        "capture_mode": "video_stream",
                        "last_skip_reason": decision.reason,
                        "video_ingest_status": dict(video_status),
                        **dict(decision.metadata),
                    }
                self._write_status(running=self.is_running())
                return None
            effective_reason = decision.reason

        context = self._perception_service.perceive_current_scene(
            camera_id=self._camera_id,
            requested_by="perception_video_runtime",
            detector_backend_name=self._detector_backend_name or None,
            store_artifact=self._store_artifact,
            detection_metadata=detection_metadata,
            open_vocabulary_classes=open_vocabulary_classes,
        )
        self._keyframe_selector.mark_processed(
            timestamp=now,
            current_pose=current_pose,
            map_context=map_context,
        )
        memory_written = self._remember_from_keyframe(
            context=context,
            current_pose=current_pose,
            map_context=map_context,
            reason=effective_reason,
            now=now,
            force=force_keyframe,
            allow_memory_write=allow_memory_write,
        )
        with self._lock:
            self._processed_frames += 1
            self._last_keyframe_reason = effective_reason
            self._last_error = None
            self._last_success_message = context.scene_summary.headline
            self._last_status_metadata = {
                "pipeline_name": context.pipeline_name,
                "camera_id": context.camera_id,
                "source_name": str(context.metadata.get("source_name") or ""),
                "detector_backend": context.detector_backend,
                "tracker_backend": context.tracker_backend,
                "observation_id": context.observation.observation_id,
                "frame_id": context.observation.frame_id,
                "detection_count": context.scene_summary.detection_count,
                "active_track_count": context.scene_summary.active_track_count,
                "store_artifact": self._store_artifact,
                "capture_mode": "keyframe" if not force_keyframe else "burst",
                "keyframe_reason": effective_reason,
                "memory_written": memory_written,
                "remember_keyframes": self._remember_keyframes,
                "detection_metadata": dict(detection_metadata or {}),
                "video_ingest_status": dict(video_status),
                **dict(decision.metadata),
            }
        self._write_status(running=self.is_running())
        return context.scene_summary.headline

    def run_burst(
        self,
        *,
        frame_count: Optional[int] = None,
        interval_sec: Optional[float] = None,
        requested_reason: str = "burst_verify",
        store_artifact: Optional[bool] = None,
        remember_result: bool = False,
        detection_metadata: Optional[Dict[str, object]] = None,
        open_vocabulary_classes: Optional[Iterable[str]] = None,
    ) -> Tuple[str, ...]:
        """执行短时 burst 复核，返回成功处理的摘要列表。"""

        if not self._enabled:
            return ()
        effective_count = max(1, int(frame_count or self._burst_frame_count))
        effective_interval_sec = max(0.05, float(interval_sec or self._burst_interval_sec))
        original_store_artifact = self._store_artifact
        if store_artifact is not None:
            self._store_artifact = bool(store_artifact)
        messages = []
        try:
            for _ in range(effective_count):
                message = self.run_once(
                    force_keyframe=True,
                    reason=requested_reason,
                    allow_memory_write=remember_result,
                    detection_metadata=detection_metadata,
                    open_vocabulary_classes=open_vocabulary_classes,
                )
                if message:
                    messages.append(message)
                if effective_interval_sec > 0:
                    time.sleep(effective_interval_sec)
        finally:
            self._store_artifact = original_store_artifact
        return tuple(messages)

    def _run_loop(self) -> None:
        while not self._stop_event.is_set():
            frame_start = time.monotonic()
            try:
                self.run_once(force_keyframe=False, reason="background_keyframe")
            except Exception as exc:
                with self._lock:
                    self._failure_count += 1
                    self._last_error = str(exc)
                    self._last_status_metadata = {
                        **dict(self._last_status_metadata),
                        "camera_id": self._camera_id,
                    }
                self._write_status(running=True)
                PERCEPTION_RUNTIME_LOGGER.exception("持续视频感知处理失败 camera=%s", self._camera_id)
                if self._stop_event.wait(self._failure_backoff_sec):
                    break
                continue

            remaining = self._interval_sec - (time.monotonic() - frame_start)
            if remaining > 0 and self._stop_event.wait(remaining):
                break

    def _resolve_runtime_context(self) -> Tuple[Optional[Pose], Optional[SemanticMapBuildResult]]:
        current_pose = None
        map_context = None
        localization_service = self._localization_service
        if localization_service is not None:
            snapshot = self._refresh_service_snapshot(localization_service)
            if snapshot is not None:
                current_pose = snapshot.current_pose

        mapping_service = self._mapping_service
        if mapping_service is not None:
            map_snapshot = self._refresh_service_snapshot(mapping_service)
            map_context = self._semantic_map_builder.build(map_snapshot)
        return current_pose, map_context

    def _refresh_service_snapshot(self, service):
        refresher = getattr(service, "refresh", None)
        if callable(refresher):
            try:
                return refresher()
            except Exception:
                PERCEPTION_RUNTIME_LOGGER.debug("刷新持续感知上下文失败。", exc_info=True)
        getter = getattr(service, "get_latest_snapshot", None)
        if callable(getter):
            return getter()
        return None

    def _remember_from_keyframe(
        self,
        *,
        context,
        current_pose: Optional[Pose],
        map_context: Optional[SemanticMapBuildResult],
        reason: str,
        now: datetime,
        force: bool,
        allow_memory_write: bool,
    ) -> bool:
        del current_pose, map_context
        if not allow_memory_write:
            return False
        if self._memory_service is None:
            return False
        if not self._remember_keyframes and not force:
            return False
        if hasattr(self._memory_service, "is_enabled") and not self._memory_service.is_enabled():
            return False
        if not force and self._last_memory_write_at is not None:
            elapsed_sec = max(0.0, (now - self._last_memory_write_at).total_seconds())
            if elapsed_sec < self._remember_min_interval_sec:
                return False
        try:
            self._memory_service.remember_current_scene(
                title="自动场景记忆：%s" % context.scene_summary.headline[:48],
                camera_id=context.camera_id,
                summary_override=context.scene_summary.headline,
                tags=self._build_memory_tags(context),
                metadata={
                    "requested_by": "perception_video_runtime",
                    "source": "perception_video_runtime",
                    "capture_mode": "burst" if force else "keyframe",
                    "keyframe_reason": reason,
                },
            )
        except Exception as exc:
            PERCEPTION_RUNTIME_LOGGER.warning(
                "关键帧记忆写入失败，已跳过本次记忆 camera=%s reason=%s error=%s",
                context.camera_id,
                reason,
                exc,
            )
            return False
        self._last_memory_write_at = now
        return True

    def _build_memory_tags(self, context) -> Tuple[str, ...]:
        tags = []
        for item in context.scene_summary.objects:
            label = str(item.label).strip().lower()
            if label:
                tags.append(label)
        for tag in context.scene_summary.metadata.get("semantic_tags") or []:
            normalized = str(tag).strip().lower()
            if normalized:
                tags.append(normalized)
        seen = set()
        result = []
        for item in tags:
            if item in seen:
                continue
            seen.add(item)
            result.append(item)
        return tuple(result[:12])

    def _write_status(self, *, running: bool) -> None:
        status = self.get_status().model_copy(update={"running": running}, deep=True)
        self._state_store.write(
            StateNamespace.SYSTEM,
            "perception_video_runtime",
            status,
            source="perception_video_runtime",
            metadata={"kind": "perception_video_runtime"},
        )
