from __future__ import annotations

import logging
import threading
import time
from contextlib import nullcontext
from datetime import datetime
from typing import Dict, Iterable, Optional, Tuple

from contracts.base import utc_now
from contracts.geometry import Pose
from contracts.runtime_views import PerceptionRuntimeStatus
from core import EventBus, StateNamespace, StateStore
from core.errors import GatewayError
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
        live_pose_provider=None,
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
        keyframe_translation_threshold_m: float = 0.75,
        keyframe_yaw_threshold_deg: float = 20.0,
        remember_keyframes: bool = True,
        burst_frame_count: int = 8,
        burst_interval_sec: float = 0.15,
        # 定位模式（mapping_localization）下默认不联动后台感知线程；
        # 只影响 auto_start 时的宿主机启动联动决策，不影响手动 start_perception_runtime MCP。
        skip_when_localization: bool = True,
    ) -> None:
        self._perception_service = perception_service
        self._state_store = state_store
        self._localization_service = localization_service
        self._live_pose_provider = live_pose_provider
        self._mapping_service = mapping_service
        self._memory_service = memory_service
        self._event_bus = event_bus
        self._enabled = bool(enabled)
        self._auto_start = bool(auto_start)
        self._skip_when_localization = bool(skip_when_localization)
        self._camera_id = str(camera_id or "front_camera")
        self._interval_sec = max(0.05, float(interval_sec))
        self._detector_backend_name = str(detector_backend_name or "").strip()
        self._store_artifact = bool(store_artifact)
        self._failure_backoff_sec = max(0.1, float(failure_backoff_sec))
        self._remember_keyframes = bool(remember_keyframes)
        self._burst_frame_count = max(1, int(burst_frame_count))
        self._burst_interval_sec = max(0.05, float(burst_interval_sec))
        self._keyframe_selector = PerceptionKeyframeSelector(
            min_interval_sec=keyframe_min_interval_sec,
            translation_threshold_m=keyframe_translation_threshold_m,
            yaw_threshold_deg=keyframe_yaw_threshold_deg,
        )
        self._lock = threading.RLock()
        self._activity_condition = threading.Condition(self._lock)
        self._active_runs = 0
        self._active_schedules = 0
        self._closing = False
        self._closed = False
        # P1-2：串行 run_once 的三个入口（background 后台线程 / burst 复核 / manual 能力调用），
        # 避免并发触发感知管线与共享运行时状态竞态。与 _lock（护状态字段）分开，
        # 因为 run_once 全程持有此锁、内部还会取 _lock 更新状态。
        self._run_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self._processed_frames = 0
        self._last_processed_observation_id = None
        self._memory_write_thread = None
        self._failure_count = 0
        self._last_keyframe_reason = None
        self._last_success_message = None
        self._last_error = None
        self._last_status_metadata = {}
        # vis-three-fixes 任务 2.6：启动期预热状态，由网关装配期写入，get_status 透传。
        self._prewarm_status = {"status": "pending"}
        # 任务 6.4：性能摘要日志节流（5 分钟一次），避免每 tick 刷屏。
        self._perf_log_interval_sec = 300.0
        self._last_perf_log_at = 0.0
        self._fast_processed_frames = 0
        self._continuous_ingest_count = 0
        self._last_fast_loop_duration_ms = None
        self._last_video_detections_interval_ms = None
        self._memory_write_lock = threading.Lock()
        self._memory_write_in_flight = False
        self._memory_write_pending = None
        self._memory_write_current = None
        self._memory_write_scheduled_count = 0
        self._memory_write_completed_count = 0
        self._memory_write_failed_count = 0
        self._last_memory_write_error = None
        # 跳帧诊断限流：避免 0.5s 循环刷屏，同时让"视频不可用"等失败可见。
        self._skip_log_interval_sec = 10.0
        self._last_skip_log_at = 0.0
        self._last_skip_log_reason = None
        self._consecutive_skip_count = 0
        self._write_status(running=False)

    def is_enabled(self) -> bool:
        """返回功能是否启用。"""

        return self._enabled

    def should_auto_start(self) -> bool:
        """返回是否应在宿主机运行时启动时自动拉起。"""

        if not (self._enabled and self._auto_start):
            return False
        # 定位模式门控：live_pose_provider 是 PlatformGeometryProviderBundle 鸭子类型代理，
        # 它不直接暴露 _external_geometry_process_is_localization；改用上游 platform_geometry_runtime
        # 经由 robot 装配层或外部 owner 引用读取——这里取一个不破坏封装的判定方法 is_localization_mode()。
        if self._skip_when_localization and self._is_localization_mode():
            return False
        if self._memory_service is None:
            return True
        if hasattr(self._memory_service, "is_enabled"):
            try:
                return bool(self._memory_service.is_enabled())
            except Exception:
                return False
        return True

    def _is_localization_mode(self) -> bool:
        """判断平台几何运行时当前是否处于定位模式。

        通过 live_pose_provider（=PlatformGeometryProviderBundle）暴露的
        is_localization_available / _external_geometry_process_is_localization /
        activation_backend_mode 三个口径来判定，任一为 True 即视为定位模式。
        鸭子类型：live_pose_provider 为 None 时返回 False（系统尚未装配完）。
        """

        provider = self._live_pose_provider
        if provider is None:
            return False
        checker = getattr(provider, "is_localization_active", None)
        if callable(checker):
            try:
                return bool(checker())
            except Exception:
                pass
        direct = getattr(provider, "_external_geometry_process_is_localization", None)
        if callable(direct):
            try:
                if bool(direct()):
                    return True
            except Exception:
                pass
        mode_getter = getattr(provider, "get_active_backend_mode", None)
        if callable(mode_getter):
            try:
                if str(mode_getter() or "") == "mapping_localization":
                    return True
            except Exception:
                pass
        return False

    def apply_tuning(self, config) -> None:
        """在两帧之间更新采样和关键帧阈值，保留任务、模型与历史基准。"""
        with self._run_lock, self._lock:
            self._interval_sec = config.interval_sec
            self._failure_backoff_sec = config.failure_backoff_sec
            self._remember_keyframes = config.remember_keyframes
            self._burst_frame_count = config.burst_frame_count
            self._burst_interval_sec = config.burst_interval_sec
            self._keyframe_selector.update_thresholds(
                min_interval_sec=config.keyframe_min_interval_sec,
                translation_threshold_m=config.keyframe_translation_threshold_m,
                yaw_threshold_deg=config.keyframe_yaw_threshold_deg,
            )

    def bind_memory_service(self, memory_service) -> None:
        """显式注入 memory_service，避免外部直接赋值私有字段。"""

        self._memory_service = memory_service

    def set_prewarm_status(self, prewarm_status: Dict[str, object]) -> None:
        """注入启动期预热状态（vis-three-fixes 任务 2.6）。

        网关装配期调 prewarm_yolo_world 后写入；get_status 在 metadata.prewarm_status 透传。
        """

        with self._lock:
            self._prewarm_status = dict(prewarm_status or {})

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
                metadata={
                    **dict(self._last_status_metadata),
                    "prewarm_status": dict(self._prewarm_status),
                    "lifecycle": self.get_lifecycle_status(),
                    "stopping": self._stop_event.is_set() and self.is_running(),
                    "memory_write_in_flight": self._memory_write_in_flight,
                    "memory_write_completed_count": self._memory_write_completed_count,
                    "memory_write_failed_count": self._memory_write_failed_count,
                    "last_memory_write_error": self._last_memory_write_error,
                },
            )
        return status

    def start(self) -> PerceptionRuntimeStatus:
        """启动后台持续感知线程。"""

        if not self._enabled:
            return self.get_status()
        with self._lock:
            if self._closing:
                raise GatewayError("持续感知运行时已关闭。", error_code="perception_runtime_closed", http_status=409)
            with self._memory_write_lock:
                memory_busy = self._memory_write_in_flight or bool(
                    self._memory_write_thread is not None and self._memory_write_thread.is_alive()
                )
            if self.is_running() or memory_busy or self._active_runs or self._active_schedules:
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

    def get_lifecycle_status(self):
        """只读取有界运行态；不查文件、模型或记忆仓储。"""
        with self._lock, self._memory_write_lock:
            memory_thread = self._memory_write_thread
            return {"closing": self._closing, "closed": self._closed,
                    "active_operations": self._active_runs + self._active_schedules,
                    "running": self.is_running(),
                    "memory_write_pending": self._memory_write_pending is not None,
                    "memory_write_in_flight": self._memory_write_in_flight,
                    "memory_worker_alive": bool(memory_thread is not None and memory_thread.is_alive()),
                    "pending_artifact_count": self._job_artifact_count(self._memory_write_pending),
                    "inflight_artifact_count": self._job_artifact_count(self._memory_write_current)}

    @staticmethod
    def _job_artifact_count(job):
        hold = job.get("artifact_hold") if job is not None else None
        return len(hold.artifact_ids) if hold is not None else 0

    @staticmethod
    def _release_job(job):
        if job is not None:
            hold = job.pop("artifact_hold", None)
            if hold is not None:
                hold.release()

    def stop(self, timeout=None) -> PerceptionRuntimeStatus:
        """可复用停止；有界超时保留线程和正在写入的拥有方。"""
        duration = max(1.0, self._interval_sec + self._failure_backoff_sec + 0.5) if timeout is None else max(0.0, float(timeout))
        deadline = time.monotonic() + duration
        with self._lock:
            thread = self._thread
            self._stop_event.set()
        with self._memory_write_lock:
            pending = self._memory_write_pending
            self._memory_write_pending = None
            memory_thread = self._memory_write_thread
        self._release_job(pending)
        for worker in (thread, memory_thread):
            if worker is not None and worker is not threading.current_thread() and worker.is_alive():
                worker.join(timeout=max(0.0, deadline - time.monotonic()))
        with self._activity_condition:
            while self._active_runs or self._active_schedules:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._activity_condition.wait(remaining)
            if thread is not None and not thread.is_alive() and self._thread is thread:
                self._thread = None
            status = self.get_status()
        self._write_status(running=status.running)
        return status

    def close(self, timeout=5.0):
        """终止新准入；调用方仅在返回 True 后才能关闭记忆等依赖。"""
        with self._lock:
            self._closing = True
        self.stop(timeout=timeout)
        with self._lock, self._memory_write_lock:
            memory_thread = self._memory_write_thread
            self._closed = not (
                self._active_runs or self._active_schedules or self.is_running()
                or self._memory_write_in_flight or self._memory_write_pending is not None
                or (memory_thread is not None and memory_thread.is_alive())
            )
            return self._closed

    def run_once(
        self,
        *,
        force_keyframe: bool = False,
        reason: str = "manual",
        allow_memory_write: bool = True,
        detection_metadata: Optional[Dict[str, object]] = None,
        open_vocabulary_classes: Optional[Iterable[str]] = None,
    ) -> Optional[str]:
        """手动执行一次抓帧与感知，返回摘要标题。

        P1-2：以 _run_lock 串行 background/burst/manual 三入口，避免并发触发
        感知管线与运行时状态竞态。
        """

        with self._activity_condition:
            if self._closing:
                raise GatewayError("持续感知运行时已关闭。", error_code="perception_runtime_closed", http_status=409)
            self._active_runs += 1
        try:
            with self._run_lock:
                with self._lock:
                    if self._closing:
                        raise GatewayError("持续感知运行时已关闭。", error_code="perception_runtime_closed", http_status=409)
                return self._run_once_locked(
                    force_keyframe=force_keyframe,
                    reason=reason,
                    allow_memory_write=allow_memory_write,
                    detection_metadata=detection_metadata,
                    open_vocabulary_classes=open_vocabulary_classes,
                )

        finally:
            with self._activity_condition:
                self._active_runs -= 1
                self._activity_condition.notify_all()

    def _run_once_locked(
        self,
        *,
        force_keyframe: bool = False,
        reason: str = "manual",
        allow_memory_write: bool = True,
        detection_metadata: Optional[Dict[str, object]] = None,
        open_vocabulary_classes: Optional[Iterable[str]] = None,
    ) -> Optional[str]:
        """run_once 的实际实现，调用方须已持有 _run_lock。"""

        if not self._enabled:
            self._last_error = "持续视频感知功能未启用。"
            self._write_status(running=self.is_running())
            return None

        now = utc_now()
        library_getter = getattr(self._memory_service, "get_active_library_name", None)
        observation_library = library_getter() if callable(library_getter) else None
        current_pose, map_snapshot, localization_snapshot = self._resolve_runtime_context()
        decision = self._keyframe_selector.decide(
            now=now,
            current_pose=current_pose,
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
            self._log_skip(
                reason="video_ingest_not_ready",
                level=logging.WARNING,
                detail="端侧视频推流未就绪（共享内存/视频源不可用），持续感知本次跳过。video_status=%s" % dict(video_status),
            )
            self._write_status(running=self.is_running())
            return None

        fast_started_at = time.monotonic()
        # 高频检测观测链路：每 tick 只执行 detector + 2D tracker + video.detections。
        # segmentation/geometry/scene_description/keyframe memory 属于慢链路，不能反压下一次 YOLO tick。
        try:
            context = self._perception_service.detect_current_frame_fast(
                camera_id=self._camera_id,
                requested_by="perception_video_runtime",
                detector_backend_name=self._detector_backend_name or None,
                detection_metadata=detection_metadata,
                open_vocabulary_classes=open_vocabulary_classes,
            )
        except GatewayError as exc:
            if exc.error_code != "video_ingest_not_ready":
                raise
            # 共享帧读取器可能尚无新帧；等待输入不属于模型失败，也不触发失败退避。
            self._last_status_metadata = {**self._last_status_metadata, "waiting_for_frame": True}
            self._last_error = None
            self._write_status(running=self.is_running())
            return None
        detector_status = str(dict(context.metadata.get("stage_status") or {}).get("detector") or "completed")
        if detector_status not in {"completed", "fallback"}:
            self._last_error = "检测模型尚未完成有效推理：%s" % detector_status
            self._write_status(running=self.is_running())
            return None
        observation_id = context.observation.observation_id
        if observation_id == self._last_processed_observation_id:
            return None
        self._last_processed_observation_id = observation_id
        fast_loop_duration_ms = round((time.monotonic() - fast_started_at) * 1000.0, 3)
        stage_duration_ms = dict(context.metadata.get("stage_duration_ms") or {})
        last_full_stage_duration_ms_total = self._latest_full_stage_duration_ms_total()
        video_detections_interval_ms = self._coerce_optional_float(
            context.observation.metadata.get("publish_interval_ms")
        )
        if video_detections_interval_ms is None:
            latest_detection_frame = self._latest_stream_frame("video.detections")
            if latest_detection_frame is not None:
                video_detections_interval_ms = self._coerce_optional_float(
                    (latest_detection_frame.payload or {}).get("publish_interval_ms")
                    if isinstance(latest_detection_frame.payload, dict)
                    else None
                )
        write_memory = force_keyframe or decision.should_process
        effective_reason = decision.reason if (decision.should_process and not force_keyframe) else reason
        memory_written = False
        continuous_scheduled = False
        memory_scope_valid = not callable(library_getter) or (
            observation_library is not None and library_getter() == observation_library
        )
        allow_memory_write = allow_memory_write and memory_scope_valid
        if write_memory:
            memory_written = self._schedule_memory_write(
                context=context,
                current_pose=current_pose,
                map_snapshot=map_snapshot,
                localization_snapshot=localization_snapshot,
                expected_library_name=observation_library,
                reason=effective_reason,
                now=now,
                force=force_keyframe,
                allow_memory_write=allow_memory_write,
            )
            if memory_written:
                self._keyframe_selector.mark_processed(timestamp=now, current_pose=current_pose)
        if not memory_written and allow_memory_write and not self._stop_event.is_set():
            continuous_scheduled = self._schedule_memory_write(
                context=context,
                map_snapshot=map_snapshot,
                localization_snapshot=localization_snapshot,
                current_pose=current_pose,
                now=now,
                expected_library_name=observation_library,
                reason="continuous",
                force=False,
                allow_memory_write=True,
                continuous=True,
            )

        capture_mode = "burst" if force_keyframe else ("keyframe" if decision.should_process else "continuous")
        with self._lock:
            self._processed_frames += 1
            self._fast_processed_frames += 1
            self._last_fast_loop_duration_ms = fast_loop_duration_ms
            self._last_video_detections_interval_ms = video_detections_interval_ms
            self._last_keyframe_reason = effective_reason
            self._last_error = None
            self._consecutive_skip_count = 0
            self._last_skip_log_reason = None
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
                "capture_mode": capture_mode,
                "keyframe_reason": effective_reason,
                "memory_written": False,
                "memory_write_scheduled": memory_written,
                "remember_keyframes": self._remember_keyframes,
                "detection_metadata": dict(detection_metadata or {}),
                "video_ingest_status": dict(video_status),
                "fast_path": True,
                "fast_processed_frames": self._fast_processed_frames,
                "continuous_ingest_count": self._continuous_ingest_count,
                "continuous_ingest_scheduled": continuous_scheduled,
                "fast_loop_duration_ms": fast_loop_duration_ms,
                "detector_duration_ms": stage_duration_ms.get("detector"),
                "tracker_duration_ms": stage_duration_ms.get("tracker"),
                "worker_inference_ms": dict(context.metadata.get("detection_metadata") or {}).get("worker_inference_ms"),
                "pipeline_lock_wait_ms": context.metadata.get("pipeline_lock_wait_ms"),
                "nms_iou_threshold": self._perception_service.detector_pipeline.nms_iou_threshold,
                "nms_stats": dict(context.metadata.get("detection_metadata") or {}).get("postprocess"),
                "video_detections_interval_ms": video_detections_interval_ms,
                "last_full_stage_duration_ms_total": last_full_stage_duration_ms_total,
                "memory_write_in_flight": self._memory_write_in_flight,
                "memory_write_scheduled_count": self._memory_write_scheduled_count,
                "memory_write_completed_count": self._memory_write_completed_count,
                "memory_write_failed_count": self._memory_write_failed_count,
                "last_memory_write_error": self._last_memory_write_error,
                **dict(decision.metadata),
            }
        self._write_status(running=self.is_running())
        return context.scene_summary.headline

    def _ingest_continuous_observation(
        self,
        *,
        context,
        map_snapshot,
        localization_snapshot,
        current_pose: Optional[Pose],
        now: datetime,
        expected_library_name: Optional[str] = None,
    ) -> bool:
        """非关键帧 tick 的连续感知摄入：关联 + 射线累积，不落盘不向量化。

        方案 B：让 track_id 帧间连续、射线累积进同一实例的 deferred_projection_cache，
        三角化得以获得多视角输入。memory_service 缺失或未启用时静默跳过。
        """
        memory_service = self._memory_service
        if memory_service is None or current_pose is None:
            return False
        if hasattr(memory_service, "is_enabled") and not memory_service.is_enabled():
            return False
        ingest = getattr(memory_service, "ingest_perception_observation", None)
        if not callable(ingest):
            return False
        try:
            # 与记忆库切换共享已有结构锁，避免识别过程中切库后回灌旧图观察。
            with getattr(memory_service, "_instances_lock", nullcontext()):
                library_getter = getattr(memory_service, "get_active_library_name", None)
                if expected_library_name is not None and callable(library_getter) and library_getter() != expected_library_name:
                    return False
                ingest(
                    perception_context=context,
                    map_snapshot=map_snapshot,
                    localization_snapshot=localization_snapshot,
                    current_pose=current_pose,
                    camera_id=self._camera_id,
                    now=now,
                    persist=False,
                )
            return True
        except Exception:
            PERCEPTION_RUNTIME_LOGGER.debug(
                "连续感知摄入失败 camera=%s。", self._camera_id, exc_info=True
            )
            return False

    def _schedule_memory_write(
        self,
        *,
        context,
        current_pose: Optional[Pose],
        reason: str,
        now: datetime,
        force: bool,
        allow_memory_write: bool,
        map_snapshot=None,
        localization_snapshot=None,
        expected_library_name: Optional[str] = None,
        continuous: bool = False,
    ) -> bool:
        """关键帧与连续观察共用后台工作线程，检测线程只提交最新候选。"""

        with self._activity_condition:
            if self._closing or self._stop_event.is_set():
                return False
            self._active_schedules += 1
        try:
            return self._schedule_memory_write_owned(
                context=context, current_pose=current_pose, reason=reason, now=now, force=force,
                allow_memory_write=allow_memory_write, map_snapshot=map_snapshot,
                localization_snapshot=localization_snapshot, expected_library_name=expected_library_name,
                continuous=continuous,
            )
        finally:
            with self._activity_condition:
                self._active_schedules -= 1
                self._activity_condition.notify_all()

    def _schedule_memory_write_owned(self, *, context, current_pose, reason, now, force,
                                     allow_memory_write, map_snapshot, localization_snapshot,
                                     expected_library_name, continuous):
        if not allow_memory_write or self._stop_event.is_set():
            return False
        memory_service = self._memory_service
        if memory_service is None:
            return False
        if not continuous and not self._remember_keyframes and not force:
            return False
        if continuous and current_pose is None:
            return False
        if hasattr(memory_service, "is_enabled"):
            try:
                if not memory_service.is_enabled():
                    return False
            except Exception:
                return False
        library_getter = getattr(memory_service, "get_active_library_name", None)
        current_library = library_getter() if callable(library_getter) else None
        if expected_library_name is not None and current_library != expected_library_name:
            return False
        # 空快速帧不访问制品服务；非空引用必须先取得独立队列拥有方。
        ids = PerceptionService._context_artifact_ids(context)
        hold = self._perception_service.acquire_context_artifacts(
            context, owner="perception.video_queue",
        ) if ids else None
        job = {
            "continuous": continuous,
            "artifact_hold": hold,
            "library_name": expected_library_name if expected_library_name is not None else current_library,
            "context": context,
            "current_pose": current_pose,
            "map_snapshot": map_snapshot,
            "localization_snapshot": localization_snapshot,
            "reason": reason,
            "now": now,
            "force": force,
            "allow_memory_write": allow_memory_write,
        }
        installed = False
        try:
            with self._memory_write_lock:
                if self._stop_event.is_set():
                    return False
                if self._memory_write_in_flight:
                    # 已排队关键帧不被连续快速观察挤掉。
                    if continuous and self._memory_write_pending is not None and not self._memory_write_pending["continuous"]:
                        return False
                    old = self._memory_write_pending
                    self._memory_write_pending = job
                    installed = True
                    self._memory_write_scheduled_count += 1
                    self._release_job(old)
                    return True
                thread = threading.Thread(
                    target=self._memory_write_worker, args=(job,),
                    name="nuwax_perception_memory_write", daemon=True,
                )
                self._memory_write_in_flight = True
                self._memory_write_current = job
                self._memory_write_thread = thread
                try:
                    # 注册和启动共享队列锁，stop 不会错过尚未启动的工作。
                    thread.start()
                except BaseException:
                    self._memory_write_in_flight = False
                    self._memory_write_current = None
                    self._memory_write_thread = None
                    raise
                installed = True
                self._memory_write_scheduled_count += 1
            return True
        finally:
            if not installed:
                self._release_job(job)

    def _memory_write_worker(self, initial_job) -> None:
        job = initial_job
        worker_owned = True
        try:
            while job is not None:
                ok = False
                error = None
                try:
                    parameters = dict(job)
                    parameters.pop("artifact_hold", None)
                    continuous = parameters.pop("continuous", False)
                    expected_library = parameters.pop("library_name", None)
                    parameters["expected_library_name"] = expected_library
                    library_getter = getattr(self._memory_service, "get_active_library_name", None)
                    if self._stop_event.is_set():
                        error = "memory_write_cancelled"
                    elif expected_library is not None and callable(library_getter) and library_getter() != expected_library:
                        error = "memory_library_changed"
                    else:
                        if continuous:
                            for key in ("reason", "force", "allow_memory_write"):
                                parameters.pop(key)
                            ok = self._ingest_continuous_observation(**parameters)
                        else:
                            ok = self._remember_from_keyframe(**parameters)
                except Exception as exc:
                    error = str(exc)
                    PERCEPTION_RUNTIME_LOGGER.debug("记忆后台写入失败 camera=%s error=%s", self._camera_id, exc, exc_info=True)
                finally:
                    self._release_job(job)
                with self._lock:
                    if ok:
                        if job.get("continuous"):
                            self._continuous_ingest_count += 1
                        self._memory_write_completed_count += 1
                        self._last_memory_write_error = None
                    else:
                        self._memory_write_failed_count += 1
                        self._last_memory_write_error = error or "memory_write_skipped_or_failed"
                with self._memory_write_lock:
                    pending = self._memory_write_pending
                    self._memory_write_pending = None
                    job = None if self._stop_event.is_set() else pending
                    self._memory_write_current = job
                    if job is None:
                        self._release_job(pending)
                        self._memory_write_in_flight = False
                        worker_owned = False
                        return
        finally:
            if worker_owned:
                with self._memory_write_lock:
                    self._release_job(job)
                    self._release_job(self._memory_write_pending)
                    self._memory_write_pending = None
                    self._memory_write_current = None
                    self._memory_write_in_flight = False
            # 保留最后线程句柄；stop/status 可精确观察其最终退出。

    @staticmethod
    def _coerce_optional_float(value) -> Optional[float]:
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _latest_full_stage_duration_ms_total(self) -> Optional[float]:
        getter = getattr(self._perception_service, "get_latest_full_perception", None)
        if not callable(getter):
            return None
        try:
            context = getter(self._camera_id)
        except Exception:
            return None
        if context is None:
            return None
        stage_duration_ms = dict(getattr(context, "metadata", {}).get("stage_duration_ms") or {})
        return self._coerce_optional_float(stage_duration_ms.get("total"))

    def _latest_stream_frame(self, topic: str):
        stream_bus = getattr(self._perception_service, "_stream_bus", None)
        if stream_bus is None:
            return None
        latest_frame = getattr(stream_bus, "latest_frame", None)
        if not callable(latest_frame):
            return None
        try:
            return latest_frame(topic)
        except Exception:
            return None

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
                if self._stop_event.is_set():
                    break
                message = self.run_once(
                    force_keyframe=True,
                    reason=requested_reason,
                    allow_memory_write=remember_result,
                    detection_metadata=detection_metadata,
                    open_vocabulary_classes=open_vocabulary_classes,
                )
                if message:
                    messages.append(message)
                if effective_interval_sec > 0 and self._stop_event.wait(effective_interval_sec):
                    break
        finally:
            self._store_artifact = original_store_artifact
        return tuple(messages)

    def _run_loop(self) -> None:
        while not self._stop_event.is_set():
            frame_start = time.monotonic()
            try:
                self.run_once(force_keyframe=False, reason="background_keyframe")
                self._maybe_log_perf_summary()
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
        self._write_status(running=False)

    def _log_skip(self, *, reason: str, level: int, detail: str) -> None:
        """限流记录跳帧诊断：原因变化或超过间隔才输出，避免 0.5s 循环刷屏。"""

        now = time.monotonic()
        with self._lock:
            self._consecutive_skip_count += 1
            skip_count = self._consecutive_skip_count
            reason_changed = reason != self._last_skip_log_reason
            due = (now - self._last_skip_log_at) >= self._skip_log_interval_sec
            if not (reason_changed or due):
                return
            self._last_skip_log_at = now
            self._last_skip_log_reason = reason
        PERCEPTION_RUNTIME_LOGGER.log(
            level,
            "持续视频感知跳过 camera=%s reason=%s 连续跳过=%d 次：%s",
            self._camera_id,
            reason,
            skip_count,
            detail,
        )

    def _maybe_log_perf_summary(self) -> None:
        """节流（5 分钟一次）记录感知延迟摘要。vis-three-fixes 任务 6.4。

        数据已在 get_perception_runtime_status / video.detections 暴露，本日志仅作离线排查用，
        节流避免污染实机日志（呼应近期"降低实机日志噪声"提交）。
        """
        now = time.monotonic()
        if (now - self._last_perf_log_at) < self._perf_log_interval_sec:
            return
        self._last_perf_log_at = now
        with self._lock:
            meta = dict(self._last_status_metadata)
        PERCEPTION_RUNTIME_LOGGER.info(
            "感知延迟摘要 camera=%s detector_ms=%s fast_loop_ms=%s worker_ms=%s lock_wait_ms=%s interval_ms=%s nms_iou=%s",
            self._camera_id,
            meta.get("detector_duration_ms"),
            meta.get("fast_loop_duration_ms"),
            meta.get("worker_inference_ms"),
            meta.get("pipeline_lock_wait_ms"),
            meta.get("video_detections_interval_ms"),
            meta.get("nms_iou_threshold"),
        )

    def _resolve_runtime_context(self):
        """返回 (current_pose, map_snapshot, localization_snapshot)。

        生产网关使用定位订阅的实时快照；关键帧判定不另行查询远程坐标树。
        未注入定位服务的独立调用者才使用其显式提供的位姿源。
        """
        current_pose = None
        localization_snapshot = None
        localization_service = self._localization_service
        if localization_service is not None:
            localization_snapshot = self._refresh_service_snapshot(localization_service)
            if localization_snapshot is not None:
                current_pose = localization_snapshot.current_pose
        else:
            current_pose = self._read_live_pose()
        map_snapshot = None
        mapping_service = self._mapping_service
        if mapping_service is not None:
            map_snapshot = self._refresh_service_snapshot(mapping_service)
        return current_pose, map_snapshot, localization_snapshot

    def _read_live_pose(self) -> Optional[Pose]:
        """从实时位姿源（几何运行时 provider 门面）取当前位姿，鸭子类型，失败返回 None。

        优先 get_exploration_pose()（探索建图时导航后端 map→base 实时 TF），
        回退 get_current_pose()（定位会话后端，仅在重定位就绪后有值）。
        """
        provider = self._live_pose_provider
        if provider is None:
            return None
        for getter_name in ("get_exploration_pose", "get_current_pose"):
            getter = getattr(provider, getter_name, None)
            if not callable(getter):
                continue
            try:
                pose = getter()
            except Exception:
                PERCEPTION_RUNTIME_LOGGER.debug(
                    "读取实时位姿失败 source=%s。", getter_name, exc_info=True
                )
                continue
            if isinstance(pose, Pose):
                return pose
        return None

    def _refresh_service_snapshot(self, service):
        """消费定位/建图拥有方已发布的快照，识别周期不发起同步几何刷新。"""
        getter = getattr(service, "get_latest_snapshot", None)
        if callable(getter):
            return getter()
        return None

    def _remember_from_keyframe(
        self,
        *,
        context,
        current_pose: Optional[Pose],
        reason: str,
        now: datetime,
        force: bool,
        allow_memory_write: bool,
        map_snapshot=None,
        localization_snapshot=None,
        expected_library_name=None,
    ) -> bool:
        if not allow_memory_write:
            return False
        if self._memory_service is None:
            return False
        if not self._remember_keyframes and not force:
            return False
        if hasattr(self._memory_service, "is_enabled") and not self._memory_service.is_enabled():
            return False
        try:
            self._memory_service.remember_current_scene(
                perception_context=context,
                observation_pose=current_pose,
                map_snapshot=map_snapshot,
                localization_snapshot=localization_snapshot,
                expected_library_name=expected_library_name,
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
