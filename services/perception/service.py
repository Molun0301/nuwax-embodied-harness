from __future__ import annotations

import json
import logging
import threading
from collections import deque
from contextlib import contextmanager, nullcontext
from datetime import timezone
from time import perf_counter
import time
from typing import (
    TYPE_CHECKING,
    Any,
    Callable,
    Deque,
    Dict,
    Iterable,
    List,
    Optional,
    Protocol,
    Tuple,
    runtime_checkable,
)
from urllib.parse import urlencode

from contracts.events import RuntimeEventCategory
from contracts.naming import build_observation_id
from contracts.perception import Detection2D, Observation, SegmentationMask2D, Track, TrackState
from contracts.runtime_views import PerceptionContext, SceneObjectSummary, SceneSummary
from contracts.streams import StreamFrame
from core import EventBus, StateNamespace, StateStore, StreamBus
from core.errors import GatewayError
from core.logging_utils import LogRateLimiter
from core.event_bus import PreparedEventPublication
from core.stream_bus import PreparedStreamPublication
from providers import CalibrationProvider, PointCloudProvider
from providers.pointcloud import DemandPointCloudProvider
from services.artifact_service import ArtifactService
from services.perception.base import (
    DetectionBundle,
    DisabledSegmentationBackend,
    SceneDescriptionBackend,
    SegmentationBackend,
    SegmentationBundle,
    TrackerBackend,
)
from services.perception.detectors import DetectorPipeline
from services.perception.geometry import DetectionGeometryAugmentor
from services.perception.observation_geometry import capture_geometry, contract_pose, _stamp
from services.perception.scene import SimpleSceneDescriptionBackend
from services.perception.self_body import (
    DisabledSelfBodyFilter,
    SelfBodyFilter,
    region_from_detection_metadata,
)
from services.perception.trackers import Basic2DTrackerBackend

LOGGER = logging.getLogger("nuwax_robot_bridge.perception.service")
_INPUT_LOG_LIMITER = LogRateLimiter()
_POINT_CLOUD_CACHE_FRAMES = 10

if TYPE_CHECKING:
    from contracts.image import ImageFrame

VideoFrameSink = Callable[[object], Optional[object]]


@runtime_checkable
class VideoFrameSource(Protocol):
    """平台已接收的标准图像源，不执行硬件抓拍。"""

    def get_latest_image(self, camera_id: Optional[str] = None) -> Optional["ImageFrame"]:
        """返回指定相机的只读最近帧；未收到或已停止时返回空。"""

    def status_snapshot(self) -> Dict[str, Any]:
        """返回接收状态，available 表示已能读取图像。"""


class PerceptionService:
    """统一感知、跟踪与场景语义摘要服务。"""

    def __init__(
        self,
        *,
        provider_owner,
        artifact_service: ArtifactService,
        state_store: StateStore,
        detector_pipeline: DetectorPipeline,
        tracker_backend: Optional[TrackerBackend] = None,
        scene_description_backend: Optional[SceneDescriptionBackend] = None,
        segmentation_backend: Optional[SegmentationBackend] = None,
        geometry_augmentor: Optional[DetectionGeometryAugmentor] = None,
        self_body_filter: Optional[SelfBodyFilter] = None,
        event_bus: Optional[EventBus] = None,
        stream_bus: Optional[StreamBus] = None,
        video_frame_sink: Optional[VideoFrameSink] = None,
        video_frame_source: Optional[VideoFrameSource] = None,
        geometry_state=None,
        geometry_client=None,
        history_limit: int = 100,
        pipeline_name: str = "default_perception_pipeline",
    ) -> None:
        self._provider_owner = provider_owner
        self._artifact_service = artifact_service
        self._state_store = state_store
        self._detector_pipeline = detector_pipeline
        self._tracker_backend = tracker_backend or Basic2DTrackerBackend()
        self._scene_description_backend = scene_description_backend or SimpleSceneDescriptionBackend()
        self._segmentation_backend = segmentation_backend or DisabledSegmentationBackend()
        self._geometry_augmentor = geometry_augmentor or DetectionGeometryAugmentor()
        self._self_body_filter = self_body_filter or DisabledSelfBodyFilter("not_configured")
        self._event_bus = event_bus
        self._stream_bus = stream_bus
        self._video_frame_sink = video_frame_sink
        self._video_frame_source = video_frame_source
        self._geometry_state = geometry_state
        self._geometry_client = geometry_client
        self._frame_tree = None
        self._history: Deque[PerceptionContext] = deque(maxlen=max(1, history_limit))
        self._fast_frame_cache = {}
        self._full_inputs = {}
        self._last_published_video_frames = {}
        self._latest_by_camera: Dict[str, PerceptionContext] = {}
        self._latest_full_by_camera: Dict[str, PerceptionContext] = {}
        self._pipeline_name = pipeline_name
        self._last_detection_overlay_publish_at_by_camera: Dict[str, float] = {}
        # 检测/跟踪共享短管线锁，完整分析另行串行；慢分割和几何不占快速检测锁。
        self._pipeline_lock = threading.Lock()
        self._full_pipeline_lock = threading.Lock()
        self._latest_frame_timestamp_by_camera = {}
        self._context_condition = threading.Condition(threading.RLock())
        self._active_operations = 0
        self._closing = False
        self._closed = False
        self._point_cloud_lock = threading.Lock()
        self._point_cloud_frames = ()
        self._point_cloud_subscription = (stream_bus.subscribe(self._cache_point_cloud, topics=("pointcloud.frame",))
                                          if stream_bus is not None else None)

    def set_video_frame_sink(self, video_frame_sink: Optional[VideoFrameSink]) -> None:
        self._video_frame_sink = video_frame_sink

    def set_video_frame_source(self, video_frame_source: Optional[VideoFrameSource]) -> None:
        self._video_frame_source = video_frame_source

    def update_detector_thresholds(self, config) -> None:
        """等待正在执行的检测完成后更新已注册后端，不创建第二个模型。"""
        with self._pipeline_lock:
            for spec in self._detector_pipeline.list_backends():
                backend = self._detector_pipeline.get_backend(spec.name)
                update = getattr(backend, "update_thresholds", None)
                if callable(update):
                    update(confidence_threshold=config.confidence_threshold,
                           iou_threshold=config.iou_threshold, max_detections=config.max_detections)

    @property
    def detector_pipeline(self) -> DetectorPipeline:
        """返回检测管线。"""

        return self._detector_pipeline

    @property
    def tracker_backend(self) -> TrackerBackend:
        """返回跟踪后端。"""

        return self._tracker_backend

    @property
    def segmentation_backend(self) -> SegmentationBackend:
        """返回分割后端。"""

        return self._segmentation_backend

    def process_image(
        self,
        image_frame,
        camera_info=None,
        *,
        image_artifact=None,
        requested_by: Optional[str] = None,
        detector_backend_name: Optional[str] = None,
        source_name: Optional[str] = None,
        detection_metadata: Optional[Dict[str, object]] = None,
        open_vocabulary_classes: Optional[Iterable[str]] = None,
    ) -> PerceptionContext:
        """消费标准图像并产出检测、跟踪和场景摘要。

        检测和跟踪先取得同帧结果，随后独立执行分割、几何与摘要。
        慢阶段运行时允许后续快速检测继续更新。
        """

        from contracts.artifacts import ArtifactRef

        if image_artifact is not None and not isinstance(image_artifact, ArtifactRef):
            raise TypeError("PerceptionService.process_image 的 image_artifact 必须是 ArtifactRef。")
        with self._operation(), self.point_cloud_consumer():
            return self._run_full_pipeline(
                image_frame, camera_info, image_artifact=image_artifact,
                requested_by=requested_by, detector_backend_name=detector_backend_name,
                source_name=source_name, detection_metadata=detection_metadata,
                open_vocabulary_classes=open_vocabulary_classes,
            )

    def _run_full_pipeline(self, image_frame, camera_info=None, **kwargs):
        requested_at = perf_counter()
        cached_geometry_inputs = self._capture_cached_geometry_inputs(image_frame) if camera_info is not None else None
        with self._full_pipeline_lock:
            with self._context_condition:
                self._ensure_open()
            acquired_at = perf_counter()
            context, scene_publication = self._process_image_locked(
                image_frame, camera_info, cached_geometry_inputs=cached_geometry_inputs,
                lock_wait_ms=(acquired_at - requested_at) * 1000.0, **kwargs,
            )
        # 场景事件在完整分析锁内预留顺序；同步订阅不延长模型串行区。
        if scene_publication is not None:
            try:
                self._event_bus.dispatch_prepared(scene_publication)
            except BaseException:
                self._event_bus.abandon_prepared(scene_publication)
                raise
        # 完整结果也能在场景通知期间更新；身份检查与游标预留必须相对槽位替换原子化。
        overlay_publication = None
        with self._pipeline_lock, self._context_condition:
            if self._latest_by_camera.get(context.camera_id) is context:
                overlay_publication = self._prepare_detection_overlay_frame(context)
        # 游标在检测顺序锁内预留；此处在两个管线锁外完成同步分发。
        # 调用方的感知操作在分发结束后退出，关闭会等待该操作完成。
        if overlay_publication is not None:
            self._stream_bus.dispatch_prepared(overlay_publication)
        return context

    def get_full_observation_input(self, context):
        """返回这一完整观察的原图和同帧几何，供显式记忆动作使用。"""
        with self._context_condition:
            value = self._full_inputs.get(context.camera_id)
            return value[1:] if value is not None and value[0] == context.observation.observation_id else None

    def _process_image_locked(
        self,
        image_frame,
        camera_info=None,
        *,
        cached_geometry_inputs,
        image_artifact=None,
        requested_by: Optional[str] = None,
        detector_backend_name: Optional[str] = None,
        source_name: Optional[str] = None,
        detection_metadata: Optional[Dict[str, object]] = None,
        open_vocabulary_classes: Optional[Iterable[str]] = None,
        lock_wait_ms: Optional[float] = None,
    ) -> Tuple[PerceptionContext, Optional[PreparedEventPublication]]:
        """完整分析的实现；调用方持有完整分析锁，检测/跟踪独立加锁。"""

        from contracts.artifacts import ArtifactRef
        from contracts.image import CameraInfo, ImageFrame

        if not isinstance(image_frame, ImageFrame):
            raise TypeError("PerceptionService.process_image 仅接受标准 ImageFrame。")
        if camera_info is not None and not isinstance(camera_info, CameraInfo):
            raise TypeError("PerceptionService.process_image 的 camera_info 必须是 CameraInfo。")
        if image_artifact is not None and not isinstance(image_artifact, ArtifactRef):
            raise TypeError("PerceptionService.process_image 的 image_artifact 必须是 ArtifactRef。")

        image_frame = self._apply_detection_metadata(
            image_frame,
            detection_metadata=detection_metadata,
            open_vocabulary_classes=open_vocabulary_classes,
        )
        source_timestamp_metadata = self._source_timestamp_metadata(image_frame)

        pipeline_started_at = perf_counter()
        stage_duration_ms: Dict[str, float] = {}
        stage_status: Dict[str, str] = {}

        detector_lock_requested_at = perf_counter()
        with self._pipeline_lock:
            with self._context_condition:
                self._ensure_open()
            lock_wait_ms = (lock_wait_ms or 0.0) + self._elapsed_ms(detector_lock_requested_at)
            stage_started_at = perf_counter()
            detection_bundle, detector_spec = self._detector_pipeline.detect(
                image_frame,
                camera_info,
                backend_name=detector_backend_name,
            )
            stage_duration_ms["detector"] = self._elapsed_ms(stage_started_at)
            stage_status["detector"] = self._provider_stage_status(
                detector_spec.backend_kind,
                detection_bundle.metadata,
            )
            stage_started_at = perf_counter()
            detection_bundle = self._apply_self_body_filter(
                detection_bundle, image_frame=image_frame, camera_info=camera_info,
                frame_tree=cached_geometry_inputs[1] if cached_geometry_inputs is not None else None,
            )
            stage_duration_ms["self_body"] = self._elapsed_ms(stage_started_at)

            stage_started_at = perf_counter()
            detector_available = stage_status["detector"] in {"completed", "fallback"}
            tracks = self._tracker_backend.update(
                image_frame,
                detections_2d=detection_bundle.detections_2d,
                detections_3d=detection_bundle.detections_3d,
                camera_info=camera_info,
            ) if detector_available else ()
            stage_duration_ms["tracker"] = self._elapsed_ms(stage_started_at)
            stage_status["tracker"] = "completed" if detector_available else "skipped_detector_unavailable"

        stage_started_at = perf_counter()
        segmentation_bundle = self._segmentation_backend.segment(
            image_frame,
            detections_2d=detection_bundle.detections_2d,
            camera_info=camera_info,
        )
        stage_duration_ms["segmentation"] = self._elapsed_ms(stage_started_at)
        segmentation_stage_status = self._provider_stage_status(
            self._segmentation_backend.spec.backend_kind,
            segmentation_bundle.metadata,
        )
        stage_status["segmentation"] = segmentation_stage_status

        detection_bundle = self._attach_segmentation_results(
            detection_bundle,
            segmentation_bundle,
        )

        stage_started_at = perf_counter()
        detection_bundle = self._augment_detection_geometry(
            detection_bundle,
            image_frame=image_frame,
            camera_info=camera_info,
            cached_geometry_inputs=cached_geometry_inputs,
        )
        stage_duration_ms["geometry"] = self._elapsed_ms(stage_started_at)
        stage_status["geometry"] = str(
            detection_bundle.metadata.get("geometry_stage_status")
            or detection_bundle.metadata.get("geometry_status")
            or "completed"
        )

        detections_2d = self._attach_track_ids(detection_bundle.detections_2d, tracks)

        stage_started_at = perf_counter()
        scene_summary = self._scene_description_backend.describe(
            camera_id=image_frame.camera_id,
            detections_2d=detections_2d,
            detections_3d=detection_bundle.detections_3d,
            tracks=tracks,
            image_frame=image_frame,
            camera_info=camera_info,
        )
        stage_duration_ms["scene_description"] = self._elapsed_ms(stage_started_at)
        stage_status["scene_description"] = "completed"
        stage_duration_ms["total"] = self._elapsed_ms(pipeline_started_at)
        provider_fallback_used = segmentation_stage_status == "fallback"
        provider_fallback_reason = self._provider_fallback_reason(
            segmentation_bundle.metadata,
            self._segmentation_backend.spec.metadata,
        )
        detector_backend_metadata = dict(detector_spec.metadata)
        detector_provider = dict(
            detector_backend_metadata.get("provider") or detection_bundle.metadata.get("detector_provider") or {}
        )
        observation = Observation(
            observation_id=build_observation_id(self._normalize_camera_id(image_frame.camera_id)),
            frame_id=image_frame.frame_id,
            summary=scene_summary.headline,
            detections_2d=list(detections_2d),
            detections_3d=list(detection_bundle.detections_3d),
            tracks=list(tracks),
            artifact_ids=[image_artifact.artifact_id] if image_artifact is not None else [],
            metadata={
                "camera_id": image_frame.camera_id,
                "detector_backend": detector_spec.name,
                "detector_provider": dict(detector_provider),
                "detector_backend_metadata": dict(detector_backend_metadata),
                "detection_metadata": dict(detection_bundle.metadata),
                "tracker_backend": self._tracker_backend.spec.name,
                "segmentation_backend": self._segmentation_backend.spec.name,
                "segmentation_backend_kind": self._segmentation_backend.spec.backend_kind,
                "segmentation_backend_metadata": dict(self._segmentation_backend.spec.metadata),
                "segmentation_metadata": dict(segmentation_bundle.metadata),
                "segmentation_count": len(segmentation_bundle.segments),
                "scene_description_backend": self._scene_description_backend.spec.name,
                "stage_duration_ms": dict(stage_duration_ms),
                "stage_status": dict(stage_status),
                "provider_fallback_used": provider_fallback_used,
                "provider_fallback_reason": provider_fallback_reason,
                "pipeline_name": self._pipeline_name,
                "requested_by": requested_by or "unknown",
                "source_name": source_name or "unknown",
                "image_encoding": image_frame.encoding.value,
                "width_px": image_frame.width_px,
                "height_px": image_frame.height_px,
                "detection_count": len(detections_2d),
                "track_count": len(tracks),
                "pipeline_lock_wait_ms": (
                    round(lock_wait_ms, 3) if lock_wait_ms is not None else None
                ),
                **dict(detection_bundle.metadata),
                **source_timestamp_metadata,
            },
        )
        context = PerceptionContext(
            camera_id=image_frame.camera_id,
            observation=observation,
            scene_summary=scene_summary,
            image_artifact=image_artifact,
            camera_info=camera_info,
            pipeline_name=self._pipeline_name,
            detector_backend=detector_spec.name,
            tracker_backend=self._tracker_backend.spec.name,
            metadata={
                "source_name": source_name or "unknown",
                "requested_by": requested_by or "unknown",
                "scene_description_backend": self._scene_description_backend.spec.name,
                "segmentation_backend": self._segmentation_backend.spec.name,
                "detector_provider": dict(detector_provider),
                "detector_backend_metadata": dict(detector_backend_metadata),
                "detection_metadata": dict(detection_bundle.metadata),
                "detector_backend_kind": detector_spec.backend_kind,
                "tracker_backend_kind": self._tracker_backend.spec.backend_kind,
                "segmentation_backend_kind": self._segmentation_backend.spec.backend_kind,
                "segmentation_backend_metadata": dict(self._segmentation_backend.spec.metadata),
                "segmentation_metadata": dict(segmentation_bundle.metadata),
                "segmentation_count": len(segmentation_bundle.segments),
                "detection_count": len(detections_2d),
                "track_count": len(tracks),
                "stage_duration_ms": dict(stage_duration_ms),
                "stage_status": dict(stage_status),
                "provider_fallback_used": provider_fallback_used,
                "provider_fallback_reason": provider_fallback_reason,
                "pipeline_lock_wait_ms": (
                    round(lock_wait_ms, 3) if lock_wait_ms is not None else None
                ),
                **source_timestamp_metadata,
            },
        )
        self._publish_context(
            context, image_frame=image_frame, source=source_name or detector_spec.name,
            kind="latest_perception", full=True, geometry_inputs=cached_geometry_inputs,
        )
        scene_publication = None
        if self._event_bus is not None:
            scene_publication = self._event_bus.prepare(
                self._event_bus.build_event(
                    "perception.scene_ready",
                    category=RuntimeEventCategory.PERCEPTION,
                    source="perception_service",
                    subject_id=observation.observation_id,
                    message=scene_summary.headline,
                    payload={
                        "camera_id": context.camera_id,
                        "detector_backend": detector_spec.name,
                        "detector_provider": dict(detector_provider),
                        "segmentation_backend": self._segmentation_backend.spec.name,
                        "segmentation_backend_kind": self._segmentation_backend.spec.backend_kind,
                        "scene_description_backend": self._scene_description_backend.spec.name,
                        "detection_count": len(detections_2d),
                        "segmentation_count": len(segmentation_bundle.segments),
                        "track_count": len(tracks),
                        "stage_duration_ms": dict(stage_duration_ms),
                        "stage_status": dict(stage_status),
                        "provider_fallback_used": provider_fallback_used,
                        "provider_fallback_reason": provider_fallback_reason,
                    },
                    metadata={
                        "camera_id": context.camera_id,
                        "detector_backend": detector_spec.name,
                        "detector_provider": dict(detector_provider),
                        "segmentation_backend": self._segmentation_backend.spec.name,
                        "scene_description_backend": self._scene_description_backend.spec.name,
                        "duration_ms": stage_duration_ms.get("total"),
                    },
                )
            )
        return context, scene_publication

    def perceive_current_scene(
        self,
        *,
        camera_id: Optional[str] = None,
        requested_by: Optional[str] = None,
        detector_backend_name: Optional[str] = None,
        store_artifact: bool = True,
        detection_metadata: Optional[Dict[str, object]] = None,
        open_vocabulary_classes: Optional[Iterable[str]] = None,
    ) -> PerceptionContext:
        """从平台已接收的端侧推流抽取最新帧并完成完整感知。"""

        with self._operation(), self.point_cloud_consumer():
            return self._perceive_current_scene(
                camera_id=camera_id, requested_by=requested_by,
                detector_backend_name=detector_backend_name, store_artifact=store_artifact,
                detection_metadata=detection_metadata, open_vocabulary_classes=open_vocabulary_classes,
            )

    def _perceive_current_scene(self, *, camera_id, requested_by, detector_backend_name,
                                store_artifact, detection_metadata, open_vocabulary_classes):
        image_frame = self._read_latest_pushed_image(camera_id)
        if image_frame is None:
            ingest_status = self._video_frame_source_status()
            required_source = "video_ingest_receiver"
            raise GatewayError(
                "端侧视频推流尚未接入或尚未收到首帧，无法执行实时感知；平台不会主动采集图像帧。",
                error_code="video_ingest_not_ready",
                http_status=503,
                details={
                    "camera_id": camera_id,
                    "required_source": required_source,
                    "platform_role": "video_ingest_receiver",
                    "ingest_status": ingest_status,
                },
            )
        camera_info = self._camera_info_from_video_frame(image_frame)
        source_name = str((image_frame.metadata or {}).get("source_kind") or "video_ingest_receiver")
        self._publish_video_frame(
            image_frame,
            source_name=source_name,
        )
        image_artifact = None
        if store_artifact:
            image_artifact = self._artifact_service.save_image_frame(
                image_frame,
                metadata={"requested_by": requested_by or "unknown", "service": "perception_service"},
            )
        return self._run_full_pipeline(
            image_frame,
            camera_info,
            image_artifact=image_artifact,
            requested_by=requested_by,
            detector_backend_name=detector_backend_name,
            source_name=source_name,
            detection_metadata=detection_metadata,
            open_vocabulary_classes=open_vocabulary_classes,
        )

    def detect_current_frame_fast(
        self,
        *,
        camera_id: Optional[str] = None,
        requested_by: Optional[str] = None,
        detector_backend_name: Optional[str] = None,
        detection_metadata: Optional[Dict[str, object]] = None,
        open_vocabulary_classes: Optional[Iterable[str]] = None,
    ) -> PerceptionContext:
        """执行高频检测观测链路。

        该路径只做视频抽帧、detector、2D tracker、track_id 附加和
        `video.detections` 发布；不执行 segmentation / geometry /
        scene_description / keyframe memory，从而让 Web 检测框和连续
        三角测量观测跟随 YOLO 节奏。
        """

        with self._operation():
            return self._detect_current_frame_fast(
                camera_id=camera_id, requested_by=requested_by,
                detector_backend_name=detector_backend_name, detection_metadata=detection_metadata,
                open_vocabulary_classes=open_vocabulary_classes,
            )

    def _detect_current_frame_fast(self, *, camera_id, requested_by, detector_backend_name,
                                   detection_metadata, open_vocabulary_classes):
        image_frame = self._read_latest_pushed_image(camera_id, wait_for_frame=requested_by != "perception_video_runtime")
        if image_frame is None:
            ingest_status = self._video_frame_source_status()
            required_source = "video_ingest_receiver"
            raise GatewayError(
                "端侧视频推流尚未接入或尚未收到首帧，无法执行实时检测；平台不会主动采集图像帧。",
                error_code="video_ingest_not_ready",
                http_status=503,
                details={
                    "camera_id": camera_id,
                    "required_source": required_source,
                    "platform_role": "video_ingest_receiver",
                    "ingest_status": ingest_status,
                },
            )
        camera_info = self._camera_info_from_video_frame(image_frame)
        source_name = str((image_frame.metadata or {}).get("source_kind") or "video_ingest_receiver")
        self._publish_video_frame(
            image_frame,
            source_name=source_name,
        )
        return self.process_image_fast(
            image_frame,
            camera_info,
            requested_by=requested_by,
            detector_backend_name=detector_backend_name,
            source_name=source_name,
            detection_metadata=detection_metadata,
            open_vocabulary_classes=open_vocabulary_classes,
        )

    def process_image_fast(
        self,
        image_frame,
        camera_info=None,
        *,
        requested_by: Optional[str] = None,
        detector_backend_name: Optional[str] = None,
        source_name: Optional[str] = None,
        detection_metadata: Optional[Dict[str, object]] = None,
        open_vocabulary_classes: Optional[Iterable[str]] = None,
    ) -> PerceptionContext:
        """消费标准图像并产出 detector/tracker 级快速观测结果。"""

        with self._operation():
            # vis-three-fixes AC2.4a：同上 process_image，记录 pipeline_lock_wait_ms。
            requested_at = perf_counter()
            with self._pipeline_lock:
                with self._context_condition:
                    self._ensure_open()
                acquired_at = perf_counter()
                context, overlay_publication = self._process_image_fast_locked(
                    image_frame,
                    camera_info,
                    requested_by=requested_by,
                    detector_backend_name=detector_backend_name,
                    source_name=source_name,
                    detection_metadata=detection_metadata,
                    open_vocabulary_classes=open_vocabulary_classes,
                    lock_wait_ms=(acquired_at - requested_at) * 1000.0,
                )
            if overlay_publication is not None:
                self._stream_bus.dispatch_prepared(overlay_publication)
            return context

    def _process_image_fast_locked(
        self,
        image_frame,
        camera_info=None,
        *,
        requested_by: Optional[str] = None,
        detector_backend_name: Optional[str] = None,
        source_name: Optional[str] = None,
        detection_metadata: Optional[Dict[str, object]] = None,
        open_vocabulary_classes: Optional[Iterable[str]] = None,
        lock_wait_ms: Optional[float] = None,
    ) -> Tuple[PerceptionContext, Optional[PreparedStreamPublication]]:
        """process_image_fast 的实际实现，调用方须已持有 _pipeline_lock。"""

        from contracts.image import CameraInfo, ImageFrame

        if not isinstance(image_frame, ImageFrame):
            raise TypeError("PerceptionService.process_image_fast 仅接受标准 ImageFrame。")
        if camera_info is not None and not isinstance(camera_info, CameraInfo):
            raise TypeError("PerceptionService.process_image_fast 的 camera_info 必须是 CameraInfo。")

        source_frame = image_frame
        cacheable = not detection_metadata and open_vocabulary_classes is None
        cache_key = (detector_backend_name or self._detector_pipeline.default_backend_name, source_name, requested_by)
        input_signature = self._fast_input_signature(image_frame, camera_info) if cacheable else None
        cacheable = cacheable and input_signature is not None
        cached = self._fast_frame_cache.get(image_frame.camera_id)
        if (
            cacheable and cached is not None and cached[0] is source_frame
            and cached[1] == input_signature and cached[2] == cache_key
        ):
            return cached[3], None

        image_frame = self._apply_detection_metadata(
            image_frame,
            detection_metadata=detection_metadata,
            open_vocabulary_classes=open_vocabulary_classes,
        )
        source_timestamp_metadata = self._source_timestamp_metadata(image_frame)
        pipeline_started_at = perf_counter()
        stage_duration_ms: Dict[str, float] = {}
        stage_status: Dict[str, str] = {
            "segmentation": "skipped_fast_path",
            "geometry": "skipped_fast_path",
            "scene_description": "skipped_fast_path",
        }

        stage_started_at = perf_counter()
        detection_bundle, detector_spec = self._detector_pipeline.detect(
            image_frame,
            camera_info,
            backend_name=detector_backend_name,
        )
        stage_duration_ms["detector"] = self._elapsed_ms(stage_started_at)
        stage_status["detector"] = self._provider_stage_status(
            detector_spec.backend_kind,
            detection_bundle.metadata,
        )
        stage_started_at = perf_counter()
        detection_bundle = self._apply_self_body_filter(
            detection_bundle, image_frame=image_frame, camera_info=camera_info, frame_tree=None,
        )
        stage_duration_ms["self_body"] = self._elapsed_ms(stage_started_at)

        stage_started_at = perf_counter()
        detector_available = stage_status["detector"] in {"completed", "fallback"}
        tracks = self._tracker_backend.update(
            image_frame,
            detections_2d=detection_bundle.detections_2d,
            detections_3d=detection_bundle.detections_3d,
            camera_info=camera_info,
        ) if detector_available else ()
        stage_duration_ms["tracker"] = self._elapsed_ms(stage_started_at)
        stage_status["tracker"] = "completed" if detector_available else "skipped_detector_unavailable"

        detections_2d = self._attach_track_ids(detection_bundle.detections_2d, tracks)
        stage_duration_ms["total"] = self._elapsed_ms(pipeline_started_at)
        detector_backend_metadata = dict(detector_spec.metadata)
        detector_provider = dict(
            detector_backend_metadata.get("provider") or detection_bundle.metadata.get("detector_provider") or {}
        )
        scene_summary = self._build_fast_scene_summary(
            camera_id=image_frame.camera_id,
            detections_2d=detections_2d,
            tracks=tracks,
            stage_duration_ms=stage_duration_ms,
        )
        if not detector_available:
            scene_summary = scene_summary.model_copy(update={
                "headline": "检测模型尚未完成有效推理，当前帧识别结果不可用。",
            })
        observation = Observation(
            observation_id=build_observation_id(self._normalize_camera_id(image_frame.camera_id)),
            frame_id=image_frame.frame_id,
            summary=scene_summary.headline,
            detections_2d=list(detections_2d),
            detections_3d=list(detection_bundle.detections_3d),
            tracks=list(tracks),
            metadata={
                "camera_id": image_frame.camera_id,
                "detector_backend": detector_spec.name,
                "detector_provider": dict(detector_provider),
                "detector_backend_metadata": dict(detector_backend_metadata),
                "detection_metadata": dict(detection_bundle.metadata),
                "tracker_backend": self._tracker_backend.spec.name,
                "segmentation_backend": self._segmentation_backend.spec.name,
                "segmentation_backend_kind": self._segmentation_backend.spec.backend_kind,
                "segmentation_count": 0,
                "scene_description_backend": "fast_detection_summary",
                "stage_duration_ms": dict(stage_duration_ms),
                "stage_status": dict(stage_status),
                "pipeline_name": self._pipeline_name,
                "requested_by": requested_by or "unknown",
                "source_name": source_name or "unknown",
                "image_encoding": image_frame.encoding.value,
                "width_px": image_frame.width_px,
                "height_px": image_frame.height_px,
                "detection_count": len(detections_2d),
                "track_count": len(tracks),
                "overlay_phase": "detector_ready",
                "fast_path": True,
                "pipeline_lock_wait_ms": (
                    round(lock_wait_ms, 3) if lock_wait_ms is not None else None
                ),
                **dict(detection_bundle.metadata),
                **source_timestamp_metadata,
            },
        )
        context = PerceptionContext(
            camera_id=image_frame.camera_id,
            observation=observation,
            scene_summary=scene_summary,
            image_artifact=None,
            camera_info=camera_info,
            pipeline_name=self._pipeline_name,
            detector_backend=detector_spec.name,
            tracker_backend=self._tracker_backend.spec.name,
            metadata={
                "source_name": source_name or "unknown",
                "requested_by": requested_by or "unknown",
                "scene_description_backend": "fast_detection_summary",
                "segmentation_backend": self._segmentation_backend.spec.name,
                "detector_provider": dict(detector_provider),
                "detector_backend_metadata": dict(detector_backend_metadata),
                "detection_metadata": dict(detection_bundle.metadata),
                "detector_backend_kind": detector_spec.backend_kind,
                "tracker_backend_kind": self._tracker_backend.spec.backend_kind,
                "segmentation_backend_kind": self._segmentation_backend.spec.backend_kind,
                "segmentation_count": 0,
                "detection_count": len(detections_2d),
                "track_count": len(tracks),
                "stage_duration_ms": dict(stage_duration_ms),
                "stage_status": dict(stage_status),
                "overlay_phase": "detector_ready",
                "fast_path": True,
                "pipeline_lock_wait_ms": (
                    round(lock_wait_ms, 3) if lock_wait_ms is not None else None
                ),
                **source_timestamp_metadata,
            },
        )
        self._publish_context(
            context, image_frame=image_frame, source=source_name or detector_spec.name,
            kind="latest_perception_fast", full=False,
        )
        if stage_status["detector"] in {"completed", "fallback"}:
            self._fast_frame_cache[context.camera_id] = (
                source_frame, input_signature if cacheable else None, cache_key, context,
                image_frame.model_copy(deep=True), camera_info.model_copy(deep=True) if camera_info is not None else None)
            if len(self._fast_frame_cache) > 8:
                self._fast_frame_cache.pop(next(iter(self._fast_frame_cache)))
        return context, self._prepare_detection_overlay_frame(context)

    def get_fast_observation_input(self, context):
        """将同一检测输入交给已有记忆任务，后续相机帧不会替换它。"""
        with self._pipeline_lock:
            cached = self._fast_frame_cache.get(context.camera_id)
            if cached is not None and cached[3].observation.observation_id == context.observation.observation_id:
                return cached[4], cached[5]
        return None

    def complete_observation_geometry(self, context, inputs, *, store_artifact=False,
                                      stop_event=None):
        """完成同帧几何与关键帧抓图，返回结果后结束感知操作。"""
        with self._operation(), self.point_cloud_consumer():
            return self._complete_observation_geometry(
                context, inputs, store_artifact=store_artifact,
                stop_event=stop_event)

    def _complete_observation_geometry(self, context, inputs, *, store_artifact=False,
                                      stop_event=None):
        """后台补几何与关键帧照片，复用本次检测结果，不再次抓拍或推理。"""
        if inputs is None:
            return context, None, None
        image_frame, camera_info = inputs
        geometry = self._capture_geometry(image_frame)
        if stop_event is not None and stop_event.is_set():
            return context, None, None
        if camera_info is None:
            detections = tuple(context.observation.detections_2d)
            metadata = {"geometry_projection_degradation_reason": "camera_info_missing"}
        else:
            detections, metadata = self._geometry_augmentor.augment_detections(
                tuple(context.observation.detections_2d), image_frame=image_frame, camera_info=camera_info,
                point_cloud=geometry.point_cloud, frame_tree=geometry.frame_tree, current_pose=geometry.pose,
                self_region=region_from_detection_metadata(context.metadata.get("detection_metadata")))
        metadata = {**metadata, "geometry_synchronization": "source_time_window_interpolation",
                    "geometry_input_sources": list(geometry.sources)}
        if geometry.reason:
            metadata["geometry_projection_degradation_reason"] = geometry.reason
            metadata["geometry_projection_degradation_message"] = geometry.message
            detections = tuple(item.model_copy(update={"attributes": {
                **item.attributes, "geometry_projection_degradation_reason": geometry.reason,
                "geometry_projection_degradation_message": geometry.message}})
                for item in detections)
        artifact = context.image_artifact
        if store_artifact and artifact is None:
            artifact = self._artifact_service.save_image_frame(image_frame, metadata={
                "requested_by": "perception_video_runtime", "observation_id": context.observation.observation_id})
        def enriched(original):
            return {**original, "detection_metadata": {**original.get("detection_metadata", {}), **metadata},
                    **metadata, "geometry_stage_status": "completed"}
        observation = context.observation.model_copy(update={
            "detections_2d": list(detections), "metadata": enriched(context.observation.metadata),
            "artifact_ids": [artifact.artifact_id] if artifact is not None else context.observation.artifact_ids})
        return context.model_copy(update={"observation": observation, "image_artifact": artifact,
                                         "metadata": {**enriched(context.metadata), "stage_status": {
                                             **context.metadata.get("stage_status", {}), "geometry": "completed"}}}), geometry.pose, geometry.frame_tree

    @staticmethod
    def _fast_input_signature(image_frame, camera_info):
        """缓存保存不可变输入签名，防止复用对象后读到旧图像或旧内参结果。"""

        try:
            metadata = json.dumps(image_frame.metadata, sort_keys=True, default=lambda item: item.model_dump(mode="json"))
            calibration = camera_info.model_dump_json() if camera_info is not None else None
        except (TypeError, ValueError, AttributeError):
            _INPUT_LOG_LIMITER.log(LOGGER, logging.DEBUG, "帧元数据无法编码，跳过本帧缓存。",
                                   key="fast_input_signature", interval_sec=5.0, exc_info=True)
            return None
        if len(metadata) > 8192 or not isinstance(image_frame.data, (bytes, type(None))):
            return None
        return (image_frame.timestamp, id(image_frame.data), len(image_frame.data or b""),
                image_frame.width_px, image_frame.height_px, image_frame.encoding, image_frame.frame_id,
                image_frame.uri, image_frame.artifact_id, metadata, calibration)

    def describe_current_scene(
        self,
        *,
        camera_id: Optional[str] = None,
        refresh: bool = True,
        requested_by: Optional[str] = None,
        detector_backend_name: Optional[str] = None,
        detection_metadata: Optional[Dict[str, object]] = None,
        open_vocabulary_classes: Optional[Iterable[str]] = None,
    ) -> PerceptionContext:
        """返回当前场景的结构化中文摘要。"""

        if refresh:
            return self.perceive_current_scene(
                camera_id=camera_id,
                requested_by=requested_by,
                detector_backend_name=detector_backend_name,
                detection_metadata=detection_metadata,
                open_vocabulary_classes=open_vocabulary_classes,
            )
        context = self.get_latest_perception(camera_id)
        if context is None:
            raise GatewayError("当前还没有可用的感知缓存。")
        return context

    def publish_current_video_frame(
        self,
        *,
        camera_id: Optional[str] = None,
        requested_by: Optional[str] = None,
    ) -> None:
        with self._operation():
            del requested_by
            image_frame = self._read_latest_pushed_image(camera_id)
            if image_frame is None:
                return
            self._publish_video_frame(
                image_frame,
                source_name=str((image_frame.metadata or {}).get("source_kind") or "video_ingest_receiver"),
            )

    def _read_latest_pushed_image(
        self,
        camera_id: Optional[str],
        *, wait_for_frame=True,
    ) -> Optional["ImageFrame"]:
        from contracts.image import ImageFrame

        video_frame_source = self._video_frame_source
        if video_frame_source is None:
            return None
        with self.video_consumer():
            image_frame = video_frame_source.get_latest_image(camera_id)
            # 单次视觉请求也要给按需推流的拥有方送达首帧的机会。
            if wait_for_frame and self._video_frame_source_status().get("source") == "video_shared_memory":
                deadline = time.monotonic() + 15.0
                while image_frame is None and not self._closing and time.monotonic() < deadline:
                    time.sleep(.02)
                    image_frame = video_frame_source.get_latest_image(camera_id)
        if image_frame is not None and not isinstance(image_frame, ImageFrame):
            raise TypeError("视频推流抽帧源必须返回标准 ImageFrame。")
        return image_frame

    def video_consumer(self):
        if self._video_frame_source_status().get("source") != "video_shared_memory":
            return nullcontext()
        from services.perception.video_demand import video_consumer
        return video_consumer()

    def point_cloud_consumer(self):
        providers = getattr(self._provider_owner, "providers", None)
        if isinstance(providers, DemandPointCloudProvider):
            return providers.point_cloud_consumer()
        return nullcontext()

    def get_latest_perception(self, camera_id: Optional[str] = None) -> Optional[PerceptionContext]:
        """返回最新感知结果。"""

        with self._context_condition:
            return self._select_latest(self._latest_by_camera, camera_id)

    @staticmethod
    def _select_latest(contexts, camera_id):
        if camera_id is not None:
            return contexts.get(camera_id)
        return max(contexts.values(), key=lambda item: item.timestamp, default=None)

    def get_latest_full_perception(self, camera_id: Optional[str] = None) -> Optional[PerceptionContext]:
        """返回最近一次完整感知结果；快速帧不会覆盖完整结果快照。"""
        with self._context_condition:
            return self._select_latest(self._latest_full_by_camera, camera_id)

    def list_history(
        self,
        *,
        camera_id: Optional[str] = None,
        limit: Optional[int] = None,
    ) -> Tuple[PerceptionContext, ...]:
        """返回感知历史。"""

        with self._context_condition:
            items = list(self._history)
        if camera_id is not None:
            items = [item for item in items if item.camera_id == camera_id]
        if limit is not None:
            items = items[-max(0, limit) :]
        return tuple(items)

    def _apply_detection_metadata(
        self,
        image_frame,
        *,
        detection_metadata: Optional[Dict[str, object]] = None,
        open_vocabulary_classes: Optional[Iterable[str]] = None,
    ):
        metadata_update = dict(detection_metadata or {})
        # 请求补充的检测条件不能覆盖图像来源、时间或标定事实。
        for key in ("source_timestamp_ns", "source_clock_domain", "source_clock_id", "source_sequence",
                    "ingest_sequence", "frame_timestamp", "frame_timestamp_source", "frame_timestamp_unknown",
                    "camera_info", "camera_id", "frame_id", "width_px", "height_px", "source_kind"):
            metadata_update.pop(key, None)
        if open_vocabulary_classes is not None:
            class_names = [str(item).strip() for item in open_vocabulary_classes if str(item or "").strip()]
            if class_names:
                metadata_update["open_vocabulary_classes"] = class_names
                metadata_update.setdefault("target_classes", class_names)
        if metadata_update:
            return image_frame.model_copy(
                update={"metadata": {**dict(image_frame.metadata or {}), **metadata_update}},
                deep=False,
            )
        return image_frame

    @staticmethod
    def _source_timestamp_metadata(image_frame) -> Dict[str, Any]:
        """保留输入时间及已声明来源，不能用结果发布时间冒充采集时间。"""

        metadata = dict(image_frame.metadata or {})
        timestamp = image_frame.timestamp
        if timestamp.tzinfo is None:
            timestamp = timestamp.replace(tzinfo=timezone.utc)
        result = {
            "frame_timestamp": None if metadata.get("frame_timestamp_unknown") else timestamp.isoformat(),
            "frame_timestamp_source": str(metadata.get("frame_timestamp_source") or "image_frame.timestamp"),
        }
        value = metadata.get("source_timestamp_ns")
        if isinstance(value, int) and not isinstance(value, bool) and value > 0:
            result["source_timestamp_ns"] = value
        sequence = metadata.get("source_sequence", metadata.get("ingest_sequence"))
        if isinstance(sequence, int) and not isinstance(sequence, bool) and sequence >= 0:
            result["source_sequence"] = sequence
        for key in ("source_clock_domain", "source_clock_id"):
            if isinstance(metadata.get(key), str):
                result[key] = metadata[key]
        return result

    def list_latest_contexts(self) -> Tuple[PerceptionContext, ...]:
        """返回各相机当前最新感知结果。"""

        with self._context_condition:
            items = sorted(self._latest_by_camera.values(), key=lambda item: (item.camera_id, item.timestamp))
        return tuple(items)


    def _publish_context(self, context, *, image_frame, source, kind, full, geometry_inputs=None):
        with self._context_condition:
            self._ensure_open()
            latest_time = self._latest_frame_timestamp_by_camera.get(context.camera_id)
            publish_latest = not full or latest_time is None or image_frame.timestamp >= latest_time
            if publish_latest:
                self._state_store.write(
                    StateNamespace.PERCEPTION, context.camera_id, context, source=source,
                    metadata={"kind": kind, "pipeline_name": self._pipeline_name},
                )
            self._history.append(context)
            if publish_latest:
                self._latest_by_camera[context.camera_id] = context
                self._latest_frame_timestamp_by_camera[context.camera_id] = image_frame.timestamp
            if full:
                self._latest_full_by_camera[context.camera_id] = context
                self._full_inputs[context.camera_id] = (context.observation.observation_id, image_frame,
                    context.camera_info, geometry_inputs[2] if geometry_inputs else None,
                    geometry_inputs[1] if geometry_inputs else None)

    def _ensure_open(self):
        if self._closing:
            raise GatewayError("感知服务正在关闭。", error_code="perception_closed", http_status=409)

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

    def close(self, timeout=5.0) -> None:
        """关闭检测执行者，并等待在途感知结束；真实执行者退出失败直接报错。"""
        with self._context_condition:
            if self._closed:
                return
            self._closing = True
        if not self._detector_pipeline.close(timeout=timeout):
            raise GatewayError("检测执行者未能在期限内退出。", error_code="perception_close_failed", http_status=503)
        with self._context_condition:
            while self._active_operations:
                self._context_condition.wait()
            for camera_id in self._latest_by_camera:
                self._state_store.write(StateNamespace.PERCEPTION, camera_id, None,
                                        source="perception_service", metadata={"kind": "closed"})
            self._latest_by_camera.clear()
            self._latest_full_by_camera.clear()
            self._full_inputs.clear()
            self._latest_frame_timestamp_by_camera.clear()
            self._history.clear()
            self._fast_frame_cache.clear()
            self._closed = True
        if self._point_cloud_subscription is not None:
            self._stream_bus.unsubscribe(self._point_cloud_subscription)
            self._point_cloud_subscription = None
        with self._point_cloud_lock:
            self._point_cloud_frames = ()

    def _attach_segmentation_results(
        self,
        detection_bundle: DetectionBundle,
        segmentation_bundle: SegmentationBundle,
    ) -> DetectionBundle:
        if not segmentation_bundle.segments:
            return detection_bundle
        segment_by_index = {
            int(segment.metadata["detection_index"]): segment
            for segment in segmentation_bundle.segments
            if isinstance(segment.metadata, dict) and "detection_index" in segment.metadata
        }
        detections = []
        changed = False
        for index, detection in enumerate(detection_bundle.detections_2d):
            segment = segment_by_index.get(index)
            if segment is None:
                detections.append(detection)
                continue
            segmentation = segment.segmentation or detection.segmentation
            segment_metadata = dict(segment.metadata or {})
            merged_attributes = dict(detection.attributes or {})
            if segmentation is not None:
                for key, value in self._segmentation_to_attributes(segmentation).items():
                    if value is not None and key not in merged_attributes:
                        merged_attributes[key] = value
            for key, value in segment_metadata.items():
                if value is not None and key not in merged_attributes:
                    merged_attributes[key] = value
            merged_attributes.setdefault("segmentation_backend", self._segmentation_backend.spec.name)
            merged_attributes.setdefault("segmentation_backend_kind", self._segmentation_backend.spec.backend_kind)
            if "segmentation_score" not in merged_attributes:
                merged_attributes["segmentation_score"] = float(segment.score)
            update = {"attributes": merged_attributes}
            if segmentation is not None:
                update["segmentation"] = segmentation
            detections.append(detection.model_copy(update=update, deep=False))
            changed = True
        if not changed:
            return detection_bundle
        return DetectionBundle(
            detections_2d=tuple(detections),
            detections_3d=detection_bundle.detections_3d,
            metadata={
                **dict(detection_bundle.metadata),
                "segmentation_backend": self._segmentation_backend.spec.name,
                "segmentation_count": len(segmentation_bundle.segments),
                "segmentation_source": segmentation_bundle.metadata.get("source", self._segmentation_backend.spec.backend_kind),
            },
        )

    def _elapsed_ms(self, started_at: float) -> float:
        return round((perf_counter() - started_at) * 1000.0, 3)

    def _provider_stage_status(self, backend_kind: str, metadata: Dict[str, object]) -> str:
        normalized_kind = str(backend_kind or "").lower()
        reason = str((metadata or {}).get("reason") or "").lower()
        source = str((metadata or {}).get("source") or "").lower()
        if "disabled" in normalized_kind or reason == "disabled_by_config":
            return "disabled"
        if "fallback" in normalized_kind or "fallback" in reason or "fallback" in source:
            return "fallback"
        if str((metadata or {}).get("stage_status") or "").strip():
            return str(metadata.get("stage_status"))
        return "completed"

    def _provider_fallback_reason(self, *metadata_items: Dict[str, object]) -> str:
        for metadata in metadata_items:
            for key in ("fallback_reason", "reason", "source"):
                value = str((metadata or {}).get(key) or "").strip()
                if value:
                    return value
        return ""

    def _segmentation_to_attributes(self, segmentation: SegmentationMask2D) -> Dict[str, object]:
        attributes: Dict[str, object] = dict(segmentation.attributes or {})
        if segmentation.polygon_px:
            polygon = [[float(x), float(y)] for x, y in segmentation.polygon_px]
            attributes.setdefault("mask_polygon_px", polygon)
            attributes.setdefault("segmentation_polygon_px", polygon)
        if segmentation.centroid_uv is not None:
            centroid = [float(segmentation.centroid_uv[0]), float(segmentation.centroid_uv[1])]
            attributes.setdefault("mask_center_px", centroid)
            attributes.setdefault("centroid_uv", centroid)
        if segmentation.bbox_xyxy is not None:
            attributes.setdefault("bbox_xyxy", [float(value) for value in segmentation.bbox_xyxy])
        if segmentation.area_px is not None:
            attributes.setdefault("area_px", float(segmentation.area_px))
        if segmentation.area_ratio is not None:
            attributes.setdefault("area_ratio", float(segmentation.area_ratio))
        if segmentation.crop_box is not None:
            attributes.setdefault("crop_box", [float(value) for value in segmentation.crop_box])
        if segmentation.contour:
            attributes.setdefault("contour", dict(segmentation.contour))
        if segmentation.frame_id:
            attributes.setdefault("frame_id", segmentation.frame_id)
        if segmentation.camera_id:
            attributes.setdefault("camera_id", segmentation.camera_id)
        if segmentation.source:
            attributes.setdefault("segmentation_source", segmentation.source)
        if segmentation.truncation_flags:
            attributes.setdefault("truncation_flags", list(segmentation.truncation_flags))
        if segmentation.occlusion_flags:
            attributes.setdefault("occlusion_flags", list(segmentation.occlusion_flags))
        quality = segmentation.quality
        if quality.score is not None:
            attributes.setdefault("segmentation_score", float(quality.score))
        if quality.predicted_iou is not None:
            attributes.setdefault("predicted_iou", float(quality.predicted_iou))
        if quality.stability_score is not None:
            attributes.setdefault("stability_score", float(quality.stability_score))
        if quality.depth_coverage is not None:
            attributes.setdefault("depth_coverage", float(quality.depth_coverage))
        if quality.map_projection_coverage is not None:
            attributes.setdefault("map_projection_coverage", float(quality.map_projection_coverage))
        if quality.reprojection_error is not None:
            attributes.setdefault("reprojection_error", float(quality.reprojection_error))
        return attributes

    def _attach_track_ids(
        self,
        detections_2d: Tuple[Detection2D, ...],
        tracks: Tuple[Track, ...],
    ) -> Tuple[Detection2D, ...]:
        active_states = {TrackState.TENTATIVE, TrackState.TRACKED}
        indexed = {}
        for track in tracks:
            if track.state in active_states and track.bbox is not None:
                box = track.bbox
                key = (track.label, box.x_px, box.y_px, box.width_px, box.height_px)
                indexed.setdefault(key, []).append(track)
        assigned: List[Detection2D] = []
        used_ids = set()
        for detection in detections_2d:
            box = detection.bbox
            key = (detection.label, box.x_px, box.y_px, box.width_px, box.height_px)
            matched = next((track for track in indexed.get(key, ())
                            if track.track_id not in used_ids
                            and (not detection.camera_id or not track.attributes.get("camera_id")
                                 or track.attributes.get("camera_id") == detection.camera_id)), None)
            resolved_track_id = matched.track_id if matched is not None else detection.track_id
            if resolved_track_id in used_ids:
                resolved_track_id = None
            if resolved_track_id:
                used_ids.add(resolved_track_id)
            attributes = dict(detection.attributes or {})
            attributes.pop("track_id", None)
            if resolved_track_id:
                attributes["track_id"] = resolved_track_id
            track_metadata = dict(matched.attributes or {}) if matched is not None else {}
            for name in ("last_seen", "velocity_hint", "short_trajectory", "track_age_frames", "track_miss_frames", "tracker"):
                if name in track_metadata:
                    attributes[name] = track_metadata[name]
            assigned.append(detection.model_copy(
                update={"track_id": resolved_track_id, "attributes": attributes}, deep=False,
            ))
        return tuple(assigned)

    def _build_fast_scene_summary(
        self,
        *,
        camera_id: str,
        detections_2d: Tuple[Detection2D, ...],
        tracks: Tuple[Track, ...],
        stage_duration_ms: Dict[str, float],
    ) -> SceneSummary:
        object_counts: Dict[str, Dict[str, object]] = {}
        active_track_ids = {
            track.track_id
            for track in tracks
            if track.state in {TrackState.TENTATIVE, TrackState.TRACKED}
        }
        for detection in detections_2d:
            label = str(detection.label or "unknown")
            bucket = object_counts.setdefault(
                label,
                {
                    "count": 0,
                    "max_score": 0.0,
                    "track_ids": set(),
                },
            )
            bucket["count"] = int(bucket["count"]) + 1
            bucket["max_score"] = max(float(bucket["max_score"]), float(detection.score))
            if detection.track_id and detection.track_id in active_track_ids:
                bucket["track_ids"].add(detection.track_id)
        objects = []
        for label, bucket in sorted(object_counts.items(), key=lambda item: item[0]):
            track_ids = sorted(str(item) for item in bucket["track_ids"])
            objects.append(
                SceneObjectSummary(
                    label=label,
                    count=int(bucket["count"]),
                    tracked_count=len(track_ids),
                    max_score=float(bucket["max_score"]),
                    camera_ids=[camera_id],
                    track_ids=track_ids,
                    attributes={"summary_source": "fast_detection"},
                )
            )
        if objects:
            headline = "当前画面检测到 " + "、".join(f"{item.label} {item.count} 个" for item in objects) + "。"
        else:
            headline = "当前画面未检测到目标。"
        active_count = sum(1 for track in tracks if track.state in {TrackState.TENTATIVE, TrackState.TRACKED})
        return SceneSummary(
            headline=headline,
            details=[
                "快速检测链路结果：仅包含 detector 与 2D tracker，未等待 segmentation/geometry/scene_description。",
            ],
            objects=objects,
            detection_count=len(detections_2d),
            active_track_count=active_count,
            metadata={
                "summary_source": "fast_detection",
                "stage_duration_ms": dict(stage_duration_ms),
            },
        )

    def _prepare_detection_overlay_frame(
        self, context: PerceptionContext,
    ) -> Optional[PreparedStreamPublication]:
        """在检测顺序锁内记录叠加帧；调用方须在管线锁外完成同步分发。"""
        if self._stream_bus is None:
            return None
        observation = context.observation
        detection_count = len(observation.detections_2d)
        detections = tuple(self._serialize_detection_overlay(detection) for detection in observation.detections_2d[:64])
        now = perf_counter()
        previous_publish_at = self._last_detection_overlay_publish_at_by_camera.get(context.camera_id)
        publish_interval_ms = None
        if previous_publish_at is not None:
            publish_interval_ms = round((now - previous_publish_at) * 1000.0, 3)
        self._last_detection_overlay_publish_at_by_camera[context.camera_id] = now
        overlay_phase = str(context.metadata.get("overlay_phase") or "full_scene_ready")
        # vis-three-fixes AC0.3 / AC3.2：从 observation.metadata.detection_metadata 中提取 NMS 统计，
        # 写入 payload.nms_stats 与 stage_duration_ms.pipeline_lock_wait_ms。
        detection_metadata = dict(context.metadata.get("detection_metadata") or {})
        nms_stats = dict(detection_metadata.get("postprocess") or {})
        stage_status = dict(context.metadata.get("stage_status") or {})
        available = stage_status.get("detector", "completed") in {"completed", "fallback"}
        payload = {
            "available": available,
            "observation_id": observation.observation_id,
            "frame_timestamp": context.metadata.get("frame_timestamp"),
            "frame_timestamp_source": context.metadata.get("frame_timestamp_source"),
            "source_clock_domain": context.metadata.get("source_clock_domain"),
            "source_clock_id": context.metadata.get("source_clock_id"),
            "source_sequence": context.metadata.get("source_sequence"),
            "provider_fallback_used": bool(context.metadata.get("provider_fallback_used") or stage_status.get("detector") == "fallback" or detection_metadata.get("worker_fallback_used")),
            "provider_fallback_reason": context.metadata.get("provider_fallback_reason") or self._provider_fallback_reason(detection_metadata),
            "worker_fallback_used": bool(detection_metadata.get("worker_fallback_used")),
            "worker_fallback_reason": detection_metadata.get("worker_fallback_reason"),
            "inference_execution": detection_metadata.get("inference_execution"),
            "camera_id": context.camera_id,
            "frame_id": observation.frame_id,
            "width_px": observation.metadata.get("width_px"),
            "height_px": observation.metadata.get("height_px"),
            "detector_backend": context.detector_backend,
            "tracker_backend": context.tracker_backend,
            "detection_count": detection_count,
            "displayed_detection_count": len(detections),
            "detections_truncated": detection_count > len(detections),
            "detections_2d": detections,
            "stage_status": dict(context.metadata.get("stage_status") or {}),
            "stage_duration_ms": dict(context.metadata.get("stage_duration_ms") or {}),
            "overlay_phase": overlay_phase,
            "publish_interval_ms": publish_interval_ms,
            "nms_stats": nms_stats,
            "output_validation": dict(detection_metadata.get("output_validation") or {}),
            "pipeline_lock_wait_ms": context.metadata.get("pipeline_lock_wait_ms"),
            "source_timestamp_ns": context.metadata.get("source_timestamp_ns"),
        }
        return self._stream_bus.prepare_publication(StreamFrame(
            topic="video.detections",
            source="perception_service",
            payload=payload,
            frame_id=observation.frame_id,
            status="available" if available else "unavailable",
            metadata={
                "camera_id": context.camera_id,
                "detector_backend": context.detector_backend,
                "detection_count": len(detections),
                "overlay_phase": overlay_phase,
                "publish_interval_ms": publish_interval_ms,
            },
        ))

    @staticmethod
    def _serialize_detection_overlay(detection: Detection2D) -> Dict[str, Any]:
        bbox = detection.bbox
        return {
            "label": detection.label,
            "score": detection.score,
            "track_id": detection.track_id,
            "bbox": {
                "x_px": bbox.x_px,
                "y_px": bbox.y_px,
                "width_px": bbox.width_px,
                "height_px": bbox.height_px,
            },
            "attributes": {key: detection.attributes[key] for key in ("display_name_zh",) if key in detection.attributes},
            # 机器人自身的检测保留显示，页面灰显，不代表环境物体。
            "self_body": detection.attributes.get("self_body") is True,
        }

    def _publish_video_frame(self, image_frame, *, source_name: str) -> None:
        if self._stream_bus is None:
            return
        preview_camera_id = getattr(self._video_frame_sink, "camera_id", None)
        if preview_camera_id and image_frame.camera_id != preview_camera_id:
            # 单槽预览按绑定来源发布，另一相机不能清空已显示画面。
            return
        input_signature = self._fast_input_signature(image_frame, None)
        previous = self._last_published_video_frames.get(image_frame.camera_id)
        if input_signature is not None and previous is not None and previous[0] is image_frame and previous[1] == input_signature:
            return
        self._last_published_video_frames[image_frame.camera_id] = (image_frame, input_signature)
        if len(self._last_published_video_frames) > 8:
            self._last_published_video_frames.pop(next(iter(self._last_published_video_frames)))
        metadata = dict(image_frame.metadata or {})
        payload_bytes = len(image_frame.data or b"")
        latest_frame = None
        if image_frame.data and image_frame.encoding.value == "jpeg" and self._video_frame_sink is not None:
            try:
                latest_frame = self._video_frame_sink(image_frame)
            except Exception:
                latest_frame = None
        sequence = getattr(latest_frame, "sequence", None)
        if not isinstance(sequence, int) and isinstance(metadata.get("ingest_sequence"), int):
            sequence = int(metadata["ingest_sequence"])
        source_kind = str(metadata.get("source_kind") or source_name or "")
        ingest_stream_frame = source_kind == "platform_video_ingest_stream"
        mjpeg_available = bool(
            (latest_frame is not None and isinstance(sequence, int))
            or (
                ingest_stream_frame
                and image_frame.data
                and image_frame.encoding.value == "jpeg"
                and isinstance(sequence, int)
            )
        )
        payload = {
            **self._source_timestamp_metadata(image_frame),
            "available": bool(mjpeg_available or image_frame.uri or image_frame.artifact_id),
            "camera_id": image_frame.camera_id,
            "frame_id": image_frame.frame_id,
            "width_px": image_frame.width_px,
            "height_px": image_frame.height_px,
            "encoding": image_frame.encoding.value,
            "payload_bytes": payload_bytes,
            "transport": "mjpeg" if mjpeg_available else "metadata",
            "stream_url": "/streams/video.mjpeg?" + urlencode({"camera_id": image_frame.camera_id}) if mjpeg_available else None,
            "metadata": dict(metadata),
            "ingest_status": self._video_frame_source_status(),
        }
        if image_frame.uri:
            payload["uri"] = image_frame.uri
        elif image_frame.artifact_id:
            payload["artifact_id"] = image_frame.artifact_id
        status = "available" if payload.get("stream_url") or payload.get("uri") or payload.get("artifact_id") else "unsupported"
        if status != "available":
            payload["message"] = f"浏览器暂不支持直接显示 {image_frame.encoding.value} 图像帧。"
        self._stream_bus.publish(
            "video.frame",
            source=source_name or "image_provider",
            payload=payload,
            frame_id=image_frame.frame_id,
            sequence=sequence,
            status=status,
            metadata={
                "camera_id": image_frame.camera_id,
                "encoding": image_frame.encoding.value,
                "payload_bytes": payload_bytes,
                "source_name": source_name or "image_provider",
                **metadata,
            },
        )

    def _video_frame_source_status(self) -> Dict[str, Any]:
        video_frame_source = self._video_frame_source
        if video_frame_source is None:
            return {"available": False, "connected": False, "reason": "video_frame_source_missing"}
        try:
            status = video_frame_source.status_snapshot()
        except Exception as exc:
            _INPUT_LOG_LIMITER.log(LOGGER, logging.ERROR, "读取视频源状态失败。",
                                   key="video_source_status", interval_sec=5.0, exc_info=True)
            return {"available": False, "connected": False, "reason": "status_snapshot_failed", "error": str(exc)}
        return dict(status or {})


    def video_frame_source_status(self) -> Dict[str, Any]:
        """返回平台端侧推流接收状态，供持续感知运行时做实时性门禁。"""

        status = self._video_frame_source_status()
        preview_camera_id = getattr(self._video_frame_sink, "camera_id", None)
        if preview_camera_id:
            status["camera_id"] = preview_camera_id
        return status

    def _camera_info_from_video_frame(self, image_frame):
        from contracts.image import CameraInfo

        metadata = dict(getattr(image_frame, "metadata", {}) or {})
        camera_info_payload = metadata.get("camera_info")
        if isinstance(camera_info_payload, CameraInfo):
            return camera_info_payload
        if isinstance(camera_info_payload, dict):
            try:
                return CameraInfo(**camera_info_payload)
            except Exception:
                LOGGER.exception("读取或解析相机内参失败 camera=%s。", image_frame.camera_id)
                raise
        # 标准提供器可以独立缓存内参，图像读取不必每次复制像素或元数据。
        getter = getattr(self._video_frame_source, "get_camera_info", None)
        if callable(getter):
            try:
                camera_info = getter(image_frame.camera_id)
            except Exception:
                LOGGER.exception("读取或解析相机内参失败 camera=%s。", image_frame.camera_id)
                raise
            if isinstance(camera_info, CameraInfo):
                return camera_info
        return None

    def _read_live_pose_and_frame_tree(self):
        pose = contract_pose(None if self._geometry_state is None else self._geometry_state.pose())
        return pose, self._frame_tree

    def _get_point_cloud_provider_optional(self) -> Optional[PointCloudProvider]:
        providers = getattr(self._provider_owner, "providers", None)
        if providers is None or not isinstance(providers, PointCloudProvider):
            return None
        if not providers.is_available():
            return None
        return providers

    @staticmethod
    def _has_usable_point_cloud(frame) -> bool:
        if frame is None:
            return False
        points = getattr(frame, "points", None)
        try:
            if points is not None and len(points) > 0:
                return True
        except TypeError:
            pass
        array = getattr(frame, "points_xyz_array", None)
        shape = getattr(array, "shape", ())
        try:
            return (
                array is not None
                and len(shape) == 2
                and shape[0] > 0
                and shape[1] >= 3
            )
        except (TypeError, IndexError):
            return False

    def _geometry_transforms(self):
        """把设备标定接口的相机静态边接回几何坐标树。"""
        from services.geometry.client import Transform
        transforms = () if self._geometry_client is None else self._geometry_client.get_frame_tree()
        providers = getattr(self._provider_owner, "providers", None)
        topology = providers.get_sensor_topology() if isinstance(providers, CalibrationProvider) else None
        if topology is not None:
            transforms = (*transforms, *(Transform(
                parent_frame=pair.parent_frame_id, child_frame=pair.child_frame_id,
                x=pair.initial_translation.x, y=pair.initial_translation.y, z=pair.initial_translation.z,
                qx=pair.initial_rotation.x, qy=pair.initial_rotation.y,
                qz=pair.initial_rotation.z, qw=pair.initial_rotation.w, timestamp_ns=0,
            ) for pair in topology.pairs))
        return transforms

    def _cache_point_cloud(self, frame):
        """使用方只留采集时刻最新的十帧，不改变总线快照保留规则。"""
        with self._point_cloud_lock:
            frames = sorted((*self._point_cloud_frames, frame), key=lambda item: _stamp(item.payload, item.payload.metadata))
            self._point_cloud_frames = tuple(frames[-_POINT_CLOUD_CACHE_FRAMES:])

    def _capture_geometry(self, image_frame):
        snapshot = None if self._geometry_state is None else self._geometry_state.snapshot()
        transforms = () if snapshot is None else self._geometry_transforms()
        with self._point_cloud_lock:
            cloud_frames = self._point_cloud_frames
        geometry = capture_geometry(self._stream_bus, image_frame, transforms=transforms,
            map_name=None if snapshot is None else snapshot.map_name, point_cloud_frames=cloud_frames)
        self._frame_tree = geometry.frame_tree
        return geometry

    def _capture_cached_geometry_inputs(self, image_frame):
        """单次完整识别在模型等待前保留已有几何输入。"""
        geometry = self._capture_geometry(image_frame)
        return geometry.point_cloud, geometry.frame_tree, geometry.pose

    def self_body_filter_status(self) -> Dict[str, Any]:
        return self._self_body_filter.status()

    def _apply_self_body_filter(self, detection_bundle, *, image_frame, camera_info, frame_tree):
        """标记机器人自身检测；只增补属性和区域元数据，检测数量与顺序不变。"""
        if not detection_bundle.detections_2d:
            return detection_bundle
        if frame_tree is None:
            frame_tree = self._latest_stream_payload("localization.frame_tree")
        if frame_tree is None:
            frame_tree = self._read_live_pose_and_frame_tree()[1]
        result = self._self_body_filter.apply(
            detection_bundle.detections_2d, image_frame=image_frame,
            camera_info=camera_info, frame_tree=frame_tree,
        )
        return detection_bundle.__class__(
            detections_2d=result.detections_2d,
            detections_3d=detection_bundle.detections_3d,
            metadata={**dict(detection_bundle.metadata), **result.metadata},
        )

    def _augment_detection_geometry(
        self,
        detection_bundle,
        *,
        image_frame,
        camera_info,
        cached_geometry_inputs,
    ):
        if camera_info is None or not detection_bundle.detections_2d:
            return detection_bundle.__class__(
                detections_2d=detection_bundle.detections_2d,
                detections_3d=detection_bundle.detections_3d,
                metadata={
                    **dict(detection_bundle.metadata),
                    "geometry_stage_status": "completed",
                    "geometry_degraded_reason": "camera_info_missing" if detection_bundle.detections_2d else "no_detections",
                },
            )

        if cached_geometry_inputs is not None:
            point_cloud_frame, frame_tree, current_pose = cached_geometry_inputs
        else:
            geometry = self._capture_geometry(image_frame)
            point_cloud_frame, frame_tree, current_pose = geometry.point_cloud, geometry.frame_tree, geometry.pose

        detections_2d, geometry_metadata = self._geometry_augmentor.augment_detections(
            detection_bundle.detections_2d,
            image_frame=image_frame,
            camera_info=camera_info,
            point_cloud=point_cloud_frame,
            frame_tree=frame_tree,
            current_pose=current_pose,
            self_region=region_from_detection_metadata(detection_bundle.metadata),
        )
        if not self._has_usable_point_cloud(point_cloud_frame):
            geometry_metadata = {
                **dict(geometry_metadata),
                "geometry_projection_status": "pointcloud_unavailable",
                "geometry_degraded_reason": geometry_metadata.get("geometry_projection_degradation_reason") or "pointcloud_unavailable",
            }
        if current_pose is None and geometry_metadata.get("geometry_projection_degradation_reason") == "point_cloud_unavailable":
            geometry_metadata.update(geometry_projection_degradation_reason="capture_pose_unavailable",
                                     geometry_projection_degradation_message="图像时刻的位姿不可用")
        geometry_metadata["geometry_synchronization"] = "source_time_window_interpolation"
        if detections_2d == detection_bundle.detections_2d and not geometry_metadata:
            return detection_bundle.__class__(
                detections_2d=detection_bundle.detections_2d,
                detections_3d=detection_bundle.detections_3d,
                metadata={
                    **dict(detection_bundle.metadata),
                    "geometry_stage_status": "completed",
                },
            )
        support_count = int(geometry_metadata.get("geometry_support_point_total") or 0)
        degraded_reason = str(geometry_metadata.get("geometry_projection_degradation_reason") or geometry_metadata.get("geometry_degraded_reason") or "")
        if support_count <= 0 and self._has_usable_point_cloud(point_cloud_frame):
            degraded_reason = degraded_reason or "pointcloud_projection_failed"
        return detection_bundle.__class__(
            detections_2d=detections_2d,
            detections_3d=detection_bundle.detections_3d,
            metadata={
                **dict(detection_bundle.metadata),
                **geometry_metadata,
                "geometry_stage_status": "completed",
                "geometry_degraded_reason": degraded_reason,
            },
        )

    def _get_latest_point_cloud_optional(self):
        provider = self._get_point_cloud_provider_optional()
        if provider is None:
            return None
        try:
            return provider.get_latest_point_cloud()
        except Exception:
            LOGGER.exception("读取点云提供器失败。")
            raise

    def _latest_stream_payload(self, topic: str):
        if self._stream_bus is None:
            return None
        frame = self._stream_bus.latest_frame(topic)
        if frame is None or frame.status != "available":
            return None
        return frame.payload

    def _normalize_camera_id(self, camera_id: str) -> str:
        normalized = []
        for char in camera_id.lower():
            if char.isalnum():
                normalized.append(char)
            else:
                normalized.append("_")
        value = "".join(normalized).strip("_")
        return value or "camera"
