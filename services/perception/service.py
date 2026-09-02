from __future__ import annotations

import threading
from collections import deque
from urllib.parse import urlencode
from time import perf_counter

from contracts.events import RuntimeEventCategory
from contracts.naming import build_observation_id
from contracts.perception import Detection2D, Observation, SegmentationMask2D, Track, TrackState
from contracts.runtime_views import PerceptionContext, SceneObjectSummary, SceneSummary
from core import EventBus, StateNamespace, StateStore, StreamBus
from core.errors import GatewayError
from providers import LocalizationProvider, PointCloudProvider
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
from services.perception.scene import SimpleSceneDescriptionBackend
from services.perception.trackers import Basic2DTrackerBackend
from typing import Any, Callable, Deque, Dict, Iterable, List, Optional, Protocol, Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    from contracts.image import ImageFrame

VideoFrameSink = Callable[[object], Optional[object]]


class VideoFrameSource(Protocol):
    """平台已接收视频流抽帧源。"""

    def get_latest_image(self, camera_id: Optional[str] = None) -> Optional[object]:
        """返回端侧推流接收缓冲中的最近图像。"""

    def status_snapshot(self) -> Dict[str, Any]:
        """返回端侧推流接收状态。"""


class PerceptionService:
    """统一感知、跟踪与场景语义摘要服务。"""

    def __init__(
        self,
        *,
        provider_owner,
        localization_provider_owner=None,
        artifact_service: ArtifactService,
        state_store: StateStore,
        detector_pipeline: DetectorPipeline,
        tracker_backend: Optional[TrackerBackend] = None,
        scene_description_backend: Optional[SceneDescriptionBackend] = None,
        segmentation_backend: Optional[SegmentationBackend] = None,
        geometry_augmentor: Optional[DetectionGeometryAugmentor] = None,
        event_bus: Optional[EventBus] = None,
        stream_bus: Optional[StreamBus] = None,
        video_frame_sink: Optional[VideoFrameSink] = None,
        video_frame_source: Optional[VideoFrameSource] = None,
        live_pose_provider=None,
        history_limit: int = 100,
        pipeline_name: str = "default_perception_pipeline",
    ) -> None:
        self._provider_owner = provider_owner
        self._localization_provider_owner = localization_provider_owner or provider_owner
        self._artifact_service = artifact_service
        self._state_store = state_store
        self._detector_pipeline = detector_pipeline
        self._tracker_backend = tracker_backend or Basic2DTrackerBackend()
        self._scene_description_backend = scene_description_backend or SimpleSceneDescriptionBackend()
        self._segmentation_backend = segmentation_backend or DisabledSegmentationBackend()
        self._geometry_augmentor = geometry_augmentor or DetectionGeometryAugmentor()
        self._event_bus = event_bus
        self._stream_bus = stream_bus
        self._video_frame_sink = video_frame_sink
        self._video_frame_source = video_frame_source
        self._live_pose_provider = live_pose_provider
        self._history: Deque[PerceptionContext] = deque(maxlen=max(1, history_limit))
        self._latest_by_camera: Dict[str, PerceptionContext] = {}
        self._latest_full_by_camera: Dict[str, PerceptionContext] = {}
        self._pipeline_name = pipeline_name
        self._last_detection_overlay_publish_at_by_camera: Dict[str, float] = {}
        # P1-2：感知全管线无并发保护——process_image 可被后台运行时线程、
        # FastAPI threadpool 能力调用、memory 到点复核并发触发，模型/跟踪器状态
        # 在无锁下会竞态损坏。粗粒度串行整条管线（推理本身是吞吐瓶颈）。
        self._pipeline_lock = threading.Lock()

    def set_video_frame_sink(self, video_frame_sink: Optional[VideoFrameSink]) -> None:
        self._video_frame_sink = video_frame_sink

    def set_video_frame_source(self, video_frame_source: Optional[VideoFrameSource]) -> None:
        self._video_frame_source = video_frame_source

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

        P1-2：整条管线（检测/分割/几何/跟踪/场景摘要）在一把锁内串行执行，
        避免后台运行时线程、HTTP threadpool 能力、memory 到点复核并发触发时
        模型与跟踪器内部状态竞态损坏。
        """

        # vis-three-fixes AC2.4a：在加锁前后各打点，计算 pipeline_lock_wait_ms。
        requested_at = perf_counter()
        with self._pipeline_lock:
            acquired_at = perf_counter()
            return self._process_image_locked(
                image_frame,
                camera_info,
                image_artifact=image_artifact,
                requested_by=requested_by,
                detector_backend_name=detector_backend_name,
                source_name=source_name,
                detection_metadata=detection_metadata,
                open_vocabulary_classes=open_vocabulary_classes,
                lock_wait_ms=(acquired_at - requested_at) * 1000.0,
            )

    def _process_image_locked(
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
        lock_wait_ms: Optional[float] = None,
    ) -> PerceptionContext:
        """process_image 的实际实现，调用方须已持有 _pipeline_lock。"""

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
        )
        stage_duration_ms["geometry"] = self._elapsed_ms(stage_started_at)
        stage_status["geometry"] = str(
            detection_bundle.metadata.get("geometry_stage_status")
            or detection_bundle.metadata.get("geometry_status")
            or "completed"
        )

        stage_started_at = perf_counter()
        tracks = self._tracker_backend.update(
            image_frame,
            detections_2d=detection_bundle.detections_2d,
            detections_3d=detection_bundle.detections_3d,
            camera_info=camera_info,
        )
        stage_duration_ms["tracker"] = self._elapsed_ms(stage_started_at)
        stage_status["tracker"] = "completed"

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
        self._latest_by_camera[context.camera_id] = context
        self._latest_full_by_camera[context.camera_id] = context
        self._history.append(context)
        self._state_store.write(
            StateNamespace.PERCEPTION,
            context.camera_id,
            context,
            source=source_name or detector_spec.name,
            metadata={"kind": "latest_perception", "pipeline_name": self._pipeline_name},
        )
        if self._event_bus is not None:
            self._event_bus.publish(
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
        self._publish_detection_overlay_frame(context)
        return context

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

        image_frame = self._read_latest_pushed_image(camera_id)
        if image_frame is None:
            ingest_status = self._video_frame_source_status()
            required_source = self._resolve_required_source_id(camera_id)
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
        return self.process_image(
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

        image_frame = self._read_latest_pushed_image(camera_id)
        if image_frame is None:
            ingest_status = self._video_frame_source_status()
            required_source = self._resolve_required_source_id(camera_id)
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

        # vis-three-fixes AC2.4a：同上 process_image，记录 pipeline_lock_wait_ms。
        requested_at = perf_counter()
        with self._pipeline_lock:
            acquired_at = perf_counter()
            return self._process_image_fast_locked(
                image_frame,
                camera_info,
                requested_by=requested_by,
                detector_backend_name=detector_backend_name,
                source_name=source_name,
                detection_metadata=detection_metadata,
                open_vocabulary_classes=open_vocabulary_classes,
                lock_wait_ms=(acquired_at - requested_at) * 1000.0,
            )

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
    ) -> PerceptionContext:
        """process_image_fast 的实际实现，调用方须已持有 _pipeline_lock。"""

        from contracts.image import CameraInfo, ImageFrame

        if not isinstance(image_frame, ImageFrame):
            raise TypeError("PerceptionService.process_image_fast 仅接受标准 ImageFrame。")
        if camera_info is not None and not isinstance(camera_info, CameraInfo):
            raise TypeError("PerceptionService.process_image_fast 的 camera_info 必须是 CameraInfo。")

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
        tracks = self._tracker_backend.update(
            image_frame,
            detections_2d=detection_bundle.detections_2d,
            detections_3d=detection_bundle.detections_3d,
            camera_info=camera_info,
        )
        stage_duration_ms["tracker"] = self._elapsed_ms(stage_started_at)
        stage_status["tracker"] = "completed"

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
        self._latest_by_camera[context.camera_id] = context
        self._history.append(context)
        self._state_store.write(
            StateNamespace.PERCEPTION,
            context.camera_id,
            context,
            source=source_name or detector_spec.name,
            metadata={"kind": "latest_perception_fast", "pipeline_name": self._pipeline_name},
        )
        self._publish_detection_overlay_frame(context)
        return context

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
    ) -> Optional["ImageFrame"]:
        from contracts.image import ImageFrame

        video_frame_source = self._video_frame_source
        if video_frame_source is None:
            return None
        getter = getattr(video_frame_source, "get_latest_image", None)
        if not callable(getter):
            return None
        try:
            image_frame = getter(camera_id)
        except TypeError:
            try:
                image_frame = getter()
            except Exception:
                return None
        except Exception:
            return None
        if image_frame is not None and not isinstance(image_frame, ImageFrame):
            raise TypeError("视频推流抽帧源必须返回标准 ImageFrame。")
        return image_frame

    def get_latest_perception(self, camera_id: Optional[str] = None) -> Optional[PerceptionContext]:
        """返回最新感知结果。"""

        if camera_id is not None:
            return self._latest_by_camera.get(camera_id)
        if not self._latest_by_camera:
            return None
        return max(self._latest_by_camera.values(), key=lambda item: item.timestamp)

    def get_latest_full_perception(self, camera_id: Optional[str] = None) -> Optional[PerceptionContext]:
        """返回最近一次完整感知结果；fast path 结果不会覆盖该缓存。"""

        if camera_id is not None:
            return self._latest_full_by_camera.get(camera_id)
        if not self._latest_full_by_camera:
            return None
        return max(self._latest_full_by_camera.values(), key=lambda item: item.timestamp)

    def list_history(
        self,
        *,
        camera_id: Optional[str] = None,
        limit: Optional[int] = None,
    ) -> Tuple[PerceptionContext, ...]:
        """返回感知历史。"""

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
    def _source_timestamp_metadata(image_frame) -> Dict[str, int]:
        value = dict(image_frame.metadata or {}).get("source_timestamp_ns")
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            return {}
        return {"source_timestamp_ns": value}

    def list_latest_contexts(self) -> Tuple[PerceptionContext, ...]:
        """返回各相机当前最新感知结果。"""

        items = sorted(self._latest_by_camera.values(), key=lambda item: (item.camera_id, item.timestamp))
        return tuple(items)

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
        assigned: List[Detection2D] = []
        for detection in detections_2d:
            matched_track_id = None
            for track in tracks:
                if track.state not in active_states:
                    continue
                if track.label != detection.label:
                    continue
                if track.bbox is None:
                    continue
                if track.bbox == detection.bbox:
                    matched_track_id = track.track_id
                    break
            track_metadata = {}
            if matched_track_id:
                for track in tracks:
                    if track.track_id == matched_track_id:
                        track_metadata = dict(track.attributes or {})
                        break
            attributes = dict(detection.attributes or {})
            resolved_track_id = matched_track_id or detection.track_id
            if resolved_track_id:
                attributes["track_id"] = resolved_track_id
            for key in ("last_seen", "velocity_hint", "short_trajectory", "track_age_frames", "track_miss_frames", "tracker"):
                if key in track_metadata:
                    attributes[key] = track_metadata[key]
            assigned.append(
                detection.model_copy(
                    update={"track_id": resolved_track_id, "attributes": attributes},
                    deep=False,
                )
            )
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

    def _publish_detection_overlay_frame(self, context: PerceptionContext) -> None:
        if self._stream_bus is None:
            return
        observation = context.observation
        detections = tuple(self._serialize_detection_overlay(detection) for detection in observation.detections_2d)
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
        payload = {
            "available": True,
            "camera_id": context.camera_id,
            "frame_id": observation.frame_id,
            "width_px": observation.metadata.get("width_px"),
            "height_px": observation.metadata.get("height_px"),
            "detector_backend": context.detector_backend,
            "tracker_backend": context.tracker_backend,
            "detection_count": len(detections),
            "detections_2d": detections,
            "stage_status": dict(context.metadata.get("stage_status") or {}),
            "stage_duration_ms": dict(context.metadata.get("stage_duration_ms") or {}),
            "overlay_phase": overlay_phase,
            "publish_interval_ms": publish_interval_ms,
            "nms_stats": nms_stats,
            "pipeline_lock_wait_ms": context.metadata.get("pipeline_lock_wait_ms"),
            "source_timestamp_ns": context.metadata.get("source_timestamp_ns"),
        }
        self._stream_bus.publish(
            "video.detections",
            source="perception_service",
            payload=payload,
            frame_id=observation.frame_id,
            status="available",
            metadata={
                "camera_id": context.camera_id,
                "detector_backend": context.detector_backend,
                "detection_count": len(detections),
                "overlay_phase": overlay_phase,
                "publish_interval_ms": publish_interval_ms,
            },
        )

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
            "attributes": dict(detection.attributes or {}),
        }

    def _publish_video_frame(self, image_frame, *, source_name: str) -> None:
        if self._stream_bus is None:
            return
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
        snapshot = getattr(video_frame_source, "status_snapshot", None)
        if not callable(snapshot):
            return {"available": True, "connected": None, "reason": "status_snapshot_unavailable"}
        try:
            status = snapshot()
        except Exception as exc:
            return {"available": False, "connected": False, "reason": "status_snapshot_failed", "error": str(exc)}
        return dict(status or {})

    def _resolve_required_source_id(self, camera_id: Optional[str] = None) -> str:
        """从当前 ImageProvider 拉取 `get_stream_source_id()`，作为 ingest 门禁期望的源标识。

        若 platform 侧未挂 ImageProvider 或 provider 不实现该协议方法，降级返回
        `video_ingest_receiver`（与 `image_frame.metadata.source_kind` 默认值一致），
        保持向后兼容。cross-robot-generalization-foundations G5。
        """

        providers = getattr(self, "_image_providers", None) or {}
        # 优先按 camera_id 命中；未命中时取第一个 provider。
        provider = None
        if camera_id is not None:
            provider = providers.get(camera_id)
        if provider is None and providers:
            provider = next(iter(providers.values()))
        if provider is not None:
            get_id = getattr(provider, "get_stream_source_id", None)
            if callable(get_id):
                try:
                    source_id = str(get_id() or "").strip()
                except Exception:
                    source_id = ""
                if source_id:
                    return source_id
        return "video_ingest_receiver"

    def video_frame_source_status(self) -> Dict[str, Any]:
        """返回平台端侧推流接收状态，供持续感知运行时做实时性门禁。"""

        return self._video_frame_source_status()

    @staticmethod
    def _camera_info_from_video_frame(image_frame):
        from contracts.image import CameraInfo

        metadata = dict(getattr(image_frame, "metadata", {}) or {})
        camera_info_payload = metadata.get("camera_info")
        if isinstance(camera_info_payload, CameraInfo):
            return camera_info_payload
        if isinstance(camera_info_payload, dict):
            try:
                return CameraInfo(**camera_info_payload)
            except Exception:
                return None
        return None

    def _get_localization_provider_optional(self) -> Optional[LocalizationProvider]:
        providers = getattr(self._localization_provider_owner, "providers", None)
        if providers is None or not isinstance(providers, LocalizationProvider):
            return None
        if not providers.is_available():
            return None
        return providers

    def _read_live_pose_and_frame_tree(self):
        """从实时位姿门面（几何运行时 providers）取探索期实时位姿与坐标树。

        鸭子类型，与 PerceptionVideoRuntime._read_live_pose 同源：优先
        get_exploration_pose()（探索建图时导航后端 map→base 实时 TF），回退
        get_current_pose()（定位会话后端，仅重定位就绪后有值）；frame_tree 走
        get_frame_tree()。任一失败返回 None，禁兜底假位姿。
        """

        from contracts.geometry import Pose

        provider = self._live_pose_provider
        if provider is None:
            return None, None
        pose = None
        for getter_name in ("get_exploration_pose", "get_current_pose"):
            getter = getattr(provider, getter_name, None)
            if not callable(getter):
                continue
            try:
                candidate = getter()
            except Exception:
                continue
            if isinstance(candidate, Pose):
                pose = candidate
                break
        frame_tree = None
        frame_tree_getter = getattr(provider, "get_frame_tree", None)
        if callable(frame_tree_getter):
            try:
                frame_tree = frame_tree_getter()
            except Exception:
                frame_tree = None
        return pose, frame_tree

    def _get_point_cloud_provider_optional(self) -> Optional[PointCloudProvider]:
        providers = getattr(self._provider_owner, "providers", None)
        if providers is None or not isinstance(providers, PointCloudProvider):
            return None
        if not providers.is_available():
            return None
        return providers

    def _augment_detection_geometry(
        self,
        detection_bundle,
        *,
        image_frame,
        camera_info,
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

        point_cloud_frame = self._latest_stream_payload("pointcloud.frame")
        if not _has_usable_point_cloud(point_cloud_frame):
            point_cloud_frame = self._get_latest_point_cloud_optional()

        frame_tree = self._latest_stream_payload("localization.frame_tree")
        current_pose = self._latest_stream_payload("localization.pose")
        if frame_tree is None or current_pose is None:
            localization_provider = self._get_localization_provider_optional()
            if localization_provider is not None:
                if frame_tree is None:
                    frame_tree_getter = getattr(localization_provider, "get_frame_tree", None)
                    if callable(frame_tree_getter):
                        frame_tree = frame_tree_getter()
                if current_pose is None:
                    pose_getter = getattr(localization_provider, "get_current_pose", None)
                    if callable(pose_getter):
                        current_pose = pose_getter()
        # P0-1：探索建图阶段定位会话尚未重定位就绪，上面流/会话源的 current_pose 恒 None，
        # 导致 map_frame_id=None、support_points_map 永不生成、下游丢失最高优先级
        # map_point_samples 投影路径。与 PerceptionVideoRuntime._read_live_pose 同源回退到
        # 实时位姿门面（导航后端 map→base 实时 TF）。frame_tree 同源缺失时一并回退。
        if current_pose is None or frame_tree is None:
            live_pose, live_frame_tree = self._read_live_pose_and_frame_tree()
            if current_pose is None and live_pose is not None:
                current_pose = live_pose
            if frame_tree is None and live_frame_tree is not None:
                frame_tree = live_frame_tree

        detections_2d, geometry_metadata = self._geometry_augmentor.augment_detections(
            detection_bundle.detections_2d,
            image_frame=image_frame,
            camera_info=camera_info,
            point_cloud=point_cloud_frame,
            frame_tree=frame_tree,
            current_pose=current_pose,
        )
        if not _has_usable_point_cloud(point_cloud_frame):
            geometry_metadata = {
                **dict(geometry_metadata),
                "geometry_projection_status": "pointcloud_unavailable",
                "geometry_degraded_reason": "pointcloud_unavailable",
            }
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
        degraded_reason = str(geometry_metadata.get("geometry_degraded_reason") or "")
        if support_count <= 0 and _has_usable_point_cloud(point_cloud_frame):
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
            return None

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
