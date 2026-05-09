from __future__ import annotations

import base64
from collections import deque
from time import perf_counter

from contracts.events import RuntimeEventCategory
from contracts.naming import build_observation_id
from contracts.perception import Detection2D, Observation, SegmentationMask2D, Track, TrackState
from contracts.runtime_views import PerceptionContext
from core import EventBus, StateNamespace, StateStore, StreamBus
from gateways.errors import GatewayError
from providers import ImageProvider, LocalizationProvider, PointCloudProvider
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
from typing import Deque, Dict, List, Optional, Tuple


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
        self._history: Deque[PerceptionContext] = deque(maxlen=max(1, history_limit))
        self._latest_by_camera: Dict[str, PerceptionContext] = {}
        self._pipeline_name = pipeline_name

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
    ) -> PerceptionContext:
        """消费标准图像并产出检测、跟踪和场景摘要。"""

        from contracts.artifacts import ArtifactRef
        from contracts.image import CameraInfo, ImageFrame

        if not isinstance(image_frame, ImageFrame):
            raise TypeError("PerceptionService.process_image 仅接受标准 ImageFrame。")
        if camera_info is not None and not isinstance(camera_info, CameraInfo):
            raise TypeError("PerceptionService.process_image 的 camera_info 必须是 CameraInfo。")
        if image_artifact is not None and not isinstance(image_artifact, ArtifactRef):
            raise TypeError("PerceptionService.process_image 的 image_artifact 必须是 ArtifactRef。")

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
                **dict(detection_bundle.metadata),
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
            },
        )
        self._latest_by_camera[context.camera_id] = context
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
        return context

    def perceive_current_scene(
        self,
        *,
        camera_id: Optional[str] = None,
        requested_by: Optional[str] = None,
        detector_backend_name: Optional[str] = None,
        store_artifact: bool = True,
    ) -> PerceptionContext:
        """从当前图像提供器抓取一帧并完成完整感知。"""

        image_provider = self._get_image_provider()
        image_frame = image_provider.capture_image(camera_id)
        resolved_camera_id = image_frame.camera_id
        camera_info = image_provider.get_camera_info(resolved_camera_id)
        self._publish_video_frame(
            image_frame,
            source_name=image_provider.provider_name,
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
            source_name=image_provider.provider_name,
        )

    def describe_current_scene(
        self,
        *,
        camera_id: Optional[str] = None,
        refresh: bool = True,
        requested_by: Optional[str] = None,
        detector_backend_name: Optional[str] = None,
    ) -> PerceptionContext:
        """返回当前场景的结构化中文摘要。"""

        if refresh:
            return self.perceive_current_scene(
                camera_id=camera_id,
                requested_by=requested_by,
                detector_backend_name=detector_backend_name,
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
        image_provider = self._get_image_provider()
        image_frame = image_provider.capture_image(camera_id)
        self._publish_video_frame(
            image_frame,
            source_name=image_provider.provider_name,
        )

    def get_latest_perception(self, camera_id: Optional[str] = None) -> Optional[PerceptionContext]:
        """返回最新感知结果。"""

        if camera_id is not None:
            return self._latest_by_camera.get(camera_id)
        if not self._latest_by_camera:
            return None
        return max(self._latest_by_camera.values(), key=lambda item: item.timestamp)

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
            detections.append(detection.model_copy(update=update, deep=True))
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
                    deep=True,
                )
            )
        return tuple(assigned)

    def _publish_video_frame(self, image_frame, *, source_name: str) -> None:
        if self._stream_bus is None:
            return
        metadata = dict(image_frame.metadata or {})
        payload_bytes = len(image_frame.data or b"")
        data_uri = self._build_video_data_uri(image_frame)
        payload = {
            "available": bool(data_uri or image_frame.uri),
            "camera_id": image_frame.camera_id,
            "frame_id": image_frame.frame_id,
            "width_px": image_frame.width_px,
            "height_px": image_frame.height_px,
            "encoding": image_frame.encoding.value,
            "payload_bytes": payload_bytes,
            "metadata": dict(metadata),
        }
        if data_uri:
            payload["data_uri"] = data_uri
        elif image_frame.uri:
            payload["uri"] = image_frame.uri
        elif image_frame.artifact_id:
            payload["artifact_id"] = image_frame.artifact_id
        status = "available" if payload.get("data_uri") or payload.get("uri") else "unsupported"
        if status != "available":
            payload["message"] = f"浏览器暂不支持直接显示 {image_frame.encoding.value} 图像帧。"
        self._stream_bus.publish(
            "video.frame",
            source=source_name or "image_provider",
            payload=payload,
            frame_id=image_frame.frame_id,
            sequence=metadata.get("sequence") if isinstance(metadata.get("sequence"), int) else None,
            status=status,
            metadata={
                "camera_id": image_frame.camera_id,
                "encoding": image_frame.encoding.value,
                "payload_bytes": payload_bytes,
                "source_name": source_name or "image_provider",
                **metadata,
            },
        )

    def _build_video_data_uri(self, image_frame) -> Optional[str]:
        if not image_frame.data:
            return None
        mime_type = self._video_mime_type(image_frame.encoding)
        if mime_type is None:
            return None
        return "data:%s;base64,%s" % (
            mime_type,
            base64.b64encode(image_frame.data).decode("ascii"),
        )

    def _video_mime_type(self, encoding) -> Optional[str]:
        value = str(getattr(encoding, "value", encoding)).lower()
        if value == "jpeg":
            return "image/jpeg"
        if value == "png":
            return "image/png"
        return None

    def _get_image_provider(self) -> ImageProvider:
        providers = getattr(self._provider_owner, "providers", None)
        if providers is None or not isinstance(providers, ImageProvider):
            raise GatewayError("当前机器人入口未提供图像提供器。")
        if not providers.is_available():
            raise GatewayError("当前图像提供器暂不可用。")
        return providers

    def _get_localization_provider_optional(self) -> Optional[LocalizationProvider]:
        providers = getattr(self._localization_provider_owner, "providers", None)
        if providers is None or not isinstance(providers, LocalizationProvider):
            return None
        if not providers.is_available():
            return None
        return providers

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

        point_cloud_frame = self._latest_stream_payload("pointcloud.frame")
        if point_cloud_frame is None or not getattr(point_cloud_frame, "points", None):
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

        detections_2d, geometry_metadata = self._geometry_augmentor.augment_detections(
            detection_bundle.detections_2d,
            image_frame=image_frame,
            camera_info=camera_info,
            point_cloud=point_cloud_frame,
            frame_tree=frame_tree,
            current_pose=current_pose,
        )
        if point_cloud_frame is None or not getattr(point_cloud_frame, "points", None):
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
        if support_count <= 0 and point_cloud_frame is not None and getattr(point_cloud_frame, "points", None):
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
