from __future__ import annotations

from collections import deque

from contracts.events import RuntimeEventCategory
from contracts.naming import build_observation_id
from contracts.perception import Detection2D, Observation, Track, TrackState
from contracts.runtime_views import PerceptionContext
from core import EventBus, StateNamespace, StateStore
from gateways.errors import GatewayError
from providers import ImageProvider, LocalizationProvider, PointCloudProvider
from services.artifact_service import ArtifactService
from services.perception.base import SceneDescriptionBackend, TrackerBackend
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
        geometry_augmentor: Optional[DetectionGeometryAugmentor] = None,
        event_bus: Optional[EventBus] = None,
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
        self._geometry_augmentor = geometry_augmentor or DetectionGeometryAugmentor()
        self._event_bus = event_bus
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

        detection_bundle, detector_spec = self._detector_pipeline.detect(
            image_frame,
            camera_info,
            backend_name=detector_backend_name,
        )
        detection_bundle = self._augment_detection_geometry(
            detection_bundle,
            image_frame=image_frame,
            camera_info=camera_info,
        )
        tracks = self._tracker_backend.update(
            image_frame,
            detections_2d=detection_bundle.detections_2d,
            detections_3d=detection_bundle.detections_3d,
            camera_info=camera_info,
        )
        detections_2d = self._attach_track_ids(detection_bundle.detections_2d, tracks)
        scene_summary = self._scene_description_backend.describe(
            camera_id=image_frame.camera_id,
            detections_2d=detections_2d,
            detections_3d=detection_bundle.detections_3d,
            tracks=tracks,
            image_frame=image_frame,
            camera_info=camera_info,
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
                "tracker_backend": self._tracker_backend.spec.name,
                "scene_description_backend": self._scene_description_backend.spec.name,
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
                "detector_backend_kind": detector_spec.backend_kind,
                "tracker_backend_kind": self._tracker_backend.spec.backend_kind,
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
                        "detection_count": len(detections_2d),
                        "track_count": len(tracks),
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
            assigned.append(
                detection.model_copy(
                    update={"track_id": matched_track_id or detection.track_id},
                    deep=True,
                )
            )
        return tuple(assigned)

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
            return detection_bundle
        point_cloud_provider = self._get_point_cloud_provider_optional()
        if point_cloud_provider is None:
            return detection_bundle
        point_cloud = point_cloud_provider.get_latest_point_cloud()
        if point_cloud is None or not point_cloud.points:
            return detection_bundle

        localization_provider = self._get_localization_provider_optional()
        frame_tree = localization_provider.get_frame_tree() if localization_provider is not None else None
        current_pose = localization_provider.get_current_pose() if localization_provider is not None else None
        detections_2d, geometry_metadata = self._geometry_augmentor.augment_detections(
            detection_bundle.detections_2d,
            image_frame=image_frame,
            camera_info=camera_info,
            point_cloud=point_cloud,
            frame_tree=frame_tree,
            current_pose=current_pose,
        )
        if detections_2d == detection_bundle.detections_2d and not geometry_metadata:
            return detection_bundle
        return detection_bundle.__class__(
            detections_2d=detections_2d,
            detections_3d=detection_bundle.detections_3d,
            metadata={
                **dict(detection_bundle.metadata),
                **geometry_metadata,
            },
        )

    def _normalize_camera_id(self, camera_id: str) -> str:
        normalized = []
        for char in camera_id.lower():
            if char.isalnum():
                normalized.append(char)
            else:
                normalized.append("_")
        value = "".join(normalized).strip("_")
        return value or "camera"
