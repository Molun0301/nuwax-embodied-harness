from __future__ import annotations

from pathlib import Path
import time
from types import SimpleNamespace

import pytest

from contracts.artifacts import ArtifactRetentionPolicy
from contracts.geometry import FrameTree, Pose, Quaternion, Transform, Vector3
from contracts.image import CameraInfo, ImageEncoding, ImageFrame
from contracts.maps import SemanticMap, SemanticRegion
from contracts.pointcloud import PointCloudFrame
from contracts.runtime_views import LocalizationSnapshot, MapSnapshot
from contracts.perception import TrackState
from contracts.runtime_views import SceneObjectSummary, SceneSummary
from core import EventBus, StateNamespace, StateStore
from gateways.artifacts import LocalArtifactStore
from gateways.errors import GatewayError
from providers.image import ImageProvider
from services import ArtifactService
from services.perception import (
    BBoxOnlySegmentationBackend,
    Basic2DTrackerBackend,
    DetectionAttributesSegmentationBackend,
    DetectorPipeline,
    DisabledSegmentationBackend,
    HybridSceneDescriptionBackend,
    MetadataDrivenDetectorBackend,
    OpenAICompatibleVisionSceneDescriptionBackend,
    PerceptionService,
    PerceptionVideoRuntime,
    SceneDescriptionBackend,
    SceneDescriptionBackendSpec,
    SegmentationBackend,
    SegmentationBackendSpec,
    SegmentationBundle,
    SegmentationCandidate,
    SimpleSceneDescriptionBackend,
    TrackLifecyclePolicy,
    UltralyticsYoloDetectorBackend,
)
from typing import Optional, Tuple


class _ProviderOwner:
    """测试用提供器宿主。"""

    def __init__(self, providers: ImageProvider) -> None:
        self.providers = providers


class UsbCameraBundle(ImageProvider):
    """模拟 USB 相机输入源。"""

    provider_name = "usb_camera_bundle"
    provider_version = "0.1.0"

    def is_available(self) -> bool:
        return True

    def capture_image(self, camera_id: Optional[str] = None) -> ImageFrame:
        camera = camera_id or "front_camera"
        return ImageFrame(
            camera_id=camera,
            frame_id=f"world/test/{camera}",
            width_px=320,
            height_px=240,
            encoding=ImageEncoding.JPEG,
            data=b"usb-image",
            metadata={
                "source_kind": "usb",
                "detections_2d": [
                    {
                        "label": "person",
                        "score": 0.95,
                        "bbox": {"x_px": 20, "y_px": 30, "width_px": 80, "height_px": 120},
                    }
                ],
            },
        )

    def get_camera_info(self, camera_id: Optional[str] = None) -> CameraInfo:
        return CameraInfo(
            camera_id=camera_id or "front_camera",
            frame_id=f"world/test/{camera_id or 'front_camera'}",
            width_px=320,
            height_px=240,
            fx=120.0,
            fy=120.0,
            cx=160.0,
            cy=120.0,
        )


class RtspCameraBundle(ImageProvider):
    """模拟 RTSP 相机输入源。"""

    provider_name = "rtsp_camera_bundle"
    provider_version = "0.1.0"

    def is_available(self) -> bool:
        return True

    def capture_image(self, camera_id: Optional[str] = None) -> ImageFrame:
        camera = camera_id or "front_camera"
        return ImageFrame(
            camera_id=camera,
            frame_id=f"world/test/{camera}",
            width_px=320,
            height_px=240,
            encoding=ImageEncoding.JPEG,
            data=b"rtsp-image",
            metadata={
                "source_kind": "rtsp",
                "detections_2d": [
                    {
                        "label": "person",
                        "score": 0.95,
                        "bbox": {"x_px": 20, "y_px": 30, "width_px": 80, "height_px": 120},
                    }
                ],
            },
        )

    def get_camera_info(self, camera_id: Optional[str] = None) -> CameraInfo:
        return CameraInfo(
            camera_id=camera_id or "front_camera",
            frame_id=f"world/test/{camera_id or 'front_camera'}",
            width_px=320,
            height_px=240,
            fx=120.0,
            fy=120.0,
            cx=160.0,
            cy=120.0,
        )


class PointCloudGeometryBundle(ImageProvider):
    """模拟同时提供图像、定位和点云的输入源。"""

    provider_name = "pointcloud_geometry_bundle"
    provider_version = "0.1.0"

    def is_available(self) -> bool:
        return True

    def capture_image(self, camera_id: Optional[str] = None) -> ImageFrame:
        camera = camera_id or "front_camera"
        return ImageFrame(
            camera_id=camera,
            frame_id="world/test/camera_front",
            width_px=100,
            height_px=100,
            encoding=ImageEncoding.JPEG,
            data=b"geometry-image",
            metadata={
                "detections_2d": [
                    {
                        "label": "bottle",
                        "score": 0.91,
                        "bbox": {"x_px": 45, "y_px": 45, "width_px": 10, "height_px": 10},
                    }
                ],
            },
        )

    def get_camera_info(self, camera_id: Optional[str] = None) -> CameraInfo:
        return CameraInfo(
            camera_id=camera_id or "front_camera",
            frame_id="world/test/camera_front",
            width_px=100,
            height_px=100,
            fx=100.0,
            fy=100.0,
            cx=50.0,
            cy=50.0,
        )

    def get_current_pose(self) -> Optional[Pose]:
        return Pose(
            frame_id="map",
            position=Vector3(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(w=1.0),
        )

    def get_frame_tree(self) -> Optional[FrameTree]:
        return FrameTree(
            root_frame_id="map",
            transforms=[
                Transform(
                    parent_frame_id="map",
                    child_frame_id="world/test/camera_front",
                    translation=Vector3(x=0.0, y=0.0, z=0.0),
                    rotation=Quaternion(w=1.0),
                )
            ],
        )

    def get_latest_point_cloud(self) -> Optional[PointCloudFrame]:
        return PointCloudFrame(
            frame_id="map",
            point_count=4,
            points=[
                Vector3(x=0.0, y=0.0, z=1.0),
                Vector3(x=0.02, y=0.0, z=1.0),
                Vector3(x=0.0, y=0.02, z=1.0),
                Vector3(x=0.8, y=0.8, z=1.0),
            ],
            metadata={"source_topic": "/test/cloud"},
        )


def _build_perception_service(
    tmp_path: Path,
    provider_owner,
    *,
    tracker_backend: Optional[Basic2DTrackerBackend] = None,
    detector_pipeline: Optional[DetectorPipeline] = None,
    scene_description_backend: Optional[SceneDescriptionBackend] = None,
    segmentation_backend: Optional[SegmentationBackend] = None,
) -> Tuple[PerceptionService, StateStore]:
    artifact_service = ArtifactService(
        LocalArtifactStore(str(tmp_path / "artifacts"), "http://testserver"),
        retention_policy=ArtifactRetentionPolicy(
            retention_days=7,
            max_count=20,
            max_total_bytes=1024 * 1024,
            cleanup_batch_size=10,
        ),
    )
    state_store = StateStore()
    service = PerceptionService(
        provider_owner=provider_owner,
        artifact_service=artifact_service,
        state_store=state_store,
        detector_pipeline=detector_pipeline or DetectorPipeline((MetadataDrivenDetectorBackend(),)),
        tracker_backend=tracker_backend,
        scene_description_backend=scene_description_backend,
        segmentation_backend=segmentation_backend,
        event_bus=EventBus(),
        history_limit=10,
        pipeline_name="test_perception_pipeline",
    )
    return service, state_store


def _assert_pipeline_observability(metadata: dict, *, segmentation_status: str) -> None:
    stage_duration_ms = metadata["stage_duration_ms"]
    stage_status = metadata["stage_status"]
    for stage_name in ("detector", "segmentation", "geometry", "tracker", "scene_description", "total"):
        assert stage_name in stage_duration_ms
        assert stage_duration_ms[stage_name] >= 0
    assert stage_status["detector"] == "completed"
    assert stage_status["segmentation"] == segmentation_status
    assert stage_status["geometry"] == "completed"
    assert stage_status["tracker"] == "completed"
    assert stage_status["scene_description"] == "completed"


class _FakeYoloDetectorBackend(UltralyticsYoloDetectorBackend):
    """通过伪造推理结果验证 YOLO 标准化逻辑。"""

    def _predict(self, image_bgr):
        del image_bgr

        class _Boxes:
            xyxy = [[18.0, 24.0, 90.0, 160.0]]
            conf = [0.93]
            cls = [0]

        class _Masks:
            xy = [
                [
                    [20.0, 30.0],
                    [80.0, 30.0],
                    [80.0, 150.0],
                    [20.0, 150.0],
                ]
            ]
            predicted_iou = [0.71]
            stability_score = [0.82]
            scores = [0.89]

        class _Result:
            boxes = _Boxes()
            masks = _Masks()
            names = {0: "Person"}

        return [_Result()]


class _FakeYoloNoMaskDetectorBackend(UltralyticsYoloDetectorBackend):
    def _predict(self, image_bgr):
        del image_bgr

        class _Boxes:
            xyxy = [[10.0, 12.0, 60.0, 72.0]]
            conf = [0.81]
            cls = [0]

        class _Result:
            boxes = _Boxes()
            masks = None
            names = {0: "Bottle"}

        return [_Result()]


class _FakeYoloLowQualityMaskDetectorBackend(UltralyticsYoloDetectorBackend):
    def _predict(self, image_bgr):
        del image_bgr

        class _Boxes:
            xyxy = [[12.0, 16.0, 92.0, 116.0]]
            conf = [0.76]
            cls = [1]

        class _Masks:
            xy = [[[12.0, 16.0], [92.0, 16.0], [92.0, 116.0], [12.0, 116.0]]]
            predicted_iou = [0.19]
            stability_score = [0.24]
            scores = [0.22]

        class _Result:
            boxes = _Boxes()
            masks = _Masks()
            names = {1: "Chair"}

        return [_Result()]


class _FakeYoloInvalidMaskDetectorBackend(UltralyticsYoloDetectorBackend):
    def _predict(self, image_bgr):
        del image_bgr

        class _Boxes:
            xyxy = [[14.0, 18.0, 44.0, 58.0]]
            conf = [0.67]
            cls = [2]

        class _Masks:
            xy = [[[14.0, 18.0], [44.0, 58.0]]]
            predicted_iou = [0.91]
            stability_score = [0.92]
            scores = [0.93]

        class _Result:
            boxes = _Boxes()
            masks = _Masks()
            names = {2: "Box"}

        return [_Result()]



class _RecordingSegmentationBackend(SegmentationBackend):
    def __init__(self) -> None:
        self.spec = SegmentationBackendSpec(name="recording_segmentation", backend_kind="recording")
        self.calls = []

    def segment(self, image_frame, *, detections_2d=(), camera_info=None) -> SegmentationBundle:
        self.calls.append((image_frame, detections_2d, camera_info))
        return SegmentationBundle(
            segments=(
                SegmentationCandidate(
                    label=detections_2d[0].label,
                    score=0.88,
                    metadata={
                        "detection_index": 0,
                        "mask_center_px": [60.0, 70.0],
                        "mask_polygon_px": [[40.0, 50.0], [80.0, 50.0], [80.0, 90.0], [40.0, 90.0]],
                        "segmentation_score": 0.88,
                    },
                ),
            ),
            metadata={"backend": "recording_segmentation", "segment_count": 1, "source": "test"},
        )


class _StubCloudSceneBackend(SceneDescriptionBackend):
    """测试用云端语义后端。"""

    def __init__(self) -> None:
        self.spec = SceneDescriptionBackendSpec(
            name="stub_cloud_scene",
            backend_kind="openai_compatible_multimodal",
        )

    def describe(
        self,
        *,
        camera_id: str,
        detections_2d,
        detections_3d,
        tracks,
        image_frame,
        camera_info=None,
    ) -> SceneSummary:
        del detections_2d, detections_3d, tracks, image_frame, camera_info
        return SceneSummary(
            headline="画面里有一名人员，靠近红色补给架。",
            details=["人员位于画面左侧。", "补给架位于人员右侧。"],
            objects=[
                SceneObjectSummary(
                    label="person",
                    count=1,
                    tracked_count=0,
                    max_score=0.82,
                    camera_ids=[camera_id],
                    track_ids=[],
                    attributes={"display_name_zh": "人员", "relative_location": "left"},
                ),
                SceneObjectSummary(
                    label="charger",
                    count=1,
                    tracked_count=0,
                    max_score=0.74,
                    camera_ids=[camera_id],
                    track_ids=[],
                    attributes={"display_name_zh": "补给架", "color": "red"},
                ),
            ],
            detection_count=1,
            active_track_count=1,
            metadata={
                "semantic_tags": ["红色补给架", "人员附近"],
                "semantic_relations": ["person_beside_charger"],
                "visual_labels": ["person", "charger"],
                "cloud_vision_model": "fake-vision-model",
            },
        )


class _StaticLocalizationService:
    """测试用定位服务。"""

    def __init__(self) -> None:
        self.pose = Pose(
            frame_id="map",
            position=Vector3(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(w=1.0),
        )

    def get_latest_snapshot(self):
        return LocalizationSnapshot(
            source_name="test_localization",
            current_pose=self.pose,
        )

    def is_available(self) -> bool:
        return True

    def refresh(self):
        return self.get_latest_snapshot()


class _RefreshingLocalizationService(_StaticLocalizationService):
    """只有显式刷新后才暴露最新位姿的定位服务。"""

    def __init__(self) -> None:
        super().__init__()
        self.live_pose = self.pose
        self._cached_pose = self.pose
        self.refresh_count = 0

    def get_latest_snapshot(self):
        return LocalizationSnapshot(
            source_name="test_localization",
            current_pose=self._cached_pose,
        )

    def refresh(self):
        self.refresh_count += 1
        self._cached_pose = self.live_pose
        return self.get_latest_snapshot()


class _RefreshingMappingService:
    """只有显式刷新后才暴露最新地图快照的映射服务。"""

    def __init__(self) -> None:
        self.live_snapshot = self._build_snapshot(region_id="traversable_1")
        self._cached_snapshot = self.live_snapshot
        self.refresh_count = 0

    def get_latest_snapshot(self):
        return self._cached_snapshot

    def is_available(self) -> bool:
        return True

    def refresh(self):
        self.refresh_count += 1
        self._cached_snapshot = self.live_snapshot
        return self._cached_snapshot

    def _build_snapshot(self, *, region_id: str) -> MapSnapshot:
        return MapSnapshot(
            source_name="test_map",
            version_id=f"mapv_test_{region_id}",
            revision=1,
            semantic_map=SemanticMap(
                map_id="semantic_test_map",
                frame_id="map",
                regions=[
                    SemanticRegion(
                        region_id=region_id,
                        label="traversable",
                        centroid=Pose(
                            frame_id="map",
                            position=Vector3(x=0.0, y=0.0, z=0.0),
                            orientation=Quaternion(w=1.0),
                        ),
                    )
                ],
            ),
        )


class _FakeMemoryService:
    """测试用记忆服务。"""

    def __init__(self, *, should_raise: bool = False, enabled: bool = True) -> None:
        self.calls = []
        self.should_raise = should_raise
        self.enabled = enabled

    def is_enabled(self) -> bool:
        return self.enabled

    def remember_current_scene(self, **kwargs):
        self.calls.append(dict(kwargs))
        if self.should_raise:
            raise GatewayError("测试记忆写入失败。")
        return {"ok": True}


def test_perception_service_requires_standard_imageframe_and_camerainfo(tmp_path: Path) -> None:
    """感知服务必须只接受标准 ImageFrame/CameraInfo。"""

    provider_owner = _ProviderOwner(UsbCameraBundle())
    service, state_store = _build_perception_service(tmp_path, provider_owner)
    image_frame = provider_owner.providers.capture_image("front_camera")
    camera_info = provider_owner.providers.get_camera_info("front_camera")

    context = service.process_image(image_frame, camera_info, requested_by="tester")
    latest_entry = state_store.read(StateNamespace.PERCEPTION, "front_camera")

    assert context.camera_info is not None
    assert context.camera_info.camera_id == "front_camera"
    assert latest_entry is not None
    assert latest_entry.value.camera_id == "front_camera"

    with pytest.raises(TypeError):
        service.process_image("not-an-image-frame", camera_info)


def test_perception_service_exposes_segmentation_backend_metadata(tmp_path: Path) -> None:
    """感知结果元数据应暴露已装配的分割后端信息。"""

    provider_owner = _ProviderOwner(UsbCameraBundle())
    segmentation_backend = DisabledSegmentationBackend(
        configured_enabled=True,
        configured_backend_name="sam2",
        configured_backend_kind="sam2",
        reason="not_implemented",
    )
    service, _ = _build_perception_service(
        tmp_path,
        provider_owner,
        segmentation_backend=segmentation_backend,
    )

    context = service.perceive_current_scene(camera_id="front_camera", requested_by="tester", store_artifact=False)

    assert service.segmentation_backend is segmentation_backend
    _assert_pipeline_observability(context.observation.metadata, segmentation_status="disabled")
    _assert_pipeline_observability(context.metadata, segmentation_status="disabled")
    assert context.pipeline_name == "test_perception_pipeline"
    assert context.observation.metadata["pipeline_name"] == "test_perception_pipeline"
    assert context.observation.metadata["provider_fallback_used"] is False
    assert context.observation.metadata["provider_fallback_reason"] == "not_implemented"
    assert context.observation.metadata["detector_provider"]["capability"] == "detection"
    assert context.observation.metadata["detector_provider"]["provider_name"] == "metadata_detector"
    assert context.observation.metadata["detector_provider"]["capabilities"]["detect"] is True
    assert context.observation.metadata["detector_backend_metadata"]["provider"] == context.observation.metadata["detector_provider"]
    assert context.metadata["detector_provider"]["capability"] == "detection"
    assert context.metadata["detector_provider"]["provider_name"] == "metadata_detector"
    assert context.metadata["detector_provider"]["capabilities"]["detect"] is True
    assert context.metadata["detector_backend_metadata"]["provider"] == context.metadata["detector_provider"]
    scene_event = service._event_bus.replay(after_cursor=0)[-1]
    assert scene_event.event_type == "perception.scene_ready"
    assert scene_event.payload["stage_status"]["segmentation"] == "disabled"
    assert scene_event.payload["stage_duration_ms"]["total"] >= 0
    assert scene_event.payload["detector_provider"]["provider_name"] == "metadata_detector"
    assert scene_event.payload["detector_provider"]["capabilities"]["detect"] is True
    assert scene_event.metadata["duration_ms"] == scene_event.payload["stage_duration_ms"]["total"]
    assert scene_event.metadata["detector_provider"]["capability"] == "detection"
    assert scene_event.metadata["detector_provider"]["provider_name"] == "metadata_detector"
    assert context.observation.metadata["segmentation_backend"] == "segmentation_disabled"
    assert context.observation.metadata["segmentation_backend_kind"] == "disabled_segmentation"
    assert context.observation.metadata["segmentation_count"] == 0
    assert context.observation.metadata["segmentation_metadata"]["reason"] == "not_implemented"
    assert context.observation.metadata["segmentation_backend_metadata"]["configured_enabled"] is True
    assert context.observation.metadata["segmentation_backend_metadata"]["configured_backend_name"] == "sam2"
    assert context.observation.metadata["segmentation_backend_metadata"]["configured_backend_kind"] == "sam2"
    assert context.observation.metadata["segmentation_backend_metadata"]["reason"] == "not_implemented"
    assert context.metadata["segmentation_backend"] == "segmentation_disabled"
    assert context.metadata["segmentation_backend_kind"] == "disabled_segmentation"
    assert context.metadata["segmentation_count"] == 0
    assert context.metadata["segmentation_metadata"]["reason"] == "not_implemented"
    assert context.metadata["segmentation_backend_metadata"]["configured_enabled"] is True
    assert context.metadata["segmentation_backend_metadata"]["configured_backend_name"] == "sam2"
    assert context.metadata["segmentation_backend_metadata"]["configured_backend_kind"] == "sam2"
    assert context.metadata["segmentation_backend_metadata"]["reason"] == "not_implemented"


def test_perception_service_calls_segmentation_backend_and_merges_attributes(tmp_path: Path) -> None:
    """感知主链应调用分割后端并把分割属性合入检测结果。"""

    provider_owner = _ProviderOwner(UsbCameraBundle())
    segmentation_backend = _RecordingSegmentationBackend()
    service, _ = _build_perception_service(
        tmp_path,
        provider_owner,
        segmentation_backend=segmentation_backend,
    )

    context = service.perceive_current_scene(camera_id="front_camera", requested_by="tester", store_artifact=False)
    detection = context.observation.detections_2d[0]

    assert segmentation_backend.calls
    _assert_pipeline_observability(context.observation.metadata, segmentation_status="completed")
    _assert_pipeline_observability(context.metadata, segmentation_status="completed")
    assert context.observation.metadata["pipeline_name"] == "test_perception_pipeline"
    assert context.observation.metadata["provider_fallback_used"] is False
    scene_event = service._event_bus.replay(after_cursor=0)[-1]
    assert scene_event.event_type == "perception.scene_ready"
    assert scene_event.payload["stage_status"]["segmentation"] == "completed"
    assert scene_event.payload["provider_fallback_used"] is False
    _, detections_2d, camera_info = segmentation_backend.calls[0]
    assert detections_2d[0].label == "person"
    assert camera_info is not None
    assert camera_info.camera_id == "front_camera"
    assert context.observation.metadata["segmentation_count"] == 1
    assert context.observation.metadata["segmentation_metadata"]["source"] == "test"
    assert context.metadata["segmentation_count"] == 1
    assert detection.attributes["mask_center_px"] == [60.0, 70.0]
    assert detection.attributes["mask_polygon_px"] == [[40.0, 50.0], [80.0, 50.0], [80.0, 90.0], [40.0, 90.0]]
    assert detection.attributes["segmentation_score"] == pytest.approx(0.88)
    assert detection.attributes["segmentation_backend"] == "recording_segmentation"
    assert detection.attributes["segmentation_backend_kind"] == "recording"


def test_perception_service_merges_bbox_only_segmentation(tmp_path: Path) -> None:
    """bbox-only 分割应合入 detection.segmentation 与检测属性。"""

    provider_owner = _ProviderOwner(UsbCameraBundle())
    segmentation_backend = BBoxOnlySegmentationBackend(
        metadata={"provider_kind": "bbox_only_fallback", "provider_name": "bbox_only_fallback"},
    )
    service, _ = _build_perception_service(
        tmp_path,
        provider_owner,
        segmentation_backend=segmentation_backend,
    )

    context = service.perceive_current_scene(camera_id="front_camera", requested_by="tester", store_artifact=False)
    detection = context.observation.detections_2d[0]

    _assert_pipeline_observability(context.observation.metadata, segmentation_status="fallback")
    _assert_pipeline_observability(context.metadata, segmentation_status="fallback")
    assert context.observation.metadata["provider_fallback_used"] is True
    assert context.observation.metadata["provider_fallback_reason"] == "bbox_only_fallback"
    assert context.observation.metadata["segmentation_backend"] == "bbox_only_segmentation"
    assert context.observation.metadata["segmentation_backend_kind"] == "bbox_only_fallback"
    assert context.observation.metadata["segmentation_count"] == 1
    assert context.observation.metadata["segmentation_metadata"]["source"] == "bbox_only_fallback"
    assert context.observation.metadata["segmentation_backend_metadata"]["provider_kind"] == "bbox_only_fallback"
    assert detection.segmentation is not None
    assert detection.segmentation.source == "bbox_only_fallback"
    assert detection.segmentation.polygon_px == ((20.0, 30.0), (100.0, 30.0), (100.0, 150.0), (20.0, 150.0))
    assert detection.segmentation.centroid_uv == (60.0, 90.0)
    assert detection.segmentation.area_ratio == pytest.approx(0.125)
    assert detection.attributes["mask_polygon_px"] == [[20.0, 30.0], [100.0, 30.0], [100.0, 150.0], [20.0, 150.0]]
    assert detection.attributes["segmentation_polygon_px"] == [[20.0, 30.0], [100.0, 30.0], [100.0, 150.0], [20.0, 150.0]]
    assert detection.attributes["mask_center_px"] == [60.0, 90.0]
    assert detection.attributes["segmentation_available"] is False
    assert detection.attributes["segmentation_status"] == "bbox_only_fallback"
    assert detection.attributes["segmentation_source"] == "bbox_only_fallback"
    assert detection.attributes["segmentation_backend"] == "bbox_only_segmentation"
    assert detection.attributes["segmentation_backend_kind"] == "bbox_only_fallback"


def test_detection_attributes_segmentation_backend_collects_segments() -> None:
    """属性分割后端应从检测属性中生成分割候选。"""

    image_frame = ImageFrame(
        camera_id="front_camera",
        frame_id="world/test/front_camera",
        width_px=160,
        height_px=120,
        encoding=ImageEncoding.JPEG,
        data=b"image",
        metadata={
            "detections_2d": [
                {
                    "label": "person",
                    "score": 0.95,
                    "bbox": {"x_px": 20, "y_px": 30, "width_px": 80, "height_px": 120},
                    "attributes": {
                        "mask_center_px": [60.0, 70.0],
                        "mask_polygon_px": [[40.0, 50.0], [80.0, 50.0], [80.0, 90.0], [40.0, 90.0]],
                    },
                }
            ],
        },
    )
    detection = MetadataDrivenDetectorBackend().detect(image_frame).detections_2d[0]

    bundle = DetectionAttributesSegmentationBackend().segment(image_frame, detections_2d=(detection,))
    segment = bundle.segments[0]

    assert bundle.metadata["segment_count"] == 1
    assert bundle.metadata["detection_count"] == 1
    assert segment.label == "person"
    assert segment.metadata["detection_index"] == 0
    assert segment.metadata["frame_id"] == "world/test/front_camera"
    assert segment.metadata["camera_id"] == "front_camera"
    assert segment.metadata["bbox_xyxy"] == (20.0, 30.0, 100.0, 120.0)
    assert segment.metadata["mask_center_px"] == [60.0, 70.0]
    assert segment.metadata["mask_polygon_px"] == [[40.0, 50.0], [80.0, 50.0], [80.0, 90.0], [40.0, 90.0]]


def test_bbox_only_segmentation_backend_builds_structured_segments() -> None:
    """bbox-only 分割后端应从检测框生成结构化最低可用分割。"""

    image_frame = ImageFrame(
        camera_id="front_camera",
        frame_id="world/test/front_camera",
        width_px=200,
        height_px=100,
        encoding=ImageEncoding.JPEG,
        data=b"image",
        metadata={
            "detections_2d": [
                {
                    "label": "box",
                    "score": 0.9,
                    "bbox": {"x_px": 20, "y_px": 10, "width_px": 80, "height_px": 40},
                }
            ],
        },
    )
    detection = MetadataDrivenDetectorBackend().detect(image_frame).detections_2d[0]

    bundle = BBoxOnlySegmentationBackend().segment(image_frame, detections_2d=(detection,))
    segment = bundle.segments[0]

    assert bundle.metadata["source"] == "bbox_only_fallback"
    assert bundle.metadata["segment_count"] == 1
    assert segment.label == "box"
    assert segment.segmentation is not None
    assert segment.segmentation.polygon_px == ((20.0, 10.0), (100.0, 10.0), (100.0, 50.0), (20.0, 50.0))
    assert segment.segmentation.centroid_uv == (60.0, 30.0)
    assert segment.segmentation.bbox_xyxy == (20.0, 10.0, 100.0, 50.0)
    assert segment.segmentation.area_px == pytest.approx(3200.0)
    assert segment.segmentation.area_ratio == pytest.approx(0.16)
    assert segment.segmentation.source == "bbox_only_fallback"
    assert segment.metadata["segmentation_available"] is False
    assert segment.metadata["segmentation_status"] == "bbox_only_fallback"


def test_metadata_detector_normalizes_detection_results() -> None:
    """检测标准化应修正标签、分数与越界框。"""

    image_frame = ImageFrame(
        camera_id="front_camera",
        frame_id="world/test/front_camera",
        width_px=100,
        height_px=80,
        encoding=ImageEncoding.JPEG,
        data=b"image",
        metadata={
            "detections_2d": [
                {
                    "label": " Person ",
                    "score": 1.7,
                    "bbox": {"x_px": -10, "y_px": -5, "width_px": 200, "height_px": 120},
                }
            ]
        },
    )

    bundle = MetadataDrivenDetectorBackend().detect(image_frame)
    detection = bundle.detections_2d[0]

    assert detection.label == "person"
    assert detection.score == pytest.approx(1.0)
    assert detection.bbox.x_px == pytest.approx(0.0)
    assert detection.bbox.y_px == pytest.approx(0.0)
    assert detection.bbox.width_px == pytest.approx(100.0)
    assert detection.bbox.height_px == pytest.approx(80.0)


def test_yolo_detector_backend_normalizes_ultralytics_results() -> None:
    """YOLO 后端应把 Ultralytics 结果规整成标准 Detection2D。"""

    image_frame = ImageFrame(
        camera_id="front_camera",
        frame_id="world/test/front_camera",
        width_px=160,
        height_px=120,
        encoding=ImageEncoding.BGR8,
        data=bytes([0, 0, 0] * 160 * 120),
    )

    backend = _FakeYoloDetectorBackend(weights="yolo26n.pt")
    bundle = backend.detect(image_frame)

    assert bundle.detections_2d
    detection = bundle.detections_2d[0]
    assert detection.label == "person"
    assert detection.score == pytest.approx(0.93)
    assert detection.attributes["model_name"] == "yolo26n.pt"
    assert detection.attributes["class_id"] == 0
    assert detection.attributes["grounding_score"] == pytest.approx(0.93)
    assert detection.attributes["mask_polygon_px"] == [[20.0, 30.0], [80.0, 30.0], [80.0, 150.0], [20.0, 150.0]]
    assert detection.attributes["segmentation_polygon_px"] == [[20.0, 30.0], [80.0, 30.0], [80.0, 150.0], [20.0, 150.0]]
    assert detection.attributes["mask_center_px"] == [50.0, 90.0]
    assert detection.attributes["centroid_uv"] == [50.0, 90.0]
    assert detection.attributes["bbox_xyxy"] == [20.0, 30.0, 80.0, 150.0]
    assert detection.attributes["crop_box"] == [20.0, 30.0, 80.0, 150.0]
    assert detection.attributes["area_px"] == pytest.approx(7200.0)
    assert detection.attributes["area_ratio"] == pytest.approx(0.375)
    assert detection.attributes["contour"] == {"point_count": 4}
    assert detection.attributes["segmentation_score"] == pytest.approx(0.89)
    assert detection.attributes["predicted_iou"] == pytest.approx(0.71)
    assert detection.attributes["stability_score"] == pytest.approx(0.82)
    assert detection.attributes["segmentation_available"] is True
    assert detection.attributes["segmentation_status"] == "segmentation_preferred"
    assert detection.attributes["segmentation_priority"] == "segmentation_over_bbox"
    assert detection.attributes["segmentation_source"] == "ultralytics_yolo_masks"
    assert detection.segmentation is not None
    assert detection.segmentation.polygon_px == ((20.0, 30.0), (80.0, 30.0), (80.0, 150.0), (20.0, 150.0))
    assert detection.segmentation.centroid_uv == (50.0, 90.0)
    assert detection.segmentation.bbox_xyxy == (20.0, 30.0, 80.0, 150.0)
    assert detection.segmentation.crop_box == (20.0, 30.0, 80.0, 150.0)
    assert detection.segmentation.area_px == pytest.approx(7200.0)
    assert detection.segmentation.area_ratio == pytest.approx(0.375)
    assert detection.segmentation.contour == {"point_count": 4}
    assert detection.segmentation.quality.score == pytest.approx(0.89)
    assert detection.segmentation.quality.predicted_iou == pytest.approx(0.71)
    assert detection.segmentation.quality.stability_score == pytest.approx(0.82)
    assert detection.segmentation.attributes["grounding_score"] == pytest.approx(0.93)


def test_yolo_detector_backend_preserves_bbox_only_when_masks_missing() -> None:
    """YOLO 无 masks 时应保持 bbox-only 降级语义。"""

    image_frame = ImageFrame(
        camera_id="front_camera",
        frame_id="world/test/front_camera",
        width_px=160,
        height_px=120,
        encoding=ImageEncoding.BGR8,
        data=bytes([0, 0, 0] * 160 * 120),
    )

    backend = _FakeYoloNoMaskDetectorBackend(weights="yolo26n.pt")
    detection = backend.detect(image_frame).detections_2d[0]

    assert detection.label == "bottle"
    assert detection.segmentation is None
    assert detection.attributes["grounding_score"] == pytest.approx(0.81)
    assert detection.attributes["segmentation_available"] is False
    assert detection.attributes["segmentation_status"] == "bbox_only"
    assert detection.attributes["segmentation_priority"] == "bbox_only"
    assert "mask_polygon_px" not in detection.attributes
    assert "segmentation_score" not in detection.attributes


def test_yolo_detector_backend_keeps_low_quality_mask_scores() -> None:
    """YOLO masks 质量较低时仍应保留结构化质量字段供下游决策。"""

    image_frame = ImageFrame(
        camera_id="front_camera",
        frame_id="world/test/front_camera",
        width_px=160,
        height_px=120,
        encoding=ImageEncoding.BGR8,
        data=bytes([0, 0, 0] * 160 * 120),
    )

    backend = _FakeYoloLowQualityMaskDetectorBackend(weights="yolo26n.pt")
    detection = backend.detect(image_frame).detections_2d[0]

    assert detection.label == "chair"
    assert detection.segmentation is not None
    assert detection.attributes["segmentation_available"] is True
    assert detection.attributes["segmentation_status"] == "segmentation_preferred"
    assert detection.attributes["segmentation_priority"] == "segmentation_over_bbox"
    assert detection.attributes["segmentation_score"] == pytest.approx(0.22)
    assert detection.attributes["predicted_iou"] == pytest.approx(0.19)
    assert detection.attributes["stability_score"] == pytest.approx(0.24)
    assert detection.segmentation.quality.score == pytest.approx(0.22)
    assert detection.segmentation.quality.predicted_iou == pytest.approx(0.19)
    assert detection.segmentation.quality.stability_score == pytest.approx(0.24)


def test_yolo_detector_backend_falls_back_for_invalid_mask_polygon() -> None:
    """YOLO mask polygon 无效时不应生成结构化 segmentation。"""

    image_frame = ImageFrame(
        camera_id="front_camera",
        frame_id="world/test/front_camera",
        width_px=160,
        height_px=120,
        encoding=ImageEncoding.BGR8,
        data=bytes([0, 0, 0] * 160 * 120),
    )

    backend = _FakeYoloInvalidMaskDetectorBackend(weights="yolo26n.pt")
    detection = backend.detect(image_frame).detections_2d[0]

    assert detection.label == "box"
    assert detection.segmentation is None
    assert detection.attributes["grounding_score"] == pytest.approx(0.67)
    assert detection.attributes["segmentation_available"] is False
    assert detection.attributes["segmentation_status"] == "invalid_polygon"
    assert detection.attributes["segmentation_priority"] == "bbox_only"
    assert "predicted_iou" not in detection.attributes
    assert "stability_score" not in detection.attributes


def test_yolo_detector_backend_prefers_engine_when_available(tmp_path: Path) -> None:
    """当 TensorRT 引擎存在时，YOLO 后端应优先加载 .engine。"""

    weights_path = tmp_path / "yolo26n.pt"
    engine_path = tmp_path / "yolo26n.engine"
    weights_path.write_bytes(b"fake-pt")
    engine_path.write_bytes(b"fake-engine")

    backend = UltralyticsYoloDetectorBackend(
        weights=str(weights_path),
        engine_path=str(engine_path),
        runtime_preference="tensorrt",
    )
    resolved_source, backend_kind = backend._resolve_model_source()

    assert resolved_source == str(engine_path)
    assert backend_kind == "ultralytics_yolo_tensorrt"


def test_perception_service_enriches_detection_geometry_from_pointcloud(tmp_path: Path) -> None:
    """感知服务应基于点云为检测框补充几何样本。"""

    provider_owner = _ProviderOwner(PointCloudGeometryBundle())
    service, _ = _build_perception_service(tmp_path, provider_owner)

    context = service.perceive_current_scene(camera_id="front_camera", requested_by="tester", store_artifact=False)
    detection = context.observation.detections_2d[0]

    assert detection.label == "bottle"
    assert detection.attributes["camera_points"]["frame_id"] == "world/test/camera_front"
    assert detection.attributes["map_points"]["frame_id"] == "map"
    assert len(detection.attributes["camera_points"]["points"]) == 3
    assert len(detection.attributes["map_points"]["points"]) == 3
    assert detection.attributes["depth_m"] == pytest.approx(1.0)
    assert detection.attributes["geometry_support_point_count"] == 3
    assert context.observation.metadata["geometry_augmented_count"] == 1
    assert context.observation.metadata["geometry_point_cloud_source_topic"] == "/test/cloud"


def test_perception_video_runtime_uses_keyframes_and_burst_without_memory_flooding(tmp_path: Path) -> None:
    """关键帧应写记忆，burst 复核默认不应把每帧都写入记忆。"""

    provider_owner = _ProviderOwner(UsbCameraBundle())
    service, state_store = _build_perception_service(tmp_path, provider_owner)
    localization_service = _StaticLocalizationService()
    memory_service = _FakeMemoryService()
    runtime = PerceptionVideoRuntime(
        perception_service=service,
        state_store=state_store,
        localization_service=localization_service,
        mapping_service=None,
        memory_service=memory_service,
        enabled=True,
        auto_start=False,
        camera_id="front_camera",
        interval_sec=0.1,
        store_artifact=False,
        keyframe_min_interval_sec=0.1,
        keyframe_max_interval_sec=2.0,
        keyframe_translation_threshold_m=0.3,
        keyframe_yaw_threshold_deg=10.0,
        remember_keyframes=True,
        remember_min_interval_sec=0.1,
        burst_frame_count=2,
        burst_interval_sec=0.01,
    )

    first_message = runtime.run_once()
    assert first_message is not None
    assert len(memory_service.calls) == 1

    skipped_message = runtime.run_once()
    assert skipped_message is None
    assert len(memory_service.calls) == 1

    time.sleep(0.12)
    localization_service.pose = Pose(
        frame_id="map",
        position=Vector3(x=0.5, y=0.0, z=0.0),
        orientation=Quaternion(w=1.0),
    )
    second_message = runtime.run_once()
    assert second_message is not None
    assert len(memory_service.calls) == 2

    burst_messages = runtime.run_burst(frame_count=2, interval_sec=0.01, remember_result=False)
    assert len(burst_messages) == 2
    assert len(memory_service.calls) == 2

    status = runtime.get_status()
    assert status.processed_frames >= 4
    assert status.metadata["camera_id"] == "front_camera"


def test_perception_video_runtime_ignores_memory_write_failure(tmp_path: Path) -> None:
    """关键帧记忆写入失败时，不应打断视频感知主流程。"""

    provider_owner = _ProviderOwner(UsbCameraBundle())
    service, state_store = _build_perception_service(tmp_path, provider_owner)
    localization_service = _StaticLocalizationService()
    memory_service = _FakeMemoryService(should_raise=True)
    runtime = PerceptionVideoRuntime(
        perception_service=service,
        state_store=state_store,
        localization_service=localization_service,
        mapping_service=None,
        memory_service=memory_service,
        enabled=True,
        auto_start=False,
        camera_id="front_camera",
        interval_sec=0.1,
        store_artifact=False,
        keyframe_min_interval_sec=0.1,
        keyframe_max_interval_sec=2.0,
        keyframe_translation_threshold_m=0.3,
        keyframe_yaw_threshold_deg=10.0,
        remember_keyframes=True,
        remember_min_interval_sec=0.1,
        burst_frame_count=2,
        burst_interval_sec=0.01,
    )

    message = runtime.run_once()
    status = runtime.get_status()

    assert message is not None
    assert len(memory_service.calls) == 1
    assert status.processed_frames == 1
    assert status.metadata["memory_written"] is False


def test_perception_video_runtime_suppresses_repeated_calls_when_pose_is_static(tmp_path: Path) -> None:
    """静止状态下即使超过最大间隔，也不应重复触发关键帧。"""

    provider_owner = _ProviderOwner(UsbCameraBundle())
    service, state_store = _build_perception_service(tmp_path, provider_owner)
    localization_service = _StaticLocalizationService()
    memory_service = _FakeMemoryService()
    runtime = PerceptionVideoRuntime(
        perception_service=service,
        state_store=state_store,
        localization_service=localization_service,
        mapping_service=None,
        memory_service=memory_service,
        enabled=True,
        auto_start=False,
        camera_id="front_camera",
        interval_sec=0.1,
        store_artifact=False,
        keyframe_min_interval_sec=0.05,
        keyframe_max_interval_sec=0.2,
        keyframe_translation_threshold_m=0.3,
        keyframe_yaw_threshold_deg=10.0,
        remember_keyframes=True,
        remember_min_interval_sec=0.05,
        burst_frame_count=2,
        burst_interval_sec=0.01,
    )

    first_message = runtime.run_once()
    assert first_message is not None
    assert len(memory_service.calls) == 1

    time.sleep(0.22)
    second_message = runtime.run_once()
    status = runtime.get_status()

    assert second_message is None
    assert len(memory_service.calls) == 1
    assert status.metadata["last_skip_reason"] == "stationary_pose_hold"
    assert status.metadata["stationary_suppressed"] is True


def test_perception_video_runtime_refreshes_live_localization_before_keyframe_decision(tmp_path: Path) -> None:
    """关键帧判定应读取刷新后的实时位姿，而不是长期复用旧快照。"""

    provider_owner = _ProviderOwner(UsbCameraBundle())
    service, state_store = _build_perception_service(tmp_path, provider_owner)
    localization_service = _RefreshingLocalizationService()
    memory_service = _FakeMemoryService()
    runtime = PerceptionVideoRuntime(
        perception_service=service,
        state_store=state_store,
        localization_service=localization_service,
        mapping_service=None,
        memory_service=memory_service,
        enabled=True,
        auto_start=False,
        camera_id="front_camera",
        interval_sec=0.1,
        store_artifact=False,
        keyframe_min_interval_sec=0.05,
        keyframe_max_interval_sec=2.0,
        keyframe_translation_threshold_m=0.3,
        keyframe_yaw_threshold_deg=10.0,
        remember_keyframes=True,
        remember_min_interval_sec=0.05,
        burst_frame_count=2,
        burst_interval_sec=0.01,
    )

    first_message = runtime.run_once()
    assert first_message is not None
    assert len(memory_service.calls) == 1

    time.sleep(0.08)
    localization_service.live_pose = Pose(
        frame_id="map",
        position=Vector3(x=0.6, y=0.0, z=0.0),
        orientation=Quaternion(w=1.0),
    )

    second_message = runtime.run_once()

    assert second_message is not None
    assert len(memory_service.calls) == 2
    assert localization_service.refresh_count >= 2


def test_perception_video_runtime_suppresses_anchor_change_when_pose_is_static(tmp_path: Path) -> None:
    """静止状态下即使语义区域标识变化，也不应重复触发视觉与记忆调用。"""

    provider_owner = _ProviderOwner(UsbCameraBundle())
    service, state_store = _build_perception_service(tmp_path, provider_owner)
    localization_service = _StaticLocalizationService()
    mapping_service = _RefreshingMappingService()
    memory_service = _FakeMemoryService()
    runtime = PerceptionVideoRuntime(
        perception_service=service,
        state_store=state_store,
        localization_service=localization_service,
        mapping_service=mapping_service,
        memory_service=memory_service,
        enabled=True,
        auto_start=False,
        camera_id="front_camera",
        interval_sec=0.1,
        store_artifact=False,
        keyframe_min_interval_sec=0.05,
        keyframe_max_interval_sec=2.0,
        keyframe_translation_threshold_m=0.3,
        keyframe_yaw_threshold_deg=10.0,
        remember_keyframes=True,
        remember_min_interval_sec=0.05,
        burst_frame_count=2,
        burst_interval_sec=0.01,
    )

    first_message = runtime.run_once()
    assert first_message is not None
    assert len(memory_service.calls) == 1

    time.sleep(0.08)
    mapping_service.live_snapshot = mapping_service._build_snapshot(region_id="traversable_2")

    second_message = runtime.run_once()
    status = runtime.get_status()

    assert second_message is None
    assert len(memory_service.calls) == 1
    assert status.processed_frames == 1
    assert status.metadata["last_skip_reason"] == "stationary_anchor_hold"
    assert status.metadata["stationary_suppressed"] is True


def test_perception_video_runtime_auto_start_waits_until_memory_enabled(tmp_path: Path) -> None:
    """自动拉起持续视觉时，应等待记忆库显式启用后再启动。"""

    provider_owner = _ProviderOwner(UsbCameraBundle())
    service, state_store = _build_perception_service(tmp_path, provider_owner)
    localization_service = _StaticLocalizationService()
    memory_service = _FakeMemoryService(enabled=False)
    runtime = PerceptionVideoRuntime(
        perception_service=service,
        state_store=state_store,
        localization_service=localization_service,
        mapping_service=None,
        memory_service=memory_service,
        enabled=True,
        auto_start=True,
        camera_id="front_camera",
        interval_sec=0.1,
        store_artifact=False,
        keyframe_min_interval_sec=0.05,
        keyframe_max_interval_sec=0.2,
        keyframe_translation_threshold_m=0.3,
        keyframe_yaw_threshold_deg=10.0,
        remember_keyframes=True,
        remember_min_interval_sec=0.05,
        burst_frame_count=2,
        burst_interval_sec=0.01,
    )

    assert runtime.should_auto_start() is False

    memory_service.enabled = True

    assert runtime.should_auto_start() is True


def test_openai_compatible_vision_backend_uses_responses_api_for_ark() -> None:
    """豆包方舟兼容地址应自动切到 responses 接口。"""

    image_provider = UsbCameraBundle()
    image_frame = image_provider.capture_image("front_camera")
    camera_info = image_provider.get_camera_info("front_camera")
    response_payload = (
        '{"headline":"看见一名人员和补给架。","details":["人员站在补给架旁边。"],'
        '"tags":["人员","补给架"],"relations":["beside_dock"],'
        '"objects":[{"label":"person","count":1,"confidence":0.92,"attributes":{"display_name_zh":"人员"}},'
        '{"label":"charger","count":1,"confidence":0.81,"attributes":{"display_name_zh":"补给架"}}]}'
    )

    class _FakeResponsesApi:
        def __init__(self) -> None:
            self.calls = []

        def create(self, **kwargs):
            self.calls.append(kwargs)
            return SimpleNamespace(output_text=response_payload)

    class _FailingChatCompletions:
        def create(self, **kwargs):
            raise AssertionError("不应走 chat.completions")

    responses_api = _FakeResponsesApi()
    backend = OpenAICompatibleVisionSceneDescriptionBackend(
        model="doubao-seed-2-0-pro-260215",
        base_url="https://ark.cn-beijing.volces.com/api/v3",
        api_style="auto",
    )
    backend._client = SimpleNamespace(
        responses=responses_api,
        chat=SimpleNamespace(completions=_FailingChatCompletions()),
    )

    summary = backend.describe(
        camera_id="front_camera",
        detections_2d=(),
        detections_3d=(),
        tracks=(),
        image_frame=image_frame,
        camera_info=camera_info,
    )

    assert responses_api.calls
    request_payload = responses_api.calls[0]
    assert request_payload["input"][0]["content"][0]["type"] == "input_image"
    assert request_payload["input"][0]["content"][1]["type"] == "input_text"
    assert summary.headline == "看见一名人员和补给架。"
    assert summary.metadata["cloud_vision_api_style"] == "responses"
    assert "person" in summary.metadata["visual_labels"]


def test_openai_compatible_vision_backend_falls_back_to_raw_http_responses_for_old_sdk(monkeypatch) -> None:
    """旧版 openai SDK 缺少 responses 接口时，应回退到原生 HTTP 请求。"""

    image_provider = UsbCameraBundle()
    image_frame = image_provider.capture_image("front_camera")
    camera_info = image_provider.get_camera_info("front_camera")
    backend = OpenAICompatibleVisionSceneDescriptionBackend(
        model="doubao-seed-2-0-pro-260215",
        base_url="https://ark.cn-beijing.volces.com/api/v3",
        api_style="auto",
    )
    backend._client = SimpleNamespace(
        chat=SimpleNamespace(completions=SimpleNamespace(create=lambda **kwargs: (_ for _ in ()).throw(AssertionError("不应走 chat"))))
    )

    recorded_payloads = []

    def _fake_post_responses_request(request_payload):
        recorded_payloads.append(request_payload)
        return {
            "output": [
                {
                    "content": [
                        {
                            "type": "output_text",
                            "text": (
                                '{"headline":"看见前方有办公桌。","details":["办公桌位于画面中央。"],'
                                '"tags":["办公桌"],"relations":["center"],'
                                '"objects":[{"label":"desk","count":1,"confidence":0.88,'
                                '"attributes":{"display_name_zh":"办公桌"}}]}'
                            ),
                        }
                    ]
                }
            ]
        }

    monkeypatch.setattr(backend, "_post_responses_request", _fake_post_responses_request)

    summary = backend.describe(
        camera_id="front_camera",
        detections_2d=(),
        detections_3d=(),
        tracks=(),
        image_frame=image_frame,
        camera_info=camera_info,
    )

    assert recorded_payloads
    assert recorded_payloads[0]["input"][0]["content"][0]["type"] == "input_image"
    assert summary.headline == "看见前方有办公桌。"
    assert summary.metadata["cloud_vision_api_style"] == "responses"
    assert "desk" in summary.metadata["visual_labels"]


def test_openai_compatible_vision_backend_keeps_chat_completions_for_openai() -> None:
    """标准 OpenAI 兼容地址仍应走 chat.completions 接口。"""

    image_provider = UsbCameraBundle()
    image_frame = image_provider.capture_image("front_camera")
    camera_info = image_provider.get_camera_info("front_camera")
    response_payload = (
        '{"headline":"看见一名人员。","details":["人员位于画面中央。"],'
        '"tags":["人员"],"relations":["center"],'
        '"objects":[{"label":"person","count":1,"confidence":0.95,"attributes":{"display_name_zh":"人员"}}]}'
    )

    class _FakeChatCompletions:
        def __init__(self) -> None:
            self.calls = []

        def create(self, **kwargs):
            self.calls.append(kwargs)
            return SimpleNamespace(
                choices=[
                    SimpleNamespace(
                        message=SimpleNamespace(content=response_payload),
                    )
                ]
            )

    class _FailingResponsesApi:
        def create(self, **kwargs):
            raise AssertionError("不应走 responses")

    chat_completions = _FakeChatCompletions()
    backend = OpenAICompatibleVisionSceneDescriptionBackend(
        model="gpt-4o-mini",
        base_url="https://api.openai.com/v1",
        api_style="auto",
    )
    backend._client = SimpleNamespace(
        responses=_FailingResponsesApi(),
        chat=SimpleNamespace(completions=chat_completions),
    )

    summary = backend.describe(
        camera_id="front_camera",
        detections_2d=(),
        detections_3d=(),
        tracks=(),
        image_frame=image_frame,
        camera_info=camera_info,
    )

    assert chat_completions.calls
    assert summary.headline == "看见一名人员。"
    assert summary.metadata["cloud_vision_api_style"] == "chat_completions"


def test_openai_compatible_vision_backend_retries_when_json_is_truncated() -> None:
    """云端响应因 max_tokens 截断时，应自动放大配额后重试一次。"""

    image_provider = UsbCameraBundle()
    image_frame = image_provider.capture_image("front_camera")
    camera_info = image_provider.get_camera_info("front_camera")
    truncated_payload = (
        '{"headline":"办公室内有人坐在工位前。","details":["右侧有人正在使用电脑。"],'
        '"tags":["人员","办公桌"],"relations":["person_near_desk"],'
        '"objects":[{"label":"person","count":1,"confidence":0.95,"attributes":{"display_name_zh":"人员"}},'
    )
    complete_payload = (
        '{"headline":"办公室内有人坐在工位前。","details":["右侧有人正在使用电脑。"],'
        '"tags":["人员","办公桌"],"relations":["person_near_desk"],'
        '"objects":[{"label":"person","count":1,"confidence":0.95,"attributes":{"display_name_zh":"人员"}},'
        '{"label":"desk","count":1,"confidence":0.9,"attributes":{"display_name_zh":"办公桌"}}]}'
    )

    class _FakeChatCompletions:
        def __init__(self) -> None:
            self.calls = []

        def create(self, **kwargs):
            self.calls.append(kwargs)
            payload = truncated_payload if len(self.calls) == 1 else complete_payload
            return SimpleNamespace(
                choices=[
                    SimpleNamespace(
                        message=SimpleNamespace(content=payload),
                    )
                ]
            )

    chat_completions = _FakeChatCompletions()
    backend = OpenAICompatibleVisionSceneDescriptionBackend(
        model="qwen3-vl-flash",
        base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
        api_style="chat_completions",
        max_tokens=700,
    )
    backend._client = SimpleNamespace(
        chat=SimpleNamespace(completions=chat_completions),
    )

    summary = backend.describe(
        camera_id="front_camera",
        detections_2d=(),
        detections_3d=(),
        tracks=(),
        image_frame=image_frame,
        camera_info=camera_info,
    )

    assert len(chat_completions.calls) == 2
    assert chat_completions.calls[0]["max_tokens"] == 700
    assert chat_completions.calls[1]["max_tokens"] == 1400
    assert summary.headline == "办公室内有人坐在工位前。"
    assert "person" in summary.metadata["visual_labels"]


def test_tracker_lifecycle_flows_from_tentative_to_removed(tmp_path: Path) -> None:
    """首版二维跟踪器应明确 tentative/tracked/lost/removed 语义。"""

    tracker_backend = Basic2DTrackerBackend(
        lifecycle_policy=TrackLifecyclePolicy(
            iou_match_threshold=0.3,
            tentative_hits_required=2,
            lost_ttl_frames=1,
        )
    )
    service, _ = _build_perception_service(tmp_path, _ProviderOwner(UsbCameraBundle()), tracker_backend=tracker_backend)

    def _frame(with_detection: bool) -> ImageFrame:
        metadata = {}
        if with_detection:
            metadata["detections_2d"] = [
                {
                    "label": "person",
                    "score": 0.92,
                    "bbox": {"x_px": 20, "y_px": 30, "width_px": 80, "height_px": 120},
                }
            ]
        return ImageFrame(
            camera_id="front_camera",
            frame_id="world/test/front_camera",
            width_px=320,
            height_px=240,
            encoding=ImageEncoding.JPEG,
            data=b"frame",
            metadata=metadata,
        )

    camera_info = CameraInfo(
        camera_id="front_camera",
        frame_id="world/test/front_camera",
        width_px=320,
        height_px=240,
        fx=120.0,
        fy=120.0,
        cx=160.0,
        cy=120.0,
    )

    first = service.process_image(_frame(True), camera_info)
    second = service.process_image(_frame(True), camera_info)
    third = service.process_image(_frame(False), camera_info)
    fourth = service.process_image(_frame(False), camera_info)

    assert first.observation.tracks[0].state == TrackState.TENTATIVE
    assert second.observation.tracks[0].state == TrackState.TRACKED
    assert third.observation.tracks[0].state == TrackState.LOST
    assert fourth.observation.tracks[0].state == TrackState.REMOVED


def test_source_switch_does_not_change_top_level_perception_logic(tmp_path: Path) -> None:
    """切换相机来源时，上层感知逻辑不应改动。"""

    usb_service, _ = _build_perception_service(tmp_path / "usb", _ProviderOwner(UsbCameraBundle()))
    rtsp_service, _ = _build_perception_service(tmp_path / "rtsp", _ProviderOwner(RtspCameraBundle()))

    usb_context = usb_service.perceive_current_scene(camera_id="front_camera", requested_by="tester")
    rtsp_context = rtsp_service.perceive_current_scene(camera_id="front_camera", requested_by="tester")

    assert usb_context.scene_summary.headline == rtsp_context.scene_summary.headline
    assert usb_context.observation.detections_2d[0].label == "person"
    assert rtsp_context.observation.detections_2d[0].track_id is not None
    assert usb_context.detector_backend == rtsp_context.detector_backend


def test_hybrid_scene_backend_merges_local_and_cloud_semantics(tmp_path: Path) -> None:
    """本地检测与云端语义识别应合并为统一场景摘要。"""

    provider_owner = _ProviderOwner(UsbCameraBundle())
    hybrid_backend = HybridSceneDescriptionBackend(
        local_backend=SimpleSceneDescriptionBackend(),
        cloud_backend=_StubCloudSceneBackend(),
    )
    service, _ = _build_perception_service(
        tmp_path,
        provider_owner,
        scene_description_backend=hybrid_backend,
    )

    context = service.perceive_current_scene(camera_id="front_camera", requested_by="tester")

    assert context.scene_summary.headline == "画面里有一名人员，靠近红色补给架。"
    assert "红色补给架" in context.scene_summary.metadata["semantic_tags"]
    assert "person_beside_charger" in context.scene_summary.metadata["semantic_relations"]
    assert context.scene_summary.metadata["scene_provider"]["provider_name"] == "hybrid_scene"
    assert context.scene_summary.metadata["scene_provider"]["provider_kind"] == "hybrid"
    assert context.scene_summary.metadata["scene_backend_chain"] == [
        hybrid_backend.spec.metadata["local_backend"],
        hybrid_backend.spec.metadata["cloud_backend"],
    ]
    assert hybrid_backend.spec.metadata["provider"]["provider_name"] == "hybrid_scene"
    assert context.scene_summary.objects[0].label == "person"
