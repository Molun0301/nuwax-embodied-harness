from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Optional, Tuple

from contracts.geometry import Pose, Vector3
from contracts.image import CameraInfo, ImageEncoding, ImageFrame
from contracts.perception import BoundingBox2D, Detection2D, Observation, Track
from contracts.runtime_views import PerceptionContext, SceneSummary
from core import StateStore, StreamBus
from services.perception.base import (
    DetectionBundle,
    DetectorBackendSpec,
    SceneDescriptionBackend,
    SceneDescriptionBackendSpec,
    SegmentationBackend,
    SegmentationBackendSpec,
)
from services.perception.service import PerceptionService
from services.perception.trackers import Basic2DTrackerBackend
from services.perception.video_runtime import PerceptionVideoRuntime


def _image_frame() -> ImageFrame:
    return ImageFrame(
        camera_id="front_camera",
        frame_id="world/test/camera_front",
        width_px=640,
        height_px=480,
        encoding=ImageEncoding.JPEG,
        data=b"fake-jpeg",
        metadata={
            "source_kind": "platform_video_ingest_stream",
            "camera_info": {
                "camera_id": "front_camera",
                "frame_id": "world/test/camera_front",
                "width_px": 640,
                "height_px": 480,
                "fx": 320.0,
                "fy": 320.0,
                "cx": 320.0,
                "cy": 240.0,
            },
        },
    )


def _detection(label: str = "chair", score: float = 0.9) -> Detection2D:
    return Detection2D(
        label=label,
        score=score,
        bbox=BoundingBox2D(x_px=10.0, y_px=20.0, width_px=30.0, height_px=40.0),
        camera_id="front_camera",
    )


class _VideoSource:
    def __init__(self) -> None:
        self.calls = 0

    def get_latest_image(self, camera_id: Optional[str] = None) -> ImageFrame:
        self.calls += 1
        assert camera_id in (None, "front_camera")
        return _image_frame()

    def status_snapshot(self) -> dict:
        return {"available": True, "connected": True}


class _DetectorPipeline:
    default_backend_name = "fake_detector"

    def __init__(self) -> None:
        self.calls = 0

    def detect(self, image_frame, camera_info=None, *, backend_name=None):
        self.calls += 1
        return (
            DetectionBundle(
                detections_2d=(_detection(),),
                metadata={
                    "backend": backend_name or "fake_detector",
                    "detector_provider": {"runtime": "fake_tensorrt"},
                },
            ),
            DetectorBackendSpec(
                name=backend_name or "fake_detector",
                backend_kind="fake_tensorrt",
                metadata={"provider": {"runtime": "fake_tensorrt"}},
            ),
        )


class _RaisingSegmentationBackend(SegmentationBackend):
    def __init__(self) -> None:
        self.spec = SegmentationBackendSpec(name="raising_segmentation", backend_kind="raising")
        self.calls = 0

    def segment(self, image_frame, *, detections_2d: Tuple[Detection2D, ...] = (), camera_info=None):
        self.calls += 1
        raise AssertionError("fast path must not call segmentation")


class _RaisingSceneDescriptionBackend(SceneDescriptionBackend):
    def __init__(self) -> None:
        self.spec = SceneDescriptionBackendSpec(name="raising_scene", backend_kind="raising")
        self.calls = 0

    def describe(
        self,
        *,
        camera_id: str,
        detections_2d: Tuple[Detection2D, ...],
        detections_3d=(),
        tracks: Tuple[Track, ...] = (),
        image_frame=None,
        camera_info: Optional[CameraInfo] = None,
    ) -> SceneSummary:
        self.calls += 1
        raise AssertionError("fast path must not call scene_description")


class _RaisingGeometryAugmentor:
    def __init__(self) -> None:
        self.calls = 0

    def augment_detections(self, *args, **kwargs):
        self.calls += 1
        raise AssertionError("fast path must not call geometry")


def _service(stream_bus: StreamBus) -> tuple[PerceptionService, _DetectorPipeline, _RaisingSegmentationBackend, _RaisingSceneDescriptionBackend, _RaisingGeometryAugmentor]:
    detector = _DetectorPipeline()
    segmentation = _RaisingSegmentationBackend()
    scene = _RaisingSceneDescriptionBackend()
    geometry = _RaisingGeometryAugmentor()
    service = PerceptionService(
        provider_owner=object(),
        artifact_service=object(),
        state_store=StateStore(),
        detector_pipeline=detector,
        tracker_backend=Basic2DTrackerBackend(),
        scene_description_backend=scene,
        segmentation_backend=segmentation,
        geometry_augmentor=geometry,
        stream_bus=stream_bus,
        video_frame_source=_VideoSource(),
        history_limit=10,
    )
    return service, detector, segmentation, scene, geometry


def test_fast_detection_path_publishes_overlay_without_slow_stages() -> None:
    stream_bus = StreamBus()
    service, detector, segmentation, scene, geometry = _service(stream_bus)

    context = service.detect_current_frame_fast(
        camera_id="front_camera",
        requested_by="unit_test",
        detector_backend_name="fake_detector",
    )

    assert detector.calls == 1
    assert segmentation.calls == 0
    assert scene.calls == 0
    assert geometry.calls == 0
    assert context.metadata["fast_path"] is True
    assert context.metadata["stage_status"]["segmentation"] == "skipped_fast_path"
    assert context.observation.detections_2d[0].track_id

    frame = stream_bus.latest_frame("video.detections")
    assert frame is not None
    payload = frame.payload
    assert payload["overlay_phase"] == "detector_ready"
    assert payload["detection_count"] == 1
    assert payload["stage_status"]["geometry"] == "skipped_fast_path"
    assert payload["detections_2d"][0]["track_id"] == context.observation.detections_2d[0].track_id


class _RuntimePerceptionService:
    def __init__(self) -> None:
        self.detector_pipeline = type("_Pipeline", (), {"default_backend_name": "fake_detector"})()
        self.detect_calls = 0
        self.perceive_calls = 0

    def video_frame_source_status(self) -> dict:
        return {"available": True}

    def publish_current_video_frame(self, **kwargs) -> None:
        return None

    def detect_current_frame_fast(self, **kwargs) -> PerceptionContext:
        self.detect_calls += 1
        detection = _detection().model_copy(update={"track_id": f"trk_{self.detect_calls}"}, deep=False)
        observation = Observation(
            observation_id=f"obs_front_camera_20260624T00000{self.detect_calls}Z_deadbeef",
            frame_id="world/test/camera_front",
            summary="fast",
            detections_2d=[detection],
            metadata={
                "camera_id": "front_camera",
                "width_px": 640,
                "height_px": 480,
                "stage_duration_ms": {"detector": 1.0, "tracker": 1.0, "total": 2.0},
                "stage_status": {
                    "detector": "completed",
                    "tracker": "completed",
                    "segmentation": "skipped_fast_path",
                    "geometry": "skipped_fast_path",
                    "scene_description": "skipped_fast_path",
                },
            },
        )
        return PerceptionContext(
            camera_id="front_camera",
            observation=observation,
            scene_summary=SceneSummary(
                headline="fast",
                detection_count=1,
                active_track_count=1,
                metadata={"summary_source": "fast_detection"},
            ),
            pipeline_name="test",
            detector_backend="fake_detector",
            tracker_backend="fake_tracker",
            metadata={
                "source_name": "video_ingest_receiver",
                "stage_duration_ms": observation.metadata["stage_duration_ms"],
                "stage_status": observation.metadata["stage_status"],
                "fast_path": True,
            },
        )

    def perceive_current_scene(self, **kwargs) -> PerceptionContext:
        self.perceive_calls += 1
        raise AssertionError("runtime fast tick must not call full perception")


class _PoseProvider:
    def __init__(self) -> None:
        self.pose = Pose(frame_id="map", position=Vector3(x=1.0, y=2.0, z=0.0))

    def get_current_pose(self) -> Pose:
        return self.pose


class _MemoryService:
    def __init__(self) -> None:
        self.ingest_calls = []

    def is_enabled(self) -> bool:
        return True

    def ingest_perception_observation(self, **kwargs):
        self.ingest_calls.append(kwargs)
        return (), (), []


def test_runtime_uses_fast_path_and_continuous_ingest_persist_false() -> None:
    perception_service = _RuntimePerceptionService()
    runtime = PerceptionVideoRuntime(
        perception_service=perception_service,  # type: ignore[arg-type]
        state_store=StateStore(),
        enabled=True,
        auto_start=False,
        camera_id="front_camera",
        interval_sec=0.5,
        keyframe_min_interval_sec=60.0,
        live_pose_provider=_PoseProvider(),
    )
    memory_service = _MemoryService()
    runtime.bind_memory_service(memory_service)

    first = runtime.run_once(allow_memory_write=False)
    second = runtime.run_once(allow_memory_write=False)

    assert first == "fast"
    assert second == "fast"
    assert perception_service.detect_calls == 2
    assert perception_service.perceive_calls == 0
    assert len(memory_service.ingest_calls) == 1
    assert memory_service.ingest_calls[0]["persist"] is False
    status = runtime.get_status()
    assert status.metadata["fast_path"] is True
    assert status.metadata["fast_processed_frames"] == 2
    assert status.metadata["continuous_ingest_count"] == 1
    assert status.metadata["detector_duration_ms"] == 1.0


class _BlockingMemoryService(_MemoryService):
    def __init__(self) -> None:
        super().__init__()
        self.remember_calls = 0
        self.started = threading.Event()
        self.release = threading.Event()

    def remember_current_scene(self, **kwargs):
        self.remember_calls += 1
        self.started.set()
        self.release.wait(timeout=1.5)
        return object()


def test_keyframe_memory_write_is_latest_only_background_work() -> None:
    perception_service = _RuntimePerceptionService()
    runtime = PerceptionVideoRuntime(
        perception_service=perception_service,  # type: ignore[arg-type]
        state_store=StateStore(),
        enabled=True,
        auto_start=False,
        camera_id="front_camera",
        interval_sec=0.5,
        keyframe_min_interval_sec=0.0,
        live_pose_provider=_PoseProvider(),
    )
    memory_service = _BlockingMemoryService()
    runtime.bind_memory_service(memory_service)

    started_at = time.monotonic()
    first = runtime.run_once(allow_memory_write=True)
    first_duration = time.monotonic() - started_at

    assert first == "fast"
    assert first_duration < 0.5
    assert memory_service.started.wait(timeout=1.0)

    second = runtime.run_once(force_keyframe=True, allow_memory_write=True)

    assert second == "fast"
    assert perception_service.detect_calls == 2
    assert perception_service.perceive_calls == 0
    status = runtime.get_status()
    assert status.metadata["memory_write_in_flight"] is True
    assert status.metadata["memory_write_scheduled_count"] == 2

    memory_service.release.set()


def test_monitor_web_keeps_existing_video_detections_subscription_without_new_visible_fields() -> None:
    html = Path("gateways/http/static/debug_panel.html").read_text(encoding="utf-8")

    assert '"video.detections"' in html
    assert "state.videoDetections" in html
    assert "overlay_phase" not in html
    assert "publish_interval_ms" not in html
    assert "fast_loop_duration_ms" not in html
