from tests.conftest import create_memory_mapping_service
"""生产感知/制品/事件实现的定向测试；检测器为无模型单元夹具。"""
import threading
from types import SimpleNamespace

import pytest

from contracts.artifacts import ArtifactRetentionPolicy
from contracts.geometry import Pose, Vector3
from contracts.image import ImageFrame, ImageEncoding
from core import StateStore, EventBus, StreamBus
from services.artifact_service import ArtifactService
from services.artifact_store import LocalArtifactStore
from services.perception.base import DetectionBundle, DetectorBackendSpec
from services.perception.service import PerceptionService


class Detector:
    def detect(self, image_frame, camera_info=None, **kwargs):
        return DetectionBundle(detections_2d=()), DetectorBackendSpec(name="unit", backend_kind="unit")


@pytest.fixture
def setup(tmp_path):
    store = LocalArtifactStore(str(tmp_path / "artifacts"), "http://unit.invalid")
    artifacts = ArtifactService(store, retention_policy=ArtifactRetentionPolicy())
    frame = ImageFrame(camera_id="front", frame_id="camera/front", width_px=1, height_px=1,
                       encoding=ImageEncoding.JPEG, data=b"unit-image")
    capture = {"library_name": "room", "activation_generation": 3,
               "observation_pose": Pose(frame_id="map", position=Vector3(x=2)),
               "map_snapshot": None, "localization_snapshot": None}
    calls = []
    def provider():
        calls.append(True)
        return capture
    event_bus = EventBus()
    service = PerceptionService(provider_owner=object(), artifact_service=artifacts,
        state_store=StateStore(), detector_pipeline=Detector(), event_bus=event_bus,
        stream_bus=StreamBus(), history_limit=1, spatial_context_provider=provider,
        video_frame_source=SimpleNamespace(get_latest_image=lambda camera_id=None: frame))
    yield service, capture, calls, frame, store, event_bus
    assert service.close()


def test_capture_is_frozen_before_model_wait_and_published_with_result(setup, monkeypatch):
    service, capture, calls, frame, store, events = setup
    entered, release = threading.Event(), threading.Event()
    detect = service._detector_pipeline.detect
    def blocked(*args, **kwargs):
        entered.set()
        assert release.wait(3)
        return detect(*args, **kwargs)
    monkeypatch.setattr(service._detector_pipeline, "detect", blocked)
    result, errors = [], []
    def run():
        try: result.append(service.perceive_current_scene())
        except BaseException as exc: errors.append(exc)
    worker = threading.Thread(target=run); worker.start()
    try:
        assert entered.wait(2)
        assert not service._observation_spatial_contexts
        capture["observation_pose"].position.x = 99
        capture["activation_generation"] = 4
    finally:
        release.set(); worker.join(3)
    assert not errors and not worker.is_alive()
    context = result[0]
    saved = service.get_observation_spatial_context(context.observation.observation_id)
    assert saved["observation_pose"].position.x == 2 and saved["activation_generation"] == 3
    assert len(calls) == 1 and context.image_artifact is not None
    saved["observation_pose"].position.x = 80
    assert service.get_observation_spatial_context(context.observation.observation_id)["observation_pose"].position.x == 2
    assert store.get_file_path(context.image_artifact.artifact_id).exists()


def test_snapshot_eviction_and_close_follow_result_owners(setup):
    service, capture, calls, frame, store, events = setup
    first = service.perceive_current_scene()
    second = service.perceive_current_scene()
    assert first.observation.observation_id != second.observation.observation_id
    assert service.get_observation_spatial_context(first.observation.observation_id) == {}
    assert service.get_observation_spatial_context(second.observation.observation_id)
    assert len(service._observation_spatial_contexts) == 1
    assert service.close()
    assert not service._observation_spatial_contexts
    assert not service._full_holds and not service._history_holds


@pytest.mark.parametrize("failure", ["detect", "state"])
def test_failure_before_publication_does_not_leave_snapshot_or_holders(setup, monkeypatch, failure):
    service, capture, calls, frame, store, events = setup
    def fail(*args, **kwargs): raise RuntimeError("injected failure")
    if failure == "detect": monkeypatch.setattr(service._detector_pipeline, "detect", fail)
    else: monkeypatch.setattr(service._state_store, "write", fail)
    with pytest.raises(RuntimeError, match="injected failure"):
        service.perceive_current_scene()
    assert not service._observation_spatial_contexts and service.get_latest_full_perception() is None
    assert not service._full_holds and not service._history_holds and service._active_operations == 0


def test_subscriber_observes_snapshot_outside_both_pipeline_locks(setup):
    service, capture, calls, frame, store, events = setup
    observed = []
    def receive(event):
        if event.event_type != "perception.scene_ready": return
        assert service._full_pipeline_lock.acquire(blocking=False)
        service._full_pipeline_lock.release()
        assert service._pipeline_lock.acquire(blocking=False)
        service._pipeline_lock.release()
        context = service.get_latest_full_perception()
        assert service.get_observation_spatial_context(context.observation.observation_id)["activation_generation"] == 3
        observed.append(True)
    events.subscribe(receive)
    service.perceive_current_scene()
    assert observed == [True]


def test_explicit_missing_spatial_snapshot_does_not_recapture_provider(setup):
    service, capture, calls, frame, store, events = setup
    context = service.process_image(frame, spatial_context={})
    assert not calls
    assert service.get_observation_spatial_context(context.observation.observation_id) == {}


def test_real_capture_to_memory_preserves_artifact_and_original_pose(setup):
    from contracts.runtime_views import MapSnapshot, LocalizationSnapshot
    from services.memory.service import MemoryService
    service, capture, calls, frame, store, events = setup
    pose = capture["observation_pose"]
    snapshot = MapSnapshot(map_name="room", source_name="unit", source_epoch="origin", update_sequence=1, frame_id="map")
    localization = LocalizationSnapshot(source_name="unit", current_pose=pose)
    memory = MemoryService(
        localization_service=SimpleNamespace(get_latest_snapshot=lambda: localization, get_active_session=lambda: None),
        mapping_service=create_memory_mapping_service(snapshot),
        observation_service=SimpleNamespace(get_latest_observation=lambda camera_id=None: None),
        perception_service=service, state_store=StateStore(), artifact_store=store,
        text_embedding_model="disabled", image_embedding_model="disabled")
    memory.activate_memory_library(library_name="room")
    # 本组使用真实服务方法显式接线；生产网关接线属于后续独立合并范围。
    service._spatial_context_provider = memory.capture_observation_spatial_context
    try:
        context = service.perceive_current_scene()
        original_x = pose.position.x
        localization = localization.model_copy(update={"current_pose": Pose(frame_id="map", position=Vector3(x=20))})
        entry = memory.remember_current_scene()
        assert entry.pose.position.x == original_x
        assert entry.observation_id == context.observation.observation_id
        assert entry.artifact_ids == [context.image_artifact.artifact_id]
        assert service.close()
        assert store.get_file_path(entry.artifact_ids[0]).exists()
        assert memory._repository.load_semantic_memories()[0].pose == entry.pose
    finally:
        assert memory.close()
