from tests.conftest import create_memory_mapping_service
"""标记当前位置与关联原照片保持独立空间语义，并沿同观察检索地点名称。"""
from datetime import timedelta
from types import SimpleNamespace
import pytest

from contracts.artifacts import ArtifactRef
from contracts.naming import build_artifact_id
from core.errors import GatewayError
from contracts.geometry import Pose, Vector3, Quaternion
from contracts.perception import Observation
from contracts.runtime_views import PerceptionContext, SceneSummary, MapSnapshot, LocalizationSnapshot
from core import StateStore
from services.memory.service import MemoryService


def make_map_snapshot(**kwargs):
    return MapSnapshot(update_sequence=1, **kwargs)


def _snapshot_memory_fixture():
    full = PerceptionContext(camera_id="front_camera",
        observation=Observation(observation_id="obs_camera_front_20260401T120000Z_deadbeef", frame_id="camera/front"),
        scene_summary=SceneSummary(headline="等候区域"), pipeline_name="test", detector_backend="test", tracker_backend="test")
    owner = SimpleNamespace(get_latest_snapshot=lambda: None, get_active_session=lambda: None,
                            refresh_current_session=lambda **kwargs: None, is_available=lambda: False)
    memory = MemoryService(localization_service=owner, mapping_service=create_memory_mapping_service(),
        observation_service=SimpleNamespace(get_latest_observation=lambda camera_id=None: None),
        perception_service=SimpleNamespace(get_latest_perception=lambda camera_id=None: full),
        state_store=StateStore(), text_embedding_model="disabled", image_embedding_model="disabled")
    memory.activate_memory_library(library_name="unit_test_library")
    pose = Pose(frame_id="map", position=Vector3(x=2), orientation=Quaternion(w=1))
    localization = LocalizationSnapshot(source_name="capture", current_pose=pose,
        metadata={"map_name":"unit_test_library", "session_id":"captured_session"})
    return memory, full, pose, localization


@pytest.fixture
def scene():
    memory, full, captured_pose, localization = _snapshot_memory_fixture()
    artifact=ArtifactRef(artifact_id=build_artifact_id('image'),kind='image',mime_type='image/jpeg',
        metadata={'camera_id':'front_camera','frame_timestamp':captured_pose.timestamp.isoformat()})
    full=full.model_copy(update={'image_artifact':artifact,'metadata':{
        **full.metadata,'frame_timestamp':captured_pose.timestamp.isoformat(),'frame_timestamp_source':'image_frame.timestamp'}})
    fast=full.model_copy(update={'image_artifact':None,'observation':full.observation.model_copy(update={'observation_id':'obs_front_20261010T100804Z_12345678'})})
    snapshot=make_map_snapshot(map_name='unit_test_library',source_name='capture',source_epoch='same-map-origin',frame_id='map')
    later=captured_pose.model_copy(update={'timestamp':captured_pose.timestamp+timedelta(seconds=4),
        'position':captured_pose.position.model_copy(update={'x':captured_pose.position.x+2})})
    current=localization.model_copy(update={'current_pose':later})
    captured={'library_name':'unit_test_library','activation_generation':memory._durable_index.activation_generation,'observation_pose':captured_pose,'map_snapshot':snapshot,'localization_snapshot':localization}
    memory._get_or_refresh_localization=lambda **kwargs:current
    memory._get_map_snapshot=lambda:snapshot
    memory._mapping_service = create_memory_mapping_service(snapshot)
    memory._perception_service=SimpleNamespace(get_latest_perception=lambda camera_id=None:fast,
        get_latest_full_perception=lambda camera_id=None:full,get_observation_spatial_context=lambda identifier:captured)
    memory._observation_service=SimpleNamespace(get_latest_observation=lambda camera_id=None:pytest.fail('有完整观察时不得混入另一次抓拍'))
    yield memory,full,captured_pose,later,captured
    assert memory.close()


def test_tag_retains_full_photo_capture_and_current_place_then_links_remember(scene):
    memory,full,original,current,captured=scene
    location,note=memory.tag_location('等候点')
    assert location.pose==current
    assert location.observation_id==full.observation.observation_id
    assert note.pose==original and note.artifact_ids==[full.image_artifact.artifact_id]
    assert note.metadata['frame_timestamp']==original.timestamp.isoformat()
    assert note.metadata['camera_id']=='front_camera'
    assert note.linked_location_id==location.location_id and location.memory_id==note.memory_id
    later=memory.remember_current_scene()
    assert later.linked_location_id==location.location_id
    assert later.pose==original and later.artifact_ids==note.artifact_ids and '等候点' in later.tags
    query=memory.query_location('等候点',similarity_threshold=0)
    match=next(m for m in query.matches if m.tagged_location and m.tagged_location.location_id==location.location_id)
    assert match.tagged_location.memory_id==later.memory_id
    assert match.metadata['linked_photo']['artifact_ids']==note.artifact_ids
    assert match.metadata['linked_photo']['observation_pose']==original.model_dump(mode='json')
    photos=memory.query_semantic_memory('等候点',similarity_threshold=0,limit=20)
    assert any(m.semantic_memory and m.semantic_memory.linked_location_id==location.location_id
               and m.semantic_memory.artifact_ids==note.artifact_ids for m in photos.matches)
    memory._tagged_locations_by_id.clear();memory._semantic_entries_by_id.clear()
    memory._load_persisted_memory(library_name='unit_test_library')
    assert memory._tagged_locations_by_id[location.location_id].memory_id==later.memory_id
    assert memory._semantic_entries_by_id[note.memory_id].pose==original
    assert memory.remember_current_scene().linked_location_id==location.location_id


def test_explicit_location_link_never_overwrites_unknown_captured_pose(scene):
    memory,full,original,current,captured=scene
    location,_=memory.tag_location('等待处',auto_create_memory=False)
    captured['observation_pose']=None;captured['localization_snapshot']=None
    note=memory.remember_current_scene(linked_location_id=location.location_id)
    assert note.pose is None and note.linked_location_id==location.location_id
    assert note.artifact_ids==[full.image_artifact.artifact_id]


@pytest.mark.parametrize('case',['library','epoch'])
def test_tag_cannot_attach_old_map_photo(case,scene):
    memory,full,original,current,captured=scene
    if case=='library':captured['library_name']='other-library'
    else:captured['map_snapshot']=captured['map_snapshot'].model_copy(update={'source_epoch':'other-origin'})
    with pytest.raises(GatewayError):memory.tag_location('等候点')
    assert memory._repository.load_tagged_locations()==[]
    assert memory._repository.load_semantic_memories()==[]


def test_remember_does_not_guess_ambiguous_or_different_observation(scene):
    memory,full,original,current,captured=scene
    memory.tag_location('甲点',auto_create_memory=False)
    memory.tag_location('乙点',auto_create_memory=False)
    assert memory.remember_current_scene().linked_location_id is None
    memory._tagged_locations_by_id.clear()
    location,_=memory.tag_location('唯一点',auto_create_memory=False)
    changed=full.model_copy(update={'observation':full.observation.model_copy(update={'observation_id':'obs_front_20261010T101000Z_87654321'})})
    memory._perception_service.get_latest_full_perception=lambda camera_id=None:changed
    assert memory.remember_current_scene().linked_location_id is None


def test_existing_place_without_photo_can_gain_explicit_photo_without_retag(scene):
    memory,full,original,current,captured=scene
    location,_=memory.tag_location('等候点',attach_latest_perception=False,attach_latest_observation=False)
    old_id=location.location_id
    assert not memory._semantic_entries_by_id[location.memory_id].artifact_ids
    photo=memory.remember_current_scene(linked_location_id=old_id)
    saved=memory._repository.load_tagged_locations()[0]
    assert saved.location_id==old_id and saved.pose==current and saved.name=='等候点'
    assert saved.memory_id==photo.memory_id and photo.pose==original
    without_photo=full.model_copy(update={'image_artifact':None})
    memory._perception_service.get_latest_full_perception=lambda camera_id=None:without_photo
    memory.remember_current_scene(linked_location_id=old_id)
    assert memory._repository.load_tagged_locations()[0].memory_id==photo.memory_id


def test_explicit_link_rejects_different_map_before_writing(scene):
    memory,full,original,current,captured=scene
    location,_=memory.tag_location('等候点',auto_create_memory=False)
    captured['map_snapshot']=captured['map_snapshot'].model_copy(update={'source_epoch':'rebuilt-origin'})
    with pytest.raises(GatewayError):memory.remember_current_scene(linked_location_id=location.location_id)
    assert not memory._repository.load_semantic_memories()


@pytest.mark.parametrize("operation", ["tag", "remember"])
def test_same_library_reactivation_rejects_old_observation(scene, operation):
    memory, full, original, current, captured = scene
    memory.activate_memory_library(library_name="unit_test_library")
    with pytest.raises(GatewayError) as exc:
        if operation == "tag":
            memory.tag_location("旧观察")
        else:
            memory.remember_current_scene()
    assert exc.value.error_code == "memory_library_changed"
    assert not memory._repository.load_semantic_memories()


def test_photo_link_source_prepare_does_not_hold_instances_lock(scene, monkeypatch):
    import threading
    memory, full, original, current, captured = scene
    location, _ = memory.tag_location("等候点", auto_create_memory=False)
    prepare = memory._durable_index.prepare
    observed = []
    def checked(kind, record, **kwargs):
        if kind == "location" and record.memory_id:
            acquired = []
            def probe():
                ok = memory._instances_lock.acquire(timeout=1)
                acquired.append(ok)
                if ok: memory._instances_lock.release()
            thread = threading.Thread(target=probe); thread.start(); thread.join(2)
            assert acquired == [True]
            observed.append(True)
        return prepare(kind, record, **kwargs)
    monkeypatch.setattr(memory._durable_index, "prepare", checked)
    note = memory.remember_current_scene(linked_location_id=location.location_id)
    assert observed and memory._tagged_locations_by_id[location.location_id].memory_id == note.memory_id


def test_photo_link_rejects_reactivation_between_prepare_and_commit(scene, monkeypatch):
    from services.memory.vectorization_outbox import OutboxConflict
    memory, full, original, current, captured = scene
    location, _ = memory.tag_location("等候点", auto_create_memory=False)
    prepare = memory._durable_index.prepare
    def changed(kind, record, **kwargs):
        prepared = prepare(kind, record, **kwargs)
        if kind == "location" and record.memory_id:
            memory.activate_memory_library(library_name="unit_test_library")
        return prepared
    monkeypatch.setattr(memory._durable_index, "prepare", changed)
    with pytest.raises(OutboxConflict):
        memory.remember_current_scene(linked_location_id=location.location_id)
    saved = memory._repository.load_tagged_locations()[0]
    assert saved.memory_id is None


def test_capture_rejects_library_reactivation_during_spatial_read(scene, monkeypatch):
    memory, full, original, current, captured = scene
    def changed():
        memory.activate_memory_library(library_name="unit_test_library")
        return captured["map_snapshot"]
    monkeypatch.setattr(memory, "_get_map_snapshot", changed)
    with pytest.raises(GatewayError) as exc:
        memory.capture_observation_spatial_context()
    assert exc.value.error_code == "memory_library_changed"


def test_capture_records_current_generation_and_missing_pose_without_invention(scene):
    memory, full, original, current, captured = scene
    memory._get_or_refresh_localization = lambda **kwargs: None
    capture = memory.capture_observation_spatial_context()
    assert capture["activation_generation"] == memory._durable_index.activation_generation
    assert capture["library_name"] == "unit_test_library"
    assert capture["observation_pose"] is None and capture["localization_snapshot"] is None
