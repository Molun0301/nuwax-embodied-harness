"""同一观察的照片、拍摄位姿与地点关联在真实库中保持完整。"""
from datetime import timedelta
from types import SimpleNamespace
import pytest

from contracts.artifacts import ArtifactRef
from contracts.naming import build_artifact_id, build_observation_id
from tests.test_a2d_memory_inputs import memory_at, observed, position
from tests.test_a2c_memory_core import png


@pytest.fixture
def scene(tmp_path):
    original = position(2, 1, .7)
    current = original.model_copy(update={'timestamp': original.timestamp+timedelta(seconds=4),
        'position': original.position.model_copy(update={'x': 4})})
    artifact = ArtifactRef(artifact_id=build_artifact_id('image'), kind='image', mime_type='image/png')
    full = observed(now=original.timestamp).model_copy(update={'image_artifact': artifact,
        'metadata': {'frame_timestamp': original.timestamp.isoformat(), 'camera_id': 'front_camera'}})
    with memory_at(tmp_path, name='unit_test_library', pose=current,
                   perception=SimpleNamespace(get_latest_full_perception=lambda camera=None: full)) as (memory, geometry):
        yield memory, geometry, full, original, current


def remember(memory, full, original, **kwargs):
    return memory.remember_current_scene(map_name=memory.current_library,
        perception_context=full, observation_pose=original,
        image_bytes=png() if full.image_artifact is not None else None, **kwargs)


def test_tag_current_place_links_original_photo_and_survives_reopen(scene, tmp_path):
    memory, geometry, full, original, current = scene
    location, _ = memory.tag_location('等候点', map_name=memory.current_library,
        perception_context=full, observation_pose=current, image_bytes=png())
    note = remember(memory, full, original)
    assert location.pose == current and note.pose == original
    assert location.observation_id == note.observation_id == full.observation.observation_id
    assert note.artifact_ids == [full.image_artifact.artifact_id]
    assert note.metadata['frame_timestamp'] == original.timestamp.isoformat()
    assert note.metadata['camera_id'] == 'front_camera'
    assert note.linked_location_id == location.location_id
    assert '等候点' in note.tags
    query = memory.query_location('等候点', similarity_threshold=0)
    match = next(m for m in query.matches if m.tagged_location.location_id == location.location_id)
    assert match.tagged_location.memory_id == note.memory_id
    assert match.metadata['linked_photo']['artifact_ids'] == note.artifact_ids
    assert match.metadata['linked_photo']['observation_pose'] == original.model_dump(mode='json')
    photos = memory.query_semantic_memory('等候点', similarity_threshold=0, limit=20)
    assert any(m.semantic_memory and m.semantic_memory.linked_location_id == location.location_id
               and m.semantic_memory.artifact_ids == note.artifact_ids for m in photos.matches)
    memory.close()
    with memory_at(tmp_path, name='unit_test_library') as (reopened, _):
        assert reopened.list_tagged_locations()[0].memory_id == note.memory_id
        assert next(x for x in reopened.list_semantic_memories() if x.memory_id == note.memory_id).pose == original
        assert reopened.read_photo(note.artifact_ids[0]) == png()
        assert remember(reopened, full, original).linked_location_id == location.location_id


def test_explicit_link_keeps_unknown_photo_pose(scene):
    memory, _, full, original, current = scene
    location, _ = memory.tag_location('等待处', map_name=memory.current_library, auto_create_memory=False,
        perception_context=full, observation_pose=current, image_bytes=png())
    note = remember(memory, full, None, linked_location_id=location.location_id)
    assert note.pose is None and note.linked_location_id == location.location_id
    assert note.artifact_ids == [full.image_artifact.artifact_id]


def test_ambiguous_or_different_observation_does_not_guess_link(scene):
    memory, _, full, original, current = scene
    for name in ('甲点', '乙点'):
        memory.tag_location(name, map_name=memory.current_library, auto_create_memory=False,
            perception_context=full, observation_pose=current, image_bytes=png())
    assert remember(memory, full, original).linked_location_id is None
    changed = full.model_copy(update={'observation': full.observation.model_copy(update={
        'observation_id': build_observation_id('other')})})
    assert remember(memory, changed, original).linked_location_id is None


def test_photo_can_attach_to_existing_place_without_retag_and_later_text_keeps_it(scene):
    memory, _, full, original, current = scene
    location, _ = memory.tag_location('等候点', map_name=memory.current_library,
        attach_latest_perception=False, attach_latest_observation=False, observation_pose=current)
    before = next(x for x in memory.list_semantic_memories() if x.memory_id == location.memory_id)
    assert not before.artifact_ids
    photo = remember(memory, full, original, linked_location_id=location.location_id)
    saved = memory.list_tagged_locations()[0]
    assert saved.location_id == location.location_id and saved.name == '等候点' and saved.pose == current
    assert saved.memory_id == photo.memory_id and photo.pose == original
    remember(memory, full.model_copy(update={'image_artifact': None}), original,
             linked_location_id=location.location_id)
    assert memory.list_tagged_locations()[0].memory_id == photo.memory_id


def test_old_map_observation_does_not_write_the_new_library(scene):
    memory, geometry, full, original, current = scene
    geometry.change('other-map')
    memory.write(lambda store: None)
    assert memory.remember_current_scene(map_name='unit_test_library', perception_context=full,
        observation_pose=original, image_bytes=png()) is None
    assert memory.tag_location('旧观察', map_name='unit_test_library', perception_context=full,
        observation_pose=current, image_bytes=png()) == (None, None)
    assert memory.list_tagged_locations() == () and memory.list_semantic_memories() == ()


def test_explicit_image_bytes_without_artifact_still_save_a_real_photo(scene):
    memory,_,full,original,current=scene
    entry=memory.remember_current_scene(map_name=memory.current_library,
        perception_context=full.model_copy(update={'image_artifact':None}),
        observation_pose=original,image_bytes=png())
    assert entry.artifact_ids and memory.read_photo(entry.artifact_ids[0])==png()
