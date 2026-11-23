"""Observed body poses coexist with object extent, identity and merge ownership."""
from dataclasses import replace
from datetime import datetime, timedelta, timezone
from types import SimpleNamespace
from unittest.mock import patch
import math

import pytest
from contracts.geometry import FrameTree, Pose, Transform, Vector3
from contracts.perception import Observation
from contracts.spatial_memory import InstanceDynamicState
from tests import test_a2d_memory_inputs as integration
from services.memory.inspection_pose_planner import InspectionPosePlanner
from services.memory.instance_association_service import InstanceAssociationService
from services.memory.object_extent import object_extent_payload
from services.memory.semantic_map.semantic_map_builder import SemanticMapBuildResult
from services.memory.vision_to_map_projection_service import ProjectedObservation, VisionToMapProjectionService

NOW = datetime(2026, 10, 14, 4, tzinfo=timezone.utc)


def observation(x=2, *, source='first', fallback=False, **attributes):
    attrs = {'projection_method': 'current_pose_fallback' if fallback else 'detection3d_pose',
             'projection_confidence': .99, 'pose_quality_level': 'fallback' if fallback else 'high',
             'support_point_count': 0 if fallback else 10,
             'size_x_m': .08, 'size_y_m': .08, 'size_z_m': .12, **attributes}
    return ProjectedObservation(label='cup', score=.98,
        pose=Pose(frame_id='map', position=Vector3(x=x,y=3,z=.5)), camera_id='front', track_id='t',
        instance_type='object', source_observation_id=source, source_artifact_ids=(),
        semantic_region_id=None, topo_node_id=None, semantic_labels=(), visual_labels=('cup',),
        attributes=attrs)


class Sequence:
    def __init__(self):
        self.service = InstanceAssociationService(text_embedder=None)
        self.instances = {}

    def add(self, item, *, seconds=0):
        self.instances, outcomes = self.service.associate([item], existing_instances=self.instances,
            map_name='house',  now=NOW+timedelta(seconds=seconds))
        return outcomes[0].instance


def project(robot, *, tree=None, frame='map'):
    box = integration.detection3d('cup', x=3, y=2, z=1.5, size=(.08,.08,.12))
    support = integration.detection2d('cup', x=3, y=2, z=1.5, size=(.08,.08,.12))
    context = SimpleNamespace(observation=Observation(observation_id='obs_merge_20261014T040000Z_00000001',
        frame_id='map', detections_3d=[box], detections_2d=[support]), camera_id='front_camera', camera_info=None)
    map_context = SemanticMapBuildResult(map_name='house', frame_id=frame,
        semantic_regions=(), anchors_by_id={}, topology_nodes_by_id={})
    return VisionToMapProjectionService().project(context, current_pose=robot, map_context=map_context,
        frame_tree=tree)


def test_projection_keeps_observer_height_and_capture_time_separate_from_object_extent():
    robot = Pose(frame_id='map', position=Vector3(x=1,y=2,z=-.35),
        timestamp=datetime(2026,10,14,4,tzinfo=timezone.utc))
    observations = project(robot)
    assert len(observations) == 2
    for observation in observations:
        assert observation.observer_body_pose == robot
        assert observation.attributes['observer_body_pose'] == robot.model_dump(mode='json')
        extent = object_extent_payload(observation)
        assert extent['center']['z'] == pytest.approx(1.5)
        assert extent['size_m']['z'] == pytest.approx(.12)
        poses = InspectionPosePlanner().plan_for_target(target_pose=observation.pose,
            observer_body_pose=observation.observer_body_pose)
        assert poses[0] == robot
        assert all(p.position.z == -.35 for p in poses)


def test_projection_transforms_observer_to_map_and_preserves_capture_timestamp():
    robot = Pose(frame_id='odom', position=Vector3(x=1,y=2,z=-.35),
        timestamp=datetime(2026,10,14,4,tzinfo=timezone.utc))
    tree = FrameTree(root_frame_id='map', transforms=[Transform(parent_frame_id='map', child_frame_id='odom',
        translation=Vector3(x=10,y=20,z=1))])
    for observation in project(robot, tree=tree):
        observer = observation.observer_body_pose
        assert observer.frame_id == 'map'
        assert observer.position.model_dump() == pytest.approx({'x':11,'y':22,'z':.65})
        assert observer.timestamp == robot.timestamp
        assert observation.attributes['observer_body_pose'] == observer.model_dump(mode='json')


def test_dynamic_merge_uses_final_position_and_latest_observer_without_losing_extent_or_attributes():
    seq = Sequence()
    first_observer = Pose(frame_id='map', position=Vector3(x=1,y=2,z=.2))
    first_observation = replace(observation(prior={'kept':[1]}),
        observer_body_pose=first_observer)
    first = seq.add(first_observation)
    next_observer = Pose(frame_id='map', position=Vector3(x=2,y=2,z=.4))
    next_observation = replace(observation(x=3, source='move',
        added={'value':[2]}, observer_body_pose={'stale':True}), observer_body_pose=next_observer)
    planner = seq.service._inspection_pose_planner
    with patch.object(planner, 'plan_for_target', wraps=planner.plan_for_target) as call:
        moved = seq.add(next_observation, seconds=1)
    assert moved.instance_id == first.instance_id
    assert moved.dynamic_state == InstanceDynamicState.DYNAMIC
    assert moved.pose.position.x == 3
    assert call.call_args.kwargs['target_pose'] == moved.pose
    assert call.call_args.kwargs['observer_body_pose'] == next_observer
    assert moved.inspection_poses[0] == next_observer
    assert all(p.position.z == .4 for p in moved.inspection_poses)
    assert moved.attributes['prior'] == {'kept':[1]}
    assert moved.attributes['added'] == {'value':[2]}
    assert moved.attributes['observer_body_pose'] == next_observer.model_dump(mode='json')
    assert object_extent_payload(moved)['center']['x'] == 3
    assert object_extent_payload(moved)['size_m'] == {'x':.08,'y':.08,'z':.12}
    next_observation.attributes['added']['value'].append(999)
    first.attributes['prior']['kept'].append(999)
    next_observer.position.z = 99
    assert moved.attributes['added'] == {'value':[2]}
    assert moved.attributes['prior'] == {'kept':[1]}
    assert moved.attributes['observer_body_pose']['position']['z'] == .4
    assert moved.inspection_poses[0].position.z == .4


def test_observerless_fallback_retains_previous_observer_extent_and_inspection_poses():
    seq = Sequence()
    robot = Pose(frame_id='map', position=Vector3(x=1,y=2,z=-.2))
    original = replace(observation(
        observer_body_pose=robot.model_dump(mode='json')), observer_body_pose=robot)
    known = seq.add(original)
    outcome = seq.add(observation(x=20, fallback=True, source='weak'),seconds=1)
    assert outcome is None
    fallback=seq.instances[known.instance_id]
    assert fallback.instance_id == known.instance_id
    assert fallback.pose == known.pose
    assert fallback.metadata['object_extent'] == known.metadata['object_extent']
    assert fallback.inspection_poses == known.inspection_poses
    assert fallback.attributes['observer_body_pose'] == robot.model_dump(mode='json')


def test_real_ingest_sqlite_reopen_retains_latest_observer_and_original_object_identity(tmp_path):
    first_robot = integration.position(.5,.5,-.25)
    with integration.memory_at(tmp_path) as (service, _):
        for second in range(3):
            integration.ingest(service,[integration.detection3d()],pose=first_robot,now=NOW+timedelta(seconds=second))
        before = service.records('instance')[0]
        assert before.attributes['observer_body_pose'] == first_robot.model_dump(mode='json')
        assert before.inspection_poses[0] == first_robot
    next_robot = integration.position(1,1,-.4)
    with integration.memory_at(tmp_path) as (reopened, _):
        integration.ingest(reopened,[integration.detection3d()],pose=next_robot,now=NOW+timedelta(seconds=86400))
        stored = reopened.records('instance')
        assert len(stored) == 1
        after = stored[0]
        assert after.instance_id == before.instance_id
        assert after.observation_count == 4
        assert after.attributes['observer_body_pose'] == next_robot.model_dump(mode='json')
        assert after.inspection_poses[0] == next_robot
        assert all(p.position.z == -.4 for p in after.inspection_poses)
        assert object_extent_payload(after)['center']['z'] == pytest.approx(.7)
        assert object_extent_payload(after)['size_m'] == {'x':.9,'y':.9,'z':1.1}
    with integration.memory_at(tmp_path) as (twice, _):
        final = twice.records('instance')[0]
        assert final.instance_id == before.instance_id
        assert final.attributes['observer_body_pose'] == next_robot.model_dump(mode='json')
        assert final.inspection_poses[0] == next_robot


def test_planner_rejects_mismatched_observer_frame():
    with pytest.raises(ValueError, match='地图坐标系'):
        InspectionPosePlanner().plan_for_target(target_pose=Pose(frame_id='map'),
            observer_body_pose=Pose(frame_id='odom'))


def test_map_correction_reopens_advances_and_rebases_in_flight_observations(tmp_path):
    from contracts.naming import build_scene_space_id
    from contracts.spatial_memory import NavigationTrajectoryEvent, NavigationTrajectoryEventKind
    from contracts.spatial_memory import SceneObservationSample, SceneSpace, SceneSpaceBoundary
    from services.geometry.proto_gen import geometry_runtime_pb2 as pb
    from services.memory.map_correction import SESSION_KEY
    from services.perception.observation_geometry import _interpolate_pose

    stamp_first, stamp_second = 1_000_000_000, 7_000_000_000
    session = 37

    def keyframe(stamp, shift, angle=0.0, shift_y=0.0, old_x=0.0):
        return pb.MapCorrectionKeyframe(timestamp_ns=stamp, old_position=pb.Vec3(x=old_x),
            translation=pb.Vec3(x=shift, y=shift_y),
            rotation=pb.Quat(w=math.cos(angle / 2), z=math.sin(angle / 2)))

    events = {
        1: pb.MapCorrection(keyframes=[keyframe(stamp_first, .1), keyframe(stamp_second, .2)]),
        2: pb.MapCorrection(keyframes=[keyframe(stamp_first, .5),
            keyframe(stamp_second, 3.8, angle=math.pi / 2, shift_y=-1.1, old_x=6.0)]),
    }
    calls = []
    published = []

    def get_event(owner, seq):
        assert owner == session
        calls.append(seq)
        return events[seq]

    def source(seq, stamp, x=0.0, *, owner=session, frame='map'):
        return Pose(frame_id=frame, position=Vector3(x=x, y=1, z=.7),
            session_id=owner, correction_seq=seq, source_timestamp_ns=stamp)

    def state(geometry, seq, pose_seq=None, x=0.0):
        geometry.value = replace(geometry.value, session_id=session, correction_seq=seq,
            pose=replace(geometry.value.pose, session_id=session, x=x, y=1,
                         correction_seq=seq if pose_seq is None else pose_seq, timestamp_ns=stamp_second))
        for callback in tuple(geometry.callbacks):
            callback(geometry.value)

    class Bus:
        def publish_frame(self, frame):
            published.append(frame.topic)

    def door(x):
        detection = integration.detection3d('door', x=x)
        detection.attributes.update(dynamic_state="static",
            support_points_map={"frame_id": "map", "points": [[x, 1.0, .7], [x, 1.1, .7]]},
            camera_points={"frame_id": "camera", "points": [[0.0, 0.0, 1.0]]})
        return detection

    def remember(service, title, pose, detection=None, *, automatic=True):
        items = () if detection is None else (detection,)
        return service.remember_current_scene(map_name='house', title=title,
            perception_context=integration.observed(items), observation_pose=pose,
            metadata={"source": "perception_video_runtime"} if automatic else {})

    with integration.memory_at(tmp_path, stream_bus=Bus()) as (service, geometry):
        geometry.get_map_correction = get_event
        state(geometry, 0)
        first = source(0, stamp_first, 2.0)
        remember(service, '校正前', first, door(2.0))
        remember(service, '他会话', source(0, stamp_first, 9.0, owner=36))
        remember(service, '里程计', source(0, stamp_first, 8.0, frame='odom'))
        service.ingest_perception_observation(map_name='house',
            perception_context=integration.observed((door(2.1),)),
            current_pose=source(1, stamp_second, 2.1), admission='explicit')
        assert calls == [1]
        before = service.records("instance")[0]
        assert before.pose.position.x == pytest.approx(2.1)
        assert len(before.correction_buckets) == 2
        fixed = source(0, stamp_first, -4.0)
        space = SceneSpace(scene_space_id=build_scene_space_id('test'), map_name='house', frame_id='map',
            boundary=SceneSpaceBoundary(frame_id='map', centroid=fixed),
            inspection_poses=[fixed], metadata={"source": "room_segmentation", SESSION_KEY: 36},
            observation_samples=[SceneObservationSample(sample_id='source', pose=source(1, stamp_first, 2.1))])
        service.write(lambda store: store.put("scene_space", space))
        kind = NavigationTrajectoryEventKind.VISITED_POSE
        trajectory = NavigationTrajectoryEvent(trajectory_event_id=service._build_navigation_trajectory_event_id(
            kind, "test", "external"), event_kind=kind, map_name="house", pose=source(1, stamp_first, 2.1),
            target_pose=Pose(frame_id="map", position=Vector3(x=4.0)))
        service.write(lambda store: service._upsert_navigation_trajectory_event(trajectory))
        remember(service, '两次之间', source(1, stamp_first, 2.1))
        assert service.write(lambda store: store.correction_progress()) == (session, 1)

    calls.clear()
    published.clear()
    with integration.memory_at(tmp_path, stream_bus=Bus()) as (service, geometry):
        geometry.get_map_correction = get_event
        state(geometry, 2, x=42.0)
        service.flush_observations()
        assert calls == [2]
        assert {'memory.instances', 'memory.scene_spaces'} <= set(published)
        assert service.write(lambda store: store.correction_progress()) == (session, 2)
        after = service.records("instance")[0]
        assert after.instance_id == before.instance_id
        assert after.pose.position.x == pytest.approx(2.7)
        assert after.pose.position.y == pytest.approx(1.0)
        assert after.pose.orientation.z == pytest.approx(math.sin(math.pi / 4))
        assert after.support_evidence.voxel_keys
        observed = service.records("observation")[0]
        assert observed.pose.position.x == pytest.approx(2.6)
        assert observed.metadata["support_points_map"]["points"][0] == pytest.approx([2.6, 1.0, .7])
        assert observed.metadata["camera_points"] == {"frame_id": "camera", "points": [[0.0, 0.0, 1.0]]}
        travelled = service.records("trajectory")[0]
        assert travelled.pose.position.x == pytest.approx(2.6)
        assert travelled.target_pose.position.model_dump() == pytest.approx({"x": 3.8, "y": 2.9, "z": 0.0})
        previous_station, current_station = before.inspection_poses[0], after.inspection_poses[0]
        assert current_station.position.x - after.pose.position.x == pytest.approx(
            -(previous_station.position.y - before.pose.position.y))
        assert current_station.position.y - after.pose.position.y == pytest.approx(
            previous_station.position.x - before.pose.position.x)
        stored_space = service.records("scene_space")[0]
        assert stored_space.boundary == space.boundary and stored_space.inspection_poses == [fixed]
        assert stored_space.observation_samples[0].pose.position.x == pytest.approx(2.6)

        remember(service, '第二次之后', source(2, stamp_first, 2.6))
        remember(service, '在途', source(0, stamp_first, 2.0))
        mixed = _interpolate_pose([(stamp_first, source(1, stamp_first, 3.0)),
            (stamp_first + 2, source(2, stamp_first + 2, 3.0))], stamp_first + 1)
        assert mixed.mixed_correction
        assert remember(service, '混合关键帧', mixed) is None
        explicit = remember(service, '显式写入', mixed, automatic=False)
        assert explicit.pose.position.x == pytest.approx(3.5)
        assert explicit.pose.correction_seq == 2
        locations = {entry.title: entry.pose.position.x for entry in service.records("memory")}
        assert locations == pytest.approx({'校正前': 2.6, '两次之间': 2.6, '第二次之后': 2.6,
            '在途': 2.6, '他会话': 9.0, '里程计': 8.0, '显式写入': 3.5})
        assert calls == [2, 1, 2, 2]

    calls.clear()
    with integration.memory_at(tmp_path) as (service, geometry):
        geometry.get_map_correction = get_event
        state(geometry, 2)
        service.flush_observations()
        assert calls == []
        assert service.records("instance")[0].pose == after.pose
        assert service.records("instance")[0].metadata[SESSION_KEY] == session


def test_map_correction_explicit_writes_use_the_supplied_observation_pose(tmp_path):
    from services.geometry.proto_gen import geometry_runtime_pb2 as pb

    session_id = 37
    stamp = 1_000_000_000
    correction = pb.MapCorrection(keyframes=[pb.MapCorrectionKeyframe(
        timestamp_ns=stamp, translation=pb.Vec3(x=5), rotation=pb.Quat(w=1))])
    observed_pose = Pose(frame_id="map", position=Vector3(x=2), source_timestamp_ns=stamp,
                         session_id=session_id)
    corrected_pose = observed_pose.model_copy(update={"position": Vector3(x=7), "correction_seq": 1})
    with integration.memory_at(tmp_path) as (service, geometry):
        geometry.get_map_correction = lambda owner, seq: {(session_id, 1): correction}[owner, seq]
        geometry.value = replace(geometry.value, session_id=session_id, correction_seq=1,
            pose=replace(geometry.value.pose, x=42, session_id=session_id, correction_seq=1))
        geometry.change("house")
        service.flush_observations()

        scene = service.remember_current_scene(map_name="house", perception_context=integration.observed(),
                                               observation_pose=observed_pose)
        location, note = service.tag_location("观察点", map_name="house", observation_pose=observed_pose,
                                              attach_latest_observation=False, attach_latest_perception=False)

        assert service.current_pose().x == 42
        assert service.get_record("memory", scene.memory_id).pose == corrected_pose
        assert service.get_record("location", location.location_id).pose == corrected_pose
        assert service.get_record("memory", note.memory_id).pose == corrected_pose


def test_find_target_navigates_to_a_written_door_observation(tmp_path):
    robot = integration.position(0, 0, 0)
    door = integration.detection3d("door")
    first_frame = datetime.now(timezone.utc) - timedelta(seconds=2)
    with integration.memory_at(tmp_path, pose=robot) as (service, _):
        map_name = service.current_library
        for offset in range(3):
            context = integration.observed([door], now=first_frame + timedelta(seconds=offset))
            service.remember_current_scene(map_name=map_name, perception_context=context, observation_pose=robot)

        events = service.records("observation")
        assert len(events) == 3
        event_ids = {event.event_id for event in events if event.visual_labels == ["door"]}
        candidates = service.resolve_navigation_candidates("door", map_name=map_name, object_target=True)
        assert any(
            candidate.record_id in event_ids
            and candidate.metadata["allowed_for_navigation"] is True
            and candidate.metadata["resolution_decision"] == "navigate"
            for candidate in candidates
        )


def test_inspection_waits_for_stop_and_confirms_from_each_source_frame(tmp_path, monkeypatch):
    from contracts.runtime_views import SceneObjectSummary
    from core.settings import PerceptionConfig
    from gateways.capabilities import navigation
    from services.navigation.motion_client import MeasuredMotion
    from services.perception.runtime_sample import PerceptionSample

    robot = integration.position(0, 0, 0)
    apple = integration.detection2d("apple", x=1, y=1, size=(.08, .08, .08))
    samples = []
    for offset in range(3):
        context = integration.observed([apple], now=NOW + timedelta(seconds=offset))
        context.scene_summary.objects = [SceneObjectSummary(label="apple", count=1, max_score=.97)]
        samples.append(PerceptionSample(context, "house", pose=robot, complete=True))
    feedback = iter((
        MeasuredMotion("robot", 99, 0, 0, "base_link", (0., 0., 0.), (0., 0., 0.), None),
        MeasuredMotion("robot", 100, 0, 0, "base_link", (.1, 0., 0.), (0., 0., 0.), None),
        MeasuredMotion("robot", 101, 0, 0, "base_link", (0., 0., 0.), (0., 0., 0.), None),
    ))
    waits = []

    def burst(ctx, *, reason, frame_count, interval_sec, source_after_ns):
        assert source_after_ns == 101 and waits == [.05, .05]
        return tuple(samples[:frame_count])

    monkeypatch.setattr(navigation.clock, "source_now_ns", lambda: 100)
    video = SimpleNamespace(acquire=lambda *a, **kw: None, release=lambda reason: None,
                            run_burst=burst, complete_sample=lambda sample: sample)
    ctx = SimpleNamespace(task_id="inspection", progress=lambda *a: None, check=lambda: None, wait=waits.append)
    with integration.memory_at(tmp_path, pose=robot) as (memory, geometry):
        platform = SimpleNamespace(config=SimpleNamespace(perception=PerceptionConfig()),
            motion=SimpleNamespace(measured_motion=lambda: next(feedback)), geometry_state=geometry,
            memory=memory, perception=SimpleNamespace(video_runtime=video))
        result = navigation.inspect_target(platform, ctx, {"query": "apple"})
        assert result["frame_count"] == 3
        events = memory.records("observation")
        assert {event.source_observation_id for event in events} == {
            sample.context.observation.observation_id for sample in samples}
        assert sum(bool(event.linked_instance_ids) for event in events) == 1
        instance, = memory.records("instance")
        assert instance.support_evidence.confirmation_observation_ids == [
            sample.context.observation.observation_id for sample in samples]


def test_burst_counts_only_images_captured_after_stop(tmp_path):
    from concurrent.futures import ThreadPoolExecutor
    from tests.test_a2c_perception_runtime import Context, eventually, setup

    with setup(tmp_path) as camera, ThreadPoolExecutor(max_workers=1) as executor:
        camera.runtime.acquire("inspect")
        first = camera.source.current.metadata["source_timestamp_ns"]
        burst = executor.submit(camera.runtime.run_burst, Context(),
            reason="inspect", frame_count=1, source_after_ns=first + 1_000_000_000)
        eventually(lambda: bool(camera.runtime._bursts))
        camera.source.advance()
        eventually(lambda: camera.runtime.latest().context.observation.metadata["source_sequence"] == 2)
        assert not burst.done()
        camera.source.advance()
        sample, = burst.result(timeout=3)
        assert sample.context.observation.metadata["source_sequence"] == 3


def test_find_target_region_excludes_outside_memory_and_live_detections(tmp_path, monkeypatch):
    from contracts.naming import build_scene_space_id
    from contracts.runtime_views import SceneObjectSummary
    from contracts.spatial_memory import SceneSpace, SceneSpaceBoundary
    from gateways.capabilities import navigation
    from services.perception.runtime_sample import PerceptionSample

    robot = integration.position(0, 0, 0)
    kitchen = SceneSpace(scene_space_id=build_scene_space_id("kitchen"),
        primary_label="厨房", map_name="house", frame_id="map",
        boundary=SceneSpaceBoundary(frame_id="map", polygon_points=[
            Vector3(x=1, y=-1), Vector3(x=4, y=-1), Vector3(x=4, y=3), Vector3(x=1, y=3)]))
    real = integration.detection2d("apple", x=2, y=1, z=1, size=(.08, .08, .08), track="kitchen")
    outside = integration.detection2d("apple", x=-2, y=1, z=1, size=(.08, .08, .08), track="bedroom")
    samples = []
    for offset, image in enumerate(([outside], [outside, real])):
        context = integration.observed(image, now=NOW + timedelta(seconds=offset))
        context.scene_summary.objects = [SceneObjectSummary(label="apple", count=len(image), max_score=.97)]
        samples.append(PerceptionSample(context, "house", pose=robot, complete=True))
    ctx = SimpleNamespace(task_id="finding", progress=lambda *a, **kw: None, check=lambda: None)
    visits = []

    def arrive(platform, ctx, goal):
        visits.append(goal)
        return {"goal_id": "goal", "arrival": "target"}

    monkeypatch.setattr(navigation, "navigate_to_pose", arrive)
    with integration.memory_at(tmp_path, pose=robot) as (memory, geometry):
        for sample in samples:
            memory.remember_current_scene(map_name="house", perception_context=sample.context,
                                          observation_pose=robot)
        captured = iter(samples)
        video = SimpleNamespace(acquire=lambda *a, **kw: None, release=lambda reason: None,
            run_burst=lambda *a, **kw: (next(captured),), complete_sample=lambda sample: sample)
        region_lookups = []

        def resolve(name):
            region_lookups.append(name)
            return (kitchen,)

        platform = SimpleNamespace(memory=memory, geometry_state=geometry,
            perception=SimpleNamespace(video_runtime=video),
            scene_mapping=SimpleNamespace(graph=SimpleNamespace(resolve_regions=resolve)))
        result = navigation.find_target(platform, ctx, {"query":"apple", "region":"厨房", "motion_generation":1})
        assert region_lookups == ["厨房"] and len(visits) == 1
        assert result["found_mode"] == "memory_candidate_navigation"
        assert result["navigation_candidate"].anchor_pose.position.x == pytest.approx(2)
        assert result["arrival_verification"].verified
        assert result["perception_refresh"]["detection_evidence"]["matched_detection_count"] == 1

        captured = iter(samples)
        current = navigation.find_target(platform, ctx, {"query":"apple", "motion_generation":1})
        assert current["found_mode"] == "current_scene"
        assert region_lookups == ["厨房"] and len(visits) == 1
