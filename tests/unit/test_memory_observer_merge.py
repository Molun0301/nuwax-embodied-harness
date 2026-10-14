"""Observed body poses coexist with object extent, identity and merge ownership."""
from dataclasses import replace
from datetime import datetime, timedelta, timezone
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from contracts.geometry import FrameTree, Pose, Transform, Vector3
from contracts.perception import Observation
from contracts.spatial_memory import InstanceDynamicState
from scripts.diagnostics import object_memory_integration_20261014 as integration
from services.memory.inspection_pose_planner import InspectionPosePlanner
from services.memory.instance_association_service import InstanceAssociationService
from services.memory.object_extent import object_extent_payload
from services.memory.semantic_map_builder import SemanticMapBuildResult
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
            map_name='house', localization_session_id='session', now=NOW+timedelta(seconds=seconds))
        return outcomes[0].instance


def project(robot, *, tree=None, frame='map'):
    box = integration.detection3d('cup', x=3, y=2, z=1.5, size=(.08,.08,.12))
    support = integration.detection2d('cup', x=3, y=2, z=1.5, size=(.08,.08,.12))
    context = SimpleNamespace(observation=Observation(observation_id='obs_merge_20261014T040000Z_00000001',
        frame_id='map', detections_3d=[box], detections_2d=[support]), camera_id='front_camera', camera_info=None)
    map_context = SemanticMapBuildResult(map_name='house', frame_id=frame,
        semantic_regions=(), anchors_by_id={}, topology_nodes_by_id={})
    return VisionToMapProjectionService().project(context, current_pose=robot, map_context=map_context,
        localization_snapshot=SimpleNamespace(frame_tree=tree, current_pose=robot))


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
    fallback = seq.add(observation(x=20, fallback=True, source='weak'),seconds=1)
    assert fallback.instance_id == known.instance_id
    assert fallback.pose == known.pose
    assert fallback.metadata['object_extent'] == known.metadata['object_extent']
    assert fallback.inspection_poses == known.inspection_poses
    assert fallback.attributes['observer_body_pose'] == robot.model_dump(mode='json')


def test_real_ingest_sqlite_reopen_retains_latest_observer_and_original_object_identity():
    case = integration.ObjectMemoryIntegration()
    case.setUp()
    try:
        service = case.service()
        first_robot = integration.position(.5,.5,-.25)
        case.settle(service, [integration.detection3d()], pose=first_robot)
        before = case.stored(service)[0]
        assert before.attributes['observer_body_pose'] == first_robot.model_dump(mode='json')
        assert before.inspection_poses[0] == first_robot
        service.close()
        reopened = case.service()
        next_robot = integration.position(1,1,-.4)
        case.ingest(reopened,[integration.detection3d()],second=86400,pose=next_robot,session='next_day')
        stored = case.stored(reopened)
        assert len(stored) == 1
        after = stored[0]
        assert after.instance_id == before.instance_id
        assert after.observation_count == 4
        assert after.attributes['observer_body_pose'] == next_robot.model_dump(mode='json')
        assert after.inspection_poses[0] == next_robot
        assert all(p.position.z == -.4 for p in after.inspection_poses)
        assert object_extent_payload(after)['center']['z'] == pytest.approx(.7)
        assert object_extent_payload(after)['size_m'] == {'x':.9,'y':.9,'z':1.1}
        reopened.close()
        twice = case.service()
        final = case.stored(twice)[0]
        assert final.instance_id == before.instance_id
        assert final.attributes['observer_body_pose'] == next_robot.model_dump(mode='json')
        assert final.inspection_poses[0] == next_robot
    finally:
        case.doCleanups()


def test_planner_rejects_mismatched_observer_frame():
    with pytest.raises(ValueError, match='地图坐标系'):
        InspectionPosePlanner().plan_for_target(target_pose=Pose(frame_id='map'),
            observer_body_pose=Pose(frame_id='odom'))
