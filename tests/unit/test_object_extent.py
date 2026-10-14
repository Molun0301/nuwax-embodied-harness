"""Metric observed-object geometry, without inventing shape from a 2D label."""
from types import SimpleNamespace
import math

import pytest

from contracts.geometry import Pose, Vector3, Quaternion, FrameTree, Transform
from contracts.perception import Detection3D, Observation
from services.memory.object_extent import resolve_object_extent, merge_object_extent, object_extent_payload
from services.memory.semantic_map_builder import SemanticMapBuildResult
from services.memory.vision_to_map_projection_service import VisionToMapProjectionService, ProjectedObservation


def observation(attributes=None, pose=None):
    return ProjectedObservation(label='cabinet', score=.9,
        pose=pose or Pose(frame_id='map', position=Vector3(x=2, y=3, z=-1)),
        camera_id='camera', track_id=None, instance_type='object', source_observation_id='obs_camera_20261014T040000Z_00000001',
        source_artifact_ids=(), semantic_region_id=None, topo_node_id=None, semantic_labels=(), visual_labels=(),
        attributes=attributes or {})


def instance(obs, extent=None):
    return SimpleNamespace(pose=obs.pose, attributes=obs.attributes,
        metadata={'object_extent': extent} if extent else {})


def projected_detection(frame='map', q=None, tree=None, sizes=(2, .5, 3)):
    detection = Detection3D(label='cabinet', score=.9, pose=Pose(frame_id=frame,
        position=Vector3(x=2, y=0, z=-1), orientation=q or Quaternion()),
        size_x_m=sizes[0], size_y_m=sizes[1], size_z_m=sizes[2])
    context = SimpleNamespace(observation=Observation(observation_id='obs_camera_20261014T040000Z_00000001', frame_id='camera', detections_3d=[detection]),
        camera_id='camera', camera_info=None)
    robot = Pose(frame_id='map', position=Vector3(x=10), orientation=Quaternion(z=math.sqrt(.5), w=math.sqrt(.5)))
    localization = SimpleNamespace(frame_tree=tree, current_pose=robot)
    map_context = SemanticMapBuildResult(map_name='house', frame_id='map', semantic_regions=(), anchors_by_id={}, topology_nodes_by_id={})
    return VisionToMapProjectionService().project(context, current_pose=robot, map_context=map_context, localization_snapshot=localization)[0]


def test_point_bounds_use_midpoint_without_changing_observed_centroid_or_floor():
    pose, attributes = VisionToMapProjectionService()._pose_from_resolved_points(
        [Vector3(x=1,y=2,z=-2), Vector3(x=1,y=2,z=-2), Vector3(x=5,y=4,z=0)],
        current_pose=Pose(frame_id='map', orientation=Quaternion(z=1)), map_frame='map',
        projection_method='map_point_samples', projection_confidence=.9)
    assert pose.position.x == pytest.approx(7/3)
    assert attributes['centroid_map']['x'] == pytest.approx(7/3, abs=1e-6)
    assert attributes['bbox_3d_map']['center'] == {'x':3, 'y':3, 'z':-1}
    payload = object_extent_payload(observation(attributes, pose))
    assert payload['center'] == pytest.approx({'x':3, 'y':3, 'z':-1})
    assert payload['size_m'] == {'x':4, 'y':2, 'z':2}
    assert payload['orientation'] == {'x':0,'y':0,'z':0,'w':1}
    assert payload['top_z_m'] == 0


def test_legacy_bbox_prefers_extrema_over_incorrect_old_centroid():
    obs = observation({'bbox_3d_map': {'frame_id':'map', 'center':{'x':99,'y':99,'z':99},
        'extent_m':{'x':99,'y':99,'z':99}, 'min':{'x':1,'y':2,'z':-3}, 'max':{'x':3,'y':6,'z':-1}}})
    result = object_extent_payload(instance(obs))
    assert result['center'] == {'x':2,'y':4,'z':-2}
    assert result['size_m'] == {'x':2,'y':4,'z':2}
    assert result['source'] == 'observed_points'


@pytest.mark.parametrize('bad', [None, -1, float('nan'), float('inf'), True, '2'])
def test_invalid_dimensions_remain_unknown(bad):
    obs = observation({'projection_method':'detection3d_pose', 'size_x_m':bad,'size_y_m':1,'size_z_m':2})
    assert resolve_object_extent(obs) is None


def test_zero_point_span_is_preserved_and_single_depth_or_2d_bbox_is_not_size():
    service = VisionToMapProjectionService()
    for points, expected in [([Vector3(x=1), Vector3(x=3)], {'x':2,'y':0,'z':0}), ([Vector3(x=2)], None)]:
        pose, attrs = service._pose_from_resolved_points(points,current_pose=Pose(frame_id='map'),
            map_frame='map',projection_method='map_point_samples',projection_confidence=.9)
        result = object_extent_payload(observation(attrs,pose))
        assert (result['size_m'] if result else None) == expected
    assert resolve_object_extent(observation({'bbox_xyxy':[0,0,800,600], 'depth_m':3})) is None


def test_detector_rotation_is_transformed_with_position_and_keeps_non_cubic_size():
    obs = projected_detection('base_link')
    result = object_extent_payload(obs)
    assert result['size_m'] == {'x':2,'y':.5,'z':3}
    assert result['center'] == pytest.approx({'x':10,'y':2,'z':-1})
    assert result['orientation']['z'] == pytest.approx(math.sqrt(.5))
    xs = [point['x'] for point in result['footprint_xy']]
    ys = [point['y'] for point in result['footprint_xy']]
    assert max(xs)-min(xs) == pytest.approx(.5)
    assert max(ys)-min(ys) == pytest.approx(2)
    assert result['top_z_m'] == pytest.approx(.5)


def test_camera_box_needs_verified_extrinsics():
    assert object_extent_payload(projected_detection('camera')) is None
    tree = FrameTree(root_frame_id='map', transforms=[Transform(parent_frame_id='map', child_frame_id='camera',
        translation=Vector3(x=4), rotation=Quaternion(x=math.sqrt(.5),w=math.sqrt(.5)))])
    result = object_extent_payload(projected_detection('camera', tree=tree))
    assert result['orientation']['x'] == pytest.approx(math.sqrt(.5))
    assert result['top_z_m'] == pytest.approx(.25)


def test_missing_detector_axis_stays_unknown():
    assert object_extent_payload(projected_detection(sizes=(2,None,3))) is None


def test_size_survives_position_only_and_fallback_and_follows_smoothed_pose():
    first = projected_detection()
    current = instance(first, merge_object_extent(None,first))
    for later in [observation({'projection_method':'track_pose'}), observation({'projection_method':'current_pose_fallback'})]:
        assert merge_object_extent(current,later) == current.metadata['object_extent']
    current.pose = Pose(frame_id='map',position=Vector3(x=20,y=30,z=-4))
    assert object_extent_payload(current)['center'] == {'x':20,'y':30,'z':-4}
    assert current.metadata['object_extent']['source_observation_id'] == 'obs_camera_20261014T040000Z_00000001'


def test_remeasurement_updates_size_but_partial_support_does_not_replace_detector():
    first = projected_detection()
    current = instance(first,merge_object_extent(None,first))
    points = observation({'bbox_3d_map':{'frame_id':'map','center':{'x':2,'y':3,'z':-1}, 'extent_m':{'x':.1,'y':0,'z':.3}}})
    assert merge_object_extent(current,points)['source'] == 'detection3d'
    later = projected_detection(sizes=(4,1,3))
    assert merge_object_extent(current,later)['size_m']['x'] == 4


@pytest.mark.parametrize('frame', ['', 'camera', 'invalid frame'])
def test_bad_or_mismatched_extent_frames_stay_unknown(frame):
    assert resolve_object_extent(observation({'bbox_3d_map':{'frame_id':frame,
        'center':{'x':2,'y':3,'z':-1},'extent_m':{'x':1,'y':1,'z':1}}})) is None


def test_camera_point_fallback_is_not_precise_map_size():
    obs=observation({'projection_degradation_reason':'camera_points_relative_pose_fallback',
        'bbox_3d_map':{'frame_id':'map','center':{'x':2,'y':3,'z':-1},'extent_m':{'x':1,'y':1,'z':1}}})
    assert resolve_object_extent(obs) is None


def test_tilted_box_top_and_convex_footprint_cover_all_eight_corners():
    result = object_extent_payload(projected_detection(q=Quaternion(x=math.sin(math.pi/8),w=math.cos(math.pi/8))))
    assert result['top_z_m'] == pytest.approx(-1 + (3+.5)*math.sqrt(.5)/2)
    assert len(result['footprint_xy']) == 4
    assert max(p['y'] for p in result['footprint_xy']) == pytest.approx((3+.5)*math.sqrt(.5)/2)


def test_changing_surface_sampling_preserves_box_midpoint_through_pose_blending():
    from dataclasses import replace
    from datetime import timedelta
    from contracts.base import utc_now
    from services.memory.instance_association_service import InstanceAssociationService
    projection = VisionToMapProjectionService()
    policy = InstanceAssociationService(text_embedder=None)
    now = utc_now()
    samples = [Vector3(x=x,y=y,z=z) for x in (1,3) for y in (2,4) for z in (-2,0)]
    def measured(extra, index):
        pose, attrs = projection._pose_from_resolved_points(samples+[Vector3(x=extra,y=3,z=-1)],
            current_pose=Pose(frame_id='map'), map_frame='map', projection_method='map_point_samples', projection_confidence=.95)
        return replace(observation(attrs,pose),source_observation_id='sample_'+str(index))
    first = measured(1.5,0)
    current = policy._create_instance(first,map_name='house',localization_session_id='session',
        occupancy_grid=None,cost_map=None,now=now)
    for index,extra in enumerate((2.5,1.5,2.5),1):
        incoming = measured(extra,index)
        current = policy._merge_instance(current,incoming,map_name='house',localization_session_id='session',
            occupancy_grid=None,cost_map=None,now=now+timedelta(seconds=index*10))
        result = object_extent_payload(current)
        assert result['center'] == pytest.approx({'x':2,'y':3,'z':-1})
        assert result['size_m'] == {'x':2,'y':2,'z':2}
        assert current.pose.position.x != incoming.pose.position.x
        center,sizes,_ = policy._spatial_geometry(current)
        assert center == pytest.approx(tuple(result['center'][axis] for axis in 'xyz'))
        assert sizes == pytest.approx(tuple(result['size_m'][axis] for axis in 'xyz'))


def test_nonfinite_world_corners_are_never_published():
    first = projected_detection()
    current = instance(first, merge_object_extent(None,first))
    current.pose = Pose(frame_id='map',position=Vector3(x=1.7e308))
    current.metadata['object_extent']['size_m']['x'] = 1.7e308
    assert object_extent_payload(current) is None


@pytest.mark.parametrize('frame', ['camera', 'bad frame'])
def test_map_point_payload_with_wrong_frame_does_not_claim_calibrated_extent(frame):
    pose, attrs = VisionToMapProjectionService()._pose_from_point_samples(
        attributes={'map_points':{'frame_id':frame,'points':[[1,2,-1],[3,4,0]]}},
        current_pose=Pose(frame_id='map'),map_frame='map',frame_tree=None,camera_frame_id='camera')
    assert pose is not None  # Existing projection/navigation behavior is unchanged.
    assert resolve_object_extent(observation(attrs,pose)) is None


def test_explicit_detection3d_identity_reaches_same_source_2d_association():
    from contracts.perception import BoundingBox2D, Detection2D
    identity = {'camera_id':'front_camera','track_id':'track_cabinet'}
    detected3d = Detection3D(label='cabinet',score=.9,pose=Pose(frame_id='map'),
        size_x_m=2,size_y_m=1,size_z_m=3,attributes=identity)
    detected2d = Detection2D(label='cabinet',score=.9,camera_id='front_camera',track_id='track_cabinet',
        bbox=BoundingBox2D(x_px=0,y_px=0,width_px=20,height_px=30),
        attributes={'map_pose':{'frame_id':'map','x':0,'y':0,'z':0}})
    context = SimpleNamespace(observation=Observation(frame_id='camera',
        observation_id='obs_camera_20261014T040000Z_00000001',detections_3d=[detected3d],detections_2d=[detected2d]),
        camera_id='different_context_camera',camera_info=None)
    service = VisionToMapProjectionService()
    maps = SemanticMapBuildResult(map_name='house',frame_id='map',semantic_regions=(),anchors_by_id={},topology_nodes_by_id={})
    results = service.project(context,current_pose=Pose(frame_id='map'),map_context=maps)
    assert len(results) == 2
    assert {(result.camera_id,result.track_id,result.source_observation_id) for result in results} == {
        ('front_camera','track_cabinet',context.observation.observation_id)}
    for attributes in ({}, {'camera_id':123,'track_id':{}}, {'camera_id':' ', 'track_id':''}):
        context.observation.detections_3d = [detected3d.model_copy(update={'attributes':attributes})]
        result = service.project(context,current_pose=Pose(frame_id='map'),map_context=maps)[0]
        assert result.camera_id is None
        assert result.track_id is None
