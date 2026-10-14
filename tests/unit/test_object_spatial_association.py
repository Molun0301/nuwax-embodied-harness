"""地图物体的跨观察身份与保守几何边界，无模型、网络或设备。"""
from dataclasses import replace
from datetime import datetime, timedelta, timezone
import math
from unittest.mock import patch

import pytest

from contracts.geometry import Pose, Vector3
from contracts.spatial_memory import InstanceDynamicState, InstanceLifecycleState
from services.memory.instance_association_service import InstanceAssociationService
from services.memory.vision_to_map_projection_service import ProjectedObservation

NOW = datetime(2026, 10, 14, tzinfo=timezone.utc)


def observation(x=2.0, y=3.0, z=.5, *, label='cup', size=(.08, .08, .12), source='first', track=None,
                camera='front', frame='map', fallback=False, **attributes):
    attrs = {'projection_method': 'current_pose_fallback' if fallback else 'detection3d_pose',
             'projection_confidence': .99, 'pose_quality_level': 'fallback' if fallback else 'high',
             'support_point_count': 0 if fallback else 10, **attributes}
    if size is not None:
        attrs.update(zip(('size_x_m', 'size_y_m', 'size_z_m'), size))
    return ProjectedObservation(label, .98, Pose(frame_id=frame, position=Vector3(x=x, y=y, z=z)),
        camera, track, 'object', source, (), None, None, (), (label,), attrs)


class Sequence:
    def __init__(self):
        self.service = InstanceAssociationService(text_embedder=None)
        self.instances = {}
    def add(self, *observations, seconds=0, map_name='house', session='session', asset=None):
        self.instances, outcomes = self.service.associate(observations,
            existing_instances=self.instances, map_name=map_name, localization_session_id=session,
            map_asset_id=asset, now=NOW+timedelta(seconds=seconds))
        return [outcome.instance for outcome in outcomes]


def test_umbrella_mask_moves_across_images_and_next_day_reuses_identity():
    seq = Sequence()
    first = seq.add(observation(label='umbrella', size=(.1,.4,1.1), centroid_uv=[100,100],
        segmentation_score=.9, track='old'))[0]
    second = seq.add(observation(x=2.03,z=.59,label='umbrella',size=(.1,.4,1.1),
        centroid_uv=[1000,700],segmentation_score=.9,camera='rear',track='new',source='next'),
        seconds=86400,session='restarted')[0]
    assert len(seq.instances) == 1
    assert second.instance_id == first.instance_id
    assert second.anchor_id == first.anchor_id
    assert second.first_seen_ts == first.first_seen_ts
    assert second.observation_count == 2


def test_alternating_adjacent_cups_and_reordered_batch_keep_two_ids():
    seq = Sequence()
    left = seq.add(observation(x=1))[0]
    right = seq.add(observation(x=1.13,source='second'),seconds=1)[0]
    assert left.instance_id != right.instance_id
    pair = seq.add(observation(x=1.12,source='third'),observation(x=1.01,source='third'),seconds=2)
    assert [item.instance_id for item in pair] == [right.instance_id,left.instance_id]
    assert len(seq.instances) == 2


def test_better_geometry_assignment_wins_before_weaker_detection_regardless_order():
    for reverse in (False,True):
        seq = Sequence()
        known = seq.add(observation(x=1,size=(.2,.2,.2)))[0]
        close = observation(x=1.01,size=(.2,.2,.2),source='next')
        farther = observation(x=1.09,size=(.2,.2,.2),source='next')
        values = [close,farther] if not reverse else [farther,close]
        result = seq.add(*values,seconds=1)
        assert result[values.index(close)].instance_id == known.instance_id
        assert result[values.index(farther)].instance_id != known.instance_id


def test_track_movement_updates_position_not_lifetime_mean_and_scope_blocks_reused_track():
    seq = Sequence()
    first = seq.add(observation(track='t'))[0]
    moved = seq.add(observation(x=3,track='t',source='move'),seconds=1)[0]
    assert moved.instance_id == first.instance_id
    assert moved.pose.position.x == 3
    assert moved.dynamic_state == InstanceDynamicState.DYNAMIC
    other = seq.add(observation(x=8,track='t',source='reuse'),seconds=2,session='new')[0]
    assert other.instance_id != moved.instance_id


def test_map_frame_and_assigned_asset_isolation_but_runtime_session_restart_allowed():
    seq = Sequence()
    first = seq.add(observation(),asset='asset-a')[0]
    same = seq.add(observation(source='next'),seconds=86400,session='restart',asset='asset-a')[0]
    assert same.instance_id == first.instance_id
    replaced = seq.add(observation(source='replacement'),seconds=86401,asset='asset-b')[0]
    assert replaced.instance_id != first.instance_id
    other_map = seq.add(observation(source='elsewhere'),map_name='office',seconds=86402)[0]
    assert other_map.instance_id not in (first.instance_id,replaced.instance_id)
    other_frame = seq.add(observation(frame='odom',source='odom'),seconds=86403)[0]
    assert other_frame.instance_id not in (first.instance_id,replaced.instance_id)


def test_geometryless_fallback_requires_scoped_track_and_never_moves_saved_geometry():
    seq = Sequence()
    first = seq.add(observation(track='t'))[0]
    weak = seq.add(observation(x=20,track='t',fallback=True,source='weak'),seconds=1)[0]
    assert weak.instance_id == first.instance_id
    assert weak.pose == first.pose
    assert weak.metadata['object_extent'] == first.metadata['object_extent']
    unrelated = seq.add(observation(x=20,fallback=True,source='unknown'),seconds=2)[0]
    assert unrelated.instance_id != first.instance_id


def test_centroid_only_projection_uses_measured_ray_support_not_fixed_meter_radius():
    seq = Sequence()
    first = seq.add(observation(size=None,ray_origin_map=[0,3,.5],ray_cone_half_angle_rad=.025))[0]
    again = seq.add(observation(x=2.02,size=None,source='again',
        ray_origin_map=[0,3,.5],ray_cone_half_angle_rad=.025),seconds=3600)[0]
    assert again.instance_id == first.instance_id
    neighbor = seq.add(observation(x=2.2,size=None,source='neighbor',
        ray_origin_map=[0,3,.5],ray_cone_half_angle_rad=.025),seconds=3601)[0]
    assert neighbor.instance_id != first.instance_id
    assert 'object_extent' not in again.metadata


def test_legacy_centroid_reacquired_using_new_size_and_triangulation_residual():
    seq = Sequence()
    first = seq.add(observation(size=None))[0]
    repeat = seq.add(observation(x=2.02,source='size_now'),seconds=3600)[0]
    assert repeat.instance_id == first.instance_id
    points = Sequence()
    first = points.add(observation(size=None,triangulation_residual_m=.04))[0]
    repeat = points.add(observation(x=2.02,size=None,triangulation_residual_m=.04,source='next'),seconds=3600)[0]
    assert repeat.instance_id == first.instance_id


def test_repeated_source_is_idempotent_and_multiple_sources_update_same_id_within_batch():
    seq = Sequence()
    first = seq.add(observation(track='t'))[0]
    replay = seq.add(observation(track='t'),seconds=1)[0]
    assert replay.instance_id == first.instance_id
    assert replay.observation_count == 1
    assert replay.last_seen_ts == first.last_seen_ts
    assert len(replay.observation_window) == 1
    batch = seq.add(observation(source='second',track='t'),observation(source='third',track='t'),seconds=2)
    assert [item.instance_id for item in batch] == [first.instance_id]*2
    assert seq.instances[first.instance_id].observation_count == 3
    assert len(seq.instances[first.instance_id].observation_window) == 3


def test_old_automatic_aging_does_not_erase_identity_but_explicit_removal_does():
    seq = Sequence()
    first = seq.add(observation())[0]
    seq.add(seconds=30*86400)
    assert seq.instances[first.instance_id].lifecycle_state == InstanceLifecycleState.REMOVED
    revived = seq.add(observation(source='return'),seconds=31*86400)[0]
    assert revived.instance_id == first.instance_id
    assert "lifecycle_aged" not in revived.metadata
    removed = revived.model_copy(update={'lifecycle_state':InstanceLifecycleState.REMOVED})
    seq.instances[removed.instance_id] = removed
    replacement = seq.add(observation(source='after_remove'),seconds=31*86400+1)[0]
    assert replacement.instance_id != first.instance_id
    assert seq.instances[first.instance_id].lifecycle_state == InstanceLifecycleState.REMOVED
    assert seq.add(observation(source='later'),seconds=31*86400+2)[0].instance_id == replacement.instance_id


def test_identical_historical_rows_use_stable_oldest_without_deleting_other_rows():
    seq = Sequence()
    first = seq.add(observation())[0]
    other = first.model_copy(update={'instance_id':first.instance_id[:-8]+'eeeeeeee'})
    seq.instances[other.instance_id] = other
    expected = min(first.instance_id,other.instance_id)
    result = seq.add(observation(source='next'),seconds=60)[0]
    assert result.instance_id == expected
    assert len(seq.instances) == 2
    assert sorted(item.observation_count for item in seq.instances.values()) == [1,2]


def test_large_history_scores_local_candidates_not_every_same_label_object():
    seq = Sequence()
    base = seq.add(observation())[0]
    seq.instances = {base.instance_id[:-8]+f'{i:08x}': base.model_copy(update={
        'instance_id':base.instance_id[:-8]+f'{i:08x}',
        'pose':Pose(frame_id='map',position=Vector3(x=2+i*2,y=3,z=.5))}) for i in range(2000)}
    original = seq.service._association_score
    with patch.object(seq.service,'_association_score',wraps=original) as score:
        seq.add(observation(source='next'),seconds=1)
    assert score.call_count == 1
    assert len(seq.instances) == 2000


def test_different_colors_are_distinct_even_with_overlapping_geometry():
    seq = Sequence()
    first = seq.add(observation(color='red'))[0]
    other = seq.add(observation(color='blue',source='next'),seconds=1)[0]
    assert first.instance_id != other.instance_id


def test_nonfinite_position_cannot_match_known_object():
    seq = Sequence()
    first = seq.add(observation())[0]
    bad = seq.add(observation(x=math.nan,track='t',source='invalid'),seconds=1)[0]
    assert bad.instance_id != first.instance_id


def test_same_camera_source_track_modalities_share_identity_but_different_tracks_do_not():
    seq = Sequence()
    full = observation(track='same')
    projected = replace(full,attributes={**full.attributes,'projection_method':'support_points_map',
        'bbox_3d_map':{'frame_id':'map','center':{'x':2.,'y':3.,'z':.5},
                       'extent_m':{'x':.06,'y':.06,'z':.1}}})
    first,paired = seq.add(full,projected)
    assert first.instance_id == paired.instance_id
    assert paired.observation_count == 1
    assert len(seq.instances) == 1
    other = seq.add(replace(full,source_observation_id='next',track_id='one'),
                    replace(projected,source_observation_id='next',track_id='two'),seconds=1)
    assert len({item.instance_id for item in other}) == 2


@pytest.mark.parametrize('size',[(20.,.01,.01),(.01,20.,.01),(.01,.01,20.),(.08,.08,.12)])
def test_conservative_index_keeps_large_thin_and_exact_boundary_matches(size):
    from services.memory.spatial_candidate_index import SpatialCandidateIndex
    seq = Sequence()
    first = seq.add(observation(size=size))[0]
    right = seq.service._spatial_geometry(first)
    # 使用相同计算构造精确关联边界，含与最长尺寸正交的细轴容差。
    for axis in range(3):
        gate = .5*right[1][axis]+right[2]
        positions={'x':2.,'y':3.,'z':.5}
        positions['xyz'[axis]] += gate
        item=observation(size=size,source='boundary',**positions)
        assert seq.service._geometry_fit(item,first) is not None
        index=SpatialCandidateIndex([(first.instance_id,seq.service._spatial_bounds(first))])
        assert list(index.query(seq.service._spatial_bounds(item))) == [first.instance_id]


def test_interval_index_includes_touching_bounds_and_large_enclosing_intervals():
    from services.memory.spatial_candidate_index import SpatialCandidateIndex
    index=SpatialCandidateIndex([('touching',((0.,0.,0.),(1.,1.,1.))),
        ('enclosing',((-100.,-100.,-100.),(100.,100.,100.))),
        ('separate',((3.,3.,3.),(4.,4.,4.)))])
    assert set(index.query(((1.,1.,1.),(2.,2.,2.)))) == {'touching','enclosing'}
