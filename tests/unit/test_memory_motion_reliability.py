"""生产实例关联的可靠运动证据和动态导航约束回归。"""
from dataclasses import replace
from datetime import timedelta
import pytest
from contracts.base import utc_now
from contracts.geometry import Pose, Vector3, Quaternion
from contracts.spatial_memory import InstanceDynamicState
from services.memory.instance_association_service import InstanceAssociationService
from services.memory.vision_to_map_projection_service import ProjectedObservation


def observation(x=8.385, *, fallback=False, label='refrigerator', **attributes):
    return ProjectedObservation(
        label=label, score=.95,
        pose=Pose(frame_id='map', position=Vector3(x=x, y=-1.029 if not fallback else 0, z=.627 if not fallback else 0), orientation=Quaternion(w=1)),
        camera_id='front_camera', track_id='fridge_track', instance_type='object',
        source_observation_id='observation', source_artifact_ids=(), semantic_region_id=None,
        topo_node_id=None, semantic_labels=(), visual_labels=(label,),
        attributes={'projection_method':'current_pose_fallback' if fallback else 'support_points_map',
                    'support_point_count':0 if fallback else 18,
                    'projection_confidence':.2 if fallback else .99,
                    'projection_degradation_reason':'current_pose_fallback_no_geometry' if fallback else '',
                    'pose_quality_level':'fallback' if fallback else 'high', **attributes},
    )


class Sequence:
    def __init__(self):
        self.service = InstanceAssociationService(text_embedder=None)
        self.instances = {}
        self.now = utc_now()
        self.index = 0
    def add(self, item):
        self.index += 1
        self.instances, outcomes = self.service.associate(
            [replace(item, source_observation_id=f'obs_{self.index}')],
            existing_instances=self.instances, map_name='house',
            now=self.now + timedelta(seconds=self.index),
         admission="explicit")
        if not self.instances:
            assert item.attributes['projection_method']=='current_pose_fallback'
            assert outcomes[0].instance is None
            return None
        assert len(self.instances)==1
        return outcomes[0].instance or next(iter(self.instances.values()))


@pytest.mark.parametrize('reliable_count', [0, 1])
def test_fallback_observer_positions_never_supply_motion_or_static_evidence(reliable_count):
    seq = Sequence()
    if reliable_count:
        seq.add(observation())
    for i in range(30):
        obj = seq.add(observation(float(i), fallback=True))
        if not reliable_count:
            assert obj is None and not seq.instances
            continue
        assert obj.dynamic_state == InstanceDynamicState.UNKNOWN
        assert obj.metadata['dynamic_observation_count'] == reliable_count
        assert obj.metadata['dynamic_window_displacement_m'] == 0
        assert len(obj.observation_window) <= 8
    if reliable_count:
        assert obj.pose.position.x == pytest.approx(8.385)
        assert not obj.metadata['observation_only']
    else:
        assert obj is None


def test_static_geometric_history_survives_many_fast_frames_bounded():
    seq = Sequence()
    for _ in range(3):obj = seq.add(observation())
    assert obj.dynamic_state == InstanceDynamicState.STATIC
    for i in range(100):
        obj = seq.add(observation(-i, fallback=True))
        assert obj.dynamic_state == InstanceDynamicState.STATIC
        assert obj.metadata['dynamic_observation_count'] == 3
        assert obj.metadata['dynamic_window_duration_sec'] == 2
        assert len(obj.observation_window) <= 8
        assert obj.observation_window[-1].metadata['projection_method'] == 'support_points_map'
    assert obj.metadata.get('non_navigable_reason') not in {'dynamic_entity', 'quasi_static_entity'}


def test_real_motion_stays_excluded_through_fast_frames_then_reliable_rest_recovers():
    seq = Sequence()
    seq.add(observation(8.0))
    obj = seq.add(observation(8.3))
    assert obj.dynamic_state == InstanceDynamicState.DYNAMIC
    for i in range(30):
        obj = seq.add(observation(i, fallback=True))
        assert obj.dynamic_state == InstanceDynamicState.DYNAMIC
        assert obj.metadata['dynamic_window_speed_mps'] == pytest.approx(.3)
        assert obj.metadata['navigable'] is False
    for _ in range(10):obj = seq.add(observation(8.3))
    assert obj.dynamic_state == InstanceDynamicState.STATIC
    assert obj.metadata.get('non_navigable_reason') not in {'dynamic_entity', 'quasi_static_entity'}
    assert obj.metadata.get('navigable') is not False
    assert 'navigation_exclusion_scope' not in obj.metadata
    assert 'local_retry_policy' not in obj.metadata


@pytest.mark.parametrize('reason', ['geometry_missing', 'operator_blocked'])
def test_dynamic_recovery_preserves_other_navigation_rejection(reason):
    seq = Sequence()
    seq.add(observation(8.0, navigable=False, non_navigable_reason=reason))
    obj = seq.add(observation(8.3))
    assert obj.dynamic_state == InstanceDynamicState.DYNAMIC
    for _ in range(10):obj = seq.add(observation(8.3))
    assert obj.dynamic_state == InstanceDynamicState.STATIC
    assert obj.metadata['navigable'] is False
    assert obj.metadata['non_navigable_reason'] == reason


def test_legacy_dynamic_reason_is_not_restored_by_fallback_pose_preservation():
    seq = Sequence()
    for _ in range(3):obj = seq.add(observation())
    obj.metadata.update(navigable=False, non_navigable_reason='dynamic_entity', navigation_exclusion_scope='long_term_static_target', local_retry_policy='retry_after_dynamic_obstacle_clears')
    obj = seq.add(observation(0, fallback=True,ray_origin_map=[0,-1.029,.627],ray_dir_map=[1,0,0],ray_cone_half_angle_rad=.02))
    assert obj.dynamic_state == InstanceDynamicState.STATIC
    assert obj.metadata.get('non_navigable_reason') != 'dynamic_entity'
    assert obj.metadata.get('navigable') is not False
    assert 'local_retry_policy' not in obj.metadata


def test_dynamic_window_keeps_repeated_observations_and_excludes_other_frames():
    service = InstanceAssociationService(text_embedder=None)
    now = utc_now()
    first = service._build_observation_window([], observation(), now=now, map_name='house')
    second = service._build_observation_window(first, observation(), now=now+timedelta(seconds=1), map_name='house')
    assert service._dynamic_window_metrics(second)['observation_count'] == 2
    other_frame = replace(observation(20), pose=Pose(frame_id='odom', position=Vector3(x=20)))
    third = service._build_observation_window(second, other_frame, now=now+timedelta(seconds=2), map_name='house')
    metrics = service._dynamic_window_metrics(third)
    assert metrics['observation_count'] == 1
    assert metrics['max_displacement_m'] == 0


@pytest.mark.parametrize('nested', [False, True])
def test_current_static_classification_clears_earlier_eligibility_rejection(nested):
    seq = Sequence()
    obj = seq.add(observation(label='potted plant'))
    assert obj.dynamic_state == InstanceDynamicState.UNKNOWN
    obj.metadata.update(navigable=False, non_navigable_reason='not_long_term_static_target')
    if nested:
        obj.metadata.update(navigable=False, non_navigable_reason='dynamic_entity',
            dynamic_navigation_previous={'navigable': False, 'non_navigable_reason': 'not_long_term_static_target'})
    for _ in range(3):
        obj = seq.add(observation(label='potted plant'))
    assert obj.dynamic_state == InstanceDynamicState.STATIC
    assert obj.metadata['long_term_static_target_eligible'] is True
    assert obj.metadata.get('navigable') is not False
    assert obj.metadata.get('non_navigable_reason') not in {
        'dynamic_entity', 'quasi_static_entity', 'not_long_term_static_target'}
