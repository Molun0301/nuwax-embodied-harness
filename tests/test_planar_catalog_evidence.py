"""Real durable catalog evidence admission for planar and cloud, with hashes forbidden."""
import copy
import json
import unittest
import tests.test_planar_asset_catalog as assets
from contracts.map_activation import AssignedActivationReceipt
from contracts.map_relocalization import current_assigned_relocalization_policy, current_assigned_planar_relocalization_policy


def activation_fixture(kind):
    source_occupancy = assets.occupancy_receipt(kind)
    source_cloud = None if kind == 'slam_toolbox' else dict(commit_id=source_occupancy['commit_id'], session_generation=3,
        occupancy_revision=6, cloud_revision=7, source_timestamp_ns=11, point_count=1)
    source = assets.store._identity_document(assets.store.CheckpointMemberIdentity('a' * 32,
        'planar_graph' if kind == 'slam_toolbox' else 'canonical_cloud', source_occupancy, source_cloud, 'map'))
    request = dict(activation_request_id='activate:test', session_id='loaded:test', map_name='map',
        asset_identity=dict(asset_id='d' * 32), checkpoint_id='a' * 32, expected_runtime_generation=17)
    activation = dict(request=request, runtime_generation=17, lifecycle_epoch=8, session_generation=4,
        backend_kind=kind, backend_generation=3)
    occupancy = dict(commit_id=dict(boot_nonce='50' * 16, sequence=10), session_id='loaded:test', session_generation=4,
        occupancy_revision=8, source_timestamp_ns=11, backend_kind=kind, backend_generation=3)
    target = dict(activation=activation, occupancy_receipt=occupancy)
    if kind == 'slam_toolbox':
        target['source_graph_member'] = source
    else:
        target['source_cloud_member'] = source
        target['cloud_receipt'] = dict(commit_id=occupancy['commit_id'], session_generation=4,
            occupancy_revision=8, cloud_revision=9, source_timestamp_ns=11, point_count=source['cloud_receipt']['point_count'])
    return request, AssignedActivationReceipt.admit(dict(request=request, activation=activation, target=target))
