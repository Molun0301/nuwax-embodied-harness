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


class CatalogEvidenceTests(assets.AssetCatalogTests):
    def begin(self, kind):
        request, receipt = activation_fixture(kind)
        identity = assets.AssetIdentityV2(**request['asset_identity'])
        shared = dict(target_map_name='map', target_asset_identity=identity, target_checkpoint_id='a' * 32,
            activation_request_id=request['activation_request_id'], activation_request=request)
        self.catalog.begin_assigned_map_activation(previous_map_name=None, target_map_name='map',
            target_asset_identity=identity, target_checkpoint_id='a' * 32,
            load_memory_history=False, activation_request=request)
        self.catalog.update_assigned_map_activation_phase('backend_dispatching', **shared, expected_phases={'prepared'})
        return shared, receipt

    def test_current_planar_policy_persists_replays_and_rejects_cloud_policy(self):
        shared, receipt = self.begin('slam_toolbox')
        options = dict(activation_receipt=receipt, relocalization_policy_v2=current_assigned_planar_relocalization_policy(),
            relocalization_target_available=True, activation_receipt_disposition=2, activation_receipt_disposition_name='sealed')
        result = self.catalog.record_assigned_map_activation_evidence(**shared, **options, expected_phases={'backend_dispatching'})
        self.assertEqual(result['phase'], 'backend_applied')
        self.assertEqual(result['backend_evidence']['relocalization_policy_v2'], current_assigned_planar_relocalization_policy())
        self.assertEqual(self.catalog.get_map_activation()['backend_evidence']['activation_receipt_v2'], receipt.to_dict())
        replay = self.catalog.record_assigned_map_activation_evidence(**shared, **options, expected_phases={'backend_applied'})
        self.assertEqual(replay, result)
        with self.assertRaises(ValueError):
            self.catalog.record_assigned_map_activation_evidence(**shared, **dict(options,
                relocalization_policy_v2=current_assigned_relocalization_policy()), expected_phases={'backend_applied'})
        self.assertEqual(self.catalog.get_map_activation(), result)

    def test_cloud_keeps_exact_nonempty_policy_contract(self):
        shared, receipt = self.begin('voxel')
        options = dict(activation_receipt=receipt, relocalization_target_available=True,
            activation_receipt_disposition=2, activation_receipt_disposition_name='sealed')
        policy = current_assigned_relocalization_policy()
        result = self.catalog.record_assigned_map_activation_evidence(**shared, **options,
            relocalization_policy_v2=policy, expected_phases={'backend_dispatching'})
        self.assertEqual(result['backend_evidence']['relocalization_policy_v2'], policy)
        self.assertEqual(set(result['backend_evidence']['activation_receipt_v2']['target']),
            {'activation', 'source_cloud_member', 'occupancy_receipt', 'cloud_receipt'})

        document = json.loads(self.catalog._activation_journal_path.read_text())
        document['backend_evidence']['relocalization_policy_v2'] = None
        self.catalog._activation_journal_path.write_text(json.dumps(document))
        with self.assertRaises((TypeError, ValueError)):
            self.catalog.get_map_activation()


if __name__ == '__main__':
    result = unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite(
        CatalogEvidenceTests(name) for name in CatalogEvidenceTests.__dict__ if name.startswith('test_')))
    raise SystemExit(not result.wasSuccessful())
