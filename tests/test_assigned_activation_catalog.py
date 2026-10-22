"""Explicit assigned durable records; no native process, RPC or payload hash."""
from copy import deepcopy
import unittest

from contracts.map_workspace import AssetIdentityV2
from contracts.map_activation import AssignedActivationReceipt

ASSET = AssetIdentityV2(asset_id='a' * 32)
CHECKPOINT = 'b' * 32


def receipt(request_id, *, checkpoint=CHECKPOINT, epoch=9):
    request = dict(activation_request_id=request_id, session_id=request_id, map_name='room',
                   asset_identity=ASSET.model_dump(), checkpoint_id=checkpoint, expected_runtime_generation=3)
    activation = dict(request=deepcopy(request), runtime_generation=3, lifecycle_epoch=epoch,
                      session_generation=7, backend_kind='voxel', backend_generation=2)
    source_occ = dict(commit_id=dict(boot_nonce='c' * 32, sequence=4), session_id='源地图',
                      session_generation=6, occupancy_revision=4, source_timestamp_ns=21,
                      backend_kind='voxel', backend_generation=1)
    source_cloud = dict(commit_id=dict(boot_nonce='c' * 32, sequence=3), session_generation=6,
                        occupancy_revision=3, cloud_revision=8, source_timestamp_ns=20, point_count=2)
    source = dict(checkpoint_id=checkpoint, role='canonical_cloud', occupancy_receipt=source_occ,
                  cloud_receipt=source_cloud, frame_id='map', frame_generation=0, producer_generation=0)
    occupancy = dict(commit_id=dict(boot_nonce='d' * 32, sequence=17), session_id=request_id,
                     session_generation=7, occupancy_revision=19, source_timestamp_ns=55,
                     backend_kind='voxel', backend_generation=2)
    cloud = dict(commit_id=deepcopy(occupancy['commit_id']), session_generation=7,
                 occupancy_revision=19, cloud_revision=22, source_timestamp_ns=55, point_count=2)
    return dict(request=request, activation=activation,
                target=dict(activation=deepcopy(activation), source_cloud_member=source,
                            occupancy_receipt=occupancy, cloud_receipt=cloud))


class AssignedReceiptTests(unittest.TestCase):
    def test_owned_descriptor_is_immutable_and_has_no_payload(self):
        original = receipt('mapload:' + '1' * 32)
        owner = AssignedActivationReceipt.admit(original)
        original['request']['map_name'] = 'mutated'
        view = owner.to_dict(); view['activation']['lifecycle_epoch'] = 99
        self.assertEqual(owner.to_dict()['request']['map_name'], 'room')
        self.assertEqual(owner.to_dict()['activation']['lifecycle_epoch'], 9)
        self.assertTrue(owner.exact_replay_of(AssignedActivationReceipt.admit(owner.to_dict())))
        with self.assertRaises(Exception): owner.request_id = 'changed'

    def test_source_member_uses_existing_native_checkpoint_translation(self):
        from services.geometry import geometry_runtime_provider as provider
        from services.geometry.mapping_backend_checkpoint_store import _identity_document
        candidate = receipt('mapload:'+'1'*32)
        source = candidate['target']['source_cloud_member']
        def occupancy(value):
            return provider.pb2.CanonicalOccupancyCommitReceipt(
                commit_id=provider.pb2.MappingCommitIdentity(
                    boot_nonce=bytes.fromhex(value['commit_id']['boot_nonce']), sequence=value['commit_id']['sequence']),
                **{key: item for key, item in value.items() if key != 'commit_id'})
        def cloud(value):
            return provider.pb2.CanonicalCloudCommitReceipt(
                commit_id=provider.pb2.MappingCommitIdentity(
                    boot_nonce=bytes.fromhex(value['commit_id']['boot_nonce']), sequence=value['commit_id']['sequence']),
                **{key: item for key, item in value.items() if key != 'commit_id'})
        wire = provider.pb2.CheckpointMemberIdentity(checkpoint_id=CHECKPOINT, role=3, frame_id='map',
            occupancy_receipt=occupancy(source['occupancy_receipt']), cloud_receipt=cloud(source['cloud_receipt']))
        translated = _identity_document(provider._member_identity_from_proto(wire))
        self.assertEqual(translated, source)
        candidate['target']['source_cloud_member'] = translated
        self.assertEqual(AssignedActivationReceipt.admit(candidate).to_dict()['target']['source_cloud_member']['role'],
                         'canonical_cloud')

    def test_direct_owner_construction_cannot_skip_admission(self):
        with self.assertRaises(TypeError): AssignedActivationReceipt()

    def test_receipt_binding_mutations_and_mixed_fields_rejected(self):
        mutations = [lambda d: d['request'].update(checkpoint_id='e'*32),
                     lambda d: d['target']['cloud_receipt'].update(point_count=3),
                     lambda d: d['target']['occupancy_receipt']['commit_id'].update(sequence=18),
                     lambda d: d['target']['source_cloud_member'].update(role='fine_map'),
                     lambda d: d['target']['source_cloud_member'].update(producer_generation=1),
                     lambda d: d['activation'].update(runtime_generation=4),
                     lambda d: d.update(asset_manifest_sha256='0'*64),
                     lambda d: d['target'].update(xyz_f32_le=object()),
                     lambda d: d['request'].update(expected_runtime_generation=True)]
        for mutate in mutations:
            candidate = receipt('mapload:'+'1'*32); mutate(candidate)
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                AssignedActivationReceipt.admit(candidate)




if __name__ == '__main__': unittest.main()
