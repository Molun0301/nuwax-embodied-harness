"""AR5 Python consumers use real storage and protobufs; only the RPC peer is fake.

Set NEH_HEIGHTFIELD_ARCHIVE_DIR to the existing Rust qualification artifacts to
exercise their unmodified bytes. These tests do not establish native activation.
"""
import copy
import json
import os
import struct
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import tests.test_checkpoint_identity_v2 as identity
import tests.test_heightfield_archive as heightfield
import tests.test_legacy_cloud_import as legacy
from services.geometry import geometry_runtime_provider as provider
from services.geometry import mapping_backend_checkpoint_store as checkpoint_store
from services.geometry.checkpoint_payload import FrozenPayload
from services.geometry.fine_map_archive import read_fine_archive_root
from services.mapping import asset_store


def response_for(payload):
    """Bind the existing canonical checkpoint fixture to the archive's receipts."""
    _, root, _ = read_fine_archive_root(payload)
    domain = root['domain']
    response = identity.response_fixture()
    response.ClearField('dense_cloud_artifact')
    response.source_session_id = domain['session']
    response.source_session_generation = domain['generation']
    response.created_at_ns = root['source_timestamp_ns'] + 1
    occupancy, cloud = response.occupancy_receipt, response.cloud_receipt
    occupancy.session_id = domain['session']
    occupancy.session_generation = domain['generation']
    occupancy.source_timestamp_ns = root['source_timestamp_ns']
    cloud.session_generation = domain['generation']
    cloud.cloud_revision = root['revision']
    cloud.source_timestamp_ns = root['source_timestamp_ns']
    response.canonical_occupancy_checkpoint.meta.timestamp_ns = root['source_timestamp_ns']
    response.canonical_cloud_checkpoint.source_revision = root['revision']
    for member in (response.backend_identity, response.occupancy_identity,
                   response.canonical_cloud_checkpoint.member_identity,
                   response.fine_map_artifact.member_identity):
        member.occupancy_receipt.CopyFrom(occupancy)
        member.cloud_receipt.CopyFrom(cloud)
    fine = response.fine_map_artifact
    fine.size = len(payload)
    fine.member_identity.frame_generation = domain['frame_generation']
    fine.member_identity.producer_generation = domain['producer_generation']
    response.payload = b'NWMBCKP\0' + struct.pack('<IBBHQQQQddd',
        1, 0, 0, 0, 16, 1, 1, root['source_timestamp_ns'], 1, 2, 3)
    response.payload_size = len(response.payload)
    return response


def peer_for(payload, versions=(5,)):
    response = response_for(payload)
    stub = identity.TransferStub(response)
    stub.entries = {response.fine_map_artifact.id: [
        copy.deepcopy(response.fine_map_artifact), bytearray(payload), True]}
    client = identity.runtime(stub)
    client._assigned_fine_archive_versions = frozenset(versions)
    return client, stub, response


class HeightfieldArchiveConsumerTests(unittest.TestCase):
    def setUp(self):
        # Existing sentinels prohibit application hashes throughout actual I/O.
        self.fixture = identity.IdentityV2Tests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)

    def roundtrip(self, payload, directory):
        version = read_fine_archive_root(payload)[1]['version']
        client, stub, response = peer_for(payload, (version,))
        bundle = client.export_mapping_backend_checkpoint(checkpoint_schema_version=2)
        self.assertIsInstance(bundle.checkpoint.fine_archive, FrozenPayload)
        retained_owner = bundle.checkpoint.fine_archive
        directory.mkdir(exist_ok=True)
        stored = bundle.publish_to(directory)
        self.assertIs(stored.checkpoint.fine_archive, retained_owner)
        reopened = provider.reopen_mapping_backend_checkpoint(directory, 'voxel')
        self.assertEqual(bytes(reopened.checkpoint.fine_archive), payload)
        loaded = checkpoint_store.MappingBackendCheckpointStore(directory).load('voxel', schema_version=2)
        self.assertEqual(provider._mapping_backend_checkpoint_proto(loaded), response)
        self.assertEqual(loaded.checkpoint.fine_reference, bundle.checkpoint.fine_reference)
        uploaded = client._upload_graph_artifact(reopened.checkpoint,
            payload=reopened.checkpoint.fine_archive, reference=reopened.checkpoint.fine_reference)
        self.assertEqual(bytes(stub.entries[uploaded.id][1]), payload)
        downloaded = client._download_graph_artifact(uploaded, provider.MAX_FINE_ARCHIVE_BYTES)
        self.assertEqual(bytes(downloaded), payload)
        self.assertEqual(read_fine_archive_root(downloaded)[1], read_fine_archive_root(payload)[1])
        self.assertEqual(stub.exports, [2])

    def legacy_import(self, payload):
        fixture = legacy.LegacyCloudImportTests()
        fixture.setUp()
        try:
            fixture.prepare(dense=False)
            _, root, _ = read_fine_archive_root(payload)
            domain = root['domain']
            source = fixture.bundle.geometry_asset.metadata['mapping_backend_checkpoint']
            manifest = fixture.current / source['manifest_path']
            document = json.loads(manifest.read_text())
            document.update(source_session_id=domain['session'],
                source_session_generation=domain['generation'], created_at_ns=root['source_timestamp_ns'] + 1)
            occupancy = document['canonical_receipts']['occupancy']
            occupancy.update(session_id=domain['session'], session_generation=domain['generation'],
                source_timestamp_ns=root['source_timestamp_ns'])
            document['canonical_receipts']['cloud'].update(session_generation=domain['generation'],
                cloud_revision=root['revision'], source_timestamp_ns=root['source_timestamp_ns'])
            document['canonical_assets']['occupancy']['timestamp_ns'] = root['source_timestamp_ns']
            document['canonical_assets']['cloud']['source_cloud_revision'] = root['revision']
            manifest.write_bytes(checkpoint_store._encode_json(document))
            fixture.add_fine_archive(payload)
            before = legacy.inventory(fixture.current)
            result = fixture.import_current()
            self.assertIsInstance(result, asset_store._AssignedActivationRead)
            checkpoint = result.stored_checkpoint.checkpoint
            self.assertEqual(bytes(checkpoint.fine_archive), payload)
            member = checkpoint.fine_reference.member_identity
            self.assertEqual((member.frame_generation, member.producer_generation),
                (domain['frame_generation'], domain['producer_generation']))
            self.assertEqual(member.cloud_receipt, checkpoint.cloud_receipt)
            self.assertEqual(checkpoint.fine_reference.legacy_alias, b'')
            self.assertEqual(legacy.inventory(fixture.current), before)
            with patch.object(asset_store.uuid, 'uuid4', side_effect=AssertionError('retry allocation')):
                retried = fixture.import_current()
            self.assertEqual(retried.bundle.asset_identity, result.bundle.asset_identity)
            self.assertEqual(bytes(retried.stored_checkpoint.checkpoint.fine_archive), payload)
        finally:
            fixture.doCleanups()

    def test_ar5_export_store_reopen_transfer_keeps_exact_owner_and_bytes(self):
        self.roundtrip(heightfield.archive(heightfield.full()), self.fixture.root)

    def test_ar5_schema1_asset_import_preserves_archive_and_reuses_publication(self):
        self.legacy_import(heightfield.archive(heightfield.full()))

    def test_ar5_capability_is_required_at_transfer_boundary_not_offline_storage(self):
        payload = heightfield.archive(heightfield.full())
        # Existing AR4 semantics: export/storage are allowed, activation/upload
        # require the connected runtime's advertised reader capability.
        client, stub, _ = peer_for(payload, (1, 2, 3, 4, 99))
        bundle = client.export_mapping_backend_checkpoint(checkpoint_schema_version=2)
        stored = bundle.publish_to(self.fixture.root)
        origin = client.capture_assigned_activation_origin()
        self.assertFalse(client.supports_assigned_fine_archive_version(5, expected_origin=origin))
        self.assertFalse(client.supports_assigned_fine_archive_version(99, expected_origin=origin))
        before = len(stub.requests)
        with self.assertRaisesRegex(RuntimeError, 'does not support assigned AR5'):
            client.require_assigned_fine_archive_support(bundle.checkpoint, expected_origin=origin)
        with self.assertRaisesRegex(RuntimeError, 'does not support assigned AR5'):
            client._upload_graph_artifact(bundle.checkpoint, payload=bundle.checkpoint.fine_archive)
        client._assigned_activation_attempt = None
        with self.assertRaisesRegex(RuntimeError, 'does not support assigned AR5'):
            client.activate_assigned_platform_map(stored_checkpoint=stored,
                activation_request=dict(activation_request_id='activate:ar5', session_id='mapload:ar5',
                    map_name='map', checkpoint_id=stored.checkpoint.checkpoint_id,
                    asset_identity={'asset_id': 'd' * 32}, expected_runtime_generation=7))
        self.assertEqual(len(stub.requests), before)
        client._assigned_fine_archive_versions = frozenset((5,))
        self.assertTrue(client.supports_assigned_fine_archive_version(5, expected_origin=origin))
        client.require_assigned_fine_archive_support(bundle.checkpoint, expected_origin=origin)
        client._map_connection_epoch += 1
        with self.assertRaisesRegex(RuntimeError, 'connection changed'):
            client.require_assigned_fine_archive_support(bundle.checkpoint, expected_origin=origin)
        client, stub, response = peer_for(payload, ())
        with self.assertRaisesRegex(RuntimeError, 'does not support assigned AR5'):
            client._download_graph_artifact(response.fine_map_artifact, provider.MAX_FINE_ARCHIVE_BYTES)
        self.assertEqual([request.action for request in stub.requests], [4, 5])
        self.assertEqual(stub.entries, {})

    def test_ar5_malformed_and_receipt_mismatch_keep_original_rejection(self):
        payload = heightfield.archive(heightfield.full())
        for malformed in (payload[:-1], heightfield.archive(heightfield.full()[:-1])):
            client, stub, _ = peer_for(malformed)
            with self.assertRaises(ValueError):
                client.export_mapping_backend_checkpoint(checkpoint_schema_version=2)
            self.assertEqual(stub.entries, {})
        response = response_for(payload)
        response.fine_map_artifact.member_identity.frame_generation += 1
        with self.assertRaisesRegex(ValueError, '原占据/点云提交不一致'):
            provider._checkpoint_export_bundle_from_proto(response, fine_payload=payload)
        response.fine_map_artifact.member_identity.frame_generation -= 1
        with self.assertRaisesRegex(ValueError, 'supported archive'):
            provider._checkpoint_export_bundle_from_proto(response,
                fine_payload=b'NUWAXAR6' + payload[8:])

    def test_actual_rust_archives_survive_checkpoint_and_asset_consumers(self):
        directory = os.environ.get('NEH_HEIGHTFIELD_ARCHIVE_DIR')
        if not directory:
            self.skipTest('set NEH_HEIGHTFIELD_ARCHIVE_DIR to Rust storage qualification output')
        for name in ('cross-block.ar5', 'zero-mass-withdrawn.ar5', 'mixed.ar5',
                     'full-1024.ar5', 'legacy.ar4'):
            with self.subTest(name=name):
                payload = (Path(directory) / name).read_bytes()
                self.roundtrip(payload, self.fixture.root / name)
                self.legacy_import(payload)
                print('AR5_CONSUMER_RUST_ARCHIVE name=%s bytes=%d exact_roundtrip=PASS' % (name, len(payload)))
        payload = (Path(directory) / 'invalid-rebound.ar5').read_bytes()
        client, stub, _ = peer_for(payload)
        with self.assertRaisesRegex(ValueError, '空间绑定'):
            client.export_mapping_backend_checkpoint(checkpoint_schema_version=2)
        self.assertEqual(stub.entries, {})


if __name__ == '__main__':
    unittest.main()
