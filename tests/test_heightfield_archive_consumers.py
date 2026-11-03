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
        bundle = client.export_mapping_backend_checkpoint()
        self.assertIsInstance(bundle.checkpoint.fine_archive, FrozenPayload)
        retained_owner = bundle.checkpoint.fine_archive
        directory.mkdir(exist_ok=True)
        stored = bundle.publish_to(directory)
        self.assertIs(stored.checkpoint.fine_archive, retained_owner)
        reopened = provider.reopen_mapping_backend_checkpoint(directory, 'voxel')
        self.assertEqual(bytes(reopened.checkpoint.fine_archive), payload)
        loaded = checkpoint_store.MappingBackendCheckpointStore(directory).load('voxel', schema_version=2)
        self.assertEqual(provider._mapping_backend_checkpoint_v2_proto(loaded), response)
        self.assertEqual(loaded.checkpoint.fine_reference, bundle.checkpoint.fine_reference)
        uploaded = client._upload_graph_artifact(reopened.checkpoint,
            payload=reopened.checkpoint.fine_archive, reference=reopened.checkpoint.fine_reference)
        self.assertEqual(bytes(stub.entries[uploaded.id][1]), payload)
        downloaded = client._download_graph_artifact(uploaded, provider.MAX_FINE_ARCHIVE_BYTES)
        self.assertEqual(bytes(downloaded), payload)
        self.assertEqual(read_fine_archive_root(downloaded)[1], read_fine_archive_root(payload)[1])
        self.assertEqual(stub.exports, [b''])


    def test_ar5_export_store_reopen_transfer_keeps_exact_owner_and_bytes(self):
        self.roundtrip(heightfield.archive(heightfield.full()), self.fixture.root)



    def test_ar5_malformed_and_receipt_mismatch_keep_original_rejection(self):
        payload = heightfield.archive(heightfield.full())
        for malformed in (payload[:-1], heightfield.archive(heightfield.full()[:-1])):
            client, stub, _ = peer_for(malformed)
            with self.assertRaises(ValueError):
                client.export_mapping_backend_checkpoint()
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
                     'full-1024.ar5', 'legacy.ar4', 'BL6-3层.ar5', 'BL6-4层.ar5', 'BL6-旧平面大块.ar4'):
            with self.subTest(name=name):
                payload = (Path(directory) / name).read_bytes()
                self.roundtrip(payload, self.fixture.root / name)
                self.legacy_import(payload)
                print('AR5_CONSUMER_RUST_ARCHIVE name=%s bytes=%d exact_roundtrip=PASS' % (name, len(payload)))
        payload = (Path(directory) / 'invalid-rebound.ar5').read_bytes()
        client, stub, _ = peer_for(payload)
        with self.assertRaisesRegex(ValueError, '空间绑定'):
            client.export_mapping_backend_checkpoint()
        self.assertEqual(stub.entries, {})


if __name__ == '__main__':
    unittest.main()
