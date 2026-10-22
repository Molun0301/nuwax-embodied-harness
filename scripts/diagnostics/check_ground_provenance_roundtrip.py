#!/usr/bin/env python3
"""Check the map-chunks synthetic AR4 Writer fixture through Python checkpoint I/O.

Generate input with source_provenance_tests::source_watermarks_survive_ar4_restore_without_cross_clock_cutoffs
and NEH_PROVENANCE_ARCHIVE_OUT set to a file outside the checkout. This is an offline
synthetic diagnostic, not a live sensor or clock-synchronization qualification.
"""
import argparse
import json
import struct
import sys
import types
from pathlib import Path

root = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('archive', type=Path, help='Rust-generated synthetic AR4 input')
args = parser.parse_args()
sys.path[:0] = [str(root), str(root / 'tests')]
for name in ('services', 'services.geometry'):
    package = types.ModuleType(name)
    package.__path__ = [str(root / name.replace('.', '/'))]
    sys.modules[name] = package
from services.geometry.fine_map_archive import validate_fine_archive, read_fine_archive_root
from test_checkpoint_identity_v2 import IdentityV2Tests, TransferStub, runtime, provider

fixture = args.archive.read_bytes()
expected = validate_fine_archive(fixture)
assert expected['version'] == 4
magic, _, offset = read_fine_archive_root(fixture)
block = fixture[offset + 12:]
assert block[:8] == b'NUWAXBL3'
points = struct.unpack('<I', block[8:12])[0]
layers = json.loads(block[16 + points * 12:])
samples = layers[0][1]['samples']
groups = layers[0][1]['sample_sources']
assert len(expected['source_watermarks']) == 3 and len(groups) == 3 and len(samples) == 3
assert any(sample['source_sequence'] == 0 and sample['source_timestamp_ns'] == 2 for sample in samples)
assert any(mark['source'].get('runtime_generation') == 2 ** 63 - 1 for mark in expected['source_watermarks'])
assert len([sample for sample in samples if sample['source_sequence'] == 9 and sample['source_timestamp_ns'] == 20 and sample['point_index'] == 0]) == 2

case = IdentityV2Tests()
case.setUp()  # Existing no-hash sentinels and temporary checkpoint repository.
try:
    response = case.response
    response.ClearField('dense_cloud_artifact')
    response.source_session_id = expected['domain']['session']
    response.source_session_generation = expected['domain']['generation']
    response.created_at_ns = 21
    response.occupancy_receipt.session_id = response.source_session_id
    response.occupancy_receipt.session_generation = 1
    response.occupancy_receipt.occupancy_revision = 1
    response.occupancy_receipt.source_timestamp_ns = 20
    response.cloud_receipt.session_generation = 1
    response.cloud_receipt.occupancy_revision = 1
    response.cloud_receipt.cloud_revision = 1
    response.cloud_receipt.source_timestamp_ns = 20
    response.canonical_occupancy_checkpoint.meta.revision = 1
    response.canonical_occupancy_checkpoint.meta.timestamp_ns = 20
    response.canonical_cloud_checkpoint.source_revision = 1
    response.payload = b'NWMBCKP\0' + struct.pack('<IBBHQQQQddd', 1, 0, 0, 0, 16, 1, 1, 20, 1, 2, 3)
    response.payload_size = len(response.payload)
    for member in (response.backend_identity, response.occupancy_identity,
                   response.canonical_cloud_checkpoint.member_identity,
                   response.fine_map_artifact.member_identity):
        member.occupancy_receipt.CopyFrom(response.occupancy_receipt)
        member.cloud_receipt.CopyFrom(response.cloud_receipt)
    response.fine_map_artifact.member_identity.producer_generation = 1
    response.fine_map_artifact.size = len(fixture)
    stub = TransferStub(response)
    stub.entries[response.fine_map_artifact.id][1] = bytearray(fixture)
    client = runtime(stub)
    client._assigned_fine_archive_versions = frozenset([4])
    bundle = client.export_mapping_backend_checkpoint()
    assert bytes(bundle.checkpoint.fine_archive) == fixture
    bundle.publish_to(case.root)
    reopened = provider.reopen_mapping_backend_checkpoint(case.root, 'voxel')
    assert bytes(reopened.checkpoint.fine_archive) == fixture
    uploaded = client._upload_graph_artifact(reopened.checkpoint,
        payload=reopened.checkpoint.fine_archive, reference=reopened.checkpoint.fine_reference)
    downloaded = client._download_graph_artifact(uploaded, provider.MAX_FINE_ARCHIVE_BYTES)
    assert bytes(downloaded) == fixture
    assert validate_fine_archive(downloaded) == expected
    report = dict(python=sys.version.split()[0], archive_bytes=len(fixture), archive_magic=magic.decode(),
                  block_magic=block[:8].decode(), source_groups=groups, samples=samples,
                  source_watermarks=expected['source_watermarks'],
                  passed=['Rust Writer bytes admitted in Python', 'three producer namespaces preserved',
                          'auxiliary sequence zero preserved', 'same numeric sample IDs remain separately sourced',
                          'checkpoint export/store/reopen/reupload exact bytes', 'no-hash sentinels active'])
    print(json.dumps(report, indent=2))
finally:
    case.doCleanups()
