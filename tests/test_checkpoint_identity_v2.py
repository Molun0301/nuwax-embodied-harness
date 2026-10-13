"""Recovered offline checkpoint regressions. No historical test result applies to these bytes."""
import copy
import hashlib
import json
import os
import struct
import sys
import tempfile
import threading
import unittest
from contextlib import ExitStack
from dataclasses import replace
from pathlib import Path
from datetime import datetime, timezone
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.geometry import checkpoint_payload as payload_module
from services.geometry import mapping_backend_checkpoint_store as store_module
from services.geometry import geometry_runtime_provider as provider
from services.geometry.checkpoint_payload import FrozenPayload
from services.geometry.mapping_backend_checkpoint_store import MappingBackendCheckpointStore, PUBLISH_FAULT_PHASES
pb = provider.pb2


def fine_bytes(legacy=False):
    ref = dict(key=[0, 0, 0], points=1)
    if legacy:
        ref.update(sha256=[20] * 32, geometry_sha256=[30] * 32)
    else:
        ref.update(object=dict(kind='assigned', value=[40] * 16), block_revision=7, geometry_revision=7)
    root = dict(version=1 if legacy else 2, domain=dict(session='source', generation=3,
        frame_generation=0, producer_generation=8), revision=7, source_timestamp_ns=11,
        voxel_size_m=0.125, blocks=[ref])
    if not legacy:
        root['geometry_revision'] = 7
    raw = json.dumps(root, separators=(',', ':')).encode()
    block = b'NUWAXBL1' + struct.pack('<IIHQH', 1, 0, 0, 11, 1)
    return ((b'NUWAXAR1' if legacy else b'NUWAXAR2') + struct.pack('<Q', len(raw))
        + (b'X' * 32 if legacy else b'') + raw + struct.pack('<QI', 1, len(block)) + block)


def dense_bytes():
    metadata = dict(version=1, session_id='source', session_generation=3, cloud_revision=7,
        source_timestamp_ns=11, frame_id='map', point_count=2, bounds_min=[1, 2, 3], bounds_max=[4, 5, 6])
    raw = json.dumps(metadata, separators=(',', ':')).encode()
    return b'NUWAXDC1' + struct.pack('<I', len(raw)) + raw + struct.pack('<ffffff', 1, 2, 3, 4, 5, 6)


def response_fixture(checkpoint_id='a' * 32, fine_id='b' * 32, dense_id='c' * 32):
    occupancy = pb.CanonicalOccupancyCommitReceipt(commit_id=pb.MappingCommitIdentity(boot_nonce=b'N' * 16, sequence=5),
        session_id='source', session_generation=3, occupancy_revision=6, source_timestamp_ns=11,
        backend_kind='voxel', backend_generation=2)
    cloud = pb.CanonicalCloudCommitReceipt(commit_id=occupancy.commit_id, session_generation=3, occupancy_revision=6,
        cloud_revision=7, source_timestamp_ns=11, point_count=1)
    def identity(role, producer=0):
        return pb.CheckpointMemberIdentity(checkpoint_id=checkpoint_id, role=role,
            occupancy_receipt=occupancy, cloud_receipt=cloud, frame_id='map', producer_generation=producer)
    backend = b'NWMBCKP\0' + struct.pack('<IBBHQQQQddd', 1, 0, 0, 0, 16, 1, 1, 11, 1, 2, 3)
    response = pb.MappingBackendCheckpoint(schema_version=2, backend_kind='voxel', source_session_id='source',
        source_session_generation=3, checkpoint_id=checkpoint_id, backend_payload_schema_version=1,
        payload_size=len(backend), payload=backend, created_at_ns=12, occupancy_receipt=occupancy, cloud_receipt=cloud,
        backend_identity=identity(1), occupancy_identity=identity(2),
        canonical_occupancy_checkpoint=pb.OccupancyGridData(meta=pb.OccupancyGridMeta(width=2, height=1, resolution=0.125,
            origin_x=1.25, origin_y=-2.5, frame_id='map', revision=6, timestamp_ns=11), data=b'\xff\x64'),
        canonical_cloud_checkpoint=pb.CloudMapCheckpoint(frame_id='map', source_revision=7, point_count=1, voxel_size=0.125,
            xyz_f32_le=struct.pack('<fff', 1, 2, 3), payload_schema_version=2, member_identity=identity(3)),
        fine_map_artifact=pb.GraphArtifactRef(id=fine_id, size=len(fine_bytes()), identity_schema_version=2, member_identity=identity(4, 8)),
        dense_cloud_artifact=pb.GraphArtifactRef(id=dense_id, size=len(dense_bytes()), identity_schema_version=2, member_identity=identity(5)))
    return pb.MappingBackendCheckpoint.FromString(response.SerializeToString())


class TransferStub:
    def __init__(self, response):
        self.response = response
        self.entries = {
            response.fine_map_artifact.id: [copy.deepcopy(response.fine_map_artifact), bytearray(fine_bytes()), True],
            response.dense_cloud_artifact.id: [copy.deepcopy(response.dense_cloud_artifact), bytearray(dense_bytes()), True],
        }
        self.requests = []
        self.exports = []
    def ExportMappingBackendCheckpoint(self, request, timeout):
        assert isinstance(request, pb.ExportMappingBackendCheckpointRequest)
        self.exports.append(request.checkpoint_schema_version)
        return copy.deepcopy(self.response)
    def TransferGraphArtifact(self, request, timeout):
        self.requests.append(copy.deepcopy(request))
        ref, action = request.artifact, request.action
        if action == 1 and ref.id not in self.entries:
            self.entries[ref.id] = [copy.deepcopy(ref), bytearray(), False]
        if action == 5:
            if ref.id in self.entries:
                assert self.entries[ref.id][0] == ref
                self.entries.pop(ref.id)
            return pb.GraphArtifactTransferResponse(artifact=ref)
        retained, data, sealed = self.entries[ref.id]
        assert retained == ref
        if action == 2:
            end = request.offset + len(request.data)
            if request.offset < len(data):
                assert data[request.offset:end] == request.data
            else:
                assert request.offset == len(data)
                data.extend(request.data)
        if action == 3:
            assert len(data) == ref.size
            self.entries[ref.id][2] = sealed = True
        chunk = bytes(data[request.offset:request.offset + 1024 * 1024]) if action == 4 else b''
        return pb.GraphArtifactTransferResponse(artifact=ref, offset=len(data), sealed=sealed, data=chunk)


def runtime(stub):
    instance = provider.GeometryRuntimeProvider.__new__(provider.GeometryRuntimeProvider)
    instance._connected = True
    instance._stub = stub
    instance._map_state_lock = threading.Lock()
    instance._stopping = False
    instance._map_connection_epoch = 1
    instance._raise_if_stopping = lambda *args: None
    return instance


class IdentityV2Tests(unittest.TestCase):
    def setUp(self):
        self.guard = ExitStack()
        self.addCleanup(self.guard.close)
        for name in hashlib.algorithms_guaranteed | {'new', 'file_digest'}:
            if hasattr(hashlib, name):
                self.guard.enter_context(patch.object(hashlib, name, side_effect=AssertionError('application hash called')))
        for module in (payload_module, store_module, provider):
            self.guard.enter_context(patch.object(module, 'payload_digest', side_effect=AssertionError('payload digest called')))
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.response = response_fixture()

    def bundle(self, response=None):
        return provider._checkpoint_export_bundle_from_proto(self.response if response is None else response,
            fine_payload=fine_bytes(), dense_payload=dense_bytes())

    def test_actual_proto_export_store_restart_reopen_reupload(self):
        stub = TransferStub(self.response)
        client = runtime(stub)
        bundle = client.export_mapping_backend_checkpoint(checkpoint_schema_version=2)
        self.assertEqual(stub.exports, [2])
        self.assertIsInstance(bundle.checkpoint.fine_archive, FrozenPayload)
        fine_owner = bundle.checkpoint.fine_archive
        with patch.object(MappingBackendCheckpointStore, '_load_optional', side_effect=AssertionError('postpublish reload')):
            stored = bundle.publish_to(self.root)
        self.assertIs(stored.checkpoint, bundle.checkpoint)
        self.assertIs(stored.checkpoint.fine_archive, fine_owner)
        reopened = provider.reopen_mapping_backend_checkpoint(self.root, 'voxel')
        self.assertEqual(reopened.checkpoint.checkpoint_id, self.response.checkpoint_id)
        self.assertEqual(reopened.canonical_occupancy_checkpoint(), self.response.canonical_occupancy_checkpoint)
        self.assertEqual(reopened.canonical_cloud_checkpoint(), self.response.canonical_cloud_checkpoint)
        loaded = MappingBackendCheckpointStore(self.root).load('voxel', schema_version=2)
        self.assertEqual(provider._mapping_backend_checkpoint_proto(loaded), self.response)
        for field, payload in (('fine_reference', reopened.checkpoint.fine_archive), ('dense_reference', reopened.checkpoint.dense_archive)):
            retained = getattr(reopened.checkpoint, field)
            uploaded = client._upload_graph_artifact(reopened.checkpoint, payload=payload, reference=retained)
            self.assertEqual(uploaded, getattr(self.response, 'fine_map_artifact' if field == 'fine_reference' else 'dense_cloud_artifact'))
            self.assertEqual(client._download_graph_artifact(uploaded, provider.MAX_DENSE_ARCHIVE_BYTES), payload)
        self.assertEqual(loaded.identity, ('a' * 32, 'manifest'))
        self.assertEqual(loaded.manifest_sha256, '')
        self.assertIsNone(loaded.legacy_alias)

    def test_ar3_layers_export_store_restart_and_reupload_keep_v2_identity(self):
        original = fine_bytes()
        root_length = struct.unpack('<Q', original[8:16])[0]
        root = json.loads(original[16:16 + root_length])
        root['version'] = 3
        raw = json.dumps(root, separators=(',', ':')).encode()
        layer = dict(members=[0], normal=[0, 0, 1], plane_origin=[0.0625, 0.0625, 0.08],
            samples=[dict(point=[0.04, 0.06, 0.08], source_sequence=5, source_timestamp_ns=11, point_index=0)])
        tail = json.dumps([[0, layer]], separators=(',', ':')).encode()
        block = b'NUWAXBL2' + struct.pack('<IIHQH', 1, len(tail), 0, 11, 1) + tail
        payload = b'NUWAXAR3' + struct.pack('<Q', len(raw)) + raw + struct.pack('<QI', 1, len(block)) + block
        self.response.fine_map_artifact.size = len(payload)
        stub = TransferStub(self.response)
        stub.entries[self.response.fine_map_artifact.id][1] = bytearray(payload)
        client = runtime(stub)
        bundle = client.export_mapping_backend_checkpoint(checkpoint_schema_version=2)
        self.assertEqual(bundle.checkpoint.fine_reference.identity_schema_version, 2)
        self.assertEqual(bundle.checkpoint.fine_archive, payload)
        bundle.publish_to(self.root)
        reopened = provider.reopen_mapping_backend_checkpoint(self.root, 'voxel')
        self.assertEqual(reopened.checkpoint.fine_archive, payload)
        retained = reopened.checkpoint.fine_reference
        uploaded = client._upload_graph_artifact(reopened.checkpoint,
            payload=reopened.checkpoint.fine_archive, reference=retained)
        self.assertEqual(uploaded, self.response.fine_map_artifact)
        self.assertEqual(client._download_graph_artifact(uploaded, provider.MAX_FINE_ARCHIVE_BYTES), payload)

    def test_explicit_backend_and_occupancy_members_reject_skew_and_conflicts(self):
        for field, role in (("backend_identity", 1), ("occupancy_identity", 2)):
            response = copy.deepcopy(self.response)
            response.ClearField(field)
            with self.assertRaises(ValueError):
                self.bundle(response)
            for mutate in (
                lambda member: setattr(member, "role", 3),
                lambda member: setattr(member, "checkpoint_id", "b" * 32),
                lambda member: setattr(member, "frame_id", "odom"),
                lambda member: setattr(member, "producer_generation", 8),
                lambda member: setattr(member.occupancy_receipt, "occupancy_revision", 999),
                lambda member: setattr(member.cloud_receipt, "cloud_revision", 999),
            ):
                response = copy.deepcopy(self.response)
                mutate(getattr(response, field))
                with self.assertRaises(ValueError):
                    self.bundle(response)
        self.assertFalse((self.root / "mapping_backend").exists())

    def test_legacy_wire_rejects_new_member_fields_before_hash_or_artifact_reads(self):
        for field in ("backend_identity", "occupancy_identity"):
            response = pb.MappingBackendCheckpoint(schema_version=1, backend_kind="voxel")
            getattr(response, field).CopyFrom(getattr(self.response, field))
            with self.assertRaisesRegex(ValueError, "legacy checkpoint cannot carry assigned"):
                self.bundle(response)

    def test_public_export_rejects_missing_member_before_artifact_download(self):
        for field in ("backend_identity", "occupancy_identity"):
            response = copy.deepcopy(self.response)
            response.ClearField(field)
            stub = TransferStub(response)
            with self.assertRaises(ValueError):
                runtime(stub).export_mapping_backend_checkpoint(checkpoint_schema_version=2)
            self.assertEqual(stub.requests, [])

    def test_saved_member_inventory_roundtrips_without_inventing_identity(self):
        bundle = self.bundle()
        stored = bundle.publish_to(self.root)
        loaded = MappingBackendCheckpointStore(self.root).load("voxel", schema_version=2)
        for field in ("backend_identity", "occupancy_identity", "cloud_identity"):
            self.assertEqual(getattr(loaded.checkpoint, field), getattr(bundle.checkpoint, field))
        # The pre-Stage-A v2 manifest already stored these exact descriptors.
        # Reading it reuses that inventory, without synthesizing a wire identity.
        document = json.loads(loaded.manifest_path.read_text())
        self.assertEqual(document["members"]["backend_state"]["identity"]["role"], "backend_state")
        self.assertEqual(provider._mapping_backend_checkpoint_proto(loaded), self.response)

    def test_pre_stage_a_saved_manifest_fixture_reopens_exact_member_ids(self):
        # Frozen metadata was generated by the unmodified 166380 serializer.
        # Bodies are deterministic test fixtures; this is not a historical
        # user's saved map or a claim about historical runtime test results.
        fixture = Path(__file__).parent / "fixtures/checkpoint-v2-166380-member-inventory.json"
        stored = self.bundle().publish_to(self.root)
        self.assertEqual(json.loads(stored.manifest_path.read_text()), json.loads(fixture.read_text()))
        stored.manifest_path.write_bytes(fixture.read_bytes())
        loaded = MappingBackendCheckpointStore(self.root).load("voxel", schema_version=2)
        self.assertEqual(provider._mapping_backend_checkpoint_proto(loaded), self.response)

    def test_asset_identity_is_strict_immutable_and_not_a_digest(self):
        from contracts.map_workspace import AssetIdentityV2
        asset = AssetIdentityV2(asset_id="a" * 32)
        self.assertEqual(pb.AssetIdentityV2(asset_id=asset.asset_id).asset_id, asset.asset_id)
        for invalid in ("0" * 32, "A" * 32, "a" * 64, " a" * 16, 123, None):
            with self.assertRaises(ValueError):
                AssetIdentityV2(asset_id=invalid)
        with self.assertRaises(ValueError):
            asset.asset_id = "b" * 32
        with self.assertRaises(ValueError):
            AssetIdentityV2(asset_id="a" * 32, revision=1)

    def test_assigned_activation_wire_contains_one_canonical_checkpoint_body(self):
        from contracts.map_workspace import AssetIdentityV2
        stored = self.bundle().publish_to(self.root)
        materialized = []
        original = provider._protobuf_payload_bytes
        def once(value):
            materialized.append(id(value))
            return original(value)
        with patch.object(provider, "_canonical_cloud_from_stored", side_effect=AssertionError("intermediate cloud copy")), \
             patch.object(provider, "_canonical_occupancy_from_stored", side_effect=AssertionError("intermediate occupancy copy")), \
             patch.object(provider, "_protobuf_payload_bytes", side_effect=once):
            request = provider._assigned_activation_request_proto(stored,
                asset_identity=AssetIdentityV2(asset_id="d" * 32), session_id="loaded:one", map_name="warehouse",
                activation_request_id="activate:one", expected_runtime_generation=17)
        self.assertEqual(request.activation_contract_version, 2)
        self.assertFalse(request.HasField("checkpoint"))
        self.assertFalse(request.HasField("cloud_checkpoint"))
        self.assertEqual(request.asset_manifest_sha256, b"")
        self.assertEqual(request.asset_bundle_sha256, b"")
        self.assertEqual(request.mapping_backend_checkpoint, self.response)
        self.assertEqual(request.asset_identity.asset_id, "d" * 32)
        self.assertEqual(materialized, [id(stored.checkpoint.payload), id(stored.checkpoint.canonical_occupancy_data),
                                        id(stored.checkpoint.canonical_cloud_xyz)])
        decoded = pb.ActivateMapRequest.FromString(request.SerializeToString())
        self.assertEqual(decoded, request)
        self.assertEqual(request.SerializeToString().count(self.response.canonical_cloud_checkpoint.xyz_f32_le), 1)

    def test_assigned_activation_metadata_rejects_before_body_materialization(self):
        from contracts.map_workspace import AssetIdentityV2
        stored = self.bundle().publish_to(self.root)
        valid = dict(asset_identity=AssetIdentityV2(asset_id="d" * 32), session_id="loaded:one", map_name="warehouse",
                     activation_request_id="activate:one", expected_runtime_generation=17)
        for field, value in (("asset_identity", "d" * 32), ("session_id", " bad"),
                             ("activation_request_id", "bad/one"), ("map_name", " map "),
                             ("expected_runtime_generation", 0), ("expected_runtime_generation", True),
                             ("expected_runtime_generation", 1 << 64)):
            with patch.object(provider, "_protobuf_payload_bytes", side_effect=AssertionError("body read before metadata rejection")):
                with self.assertRaises(ValueError):
                    provider._assigned_activation_request_proto(stored, **dict(valid, **{field: value}))
        forged = replace(stored, checkpoint=replace(stored.checkpoint))
        with self.assertRaisesRegex(ValueError, "admitted assigned"):
            provider._assigned_activation_request_proto(forged, **valid)

    def test_same_id_retry_streams_and_reuses_input(self):
        bundle = self.bundle()
        bundle.publish_to(self.root)
        original = store_module._read_regular_file_at
        def small_only(*args, **kwargs):
            self.assertLessEqual(kwargs['maximum_bytes'], store_module._MAX_JSON_BYTES)
            return original(*args, **kwargs)
        with patch.object(store_module, '_read_regular_file_at', side_effect=small_only):
            stored = bundle.publish_to(self.root)
        self.assertIs(stored.checkpoint, bundle.checkpoint)

    def test_reopen_checks_expected_assigned_identity_without_legacy_fallback(self):
        self.bundle().publish_to(self.root)
        reopened = provider.reopen_mapping_backend_checkpoint(
            self.root, 'voxel', expected_checkpoint_id='a' * 32,
        )
        self.assertEqual(reopened.checkpoint.checkpoint_id, 'a' * 32)
        with self.assertRaisesRegex(ValueError, 'expected assigned identity'):
            provider.reopen_mapping_backend_checkpoint(
                self.root, 'voxel', expected_checkpoint_id='d' * 32,
            )
        for invalid in ('', 'A' * 32, 'd' * 64):
            with self.assertRaises(ValueError):
                provider.reopen_mapping_backend_checkpoint(
                    self.root, 'voxel', expected_checkpoint_id=invalid,
                )
        with self.assertRaisesRegex(ValueError, 'requires schema 2'):
            MappingBackendCheckpointStore(self.root).load(
                'voxel', schema_version=1, expected_checkpoint_id='a' * 32,
            )

    def test_v2_activation_gate_precedes_rpc_and_asset_reads(self):
        client = runtime(None)
        with patch.object(Path, 'resolve', side_effect=AssertionError('asset opened')):
            with self.assertRaisesRegex(RuntimeError, 'CHECKPOINT_V2_ACTIVATION_UNAVAILABLE'):
                client.activate_platform_map(
                    map_name='map', bundle_root=self.root, mapping_database_path='state.db',
                    pcd_path='cloud.pcd', nav_map_yaml_path='map.yaml', asset_manifest_sha256='',
                    occupancy_grid=None, expected_backend_checkpoint_schema_version=2,
                    expected_backend_checkpoint_id='a' * 32,
                )

    def test_v2_pcd_materialization_streams_retained_cloud_member(self):
        from services.geometry.runtime import PlatformGeometryRuntime
        instance = PlatformGeometryRuntime.__new__(PlatformGeometryRuntime)
        instance._pcd_min_points = lambda: 1
        instance._is_usable_pcd_file = lambda path: path.is_file()
        bundle = self.bundle()
        target = self.root / 'cloud.pcd'
        with patch.object(provider.MappingBackendCheckpointExportBundle, 'canonical_cloud_checkpoint',
                          side_effect=AssertionError('protobuf cloud copy')):
            with patch.object(Path, 'read_bytes', side_effect=AssertionError('full file reread')):
                metadata = instance._materialize_checkpoint_pcd_asset(target, bundle)
        header, xyz = target.read_bytes().split(b'DATA binary\n', 1)
        self.assertIn(b'POINTS 1\n', header)
        self.assertEqual(xyz, self.response.canonical_cloud_checkpoint.xyz_f32_le)
        self.assertEqual(metadata['checkpoint_id'], 'a' * 32)
        self.assertEqual(metadata['member_role'], 'canonical_cloud')
        self.assertFalse(any('sha256' in key for key in metadata))

    def test_v2_native_database_uses_assigned_identity_column(self):
        import sqlite3
        from services.geometry.runtime import PlatformGeometryRuntime
        instance = PlatformGeometryRuntime.__new__(PlatformGeometryRuntime)
        bundle = SimpleNamespace(map_name='map', frame_id='map', timestamp=datetime.now(timezone.utc),
            snapshot=SimpleNamespace(source_epoch='source:3', update_sequence=6, source_name='geometry', frame_id='map'))
        target = self.root / 'native.db'
        metadata = instance._materialize_native_bundle_database(
            target, bundle=bundle, checkpoint_bundle=self.bundle(),
            mapping_context={'session_id': 'source', 'session_generation': 3},
        )
        with sqlite3.connect(str(target)) as database:
            self.assertEqual(database.execute('PRAGMA user_version').fetchone(), (4,))
            columns = [row[1] for row in database.execute('PRAGMA table_info(map_asset_identity)')]
            self.assertIn('backend_checkpoint_id', columns)
            self.assertNotIn('backend_checkpoint_content_sha256', columns)
            self.assertEqual(database.execute('SELECT backend_checkpoint_id FROM map_asset_identity').fetchone(), ('a' * 32,))
        self.assertEqual(metadata['database_identity']['backend_checkpoint_id'], 'a' * 32)
        self.assertFalse(any('sha256' in key for key in metadata['database_identity']))

    def test_runtime_v2_activation_gate_precedes_native_activator(self):
        from services.geometry.runtime import PlatformGeometryRuntime
        instance = PlatformGeometryRuntime.__new__(PlatformGeometryRuntime)
        instance.config = SimpleNamespace(stack='nuwax')
        instance._summarize_geometry_asset = lambda value: {}
        def forbidden(**kwargs):
            raise AssertionError('unpaired activation dispatched')
        instance._resolve_mapping_backend = lambda: SimpleNamespace(activate_platform_map=forbidden)
        bundle = SimpleNamespace(map_name='map', manifest_sha256=None, frame_id='map',
            geometry_asset=SimpleNamespace(mapping_database_path='state.db', pcd_path='cloud.pcd',
                metadata={'mapping_backend_checkpoint': {'present': True, 'schema_version': 2, 'checkpoint_id': 'a' * 32}}))
        with self.assertRaisesRegex(RuntimeError, 'CHECKPOINT_V2_ACTIVATION_UNAVAILABLE'):
            instance._activate_backend_map_bundle(bundle, bundle_root=self.root)

    def prepare_v2_bundle(self, root):
        from contracts.geometry_runtime import PlatformGeometryConfig
        from contracts.map_workspace import MapAssetBundle
        from contracts.runtime_views import MapSnapshot
        from services.geometry.runtime import PlatformGeometryRuntime
        instance = PlatformGeometryRuntime.__new__(PlatformGeometryRuntime)
        instance.config = PlatformGeometryConfig()
        context = dict(active=True, session_id='source', session_generation=3, map_name='map',
                       mapping_backend_kind='voxel', revision=6)
        exported = self.bundle()
        calls = []
        def export(**kwargs):
            calls.append(kwargs)
            return exported
        instance._is_nuwax_mapping_active = lambda: True
        instance.get_mapping_session_context = lambda: dict(context)
        instance._runtime_state_lock = threading.RLock()
        instance._remote_provider = runtime(TransferStub(self.response))
        instance._remote_provider.export_mapping_backend_checkpoint = export
        instance._provider_attach_epoch = 1
        instance._resolve_mapping_backend = lambda: instance._remote_provider
        instance._managed_processes = {}
        instance._external_process_name = lambda: 'geometry'
        instance._locate_live_mapping_database = lambda: None
        instance._materialize_backend_state = lambda target: {'database_present': False}
        instance._pcd_min_points = lambda: 1
        instance._is_usable_pcd_file = lambda path: path.is_file()
        instance._build_geometry_quality_metrics = lambda **kwargs: dict(
            database={'required': True, 'present': kwargs['database_metadata']['database_present']},
            pcd={'present': kwargs['pcd_metadata']['pcd_present']},
            navigation_map={'present': kwargs['nav2_map_metadata']['present']},
            checkpoint_ready=True,
        )
        candidate = MapAssetBundle(map_name='map', source_name='geometry', frame_id='map',
            snapshot=MapSnapshot(map_name='map', source_name='geometry', source_epoch='source:3',
                                 update_sequence=6, frame_id='map'),
            metadata={'mapping_session_id': 'source', 'session_generation': 3})
        with patch.object(provider.MappingBackendCheckpointExportBundle, 'canonical_cloud_checkpoint',
                          side_effect=AssertionError('protobuf cloud copy')):
            prepared = instance.prepare_map_asset_bundle(
                candidate, bundle_root=root, checkpoint_schema_version=2,
            )
        self.assertEqual(calls, [{'checkpoint_schema_version': 2}])
        return prepared, exported

    def test_explicit_v2_preparation_keeps_assigned_metadata_and_exact_members(self):
        prepared, exported = self.prepare_v2_bundle(self.root)
        checkpoint_metadata = prepared.geometry_asset.metadata['mapping_backend_checkpoint']
        self.assertEqual(checkpoint_metadata['schema_version'], 2)
        self.assertEqual(checkpoint_metadata['checkpoint_id'], 'a' * 32)
        self.assertFalse(any('sha256' in key for key in checkpoint_metadata))
        self.assertEqual(set(checkpoint_metadata),
                         {'present', 'schema_version', 'backend_kind', 'checkpoint_id', 'manifest_path'})
        diagnostics = prepared.metadata['mapping_backend_checkpoint']
        self.assertEqual(set(diagnostics['member_paths']),
                         {'backend_state', 'canonical_occupancy', 'canonical_cloud', 'fine_map', 'dense_cloud'})
        self.assertEqual(diagnostics['payload_size'], exported.checkpoint.payload_size)
        self.assertEqual(diagnostics['source_session_id'], 'source')
        self.assertEqual(diagnostics['source_session_generation'], 3)
        self.assertEqual(prepared.snapshot.occupancy_grid.data, [-1, 100])
        pcd = self.root / prepared.geometry_asset.pcd_path
        self.assertEqual(pcd.read_bytes().split(b'DATA binary\n', 1)[1],
                         self.response.canonical_cloud_checkpoint.xyz_f32_le)
        reopened = provider.reopen_mapping_backend_checkpoint(
            self.root, 'voxel', expected_checkpoint_id='a' * 32,
        )
        self.assertEqual(reopened.canonical_cloud_checkpoint(), self.response.canonical_cloud_checkpoint)

    def test_actual_v2_preparation_finalizes_and_reopens_assigned_asset(self):
        from contracts.map_workspace import AssetIdentityV2, MapAssetBundle
        from services.mapping.asset_store import MapAssetBundleRepository
        repository = MapAssetBundleRepository(str(self.root / 'assets'))
        stage = repository.create_staging_directory('map')
        prepared, exported = self.prepare_v2_bundle(stage)
        self.assertEqual(prepared.identity_schema_version, 1)
        # Preparation precedes assignment: do not mutate an existing immutable
        # owner or repair the producer's geometry metadata in this fixture.
        identity = AssetIdentityV2(asset_id='d' * 32)
        values = {name: getattr(prepared, name) for name in type(prepared).model_fields}
        values.update(identity_schema_version=2, asset_identity=identity, asset_state='candidate')
        candidate = MapAssetBundle(**values)
        finalized = repository.finalize_assigned_map_asset_bundle(
            candidate, staging_dir=stage, asset_identity=identity)
        reopened = MapAssetBundleRepository(str(self.root / 'assets')).get_map_asset_bundle(
            'map', migrate_legacy=False, expected_asset_identity=identity)
        self.assertEqual(finalized.asset_identity, identity)
        self.assertEqual(reopened.asset_identity, identity)
        self.assertEqual(reopened.asset_state, 'finalized')
        self.assertEqual(reopened.geometry_asset.metadata['mapping_backend_checkpoint'],
                         prepared.geometry_asset.metadata['mapping_backend_checkpoint'])
        self.assertEqual(reopened.metadata['mapping_backend_checkpoint'],
                         prepared.metadata['mapping_backend_checkpoint'])
        self.assertEqual(reopened.snapshot.occupancy_grid.data, [-1, 100])
        current = repository.current_directory('map')
        checkpoint = provider.reopen_mapping_backend_checkpoint(
            current, 'voxel', expected_checkpoint_id=exported.checkpoint.checkpoint_id)
        self.assertEqual(checkpoint.canonical_occupancy_checkpoint(),
                         self.response.canonical_occupancy_checkpoint)
        self.assertEqual(checkpoint.canonical_cloud_checkpoint(),
                         self.response.canonical_cloud_checkpoint)
        self.assertEqual(checkpoint.checkpoint.occupancy_receipt['source_timestamp_ns'], 11)
        self.assertEqual(checkpoint.checkpoint.checkpoint_id, exported.checkpoint.checkpoint_id)
        self.assertEqual((current / reopened.geometry_asset.pcd_path).read_bytes().split(b'DATA binary\n', 1)[1],
                         self.response.canonical_cloud_checkpoint.xyz_f32_le)

    def test_finalization_default_stays_legacy_until_activation_contract_is_paired(self):
        import inspect
        from services.geometry.runtime import PlatformGeometryRuntime
        signature = inspect.signature(PlatformGeometryRuntime.prepare_map_asset_bundle)
        self.assertEqual(signature.parameters['checkpoint_schema_version'].default, 1)

    def test_same_checkpoint_identity_conflicts_before_overwrite(self):
        bundle = self.bundle()
        first = bundle.publish_to(self.root)
        selector = first.selector_path.read_bytes()
        candidates = [replace(bundle.checkpoint, created_at_ns=13),
            replace(bundle.checkpoint, canonical_occupancy_data=b'\x00\x64'),
            replace(bundle.checkpoint, canonical_cloud_xyz=struct.pack('<fff', 2, 2, 3)),
            replace(bundle.checkpoint, payload=bundle.checkpoint.payload[:-8] + struct.pack('<d', 4))]
        for changed in candidates:
            with self.assertRaises(ValueError):
                MappingBackendCheckpointStore(self.root).publish(changed, occupancy_asset=bundle.occupancy_asset, cloud_asset=bundle.cloud_asset)
            self.assertEqual(first.selector_path.read_bytes(), selector)
        self.assertEqual(MappingBackendCheckpointStore(self.root).load('voxel', schema_version=2).checkpoint.payload, bundle.checkpoint.payload)

    def test_artifact_id_descriptor_conflict_across_checkpoint(self):
        self.bundle().publish_to(self.root)
        with self.assertRaisesRegex(ValueError, 'artifact ID reused'):
            self.bundle(response_fixture('d' * 32)).publish_to(self.root)
        self.assertFalse((self.root / 'mapping_backend/voxel/objects' / ('id-' + 'd' * 32 + '-backend.bin')).exists())

    def test_download_requires_full_reference_and_correct_offset(self):
        stub = TransferStub(self.response)
        original = stub.TransferGraphArtifact
        def changed(request, timeout):
            result = original(request, timeout)
            if request.action == 4:
                result.artifact.member_identity.frame_id = 'changed'
            return result
        stub.TransferGraphArtifact = changed
        with self.assertRaises(RuntimeError):
            runtime(stub)._download_graph_artifact(self.response.fine_map_artifact)

    def test_explicit_version_rejects_old_response_before_download(self):
        self.response.schema_version = 1
        stub = TransferStub(self.response)
        with self.assertRaisesRegex(ValueError, 'downgrade'):
            runtime(stub).export_mapping_backend_checkpoint(checkpoint_schema_version=2)
        self.assertFalse(stub.requests)
        self.assertEqual(pb.ExportMappingBackendCheckpointRequest.FromString(pb.Empty().SerializeToString()).checkpoint_schema_version, 0)

    def test_mixed_identity_and_source_conflicts(self):
        mutations = [lambda r: setattr(r, 'content_sha256', b'x' * 32),
            lambda r: setattr(r.canonical_cloud_checkpoint, 'xyz_sha256', 'x' * 64),
            lambda r: setattr(r.fine_map_artifact, 'sha256', b'x' * 32),
            lambda r: setattr(r.fine_map_artifact.member_identity, 'checkpoint_id', 'd' * 32),
            lambda r: setattr(r.fine_map_artifact.member_identity, 'producer_generation', 9),
            lambda r: setattr(r.dense_cloud_artifact.member_identity, 'frame_id', 'wrong'),
            lambda r: setattr(r.canonical_cloud_checkpoint, 'xyz_f32_le', struct.pack('<fff', float('nan'), 2, 3))]
        for mutate in mutations:
            response = copy.deepcopy(self.response)
            mutate(response)
            with self.assertRaises(ValueError):
                self.bundle(response)

    def test_frozen_payload_constructs_without_digest(self):
        frozen = FrozenPayload((b'x' * 1024, b'y'))
        self.assertFalse(hasattr(frozen, '_digest'))
        self.assertEqual(list(frozen.chunks()), [b'x' * 1024 + b'y'])

    def test_fault_phases_keep_previous_selector_and_retry(self):
        self.bundle().publish_to(self.root)
        previous = self.root / 'mapping_backend/voxel/checkpoint-v2.json'
        old_bytes = previous.read_bytes()
        for phase in PUBLISH_FAULT_PHASES:
            index = PUBLISH_FAULT_PHASES.index(phase)
            candidate = self.bundle(response_fixture(f'{index+100:032x}', f'{index+200:032x}', f'{index+300:032x}'))
            def fail(here):
                if here == phase:
                    raise RuntimeError(phase)
            with self.subTest(phase=phase), self.assertRaises(RuntimeError):
                MappingBackendCheckpointStore(self.root).publish(candidate.checkpoint,
                    occupancy_asset=candidate.occupancy_asset, cloud_asset=candidate.cloud_asset, fault_injector=fail)
            if index < PUBLISH_FAULT_PHASES.index('selector_installed'):
                self.assertEqual(previous.read_bytes(), old_bytes)
            candidate.publish_to(self.root)
            old_bytes = previous.read_bytes()

    def test_corrupt_live_selector_preserves_uncertain_gc_set(self):
        self.bundle().publish_to(self.root)
        backend = self.root / 'mapping_backend/voxel'
        (backend / 'checkpoint-v1.json').write_text('{bad')
        orphan = backend / 'objects' / ('id-' + 'f' * 32 + '-cloud.bin')
        orphan.write_bytes(b'orphan')
        result = MappingBackendCheckpointStore(self.root, orphan_grace_seconds=0).cleanup('voxel', schema_version=2)
        self.assertTrue(result.issues)
        self.assertTrue(orphan.exists())
        MappingBackendCheckpointStore(self.root).load('voxel', schema_version=2)

    def test_single_freeze_per_large_reopened_member(self):
        self.bundle().publish_to(self.root)
        calls = []
        original = store_module.FrozenPayload
        original_init = original.__init__
        def init(instance, *args, **kwargs):
            original_init(instance, *args, **kwargs)
            calls.append(instance)
        with patch.object(original, '__init__', init):
            loaded = MappingBackendCheckpointStore(self.root).load('voxel', schema_version=2)
        self.assertEqual(len(calls), 3)
        self.assertTrue(any(loaded.checkpoint.fine_archive is item for item in calls))

    def install_legacy(self):
        occupancy_receipt = provider._occupancy_receipt_dict(self.response.occupancy_receipt)
        cloud_receipt = provider._cloud_receipt_dict(self.response.cloud_receipt)
        canonical = self.bundle()
        occupancy = dict(canonical.occupancy_asset, raw_data_sha256=b'O' * 32)
        cloud = dict(canonical.cloud_asset, xyz_sha256=b'C' * 32, payload_schema_version=1)
        backend = self.root / 'mapping_backend/voxel'
        (backend / 'objects').mkdir(parents=True, exist_ok=True)
        (backend / 'manifests').mkdir(exist_ok=True)
        aliases = {'payload': '55' * 32, 'fine_archive': '66' * 32, 'dense_archive': '77' * 32}
        payloads = {'payload': bytes(self.response.payload), 'fine_archive': fine_bytes(legacy=True), 'dense_archive': dense_bytes()}
        references = {}
        for role, alias in aliases.items():
            relative = 'objects/sha256-' + alias + '.bin'
            (backend / relative).write_bytes(payloads[role])
            references[role] = dict(sha256=alias, path=relative, size=len(payloads[role]))
        document = dict(schema_version=1, backend_kind='voxel', source_session_id='source', source_session_generation=3,
            created_at_ns=12, canonical_receipts=dict(occupancy=store_module._receipt_document(occupancy_receipt),
                cloud=store_module._receipt_document(cloud_receipt)),
            canonical_assets=dict(occupancy=store_module._asset_document(occupancy, sha256_field='raw_data_sha256'),
                cloud=store_module._asset_document(cloud, sha256_field='xyz_sha256')), **references)
        manifest_alias = '88' * 32
        manifest_relative = 'manifests/sha256-' + manifest_alias + '.json'
        (backend / manifest_relative).write_bytes(store_module._encode_json(document))
        selector = dict(schema_version=1, backend_kind='voxel', manifest=dict(sha256=manifest_alias, path=manifest_relative))
        (backend / 'checkpoint-v1.json').write_bytes(store_module._encode_json(selector))
        return backend, document

    def test_legacy_read_preserves_literal_aliases_and_missing_canonical_bytes(self):
        backend, document = self.install_legacy()
        loaded = MappingBackendCheckpointStore(self.root).load('voxel', schema_version=1)
        self.assertEqual(loaded.legacy_alias, '88' * 32)
        self.assertIsNone(loaded.identity)
        self.assertEqual(loaded.checkpoint.content_sha256, bytes.fromhex('55' * 32))
        self.assertIsNone(loaded.checkpoint.canonical_occupancy_data)
        self.assertIsNone(loaded.checkpoint.canonical_cloud_xyz)
        self.assertEqual(loaded.checkpoint.fine_archive, fine_bytes(legacy=True))
        self.assertEqual(set(loaded.member_paths), {'backend_state', 'fine_map', 'dense_cloud'})
        self.assertNotIn('manifest_sha256', document)

    def test_gc_keeps_union_of_v1_v2_paths_without_payload_reads(self):
        self.install_legacy()
        self.bundle().publish_to(self.root)
        backend = self.root / 'mapping_backend/voxel'
        live = set((backend / 'objects').iterdir()) | set((backend / 'manifests').iterdir())
        orphan = backend / 'objects' / ('id-' + 'f' * 32 + '.bin')
        orphan.write_bytes(b'orphan')
        for path in live | {orphan}:
            os.utime(path, ns=(1, 1))
        original = store_module._read_regular_file_at
        def metadata_only(*args, **kwargs):
            self.assertLessEqual(kwargs['maximum_bytes'], store_module._MAX_JSON_BYTES)
            return original(*args, **kwargs)
        with patch.object(store_module, '_read_regular_file_at', side_effect=metadata_only):
            result = MappingBackendCheckpointStore(self.root, orphan_grace_seconds=0).cleanup('voxel', schema_version=2)
        self.assertFalse(result.issues)
        self.assertFalse(orphan.exists())
        self.assertTrue(all(path.exists() for path in live))
        MappingBackendCheckpointStore(self.root).load('voxel', schema_version=1)
        MappingBackendCheckpointStore(self.root).load('voxel', schema_version=2)

    def test_fine_and_dense_same_id_changed_bytes_fail(self):
        bundle = self.bundle()
        bundle.publish_to(self.root)
        fine = bytearray(fine_bytes())
        fine[-2:] = struct.pack('<H', 2)
        dense = dense_bytes()
        offset = 12 + struct.unpack('<I', dense[8:12])[0]
        dense = dense[:offset] + dense[offset + 12:] + dense[offset:offset + 12]
        for changed in (replace(bundle.checkpoint, fine_archive=bytes(fine)), replace(bundle.checkpoint, dense_archive=dense)):
            with self.assertRaisesRegex(ValueError, 'byte conflict'):
                MappingBackendCheckpointStore(self.root).publish(changed, occupancy_asset=bundle.occupancy_asset, cloud_asset=bundle.cloud_asset)

    def test_symlink_hardlink_and_traversal_members_are_rejected(self):
        stored = self.bundle().publish_to(self.root)
        path = stored.member_paths['canonical_occupancy']
        backup = path.read_bytes()
        path.unlink()
        external = self.root / 'external'
        external.write_bytes(backup)
        path.symlink_to(external)
        with self.assertRaises(ValueError):
            MappingBackendCheckpointStore(self.root).load('voxel', schema_version=2)
        path.unlink()
        os.link(external, path)
        with self.assertRaises(ValueError):
            MappingBackendCheckpointStore(self.root).load('voxel', schema_version=2)
        path.unlink()
        path.write_bytes(backup)
        manifest = json.loads(stored.manifest_path.read_text())
        manifest['members']['canonical_occupancy']['path'] = '../../external'
        stored.manifest_path.write_bytes(store_module._encode_json(manifest))
        with self.assertRaises(ValueError):
            MappingBackendCheckpointStore(self.root).load('voxel', schema_version=2)

    def test_publish_does_not_overwrite_unknown_existing_artifact(self):
        bundle = self.bundle()
        backend = self.root / 'mapping_backend/voxel'
        (backend / 'objects').mkdir(parents=True)
        (backend / 'manifests').mkdir()
        unknown = backend / 'objects' / ('id-' + 'b' * 32 + '.bin')
        unknown.write_bytes(fine_bytes())
        with self.assertRaisesRegex(ValueError, 'lacks its immutable descriptor'):
            bundle.publish_to(self.root)
        self.assertEqual(unknown.read_bytes(), fine_bytes())

    def test_sealed_reupload_replays_all_chunks(self):
        bundle = self.bundle()
        stub = TransferStub(self.response)
        uploaded = runtime(stub)._upload_graph_artifact(bundle.checkpoint, payload=bundle.checkpoint.fine_archive)
        self.assertEqual(b''.join(request.data for request in stub.requests if request.action == 2), fine_bytes())
        self.assertEqual(uploaded, self.response.fine_map_artifact)

    def test_default_export_remains_explicitly_legacy(self):
        stub = TransferStub(self.response)
        with self.assertRaisesRegex(ValueError, 'different checkpoint schema'):
            runtime(stub).export_mapping_backend_checkpoint()
        self.assertEqual(stub.exports, [1])

    def test_distinct_roles_cannot_reuse_artifact_id(self):
        response = copy.deepcopy(self.response)
        response.dense_cloud_artifact.id = response.fine_map_artifact.id
        with self.assertRaisesRegex(ValueError, 'reuse one artifact ID'):
            self.bundle(response).publish_to(self.root)
        self.assertFalse((self.root / 'mapping_backend').exists())

    def test_upload_rejects_forged_reference_and_changed_bytes_before_transfer(self):
        bundle = self.bundle()
        stub = TransferStub(self.response)
        client = runtime(stub)
        forged = replace(bundle.checkpoint.fine_reference, id='d' * 32)
        with self.assertRaisesRegex(ValueError, 'not a retained'):
            client._upload_graph_artifact(bundle.checkpoint, payload=bundle.checkpoint.fine_archive, reference=forged)
        changed = fine_bytes()[:-2] + struct.pack('<H', 2)
        with self.assertRaisesRegex(ValueError, 'bytes differ'):
            client._upload_graph_artifact(bundle.checkpoint, payload=changed, reference=bundle.checkpoint.fine_reference)
        self.assertFalse(stub.requests)

    @unittest.skipUnless(hasattr(os, 'fork'), 'requires Linux fork for abrupt death')
    def test_abrupt_exit_after_atomic_install_can_restart_and_retry(self):
        for suffix in ('b' * 32 + '.json', 'a' * 32 + '-backend.bin', 'a' * 32 + '.json'):
            with tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                bundle = self.bundle()
                pid = os.fork()
                if pid == 0:
                    original = store_module._rename_noreplace_at
                    def die_after_install(directory_fd, source, target):
                        original(directory_fd, source, target)
                        if target == 'id-' + suffix:
                            os._exit(23)
                    store_module._rename_noreplace_at = die_after_install
                    bundle.publish_to(root)
                    os._exit(24)
                _, status = os.waitpid(pid, 0)
                self.assertTrue(os.WIFEXITED(status))
                self.assertEqual(os.WEXITSTATUS(status), 23)
                bundle.publish_to(root)
                loaded = MappingBackendCheckpointStore(root).load('voxel', schema_version=2)
                self.assertEqual(loaded.checkpoint.checkpoint_id, 'a' * 32)
                self.assertTrue(all(path.stat().st_nlink == 1 for path in loaded.member_paths.values()))

    def test_canonical_role_budgets_fail_before_publication(self):
        bundle = self.bundle()
        size = 64 * 1024 * 1024 + 1
        occupancy = dict(bundle.occupancy_asset, width=size, raw_data_size=size)
        changed = replace(bundle.checkpoint, canonical_occupancy_data=bytes(size))
        with self.assertRaisesRegex(ValueError, 'canonical occupancy bytes'):
            MappingBackendCheckpointStore(self.root).publish(changed, occupancy_asset=occupancy, cloud_asset=bundle.cloud_asset)
        self.assertFalse((self.root / 'mapping_backend').exists())
        cloud_size = (128 * 1024 * 1024 // 12 + 1) * 12
        receipt = dict(bundle.checkpoint.cloud_receipt, point_count=cloud_size // 12)
        changed = replace(bundle.checkpoint, canonical_cloud_xyz=bytes(cloud_size), cloud_receipt=receipt,
            cloud_identity=replace(bundle.checkpoint.cloud_identity, cloud_receipt=receipt),
            fine_archive=None, fine_reference=None, dense_archive=None, dense_reference=None)
        cloud = dict(bundle.cloud_asset, xyz_size=cloud_size, point_count=cloud_size // 12)
        with self.assertRaisesRegex(ValueError, 'canonical cloud bytes'):
            MappingBackendCheckpointStore(self.root).publish(changed, occupancy_asset=bundle.occupancy_asset, cloud_asset=cloud)
        self.assertFalse((self.root / 'mapping_backend').exists())

    def test_assigned_nil_ids_rejected_and_legacy_ids_stay_opaque(self):
        for mutate in (lambda r: setattr(r, 'checkpoint_id', '0' * 32), lambda r: setattr(r.fine_map_artifact, 'id', '0' * 32)):
            response = copy.deepcopy(self.response)
            mutate(response)
            with self.assertRaises(ValueError):
                self.bundle(response)
        reference = pb.GraphArtifactRef(id='legacy-native-artifact', size=1, sha256=b'Q' * 32)
        decoded = provider._artifact_reference_from_proto(reference)
        self.assertEqual(decoded.id, 'legacy-native-artifact')
        self.assertEqual(decoded.legacy_alias, b'Q' * 32)
        self.assertIsNone(decoded.member_identity)

    def test_multichunk_sealed_and_partial_reupload_use_retained_frontier(self):
        points = 100_000
        metadata = dict(version=1, session_id='source', session_generation=3, cloud_revision=7,
            source_timestamp_ns=11, frame_id='map', point_count=points, bounds_min=[1, 2, 3], bounds_max=[1, 2, 3])
        raw = json.dumps(metadata, separators=(',', ':')).encode()
        dense = b'NUWAXDC1' + struct.pack('<I', len(raw)) + raw + struct.pack('<fff', 1, 2, 3) * points
        response = copy.deepcopy(self.response)
        response.dense_cloud_artifact.size = len(dense)
        bundle = provider._checkpoint_export_bundle_from_proto(response, fine_payload=fine_bytes(), dense_payload=dense)
        for initial_size, sealed in ((len(dense), True), (1_100_000, False), (1_048_576, False), (0, False)):
            stub = TransferStub(response)
            stub.entries[response.dense_cloud_artifact.id] = [copy.deepcopy(response.dense_cloud_artifact), bytearray(dense[:initial_size]), sealed]
            original = stub.TransferGraphArtifact
            def transfer(request, timeout):
                if request.action == 2:
                    retained, data, was_sealed = stub.entries[request.artifact.id]
                    if request.offset < len(data) < request.offset + len(request.data):
                        overlap = len(data) - request.offset
                        self.assertEqual(data[request.offset:], request.data[:overlap])
                        data.extend(request.data[overlap:])
                return original(request, timeout)
            stub.TransferGraphArtifact = transfer
            uploaded = runtime(stub)._upload_graph_artifact(bundle.checkpoint, payload=dense, reference=bundle.checkpoint.dense_reference)
            self.assertEqual(uploaded, response.dense_cloud_artifact)
            writes = [request for request in stub.requests if request.action == 2]
            self.assertEqual(len(writes), 2)
            self.assertEqual(b''.join(request.data for request in writes), dense)
            self.assertEqual(bytes(stub.entries[uploaded.id][1]), dense)

    def test_upload_checks_initial_and_final_frontier(self):
        bundle = self.bundle()
        for action, offset, sealed in ((1, len(fine_bytes()) + 1, False), (1, 0, True), (3, 0, True)):
            stub = TransferStub(self.response)
            original = stub.TransferGraphArtifact
            def changed(request, timeout):
                result = original(request, timeout)
                if request.action == action:
                    result.offset = offset
                    result.sealed = sealed
                return result
            stub.TransferGraphArtifact = changed
            with self.assertRaises(RuntimeError):
                runtime(stub)._upload_graph_artifact(bundle.checkpoint, payload=bundle.checkpoint.fine_archive)

    def test_cloud_target_kind_is_empty_and_unadmitted_owner_cannot_upload(self):
        response = copy.deepcopy(self.response)
        response.target_kind = 'cloud'
        with self.assertRaisesRegex(ValueError, 'target kind'):
            self.bundle(response)
        bundle = self.bundle()
        malformed = replace(bundle.checkpoint, fine_archive=b'')
        stub = TransferStub(self.response)
        with self.assertRaisesRegex(ValueError, 'retained owner type/size'):
            runtime(stub)._upload_graph_artifact(malformed, payload=fine_bytes(), reference=malformed.fine_reference)
        self.assertFalse(stub.requests)


if __name__ == '__main__':
    unittest.main(verbosity=2)
