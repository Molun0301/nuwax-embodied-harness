"""Focused runtime preparation checks for public assigned-cloud saves."""
import hashlib
import inspect
import sqlite3
import tempfile
import threading
import unittest
from contextlib import ExitStack
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

from tests.conftest import (
    TransferStub,
    dense_bytes,
    fine_bytes,
    response_fixture,
)
from tests.conftest import (
    runtime as provider_fixture,
)

from contracts.geometry import Pose
from contracts.geometry_runtime import PlatformGeometryConfig
from contracts.map_workspace import AssetIdentityV2, MapAssetBundle
from contracts.maps import CostMap
from contracts.occupancy_cells import PackedOccupancyCells
from contracts.runtime_views import MapSnapshot
from services.geometry import checkpoint_payload
from services.geometry import geometry_runtime_provider as provider_module
from services.geometry import mapping_backend_checkpoint_store as store_module
from services.geometry.runtime import PlatformGeometryRuntime


class RuntimeAssignedSavePreparationTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.response = response_fixture()
        self.exported = provider_module._checkpoint_export_bundle_from_proto(
            self.response, fine_payload=fine_bytes(), dense_payload=dense_bytes())
        self.provider = provider_fixture(TransferStub(self.response))
        self.exports = []
        def export(**kwargs):
            self.assertFalse(self.instance._runtime_state_lock._is_owned())
            self.exports.append(kwargs)
            return self.exported
        self.provider.export_mapping_backend_checkpoint = export
        self.instance = PlatformGeometryRuntime.__new__(PlatformGeometryRuntime)
        self.instance.config = PlatformGeometryConfig()
        self.instance._runtime_state_lock = threading.RLock()
        self.instance._remote_provider = self.provider
        self.instance._provider_attach_epoch = 3
        self.context = {'active': True, 'session_id': 'source', 'session_generation': 3,
                        'map_name': 'map', 'mapping_backend_kind': 'voxel', 'revision': 6}
        self.instance._is_nuwax_mapping_active = lambda: True
        self.instance.get_mapping_session_context = lambda: dict(self.context)
        self.instance._resolve_mapping_backend = lambda: self.instance._remote_provider
        self.instance._managed_processes = {}
        self.instance._external_process_name = lambda: 'geometry'
        self.instance._locate_live_mapping_database = lambda: None
        self.instance._pcd_min_points = lambda: 1
        self.instance._is_usable_pcd_file = lambda path: path.is_file()
        self.instance._build_geometry_quality_metrics = lambda **kwargs: {
            'database': {'required': True, 'present': kwargs['database_metadata']['database_present']},
            'pcd': {'present': kwargs['pcd_metadata']['pcd_present']},
            'navigation_map': {'present': kwargs['nav2_map_metadata']['present']},
            'checkpoint_ready': True,
        }
        self.candidate = MapAssetBundle(
            map_name='map', source_name='geometry', frame_id='map',
            snapshot=MapSnapshot(
                map_name='map', source_name='geometry', source_epoch='source:3',
                update_sequence=6, frame_id='map',
                cost_map=CostMap(map_id='map', frame_id='map', width=2, height=1,
                    resolution_m=0.125, origin=Pose(frame_id='map'), data=[4.0, 5.0]),
                metadata={'source': {'note': 'before'}},
            ),
            metadata={'mapping_session_id': 'source', 'session_generation': 3},
        )

    def prepare(self):
        return self.instance.prepare_map_asset_bundle(
            self.candidate, bundle_root=self.root, checkpoint_schema_version=2)

    def test_v2_keeps_packed_owner_without_whole_model_or_cloud_copies(self):
        with ExitStack() as guard:
            for model in (MapAssetBundle, MapSnapshot):
                for method in ('model_copy', 'model_dump', 'model_dump_json', '__deepcopy__'):
                    guard.enter_context(patch.object(model, method,
                        side_effect=AssertionError('whole snapshot/bundle adapter called')))
            for method in ('to_list', '__iter__', '__deepcopy__'):
                guard.enter_context(patch.object(PackedOccupancyCells, method,
                    side_effect=AssertionError('whole occupancy expanded')))
            for method in ('canonical_cloud_checkpoint', 'canonical_occupancy_checkpoint'):
                guard.enter_context(patch.object(provider_module.MappingBackendCheckpointExportBundle, method,
                    side_effect=AssertionError('whole protobuf adapter called')))
            guard.enter_context(patch.object(provider_module, '_protobuf_payload_bytes',
                side_effect=AssertionError('whole cloud bytes copied')))
            guard.enter_context(patch.object(self.instance, '_materialize_backend_state',
                side_effect=AssertionError('live backend state copied')))
            for name in hashlib.algorithms_guaranteed | {'new', 'file_digest'}:
                if hasattr(hashlib, name):
                    guard.enter_context(patch.object(hashlib, name,
                        side_effect=AssertionError('application hash called')))
            for module in (checkpoint_payload, provider_module, store_module):
                guard.enter_context(patch.object(module, 'payload_digest',
                    side_effect=AssertionError('legacy payload digest called')))
            prepared = self.prepare()
        self.assertEqual(self.exports, [{'checkpoint_schema_version': 2}])
        self.assertEqual(prepared.identity_schema_version, 1)
        cells = prepared.snapshot.occupancy_grid.data
        self.assertIsInstance(cells, PackedOccupancyCells)
        self.assertIs(cells.raw, self.exported.checkpoint.canonical_occupancy_data)
        self.assertIs(prepared.snapshot.cost_map, self.candidate.snapshot.cost_map)
        self.assertEqual(cells, [-1, 100])
        pcd = self.root / prepared.geometry_asset.pcd_path
        self.assertEqual(pcd.read_bytes().split(b'DATA binary\n', 1)[1],
                         self.exported.checkpoint.canonical_cloud_xyz)
        pgm = self.instance._bundle_nav2_map_pgm_path(self.root)
        self.assertEqual(pgm.read_bytes(), b'P5\n2 1\n255\n\xcd\x00')
        with sqlite3.connect(self.root / prepared.geometry_asset.mapping_database_path) as connection:
            record = connection.execute(
                'SELECT backend_kind, backend_checkpoint_id, mapping_session_id FROM map_asset_identity').fetchone()
        self.assertEqual(record, ('voxel', 'a' * 32, 'source'))

        # The final assigned constructor is the only mutable-data capture.
        fields = {name: getattr(prepared, name) for name in type(prepared).model_fields}
        assigned = MapAssetBundle(**{**fields, 'identity_schema_version': 2,
            'asset_identity': AssetIdentityV2(asset_id='d' * 32), 'asset_state': 'candidate'})
        self.assertIs(assigned.snapshot.occupancy_grid.data, cells)
        self.candidate.snapshot.cost_map.data[0] = 99
        self.candidate.snapshot.metadata['source']['note'] = 'after'
        self.assertEqual(assigned.snapshot.cost_map.data, (4.0, 5.0))
        self.assertEqual(assigned.snapshot.metadata['source']['note'], 'before')

    def test_v2_reopened_occupancy_uses_one_compact_byte_capture(self):
        payload = checkpoint_payload.FrozenPayload([b'\xff\x64'])
        exported = replace(self.exported, checkpoint=replace(
            self.exported.checkpoint, canonical_occupancy_data=payload))
        updated = self.instance._replace_bundle_occupancy_from_checkpoint(self.candidate, exported)
        self.assertIsInstance(updated.snapshot.occupancy_grid.data, PackedOccupancyCells)
        self.assertEqual(updated.snapshot.occupancy_grid.data.raw, b'\xff\x64')

    def test_v2_rejects_invalid_occupancy_and_layer_identity(self):
        for raw in (b'\xfe\x64', b'\x65\x64', b'\xff'):
            with self.subTest(raw=raw):
                exported = replace(self.exported, checkpoint=replace(
                    self.exported.checkpoint, canonical_occupancy_data=raw))
                with self.assertRaisesRegex(RuntimeError, 'occupancy'):
                    self.instance._replace_bundle_occupancy_from_checkpoint(self.candidate, exported)
        for changes in ({'frame_id': ''}, {'frame_id': 'odom'}, {'resolution': 0.0}):
            with self.subTest(changes=changes):
                exported = replace(self.exported, occupancy_asset={**self.exported.occupancy_asset, **changes})
                with self.assertRaises((ValueError, RuntimeError)):
                    self.instance._replace_bundle_occupancy_from_checkpoint(self.candidate, exported)

    def test_v2_cannot_rewrite_an_assigned_or_sealed_owner(self):
        fields = {name: getattr(self.candidate, name) for name in type(self.candidate).model_fields}
        owners = (
            MapAssetBundle(**{**fields, 'manifest_sha256': 'e' * 64}),
            MapAssetBundle(**{**fields, 'identity_schema_version': 2,
                'asset_identity': AssetIdentityV2(asset_id='d' * 32), 'asset_state': 'candidate'}),
        )
        for owner in owners:
            with self.subTest(schema=owner.identity_schema_version):
                with self.assertRaisesRegex(TypeError, 'unsealed legacy draft'):
                    self.instance._replace_bundle_occupancy_from_checkpoint(owner, self.exported)
                with self.assertRaisesRegex(TypeError, 'unsealed legacy draft'):
                    self.instance.prepare_map_asset_bundle(owner, bundle_root=self.root, checkpoint_schema_version=2)
        self.assertEqual(self.exports, [])
        self.assertEqual(list(self.root.iterdir()), [])

    def test_v2_rejects_unknown_backend_before_export(self):
        for backend in ('unknown', None):
            self.context['mapping_backend_kind'] = backend
            with self.subTest(backend=backend), self.assertRaisesRegex(RuntimeError, 'active native backend'):
                self.prepare()
        self.assertEqual(self.exports, [])
        self.assertEqual(list(self.root.iterdir()), [])

    def test_v2_rejects_supported_planar_context_with_cloud_export(self):
        self.context['mapping_backend_kind'] = 'slam_toolbox'
        with self.assertRaisesRegex(RuntimeError, 'backend changed during export'):
            self.prepare()
        self.assertEqual(self.exports, [{'checkpoint_schema_version': 2}])
        self.assertEqual(list(self.root.iterdir()), [])

    def test_v2_rejects_export_backend_and_session_drift(self):
        self.context.update(runtime_generation=8, backend_generation=2)
        cases = (
            lambda: self.context.update(mapping_backend_kind='slam_toolbox'),
            lambda: self.context.update(session_generation=4),
            lambda: self.context.update(runtime_generation=9),
            lambda: self.context.update(backend_generation=3),
            lambda: setattr(self, 'exported', replace(self.exported,
                checkpoint=replace(self.exported.checkpoint, backend_kind='slam_toolbox'))),
        )
        for mutate in cases:
            with self.subTest(mutate=mutate):
                prior_context, prior_export = dict(self.context), self.exported
                def export(mutate=mutate, **kwargs):
                    mutate()
                    return self.exported
                self.provider.export_mapping_backend_checkpoint = export
                with self.assertRaisesRegex(RuntimeError, 'changed'):
                    self.prepare()
                self.context, self.exported = prior_context, prior_export
        self.assertEqual(list(self.root.iterdir()), [])

    def test_v2_rejects_missing_context_before_or_after_export(self):
        for contexts, expected_error, export_count in (
                ((None,), 'active native backend', 0),
                ((dict(self.context), None), 'backend changed during export', 1)):
            with self.subTest(contexts=contexts):
                self.exports.clear()
                values = iter(contexts)
                self.instance.get_mapping_session_context = lambda values=values: next(values)
                with self.assertRaisesRegex(RuntimeError, expected_error):
                    self.prepare()
                self.assertEqual(len(self.exports), export_count)
                self.assertEqual(list(self.root.iterdir()), [])

    def test_v2_rejects_runtime_and_provider_origin_drift_before_return(self):
        changes = (
            lambda: setattr(self.instance, '_provider_attach_epoch', 4),
            lambda: setattr(self.instance, '_remote_provider', provider_fixture(TransferStub(self.response))),
            lambda: setattr(self.provider, '_map_connection_epoch', 2),
        )
        materialize = self.instance._materialize_checkpoint_pcd_asset
        for index, mutate in enumerate(changes):
            with self.subTest(index=index):
                self.root = Path(self.directory.name) / str(index)
                self.root.mkdir()
                self.instance._provider_attach_epoch = 3
                self.instance._remote_provider = self.provider
                self.provider._map_connection_epoch = 1
                def materialize_then_replace(*args, mutate=mutate):
                    result = materialize(*args)
                    mutate()
                    return result
                with patch.object(self.instance, '_materialize_checkpoint_pcd_asset', materialize_then_replace), \
                        self.assertRaisesRegex(RuntimeError, 'ownership changed|connection changed'):
                    self.prepare()

    def test_schema1_preserves_list_and_independent_snapshot_copy(self):
        legacy_checkpoint = replace(self.exported.checkpoint, schema_version=1, content_sha256=b'X' * 32)
        legacy = replace(self.exported, checkpoint=legacy_checkpoint,
            canonical_occupancy_checkpoint_bytes=self.response.canonical_occupancy_checkpoint.SerializeToString())
        updated = self.instance._replace_bundle_occupancy_from_checkpoint(self.candidate, legacy)
        self.assertIs(type(updated.snapshot.occupancy_grid.data), list)
        self.assertEqual(updated.snapshot.occupancy_grid.data, [-1, 100])
        self.assertIsNot(updated.snapshot.cost_map, self.candidate.snapshot.cost_map)
        self.candidate.snapshot.cost_map.data[0] = 99
        self.assertEqual(updated.snapshot.cost_map.data, [4.0, 5.0])
        self.assertIn('content_sha256', updated.metadata['mapping_backend_checkpoint'])
        signature = inspect.signature(PlatformGeometryRuntime.prepare_map_asset_bundle)
        self.assertEqual(signature.parameters['checkpoint_schema_version'].default, 1)


class AssignedCheckpointExportOriginTests(unittest.TestCase):
    def test_export_rejects_replacement_before_artifact_reads(self):
        response = response_fixture()
        old, new = TransferStub(response), TransferStub(response)
        client = provider_fixture(old)
        export = old.ExportMappingBackendCheckpoint
        def swap(request, timeout):
            result = export(request, timeout)
            client._stub = new
            return result
        old.ExportMappingBackendCheckpoint = swap
        with self.assertRaisesRegex(RuntimeError, 'connection changed'):
            client.export_mapping_backend_checkpoint(checkpoint_schema_version=2)
        self.assertEqual([request.action for request in old.requests], [5, 5])
        self.assertEqual(old.entries, {})
        self.assertEqual(new.requests, [])

    def test_download_and_release_remain_on_export_stub_after_replacement(self):
        response = response_fixture()
        old, new = TransferStub(response), TransferStub(response)
        client = provider_fixture(old)
        transfer = old.TransferGraphArtifact
        def swap(request, timeout):
            result = transfer(request, timeout)
            if request.action == 4:
                client._stub = new
            return result
        old.TransferGraphArtifact = swap
        with self.assertRaisesRegex(RuntimeError, 'connection changed'):
            client.export_mapping_backend_checkpoint(checkpoint_schema_version=2)
        self.assertEqual([request.action for request in old.requests], [4, 5, 5])
        self.assertEqual(new.requests, [])
        self.assertEqual(old.entries, {})

    def test_failed_download_preserves_cancellation_and_releases_remaining_leases(self):
        response = response_fixture()
        stub = TransferStub(response)
        client = provider_fixture(stub)
        transfer = stub.TransferGraphArtifact
        cancellation = KeyboardInterrupt('cancel assigned save')
        released = []
        def fail(request, timeout):
            self.assertFalse(client._map_state_lock.locked())
            if request.action == 4:
                raise cancellation
            if request.action == 5:
                released.append(request.artifact.id)
                if request.artifact.id == response.fine_map_artifact.id:
                    raise RuntimeError('first cleanup failed')
            return transfer(request, timeout)
        stub.TransferGraphArtifact = fail
        with self.assertLogs(provider_module.LOGGER.name, level='ERROR'), \
                self.assertRaises(KeyboardInterrupt) as caught:
            client.export_mapping_backend_checkpoint(checkpoint_schema_version=2)
        self.assertIs(caught.exception, cancellation)
        self.assertEqual(released, [response.fine_map_artifact.id, response.dense_cloud_artifact.id])
        self.assertNotIn(response.dense_cloud_artifact.id, stub.entries)

    def test_successful_download_reports_cleanup_failure_after_all_releases(self):
        response = response_fixture()
        stub = TransferStub(response)
        client = provider_fixture(stub)
        transfer = stub.TransferGraphArtifact
        failure = RuntimeError('first cleanup failed')
        released = []
        def fail(request, timeout):
            if request.action == 5:
                released.append(request.artifact.id)
                if request.artifact.id == response.fine_map_artifact.id:
                    raise failure
            return transfer(request, timeout)
        stub.TransferGraphArtifact = fail
        with self.assertLogs(provider_module.LOGGER.name, level='ERROR'), \
                self.assertRaises(RuntimeError) as caught:
            client.export_mapping_backend_checkpoint(checkpoint_schema_version=2)
        self.assertIs(caught.exception, failure)
        self.assertEqual(released, [response.fine_map_artifact.id, response.dense_cloud_artifact.id])

    def test_invalid_artifact_is_never_released_using_an_unadmitted_reference(self):
        response = response_fixture()
        response.dense_cloud_artifact.member_identity.checkpoint_id = 'd' * 32
        stub = TransferStub(response)
        with self.assertRaisesRegex(ValueError, 'checkpoint role'):
            provider_fixture(stub).export_mapping_backend_checkpoint(checkpoint_schema_version=2)
        self.assertEqual([(request.action, request.artifact.id) for request in stub.requests],
                         [(5, response.fine_map_artifact.id)])
        self.assertIn(response.dense_cloud_artifact.id, stub.entries)

    def test_export_rejects_planar_reply_without_graph_member_before_download(self):
        response = response_fixture()
        response.backend_kind = 'slam_toolbox'
        stub = TransferStub(response)
        with self.assertRaisesRegex(ValueError, 'requires field graph_artifact'):
            provider_fixture(stub).export_mapping_backend_checkpoint(checkpoint_schema_version=2)
        self.assertEqual(stub.requests, [])

    def test_export_rejects_unknown_backend_before_envelope_or_download(self):
        response = response_fixture()
        response.backend_kind = 'unknown'
        stub = TransferStub(response)
        # Supported-backend admission precedes even the member-envelope parser.
        with patch.object(provider_module, '_validate_checkpoint_identity_envelope',
                          side_effect=AssertionError('unknown backend reached envelope')):
            with self.assertRaisesRegex(ValueError, 'requires a paired backend'):
                provider_fixture(stub).export_mapping_backend_checkpoint(checkpoint_schema_version=2)
        self.assertEqual(stub.requests, [])


if __name__ == '__main__':
    unittest.main()
