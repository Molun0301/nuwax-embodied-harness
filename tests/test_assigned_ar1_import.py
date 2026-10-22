"""Capability-fenced AR1 source imports, with real stores and fake native RPCs."""
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import tests.test_legacy_cloud_import as legacy
from tests.test_legacy_cloud_import import fine_archive, inventory
from tests.conftest import LoaderStub
from tests.conftest import response_fixture
from services.geometry import geometry_runtime_provider as provider
from services.geometry import runtime as runtime_module
from services.mapping import asset_store
from contracts.geometry_runtime import PlatformGeometryConfig


class AssignedAr1ImportTests(unittest.TestCase):
    def setUp(self):
        # This installs the hash sentinels before any fixture construction.
        self.fixture = legacy.LegacyCloudImportTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)

    def source(self, version=1, *, dense=True):
        f = self.fixture
        f.prepare(dense=dense)
        payload = fine_archive(version)
        f.add_fine_archive(payload)
        return payload

    def connect(self, versions=(1, 2, 3), *, client=None, absent=False):
        stub = LoaderStub(response_fixture())
        status = (provider.pb2.RuntimeStatus(phase='READY') if absent else
            provider.pb2.RuntimeStatus(phase='READY', supported_assigned_fine_archive_versions=versions))
        stub.GetStatus = Mock(return_value=status)
        if client is None:
            client = provider.GeometryRuntimeProvider(SimpleNamespace(
                is_running=True, grpc_target='unused-fixture', disconnect=Mock()))
        with patch.object(provider.grpc, 'insecure_channel', return_value=SimpleNamespace(close=Mock())), \
             patch.object(provider.pb2_grpc, 'GeometryRuntimeStub', return_value=stub):
            self.assertTrue(client.connect())
        stub.owner = client
        return client, stub

    def runtime(self, client):
        value = runtime_module.PlatformGeometryRuntime(config=PlatformGeometryConfig(),
            catalog_root=str(self.fixture.root), robot=SimpleNamespace(providers=SimpleNamespace()))
        value._remote_provider = client
        value._provider_attach_epoch = 1
        return value

    def workspace(self, client):
        owner = self.fixture.service()
        owner._geometry_runtime = self.runtime(client)
        return owner

    def imported(self, client):
        return self.workspace(client)._import_legacy_cloud_for_activation('map')

    @staticmethod
    def request(result):
        return dict(activation_request_id='activate:ar1', session_id='mapload:ar1', map_name='map',
            checkpoint_id=result.stored_checkpoint.checkpoint.checkpoint_id,
            asset_identity=result.bundle.asset_identity.model_dump(), expected_runtime_generation=7)

    def test_status_capability_uses_one_existing_connect_rpc_and_defaults_fail_closed(self):
        client = None
        for versions, absent, supports in (((), True, False), ((), False, False),
                                          ((99,), False, False), ((1, 2, 3), False, True),
                                          ((1, 2, 3, 4), False, True)):
            with self.subTest(versions=versions, absent=absent):
                client, stub = self.connect(versions, client=client, absent=absent)
                origin = client.capture_assigned_activation_origin()
                for _ in range(3):
                    self.assertEqual(client.supports_assigned_fine_archive_version(1, expected_origin=origin), supports)
                    self.assertTrue(client.supports_assigned_fine_archive_version(2, expected_origin=origin))
                    self.assertTrue(client.supports_assigned_fine_archive_version(3, expected_origin=origin))
                    self.assertEqual(client.supports_assigned_fine_archive_version(4, expected_origin=origin),
                                     4 in versions and not absent)
                    self.assertFalse(client.supports_assigned_fine_archive_version(99, expected_origin=origin))
                self.assertEqual(stub.GetStatus.call_count, 1)
        current = client.capture_assigned_activation_origin()
        client._cleanup_after_failed_connect()
        self.assertEqual(client._assigned_fine_archive_versions, frozenset())
        with self.assertRaises(RuntimeError):
            client.supports_assigned_fine_archive_version(1, expected_origin=current)

    def test_ar1_on_disk_import_and_normal_provider_activation_keep_exact_source(self):
        payload = self.source()
        before = inventory(self.fixture.current)
        client, stub = self.connect()
        result = self.imported(client)
        self.assertIsInstance(result, asset_store._AssignedActivationRead)
        checkpoint = result.stored_checkpoint.checkpoint
        self.assertEqual(bytes(checkpoint.fine_archive), payload)
        member = checkpoint.fine_reference.member_identity
        self.assertEqual((member.frame_generation, member.producer_generation), (4, 8))
        self.assertEqual(member.cloud_receipt['cloud_revision'], 7)
        self.assertEqual(member.cloud_receipt, checkpoint.cloud_receipt)
        self.assertEqual(checkpoint.fine_reference.legacy_alias, b'')
        self.assertEqual(self.fixture.catalog.get_map_asset('map').asset_identity, result.bundle.asset_identity)
        prepared = self.runtime(client).prepare_retained_assigned_map_activation(result,
            repository=self.fixture.repository, bundle_root=result.root)
        self.assertIs(prepared.stored_checkpoint, result.stored_checkpoint)
        returned = client.activate_assigned_platform_map(activation_request=self.request(result),
            stored_checkpoint=result.stored_checkpoint)
        self.assertEqual(returned['activation_receipt_schema_version'], 2)
        fine_id = checkpoint.fine_reference.id
        transmitted = b''.join(bytes(request.data) for request in stub.requests
                               if request.action == 2 and request.artifact.id == fine_id)
        self.assertEqual(transmitted, payload)
        self.assertEqual(inventory(self.fixture.current), before)
        self.assertEqual(stub.GetStatus.call_count, 1)

    def test_missing_unknown_and_old_capabilities_leave_ar1_unselected_without_allocation(self):
        self.source()
        before = inventory(self.fixture.current)
        client = None
        for versions, absent in (((), True), ((), False), ((99,), False), ((2, 3), False)):
            client, _ = self.connect(versions, client=client, absent=absent)
            workspace = self.workspace(client)
            with patch.object(asset_store.uuid, 'uuid4', side_effect=AssertionError('unsupported allocation')):
                self.assertIsNone(workspace._import_legacy_cloud_for_activation('map'))
            self.assertEqual(self.fixture.catalog.get_map_asset('map').identity_schema_version, 1)
            self.assertEqual(list(self.fixture.repository._map_dir('map').glob('assigned-id-*')), [])
            self.assertEqual(inventory(self.fixture.current), before)

    def test_ar1_malformed_or_receipt_mismatched_source_never_adopts_catalog(self):
        self.source()
        client, _ = self.connect()
        for payload in (fine_archive(1)[:-1], fine_archive(1, domain_update={'generation': 4}),
                        fine_archive(1, root_update={'revision': 8})):
            with self.subTest(payload_size=len(payload)):
                self.fixture.add_fine_archive(payload)
                with self.assertRaises(ValueError):
                    self.imported(client)
                self.assertEqual(self.fixture.catalog.get_map_asset('map').identity_schema_version, 1)
                self.assertEqual(list(self.fixture.repository._map_dir('map').glob('assigned-id-*')), [])

    def test_older_status_decoder_ignores_additive_capability_field(self):
        from google.protobuf import descriptor_pb2, descriptor_pool, message_factory
        old = descriptor_pb2.FileDescriptorProto(name='old_runtime_status.proto', syntax='proto3')
        status = old.message_type.add(name='RuntimeStatus')
        provider.pb2.RuntimeStatus.DESCRIPTOR.CopyToProto(status)
        del status.field[5:]
        pool = descriptor_pool.DescriptorPool()
        pool.Add(old)
        old_type = message_factory.GetMessageClass(pool.FindMessageTypeByName('RuntimeStatus'))
        current = provider.pb2.RuntimeStatus(phase='READY', rss_kb=12,
            supported_assigned_fine_archive_versions=[1, 2, 3])
        decoded = old_type.FromString(current.SerializeToString())
        self.assertEqual((decoded.phase, decoded.rss_kb), ('READY', 12))
        self.assertFalse(hasattr(decoded, 'supported_assigned_fine_archive_versions'))
        old_response = old_type(phase='READY').SerializeToString()
        self.assertEqual(list(provider.pb2.RuntimeStatus.FromString(old_response)
            .supported_assigned_fine_archive_versions), [])

    def test_ar2_still_imports_without_advertised_capability(self):
        payload = self.source(2, dense=False)
        client, _ = self.connect(absent=True)
        result = self.imported(client)
        self.assertEqual(bytes(result.stored_checkpoint.checkpoint.fine_archive), payload)
        self.assertIsNone(result.stored_checkpoint.checkpoint.dense_archive)

    def test_reconnect_during_import_prevents_catalog_adoption_and_retry_reuses_owner(self):
        payload = self.source()
        client, _ = self.connect()
        owner = self.workspace(client)
        real_import = self.fixture.repository.import_legacy_cloud_bundle
        retained = []
        def changed(*args, **kwargs):
            result = real_import(*args, **kwargs)
            retained.append(result)
            self.connect((2, 3), client=client)
            return result
        with patch.object(self.fixture.repository, 'import_legacy_cloud_bundle', side_effect=changed):
            with self.assertRaisesRegex(RuntimeError, 'connection changed'):
                owner._import_legacy_cloud_for_activation('map')
        self.assertEqual(self.fixture.catalog.get_map_asset('map').identity_schema_version, 1)
        self.assertEqual(len(list(self.fixture.repository._map_dir('map').glob('assigned-id-*'))), 1)
        self.connect(client=client)
        with patch.object(asset_store.uuid, 'uuid4', side_effect=AssertionError('retry allocation')):
            retried = owner._import_legacy_cloud_for_activation('map')
        self.assertEqual(retried.bundle.asset_identity, retained[0].bundle.asset_identity)
        self.assertEqual(bytes(retried.stored_checkpoint.checkpoint.fine_archive), payload)

    def test_reconnect_after_import_precheck_before_catalog_adoption_is_rejected(self):
        from skills import assigned_map_save
        self.source()
        client, _ = self.connect()
        owner = self.workspace(client)
        reconcile = assigned_map_save.reconcile_assigned_current
        def changed(*args, **kwargs):
            self.connect((2, 3), client=client)
            return reconcile(*args, **kwargs)
        with patch.object(assigned_map_save, 'reconcile_assigned_current', side_effect=changed):
            with self.assertRaisesRegex(RuntimeError, 'connection changed'):
                owner._import_legacy_cloud_for_activation('map')
        self.assertEqual(self.fixture.catalog.get_map_asset('map').identity_schema_version, 1)

    def test_reconnect_after_catalog_file_fsync_before_rename_preserves_old_catalog(self):
        from services.mapping import catalog
        import os
        self.source()
        client, _ = self.connect()
        owner = self.workspace(client)
        atomic = catalog._atomic_write_text
        fsync = os.fsync
        interleaved = []
        def write(path, text, **kwargs):
            def before_rename(fd):
                fsync(fd)
                if '.metadata.json.' in os.readlink('/proc/self/fd/%d' % fd):
                    self.assertFalse(client._map_state_lock.locked())
                    self.assertFalse(owner._geometry_runtime._runtime_state_lock._is_owned())
                    self.connect((2, 3), client=client)
                    interleaved.append(True)
            with patch.object(catalog.os, 'fsync', side_effect=before_rename):
                return atomic(path, text, **kwargs)
        with patch.object(catalog, '_atomic_write_text', side_effect=write):
            with self.assertRaisesRegex(RuntimeError, 'connection changed'):
                owner._import_legacy_cloud_for_activation('map')
        self.assertEqual(interleaved, [True])
        self.assertEqual(self.fixture.catalog.get_map_asset('map').identity_schema_version, 1)
        self.assertEqual(list(self.fixture.current.parent.glob('.metadata.json.*.tmp')), [])

    def test_reconnect_after_guarded_catalog_rename_allows_durable_confirmation(self):
        from services.mapping import catalog
        self.source()
        client, _ = self.connect()
        owner = self.workspace(client)
        atomic = catalog._atomic_write_text
        directory_fsync = catalog._fsync_directory
        replace = catalog.os.replace
        order = []
        def write(path, text, **kwargs):
            def commit(source, target):
                self.assertTrue(client._map_state_lock.locked())
                self.assertTrue(owner._geometry_runtime._runtime_state_lock._is_owned())
                replace(source, target)
                order.append('rename')
            def after_rename(directory):
                self.assertFalse(client._map_state_lock.locked())
                self.assertFalse(owner._geometry_runtime._runtime_state_lock._is_owned())
                self.connect((2, 3), client=client)
                order.append('reconnect')
                directory_fsync(directory)
            with patch.object(catalog.os, 'replace', side_effect=commit), \
                 patch.object(catalog, '_fsync_directory', side_effect=after_rename):
                return atomic(path, text, **kwargs)
        with patch.object(catalog, '_atomic_write_text', side_effect=write):
            result = owner._import_legacy_cloud_for_activation('map')
        self.assertEqual(order, ['rename', 'reconnect'])
        self.assertEqual(self.fixture.catalog.get_map_asset('map').asset_identity, result.bundle.asset_identity)
        # Later upload still requires the now-current server to support this payload.
        with self.assertRaisesRegex(RuntimeError, 'does not support'):
            client.activate_assigned_platform_map(activation_request=self.request(result),
                stored_checkpoint=result.stored_checkpoint)

    def test_provider_attach_replacement_prevents_catalog_adoption(self):
        self.source()
        client, _ = self.connect()
        owner = self.workspace(client)
        replacement, _ = self.connect((2, 3))
        real_import = self.fixture.repository.import_legacy_cloud_bundle
        def changed(*args, **kwargs):
            result = real_import(*args, **kwargs)
            owner._geometry_runtime._remote_provider = replacement
            owner._geometry_runtime._provider_attach_epoch += 1
            return result
        with patch.object(self.fixture.repository, 'import_legacy_cloud_bundle', side_effect=changed):
            with self.assertRaisesRegex(RuntimeError, 'ownership changed'):
                owner._import_legacy_cloud_for_activation('map')
        self.assertEqual(self.fixture.catalog.get_map_asset('map').identity_schema_version, 1)

    def test_saved_ar1_old_connection_rejects_before_preparation_dispatch_or_upload(self):
        self.source()
        client, _ = self.connect()
        result = self.imported(client)
        checkpoint = result.stored_checkpoint.checkpoint
        _, old_stub = self.connect((2, 3), client=client)
        origin = client.capture_assigned_activation_origin()
        with self.assertRaisesRegex(RuntimeError, 'does not support'):
            client.activate_assigned_platform_map(activation_request=self.request(result),
                stored_checkpoint=result.stored_checkpoint)
        with self.assertRaisesRegex(RuntimeError, 'does not support'):
            client._upload_graph_artifact(checkpoint, payload=checkpoint.fine_archive,
                reference=checkpoint.fine_reference, expected_origin=origin)
        runtime = self.runtime(client)
        with self.assertRaisesRegex(RuntimeError, 'does not support'):
            runtime.prepare_retained_assigned_map_activation(result,
                repository=self.fixture.repository, bundle_root=result.root)
        self.assertEqual(old_stub.calls, [])
        self.assertEqual(old_stub.requests, [])
        self.assertFalse(client._mapping_transition_inflight)

    def test_replaced_origin_cannot_upload_even_if_new_server_also_supports_ar1(self):
        self.source()
        client, _ = self.connect()
        result = self.imported(client)
        origin = client.capture_assigned_activation_origin()
        _, new_stub = self.connect(client=client)
        checkpoint = result.stored_checkpoint.checkpoint
        with self.assertRaisesRegex(RuntimeError, 'connection changed'):
            client._upload_graph_artifact(checkpoint, payload=checkpoint.fine_archive,
                reference=checkpoint.fine_reference, expected_origin=origin)
        self.assertEqual(new_stub.requests, [])


if __name__ == '__main__':
    unittest.main()
