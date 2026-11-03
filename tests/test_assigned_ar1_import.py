"""Capability-fenced AR1 source imports, with real stores and fake native RPCs."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import tests.test_legacy_cloud_import as legacy
from tests.test_legacy_cloud_import import fine_archive
from tests.conftest import LoaderStub
from tests.conftest import response_fixture
from services.geometry import geometry_runtime_provider as provider
from services.geometry import runtime as runtime_module
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










if __name__ == '__main__':
    unittest.main()
