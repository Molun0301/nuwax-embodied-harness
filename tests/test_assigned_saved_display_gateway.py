"""Assigned retained display through actual gateway/cache/page boundaries (CPU only)."""
import asyncio
import base64
import json
import os
import struct
import threading
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import tests.test_assigned_saved_cloud_sources as fixtures

from contracts.streams import StreamFrame, StreamPayloadKind
from gateways.runtime import GatewayRuntime, _GatewayObservedPointCloudMapSource, _GatewaySavedPointCloudMapSource
from gateways.ws.data_streams import _legacy_pointcloud_frame
from gateways.ws.pointcloud_stream import PointCloudTransfer, snapshot_identity
from services.geometry.dense_cloud import SavedDenseCloudCache, read_dense_metadata
from services.geometry.pointcloud_map import (
    AssignedSavedPointCloudMapSource,
    PointCloudMapSourceError,
    PointCloudMapSourceSwitch,
)


class AssignedSavedDisplayGatewayTests(unittest.TestCase):
    CANONICAL_POINTS = 5000
    DENSE_POINTS = 65539

    def setUp(self):
        from tests.conftest import create_isolated_gateway
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.gateway = create_isolated_gateway(Path(directory.name))
        self.addCleanup(self.gateway.stop)
        fixtures.AssignedSavedCloudTests.setUp(self)
        self.handle = self.runtime.capture_assigned_saved_cloud_source()
        self.source = AssignedSavedPointCloudMapSource(self.handle)
        self.addCleanup(self.source.close)
        self.snapshot = self.source.read()
        binding = patch.object(self.gateway, "platform_geometry_runtime", self.runtime)
        binding.start()
        self.addCleanup(binding.stop)
        self.gateway._pointcloud_map_current_snapshot = self.snapshot

    def replace_installation(self):
        with self.runtime._runtime_state_lock:
            old = self.runtime._map_activation_snapshot
            new = replace(old, generation=old.generation + 1)
            self.runtime._map_activation_snapshot = new
            self.runtime._assigned_map_installation_origin = (new, self.prepared)

    def dense(self, **kwargs):
        return self.gateway.get_dense_cloud_chunk(expected_source_epoch=self.handle.source_epoch, **kwargs)

    def test_initial_dense_response_matches_display_owner_and_original_archive_source(self):
        metadata, data = self.dense()
        original, offset = read_dense_metadata(self.handle.dense_archive)
        self.assertEqual(metadata['assigned_source'], self.handle.assigned_source)
        self.assertEqual(metadata['display_source_epoch'], self.snapshot.source_epoch)
        self.assertEqual(metadata['epoch'], self.snapshot.source_epoch)
        self.assertEqual(metadata['session_id'], original['session_id'])
        self.assertEqual(data, self.handle.dense_archive[offset:offset + 65536*12])
        self.assertNotEqual(metadata['session_id'], self.provider._stub.state.session_id)

    def test_missing_or_wrong_initial_display_identity_rejects_before_slice(self):
        with patch.object(self.gateway._saved_dense_cloud_cache, 'read', side_effect=AssertionError('premature slice')):
            for expected in ('', 'assigned:other'):
                with self.assertRaisesRegex(RuntimeError, 'SOURCE_CHANGED'):
                    self.gateway.get_dense_cloud_chunk(expected_source_epoch=expected)

    def test_cached_snapshot_retired_by_reinstallation_cannot_authorize_dense(self):
        self.replace_installation()
        self.assertIsNone(self.gateway.get_pointcloud_map_snapshot())
        with self.assertRaisesRegex(RuntimeError, 'SOURCE_CHANGED'):
            self.dense()

    def test_owner_changes_during_dense_read_discard_the_chunk(self):
        read = self.gateway._saved_dense_cloud_cache.read
        def change(*args, **kwargs):
            value = read(*args, **kwargs)
            self.replace_installation()
            return value
        with patch.object(self.gateway._saved_dense_cloud_cache, 'read', side_effect=change), self.assertRaises(RuntimeError):
            self.dense()

    def test_configured_pcd_display_cannot_read_an_assigned_dense_owner(self):
        self.gateway._pointcloud_map_current_snapshot = replace(self.snapshot,
            assigned_source_owner=None, source_epoch='path:configured', provenance={})
        with self.assertRaisesRegex(RuntimeError, 'SOURCE_CHANGED'):
            self.gateway.get_dense_cloud_chunk(expected_source_epoch='path:configured')

    def test_closed_gateway_and_activation_fence_reject(self):
        self.gateway._closed = True
        with self.assertRaises(RuntimeError): self.dense()
        self.gateway._closed = False
        with patch.object(self.runtime, 'is_map_asset_activation_recovery_pending', return_value=True):
            self.assertIsNone(self.gateway.get_pointcloud_map_snapshot())
            with self.assertRaises(RuntimeError): self.dense()

    def test_display_stays_offline_without_provider_ready(self):
        self.provider._connected = False
        self.assertIs(self.gateway.get_pointcloud_map_snapshot(), self.snapshot)
        metadata, _data = self.dense()
        self.assertEqual(metadata['assigned_source'], self.handle.assigned_source)

    def test_saved_gateway_delegate_bypasses_pcd_and_reuses_exact_pages(self):
        source = _GatewaySavedPointCloudMapSource(lambda: (_ for _ in ()).throw(AssertionError('PCD summary')),
            cache=None, default_frame_id='map', assigned_source_provider=self.runtime.capture_assigned_saved_cloud_source)
        self.addCleanup(source.close)
        first, second = source.read(), source.read()
        self.assertIs(first, second)
        self.assertIs(first.chunks, second.chunks)
        self.assertEqual(first.provenance['assigned_source'], self.handle.assigned_source)

    def test_gateway_preserves_explicit_display_budget_error(self):
        source = _GatewaySavedPointCloudMapSource(dict, cache=None, default_frame_id='map',
            assigned_source_provider=self.runtime.capture_assigned_saved_cloud_source)
        self.addCleanup(source.close)
        failure = PointCloudMapSourceError('saved_display_budget_exceeded', '32 MiB display budget',
            source='saved', terminal=True)
        with patch('gateways.runtime.AssignedSavedPointCloudMapSource', side_effect=failure), self.assertRaises(PointCloudMapSourceError) as error:
            source.read()
        self.assertIs(error.exception, failure)
        self.assertTrue(error.exception.terminal)

    def test_assigned_summary_without_retained_owner_never_uses_pcd(self):
        source = _GatewaySavedPointCloudMapSource(lambda: {'identity_schema_version': 2,
                'available': True, 'pcd_path': '/should-not-open.pcd'},
            cache=None, default_frame_id='map', assigned_source_provider=lambda: None)
        self.addCleanup(source.close)
        with self.assertRaises(PointCloudMapSourceError): source.read()

    def test_stale_live_shm_falls_back_but_current_live_session_remains_preferred(self):
        class Static:
            def __init__(self, value): self.value, self.source = value, value.source
            def open(self): pass
            def close(self): pass
            def cancel(self): pass
            def read(self, _cancel=None): return self.value
        saved = Static(self.snapshot)
        live = Static(replace(self.snapshot, source='live', assigned_source_owner=None,
            source_epoch='shm:old', provenance={'mapping_source_epoch': 'old:1'}))
        observed = _GatewayObservedPointCloudMapSource(live, lambda _error: None,
            snapshot_validator=self.gateway._require_pointcloud_source_current)
        switch = PointCloudMapSourceSwitch(observed, saved)
        self.addCleanup(switch.close)
        self.assertEqual(switch.read().source, 'saved')
        live.value = replace(live.value, source_epoch='shm:current',
            provenance={'mapping_source_epoch': self.handle.installed_mapping_source_epoch})
        self.assertEqual(switch.read().source, 'live')

    def test_assigned_legacy_adapter_is_bounded_unavailable_without_materialization(self):
        frame = StreamFrame(topic='pointcloud_map.snapshot', source='test', cursor=1,
            payload={'chunked': True}, metadata={'chunked': True})
        with patch('base64.b64encode', side_effect=AssertionError('full base64')):
            value = _legacy_pointcloud_frame(frame, self.snapshot)
        self.assertEqual(value.status, 'unavailable')
        self.assertEqual(value.payload_kind, StreamPayloadKind.JSON)
        self.assertEqual(value.payload['reason'], 'assigned_saved_requires_page_stream')

    def test_assigned_transport_is_full_reset_with_exact_typed_identity(self):
        transfer = PointCloudTransfer(self.snapshot, snapshot_identity(self.snapshot), 'assigned1')
        packet = transfer.next_packet()
        _magic, _version, size = struct.unpack_from('<8sBI', packet)
        header = json.loads(packet[13:13+size])
        self.assertTrue(header['reset'])
        self.assertEqual(header['base_revision_ordinal'], '0')
        self.assertFalse(header['source_incremental'])
        self.assertEqual(header['assigned_source'], self.handle.assigned_source)
        self.assertEqual(header['mapping_source_epoch'], self.handle.source_epoch)
        self.assertEqual(packet[13+size:], self.handle.canonical_cloud_xyz[:4096*12])
        self.assertFalse(transfer.complete)
        self.assertEqual(transfer.packet_count, 2)
        self.assertLess(len(packet), 64*1024)

    def test_legacy_cached_saved_and_missing_origin_live_are_not_displayed(self):
        legacy = replace(self.snapshot, assigned_source_owner=None, source_epoch='path:old', provenance={})
        self.gateway._pointcloud_map_current_snapshot = legacy
        self.assertIsNone(self.gateway.get_pointcloud_map_snapshot())
        self.runtime._assigned_map_installation_origin = None
        self.gateway._pointcloud_map_current_snapshot = replace(legacy, source='live')
        self.assertIsNone(self.gateway.get_pointcloud_map_snapshot())
        with self.assertRaises(RuntimeError): self.gateway.get_dense_cloud_chunk()

    def test_same_live_source_revision_during_dense_rpc_remains_usable(self):
        live = replace(self.snapshot, source='live', assigned_source_owner=None, source_epoch='live:one',
            provenance={'mapping_source_epoch': self.handle.installed_mapping_source_epoch})
        self.gateway._pointcloud_map_current_snapshot = live
        def native(**kwargs):
            self.gateway._pointcloud_map_current_snapshot = replace(live, revision_ordinal=live.revision_ordinal+1)
            return {'epoch': 'native-dense'}, b'native-chunk'
        with patch.object(self.provider, 'get_dense_cloud_chunk', side_effect=native):
            metadata, data = self.gateway.get_dense_cloud_chunk(expected_source_epoch='live:one')
        self.assertEqual(data, b'native-chunk')
        self.assertEqual(metadata['display_source_epoch'], 'live:one')

    def test_publication_callback_replacement_stops_old_followup_topics(self):
        for change_on in ('pointcloud_map.snapshot', 'pointcloud_map.meta'):
            with self.subTest(change_on=change_on):
                # A fresh retained source for each callback boundary.
                self.handle = self.runtime.capture_assigned_saved_cloud_source()
                source = AssignedSavedPointCloudMapSource(self.handle)
                self.addCleanup(source.close)
                current = source.read()
                topics = []
                def observe(frame, *, topics=topics, change_on=change_on):
                    self.assertFalse(self.gateway._lock._is_owned())
                    topics.append(frame.topic)
                    if frame.topic == change_on:
                        self.replace_installation()
                subscription = self.gateway.stream_bus.subscribe(observe,
                    topics=('pointcloud_map.snapshot', 'pointcloud_map.meta', 'pointcloud_map.chunk'))
                try:
                    with self.assertRaises(PointCloudMapSourceError):
                        self.gateway._publish_pointcloud_map_snapshot(current, reason='test')
                finally:
                    self.gateway.stream_bus.unsubscribe(subscription)
                self.assertEqual(topics[-1], change_on)
                self.assertNotIn('pointcloud_map.chunk', topics)

    def test_real_assigned_wire_fixture(self):
        target = os.environ.get('NEH_POINTCLOUD_FIXTURE_DIR')
        if not target:
            return
        transfer = PointCloudTransfer(self.snapshot, None, 'assigned-wire')
        packets = []
        while not transfer.complete:
            packets.append(base64.b64encode(transfer.next_packet()).decode())
        path = Path(target) / 'assigned_saved_wire.json'
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({'packets': packets, 'point_count': self.snapshot.point_count,
            'source_epoch': self.handle.source_epoch, 'expected_xyz': [1,2,3]*self.CANONICAL_POINTS,
            'map_snapshot': {'metadata': {'identity_schema_version': 2,
                'asset_identity': self.handle.assigned_source['asset_identity'],
                'map_activation_generation': self.handle.installation_generation,
                'activation_receipt_v2': {'request': {'checkpoint_id': self.handle.checkpoint_id}}}}}))

    def test_http_await_boundary_rechecks_exact_retained_owner(self):
        from core.errors import GatewayError
        from gateways.http import api
        with patch('gateways.settings.SettingsApplication', return_value=SimpleNamespace()):
            router = api.build_http_router(self.gateway, SimpleNamespace())
        endpoint = next(route.endpoint for route in router.routes if route.path == '/api/pointcloud/dense')
        async def delayed(function):
            value = function()
            fixtures.AssignedSavedCloudTests.replace_origin(self)
            return value
        async def run():
            with patch.object(api, 'run_in_threadpool', side_effect=delayed):
                with self.assertRaises(GatewayError) as error:
                    await endpoint(None, epoch='', source_epoch=self.handle.source_epoch, offset=0, delta=False)
                self.assertEqual(error.exception.http_status, 409)
        asyncio.run(run())

    def test_http_matching_first_chunk_returns_typed_header_and_bounded_body(self):
        from gateways.http import api
        with patch('gateways.settings.SettingsApplication', return_value=SimpleNamespace()):
            router = api.build_http_router(self.gateway, SimpleNamespace())
        endpoint = next(route.endpoint for route in router.routes if route.path == '/api/pointcloud/dense')
        async def direct(function): return function()
        async def run():
            with patch.object(api, 'run_in_threadpool', side_effect=direct):
                response = await endpoint(None, epoch='', source_epoch=self.handle.source_epoch, offset=0, delta=False)
            metadata = json.loads(response.headers['X-Nuwax-Pointcloud'])
            self.assertEqual(metadata['assigned_source'], self.handle.assigned_source)
            self.assertEqual(metadata['display_source_epoch'], self.handle.source_epoch)
            self.assertEqual(len(response.body), 65536*12)
        asyncio.run(run())

    def test_assigned_socket_aborts_owner_replaced_between_packets(self):
        asyncio.run(self._socket_replacement(False))

    def test_assigned_socket_rechecks_owner_after_packet_encoding(self):
        asyncio.run(self._socket_replacement(True))

    async def _socket_replacement(self, during_encode):
        from gateways.ws import pointcloud_stream
        class Router:
            def websocket(self, _path):
                def register(fn): self.endpoint = fn; return fn
                return register
        class Socket:
            def __init__(self): self.incoming = asyncio.Queue(); self.packets = []; self.ready = asyncio.Event()
            async def accept(self): pass
            async def send_json(self, _value): self.ready.set()
            async def send_bytes(inner, value):
                inner.packets.append(value)
                if len(inner.packets) == 1: self.replace_installation()
            async def receive(self): return await self.incoming.get()
            async def close(self, **kwargs): pass
        router, socket = Router(), Socket()
        pointcloud_stream.install_pointcloud_stream(router, self.gateway)
        old_encode = pointcloud_stream.PointCloudTransfer.next_packet
        def encode_then_change(value):
            packet = old_encode(value)
            self.replace_installation()
            return packet
        with patch.object(pointcloud_stream.PointCloudTransfer, 'next_packet',
                encode_then_change if during_encode else old_encode):
            task = asyncio.create_task(router.endpoint(socket))
            await asyncio.wait_for(socket.ready.wait(), 1)
            await socket.incoming.put({'type':'websocket.receive', 'text':json.dumps({
                'type':'pointcloud.resume', 'base':None, 'mapping_source_epoch':self.handle.source_epoch})})
            await asyncio.sleep(.02)
            self.assertEqual(len(socket.packets), 0 if during_encode else 1)
            await socket.incoming.put({'type':'websocket.disconnect'})
            await asyncio.wait_for(task, 1)


if __name__ == '__main__':
    unittest.main()
