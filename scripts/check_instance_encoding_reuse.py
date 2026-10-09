#!/usr/bin/env python3
"""实例 WS 编码复用的离线合同及真实 ASGI 路由检查；不启动设备/网络服务。"""
from __future__ import annotations

import asyncio
import gc
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import weakref

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'scripts'))

from fastapi import FastAPI
from pydantic import ValidationError
from contracts.streams import InstanceSnapshotFrame, StreamFrame, StreamPayloadKind
from core.stream_bus import StreamBus
from gateways.ws import data_streams as ws
from gateways.ws.instance_encoding import InstanceSnapshotEncoder
from services.memory.instance_stream import SemanticInstancePublisher
from check_instance_stream_contract import memory_fixture, snapshot_fixture, NOW


def frame(cursor=1, payload=None):
    return InstanceSnapshotFrame(source='memory_service', cursor=cursor, timestamp=NOW,
                                 payload=payload if payload is not None else {'instances': [{'instance_id': str(cursor)}]})


class OwnershipChecks(unittest.TestCase):
    def test_real_source_transfers_fresh_public_rows_through_real_bus(self):
        bus = StreamBus()
        self.addCleanup(bus.shutdown)
        memory = memory_fixture(2)
        publisher = None
        original = bus.publish_frame
        def publish(value):
            result = original(value)
            publisher._stopping = True
            return result
        publisher = SemanticInstancePublisher(bus, memory.get_semantic_instances_snapshot)
        with patch('services.memory.service.utc_now', return_value=NOW), patch.object(bus, 'publish_frame', side_effect=publish):
            publisher._run()
        value = bus.latest_frame('memory.instances')
        self.assertIs(type(value), InstanceSnapshotFrame)
        self.assertNotIn('_next_update_at', value.payload)
        self.assertIs(bus.replay()[0], value)
        with self.assertRaises(ValidationError):
            value.source = 'changed'
        original_name = value.payload['instances'][0]['display_name']
        source = memory._semantic_instances_by_id[value.payload['instances'][0]['instance_id']]
        source.display_name = 'new source state'
        self.assertEqual(value.payload['instances'][0]['display_name'], original_name)
        fresh = memory.get_semantic_instances_snapshot()
        self.assertIsNot(fresh['instances'], value.payload['instances'])
        self.assertTrue(all(a is not b for a, b in zip(fresh['instances'], value.payload['instances'])))

    def test_producer_failure_publishes_owned_error_frame(self):
        bus = StreamBus()
        self.addCleanup(bus.shutdown)
        publisher = SemanticInstancePublisher(bus, Mock(side_effect=ValueError('source failure')))
        original = bus.publish_frame
        def publish(value):
            publisher._stopping = True
            return original(value)
        with patch.object(bus, 'publish_frame', side_effect=publish), self.assertLogs('services.memory.instance_stream', level='ERROR'):
            publisher._run()
        value = bus.latest_frame('memory.instances')
        self.assertIs(type(value), InstanceSnapshotFrame)
        self.assertEqual(value.status, 'error')


class EncodingChecks(unittest.TestCase):
    def test_same_published_object_is_encoded_once_and_matches_original(self):
        encode = Mock(wraps=ws._encode_ws_frame)
        owner = InstanceSnapshotEncoder(encode)
        for count in (500, 1000):
            value = frame(2**53 + count, snapshot_fixture(count=count, limit=count))
            expected = ws._encode_ws_frame(value)
            outputs = [owner.encode(value) for _ in range(4)]
            self.assertEqual(outputs, [expected] * 4)
            self.assertTrue(all(item is outputs[0] for item in outputs))
            self.assertEqual(len(json.loads(outputs[0])['frame']['payload']['instances']), count)
        self.assertEqual(encode.call_count, 2)

    def test_same_topic_source_cursor_does_not_admit_arbitrary_mutable_frame(self):
        encode = Mock(wraps=ws._encode_ws_frame)
        owner = InstanceSnapshotEncoder(encode)
        value = StreamFrame(topic='memory.instances', source='memory_service', cursor=1, payload={'name': 'first'})
        before = owner.encode(value)
        value.payload['name'] = 'second'
        after = owner.encode(value)
        self.assertNotEqual(before, after)
        self.assertEqual(encode.call_count, 2)
        self.assertFalse(owner._entries)

    def test_unpublished_frame_and_distinct_objects_are_not_shared(self):
        encode = Mock(wraps=ws._encode_ws_frame)
        owner = InstanceSnapshotEncoder(encode)
        unpublished = frame().model_copy(update={'cursor': None})
        owner.encode(unpublished); owner.encode(unpublished)
        one = frame()
        two = one.model_copy()
        owner.encode(one); owner.encode(two)
        self.assertEqual(encode.call_count, 4)

    def test_reused_object_number_requires_matching_live_reference(self):
        owner = InstanceSnapshotEncoder(ws._encode_ws_frame)
        first, second = frame(1), frame(2)
        with patch('gateways.ws.instance_encoding.id', return_value=7, create=True):
            old = owner.encode(first)
            new = owner.encode(second)
        self.assertNotEqual(old, new)
        self.assertEqual(new, ws._encode_ws_frame(second))

    def test_owner_projection_options_are_separate_and_sse_unchanged(self):
        value = frame(payload={'instances': [{'instance_id': '汉字\nα'}]})
        json_encode = Mock(wraps=ws._encode_ws_frame)
        ascii_encode = Mock(side_effect=lambda item: ws._stream_json_dumps({'type': 'stream.frame', 'frame': item}, ensure_ascii=True))
        public = InstanceSnapshotEncoder(json_encode)
        other_router = InstanceSnapshotEncoder(json_encode)
        other_options = InstanceSnapshotEncoder(ascii_encode)
        full = public.encode(value)
        self.assertEqual(other_router.encode(value), full)
        self.assertNotEqual(other_options.encode(value), full)
        self.assertEqual(json.loads(other_options.encode(value)), json.loads(full))
        self.assertEqual(json_encode.call_count, 2)
        self.assertEqual(ascii_encode.call_count, 1)
        self.assertIn('\\u6c49', ws._format_sse_payload(value))
        self.assertEqual(json_encode.call_count, 2)

    def test_entry_byte_limits_weak_ownership_and_close(self):
        encode = Mock(wraps=ws._encode_ws_frame)
        values = [frame(index) for index in range(1, 6)]
        owner = InstanceSnapshotEncoder(encode, max_entries=2, max_bytes=2048)
        for value in values:
            owner.encode(value)
            self.assertLessEqual(len(owner._entries), 2)
            self.assertLessEqual(owner._retained_bytes, 2048)
        owner.encode(values[0])
        self.assertEqual(encode.call_count, 6)
        large = frame(10, {'text': 'x' * 3000})
        self.assertEqual(owner.encode(large), ws._encode_ws_frame(large))
        self.assertNotIn(id(large), owner._entries)
        # The mock retains arguments; use an uninstrumented owner for lifetime proof.
        weak_owner = InstanceSnapshotEncoder(ws._encode_ws_frame)
        value = frame(11)
        reference = weakref.ref(value)
        weak_owner.encode(value)
        del value
        gc.collect()
        self.assertIsNone(reference())
        owner.close(); weak_owner.close()
        self.assertEqual(owner._retained_bytes, 0)
        self.assertFalse(owner._entries)
        self.assertEqual(owner.encode(values[0]), ws._encode_ws_frame(values[0]))
        self.assertFalse(owner._entries)

    def test_encoding_exception_not_cached_and_zero_budget_falls_back(self):
        encode = Mock(side_effect=RuntimeError('encode failure'))
        owner = InstanceSnapshotEncoder(encode)
        value = frame()
        for _ in range(2):
            with self.assertRaisesRegex(RuntimeError, 'encode failure'):
                owner.encode(value)
        self.assertEqual(encode.call_count, 2)
        self.assertFalse(owner._entries)
        no_cache = InstanceSnapshotEncoder(ws._encode_ws_frame, max_entries=0)
        self.assertEqual(no_cache.encode(value), ws._encode_ws_frame(value))
        self.assertFalse(no_cache._entries)


class AsgiSocket:
    def __init__(self, app, *, hold=False):
        self.incoming = asyncio.Queue()
        self.incoming.put_nowait({'type': 'websocket.connect'})
        self.messages = []
        self.replayed = asyncio.Event()
        self.entered = asyncio.Event()
        self.release = asyncio.Event()
        self.hold = hold
        self.fail_send = False
        self.scope = {'type': 'websocket', 'asgi': {'version': '3.0'}, 'scheme': 'ws',
                      'path': '/streams/ws', 'raw_path': b'/streams/ws', 'root_path': '',
                      'query_string': b'topics=memory.instances,memory.scene_spaces', 'headers': [],
                      'client': ('127.0.0.1', 43210), 'server': ('127.0.0.1', 0), 'subprotocols': []}
        self.task = asyncio.create_task(app(self.scope, self.incoming.get, self.send))

    async def send(self, message):
        if message['type'] != 'websocket.send':
            return
        text = message.get('text')
        value = json.loads(text) if text else None
        if value and value.get('type') == 'stream.frame' and self.fail_send:
            raise RuntimeError('synthetic disconnected send')
        if value and value.get('type') == 'stream.replay':
            self.replayed.set()
        if value and value.get('type') == 'stream.frame' and self.hold:
            self.entered.set()
            await self.release.wait()
        self.messages.append(value)

    def frames(self, topic='memory.instances'):
        return [message['frame'] for message in self.messages
                if message and message.get('type') == 'stream.frame' and message['frame']['topic'] == topic]

    async def disconnect(self):
        self.release.set()
        self.incoming.put_nowait({'type': 'websocket.disconnect', 'code': 1000})
        await asyncio.wait_for(self.task, 2)


class RoutingChecks(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.bus = StreamBus()
        self.runtime = NS(stream_bus=self.bus, config=NS(gateway=NS(ws_ping_interval_sec=1)))
        self.app = FastAPI()
        self.app.include_router(ws.build_data_stream_router(self.runtime, None))
        self.owners = []
        original = InstanceSnapshotEncoder
        def owner(*args, **kwargs):
            value = original(*args, **kwargs)
            self.owners.append(value)
            return value
        self.owner_patch = patch.object(ws, 'InstanceSnapshotEncoder', side_effect=owner)
        self.encode = Mock(wraps=ws._encode_ws_frame)
        self.encode_patch = patch.object(ws, '_encode_ws_frame', self.encode)
        self.owner_patch.start(); self.encode_patch.start()
        self.life = self.app.router.lifespan_context(self.app)
        await self.life.__aenter__()
        self.sockets = []

    async def asyncTearDown(self):
        for socket in self.sockets:
            socket.release.set()
            if not socket.task.done():
                socket.task.cancel()
        await asyncio.gather(*(socket.task for socket in self.sockets), return_exceptions=True)
        await self.life.__aexit__(None, None, None)
        self.owner_patch.stop(); self.encode_patch.stop()
        self.assertEqual(self.bus.status()['subscription_count'], 0)
        self.assertTrue(self.bus.shutdown())
        self.assertTrue(all(not owner._entries and owner._retained_bytes == 0 for owner in self.owners))

    async def connect(self, hold=False):
        value = AsgiSocket(self.app, hold=hold)
        self.sockets.append(value)
        await asyncio.wait_for(value.replayed.wait(), 2)
        return value

    async def until(self, condition):
        async def wait():
            while not condition():
                await asyncio.sleep(.001)
        await asyncio.wait_for(wait(), 2)

    async def test_four_routes_share_real_published_frame_and_replay(self):
        sockets = [await self.connect() for _ in range(4)]
        value = self.bus.publish_frame(frame().model_copy(update={'cursor': None}))
        await self.until(lambda: all(len(socket.frames()) == 1 for socket in sockets))
        self.assertEqual(self.encode.call_count, 1)
        replay = await self.connect()
        self.assertEqual(len(replay.frames()), 1)
        self.assertEqual(self.encode.call_count, 1)
        expected = json.loads(ws._stream_json_dumps({'type': 'stream.frame', 'frame': value}))['frame']
        self.assertTrue(all(socket.frames()[0] == expected for socket in self.sockets))
        for socket in self.sockets:
            await socket.disconnect()

    async def test_slow_send_does_not_block_fast_connections_and_latest_replaces(self):
        slow = await self.connect(hold=True)
        fast = [await self.connect() for _ in range(3)]
        self.bus.publish_frame(frame(1).model_copy(update={'cursor': None}))
        await asyncio.wait_for(slow.entered.wait(), 2)
        await self.until(lambda: all(len(socket.frames()) == 1 for socket in fast))
        for sequence in range(2, 7):
            self.bus.publish_frame(frame(sequence).model_copy(update={'cursor': None}))
            await self.until(lambda: all(len(socket.frames()) == sequence for socket in fast))
        slow.release.set()
        await self.until(lambda: len(slow.frames()) == 2)
        self.assertEqual([value['payload']['instances'][0]['instance_id'] for value in slow.frames()], ['1', '6'])
        self.assertEqual(self.encode.call_count, 6)
        for socket in self.sockets:
            self.assertFalse(any(message.get('type') == 'stream.gap' for message in socket.messages))

    async def test_route_cancellation_during_send_leaves_other_reader_and_cache_alive(self):
        slow = await self.connect(hold=True)
        fast = await self.connect()
        self.bus.publish_frame(frame(1).model_copy(update={'cursor': None}))
        await asyncio.wait_for(slow.entered.wait(), 2)
        await self.until(lambda: len(fast.frames()) == 1)
        slow.task.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await slow.task
        self.assertEqual(self.bus.status()['subscription_count'], 1)
        self.bus.publish_frame(frame(2).model_copy(update={'cursor': None}))
        await self.until(lambda: len(fast.frames()) == 2)
        self.assertEqual(self.encode.call_count, 2)
        await fast.disconnect()

    async def test_send_failure_releases_subscription_without_poisoning_shared_result(self):
        broken = await self.connect()
        broken.fail_send = True
        fast = await self.connect()
        self.bus.publish_frame(frame(1).model_copy(update={'cursor': None}))
        with self.assertRaisesRegex(RuntimeError, 'synthetic disconnected send'):
            await asyncio.wait_for(broken.task, 2)
        await self.until(lambda: len(fast.frames()) == 1)
        self.assertEqual(self.bus.status()['subscription_count'], 1)
        self.assertEqual(self.encode.call_count, 1)
        self.bus.publish_frame(frame(2).model_copy(update={'cursor': None}))
        await self.until(lambda: len(fast.frames()) == 2)
        self.assertEqual(self.encode.call_count, 2)

    async def test_non_instance_fifo_gap_and_binary_route_keep_existing_behavior(self):
        queue = ws._ClientStreamQueue(2)
        for cursor in range(1, 4):
            queue.put(StreamFrame(topic='memory.scene_spaces', source='fixture', cursor=cursor), self.runtime)
        self.assertEqual([queue.get_nowait().cursor, queue.get_nowait().cursor], [2, 3])
        self.assertEqual(ws._take_queue_gap_notice(queue, self.runtime)['dropped_count'], 1)
        queue.close()
        class Socket:
            async def send_bytes(self, value): self.value = value
            async def send_text(self, value): self.value = value
        socket = Socket()
        binary = StreamFrame(topic='other.binary', source='fixture', payload_kind=StreamPayloadKind.BINARY, payload=b'raw')
        with patch.object(self.owners[0], 'encode', side_effect=AssertionError('binary entered instance cache')):
            await ws._send_ws_frame(socket, binary, instance_encoder=self.owners[0])
        self.assertEqual(socket.value, b'raw')


if __name__ == '__main__':
    unittest.main(verbosity=2)
