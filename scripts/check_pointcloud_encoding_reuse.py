#!/usr/bin/env python3
"""离线验证规范点云多客户端复用、完整身份、预算及取消/关闭；不连接设备。"""
from __future__ import annotations

import asyncio
import json
import struct
import sys
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
from fastapi import FastAPI
from fastapi.testclient import TestClient

from contracts.streams import StreamFrame, StreamPayloadKind
from core.stream_bus import StreamBus
from gateways.ws import data_streams as ws
from gateways.ws import pointcloud_encoding as encoding
from gateways.ws.pointcloud_encoding import CanonicalPointCloudEncoder, _capture


def cloud(*, count=1000, raw=None, cursor=123, **metadata):
    if raw is None:
        raw = np.arange(count * 3, dtype='<f4').tobytes()
    values = {'source': 'live', 'source_epoch': 'source-a', 'stream_generation': 1,
                  'revision': 'cloud:7', 'revision_ordinal': 7, 'frame_id': 'map',
                  'source_timestamp_ns': 123456789, 'point_count': len(raw) // 12, 'format': 'float32-xyz-le'}
    values.update(metadata)
    return StreamFrame(topic='pointcloud_map.snapshot', source='probe', cursor=cursor,
                       payload_kind=StreamPayloadKind.BINARY, payload=raw,
                       revision='fallback:7', frame_id='fallback_map', metadata=values)


class Socket:
    def __init__(self):
        self.payload = None

    async def send_bytes(self, value):
        self.payload = value

    async def send_text(self, value):
        self.payload = value


class EncodingContracts(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.owners = []
        self.release = []

    async def asyncTearDown(self):
        for event in self.release:
            event.set()
        for owner in self.owners:
            await owner.close()

    def owner(self, encode=None, **kwargs):
        value = CanonicalPointCloudEncoder(encode or ws._encode_ws_frame, ws._run_blocking, **kwargs)
        self.owners.append(value)
        return value

    async def equals_baseline(self, frame, owner=None):
        owner = owner or self.owner()
        try:
            expected = ws._encode_ws_frame(frame)
        except Exception as expected_error:  # noqa: BLE001 - 比较原编码器公开的完整异常合同。
            with self.assertRaises(type(expected_error)) as caught:
                await owner.encode(frame)
            self.assertEqual(str(caught.exception), str(expected_error))
        else:
            self.assertEqual(await owner.encode(frame), expected)
            self.assertEqual(await owner.encode(frame), expected)

    async def test_golden_live_saved_empty_unicode_and_u64(self):
        for metadata in ({}, {'source': 'saved'}, {'source_epoch': '地图:α', 'revision': '修订:1'},
                         {'stream_generation': 2**64-1, 'revision_ordinal': str(2**64-1),
                          'source_timestamp_ns': 2**53+1}):
            await self.equals_baseline(cloud(cursor=2**64-1, **metadata))
        await self.equals_baseline(cloud(count=0))

    async def test_golden_invalid_inputs_preserve_error_order(self):
        variants = [cloud(raw=b'bad'), cloud(raw=struct.pack('<fff', float('nan'), 0, 1)),
                    cloud(raw=struct.pack('<fff', 0, float('inf'), 1))]
        for field, value in [('source', 'sensor'), ('source_epoch', ''), ('point_count', True),
                             ('point_count', '01'), ('point_count', -1), ('point_count', 2**64),
                             ('format', 'f64'), ('stream_generation', False), ('stream_generation', -1),
                             ('revision_ordinal', '00'), ('source_timestamp_ns', 2**64),
                             ('source_epoch', 'x' * 65537), ('source_epoch', '\ud800')]:
            variants.append(cloud(**{field: value}))
        invalid_both = cloud(raw=struct.pack('<fff', float('nan'), 0, 1), stream_generation=-1)
        variants.append(invalid_both)
        for value in variants:
            with self.subTest(metadata=value.metadata):
                await self.equals_baseline(value)

    async def test_whole_scan_and_output_shared_for_eight_clients(self):
        owner = self.owner()
        frame = cloud()
        original = np.isfinite
        sockets = [Socket() for _ in range(8)]
        with patch.object(ws.np, 'isfinite', wraps=original) as scan:
            await asyncio.gather(*(ws._send_ws_frame(socket, frame, pointcloud_encoder=owner)
                                   for socket in sockets))
            self.assertEqual(scan.call_count, 1)
        self.assertEqual(len({id(socket.payload) for socket in sockets}), 1)
        self.assertEqual(sockets[0].payload, ws._encode_ws_frame(frame))

    async def test_all_wire_identities_and_raw_object_separate(self):
        owner = self.owner()
        frame = cloud()
        first = await owner.encode(frame)
        variants = [frame.model_copy(update={'cursor': 124})]
        for field, value in [('source', 'saved'), ('source_epoch', 'source-b'), ('stream_generation', 2),
                             ('revision', 'cloud:8'), ('revision_ordinal', 8), ('frame_id', 'another_map'),
                             ('source_timestamp_ns', 123456790)]:
            variants.append(frame.model_copy(update={'metadata': dict(frame.metadata, **{field: value})}))
        for variant in variants:
            encoded = await owner.encode(variant)
            self.assertNotEqual(encoded, first)
            self.assertEqual(encoded, ws._encode_ws_frame(variant))
        distinct = frame.model_copy(update={'payload': memoryview(frame.payload).tobytes()})
        other = await owner.encode(distinct)
        self.assertEqual(other, first)
        self.assertIsNot(other, first)
        newer = frame.model_copy(update={'payload': struct.pack('<fff', 9, 8, 7),
                                        'metadata': dict(frame.metadata, point_count=1)})
        await self.equals_baseline(newer, owner)

    async def test_mutable_frame_metadata_and_payload_rechecked(self):
        owner = self.owner()
        frame = cloud()
        first = await owner.encode(frame)
        frame.metadata['revision'] = 'cloud:changed'
        self.assertNotEqual(await owner.encode(frame), first)
        frame.metadata['format'] = 'wrong-format'
        await self.equals_baseline(frame, owner)
        frame.metadata['format'] = 'float32-xyz-le'
        frame.payload = bytearray(frame.payload)
        original = np.isfinite
        with patch.object(ws.np, 'isfinite', wraps=original) as scan:
            before = await owner.encode(frame)
            frame.payload[:4] = struct.pack('<f', 987.0)
            after = await owner.encode(frame)
            self.assertEqual(scan.call_count, 2)
        self.assertNotEqual(before, after)

    async def test_fallback_frame_id_and_revision_are_in_identity(self):
        owner = self.owner()
        frame = cloud(frame_id=None, revision=None)
        first = await owner.encode(frame)
        frame.frame_id = 'new_map'
        self.assertNotEqual(await owner.encode(frame), first)
        first = await owner.encode(frame)
        frame.revision = 'new_revision'
        self.assertNotEqual(await owner.encode(frame), first)

    async def test_original_mutations_during_encode_cannot_poison_key(self):
        entered = asyncio.Event()
        release = threading.Event()
        self.release.append(release)
        loop = asyncio.get_running_loop()
        original = cloud()
        baseline = ws._encode_ws_frame(original)
        captured = []

        def gated(frame):
            captured.append(frame)
            loop.call_soon_threadsafe(entered.set)
            if not release.wait(2):
                raise AssertionError('encoding gate timed out')
            return ws._encode_ws_frame(frame)

        owner = self.owner(gated)
        task = asyncio.create_task(owner.encode(original))
        await asyncio.wait_for(entered.wait(), 2)
        original.cursor = 555
        original.payload = struct.pack('<fff', 1, 2, 3)
        original.metadata.update(point_count=1, source_epoch='replaced')
        release.set()
        self.assertEqual(await task, baseline)
        self.assertEqual(await owner.encode(original), ws._encode_ws_frame(original))
        self.assertIsNot(captured[0].metadata, original.metadata)

    async def test_unrelated_diagnostics_not_retained_in_frozen_frame(self):
        seen = []
        def encode(frame):
            seen.append(frame)
            return ws._encode_ws_frame(frame)
        frame = cloud()
        frame.metadata['provenance'] = {'large': bytearray(100000)}
        frame.source = 'x' * 100000
        owner = self.owner(encode)
        await owner.encode(frame)
        self.assertNotIn('provenance', seen[0].metadata)
        self.assertLess(len(seen[0].source), 100)

    async def test_budgets_include_raw_and_output_and_evict_completed(self):
        frame = cloud()
        charge = _capture(frame)[-1]
        owner = self.owner(max_bytes=charge * 2, max_entries=2)
        for cursor in range(1, 20):
            await owner.encode(frame.model_copy(update={'cursor': cursor}))
            self.assertLessEqual(owner._charged_bytes, charge * 2)
            self.assertLessEqual(len(owner._entries), 2)
        for entry in owner._entries.values():
            self.assertGreaterEqual(entry.charge, len(entry.raw) + len(entry.result.result()))
        self.assertEqual(owner._charged_bytes, sum(entry.charge for entry in owner._entries.values()))

    async def test_oversize_and_disabled_cache_fall_back(self):
        frame = cloud()
        for kwargs in ({'max_bytes': 1}, {'max_entries': 0}):
            owner = self.owner(**kwargs)
            original = np.isfinite
            with patch.object(ws.np, 'isfinite', wraps=original) as scan:
                values = await asyncio.gather(owner.encode(frame), owner.encode(frame))
                self.assertEqual(scan.call_count, 2)
            self.assertEqual(values[0], values[1])
            self.assertIsNot(values[0], values[1])
            self.assertEqual(owner._charged_bytes, 0)
            self.assertFalse(owner._entries)

    async def test_errors_not_retained_and_exception_waiters_cleanup(self):
        owner = self.owner()
        invalid = cloud(format='invalid')
        self.assertEqual(json.loads(await owner.encode(invalid))['type'], 'stream.binary_rejected')
        self.assertFalse(owner._entries)
        self.assertEqual(owner._charged_bytes, 0)
        def broken(_frame):
            raise RuntimeError('encoder failure')
        broken_owner = self.owner(broken)
        results = await asyncio.gather(*(broken_owner.encode(cloud()) for _ in range(3)), return_exceptions=True)
        self.assertTrue(all(isinstance(result, RuntimeError) for result in results))
        self.assertFalse(broken_owner._entries)
        self.assertEqual(broken_owner._charged_bytes, 0)

    async def test_legacy_sse_and_nonbinary_paths_unchanged(self):
        owner = self.owner()
        frame = cloud()
        legacy = frame.model_copy(update={'topic': 'pointcloud_map.live_chunk',
                                         'payload': struct.pack('<II', 7, 1000) + frame.payload})
        await self.equals_baseline(legacy, owner)
        nonbinary = frame.model_copy(update={'payload_kind': StreamPayloadKind.JSON, 'payload': {'x': 1}})
        await self.equals_baseline(nonbinary, owner)
        sse_before = ws._format_sse_payload(frame)
        await owner.encode(frame)
        self.assertEqual(ws._format_sse_payload(frame), sse_before)

    async def test_inflight_budget_cancellation_independence_and_close(self):
        loop = asyncio.get_running_loop()
        entered = asyncio.Event()
        release = threading.Event()
        self.release.append(release)
        calls = []
        main_thread = threading.get_ident()
        frame = cloud()

        def gated(value):
            self.assertNotEqual(threading.get_ident(), main_thread)
            calls.append(value.cursor)
            if value.cursor == frame.cursor:
                loop.call_soon_threadsafe(entered.set)
                if not release.wait(2):
                    raise AssertionError('encoding gate timed out')
            return ws._encode_ws_frame(value)

        owner = self.owner(gated, max_bytes=_capture(frame)[-1], max_entries=1)
        cancelled = asyncio.create_task(owner.encode(frame))
        survivor = asyncio.create_task(owner.encode(frame))
        await asyncio.wait_for(entered.wait(), 2)
        self.assertEqual(len(owner._entries), 1)
        self.assertEqual(owner._charged_bytes, _capture(frame)[-1])
        cancelled.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await cancelled
        # 饱和时不同 key 异步等待，不提交到共享池；小位姿仍可立即发送。
        other = frame.model_copy(update={'cursor': 999})
        waiting = asyncio.create_task(owner.encode(other))
        await asyncio.sleep(0)
        self.assertFalse(waiting.done())
        pose_socket = Socket()
        pose = StreamFrame(topic='localization.pose', source='probe', cursor=999, payload={'x': 1})
        await asyncio.wait_for(ws._send_ws_frame(pose_socket, pose, pointcloud_encoder=owner), 1)
        self.assertIsInstance(pose_socket.payload, str)
        self.assertEqual(len(owner._entries), 1)
        closing = asyncio.create_task(owner.close())
        await asyncio.sleep(0)
        self.assertFalse(closing.done())
        self.assertTrue(owner._closed)
        closing.cancel()
        release.set()
        self.assertEqual(await survivor, ws._encode_ws_frame(frame))
        with self.assertRaises(asyncio.CancelledError):
            await closing
        self.assertEqual(calls.count(frame.cursor), 1)
        self.assertEqual(owner._charged_bytes, 0)
        self.assertFalse(owner._entries)
        with self.assertRaisesRegex(RuntimeError, '已关闭'):
            await waiting
        with self.assertRaisesRegex(RuntimeError, '已关闭'):
            await owner.encode(other)
        self.assertFalse(owner._entries)

    async def test_all_waiters_cancelled_work_is_reaped_on_close(self):
        loop = asyncio.get_running_loop()
        entered = asyncio.Event()
        release = threading.Event()
        self.release.append(release)
        def gated(frame):
            loop.call_soon_threadsafe(entered.set)
            if not release.wait(2):
                raise AssertionError('encoding gate timed out')
            return ws._encode_ws_frame(frame)
        owner = self.owner(gated)
        pending = asyncio.create_task(owner.encode(cloud()))
        await asyncio.wait_for(entered.wait(), 2)
        pending.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await pending
        tasks = [entry.finished for entry in owner._entries.values()]
        release.set()
        await owner.close()
        self.assertTrue(all(task.done() and not task.cancelled() for task in tasks))
        self.assertFalse(owner._entries)

    async def test_queued_cancellation_holds_budget_and_bounds_submissions(self):
        loop = asyncio.get_running_loop()
        entered = asyncio.Event()
        release = threading.Event()
        self.release.append(release)
        calls = []
        first = cloud()

        def gated(frame):
            calls.append(frame.cursor)
            if frame.cursor == first.cursor:
                loop.call_soon_threadsafe(entered.set)
                if not release.wait(2):
                    raise AssertionError('encoding gate timed out')
            return ws._encode_ws_frame(frame)

        owner = self.owner(gated, max_bytes=_capture(first)[-1] * 2 + 32, max_entries=2)
        running = asyncio.create_task(owner.encode(first))
        await asyncio.wait_for(entered.wait(), 2)
        queued = asyncio.create_task(owner.encode(first.model_copy(update={'cursor': 124})))
        await asyncio.sleep(0)
        self.assertEqual(len(owner._entries), 2)
        queued_entry = next(entry for key, entry in owner._entries.items() if key[-1][1] == 124)
        charged = owner._charged_bytes
        queued.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await queued
        self.assertTrue(queued_entry.work.cancelled())
        self.assertIsNone(queued_entry.job.frame)
        self.assertFalse(queued_entry.finished.done())
        self.assertEqual(owner._charged_bytes, charged)
        waiting = [asyncio.create_task(owner.encode(first.model_copy(update={'cursor': 1000 + index})))
                   for index in range(12)]
        await asyncio.sleep(0)
        self.assertTrue(all(not task.done() for task in waiting))
        self.assertEqual(len(owner._entries), 2)
        self.assertEqual(owner._executor._work_queue.qsize(), 1)
        # 单独的短状态工作不等待点云工作线程。
        self.assertEqual(await asyncio.wait_for(ws._run_blocking(lambda: 'status-ready'), 1), 'status-ready')
        closing = asyncio.create_task(owner.close())
        await asyncio.sleep(0)
        waiting_results = await asyncio.gather(*waiting, return_exceptions=True)
        self.assertTrue(all(isinstance(result, RuntimeError) for result in waiting_results))
        release.set()
        await running
        await closing
        self.assertEqual(calls, [123])
        self.assertEqual(owner._charged_bytes, 0)
        self.assertTrue(queued_entry.finished.done())

    async def test_new_generation_waits_async_then_gets_its_own_packet(self):
        loop = asyncio.get_running_loop()
        entered = asyncio.Event()
        release = threading.Event()
        self.release.append(release)
        first = cloud()
        second = first.model_copy(update={'metadata': dict(first.metadata, source_epoch='new-owner',
                                                           stream_generation=2)})
        calls = []

        def gated(frame):
            calls.append(frame.metadata['stream_generation'])
            if frame.metadata['stream_generation'] == 1:
                loop.call_soon_threadsafe(entered.set)
                if not release.wait(2):
                    raise AssertionError('encoding gate timed out')
            return ws._encode_ws_frame(frame)

        owner = self.owner(gated, max_entries=1)
        running = asyncio.create_task(owner.encode(first))
        await asyncio.wait_for(entered.wait(), 2)
        next_generation = asyncio.create_task(owner.encode(second))
        await asyncio.sleep(0)
        self.assertFalse(next_generation.done())
        self.assertEqual(calls, [1])
        release.set()
        old, new = await asyncio.gather(running, next_generation)
        self.assertNotEqual(old, new)
        self.assertEqual(new, ws._encode_ws_frame(second))
        self.assertEqual(calls, [1, 2])

    async def test_owned_encode_and_close_ignore_default_executor_starvation(self):
        loop = asyncio.get_running_loop()
        previous = loop._default_executor
        default = ThreadPoolExecutor(max_workers=1, thread_name_prefix='blocked-default-probe')
        loop.set_default_executor(default)
        release, started = threading.Event(), asyncio.Event()
        self.release.append(release)

        def block_default():
            loop.call_soon_threadsafe(started.set)
            if not release.wait(2):
                raise AssertionError('default-executor gate timed out')

        blocked = loop.run_in_executor(None, block_default)
        try:
            await asyncio.wait_for(started.wait(), 2)
            owner = self.owner()
            value = cloud()
            self.assertEqual(await asyncio.wait_for(owner.encode(value), 1), ws._encode_ws_frame(value))
            await asyncio.wait_for(owner.close(), 1)
            self.assertFalse(blocked.done())
            self.assertEqual(owner._charged_bytes, 0)
        finally:
            release.set()
            await blocked
            loop._default_executor = previous
            default.shutdown(wait=True)

    async def test_first_worker_start_failure_retires_queued_item(self):
        calls = []

        def encode(frame):
            calls.append(frame)
            return ws._encode_ws_frame(frame)

        class FailedStartPool(ThreadPoolExecutor):
            def _adjust_thread_count(self):
                raise RuntimeError('synthetic worker start failure')

        failed = FailedStartPool(max_workers=1)
        recovered = ThreadPoolExecutor(max_workers=1)
        owner = self.owner(encode)
        value = cloud()
        with patch.object(encoding, 'ThreadPoolExecutor', side_effect=[failed, recovered]):
            with self.assertRaisesRegex(RuntimeError, 'worker start failure'):
                await owner.encode(value)
            self.assertTrue(failed._shutdown)
            self.assertIsNone(owner._executor)
            self.assertEqual(owner._charged_bytes, 0)
            self.assertFalse(owner._entries)
            queued = failed._work_queue.get_nowait()
            self.assertIsNone(queued.args[0].frame)
            self.assertTrue(queued.args[1].cancelled())
            self.assertEqual(await owner.encode(value), ws._encode_ws_frame(value))
        self.assertEqual(len(calls), 1)
        self.assertIsNotNone(calls[0])

    async def test_cross_owner_equal_identity_is_not_shared(self):
        frame = cloud()
        one, two = self.owner(), self.owner()
        first, second = await asyncio.gather(one.encode(frame), two.encode(frame))
        self.assertEqual(first, second)
        self.assertIsNot(first, second)

    async def test_foreign_loop_falls_back_and_close_preserves_owner(self):
        owner = self.owner()
        frame = cloud()
        cached = await owner.encode(frame)
        charged = owner._charged_bytes

        async def foreign():
            result = await owner.encode(frame)
            self.assertEqual(result, cached)
            self.assertIsNot(result, cached)
            with self.assertRaisesRegex(RuntimeError, '事件循环'):
                await owner.close()

        await asyncio.get_running_loop().run_in_executor(None, lambda: asyncio.run(foreign()))
        self.assertFalse(owner._closed)
        self.assertEqual(owner._charged_bytes, charged)
        self.assertIs(await owner.encode(frame), cached)


class RouterLifecycleContracts(unittest.TestCase):
    def test_real_route_lifecycle_topic_filter_and_fanout(self):
        bus = StreamBus()
        runtime = NS(stream_bus=bus, config=NS(gateway=NS(ws_ping_interval_sec=60)))
        app = FastAPI()
        owners = []
        real_owner = CanonicalPointCloudEncoder
        def tracked(*args, **kwargs):
            result = real_owner(*args, **kwargs)
            owners.append(result)
            return result
        try:
            app.include_router(ws.build_data_stream_router(runtime, None))
            with patch.object(ws, 'CanonicalPointCloudEncoder', side_effect=tracked):
                for iteration in range(2):
                    with TestClient(app) as client:
                        with ExitStack() as stack:
                            sockets = [stack.enter_context(client.websocket_connect('/streams/ws?topics=pointcloud_map.snapshot'))
                                       for _ in range(3)]
                            filtered = stack.enter_context(client.websocket_connect('/streams/ws?topics=robot.state'))
                            for socket in sockets:
                                self.assertEqual(socket.receive_json()['type'], 'stream.replay')
                            replay = filtered.receive_json()
                            if iteration:
                                self.assertEqual(replay['frame']['topic'], 'robot.state')
                                replay = filtered.receive_json()
                            self.assertEqual(replay['type'], 'stream.replay')
                            original = cloud()
                            finite = np.isfinite
                            with patch.object(ws.np, 'isfinite', wraps=finite) as scan:
                                frame = bus.publish_frame(original)
                                values = [socket.receive_bytes() for socket in sockets]
                                self.assertEqual(scan.call_count, 1)
                            self.assertTrue(all(value == ws._encode_ws_frame(frame) for value in values))
                            bus.publish('robot.state', source='probe', payload={'iteration': iteration})
                            self.assertEqual(filtered.receive_json()['frame']['topic'], 'robot.state')
                        self.assertEqual(bus.status()['subscription_count'], 0)
                    self.assertTrue(owners[-1]._closed)
                    self.assertEqual(owners[-1]._charged_bytes, 0)
                    self.assertFalse(owners[-1]._entries)
            self.assertEqual(len(owners), 2)
            self.assertIsNot(owners[0], owners[1])
        finally:
            bus.shutdown()


if __name__ == '__main__':
    unittest.main(verbosity=2)
