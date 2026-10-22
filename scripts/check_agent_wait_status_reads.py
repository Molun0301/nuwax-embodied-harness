#!/usr/bin/env python3
"""真实任务源上的等待状态读取回归：忽略进度不重投影，返回仍用完整回执。"""
from __future__ import annotations

import asyncio
import sqlite3
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import anyio.to_thread
from starlette.concurrency import run_in_threadpool

import gateways.agent.service as service_module
from core.errors import GatewayError
from core.event_bus import EventBus
from core.task_manager import TaskManager, TaskReceiptCapture
from gateways.agent.service import AgentToolService
from gateways.runtime import GatewayRuntime
from tests.conftest import create_isolated_gateway


class WaitStatusChecks(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="neh-wait-status-")
        self.bus = EventBus()
        self.manager = TaskManager(event_bus=self.bus)
        self.release = threading.Event()
        self.entered = threading.Event()
        self.gates = []
        self.actions = []
        def action(context):
            self.actions.append(1)
            self.entered.set()
            assert self.release.wait(10), "offline action release guard"
            return {"known": True}
        self.task_id = self.manager.submit("offline_task", action).task_id
        self.assertTrue(self.entered.wait(2))
        self.stage = self.manager.get_task_status(self.task_id).stage
        for index in range(128):
            self.manager.publish_task_event(self.task_id, "task.audit", payload={"index": index})
        runtime = create_isolated_gateway(Path(self.temp.name) / "runtime", task_manager=self.manager, event_bus=self.bus)
        self.runtime = runtime
        runtime.capability_registry = NS(get_descriptor=lambda name: NS(display_name="offline"))
        self.allowed = True
        self.authorization_calls = 0
        def authorize(descriptor):
            self.assertFalse(self.manager._lock._is_owned())
            self.authorization_calls += 1
            if not self.allowed:
                raise GatewayError("not exposed", error_code="access_denied", http_status=403)
        self.service = AgentToolService(runtime, NS(authorize_capability=authorize))
        self.snapshots = self.status_reads = self.copies = self.finishes = 0
        self.reading = 0
        self.before_snapshot = self.after_status = None
        self.lock = threading.Lock()
        self.background = []
        original_snapshot, original_status = self.service._task_snapshot, self.service._wait_status
        original_copy, original_finish = TaskReceiptCapture.snapshot, self.service._finish_task_wait
        def snapshot(task_id):
            with self.lock:
                self.snapshots += 1
                number = self.snapshots
                self.reading += 1
            try:
                if self.before_snapshot:
                    self.before_snapshot(number)
                return original_snapshot(task_id)
            finally:
                with self.lock:
                    self.reading -= 1
        def status(task_id):
            with self.lock:
                self.status_reads += 1
                number = self.status_reads
                self.reading += 1
            try:
                value = original_status(task_id)
                if self.after_status:
                    self.after_status(number, value)
                return value
            finally:
                with self.lock:
                    self.reading -= 1
        def copy(capture):
            if threading.current_thread().name != "Agent-terminal-receipts":
                with self.lock:
                    self.copies += 1
            return original_copy(capture)
        def finish(snapshot, timed_out):
            self.finishes += 1
            return original_finish(snapshot, timed_out)
        self.patches = (patch.object(self.service, "_task_snapshot", snapshot),
                        patch.object(self.service, "_wait_status", status),
                        patch.object(TaskReceiptCapture, "snapshot", copy),
                        patch.object(self.service, "_finish_task_wait", finish))
        for item in self.patches:
            item.start()

    async def asyncTearDown(self):
        for gate in self.gates:
            gate.set()
        for waiter in self.background:
            if not waiter.done():
                waiter.cancel()
        await asyncio.gather(*self.background, return_exceptions=True)
        self.release.set()
        await run_in_threadpool(self.manager.wait_for_task, self.task_id, timeout=2)
        self.manager.shutdown(wait=True)
        closed = await run_in_threadpool(self.service.close, 2, source_fenced=True)
        self.assertTrue(closed["owner_closed"], closed)
        self.assertTrue(self.bus.shutdown(2))
        for item in reversed(self.patches):
            item.stop()
        self.assertEqual(self.actions, [1])
        self.assertEqual(self.service._async_waiters, {})
        await run_in_threadpool(self.runtime.stop)
        self.temp.cleanup()

    def wait(self, *, sync=False, timeout=30):
        operation = run_in_threadpool(self.service.wait_task, self.task_id, timeout) if sync else self.service.wait_task_async(self.task_id, timeout)
        waiter = asyncio.create_task(operation)
        self.background.append(waiter)
        return waiter

    async def eventually(self, predicate):
        deadline = time.monotonic() + 3
        while not predicate():
            self.assertLess(time.monotonic(), deadline, (self.snapshots, self.status_reads, self.copies))
            await asyncio.sleep(0.001)

    async def idle(self, reads, *, sync=False, waiters=1):
        await self.eventually(lambda: self.status_reads >= reads and self.reading == 0
                              and anyio.to_thread.current_default_thread_limiter().borrowed_tokens == (1 if sync else 0)
                              and (sync or len(self.service._async_waiters) == waiters))
        await asyncio.sleep(0)

    async def check_ignored_progress(self, sync, waiters):
        waiting = [self.wait(sync=sync) for _ in range(waiters)]
        await self.idle(waiters, sync=sync, waiters=waiters)
        self.assertEqual((self.snapshots, self.copies), (waiters, waiters))
        authorizations = self.authorization_calls
        for wake in range(4):
            self.manager.update_task(self.task_id, progress=(wake + 1) / 10, stage=self.stage)
            await self.idle((wake + 2) * waiters, sync=sync, waiters=waiters)
            self.assertEqual((self.snapshots, self.copies), (waiters, waiters))
            self.assertFalse(any(item.done() for item in waiting))
        self.assertGreaterEqual(self.authorization_calls - authorizations, 4 * waiters)
        self.manager.update_task(self.task_id, stage="finished_wait")
        replies = await asyncio.gather(*waiting)
        self.assertEqual((self.snapshots, self.copies, self.finishes), (2 * waiters, 2 * waiters, waiters))
        expected = self.service._finish_task_wait(self.service._task_snapshot(self.task_id), False)
        self.assertTrue(all(reply == expected for reply in replies))
        self.assertTrue(all(not reply["wait_timed_out"] and reply["status"]["state"] == "running" for reply in replies))

    async def test_async_progress_work_is_status_only_per_waiter(self):
        await self.check_ignored_progress(False, 3)

    async def test_sync_progress_work_is_status_only(self):
        await self.check_ignored_progress(True, 1)

    async def test_change_between_baseline_and_subscription_is_rechecked(self):
        original = self.service._task_snapshot
        def after_baseline(task_id):
            value = original(task_id)
            if self.snapshots == 1:
                self.manager.update_task(task_id, stage="between_baseline_and_subscribe")
            return value
        with patch.object(self.service, "_task_snapshot", after_baseline):
            reply = await asyncio.wait_for(self.wait(), 2)
        self.assertEqual(reply["status"]["stage"], "between_baseline_and_subscribe")
        self.assertFalse(reply["wait_timed_out"])
        self.assertEqual(self.snapshots, 2)

    async def test_event_during_status_read_is_not_lost(self):
        def after_status(number, value):
            if number == 1:
                self.manager.update_task(self.task_id, stage="during_status_read")
        self.after_status = after_status
        reply = await asyncio.wait_for(self.wait(), 2)
        self.assertEqual(reply["status"]["stage"], "during_status_read")
        self.assertFalse(reply["wait_timed_out"])
        self.assertEqual(self.snapshots, 2)

    async def test_cancellation_during_cheap_read_cleans_subscription_without_final_save(self):
        waiter = self.wait()
        await self.idle(1)
        entered, release = threading.Event(), threading.Event()
        self.gates.append(release)
        def after_status(number, value):
            if number == 2:
                entered.set()
                assert release.wait(3)
        self.after_status = after_status
        self.manager.update_task(self.task_id, progress=0.3)
        await self.eventually(entered.is_set)
        waiter.cancel()
        release.set()
        with self.assertRaises(asyncio.CancelledError):
            await waiter
        self.assertEqual(self.finishes, 0)
        self.assertEqual(self.snapshots, 1)
        self.assertEqual(self.service._async_waiters, {})

    async def test_final_store_failure_still_surfaces_without_changing_task_outcome(self):
        waiter = self.wait()
        await self.idle(1)
        with patch.object(self.service.store, "save_terminal", side_effect=sqlite3.OperationalError("offline persistence error")):
            self.release.set()
            with self.assertRaises(sqlite3.OperationalError):
                await waiter
        self.assertEqual(self.manager.get_task_status(self.task_id).state, "succeeded")
        self.assertEqual(self.finishes, 1)

    async def test_zero_timeout_returns_fresh_full_running_snapshot(self):
        def after_status(number, value):
            self.manager.update_task(self.task_id, progress=0.75, stage=self.stage, message="fresh timeout snapshot")
        self.after_status = after_status
        reply = await self.wait(timeout=0)
        self.assertTrue(reply["wait_timed_out"])
        self.assertEqual(reply["status"]["state"], "running")
        self.assertEqual(reply["status"]["progress"], 0.75)
        self.assertEqual(reply["status"]["message"], "fresh timeout snapshot")
        self.assertEqual((self.snapshots, self.copies, self.finishes), (2, 2, 1))

    async def test_terminal_in_final_read_wins_over_elapsed_deadline(self):
        def after_status(number, value):
            self.release.set()
            self.manager.wait_for_task(self.task_id, timeout=2)
        self.after_status = after_status
        reply = await self.wait(timeout=0)
        self.assertFalse(reply["wait_timed_out"])
        self.assertEqual(reply["status"]["state"], "succeeded")
        saved = self.service.get_task_record(self.task_id)["snapshot"]
        self.assertEqual(saved["result"], {"known": True})
        self.assertEqual(self.finishes, 1)

    async def test_stage_reversion_during_final_read_does_not_return_false_change(self):
        waiter = self.wait()
        await self.idle(1)
        def before_snapshot(number):
            if number == 2:
                self.manager.update_task(self.task_id, stage=self.stage)
        self.before_snapshot = before_snapshot
        self.manager.update_task(self.task_id, stage="transient")
        await self.idle(3)
        self.assertFalse(waiter.done())
        self.assertEqual(self.snapshots, 2)
        self.manager.update_task(self.task_id, stage="actual_change")
        reply = await waiter
        self.assertEqual(reply["status"]["stage"], "actual_change")
        self.assertFalse(reply["wait_timed_out"])
        self.assertEqual(self.snapshots, 3)

    async def test_publicly_equal_redacted_stage_is_not_false_change_or_busy_loop(self):
        self.manager.update_task(self.task_id, stage="password=first-secret")
        waiter = self.wait()
        await self.idle(1)
        self.manager.update_task(self.task_id, stage="password=second-secret")
        await self.idle(2)
        self.assertFalse(waiter.done())
        self.assertEqual(self.snapshots, 1)
        self.manager.update_task(self.task_id, stage="public_change")
        reply = await waiter
        self.assertEqual(reply["status"]["stage"], "public_change")
        self.assertEqual(self.snapshots, 2)

    async def test_permission_revocation_precedes_metadata_update(self):
        waiter = self.wait()
        await self.idle(1)
        self.allowed = False
        self.manager.update_task_metadata(self.task_id, {"note": "updated"})
        with self.assertRaises(GatewayError) as caught:
            await waiter
        self.assertEqual(caught.exception.error_code, "access_denied")
        self.assertEqual(self.snapshots, 1)
        self.assertEqual(self.copies, 1)


    async def test_new_projection_only_error_defers_until_full_timeout_response(self):
        now = [0.0]
        with patch.object(service_module, "time", NS(monotonic=lambda: now[0])):
            waiter = self.wait(timeout=30)
            await self.idle(1)
            cyclic = []
            cyclic.append(cyclic)
            self.manager.publish_task_event(self.task_id, "task.audit", payload={"cycle": cyclic})
            await self.idle(2)
            self.assertFalse(waiter.done())
            self.assertEqual((self.snapshots, self.copies), (1, 1))
            now[0] = 31.0
            self.manager.publish_task_event(self.task_id, "task.audit", payload={"deadline_wake": True})
            with self.assertRaises(GatewayError) as caught:
                await waiter
        self.assertEqual(caught.exception.error_code, "mcp_result_unavailable")
        self.assertEqual((self.snapshots, self.copies, self.finishes), (2, 2, 0))
        self.assertEqual(self.manager.get_task_status(self.task_id).state, "running")

    async def test_existing_projection_error_still_fails_initial_baseline(self):
        cycle = []
        cycle.append(cycle)
        self.manager.publish_task_event(self.task_id, "task.audit", payload={"cycle": cycle})
        with self.assertRaises(GatewayError) as caught:
            await self.wait()
        self.assertEqual(caught.exception.error_code, "mcp_result_unavailable")
        self.assertEqual((self.snapshots, self.status_reads, self.finishes), (1, 0, 0))

    async def test_close_during_final_full_read_preserves_closed_error(self):
        waiter = self.wait()
        await self.idle(1)
        entered, release = threading.Event(), threading.Event()
        self.gates.append(release)
        def before_snapshot(number):
            if number == 2:
                entered.set()
                assert release.wait(3)
        self.before_snapshot = before_snapshot
        self.manager.update_task(self.task_id, stage="change")
        await self.eventually(entered.is_set)
        self.service.close(0)
        release.set()
        with self.assertRaises(GatewayError) as caught:
            await waiter
        self.assertEqual(caught.exception.error_code, "agent_service_closed")
        self.assertEqual(self.finishes, 0)

    async def test_source_lookup_failure_preserves_not_found_without_history_read(self):
        waiter = self.wait()
        await self.idle(1)
        with patch.object(self.manager, "task_receipt_header", side_effect=KeyError(self.task_id)):
            self.manager.update_task(self.task_id, progress=0.4)
            with self.assertRaises(GatewayError) as caught:
                await waiter
        self.assertEqual(caught.exception.error_code, "not_found")
        self.assertEqual(caught.exception.details["status_source"], "live_task")
        self.assertEqual(self.snapshots, 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
