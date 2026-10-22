#!/usr/bin/env python3
"""真实网关、任务管理器与账本的等待隔离回归，不启动设备或模型。"""
from __future__ import annotations

import asyncio
import tempfile
import threading
import time
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import anyio.to_thread
import httpx
from fastapi import FastAPI
from fastapi.concurrency import run_in_threadpool
from contracts.capabilities import CapabilityDescriptor, CapabilityExecutionMode, CapabilityRiskLevel
from contracts.naming import build_task_id
from core.errors import GatewayError
from core.event_bus import EventBus
from core.task_manager import TaskManager, TaskTimeoutError
from gateways.agent.feedback import TERMINAL_STATES
from gateways.agent.router import build_agent_router
from gateways.agent.service import AgentToolService, _AsyncTaskWakeup
from gateways.auth import GatewayAccessManager
from tests.conftest import create_isolated_gateway


async def eventually(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError("等待业务条件超时")
        await asyncio.sleep(0.001)


def idle_count(service):
    with service._waiters_lock:
        return sum(bool(waiter.event._waiters) for waiter in service._async_waiters.values())


class AgentWaitIsolationTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="neh-real-agent-wait-")
        self.bus = EventBus()
        self.manager = TaskManager(event_bus=self.bus, max_workers=1)
        self.runtime = create_isolated_gateway(Path(self.temp.name), event_bus=self.bus, task_manager=self.manager)
        self.runtime.capability_registry.register(CapabilityDescriptor(
            name="wait_example", display_name="等待样例", description="等待离线执行器完成。",
            execution_mode=CapabilityExecutionMode.ASYNC_TASK, risk_level=CapabilityRiskLevel.LOW,
            exposed_to_agent=True), lambda *_args, **_kwargs: None)
        self.service = AgentToolService(self.runtime, GatewayAccessManager(self.runtime.config.gateway))
        self.app = FastAPI()
        self.app.include_router(build_agent_router(self.service))
        self.background, self.controls, self.snapshot_threads, self.store_threads = [], {}, [], []
        self.cancel_count = 0
        self.before_snapshot = self.after_snapshot = None
        self.snapshot_count = 0
        original_snapshot = self.runtime.get_task_snapshot
        original_cancel = self.runtime._handle_cancel_task
        original_save = self.service.store.save_terminal
        def snapshot(task_id, **kwargs):
            self.snapshot_count += 1
            self.snapshot_threads.append(threading.get_ident())
            if self.before_snapshot:
                self.before_snapshot(self.snapshot_count)
            result = original_snapshot(task_id, **kwargs)
            if self.after_snapshot:
                self.after_snapshot(self.snapshot_count)
            return result
        def cancel(*args, **kwargs):
            self.cancel_count += 1
            return original_cancel(*args, **kwargs)
        def save(*args, **kwargs):
            self.store_threads.append(threading.get_ident())
            return original_save(*args, **kwargs)
        self.runtime.get_task_snapshot = snapshot
        self.runtime._handle_cancel_task = cancel
        self.service.store.save_terminal = save
        self.task_id = self.add_task()
        await eventually(lambda: self.manager.get_task_status(self.task_id).state.value == "running")
        self.base_subscriptions = set(self.bus._subscriptions)
        self.limiter = anyio.to_thread.current_default_thread_limiter()
        self.old_tokens = self.limiter.total_tokens
        self.limiter.total_tokens = 4

    async def asyncTearDown(self):
        self.before_snapshot = self.after_snapshot = None
        for task in self.background:
            task.cancel()
        await asyncio.gather(*self.background, return_exceptions=True)
        for control in self.controls.values():
            control.release.set()
        await run_in_threadpool(self.manager.shutdown, wait=True)
        await run_in_threadpool(self.service.close, source_fenced=True)
        await eventually(lambda: self.limiter.borrowed_tokens == 0)
        self.assertEqual(self.service._async_waiters, {})
        await run_in_threadpool(self.runtime.stop)
        self.assertTrue(self.bus.shutdown())
        self.limiter.total_tokens = self.old_tokens
        self.temp.cleanup()

    def add_task(self):
        control = SimpleNamespace(release=threading.Event(), outcome="succeeded")
        def runner(context):
            if not control.release.wait(10):
                raise AssertionError("执行器释放超时")
            context.ensure_active()
            if control.outcome == "failed":
                raise RuntimeError("离线执行器明确失败")
            if control.outcome == "timeout":
                raise TaskTimeoutError("离线执行器超时")
            return {"complete": True}
        spec = self.manager.submit("wait_example", runner)
        self.controls[spec.task_id] = control
        return spec.task_id

    async def finish(self, state="succeeded", task_id=None):
        task_id = task_id or self.task_id
        if state == "cancelled":
            self.manager.cancel_task(task_id)
        self.controls[task_id].outcome = state
        self.controls[task_id].release.set()
        await eventually(lambda: self.manager.get_task_status(task_id).state.value in TERMINAL_STATES)

    def start_wait(self, task_id=None, timeout=30):
        task = asyncio.create_task(self.service.wait_task_async(task_id or self.task_id, timeout))
        self.background.append(task)
        return task

    async def idle(self, count=1):
        await eventually(lambda: idle_count(self.service) == count and self.limiter.borrowed_tokens == 0)

    def assert_wait_clean(self):
        self.assertEqual(set(self.bus._subscriptions), self.base_subscriptions)
        self.assertEqual(self.service._async_waiters, {})

    async def test_zero_timeout_preserves_status_and_reports_timeout(self):
        expected = await run_in_threadpool(self.service.wait_task, self.task_id, 0)
        actual = await self.service.wait_task_async(self.task_id, 0)
        self.assertEqual(actual["status"], expected["status"])
        self.assertTrue(actual["wait_timed_out"])
        self.assert_wait_clean()

    async def test_all_terminal_states_preserve_receipt_and_persistence(self):
        for index, state in enumerate(sorted(TERMINAL_STATES)):
            task_id = self.task_id if index == 0 else self.add_task()
            with self.subTest(state=state):
                await self.finish(state, task_id)
                receipt = await self.service.wait_task_async(task_id, 0)
                self.assertFalse(receipt["wait_timed_out"])
                self.assertEqual(receipt["status"]["state"], state)
                self.assertEqual(self.service.store.get_terminal(task_id)["snapshot"]["status"]["state"], state)
        self.assert_wait_clean()

    async def test_terminal_change_wakes_and_saves(self):
        waiter = self.start_wait()
        await self.idle()
        await self.finish()
        receipt = await asyncio.wait_for(waiter, 1)
        self.assertFalse(receipt["wait_timed_out"])
        self.assertEqual(receipt["status"]["state"], "succeeded")
        self.assertEqual(self.service.store.get_terminal(self.task_id)["snapshot"]["result"], {"complete": True})

    async def test_stage_change_but_not_progress_or_other_task_wakes(self):
        waiter = self.start_wait()
        await self.idle()
        self.manager.update_task(self.task_id, progress=0.4)
        other = self.add_task()
        self.manager.update_task(other, stage="unrelated")
        await self.idle()
        self.assertFalse(waiter.done())
        self.manager.update_task(self.task_id, stage="finishing")
        receipt = await asyncio.wait_for(waiter, 1)
        self.assertEqual(receipt["status"]["stage"], "finishing")
        self.assertFalse(receipt["wait_timed_out"])

    async def test_finite_timeout_returns_final_snapshot(self):
        started = time.monotonic()
        receipt = await self.service.wait_task_async(self.task_id, 0.025)
        self.assertTrue(receipt["wait_timed_out"])
        self.assertGreaterEqual(time.monotonic() - started, 0.02)
        self.assertEqual(receipt["status"]["task_id"], self.task_id)
        self.assert_wait_clean()

    async def test_invalid_timeout_bounds(self):
        for timeout in (-1, 30.001, float("nan"), float("inf"), -float("inf")):
            with self.subTest(timeout=timeout), self.assertRaises(GatewayError) as caught:
                await self.service.wait_task_async(self.task_id, timeout)
            self.assertEqual(caught.exception.error_code, "invalid_params")
        self.assertEqual(self.snapshot_count, 0)

    async def test_unknown_task_does_not_use_historical_record(self):
        old_id = build_task_id("old_example")
        self.service.store.save_terminal(old_id, {"task_id": old_id, "status": {"state": "succeeded"}},
                                         source_id="old-runtime", revision=1)
        with self.assertRaises(GatewayError) as caught:
            await self.service.wait_task_async(old_id, 0)
        self.assertEqual(caught.exception.http_status, 404)
        self.assertEqual(caught.exception.details["status_source"], "live_task")
        self.assert_wait_clean()

    async def test_authorization_failure_has_no_wait_subscription(self):
        def deny(_descriptor):
            raise GatewayError("拒绝", error_code="forbidden", http_status=403)
        with patch.object(self.service.access_manager, "authorize_capability", deny), self.assertRaises(GatewayError) as caught:
            await self.service.wait_task_async(self.task_id, 1)
        self.assertEqual(caught.exception.http_status, 403)
        self.assert_wait_clean()

    async def test_baseline_subscribe_race_rechecks_snapshot(self):
        original = self.bus.subscribe
        def subscribe(*args, **kwargs):
            if not kwargs:
                self.manager.update_task(self.task_id, stage="race-winner")
            return original(*args, **kwargs)
        with patch.object(self.bus, "subscribe", subscribe):
            receipt = await asyncio.wait_for(self.service.wait_task_async(self.task_id, 30), 1)
        self.assertEqual(receipt["status"]["stage"], "race-winner")
        self.assertFalse(receipt["wait_timed_out"])

    async def test_snapshot_wait_race_preserves_notification(self):
        def transition(read_count):
            if read_count == 1:
                self.manager.update_task(self.task_id, stage="after-snapshot")
        self.after_snapshot = transition
        receipt = await asyncio.wait_for(self.service.wait_task_async(self.task_id, 30), 1)
        self.assertEqual(receipt["status"]["stage"], "after-snapshot")
        self.assertFalse(receipt["wait_timed_out"])

    async def test_cancel_wait_unsubscribes_without_cancelling_task(self):
        waiter = self.start_wait()
        await self.idle()
        waiter.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await waiter
        self.assertEqual(self.cancel_count, 0)
        self.assertEqual(self.manager.get_task_status(self.task_id).state.value, "running")
        self.assert_wait_clean()

    async def test_snapshot_error_after_subscribe_cleans_up(self):
        waiter = self.start_wait()
        await self.idle()
        def fail(_count):
            raise GatewayError("任务已消失", error_code="not_found", http_status=404)
        self.before_snapshot = fail
        self.manager.update_task(self.task_id, stage="changed")
        with self.assertRaises(GatewayError) as caught:
            await asyncio.wait_for(waiter, 1)
        self.assertEqual(caught.exception.http_status, 404)
        self.assert_wait_clean()

    async def test_receipt_storage_error_cleans_up(self):
        await self.finish()
        with patch.object(self.service.store, "save_terminal", side_effect=OSError("保存失败")), self.assertRaises(OSError):
            await self.service.wait_task_async(self.task_id, 0)
        self.assert_wait_clean()

    async def test_service_close_wakes_all_waiters_and_rejects_new_wait(self):
        waiters = [self.start_wait() for _ in range(8)]
        await self.idle(8)
        await run_in_threadpool(self.service.close)
        for waiter in waiters:
            with self.assertRaises(GatewayError) as caught:
                await asyncio.wait_for(waiter, 1)
            self.assertEqual(caught.exception.error_code, "agent_service_closed")
        with self.assertRaises(GatewayError):
            await self.service.wait_task_async(self.task_id, 0)
        self.assertEqual(self.cancel_count, 0)
        self.assertEqual(self.service._async_waiters, {})

    async def test_snapshot_and_persistence_stay_off_event_loop(self):
        await self.finish()
        loop_thread = threading.get_ident()
        await self.service.wait_task_async(self.task_id, 0)
        self.assertTrue(self.snapshot_threads)
        self.assertTrue(self.store_threads)
        self.assertNotIn(loop_thread, self.snapshot_threads + self.store_threads)

    async def test_many_http_waiters_leave_read_and_cancel_independent(self):
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=self.app), base_url="http://test") as client:
            task_ids = [self.add_task() for _ in range(64)]
            waiters = []
            for task_id in task_ids:
                waiter = asyncio.create_task(client.get(f"/api/agent/tasks/{task_id}/wait"))
                self.background.append(waiter)
                waiters.append(waiter)
            await self.idle(64)
            read, cancel = await asyncio.wait_for(asyncio.gather(
                client.get(f"/api/agent/tasks/{self.task_id}"), client.post(f"/api/agent/tasks/{task_ids[0]}/cancel")), 2)
            self.assertEqual(read.status_code, 200)
            self.assertEqual(cancel.status_code, 200)
            self.assertEqual(self.cancel_count, 1)
            terminal = await asyncio.wait_for(waiters[0], 1)
            self.assertEqual(terminal.json()["status"]["state"], "cancelled")
            self.assertFalse(any(waiter.done() for waiter in waiters[1:]))
            for waiter in waiters[1:]:
                waiter.cancel()
            await asyncio.gather(*waiters, return_exceptions=True)
            self.assert_wait_clean()

    async def start_asgi_request(self):
        incoming = asyncio.Queue()
        sent = []
        active_receivers = max_receivers = 0
        async def receive():
            nonlocal active_receivers, max_receivers
            active_receivers += 1
            max_receivers = max(max_receivers, active_receivers)
            try:
                return await incoming.get()
            finally:
                active_receivers -= 1
        async def send(message):
            sent.append(message)
        path = f"/api/agent/tasks/{self.task_id}/wait"
        scope = {"type": "http", "asgi": {"version": "3.0"}, "http_version": "1.1", "method": "GET",
                 "scheme": "http", "path": path, "raw_path": path.encode(), "query_string": b"",
                 "headers": [], "server": ("test", 80), "client": ("test", 1)}
        task = asyncio.create_task(self.app(scope, receive, send))
        self.background.append(task)
        await incoming.put({"type": "http.request", "body": b"ignored", "more_body": True})
        await self.idle()
        return task, incoming, sent, lambda: (active_receivers, max_receivers)

    async def test_disconnect_with_chunked_get_body_has_single_receiver(self):
        task, incoming, sent, receivers = await self.start_asgi_request()
        await incoming.put({"type": "http.request", "body": b"remaining", "more_body": False})
        await incoming.put({"type": "http.disconnect"})
        await asyncio.wait_for(task, 1)
        self.assertEqual(receivers(), (0, 1))
        self.assertEqual(sent[0]["status"], 499)
        self.assertEqual(self.cancel_count, 0)
        self.assert_wait_clean()

    async def test_asgi_request_cancellation_cleans_wait_and_receive_tasks(self):
        task, _incoming, _sent, receivers = await self.start_asgi_request()
        task.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await task
        self.assertEqual(receivers(), (0, 1))
        self.assert_wait_clean()


    async def test_terminal_at_final_capture_takes_precedence_over_timeout(self):
        capture = self.manager.capture_task_receipt
        captures = 0
        def completing_capture(task_id):
            nonlocal captures
            captures += 1
            if captures == 2:
                self.controls[task_id].release.set()
                deadline = time.monotonic() + 1
                while self.manager.get_task_status(task_id).state.value not in TERMINAL_STATES:
                    if time.monotonic() >= deadline:
                        raise AssertionError("执行器未完成")
                    time.sleep(0.001)
            return capture(task_id)
        with patch.object(self.manager, "capture_task_receipt", completing_capture):
            receipt = await self.service.wait_task_async(self.task_id, 0.01)
        self.assertEqual(receipt["status"]["state"], "succeeded")
        self.assertFalse(receipt["wait_timed_out"])
        self.assert_wait_clean()

    async def test_event_loop_shutdown_cleans_pending_wait(self):
        outcomes = []
        async def wait_in_loop():
            task = asyncio.create_task(self.service.wait_task_async(self.task_id, 30))
            await eventually(lambda: idle_count(self.service) == 1)
            outcomes.append(task)
        await run_in_threadpool(lambda: asyncio.run(wait_in_loop()))
        self.assertTrue(outcomes[0].cancelled())
        self.assertEqual(self.cancel_count, 0)
        self.assert_wait_clean()

    async def test_cancel_during_snapshot_does_not_leak_subscription(self):
        entered, release = threading.Event(), threading.Event()
        def block(read_count):
            if read_count == 2:
                entered.set()
                if not release.wait(2):
                    raise AssertionError("快照释放超时")
        self.after_snapshot = block
        waiter = self.start_wait()
        try:
            await self.idle()
            self.manager.update_task(self.task_id, stage="new-stage")
            await eventually(entered.is_set)
            waiter.cancel()
            with self.assertRaises(asyncio.CancelledError):
                await asyncio.wait_for(waiter, 1)
            self.assert_wait_clean()
        finally:
            release.set()

    async def test_close_between_baseline_and_subscription(self):
        entered, release = threading.Event(), threading.Event()
        def block(read_count):
            if read_count == 1:
                entered.set()
                if not release.wait(2):
                    raise AssertionError("基线快照释放超时")
        self.after_snapshot = block
        waiter = self.start_wait()
        try:
            await eventually(entered.is_set)
            await run_in_threadpool(self.service.close)
        finally:
            release.set()
        with self.assertRaises(GatewayError) as caught:
            await asyncio.wait_for(waiter, 1)
        self.assertEqual(caught.exception.error_code, "agent_service_closed")
        self.assertEqual(self.service._async_waiters, {})

    async def test_route_errors_and_zero_timeout_receipt(self):
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=self.app), base_url="http://test") as client:
            for invalid in ("bad", "NaN", "inf", "-1", "31"):
                response = await client.get(f"/api/agent/tasks/{self.task_id}/wait", params={"timeout_sec": invalid})
                self.assertEqual(response.status_code, 422)
                self.assertEqual(response.json()["error"]["code"], "invalid_params")
            missing = await client.get("/api/agent/tasks/missing/wait", params={"timeout_sec": 0})
            self.assertEqual(missing.status_code, 404)
            response = await client.get(f"/api/agent/tasks/{self.task_id}/wait", params={"timeout_sec": 0})
            self.assertEqual(response.status_code, 200)
            self.assertTrue(response.json()["wait_timed_out"])
        self.assert_wait_clean()


class WakeupBoundTests(unittest.IsolatedAsyncioTestCase):
    async def test_coalesces_pending_and_delivered_notifications(self):
        callbacks = []
        wake = _AsyncTaskWakeup(SimpleNamespace(call_soon_threadsafe=callbacks.append))
        for _ in range(10000):
            wake.notify()
        self.assertEqual(len(callbacks), 1)
        callbacks.pop()()
        self.assertTrue(wake.event.is_set())
        for _ in range(10000):
            wake.notify()
        self.assertEqual(callbacks, [])
        wake.clear()
        wake.notify()
        wake.clear()
        wake.notify()
        self.assertEqual(len(callbacks), 1)
        callbacks.pop()()
        self.assertTrue(wake.event.is_set())
        wake.clear()
        wake.notify()
        wake.close()
        callbacks.pop()()
        self.assertFalse(wake.event.is_set())
        wake.notify()
        self.assertEqual(callbacks, [])

    async def test_event_storm_queues_one_cross_thread_callback(self):
        loop = asyncio.get_running_loop()
        wake = _AsyncTaskWakeup(loop)
        with patch.object(loop, "call_soon_threadsafe", wraps=loop.call_soon_threadsafe) as schedule:
            def publish():
                for _ in range(10000):
                    wake.notify()
            thread = threading.Thread(target=publish)
            thread.start()
            thread.join(1)
            self.assertFalse(thread.is_alive())
            self.assertEqual(schedule.call_count, 1)
        await asyncio.wait_for(wake.event.wait(), 1)
        wake.close()

    async def test_closed_loop_and_late_callback_are_safe(self):
        loop = asyncio.new_event_loop()
        wake = _AsyncTaskWakeup(loop)
        loop.close()
        wake.notify()
        wake.notify()
        wake.close()
        self.assertFalse(wake.event.is_set())


if __name__ == "__main__":
    unittest.main(verbosity=2)
