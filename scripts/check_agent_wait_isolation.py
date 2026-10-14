#!/usr/bin/env python3
"""离线验证智能体等待隔离、回执语义与断连清理，不调用设备或模型。

使用真实 AgentToolService 方法、路由、快照/回执投影，替换任务运行时与
持久存储边界；另有真实 EventBus 集成检查。运行：
.nuwax-test-env/bin/python scripts/check_agent_wait_isolation.py
"""
from __future__ import annotations

import asyncio
import sys
import threading
import time
import unittest
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import anyio.to_thread
import httpx
from fastapi import FastAPI
from fastapi.concurrency import run_in_threadpool

from contracts.events import RuntimeEventCategory
from contracts.naming import build_task_id
from core.errors import GatewayError
from core.event_bus import EventBus
from gateways.agent.feedback import TERMINAL_STATES
from gateways.agent.router import build_agent_router
from gateways.agent.service import AgentToolService, _AsyncTaskWakeup


class SyntheticBus:
    """订阅边界；不运行生产长期终态消费者，等待者的通知仍跨线程验证。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.callbacks = {}
        self.next_id = 0
        self.before_subscribe = None
        self.unsubscribed = []

    def subscribe(self, callback, **options):
        if self.before_subscribe and not options:
            self.before_subscribe()
        with self.lock:
            self.next_id += 1
            self.callbacks[self.next_id] = (callback, options)
            return self.next_id

    def unsubscribe(self, subscription):
        with self.lock:
            if subscription not in self.callbacks:
                raise AssertionError("subscription removed twice")
            del self.callbacks[subscription]
            self.unsubscribed.append(subscription)

    def publish(self, task_id):
        with self.lock:
            callbacks = tuple(self.callbacks.values())
        for callback, options in callbacks:
            if not options.get("async_dispatch"):
                callback(SimpleNamespace(task_id=task_id))

    def count(self):
        with self.lock:
            return len(self.callbacks)


class SyntheticStore:
    def __init__(self):
        self.terminals = {}
        self.threads = []

    def find_task_call_identity(self, task_id):
        self.threads.append(threading.get_ident())

    def save_terminal(self, task_id, snapshot):
        self.threads.append(threading.get_ident())
        self.terminals[task_id] = deepcopy(snapshot)


class SyntheticRuntime:
    def __init__(self):
        self._lock = threading.RLock()
        self.task_manager = self
        self.event_bus = SyntheticBus()
        self.robot = SimpleNamespace(manifest=SimpleNamespace(robot_model="synthetic"))
        self.config = SimpleNamespace(gateway=SimpleNamespace(artifact_dir="/tmp/agent-wait-check/artifacts"))
        self.capability_registry = SimpleNamespace(get_descriptor=lambda name: name)
        self.statuses = {}
        self.snapshot_count = 0
        self.snapshot_threads = []
        self.after_snapshot = None
        self.before_snapshot = None
        self.cancel_count = 0
        self.add_task("probe")

    def add_task(self, task_id):
        with self._lock:
            self.statuses[task_id] = {"task_id": task_id, "state": "running", "stage": "working"}

    def get_task_spec(self, task_id):
        self.statuses[task_id]
        return SimpleNamespace(capability_name="diagnostic.synthetic")

    def get_task_status(self, task_id):
        return dict(self.statuses[task_id])

    def get_task_events(self, task_id):
        return []

    def get_task_result(self, task_id):
        return {"complete": True} if self.statuses[task_id]["state"] in TERMINAL_STATES else None

    def get_task_error(self, task_id):
        return None

    def get_task_error_payload(self, task_id):
        return None

    def get_task_snapshot(self, task_id):
        self.snapshot_count += 1
        self.snapshot_threads.append(threading.get_ident())
        if self.before_snapshot:
            self.before_snapshot(self.snapshot_count)
        snapshot = {"task_id": task_id, "task": {"capability_name": "diagnostic.synthetic", "metadata": {}},
                    "status": self.get_task_status(task_id), "events": [], "result": self.get_task_result(task_id),
                    "error": None, "error_payload": None}
        if self.after_snapshot:
            self.after_snapshot(self.snapshot_count)
        return snapshot

    def change(self, task_id="probe", **status):
        with self._lock:
            self.statuses[task_id].update(status)
        self.event_bus.publish(task_id)

    def _handle_cancel_task(self, arguments, *, requested_by):
        self.cancel_count += 1
        self.change(arguments["task_id"], state="cancelled")


def fixture():
    runtime = SyntheticRuntime()
    store = SyntheticStore()
    with patch("gateways.agent.service.AgentCallStore", return_value=store):
        service = AgentToolService(runtime, SimpleNamespace(authorize_capability=lambda descriptor: None))
    app = FastAPI()
    app.include_router(build_agent_router(service))
    return runtime, service, app


async def eventually(predicate, timeout=2.0):
    """只以可观察条件建立屏障；超时仅用于让回归快速失败。"""
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError("condition barrier timed out")
        await asyncio.sleep(0)


def idle_count(service):
    with service._waiters_lock:
        return sum(bool(waiter.event._waiters) for waiter in service._async_waiters.values())


class AgentWaitIsolationTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.runtime, self.service, self.app = fixture()
        self.background = []
        self.limiter = anyio.to_thread.current_default_thread_limiter()
        self.old_tokens = self.limiter.total_tokens
        self.limiter.total_tokens = 4

    async def asyncTearDown(self):
        for task in self.background:
            task.cancel()
        await asyncio.gather(*self.background, return_exceptions=True)
        self.service.close()
        await eventually(lambda: self.limiter.borrowed_tokens == 0)
        self.assertEqual(self.runtime.event_bus.count(), 0)
        self.assertEqual(self.service._async_waiters, {})
        self.limiter.total_tokens = self.old_tokens

    def start_wait(self, task_id="probe", timeout=30):
        task = asyncio.create_task(self.service.wait_task_async(task_id, timeout))
        self.background.append(task)
        return task

    async def idle(self, count=1):
        await eventually(lambda: idle_count(self.service) == count and self.limiter.borrowed_tokens == 0)

    async def test_zero_timeout_preserves_sync_receipt(self):
        expected = await run_in_threadpool(self.service.wait_task, "probe", 0)
        actual = await self.service.wait_task_async("probe", 0)
        self.assertEqual(actual, expected)
        self.assertTrue(actual["wait_timed_out"])
        self.assertEqual(self.runtime.event_bus.count(), 1)

    async def test_all_terminal_states_preserve_receipt_and_persistence(self):
        for state in sorted(TERMINAL_STATES):
            with self.subTest(state=state):
                self.runtime.statuses["probe"]["state"] = state
                expected = await run_in_threadpool(self.service.wait_task, "probe", 0)
                self.assertEqual(await self.service.wait_task_async("probe", 0), expected)
                self.assertFalse(expected["wait_timed_out"])
                self.assertEqual(self.service.store.terminals["probe"]["status"]["state"], state)

    async def test_terminal_change_wakes_and_saves(self):
        waiter = self.start_wait()
        await self.idle()
        await run_in_threadpool(self.runtime.change, state="succeeded")
        receipt = await asyncio.wait_for(waiter, 1)
        self.assertFalse(receipt["wait_timed_out"])
        self.assertEqual(receipt["status"]["state"], "succeeded")
        self.assertEqual(self.service.store.terminals["probe"]["result"], {"complete": True})

    async def test_stage_change_but_not_progress_or_other_task_wakes(self):
        waiter = self.start_wait()
        await self.idle()
        reads = self.runtime.snapshot_count
        self.runtime.event_bus.publish("unrelated")
        self.runtime.change(progress=0.4)
        await eventually(lambda: self.runtime.snapshot_count > reads)
        await self.idle()
        self.assertFalse(waiter.done())
        self.runtime.change(stage="finishing")
        receipt = await asyncio.wait_for(waiter, 1)
        self.assertEqual(receipt["status"]["stage"], "finishing")
        self.assertFalse(receipt["wait_timed_out"])

    async def test_finite_timeout_and_final_snapshot(self):
        started = time.monotonic()
        receipt = await self.service.wait_task_async("probe", 0.025)
        self.assertTrue(receipt["wait_timed_out"])
        self.assertGreaterEqual(time.monotonic() - started, 0.02)
        self.assertGreaterEqual(self.runtime.snapshot_count, 3)

    async def test_terminal_at_timeout_takes_precedence(self):
        def transition(read_count):
            if read_count == 3:
                self.runtime.statuses["probe"]["state"] = "succeeded"
        self.runtime.before_snapshot = transition
        receipt = await self.service.wait_task_async("probe", 0.01)
        self.assertFalse(receipt["wait_timed_out"])
        self.assertEqual(receipt["status"]["state"], "succeeded")

    async def test_invalid_timeout_bounds(self):
        for timeout in (-1, 30.001, float("nan"), float("inf"), -float("inf")):
            with self.subTest(timeout=timeout), self.assertRaises(GatewayError) as caught:
                await self.service.wait_task_async("probe", timeout)
            self.assertEqual(caught.exception.error_code, "invalid_params")
        self.assertEqual(self.runtime.snapshot_count, 0)

    async def test_unknown_task_does_not_use_historical_record(self):
        self.service.store.terminals["old-task"] = {"status": {"state": "succeeded"}}
        with self.assertRaises(GatewayError) as caught:
            await self.service.wait_task_async("old-task", 0)
        self.assertEqual(caught.exception.http_status, 404)
        self.assertEqual(caught.exception.details["status_source"], "live_task")
        self.assertEqual(self.runtime.event_bus.count(), 1)

    async def test_authorization_failure_has_no_wait_subscription(self):
        def deny(_descriptor):
            raise GatewayError("denied", error_code="forbidden", http_status=403)
        self.service.access_manager.authorize_capability = deny
        with self.assertRaises(GatewayError) as caught:
            await self.service.wait_task_async("probe", 1)
        self.assertEqual(caught.exception.http_status, 403)
        self.assertEqual(self.runtime.event_bus.count(), 1)

    async def test_baseline_subscribe_race_rechecks_snapshot(self):
        self.runtime.event_bus.before_subscribe = lambda: self.runtime.change(stage="race-winner")
        receipt = await asyncio.wait_for(self.service.wait_task_async("probe", 30), 1)
        self.assertEqual(receipt["status"]["stage"], "race-winner")
        self.assertFalse(receipt["wait_timed_out"])

    async def test_snapshot_wait_race_preserves_notification(self):
        def transition(read_count):
            if read_count == 2:
                self.runtime.change(stage="after-snapshot")
        self.runtime.after_snapshot = transition
        receipt = await asyncio.wait_for(self.service.wait_task_async("probe", 30), 1)
        self.assertEqual(receipt["status"]["stage"], "after-snapshot")
        self.assertFalse(receipt["wait_timed_out"])

    async def test_cancel_wait_unsubscribes_without_cancelling_robot_task(self):
        waiter = self.start_wait()
        await self.idle()
        waiter.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await waiter
        self.assertEqual(self.runtime.cancel_count, 0)
        self.assertEqual(self.runtime.event_bus.count(), 1)
        self.assertEqual(self.service._async_waiters, {})

    async def test_snapshot_error_after_subscribe_cleans_up(self):
        def fail_after_subscribe(read_count):
            if read_count == 2:
                raise GatewayError("task disappeared", error_code="not_found", http_status=404)
        self.runtime.before_snapshot = fail_after_subscribe
        with self.assertRaises(GatewayError) as caught:
            await self.service.wait_task_async("probe", 30)
        self.assertEqual(caught.exception.http_status, 404)
        self.assertEqual(self.runtime.event_bus.count(), 1)
        self.assertEqual(self.service._async_waiters, {})

    async def test_receipt_storage_error_cleans_up(self):
        self.runtime.statuses["probe"]["state"] = "succeeded"
        failure = patch.object(self.service.store, "save_terminal", side_effect=OSError("synthetic store failure"))
        with self.assertRaises(OSError), failure:
            await self.service.wait_task_async("probe", 0)
        self.assertEqual(self.runtime.event_bus.count(), 1)
        self.assertEqual(self.service._async_waiters, {})

    async def test_event_loop_shutdown_cleans_pending_wait(self):
        outcomes = []
        async def run_wait():
            waiter = asyncio.create_task(self.service.wait_task_async("probe", 30))
            await eventually(lambda: idle_count(self.service) == 1)
            # asyncio.run 在关闭循环前取消未完成任务，生产 finally 必须完成注销。
            outcomes.append(waiter)
        def run_loop():
            asyncio.run(run_wait())
        await run_in_threadpool(run_loop)
        self.assertTrue(outcomes[0].cancelled())
        self.assertEqual(self.runtime.event_bus.count(), 1)
        self.assertEqual(self.service._async_waiters, {})

    async def test_cancel_during_snapshot_does_not_leak_subscription(self):
        entered = threading.Event()
        release = threading.Event()
        def block_second_read(read_count):
            if read_count == 2:
                entered.set()
                if not release.wait(2):
                    raise AssertionError("snapshot release timed out")
        self.runtime.after_snapshot = block_second_read
        waiter = self.start_wait()
        try:
            await eventually(entered.is_set)
            waiter.cancel()
            with self.assertRaises(asyncio.CancelledError):
                await waiter
            self.assertEqual(self.runtime.event_bus.count(), 1)
            self.assertEqual(self.service._async_waiters, {})
        finally:
            release.set()

    async def test_service_close_wakes_all_waiters_and_rejects_new_wait(self):
        waiters = [self.start_wait() for _ in range(8)]
        await self.idle(8)
        await run_in_threadpool(self.service.close)
        for waiter in waiters:
            with self.assertRaises(GatewayError) as caught:
                await asyncio.wait_for(waiter, 1)
            self.assertEqual(caught.exception.error_code, "agent_service_closed")
        with self.assertRaises(GatewayError):
            await self.service.wait_task_async("probe", 0)
        self.assertEqual(self.runtime.event_bus.count(), 0)
        self.assertEqual(self.runtime.cancel_count, 0)

    async def test_close_between_baseline_and_subscription(self):
        entered = threading.Event()
        release = threading.Event()
        def block_baseline(read_count):
            if read_count == 1:
                entered.set()
                if not release.wait(2):
                    raise AssertionError("baseline release timed out")
        self.runtime.after_snapshot = block_baseline
        waiter = self.start_wait()
        try:
            await eventually(entered.is_set)
            self.service.close()
        finally:
            release.set()
        with self.assertRaises(GatewayError) as caught:
            await asyncio.wait_for(waiter, 1)
        self.assertEqual(caught.exception.error_code, "agent_service_closed")
        self.assertEqual(self.runtime.event_bus.count(), 0)

    async def test_snapshot_receipt_and_persistence_stay_off_event_loop(self):
        loop_thread = threading.get_ident()
        self.runtime.statuses["probe"]["state"] = "succeeded"
        await self.service.wait_task_async("probe", 0)
        self.assertTrue(self.runtime.snapshot_threads)
        self.assertTrue(self.service.store.threads)
        self.assertNotIn(loop_thread, self.runtime.snapshot_threads + self.service.store.threads)

    async def test_route_errors_and_terminal_receipt(self):
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=self.app), base_url="http://test") as client:
            for invalid in ("bad", "NaN", "inf", "-1", "31"):
                response = await client.get("/api/agent/tasks/probe/wait", params={"timeout_sec": invalid})
                self.assertEqual(response.status_code, 422)
                self.assertEqual(response.json()["error"]["code"], "invalid_params")
            missing = await client.get("/api/agent/tasks/old/wait", params={"timeout_sec": 0})
            self.assertEqual(missing.status_code, 404)
            response = await client.get("/api/agent/tasks/probe/wait", params={"timeout_sec": 0})
            self.assertEqual(response.status_code, 200)
            self.assertTrue(response.json()["wait_timed_out"])

    async def test_many_http_waiters_leave_read_and_cancel_independent(self):
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=self.app), base_url="http://test") as client:
            waiters = []
            for index in range(64):
                task_id = f"wait-{index}"
                self.runtime.add_task(task_id)
                waiter = asyncio.create_task(client.get(f"/api/agent/tasks/{task_id}/wait"))
                self.background.append(waiter)
                waiters.append(waiter)
            await self.idle(64)
            self.assertEqual(self.limiter.borrowed_tokens, 0)
            self.assertEqual(self.runtime.event_bus.count(), 65)
            read, cancel = await asyncio.wait_for(asyncio.gather(
                client.get("/api/agent/tasks/probe"), client.post("/api/agent/tasks/wait-0/cancel")), 1)
            self.assertEqual(read.status_code, 200)
            self.assertEqual(cancel.status_code, 200)
            self.assertEqual(self.runtime.cancel_count, 1)
            terminal = await asyncio.wait_for(waiters[0], 1)
            self.assertEqual(terminal.json()["status"]["state"], "cancelled")
            self.assertFalse(any(waiter.done() for waiter in waiters[1:]))
            for waiter in waiters[1:]:
                waiter.cancel()
            await asyncio.gather(*waiters, return_exceptions=True)
            self.assertEqual(self.runtime.event_bus.count(), 1)
            self.assertEqual(self.service._async_waiters, {})

    async def start_asgi_request(self):
        incoming = asyncio.Queue()
        sent = []
        active_receivers = 0
        max_receivers = 0
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
        scope = {"type": "http", "asgi": {"version": "3.0"}, "http_version": "1.1", "method": "GET",
                 "scheme": "http", "path": "/api/agent/tasks/probe/wait", "raw_path": b"/api/agent/tasks/probe/wait",
                 "query_string": b"", "headers": [], "server": ("test", 80), "client": ("test", 1)}
        task = asyncio.create_task(self.app(scope, receive, send))
        self.background.append(task)
        await incoming.put({"type": "http.request", "body": b"ignored-get-body", "more_body": True})
        await self.idle()
        return task, incoming, sent, lambda: (active_receivers, max_receivers)

    async def test_disconnect_with_chunked_get_body_has_single_receiver(self):
        task, incoming, sent, receivers = await self.start_asgi_request()
        await incoming.put({"type": "http.request", "body": b"remaining-body", "more_body": False})
        await incoming.put({"type": "http.disconnect"})
        await asyncio.wait_for(task, 1)
        self.assertEqual(receivers(), (0, 1))
        self.assertEqual(sent[0]["status"], 499)
        self.assertEqual(self.runtime.event_bus.count(), 1)
        self.assertEqual(self.service._async_waiters, {})
        self.assertEqual(self.runtime.cancel_count, 0)

    async def test_asgi_request_cancellation_cleans_wait_and_receive_tasks(self):
        task, _incoming, _sent, receivers = await self.start_asgi_request()
        task.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await task
        self.assertEqual(receivers(), (0, 1))
        self.assertEqual(self.runtime.event_bus.count(), 1)
        self.assertEqual(self.service._async_waiters, {})

    async def test_real_event_bus_cross_thread_wakeup(self):
        bus = EventBus()
        original = self.runtime.event_bus
        # 保留构造时的终态消费者，单独把本次等待接入真实总线。
        self.runtime.event_bus = bus
        task_id = build_task_id("diagnostic_synthetic")
        self.runtime.add_task(task_id)
        waiter = self.start_wait(task_id)
        try:
            await self.idle()
            def publish():
                with self.runtime._lock:
                    self.runtime.statuses[task_id]["stage"] = "real-bus"
                bus.publish(bus.build_event("task.progress", category=RuntimeEventCategory.TASK,
                                            source="isolation_check", task_id=task_id))
            await run_in_threadpool(publish)
            receipt = await asyncio.wait_for(waiter, 1)
            self.assertEqual(receipt["status"]["stage"], "real-bus")
            self.assertEqual(len(bus._subscriptions), 0)
        finally:
            waiter.cancel()
            await asyncio.gather(waiter, return_exceptions=True)
            self.runtime.event_bus = original
            await run_in_threadpool(bus.shutdown)


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
