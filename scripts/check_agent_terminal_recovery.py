#!/usr/bin/env python3
"""离线回执恢复/关闭检查；真实源与 SQLite、假动作、门闩，无吞吐性能宣称。"""
from __future__ import annotations

import json
import sqlite3
import sys
import tempfile
import threading
import time
import unittest
from contextlib import ExitStack
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fastapi import APIRouter

import gateways.feedback as public_feedback
from contracts.events import RuntimeEventCategory
from core.agent_call_store import AgentCallStore
from tests.conftest import create_isolated_gateway
from core.errors import GatewayError
from core.event_bus import EventBus
from core.settings import GatewayServerConfig
from core.task_manager import TaskManager
from gateways.agent.service import AgentToolService
from gateways.agent.terminal_persistence import TerminalReceiptPersistence
from gateways.app import create_gateway_app
from gateways.runtime import GatewayRuntime


class RecoveryChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="neh-terminal-owner-")
        self.bus = EventBus(retention_limit=1, async_queue_limit=1)
        self.manager = TaskManager(event_bus=self.bus)
        self.runtime = create_isolated_gateway(Path(self.temp.name), event_bus=self.bus, task_manager=self.manager)
        self.runtime.capability_registry = NS(get_descriptor=lambda name: NS(display_name="offline"))
        self.allowed = True
        def authorize(descriptor):
            self.assertFalse(self.manager._lock._is_owned())
            if not self.allowed:
                raise GatewayError("not exposed", error_code="access_denied", http_status=403)
        self.access = NS(authorize_capability=authorize)
        self.services, self.releases, self.calls = [], [], []

    def tearDown(self):
        for event in self.releases:
            event.set()
        self.manager.shutdown(wait=True)
        for service in self.services:
            closed = service.close(2, source_fenced=True)
            self.assertTrue(closed["owner_closed"], closed)
        self.runtime.stop()
        self.assertTrue(self.bus.shutdown(2))
        self.temp.cleanup()

    def service(self, *, paused=False, **options):
        starts = []
        def factory(*args, **kwargs):
            owner = TerminalReceiptPersistence(*args, **kwargs, **options)
            if paused:
                starts.append(owner.start)
                owner.start = lambda: None
            return owner
        with patch("gateways.agent.service.TerminalReceiptPersistence", factory):
            service = AgentToolService(self.runtime, self.access)
        if paused:
            service._terminal_owner.start = starts[0]
        self.services.append(service)
        return service

    def gate(self):
        entered, release = threading.Event(), threading.Event()
        self.releases.append(release)
        return entered, release

    def require(self, event):
        self.assertTrue(event.wait(3), "offline gate timeout")

    def until(self, service, predicate):
        deadline = time.monotonic() + 3
        while not predicate():
            remaining = deadline - time.monotonic()
            self.assertGreater(remaining, 0, service.persistence_status())
            with service._terminal_owner._cv:
                service._terminal_owner._cv.wait(min(0.02, remaining))

    def finish(self, result=None):
        task = self.manager.submit("offline_task", lambda context: self.calls.append(1) or result)
        self.manager.wait_for_task(task.task_id, timeout=2)
        return task.task_id

    def current_saved(self, service, task_id):
        row = service.store.terminal_versions((task_id,)).get(task_id)
        return bool(row and row["revision"] == self.manager.task_receipt_version(task_id)[1])

    def test_id_only_callback_and_blocked_store_do_not_stall_task_or_bus(self):
        service = self.service(paused=True)
        owner = service._terminal_owner
        with patch.object(self.manager, "task_receipt_header", side_effect=AssertionError("callback touched source")), \
                patch.object(service.store, "save_prepared_terminal", side_effect=AssertionError("callback touched store")), \
                patch.object(service, "_task_snapshot", side_effect=AssertionError("callback projected")):
            service._on_task_terminal(NS(task_id="hint-only"))
        entered, release = self.gate()
        original = service.store.save_prepared_terminal
        def blocked(prepared):
            self.assertFalse(self.manager._lock._is_owned())
            self.assertFalse(self.bus._lock._is_owned())
            entered.set()
            self.require(release)
            return original(prepared)
        with patch.object(service.store, "save_prepared_terminal", blocked):
            task_id = self.finish({"known": True})
            owner.start()
            self.require(entered)
            callback = threading.Event()
            self.bus.subscribe(lambda event: callback.set(), categories=(RuntimeEventCategory.SYSTEM,), async_dispatch=True)
            self.bus.publish(self.bus.build_event("system.offline", category=RuntimeEventCategory.SYSTEM, source="offline"))
            self.require(callback)
            other = self.finish({"other": True})
            self.assertEqual(self.manager.get_task_status(other).state, "succeeded")
            closed = service.close(0, source_fenced=False)
            self.assertFalse(closed["owner_closed"])
            self.assertFalse(closed["complete"])
            thread = owner._thread
            self.assertFalse(service.close(0)["owner_closed"])
            self.assertIs(owner._thread, thread)
            release.set()
            self.until(service, lambda: self.current_saved(service, task_id) and self.current_saved(service, other))
        self.assertEqual(self.calls, [1, 1])

    def test_capacity_one_overflow_and_expired_replay_reconcile_all_latest_revisions(self):
        service = self.service(paused=True, capacity=1, page_size=1)
        tasks = [self.finish({"n": index}) for index in range(4)]
        self.manager.update_task_metadata(tasks[0], {"late": True})
        self.manager.publish_task_event(tasks[1], "task.custom_audit", payload={"known": True})
        health = service.persistence_status()
        self.assertGreater(health["overflow_count"], 0)
        self.assertEqual(health["mailbox_count"], 1)
        self.assertFalse(health["pending_count_complete"])
        self.assertLessEqual(health["mailbox_id_bytes"], health["mailbox_id_byte_limit"])
        service._terminal_owner.start()
        self.until(service, lambda: all(self.current_saved(service, task_id) for task_id in tasks))
        self.assertEqual(service.store.get_terminal(tasks[0])["snapshot"]["task"]["metadata"]["late"], True)
        self.assertEqual(self.calls, [1] * 4)

    def test_startup_and_lost_late_wake_repair_without_action_replay(self):
        task_id = self.finish({"known": True})
        now = [0.0]
        service = self.service(clock=lambda: now[0], scan_interval=10)
        self.until(service, lambda: service.persistence_status()["persistence_complete_at_observed_fence"])
        self.bus.unsubscribe(service._subscription_id)
        service._subscription_id = None
        self.manager.update_task_metadata(task_id, {"lost_wake": True})
        now[0] = 10.0
        service._terminal_owner.wake()
        self.until(service, lambda: self.current_saved(service, task_id))
        self.assertTrue(service.store.get_terminal(task_id)["snapshot"]["task"]["metadata"]["lost_wake"])
        self.assertEqual(self.calls, [1])

    def test_stale_owner_write_and_new_revision_while_inflight(self):
        task_id = self.finish({"v": 1})
        service = self.service(paused=True)
        entered, release = self.gate()
        original = service.store.save_prepared_terminal
        first = [True]
        dispositions = []
        def delayed(prepared):
            if threading.current_thread().name == "Agent-terminal-receipts" and first[0]:
                first[0] = False
                entered.set()
                self.require(release)
            outcome = original(prepared)
            dispositions.append(outcome["disposition"])
            return outcome
        with patch.object(service.store, "save_prepared_terminal", delayed):
            service._terminal_owner.start()
            self.require(entered)
            self.manager.update_task_metadata(task_id, {"newer": 2})
            newer = service.get_task_result(task_id)
            self.manager.update_task_metadata(task_id, {"latest": 3})
            release.set()
            self.until(service, lambda: self.current_saved(service, task_id))
        saved = service.store.get_terminal(task_id)["snapshot"]
        self.assertIn("stale", dispositions)
        self.assertEqual(saved["task"]["metadata"]["newer"], newer["task"]["metadata"]["newer"])
        self.assertEqual(saved["task"]["metadata"]["latest"], 3)
        self.assertEqual(self.calls, [1])

    def test_storage_backoff_reuses_prepared_content_without_hot_retry(self):
        now = [0.0]
        service = self.service(paused=True, clock=lambda: now[0], retry_base=1, retry_cap=4)
        task_id = self.finish({"v": 1})
        failed = threading.Event()
        attempts = []
        original = service.store.save_prepared_terminal
        def fail_once(prepared):
            attempts.append(prepared)
            if len(attempts) == 1:
                failed.set()
                raise sqlite3.OperationalError("offline busy")
            return original(prepared)
        with patch.object(service.store, "save_prepared_terminal", fail_once):
            service._terminal_owner.start()
            self.require(failed)
            self.until(service, lambda: service.persistence_status()["retry_at"] is not None)
            for _ in range(50):
                service._terminal_owner.offer(task_id)
                service._terminal_owner.wake()
            self.assertEqual(len(attempts), 1)
            self.assertFalse(service.persistence_status()["pending_count_complete"])
            now[0] = 1.0
            service._terminal_owner.wake()
            self.until(service, lambda: self.current_saved(service, task_id))
        self.assertIs(attempts[0], attempts[1])
        self.assertEqual(self.calls, [1])

    def test_long_outage_coalesces_overflow_and_backoff_cap_is_observable(self):
        now = [0.0]
        service = self.service(paused=True, capacity=1, page_size=1, clock=lambda: now[0], retry_base=1, retry_cap=2)
        tasks = [self.finish({"n": 0})]
        failing = [True]
        original = service.store.save_prepared_terminal
        def unavailable(prepared):
            if failing[0]:
                raise sqlite3.OperationalError("offline unavailable")
            return original(prepared)
        with patch.object(service.store, "save_prepared_terminal", unavailable):
            service._terminal_owner.start()
            self.until(service, lambda: service.persistence_status()["failures"] == 1)
            tasks.extend(self.finish({"n": value}) for value in range(1, 4))
            self.manager.update_task_metadata(tasks[0], {"latest": 9})
            for expected_failures, instant, next_retry in ((2, 1.0, 3.0), (3, 3.0, 5.0)):
                now[0] = instant
                service._terminal_owner.wake()
                self.until(service, lambda expected_failures=expected_failures: service.persistence_status()["failures"] == expected_failures)
                health = service.persistence_status()
                self.assertEqual(health["retry_at"], next_retry)
                self.assertLessEqual(health["mailbox_count"], 1)
                self.assertGreater(health["overflow_count"], 0)
                self.assertFalse(health["pending_count_complete"])
                self.assertIsNotNone(health["oldest_known_pending_age_sec"])
            self.assertFalse(service.close(0, source_fenced=False)["complete"])
            failing[0] = False
            now[0] = 5.0
            service._terminal_owner.wake()
            self.until(service, lambda: all(self.current_saved(service, task_id) for task_id in tasks))
        self.assertEqual(service.store.get_terminal(tasks[0])["snapshot"]["task"]["metadata"]["latest"], 9)
        self.assertEqual(self.calls, [1] * 4)

    def test_late_cancel_and_timeout_evidence_recover_once_with_lost_wakes(self):
        service = self.service()
        for timeout in (False, True):
            entered, release = self.gate()
            def runner(context, entered=entered, release=release):
                self.calls.append(1)
                entered.set()
                self.require(release)
                return {"returned_evidence": True}
            task = self.manager.submit("offline_task", runner)
            self.require(entered)
            if timeout:
                self.manager._handle_timeout(task.task_id)
            else:
                self.manager.cancel_task(task.task_id, "original cancel reason")
            before = self.manager.get_task_status(task.task_id)
            with patch.object(service._terminal_owner, "offer", return_value=None):
                release.set()
                self.manager.wait_for_task(task.task_id, timeout=2)
                self.manager.update_task_metadata(task.task_id, {"late_finalizer": True})
            service._terminal_owner.request_reconcile()
            self.until(service, lambda task=task: self.current_saved(service, task.task_id))
            saved = service.store.get_terminal(task.task_id)["snapshot"]
            self.assertEqual(saved["result"], {"returned_evidence": True})
            self.assertEqual(saved["status"]["state"], "timeout" if timeout else "cancelled")
            self.assertEqual(self.manager.get_task_status(task.task_id), before)
            self.assertFalse(self.manager._records[task.task_id].completion_committed)
        self.assertEqual(self.calls, [1, 1])

    def test_close_with_unsupported_receipt_is_explicitly_incomplete(self):
        def opaque():
            pass
        task_id = self.finish(opaque)
        service = self.service()
        result = service.close(2, source_fenced=True)
        self.assertTrue(result["owner_closed"])
        self.assertFalse(result["complete"])
        self.assertEqual(result["blocked_count"], 1)
        self.assertEqual(result["last_error_code"], "task_receipt_unsupported")
        self.assertEqual(self.manager.get_task_status(task_id).state, "succeeded")
        self.assertEqual(self.calls, [1])

    def test_paged_scan_detects_late_old_task_change_and_source_index_replacement(self):
        tasks = [self.finish({"n": index}) for index in range(3)]
        service = self.service(paused=True, page_size=1)
        original = self.manager.page_terminal_receipts
        changed = [False]
        def page(cursor, **kwargs):
            result = original(cursor, **kwargs)
            if cursor.offset == 1 and not changed[0]:
                changed[0] = True
                self.manager.update_task_metadata(tasks[0], {"old_task_late": True})
                self.finish({"appended": True})
            return result
        with patch.object(self.manager, "page_terminal_receipts", page):
            service._terminal_owner.start()
            self.until(service, lambda: service.persistence_status()["persistence_complete_at_observed_fence"])
        self.assertTrue(service.store.get_terminal(tasks[0])["snapshot"]["task"]["metadata"]["old_task_late"])
        cursor = self.manager.begin_terminal_receipt_scan()
        self.assertFalse(hasattr(cursor, "history"))
        self.manager.cleanup_history(retain=1)  # 仅已结束的假任务；不启动生产清理。
        self.assertEqual(self.manager.page_terminal_receipts(cursor)[2], "source_index_replaced")
        service._terminal_owner.request_reconcile()
        self.until(service, lambda: service.persistence_status()["source_gap"])
        self.assertFalse(service.close(2, source_fenced=True)["complete"])
        self.assertEqual(self.calls, [1] * 4)

    def test_unchanged_large_history_scan_avoids_full_projection(self):
        task_id = self.finish({"v": 1})
        for index in range(1000):
            self.manager.publish_task_event(task_id, "task.audit", payload={"index": index})
        service = self.service()
        self.until(service, lambda: service.persistence_status()["persistence_complete_at_observed_fence"])
        with patch.object(self.runtime, "get_task_snapshot", side_effect=AssertionError("unchanged history projected")), \
                patch.object(self.manager, "capture_task_receipt", side_effect=AssertionError("unchanged history captured")):
            service._terminal_owner.request_reconcile()
            self.until(service, lambda: service.persistence_status()["persistence_complete_at_observed_fence"])
        self.assertTrue(self.current_saved(service, task_id))
        self.assertEqual(self.calls, [1])

    def test_unsupported_projection_memo_preserves_outcome_and_authorization(self):
        cycle = []
        cycle.append(cycle)
        task_id = self.finish(cycle)
        service = self.service()
        self.until(service, lambda: service.persistence_status()["blocked_count"] == 1)
        self.assertEqual(self.manager.get_task_status(task_id).state, "succeeded")
        with patch.object(self.runtime, "get_task_snapshot", side_effect=AssertionError("unchanged invalid projection rebuilt")):
            with self.assertRaises(GatewayError):
                service.get_task_result(task_id)
            service._terminal_owner.request_reconcile()
            self.until(service, lambda: not service.persistence_status()["scan_required"])
        self.allowed = False
        with self.assertRaises(GatewayError) as caught:
            service.get_task_result(task_id)
        self.assertEqual(caught.exception.error_code, "access_denied")
        self.assertEqual(self.calls, [1])

    def test_one_projection_dependency_failure_does_not_starve_other_tasks(self):
        poisoned, healthy = self.finish({"bad": True}), self.finish({"good": True})
        service = self.service(paused=True, page_size=1)
        original = service._terminal_owner._snapshot
        def fail_one(task_id):
            if task_id == poisoned:
                raise RuntimeError("offline projection dependency")
            return original(task_id)
        with patch.object(service._terminal_owner, "_snapshot", fail_one):
            service._terminal_owner.start()
            self.until(service, lambda: self.current_saved(service, healthy))
            self.until(service, lambda: service.persistence_status()["blocked_count"] == 1)
            self.assertEqual(service.persistence_status()["last_error_code"], "receipt_dependency_error")
        service._terminal_owner.request_reconcile()
        self.until(service, lambda: self.current_saved(service, poisoned))
        self.assertEqual(self.calls, [1, 1])

    def test_oversize_transport_projection_is_memoized_for_unchanged_revision(self):
        task_id = self.finish({"text": "X" * 5000})
        service = self.service(paused=True)
        full = public_feedback.build_mcp_tool_result(self.runtime.get_task_snapshot(task_id))
        encode = lambda value: len(json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8"))
        limit = (encode(full["structuredContent"]) + encode(full)) // 2
        normal_limit = public_feedback.MCP_RESULT_MAX_BYTES
        with patch("gateways.feedback.MCP_RESULT_MAX_BYTES", limit), \
                patch("gateways.agent.service.build_mcp_tool_result", wraps=public_feedback.build_mcp_tool_result) as projection:
            service._terminal_owner.start()
            self.until(service, lambda: service.persistence_status()["blocked_count"] == 1)
            self.assertEqual(service.persistence_status()["last_error_code"], "mcp_response_too_large")
            count = projection.call_count
            self.assertEqual(count, 1)
            service._terminal_owner.request_reconcile()
            self.until(service, lambda: not service.persistence_status()["scan_required"])
            with self.assertRaises(GatewayError) as caught:
                service.get_task_result(task_id)
            self.assertEqual(caught.exception.error_code, "mcp_response_too_large")
            self.assertEqual(projection.call_count, count)
            with patch("gateways.feedback.MCP_RESULT_MAX_BYTES", normal_limit):
                self.assertEqual(service.get_task_result(task_id)["result"], {"text": "X" * 5000})
            self.assertEqual(projection.call_count, count + 1)
            with self.assertRaises(GatewayError) as caught:
                service.get_task_result(task_id)
            self.assertEqual(caught.exception.error_code, "mcp_response_too_large")
            self.assertEqual(projection.call_count, count + 1)
        self.assertEqual(self.manager.get_task_status(task_id).state, "succeeded")
        self.assertEqual(self.calls, [1])

    def test_item_specific_sqlite_constraint_does_not_starve_healthy_records(self):
        poisoned, healthy = self.finish({"bad": True}), self.finish({"good": True})
        service = self.service(paused=True, page_size=1, clock=lambda: 0.0)
        with sqlite3.connect(AgentCallStore.terminal_storage_status(service.store.path)["terminal_path"]) as db:
            db.execute("CREATE TRIGGER block_one BEFORE INSERT ON agent_task_terminals WHEN NEW.task_id='" + poisoned + "' BEGIN SELECT RAISE(ABORT,'offline per-item constraint'); END")
        service._terminal_owner.start()
        self.until(service, lambda: self.current_saved(service, healthy))
        self.until(service, lambda: service.persistence_status()["blocked_count"] == 1)
        health = service.persistence_status()
        self.assertEqual(health["last_error_code"], "receipt_storage_constraint")
        self.assertIsNone(health["retry_at"])
        self.assertFalse(service.close(2, source_fenced=True)["complete"])
        with sqlite3.connect(AgentCallStore.terminal_storage_status(service.store.path)["terminal_path"]) as db:
            db.execute("DROP TRIGGER block_one")
        self.assertTrue(service.close(2, source_fenced=True)["complete"])
        self.assertTrue(self.current_saved(service, poisoned))
        self.assertEqual(self.calls, [1, 1])

    def test_policy_exclusions_are_counted_and_rechecked(self):
        self.allowed = False
        task_id = self.finish({"v": 1})
        service = self.service()
        self.until(service, lambda: service.persistence_status()["policy_excluded_count"] == 1)
        health = service.persistence_status()
        self.assertFalse(health["all_retained_terminal_receipts_complete"])
        self.assertEqual(health["coverage_scope"], "currently_agent_exposed_terminal_tasks")
        self.allowed = True
        service._terminal_owner.request_reconcile()
        self.until(service, lambda: self.current_saved(service, task_id))
        self.assertEqual(self.calls, [1])

    def test_close_fence_repeated_close_and_single_reconciliation_restart(self):
        service = self.service()
        first = service.close(2)
        self.assertTrue(first["owner_closed"])
        self.assertTrue(first["persistence_complete_at_observed_fence"])
        self.assertFalse(first["complete"])
        task_id = self.finish({"after_detach": True})
        closed = service.close(2, source_fenced=True)
        self.assertTrue(closed["complete"])
        self.assertTrue(self.current_saved(service, task_id))
        thread = service._terminal_owner._thread
        self.assertTrue(service.close(2, source_fenced=True)["complete"])
        self.assertIs(service._terminal_owner._thread, thread)
        self.assertEqual(closed["reconciliation_restarts"], 1)

    def test_prepared_write_owns_encoded_content_and_factory_identity(self):
        service = self.service(paused=True)
        snapshot = {"task_id": "offline-receipt", "result": {"value": 1}}
        prepared = service.store.prepare_terminal("offline-receipt", snapshot, source_id="offline-source", revision=1)
        snapshot["result"]["value"] = 2
        other = AgentCallStore(service.store.path, robot_identity=service.robot_identity,
                               artifact_dir=Path(service.runtime.config.gateway.artifact_dir), gateway_instance_id="other")
        for store, candidate in ((other, prepared), (service.store, replace(prepared, revision=0))):
            with self.assertRaises(GatewayError) as caught:
                store.save_prepared_terminal(candidate)
            self.assertEqual(caught.exception.error_code, "agent_terminal_identity_conflict")
        service.store.save_prepared_terminal(prepared)
        self.assertEqual(service.store.get_terminal("offline-receipt")["snapshot"]["result"], {"value": 1})

    def test_sqlite_writer_contention_keeps_intent_durable_and_does_not_replay(self):
        service = self.service(paused=True)
        service.store.begin("original", "tool", "offline_task", {})
        before = service.store.get("original")
        blocker = sqlite3.connect(str(service.store.path), isolation_level=None)
        blocker.execute("BEGIN IMMEDIATE")
        original_connect = sqlite3.connect
        def quick_connection(*args, **kwargs):
            kwargs["timeout"] = 0.01
            return original_connect(*args, **kwargs)
        try:
            with patch("core.agent_call_store.sqlite3.connect", quick_connection), self.assertRaises(sqlite3.OperationalError):
                service.store.begin("new-request", "tool", "offline_task", {})
        finally:
            blocker.rollback()
            blocker.close()
        self.assertEqual(service.store.get("original"), before)
        self.error_not_found(service, "new-request")
        self.assertEqual(self.calls, [])

    def error_not_found(self, service, request):
        with self.assertRaises(GatewayError) as caught:
            service.store.get(request)
        self.assertEqual(caught.exception.error_code, "not_found")


class AppCloseChecks(unittest.IsolatedAsyncioTestCase):
    async def exercise(self, *, owned=True, stop_error=None, close_complete=True, body_error=None, close_error=None):
        flags = []
        class FakeService:
            def close(self, *, source_fenced=False):
                flags.append(source_fenced)
                if close_error:
                    raise close_error
                return {"complete": close_complete and source_fenced, "owner_closed": True}
        class FakeUpdates:
            def __init__(self, runtime):
                pass
            def start(self):
                pass
            async def close(self):
                pass
        def stop():
            if stop_error:
                raise stop_error
        runtime = NS(start=lambda: None, stop=stop)
        with ExitStack() as stack:
            for name in ("build_http_router", "build_console_router", "build_logs_router", "build_mcp_router",
                         "build_event_router", "build_data_stream_router"):
                stack.enter_context(patch("gateways.app." + name, lambda *args: APIRouter()))
            stack.enter_context(patch("gateways.app.build_agent_router", lambda resolve: (resolve(), APIRouter())[1]))
            stack.enter_context(patch("gateways.feedback_stream.build_feedback_router", lambda *args: APIRouter()))
            stack.enter_context(patch("gateways.updates.build_update_router", lambda *args: APIRouter()))
            stack.enter_context(patch("gateways.updates.UpdateService", FakeUpdates))
            stack.enter_context(patch("gateways.app.AgentToolService", lambda *args: FakeService()))
            app = create_gateway_app(runtime, NS(gateway=GatewayServerConfig()), start_runtime_on_lifespan=owned)
            try:
                async with app.router.lifespan_context(app):
                    if body_error:
                        raise body_error
            except BaseException as error:  # noqa: BLE001 - 对比原始异常身份。
                return flags, app, error
            return flags, app, None

    async def test_successful_owned_stop_is_only_source_fence(self):
        flags, app, error = await self.exercise()
        self.assertEqual(flags, [True])
        self.assertIsNone(error)
        self.assertTrue(app.state.agent_terminal_persistence_close["complete"])
        flags, app, error = await self.exercise(owned=False)
        self.assertEqual(flags, [False])
        self.assertIsInstance(error, RuntimeError)
        self.assertFalse(app.state.agent_terminal_persistence_close["complete"])

    async def test_incomplete_close_does_not_mask_runtime_or_body_error(self):
        original = RuntimeError("original stop error")
        flags, app, error = await self.exercise(stop_error=original, close_complete=False)
        self.assertIs(error, original)
        self.assertEqual(flags, [False])
        self.assertFalse(app.state.agent_terminal_persistence_close["complete"])
        body = ValueError("original body error")
        _, _, error = await self.exercise(body_error=body, close_complete=False)
        self.assertIs(error, body)
        _, _, error = await self.exercise(close_complete=False)
        self.assertIsInstance(error, RuntimeError)
        _, app, error = await self.exercise(stop_error=original, close_error=ValueError("receipt close error"))
        self.assertIs(error, original)
        self.assertFalse(app.state.agent_terminal_persistence_close["complete"])
        self.assertIsNone(app.state.agent_terminal_persistence_close["owner_closed"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
