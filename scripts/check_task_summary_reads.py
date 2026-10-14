#!/usr/bin/env python3
"""Offline bounded task-summary reads; no robot, socket, model or persistent store."""
from __future__ import annotations

import asyncio
import sys
import threading
import unittest
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
from typing import Any, Dict
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import httpx
from fastapi import FastAPI
from pydantic import TypeAdapter

import core.task_manager as task_module
from contracts.naming import build_task_id
from contracts.tasks import TaskSpec, TaskState, TaskStatus
from core.event_bus import EventBus
from core.task_manager import TaskManager, TaskRecord
from gateways.feedback import build_task_summary
from gateways.feedback_messages import build_task_feedback
from gateways.http.api import build_http_router
from gateways.runtime import GatewayRuntime


class CountedHistory(list):
    def __init__(self, values, lock):
        super().__init__(values)
        self.lock, self.visits = lock, 0

    def __iter__(self):
        for value in super().__iter__():
            assert self.lock._is_owned(), "history selection must hold the task lock"
            self.visits += 1
            yield value

    def __reversed__(self):
        for value in super().__reversed__():
            assert self.lock._is_owned(), "history selection must hold the task lock"
            self.visits += 1
            yield value


class LastOnlyEvents(list):
    def __iter__(self):
        raise AssertionError("summary must not traverse the complete event history")

    def __getitem__(self, key):
        if key != -1:
            raise AssertionError("summary must select only the last event")
        return super().__getitem__(key)


class OpaqueLabel:
    def __str__(self):
        return "synthetic opaque correlation"


class TaskSummaryReads(unittest.TestCase):
    def setUp(self):
        self.bus = EventBus()
        self.manager = TaskManager(event_bus=self.bus)
        self.runtime = NS(task_manager=self.manager, event_bus=self.bus,
                          capability_registry=NS(get_descriptor=lambda name: NS(display_name="Offline task")))
        self.gates = []

    def tearDown(self):
        for gate in self.gates:
            gate.set()
        self.manager.shutdown(wait=True)
        self.assertTrue(self.bus.shutdown(2))

    def record(self, state=TaskState.SUCCEEDED, *, metadata=None, event=True):
        task_id = build_task_id("offline_task")
        record = TaskRecord(
            spec=TaskSpec(task_id=task_id, capability_name="offline_task", metadata=metadata or {}),
            status=TaskStatus(task_id=task_id, state=state, stage=state.value, message="Synthetic status"),
        )
        with self.manager._lock:
            self.manager._records[task_id] = record
            if state in (TaskState.ACCEPTED, TaskState.RUNNING):
                self.manager._active_records[task_id] = record
            if state not in {TaskState.ACCEPTED, TaskState.RUNNING}:
                self.manager._history_order.append(task_id)
        if event:
            self.manager.publish_task_event(task_id, "task.audit", message="Synthetic feedback",
                                            payload={"nested": {"values": [1, 2]}})
        return task_id

    def old_history(self, limit):
        with self.manager._lock:
            history = [self.manager._records[task_id].status for task_id in self.manager._history_order
                       if task_id in self.manager._records]
        return deepcopy(tuple(history if limit is None else history[-limit:]))

    def old_summary(self, status):
        task_id = status.task_id
        summary = build_task_summary(status, spec=self.manager.get_task_spec(task_id),
                                     error_payload=self.manager.get_task_error_payload(task_id))
        events = self.manager.get_task_events(task_id)
        if events:
            summary["feedback"] = build_task_feedback(self.runtime, events[-1])
            summary["summary"] = summary["feedback"]["message"]
        # Match the route's Dict[str, Any] response model, including UTC Z spelling.
        return TypeAdapter(Dict[str, Any]).dump_python(summary, mode="json")

    def app(self):
        app = FastAPI()
        # Task routes are real; unrelated settings startup/configuration is inert.
        with patch("gateways.settings.SettingsApplication", return_value=NS(start=lambda: None, close=lambda: None)):
            app.include_router(build_http_router(self.runtime, NS()))
        return app

    def request(self, limit):
        async def call():
            async with httpx.AsyncClient(transport=httpx.ASGITransport(app=self.app()),
                                         base_url="http://offline.invalid") as client:
                response = await client.get("/api/tasks", params={"history_limit": limit})
                self.assertEqual(response.status_code, 200, response.text)
                return response.json()
        return asyncio.run(call())

    def test_zero_returns_empty_without_visiting_history(self):
        self.record()
        history = CountedHistory(self.manager._history_order, self.manager._lock)
        self.manager._history_order = history
        self.assertEqual(self.manager.list_history(limit=0), ())
        self.assertEqual(history.visits, 0)

    def test_positive_limit_visits_only_needed_tail(self):
        for _ in range(512):
            self.record(event=False)
        expected = self.old_history(3)
        history = CountedHistory(self.manager._history_order, self.manager._lock)
        self.manager._history_order = history
        self.assertEqual(self.manager.list_history(limit=3), expected)
        self.assertEqual(history.visits, 3)

    def test_positive_limits_keep_order_and_missing_record_semantics(self):
        ids = [self.record(event=False) for _ in range(7)]
        with self.manager._lock:
            del self.manager._records[ids[-2]]
            del self.manager._records[ids[1]]
        for limit in (1, 2, 4, 5, 6, 99, 10 ** 30):
            with self.subTest(limit=limit):
                self.assertEqual(self.manager.list_history(limit=limit), self.old_history(limit))

    def test_missing_tail_records_add_only_required_visits(self):
        ids = [self.record(event=False) for _ in range(8)]
        with self.manager._lock:
            del self.manager._records[ids[-1]]
            del self.manager._records[ids[-3]]
        expected = self.old_history(2)
        history = CountedHistory(self.manager._history_order, self.manager._lock)
        self.manager._history_order = history
        self.assertEqual(self.manager.list_history(limit=2), expected)
        self.assertEqual(history.visits, 4)

    def test_none_and_direct_negative_limits_preserve_old_slice(self):
        for _ in range(5):
            self.record(event=False)
        # Negative direct calls historically drop the first abs(limit) survivors.
        for limit in (None, -1, -3, -5, -99):
            with self.subTest(limit=limit):
                self.assertEqual(self.manager.list_history(limit=limit), self.old_history(limit))

    def test_history_status_copy_remains_independent_and_outside_lock(self):
        task_id = self.record(event=False)
        original = task_module.deepcopy
        def checked(value):
            self.assertFalse(self.manager._lock._is_owned())
            return original(value)
        with patch.object(task_module, "deepcopy", checked):
            statuses = self.manager.list_history(limit=1)
        statuses[0].message = "caller edit"
        self.assertEqual(self.manager.get_task_status(task_id).message, "Synthetic status")

    def test_latest_event_matches_full_reader_and_owns_nested_payload(self):
        task_id = self.record()
        expected = self.manager.get_task_events(task_id)[-1]
        latest = self.manager.get_latest_task_event(task_id)
        self.assertEqual(latest, expected)
        latest.payload["nested"]["values"].append(9)
        latest.message = "caller edit"
        self.assertEqual(self.manager.get_task_events(task_id)[-1], expected)

    def test_latest_event_empty_and_unknown_task(self):
        task_id = self.record(event=False)
        self.assertIsNone(self.manager.get_latest_task_event(task_id))
        with self.assertRaises(KeyError):
            self.manager.get_latest_task_event("missing")

    def test_latest_event_does_not_iterate_long_history(self):
        task_id = self.record()
        record = self.manager._records[task_id]
        last = record.events[-1]
        record.events = LastOnlyEvents([last] * 4096)
        self.assertEqual(self.manager.get_latest_task_event(task_id), last)

    def test_latest_event_copy_outside_lock_allows_concurrent_append(self):
        task_id = self.record()
        old = self.manager.get_task_events(task_id)[-1]
        entered, release, done = threading.Event(), threading.Event(), threading.Event()
        self.gates.append(release)
        original, results, errors = task_module._copy_task_value, [], []
        def gated(value, *args, **kwargs):
            if threading.current_thread().name == "summary-reader" and value is self.manager._records[task_id].events[0]:
                self.assertFalse(self.manager._lock._is_owned())
                entered.set()
                self.assertTrue(release.wait(2))
            return original(value, *args, **kwargs)
        def read():
            try:
                results.append(self.manager.get_latest_task_event(task_id))
            except BaseException as error:  # noqa: BLE001 - Forward every test-thread failure.
                errors.append(error)
            finally:
                done.set()
        reader = threading.Thread(target=read, name="summary-reader")
        with patch.object(task_module, "_copy_task_value", gated):
            reader.start()
            try:
                self.assertTrue(entered.wait(2))
                newer = self.manager.publish_task_event(task_id, "task.newer", payload={"value": 2})
                self.assertNotEqual(newer.event_id, old.event_id)
            finally:
                release.set()
                reader.join(2)
        self.assertTrue(done.is_set())
        self.assertFalse(reader.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(results, [old])
        self.assertEqual(self.manager.get_latest_task_event(task_id), newer)

    def test_opaque_context_keeps_http_event_visibility_and_acceptance_policy(self):
        task = self.manager.submit("offline_task", lambda context: None,
                                   metadata={"request_context": {"tool_name": OpaqueLabel()}})
        self.manager.wait_for_task(task.task_id, timeout=2)
        self.assertTrue(self.manager._records[task.task_id].opaque_event_context)
        self.assertIsNone(self.manager.capture_task_status_event(task.task_id)[1])
        expected = self.old_summary(self.manager.get_task_status(task.task_id))
        self.assertIn("feedback", expected)
        self.assertEqual(self.request(1)["history_tasks"], [expected])
        self.assertIsNone(self.manager.capture_task_status_event(task.task_id)[1])

    def test_http_zero_and_negative_limit_keep_active_tasks(self):
        self.record()
        active = self.record(TaskState.RUNNING)
        for limit in (0, -4):
            with self.subTest(limit=limit):
                result = self.request(limit)
                self.assertEqual(result["history_limit"], 0)
                self.assertEqual(result["history_tasks"], [])
                self.assertEqual([item["task_id"] for item in result["active_tasks"]], [active])

    def test_http_positive_limit_matches_previous_complete_summary(self):
        self.record()
        task_id = self.record(TaskState.FAILED)
        self.manager._records[task_id].error_payload = {"code": "execution_failed", "message": "synthetic failure"}
        expected = [self.old_summary(status) for status in self.old_history(1)]
        result = self.request(1)
        self.assertEqual(result["history_tasks"], expected)
        self.assertEqual(result["active_tasks"], [])

    def test_http_cap_remains_one_hundred(self):
        for _ in range(105):
            self.record(event=False)
        result = self.request(1000)
        self.assertEqual(result["history_limit"], 100)
        self.assertEqual(len(result["history_tasks"]), 100)
        self.assertEqual([item["task_id"] for item in result["history_tasks"]],
                         [status.task_id for status in self.old_history(100)])

    def test_http_summary_never_reads_complete_event_history(self):
        task_id = self.record()
        expected = self.old_summary(self.manager.get_task_status(task_id))
        last = self.manager._records[task_id].events[-1]
        self.manager._records[task_id].events = LastOnlyEvents([last] * 4096)
        with patch.object(self.manager, "get_task_events", side_effect=AssertionError("whole-history read")):
            result = self.request(1)
        self.assertEqual(result["history_tasks"], [expected])

    def test_http_empty_event_history_keeps_summary_without_feedback(self):
        task_id = self.record(event=False)
        expected = self.old_summary(self.manager.get_task_status(task_id))
        self.assertNotIn("feedback", expected)
        self.assertEqual(self.request(1)["history_tasks"], [expected])

    def test_capability_zero_and_positive_limit_use_same_contract(self):
        ids = [self.record() for _ in range(3)]
        zero = GatewayRuntime._handle_get_active_tasks(self.runtime, {"history_limit": 0})
        self.assertEqual(zero["history_tasks"], ())
        positive = GatewayRuntime._handle_get_active_tasks(self.runtime, {"history_limit": 2})
        self.assertEqual([status.task_id for status in positive["history_tasks"]], ids[-2:])

    def test_active_scalar_queries_match_all_states_and_quarantine(self):
        active_ids = []
        for state in TaskState:
            task_id = self.record(state, event=False)
            self.manager._records[task_id].resource_quarantined = True
            if state in (TaskState.ACCEPTED, TaskState.RUNNING):
                active_ids.append(task_id)
        expected = self.manager.list_active_tasks()
        with patch.object(task_module, "deepcopy", side_effect=AssertionError("scalar query copied statuses")):
            with patch.object(task_module, "_copy_task_value", side_effect=AssertionError("scalar query copied payload")):
                self.assertEqual(self.manager.count_active_tasks(), len(expected))
                self.assertTrue(self.manager.has_active_tasks())
        self.assertEqual([status.task_id for status in self.manager.list_active_tasks()], active_ids)
        self.assertEqual(len(self.manager.list_quarantined_tasks()), len(TaskState))
        expected[0].message = "caller edit"
        self.assertEqual(self.manager.get_task_status(active_ids[0]).message, "Synthetic status")

    def test_active_scalar_queries_are_locked_and_do_not_iterate(self):
        self.record(TaskState.RUNNING, event=False)
        self.record(event=False)
        manager = self.manager
        class CountedRecords(dict):
            visits = 0
            def __len__(self):
                assert manager._lock._is_owned(), "scalar selection must hold the task lock"
                return super().__len__()
            def values(self):
                raise AssertionError("scalar query must not iterate")
        records = CountedRecords(manager._active_records)
        manager._active_records = records
        self.assertTrue(manager.has_active_tasks())
        self.assertEqual(records.visits, 0)
        self.assertEqual(manager.count_active_tasks(), 1)
        self.assertEqual(records.visits, 0)

    def test_active_scalar_queries_follow_real_transitions_and_terminal_quarantine(self):
        task_id = self.record(TaskState.ACCEPTED, event=False)
        self.assertEqual(self.manager.count_active_tasks(), 1)
        self.manager._set_running(task_id)
        self.assertTrue(self.manager.has_active_tasks())
        self.manager._set_terminal_state(task_id, TaskState.CANCELLED, "Synthetic cancellation",
                                         release_resources=False)
        self.manager._records[task_id].resource_quarantined = True
        self.assertEqual(self.manager.count_active_tasks(), 0)
        self.assertFalse(self.manager.has_active_tasks())
        self.assertEqual([status.task_id for status in self.manager.list_quarantined_tasks()], [task_id])
        self.assertEqual(self.manager.list_active_tasks(), ())
        self.assertEqual(self.manager.capture_task_list(history_limit=0)["active_tasks"], [])

    def test_active_scalar_queries_observe_locked_transition(self):
        task_id = self.record(TaskState.ACCEPTED, event=False)
        started, done = threading.Event(), threading.Event()
        results, errors = [], []
        def read():
            started.set()
            try:
                results.append((self.manager.count_active_tasks(), self.manager.has_active_tasks()))
            except BaseException as error:  # noqa: BLE001 - Forward test-thread failures.
                errors.append(error)
            finally:
                done.set()
        reader = threading.Thread(target=read, name="active-scalar-reader")
        try:
            with self.manager._lock:
                reader.start()
                self.assertTrue(started.wait(2))
                self.assertFalse(done.wait(0.02))
                self.manager._set_terminal_state(task_id, TaskState.SUCCEEDED, "Synthetic completion")
        finally:
            reader.join(2)
        self.assertFalse(reader.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(results, [(0, False)])

    def test_active_snapshot_selects_index_under_lock(self):
        active = self.record(TaskState.RUNNING, event=False)
        self.record(event=False)
        source = self.manager._active_records
        manager = self.manager
        class LockedRecords(dict):
            def values(self):
                assert manager._lock._is_owned(), "active filtering must hold the task lock"
                return super().values()
        self.manager._active_records = LockedRecords(source)
        self.assertEqual([status.task_id for status in self.manager.list_active_tasks()], [active])
        self.manager._set_terminal_state(active, TaskState.SUCCEEDED, "Synthetic completion")
        self.assertEqual(self.manager.list_active_tasks(), ())


if __name__ == "__main__":
    unittest.main(verbosity=2)
