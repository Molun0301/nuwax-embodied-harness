#!/usr/bin/env python3
"""Offline explicit task cleanup ownership checks; no devices or automatic eviction."""
from __future__ import annotations

import sys
import threading
import unittest
from copy import copy
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contracts.tasks import TaskState
from core.event_bus import EventBus
from core.resource_lock import ResourceLockManager
from core.task_manager import TaskManager, TaskSafetyBarrierError


class FailableResources(ResourceLockManager):
    failing = False

    def release(self, *args, **kwargs):
        if self.failing:
            raise RuntimeError("synthetic resource release failure")
        return super().release(*args, **kwargs)


class CleanupOwnership(unittest.TestCase):
    def setUp(self):
        self.bus = EventBus()
        self.locks = FailableResources()
        self.manager = TaskManager(event_bus=self.bus, resource_lock_manager=self.locks, max_workers=1)
        self.gates, self.threads = [], []

    def tearDown(self):
        self.locks.failing = False
        for event in self.gates:
            event.set()
        for thread in self.threads:
            thread.join(3)
            self.assertFalse(thread.is_alive())
        self.manager.shutdown(wait=True)
        # Release only synthetic test-owned leases, including baseline leak witnesses.
        self.locks.release_all()
        self.assertTrue(self.bus.shutdown(2))

    def gate(self):
        entered, release = threading.Event(), threading.Event()
        self.gates.append(release)
        return entered, release

    def require(self, event):
        self.assertTrue(event.wait(3), "synthetic gate deadline")

    def spawn(self, function):
        errors = []
        def call():
            try:
                function()
            except BaseException as error:  # noqa: BLE001 - Forward every test-thread failure.
                errors.append(error)
        thread = threading.Thread(target=call, name="cleanup-interrupt")
        self.threads.append(thread)
        thread.start()
        return thread, errors

    def gated_task(self, *, deferred=False, acknowledge=False):
        entered, release = self.gate()
        calls = []
        def runner(context):
            calls.append(1)
            entered.set()
            self.require(release)
            if acknowledge:
                context.acknowledge_interrupt_barrier({"synthetic_stop": True})
            return {"late_result": True}
        task = self.manager.submit("offline_task", runner, input_payload={"goal_id": "synthetic-goal"},
                                   required_resources=["synthetic_resource"], deferred_interrupt=deferred)
        self.require(entered)
        record = self.manager._records[task.task_id]
        return task.task_id, record.future, release, calls

    def interrupt(self, task_id, timeout):
        if timeout:
            self.manager._handle_timeout(task_id)
        else:
            self.assertTrue(self.manager.cancel_task(task_id))

    def finish(self):
        task = self.manager.submit("offline_task", lambda context: {"finished": True})
        future = self.manager._records[task.task_id].future
        self.assertEqual(future.result(timeout=3), {"finished": True})
        return task.task_id

    def check_running_interrupt(self, timeout):
        task_id, future, release, calls = self.gated_task()
        self.interrupt(task_id, timeout)
        state = TaskState.TIMEOUT if timeout else TaskState.CANCELLED
        self.assertEqual(self.manager.get_task_status(task_id).state, state)
        self.assertFalse(future.done())
        cursor = self.manager.begin_terminal_receipt_scan()
        self.manager.cleanup_history(retain=0)
        self.assertIn(task_id, self.manager._records)
        self.assertIsNone(self.manager.page_terminal_receipts(cursor)[2])
        self.assertEqual(self.locks.held_resources(task_id), ("synthetic_resource",))
        release.set()
        self.assertEqual(future.result(timeout=3), {"late_result": True})
        self.assertEqual(calls, [1])
        self.assertEqual(self.manager.get_task_status(task_id).state, state)
        self.assertEqual(self.manager.get_task_result(task_id), {"late_result": True})
        self.assertEqual(self.manager.get_latest_task_event(task_id).event_type, "task.result_returned")
        self.assertEqual(self.locks.held_resources(task_id), ())
        self.assertTrue(self.manager._records[task_id].resources_released)
        self.manager.cleanup_history(retain=0)
        self.assertNotIn(task_id, self.manager._records)

    def test_running_cancel_retains_runner_and_late_receipt(self):
        self.check_running_interrupt(False)

    def test_running_timeout_retains_runner_and_late_receipt(self):
        self.check_running_interrupt(True)

    def test_completed_future_with_failed_release_retains_recovery_owner(self):
        self.locks.failing = True
        with self.assertLogs("nuwax_robot_bridge.task_manager", level="WARNING"):
            task = self.manager.submit("offline_task", lambda context: None,
                                       required_resources=["synthetic_resource"])
            future = self.manager._records[task.task_id].future
            future.result(timeout=3)
        self.assertFalse(self.manager._records[task.task_id].resources_released)
        self.manager.cleanup_history(retain=0)
        self.assertIn(task.task_id, self.manager._records)
        self.assertEqual(self.locks.held_resources(task.task_id), ("synthetic_resource",))
        self.locks.failing = False
        self.assertTrue(self.manager._release_resources(task.task_id))
        self.manager.cleanup_history(retain=0)
        self.assertNotIn(task.task_id, self.manager._records)

    def test_terminal_callback_cannot_remove_running_future(self):
        seen = []
        def cleanup(event):
            task_id = event.task_id
            self.manager.cleanup_history(retain=0)
            seen.append(task_id in self.manager._records)
        self.bus.subscribe(cleanup, event_types=("task.succeeded",))
        task_id = self.finish()
        self.assertEqual(seen, [True])
        self.manager.cleanup_history(retain=0)
        self.assertNotIn(task_id, self.manager._records)

    def test_queued_cancel_waits_for_future_without_executing_action(self):
        first, first_future, release, _ = self.gated_task()
        calls = []
        queued = self.manager.submit("offline_task", lambda context: calls.append(1))
        future = self.manager._records[queued.task_id].future
        self.manager.cancel_task(queued.task_id)
        self.assertTrue(self.manager._records[queued.task_id].resources_released)
        self.assertFalse(future.done())
        self.manager.cleanup_history(retain=0)
        self.assertIn(queued.task_id, self.manager._records)
        release.set()
        first_future.result(timeout=3)
        future.result(timeout=3)
        self.assertEqual(calls, [])
        self.manager.cleanup_history(retain=0)
        self.assertNotIn(queued.task_id, self.manager._records)
        self.assertNotIn(first, self.manager._records)

    def test_submit_registration_window_keeps_record_without_future(self):
        entered, resume = self.gate()
        original = self.manager._executor.submit
        results, calls = [], []
        def gated(*args, **kwargs):
            entered.set()
            self.require(resume)
            return original(*args, **kwargs)
        with patch.object(self.manager._executor, "submit", gated):
            thread, errors = self.spawn(lambda: results.append(self.manager.submit(
                "offline_task", lambda context: calls.append(1))))
            self.require(entered)
            task_id = next(iter(self.manager._records))
            self.assertIsNone(self.manager._records[task_id].future)
            self.manager.cancel_task(task_id)
            self.manager.cleanup_history(retain=0)
            self.assertIn(task_id, self.manager._records)
            resume.set()
            thread.join(3)
        self.assertEqual(errors, [])
        self.assertEqual(results[0].task_id, task_id)
        self.manager._records[task_id].future.result(timeout=3)
        self.assertEqual(calls, [])
        self.manager.cleanup_history(retain=0)
        self.assertNotIn(task_id, self.manager._records)

    def check_deferred(self, timeout, acknowledge):
        task_id, future, release, calls = self.gated_task(deferred=True, acknowledge=acknowledge)
        self.interrupt(task_id, timeout)
        self.manager.cleanup_history(retain=0)
        self.assertIn(task_id, self.manager._records)
        release.set()
        if acknowledge:
            future.result(timeout=3)
            self.assertEqual(self.manager.get_task_status(task_id).state,
                             TaskState.TIMEOUT if timeout else TaskState.CANCELLED)
            self.manager.cleanup_history(retain=0)
            self.assertNotIn(task_id, self.manager._records)
        else:
            with self.assertRaises(TaskSafetyBarrierError):
                future.result(timeout=3)
            self.assertTrue(self.manager.get_task_error_payload(task_id)["details"]["parking_unconfirmed"])
            self.assertEqual(self.locks.held_resources(task_id), ())
            cursor = self.manager.begin_terminal_receipt_scan()
            self.manager.cleanup_history(retain=0)
            self.assertNotIn(task_id, self.manager._records)
            self.assertEqual(self.manager.page_terminal_receipts(cursor)[2], "source_index_replaced")
        self.assertEqual(calls, [1])

    def test_deferred_cancel_acknowledgement_and_parking_failure(self):
        self.check_deferred(False, True)
        self.check_deferred(False, False)

    def test_deferred_timeout_acknowledgement_and_parking_failure(self):
        self.check_deferred(True, True)
        self.check_deferred(True, False)

    def check_late_transition(self, timeout, deferred):
        task_id, future, release, calls = self.gated_task(deferred=deferred, acknowledge=deferred)
        entered, resume = self.gate()
        method = "_set_interrupt_requested" if deferred else "_set_terminal_state"
        original = getattr(self.manager, method)
        def gated(*args, **kwargs):
            if threading.current_thread().name == "cleanup-interrupt":
                entered.set()
                self.require(resume)
            return original(*args, **kwargs)
        with patch.object(self.manager, method, gated):
            thread, errors = self.spawn(lambda: self.interrupt(task_id, timeout))
            self.require(entered)
            release.set()
            future.result(timeout=3)
            record = self.manager._records[task_id]
            self.assertTrue(record.resources_released)
            self.manager.cleanup_history(retain=0)
            self.assertNotIn(task_id, self.manager._records)
            resume.set()
            thread.join(3)
        self.assertEqual(errors, [])
        self.assertEqual(calls, [1])

    def test_late_cancel_transition_after_completed_record_eviction(self):
        self.check_late_transition(False, False)

    def test_late_timeout_transition_after_completed_record_eviction(self):
        self.check_late_transition(True, False)

    def test_late_deferred_cancel_transition_after_completed_record_eviction(self):
        self.check_late_transition(False, True)

    def test_late_deferred_timeout_transition_after_completed_record_eviction(self):
        self.check_late_transition(True, True)

    def transition(self, method, task_id, **kwargs):
        if method == "_set_terminal_state":
            return getattr(self.manager, method)(task_id, TaskState.CANCELLED, "synthetic cancel", **kwargs)
        return getattr(self.manager, method)(task_id, stage="cancel_requested", message="synthetic cancel",
                                             event_type="task.cancel_requested", **kwargs)

    def test_unexpected_missing_record_transitions_still_raise(self):
        for method in ("_set_terminal_state", "_set_interrupt_requested"):
            with self.subTest(method=method), self.assertRaises(KeyError):
                self.transition(method, "missing")

    def test_captured_unfinished_record_does_not_mask_missing_invariant(self):
        task_id, future, release, _ = self.gated_task()
        record = self.manager._records.pop(task_id)
        try:
            for method in ("_set_terminal_state", "_set_interrupt_requested"):
                with self.subTest(method=method), self.assertRaises(KeyError):
                    self.transition(method, task_id, interrupt_record=record)
        finally:
            self.manager._records[task_id] = record
            release.set()
            future.result(timeout=3)

    def test_equal_id_replacement_is_not_the_captured_record(self):
        task_id = self.finish()
        captured = self.manager._records[task_id]
        replacement = copy(captured)
        self.manager._records[task_id] = replacement
        for method in ("_set_terminal_state", "_set_interrupt_requested"):
            with self.subTest(method=method), self.assertRaises(KeyError):
                self.transition(method, task_id, interrupt_record=captured)
        self.assertIs(self.manager._records[task_id], replacement)

    def test_wrong_task_captured_record_does_not_mask_missing_invariant(self):
        task_id = self.finish()
        record = self.manager._records[task_id]
        for method in ("_set_terminal_state", "_set_interrupt_requested"):
            with self.subTest(method=method), self.assertRaises(KeyError):
                self.transition(method, "missing", interrupt_record=record)

    def test_orphan_history_index_does_not_mask_missing_invariant(self):
        task_id = self.finish()
        record = self.manager._records.pop(task_id)
        try:
            self.assertIn(task_id, self.manager._history_order)
            for method in ("_set_terminal_state", "_set_interrupt_requested"):
                with self.subTest(method=method), self.assertRaises(KeyError):
                    self.transition(method, task_id, interrupt_record=record)
        finally:
            self.manager._records[task_id] = record

    def test_public_missing_task_errors_unchanged(self):
        for action in (self.manager.cancel_task, self.manager.get_task_status, self.manager.wait_for_task):
            with self.subTest(action=action.__name__), self.assertRaises(KeyError):
                action("missing")

    def test_actual_eviction_preserves_existing_receipt_scan_invalidation(self):
        ids = [self.finish() for _ in range(3)]
        cursor = self.manager.begin_terminal_receipt_scan()
        self.manager.cleanup_history(retain=1)
        self.assertEqual([status.task_id for status in self.manager.list_history()], ids[-1:])
        self.assertEqual(self.manager.page_terminal_receipts(cursor)[2], "source_index_replaced")

    def test_default_retention_and_noop_identity_remain_explicit(self):
        ids = [self.finish() for _ in range(3)]
        self.manager._history_retention = 2
        self.assertEqual(len(self.manager.list_history()), 3)  # No new automatic eviction.
        self.manager.cleanup_history()
        self.assertEqual([status.task_id for status in self.manager.list_history()], ids[-2:])
        index = self.manager._history_order
        self.manager.cleanup_history(retain=2)
        self.assertIs(self.manager._history_order, index)


if __name__ == "__main__":
    unittest.main(verbosity=2)
