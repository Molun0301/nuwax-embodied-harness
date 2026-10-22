#!/usr/bin/env python3
"""Offline active-index lifecycle coherence using real task registration/transitions."""
from __future__ import annotations

import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import core.task_manager as task_module
from contracts.tasks import TaskState
from core.event_bus import EventBus
from core.resource_lock import ResourceLockManager
from core.task_manager import TaskManager, TaskSafetyBarrierError


class FailableResources(ResourceLockManager):
    failing = False

    def release(self, *args, **kwargs):
        if self.failing:
            raise RuntimeError("synthetic release failure")
        return super().release(*args, **kwargs)


class ActiveIndex(unittest.TestCase):
    def setUp(self):
        self.bus = EventBus()
        self.resources = FailableResources()
        self.manager = TaskManager(event_bus=self.bus, resource_lock_manager=self.resources, max_workers=1)
        self.gates, self.threads = [], []

    def tearDown(self):
        self.resources.failing = False
        for gate in self.gates:
            gate.set()
        for thread in self.threads:
            thread.join(3)
            self.assertFalse(thread.is_alive())
        self.manager.shutdown(wait=True)
        self.assert_coherent()
        self.resources.release_all()
        self.assertTrue(self.bus.shutdown(2))

    def assert_coherent(self):
        with self.manager._lock:
            expected = {task_id: record for task_id, record in self.manager._records.items()
                        if record.status.state not in task_module._TERMINAL_STATES}
            self.assertEqual(list(self.manager._active_records), list(expected))
            for task_id, record in expected.items():
                self.assertIs(self.manager._active_records[task_id], record)
            self.assertEqual(self.manager.count_active_tasks(), len(expected))
            self.assertEqual(self.manager.has_active_tasks(), bool(expected))
            self.assertEqual([status.task_id for status in self.manager.list_active_tasks()], list(expected))
            for status_only in (False, True):
                captured = self.manager.capture_task_list(history_limit=0, status_only=status_only)
                self.assertEqual([item.task_id if status_only else item["status"].task_id
                                  for item in captured["active_tasks"]], list(expected))

    def running(self, *, deferred=False):
        entered, release = threading.Event(), threading.Event()
        self.gates.append(release)
        def run(context):
            entered.set()
            self.assertTrue(release.wait(3))
            return {"synthetic": True}
        task = self.manager.submit("offline_task", run, deferred_interrupt=deferred,
                                   required_resources=["synthetic_motion"])
        self.assertTrue(entered.wait(3))
        return task.task_id, self.manager._records[task.task_id].future, release

    def test_registration_order_pending_cancel_and_new_submit(self):
        first, future, release = self.running()
        calls = []
        second = self.manager.submit("offline_task", lambda context: calls.append("cancelled"))
        third = self.manager.submit("offline_task", lambda context: None)
        self.assert_coherent()
        self.assertEqual(list(self.manager._active_records), [first, second.task_id, third.task_id])
        self.assertTrue(self.manager.cancel_task(second.task_id))
        fourth = self.manager.submit("offline_task", lambda context: None)
        self.assertEqual(list(self.manager._active_records), [first, third.task_id, fourth.task_id])
        self.assert_coherent()
        release.set()
        future.result(timeout=3)
        for task in (second, third, fourth):
            self.manager._records[task.task_id].future.result(timeout=3)
        self.assertEqual(calls, [])
        self.assert_coherent()
        self.assertFalse(self.manager.has_active_tasks())
        self.manager.cleanup_history(retain=0)
        self.assert_coherent()

    def test_accepted_registration_visible_before_future_and_cancelled_once(self):
        entered, release = threading.Event(), threading.Event()
        self.gates.append(release)
        original, results, errors = self.manager._executor.submit, [], []
        def gated(*args, **kwargs):
            entered.set()
            self.assertTrue(release.wait(3))
            return original(*args, **kwargs)
        def submit():
            try:
                results.append(self.manager.submit("offline_task", lambda context: None))
            except BaseException as error:  # noqa: BLE001 - Forward thread failures.
                errors.append(error)
        thread = threading.Thread(target=submit)
        self.threads.append(thread)
        with patch.object(self.manager._executor, "submit", gated):
            thread.start()
            try:
                self.assertTrue(entered.wait(3))
                task_id = next(iter(self.manager._records))
                self.assertIsNone(self.manager._records[task_id].future)
                self.assert_coherent()
                self.assertEqual(self.manager.count_active_tasks(), 1)
                self.manager.cancel_task(task_id)
                self.assert_coherent()
                self.assertFalse(self.manager.has_active_tasks())
            finally:
                release.set()
                thread.join(3)
        self.assertEqual(errors, [])
        self.manager._records[results[0].task_id].future.result(timeout=3)
        self.assert_coherent()

    def test_submit_rollback_removes_membership(self):
        with patch.object(self.manager._executor, "submit", side_effect=RuntimeError("synthetic submit failure")):
            with self.assertRaisesRegex(RuntimeError, "synthetic submit failure"):
                self.manager.submit("offline_task", lambda context: None, required_resources=["synthetic_motion"])
        self.assertEqual(self.manager._records, {})
        self.assert_coherent()

    def test_failed_rollback_retains_accepted_recovery_owner(self):
        self.resources.failing = True
        with patch.object(self.manager._executor, "submit", side_effect=RuntimeError("synthetic submit failure")):
            with self.assertLogs("nuwax_robot_bridge.task_manager", level="WARNING"):
                with self.assertRaisesRegex(RuntimeError, "synthetic submit failure"):
                    self.manager.submit("offline_task", lambda context: None, required_resources=["synthetic_motion"])
        self.assertEqual(len(self.manager._records), 1)
        record = next(iter(self.manager._records.values()))
        self.assertEqual(record.status.state, TaskState.ACCEPTED)
        self.assertFalse(record.resources_released)
        self.assert_coherent()
        self.assertEqual(self.manager.count_active_tasks(), 1)

    def test_terminal_dispatch_exception_cannot_leave_stale_membership(self):
        task_id, future, release = self.running()
        def failed_dispatch(event, **kwargs):
            self.assertEqual(event.state, TaskState.FAILED)
            self.assert_coherent()
            self.assertFalse(self.manager.has_active_tasks())
            raise RuntimeError("synthetic callback failure")
        with patch.object(self.manager, "_dispatch_task_event", failed_dispatch):
            with self.assertRaisesRegex(RuntimeError, "synthetic callback failure"):
                self.manager._set_terminal_state(task_id, TaskState.FAILED, "Synthetic failure", release_resources=False)
        self.assert_coherent()
        release.set()
        future.result(timeout=3)
        self.assert_coherent()

    def test_deferred_cancel_stays_active_until_real_terminal_failure(self):
        task_id, future, release = self.running(deferred=True)
        self.manager.cancel_task(task_id)
        self.assertEqual(self.manager.get_task_status(task_id).stage, "cancel_requested")
        self.assert_coherent()
        self.assertEqual(self.manager.count_active_tasks(), 1)
        release.set()
        with self.assertRaises(TaskSafetyBarrierError):
            future.result(timeout=3)
        self.assertEqual(self.manager.get_task_status(task_id).state, TaskState.FAILED)
        self.assertTrue(self.manager.get_task_error_payload(task_id)["details"]["parking_unconfirmed"])
        self.assert_coherent()
        self.assertFalse(self.manager.has_active_tasks())
        self.assertTrue(self.manager._records[task_id].resources_released)

    def test_timeout_removes_active_before_late_result(self):
        task_id, future, release = self.running()
        self.manager._handle_timeout(task_id)
        self.assertFalse(future.done())
        self.assertEqual(self.manager.get_task_status(task_id).state, TaskState.TIMEOUT)
        self.assert_coherent()
        self.assertFalse(self.manager.has_active_tasks())
        release.set()
        future.result(timeout=3)
        self.assert_coherent()

    def test_active_queries_never_scan_retained_records_and_copy_owned_summaries(self):
        task_id, future, release = self.running()
        self.manager.update_task_metadata(task_id, {"nested": {"values": [1]}})
        source = self.manager._records
        class NoScanRecords(dict):
            def values(self):
                raise AssertionError("active query scanned retained records")
            def items(self):
                raise AssertionError("active query scanned retained records")
            def __iter__(self):
                raise AssertionError("active query scanned retained records")
        self.manager._records = NoScanRecords(source)
        try:
            self.assertEqual(self.manager.count_active_tasks(), 1)
            self.assertTrue(self.manager.has_active_tasks())
            statuses = self.manager.list_active_tasks()
            captured = self.manager.capture_task_list(history_limit=20)
            statuses[0].message = "caller edit"
            captured["active_tasks"][0]["task"].metadata["nested"]["values"].append(2)
            self.assertNotEqual(self.manager.get_task_status(task_id).message, "caller edit")
            self.assertEqual(self.manager.get_task_spec(task_id).metadata["nested"]["values"], [1])
        finally:
            self.manager._records = source
        release.set()
        future.result(timeout=3)
        self.assert_coherent()

    def test_concurrent_completion_keeps_owned_list_snapshot(self):
        task_id, future, release_runner = self.running()
        entered, release_copy = threading.Event(), threading.Event()
        self.gates.append(release_copy)
        original, results, errors = task_module.deepcopy, [], []
        def gated_copy(value, *args, **kwargs):
            if threading.current_thread().name == "active-list-reader":
                self.assertFalse(self.manager._lock._is_owned())
                entered.set()
                self.assertTrue(release_copy.wait(3))
            return original(value, *args, **kwargs)
        def read():
            try:
                results.append(self.manager.list_active_tasks())
            except BaseException as error:  # noqa: BLE001 - Forward thread failures.
                errors.append(error)
        reader = threading.Thread(target=read, name="active-list-reader")
        self.threads.append(reader)
        with patch.object(task_module, "deepcopy", gated_copy):
            reader.start()
            try:
                self.assertTrue(entered.wait(3))
                release_runner.set()
                future.result(timeout=3)
                self.assert_coherent()
                self.assertFalse(self.manager.has_active_tasks())
            finally:
                release_copy.set()
                reader.join(3)
        self.assertEqual(errors, [])
        self.assertEqual(results[0][0].task_id, task_id)
        self.assertEqual(results[0][0].state, TaskState.RUNNING)
        results[0][0].message = "caller edit"
        self.assertNotEqual(self.manager.get_task_status(task_id).message, "caller edit")


if __name__ == "__main__":
    unittest.main(verbosity=2)
