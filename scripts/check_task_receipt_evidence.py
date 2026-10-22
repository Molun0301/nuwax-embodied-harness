#!/usr/bin/env python3
"""离线检查版本化任务返回证据；不调用设备、网络或模型。"""
from __future__ import annotations

import sys
import tempfile
import threading
import unittest
from datetime import datetime, timedelta, timezone
from decimal import Decimal
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


from contracts.capabilities import CapabilityExecutionMode
from contracts.tasks import TaskSpec, TaskState, TaskStatus
from core.event_bus import EventBus
from core.resource_lock import ResourceLockManager
from core.task_manager import TaskManager, TaskSafetyBarrierError
from gateways.runtime import GatewayRuntime
from tests.conftest import create_isolated_gateway


class ReceiptEvidenceChecks(unittest.TestCase):
    def setUp(self):
        self.bus = EventBus()
        self.locks = ResourceLockManager()
        self.manager = TaskManager(event_bus=self.bus, resource_lock_manager=self.locks)
        self.releases = []
        self.temp = tempfile.TemporaryDirectory(prefix="neh-task-receipts-")
        self.addCleanup(self.temp.cleanup)
        self.runtime_count = 0

    def tearDown(self):
        for release in self.releases:
            release.set()
        self.manager.shutdown(wait=True)
        self.assertTrue(self.bus.shutdown(2))

    def gate(self):
        entered, release = threading.Event(), threading.Event()
        self.releases.append(release)
        return entered, release

    def require(self, event):
        self.assertTrue(event.wait(2), "离线门闩超时")

    def runtime(self):
        runtime = create_isolated_gateway(Path(self.temp.name) / str(self.runtime_count),
                                          task_manager=self.manager, event_bus=self.bus)
        self.runtime_count += 1
        self.addCleanup(runtime.stop)
        def descriptor(name):
            self.assertFalse(self.manager._lock._is_owned())
            return NS(display_name="标签 A")
        runtime.capability_registry = NS(get_descriptor=descriptor)
        return runtime

    def finish(self, result=None):
        task = self.manager.submit("activate_map", lambda context: result)
        self.manager.wait_for_task(task.task_id, timeout=2)
        return task.task_id

    def late(self, interrupt):
        entered, release = self.gate()
        calls, returned = [], {"receipt": {"value": 7}}
        def runner(context):
            calls.append(1)
            entered.set()
            self.require(release)
            return returned
        task_id = self.manager.submit("activate_map", runner, required_resources=["map_workspace"]).task_id
        self.require(entered)
        if interrupt == "cancelled":
            self.manager.cancel_task(task_id, "原始取消原因")
        else:
            self.manager._handle_timeout(task_id)
        before = self.manager.capture_task_receipt(task_id).snapshot()
        self.assertEqual(self.locks.held_resources(task_id), ("map_workspace",))
        release.set()
        result = self.manager.wait_for_task(task_id, timeout=2)
        after = self.manager.capture_task_receipt(task_id).snapshot()
        self.assertEqual(result, returned)
        self.assertIsNot(result, returned)
        self.assertEqual(calls, [1])
        self.assertEqual(after["result"], returned)
        self.assertEqual(after["status"], before["status"])
        self.assertEqual(after["error_payload"], before["error_payload"])
        self.assertEqual(after["status"].state, interrupt)
        self.assertEqual(after["events"][-1].event_type, "task.result_returned")
        self.assertEqual(after["events"][-1].payload["evidence_kind"], "runner_returned")
        self.assertFalse(self.manager._records[task_id].completion_committed)
        self.assertEqual(self.locks.held_resources(task_id), ())

    def test_submit_snapshot_failure_happens_before_runner_starts(self):
        calls = []
        with patch("core.task_manager._snapshot", side_effect=RuntimeError("读取副本失败")):
            with self.assertRaisesRegex(RuntimeError, "读取副本失败"):
                self.manager.submit("offline_task", lambda context: calls.append(1), required_resources=["map_workspace"])
        self.manager.shutdown(wait=True)
        self.assertEqual(calls, [])
        self.assertEqual(self.manager.count_active_tasks(), 0)
        self.assertEqual(self.manager._records, {})

    def test_late_cancel_return_is_evidence(self):
        self.late("cancelled")

    def test_late_timeout_return_is_evidence(self):
        self.late("timeout")


    def test_started_public_context_tracks_metadata_update_after_acceptance(self):
        seen = []
        def callback(event):
            if event.event_type == "task.accepted":
                self.manager.update_task_metadata(event.task_id, {"request_context": {"request_id": "new-request"}})
            elif event.event_type == "task.started":
                seen.append(event.payload["request_context"]["request_id"])
        self.bus.subscribe(callback, event_types=("task.accepted", "task.started"))
        self.finish({"known": True})
        self.assertEqual(seen, ["new-request"])


    def test_restored_map_workspace_datetime_supports_task_receipt(self):
        from contracts.map_workspace import MapAsset, MapWorkspace
        from pydantic_core import TzInfo
        # 地图持久化后再次激活，契约解析器会产生其自身的固定偏移时区。
        original = MapWorkspace(active_map_name="住宅", map_asset=MapAsset(
            map_name="住宅", display_name="住宅", bound_memory_library_name="住宅",
            created_at=datetime(2026, 10, 14, 12, tzinfo=timezone(timedelta(hours=8))),
            metadata={"观察": [1]}))
        restored = MapWorkspace.model_validate_json(original.model_dump_json())
        self.assertIs(type(restored.map_asset.created_at.tzinfo), TzInfo)
        task_id = self.finish({"active_workspace": restored})
        capture = self.manager.capture_task_receipt(task_id)
        copied = capture.snapshot()["result"]["active_workspace"]
        self.assertEqual(copied.model_dump(mode="json"), restored.model_dump(mode="json"))
        copied.map_asset.metadata["观察"].append(2)
        self.assertEqual(capture.snapshot()["result"]["active_workspace"].map_asset.metadata["观察"], [1])

    def test_tuple_list_cycle_does_not_share_original_metadata(self):
        child = []
        original = (child,)
        child.append(original)
        task = self.manager.submit("offline_task", lambda context: None, metadata={"cycle": original})
        self.manager.wait_for_task(task.task_id, timeout=2)
        capture = self.manager.capture_task_receipt(task.task_id)
        copy = capture.snapshot()["task"].metadata["cycle"]
        self.assertEqual(len(copy[0]), 1)
        self.assertEqual(len(copy[0][0][0]), 1)
        copy[0].append("capture mutation")
        self.assertEqual(len(capture.snapshot()["task"].metadata["cycle"][0][0][0]), 1)


    def test_real_timeout_timer_late_return_once(self):
        entered, release = self.gate()
        timed_out = threading.Event()
        self.bus.subscribe(lambda event: timed_out.set(), event_types=("task.timeout",))
        calls = []
        def runner(context):
            calls.append(1)
            entered.set()
            self.require(release)
            return {"returned": True}
        task_id = self.manager.submit("activate_map", runner, timeout_sec=0.05).task_id
        self.require(entered)
        self.require(timed_out)
        before = self.manager.get_task_status(task_id)
        release.set()
        self.manager.wait_for_task(task_id, timeout=2)
        self.assertEqual(calls, [1])
        self.assertEqual(self.manager.get_task_status(task_id), before)
        self.assertEqual(self.manager.get_task_result(task_id), {"returned": True})

    def test_cancel_between_return_and_terminal_transition(self):
        entered, release = self.gate()
        calls = []
        original = self.manager._set_terminal_state
        def delayed(task_id, state, *args, **kwargs):
            if state == "succeeded":
                entered.set()
                self.require(release)
            return original(task_id, state, *args, **kwargs)
        with patch.object(self.manager, "_set_terminal_state", delayed):
            task_id = self.manager.submit("activate_map", lambda context: calls.append(1) or {"known": True}).task_id
            self.require(entered)
            self.manager.cancel_task(task_id, "转换窗口取消")
            before = self.manager.get_task_status(task_id)
            release.set()
            self.manager.wait_for_task(task_id, timeout=2)
        self.assertEqual(calls, [1])
        self.assertEqual(self.manager.get_task_status(task_id), before)
        self.assertEqual(self.manager.get_task_result(task_id), {"known": True})

    def test_result_and_capture_mutation_do_not_change_source(self):
        result = {"nested": [1, {"key": "source"}]}
        task_id = self.finish(result)
        capture = self.manager.capture_task_receipt(task_id)
        baseline = capture.snapshot()
        self.manager.get_task_result(task_id)["nested"][1]["key"] = "reader-alias"
        self.manager.wait_for_task(task_id, timeout=2)["nested"][1]["key"] = "waiter-alias"
        capture.snapshot()["result"]["nested"][1]["key"] = "capture-alias"
        capture.task_spec().metadata["caller"] = "mutated"
        self.assertEqual(capture.snapshot(), baseline)
        self.assertIs(self.manager.capture_task_receipt(task_id), capture)
        self.assertEqual(self.manager.get_task_result(task_id)["nested"][1]["key"], "source")

    def test_metadata_and_event_are_one_revision_owned(self):
        task_id = self.finish({"ok": True})
        old = self.manager.capture_task_receipt(task_id)
        updates = {"map_asset_finalize": {"status": "succeeded", "parts": [1]}}
        returned = self.manager.update_task_metadata(task_id, updates)
        returned.metadata["map_asset_finalize"]["status"] = "caller-mutated"
        new = self.manager.capture_task_receipt(task_id)
        snapshot = new.snapshot()
        self.assertEqual(new.revision, old.revision + 1)
        self.assertEqual(snapshot["task"].metadata["map_asset_finalize"]["status"], "succeeded")
        self.assertEqual(snapshot["events"][-1].event_type, "task.metadata_updated")
        self.assertEqual(snapshot["events"][-1].payload["metadata_updates"]["map_asset_finalize"]["status"], "succeeded")
        self.assertNotIn("map_asset_finalize", old.snapshot()["task"].metadata)
        self.assertEqual(snapshot["status"].updated_at, old.snapshot()["status"].updated_at)

    def test_event_input_and_return_aliases_are_detached(self):
        task_id = self.finish()
        payload = {"nested": {"value": [1]}}
        event = self.manager.publish_task_event(task_id, "task.audit", payload=payload)
        event.payload["nested"]["value"] = [8]
        snapshot = self.manager.capture_task_receipt(task_id).snapshot()
        self.assertEqual(tuple(snapshot["events"][-1].payload["nested"]["value"]), (1,))

    def test_source_changes_while_copying_capture(self):
        from core.task_manager import _snapshot
        entered, release = self.gate()
        task_id = self.finish({"known": True})
        capture = self.manager.capture_task_receipt(task_id)
        outputs = []
        def copy_gate(value, *args, **kwargs):
            self.assertFalse(self.manager._lock._is_owned())
            if threading.current_thread() is not threading.main_thread() and isinstance(value, dict) and value.get("task_id") == task_id:
                entered.set()
                self.require(release)
            return _snapshot(value, *args, **kwargs)
        with patch("core.task_manager._snapshot", copy_gate):
            thread = threading.Thread(target=lambda: outputs.append(capture.snapshot()))
            thread.start()
            self.require(entered)
            self.manager.update_task_metadata(task_id, {"later": True})
            release.set()
            thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertNotIn("later", outputs[0]["task"].metadata)
        self.assertGreater(self.manager.capture_task_receipt(task_id).revision, capture.revision)


    def test_cyclic_metadata_does_not_reject_action(self):
        metadata = {}
        metadata["cycle"] = metadata
        calls = []
        task_id = self.manager.submit("activate_map", lambda context: calls.append(1), metadata=metadata).task_id
        self.manager.wait_for_task(task_id, timeout=2)
        self.assertEqual(calls, [1])
        self.assertEqual(self.manager.get_task_status(task_id).state, "succeeded")

    def test_fast_async_acceptance_never_copies_result_or_loses_task_id(self):
        for project_result in (False, True):
            runtime = self.runtime()
            runtime._refresh_dynamic_capability_matrix = lambda **kwargs: None
            runtime._requires_motion_admission = lambda descriptor: False
            runtime._ensure_geometry_runtime_for_invoke = lambda name: None
            calls = []
            def handler(payload, *, requested_by, request_context):
                task = self.manager.submit("offline_task", lambda context: calls.append(1) or {"values": list(range(2048))})
                self.manager._records[task.task_id].future.result(timeout=2)
                return task
            descriptor = NS(name="offline_task", display_name="离线任务", execution_mode=CapabilityExecutionMode.ASYNC_TASK, input_schema={"type": "object"})
            runtime.capability_registry = NS(get_registration=lambda name: NS(descriptor=descriptor, handler=handler),
                                             get_descriptor=lambda name: descriptor, assert_supported=lambda *a, **kw: None)
            with patch.object(self.manager, "capture_task_receipt", side_effect=AssertionError("受理不能要求完整回执")):
                response = runtime.invoke_capability("offline_task", requested_by="local", project_result=project_result)
            self.assertEqual(calls, [1])
            accepted_id = response["task"].get("task_id") if isinstance(response["task"], dict) else response["task"].task_id
            self.assertEqual(response["task_id"], accepted_id)
            self.assertEqual(response["status"].state, "succeeded")

    def test_slim_acceptance_encodes_only_retained_fields(self):
        unused = []
        unused.append(unused)
        task_id = "task_offline_task_20261002t000000z_12345678"
        spec = TaskSpec(task_id=task_id, capability_name="offline_task",
                        input_payload={"map_name": "offline", "max_duration_sec": Decimal("2.5"), "unused": unused},
                        metadata={"unused": unused, "polling_guidance": {"hint": "wait", "delay": Decimal("1.25")}})
        status = TaskStatus(task_id=task_id, state=TaskState.ACCEPTED)
        actual = GatewayRuntime._slim_task_acceptance(spec, status)
        self.assertEqual(actual["task_id"], task_id)
        self.assertEqual(actual["input_payload"], {"map_name": "offline", "max_duration_sec": "2.5"})
        self.assertEqual(actual["polling_guidance"], {"hint": "wait", "delay": "1.25"})
        self.assertEqual(actual["status"]["updated_at"], status.model_dump(mode="json")["updated_at"])
        self.assertEqual(set(actual), {"task_id", "capability_name", "input_payload", "status", "polling_guidance", "next_step_hint"})

    def test_deferred_interrupt_without_proof_fails_and_releases(self):
        entered, release = self.gate()
        calls = []
        def runner(context):
            calls.append(1)
            entered.set()
            self.require(release)
            return {"returned": True}
        task_id = self.manager.submit("activate_map", runner, required_resources=["map_workspace"], deferred_interrupt=True).task_id
        self.require(entered)
        self.manager.cancel_task(task_id)
        release.set()
        with self.assertRaises(TaskSafetyBarrierError):
            self.manager.wait_for_task(task_id, timeout=2)
        self.assertEqual(calls, [1])
        self.assertEqual(self.manager.get_task_status(task_id).state, "failed")
        self.assertTrue(self.manager.get_task_error_payload(task_id)["details"]["parking_unconfirmed"])
        self.assertEqual(self.locks.held_resources(task_id), ())

    def test_parking_failure_receipt_preserves_original_correlation(self):
        entered, release = self.gate()
        def runner(context):
            entered.set()
            self.require(release)
            return {"returned": True}
        task_id = self.manager.submit("activate_map", runner, input_payload={"goal_id": "offline-goal"},
                                      required_resources=["map_workspace"], deferred_interrupt=True).task_id
        self.require(entered)
        self.manager.cancel_task(task_id)
        release.set()
        with self.assertRaises(TaskSafetyBarrierError):
            self.manager.wait_for_task(task_id, timeout=2)
        error = self.manager.get_task_error_payload(task_id)
        self.assertEqual(error["code"], "parking_unconfirmed")
        self.assertEqual(error["details"]["goal_id"], "offline-goal")
        self.assertEqual(self.locks.held_resources(task_id), ())
        self.assertEqual(self.manager.capture_task_receipt(task_id).snapshot()["events"][-1].event_type,
                         "task.failed")

    def test_committed_success_and_valid_interrupt_proofs_unchanged(self):
        for commit in (False, True):
            entered, release = self.gate()
            def runner(context, commit=commit, entered=entered, release=release):
                if commit:
                    self.assertTrue(context.commit_success({"domain_proof": "offline-success"}))
                entered.set()
                self.require(release)
                if not commit:
                    context.acknowledge_interrupt_barrier({"stop_proof": "offline-stop"})
                return {"returned": True}
            task_id = self.manager.submit("activate_map", runner, deferred_interrupt=True).task_id
            self.require(entered)
            self.assertEqual(self.manager.cancel_task(task_id), not commit)
            release.set()
            self.manager.wait_for_task(task_id, timeout=2)
            self.assertEqual(self.manager.get_task_status(task_id).state, "succeeded" if commit else "cancelled")
            self.assertTrue(self.manager._records[task_id].resources_released)


    def test_snapshot_metadata_selected_copies_and_revision_isolation(self):
        task_id = self.finish({"finalize_pending": True, "nested": [3]})
        self.manager.update_task_metadata(task_id, {
            "exploration_checkpoint": {"stage": "saved", "nested": {"values": [1]}},
            "map_asset_finalize": {"committed": True, "nested": {"values": [2]}},
            "unrelated": {"values": [4]},
        })
        runtime = self.runtime()
        capture = self.manager.capture_task_receipt(task_id)
        expected = runtime.get_task_snapshot(task_id, captured_task=capture)
        actual = runtime.get_task_snapshot(task_id, captured_task=capture)
        metadata = actual["task"].metadata
        self.assertIsNot(actual["exploration_checkpoint"], metadata["exploration_checkpoint"])
        self.assertIsNot(actual["map_asset_finalize"], metadata["map_asset_finalize"])
        actual["exploration_checkpoint"]["stage"] = "caller lifted change"
        actual["map_asset_finalize"]["committed"] = False
        self.assertEqual(metadata["exploration_checkpoint"]["stage"], "saved")
        self.assertTrue(metadata["map_asset_finalize"]["committed"])
        metadata["unrelated"]["values"].append(99)
        metadata["exploration_checkpoint"]["nested"]["values"].append(99)
        actual["map_asset_finalize"]["nested"]["values"].append(99)
        actual["result"]["nested"].append(99)
        self.assertEqual(runtime.get_task_snapshot(task_id, captured_task=capture), expected)
        self.assertEqual(runtime.get_task_snapshot(task_id), expected)
        self.manager.update_task_metadata(task_id, {"unrelated": {"values": [5]}})
        current = runtime.get_task_snapshot(task_id)
        self.assertEqual(current["task"].metadata["unrelated"], {"values": [5]})
        self.assertEqual(runtime.get_task_snapshot(task_id, captured_task=capture), expected)
        self.assertEqual(current["result"], expected["result"])

    def test_runtime_projection_identity_feedback_and_lock_boundary(self):
        task_id = self.finish({"known": True})
        runtime = self.runtime()
        capture = self.manager.capture_task_receipt(task_id)
        first = runtime.get_task_snapshot(task_id, captured_task=capture)
        runtime.capability_registry.get_descriptor = lambda name: NS(display_name="标签 B")
        second = runtime.get_task_snapshot(task_id, captured_task=capture)
        self.assertEqual(first["feedback"], second["feedback"])
        first["feedback"]["source"]["display_name"] = "caller"
        self.assertNotEqual(runtime.get_task_snapshot(task_id)["feedback"]["source"]["display_name"], "caller")
        self.assertNotIn("revision", second)


if __name__ == "__main__":
    unittest.main(verbosity=2)
