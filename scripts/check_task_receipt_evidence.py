#!/usr/bin/env python3
"""离线检查版本化任务返回证据；不调用设备、网络或模型。"""
from __future__ import annotations

import sys
import threading
import unittest
from collections import namedtuple
from datetime import datetime, timedelta, timezone, tzinfo
from decimal import Decimal
from enum import Enum
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pydantic import BaseModel, ConfigDict, PrivateAttr

from contracts.capabilities import CapabilityExecutionMode
from contracts.tasks import TaskSpec, TaskState, TaskStatus
from core.event_bus import EventBus
from core.resource_lock import ResourceLockManager
from core.task_manager import TaskManager, TaskReceiptUnavailableError, TaskSafetyBarrierError
from gateways.runtime import GatewayRuntime


# Complete method frozen from 7e8093ff144d956cbb20fc21c58f5b246e6c6397.
# Keep its body unchanged: this oracle must not reconstruct the candidate.
_HISTORICAL_TASK_SNAPSHOT = '''def get_task_snapshot(self, task_id: str, *, captured_task=None) -> Dict[str, Any]:
    """读取独立快照；投影与回调均在任务协调锁外运行。"""
    from core.task_manager import TaskReceiptCapture, TaskReceiptUnavailableError
    from gateways.feedback_messages import build_task_feedback

    try:
        capture = captured_task or self.task_manager.capture_task_receipt(task_id)
        if (not isinstance(capture, TaskReceiptCapture) or capture.task_id != task_id
                or capture.source_id != self.task_manager.receipt_source_id):
            raise ValueError("任务回执封套不属于当前管理器与请求任务。")
        snapshot = capture.snapshot()
    except TaskReceiptUnavailableError as error:
        raise GatewayError(str(error), error_code="task_receipt_unsupported", http_status=500,
                           details={"status_source": "live_task", "receipt_capture": "unsupported",
                                    "task_outcome_unchanged": True, "action_replay_allowed": False}) from error
    spec = snapshot["task"]
    if (snapshot.get("task_id") != task_id or spec.task_id != task_id
            or snapshot["status"].task_id != task_id):
        raise ValueError("任务回执内容身份与封套不一致。")
    events = snapshot["events"]
    snapshot["feedback"] = capture.feedback(
        lambda: build_task_feedback(self, events[-1]) if events else None)
    metadata = dict(spec.metadata)
    checkpoint = metadata.get("exploration_checkpoint")
    if isinstance(checkpoint, dict):
        snapshot["exploration_checkpoint"] = dict(checkpoint)
    receipt = metadata.get("map_asset_finalize")
    if isinstance(receipt, dict):
        snapshot["map_asset_finalize"] = dict(receipt)
        if isinstance(snapshot["result"], dict):
            snapshot["result"] = {**snapshot["result"], "finalize_pending": False}
    return self._attach_async_task_polling_advisory(task_id, snapshot)'''


def historical_task_snapshot(runtime, task_id, capture):
    namespace = dict(GatewayRuntime.get_task_snapshot.__globals__)
    exec(_HISTORICAL_TASK_SNAPSHOT, namespace)
    return namespace["get_task_snapshot"](runtime, task_id, captured_task=capture)


class ReceiptEvidenceChecks(unittest.TestCase):
    def setUp(self):
        self.bus = EventBus()
        self.locks = ResourceLockManager()
        self.manager = TaskManager(event_bus=self.bus, resource_lock_manager=self.locks)
        self.releases = []

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
        runtime = GatewayRuntime.__new__(GatewayRuntime)
        runtime.task_manager = self.manager
        runtime.event_bus = self.bus
        runtime._task_status_last_polled_at = {}
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
        self.assertIs(result, returned)
        self.assertEqual(calls, [1])
        self.assertEqual(after["result"], returned)
        self.assertEqual(after["status"], before["status"])
        self.assertEqual(after["error_payload"], before["error_payload"])
        self.assertEqual(after["status"].state, interrupt)
        self.assertEqual(after["events"][-1].event_type, "task.result_returned")
        self.assertEqual(after["events"][-1].payload["evidence_kind"], "runner_returned")
        self.assertFalse(self.manager._records[task_id].completion_committed)
        self.assertEqual(self.locks.held_resources(task_id), ())

    def test_late_cancel_return_is_evidence(self):
        self.late("cancelled")

    def test_late_timeout_return_is_evidence(self):
        self.late("timeout")

    def test_opaque_identity_and_nested_sharing_do_not_claim_detachment(self):
        def callback():
            pass
        callback.value = 1
        class IdentityCopy:
            def __deepcopy__(self, memo):
                return self
        class SharedChild:
            def __init__(self):
                self.values = [1]
            def __deepcopy__(self, memo):
                other = SharedChild()
                other.values = self.values
                return other
        for value in (callback, {"callback": callback}, IdentityCopy(), SharedChild()):
            calls = []
            task_id = self.manager.submit("offline_task", lambda context, value=value, calls=calls: calls.append(1) or value).task_id
            self.assertIs(self.manager.wait_for_task(task_id, timeout=2), value)
            self.assertEqual(calls, [1])
            self.assertEqual(self.manager.get_task_status(task_id).state, "succeeded")
            with self.assertRaises(TaskReceiptUnavailableError):
                self.manager.capture_task_receipt(task_id)
            with self.assertRaises(TaskReceiptUnavailableError):
                self.manager.get_task_result(task_id)

    def test_tuple_events_copy_out_survives_uncopyable_leaf(self):
        class BadCopy:
            def __deepcopy__(self, memo):
                raise TypeError("不透明叶子")
        task_id = self.finish({"ok": True})
        self.manager.update_task_metadata(task_id, {"opaque": BadCopy()})
        before = self.manager.get_task_events(task_id)
        after = self.manager.get_task_events(task_id)
        after[0].message = "caller mutation"
        self.assertEqual(self.manager.get_task_events(task_id)[0].message, before[0].message)
        self.assertIsNot(after[0], self.manager._records[task_id].events[0])
        with self.assertRaises(TaskReceiptUnavailableError):
            self.manager.capture_task_receipt(task_id)

    def test_prepared_callback_dictionaries_remain_mutable_and_detached(self):
        observed = []
        def callback(event):
            self.assertFalse(self.manager._lock._is_owned())
            event.payload["request_context"]["caller"] = "changed"
            observed.append(event.event_type)
        self.bus.subscribe(callback, event_types=("task.accepted", "task.started", "task.succeeded"))
        task_id = self.finish({"known": True})
        self.assertEqual(observed, ["task.accepted", "task.started", "task.succeeded"])
        for event in self.manager.get_task_events(task_id):
            self.assertNotIn("caller", event.payload["request_context"])

    def test_started_public_context_tracks_metadata_update_after_acceptance(self):
        seen = []
        def callback(event):
            if event.event_type == "task.accepted":
                self.manager.update_task_metadata(event.task_id, {"request_context": {"request_id": "new-request"}})
            elif event.event_type == "task.started":
                seen.append(event.payload["request_context"]["request_id"])
                event.payload["request_context"]["caller"] = "changed"
        self.bus.subscribe(callback, event_types=("task.accepted", "task.started"))
        task_id = self.finish({"known": True})
        self.assertEqual(seen, ["new-request"])
        self.assertIsNone(self.manager._records[task_id].accepted_public_payload)
        self.assertIsNone(self.manager._records[task_id].started_public_payload)
        for event in self.manager.get_task_events(task_id):
            self.assertNotIn("caller", event.payload["request_context"])

    def test_named_tuple_metadata_keeps_type_and_owned_values(self):
        Point = namedtuple("Point", "values")
        original = Point([1])
        task = self.manager.submit("offline_task", lambda context: {"ok": True}, metadata={"point": original})
        self.manager.wait_for_task(task.task_id, timeout=2)
        original.values.append(9)
        copied = self.manager.get_task_spec(task.task_id).metadata["point"]
        self.assertIsInstance(copied, Point)
        self.assertEqual(copied.values, [1])
        with self.assertRaises(TaskReceiptUnavailableError):
            self.manager.capture_task_receipt(task.task_id)

    def test_progress_payload_preserves_unsupported_ownership_marker(self):
        def callback():
            pass
        callback.value = 1
        calls = []
        def runner(context):
            calls.append(1)
            context.update(payload={"callback": callback})
            return {"ok": True}
        task_id = self.manager.submit("offline_task", runner).task_id
        self.manager.wait_for_task(task_id, timeout=2)
        callback.value = 2
        self.assertEqual(calls, [1])
        self.assertEqual(self.manager.get_task_status(task_id).state, "succeeded")
        with self.assertRaises(TaskReceiptUnavailableError):
            self.manager.capture_task_receipt(task_id)

    def test_mutable_dictionary_key_is_unsupported_without_failing_action(self):
        class Key:
            def __init__(self):
                self.label = "a"
            def __str__(self):
                return self.label
        key = Key()
        task_id = self.finish({key: 1})
        key.label = "b"
        self.assertEqual(self.manager.get_task_status(task_id).state, "succeeded")
        with self.assertRaises(TaskReceiptUnavailableError):
            self.manager.capture_task_receipt(task_id)

    def test_mutable_enum_value_is_unsupported_without_failing_action(self):
        class Mutable(Enum):
            VALUE = {"v": 1}  # noqa: RUF012 - 对抗用例刻意使用可变 Enum 值。
        task_id = self.finish(Mutable.VALUE)
        Mutable.VALUE.value["v"] = 2
        self.assertEqual(self.manager.get_task_status(task_id).state, "succeeded")
        with self.assertRaises(TaskReceiptUnavailableError):
            self.manager.capture_task_receipt(task_id)

    def test_model_copy_protocol_cannot_share_public_fields_or_extra(self):
        class AliasModel(BaseModel):
            model_config = ConfigDict(extra="allow")
            values: list
            _lock: object = PrivateAttr(default_factory=threading.Lock)
            def __copy__(self):
                copied = super().__copy__()
                object.__setattr__(copied, "__dict__", self.__dict__)
                return copied
        original = AliasModel(values=[1], extra_value={"items": [1]})
        task_id = self.finish(original)
        original.values.append(2)
        original.extra_value["items"].append(2)
        self.assertIs(self.manager._records[task_id].result, original)
        self.assertEqual(self.manager.get_task_status(task_id).state, "succeeded")
        with self.assertRaises(TaskReceiptUnavailableError):
            self.manager.capture_task_receipt(task_id)

    def test_custom_timezone_is_unsupported_standard_datetimes_are_owned(self):
        class MutableTZ(tzinfo):
            hours = 0
            def utcoffset(self, dt):
                return timedelta(hours=self.hours)
            def dst(self, dt):
                return timedelta(0)
        custom = MutableTZ()
        calls = []
        value = datetime(2026, 1, 1, tzinfo=custom)
        task_id = self.manager.submit("offline_task", lambda context: calls.append(1) or value).task_id
        self.assertIs(self.manager.wait_for_task(task_id, timeout=2), value)
        custom.hours = 3
        self.assertEqual(calls, [1])
        self.assertEqual(self.manager.get_task_status(task_id).state, "succeeded")
        with self.assertRaises(TaskReceiptUnavailableError):
            self.manager.capture_task_receipt(task_id)
        for zone in (None, timezone.utc, timezone(timedelta(hours=3))):
            fixed = datetime(2026, 1, 1, tzinfo=zone)
            task_id = self.finish(fixed)
            self.assertEqual(self.manager.capture_task_receipt(task_id).snapshot()["result"].isoformat(), fixed.isoformat())

    def test_tuple_list_cycle_does_not_share_original_metadata(self):
        child = []
        original = (child,)
        child.append(original)
        task = self.manager.submit("offline_task", lambda context: None, metadata={"cycle": original})
        self.manager.wait_for_task(task.task_id, timeout=2)
        capture = self.manager.capture_task_receipt(task.task_id)
        child.append("caller mutation")
        copy = capture.snapshot()["task"].metadata["cycle"]
        self.assertEqual(len(copy[0]), 1)
        self.assertEqual(len(copy[0][0][0]), 1)
        copy[0].append("capture mutation")
        self.assertEqual(len(capture.snapshot()["task"].metadata["cycle"][0][0][0]), 1)

    def test_external_enum_value_and_named_tuple_key_projection_are_unsupported(self):
        state = {"label": "a"}
        class DynamicEnum(Enum):
            ITEM = "declared"
            @property
            def value(self):
                return state["label"]
        Point = namedtuple("Point", "value")
        class DynamicKey(Point):
            __slots__ = ()
            def __str__(self):
                return state["label"]
        calls = []
        for value in (DynamicEnum.ITEM, {DynamicKey(1): 2}):
            task_id = self.manager.submit("offline_task", lambda context, value=value: calls.append(1) or value).task_id
            self.assertIs(self.manager.wait_for_task(task_id, timeout=2), value)
            state["label"] = "b"
            self.assertEqual(self.manager.get_task_status(task_id).state, "succeeded")
            with self.assertRaises(TaskReceiptUnavailableError):
                self.manager.capture_task_receipt(task_id)
        self.assertEqual(calls, [1, 1])
        task_id = self.finish(TaskState.SUCCEEDED)
        self.assertEqual(self.manager.capture_task_receipt(task_id).snapshot()["result"], TaskState.SUCCEEDED)

    def test_unsupported_outer_payload_verdict_survives_all_ingress_conversions(self):
        class CustomDict(dict):
            pass
        class FailedCopyDict(dict):
            def __deepcopy__(self, memo):
                raise TypeError("不透明外层复制")
        for payload_type in (CustomDict, FailedCopyDict):
            for route in ("input", "metadata", "metadata_update", "progress", "event"):
                with self.subTest(payload_type=payload_type, route=route):
                    def callback():
                        pass
                    callback.value = 1
                    payload = payload_type(callback=callback)
                    calls = []
                    def runner(context, route=route, payload=payload, calls=calls):
                        calls.append(1)
                        if route == "metadata_update":
                            context.update_metadata(payload)
                        elif route == "progress":
                            context.update(payload=payload)
                        elif route == "event":
                            context.publish_event("task.audit", payload=payload)
                        return {"ok": True}
                    task = self.manager.submit("offline_task", runner,
                                               input_payload=payload if route == "input" else None,
                                               metadata=payload if route == "metadata" else None)
                    self.manager.wait_for_task(task.task_id, timeout=2)
                    callback.value = 2
                    self.assertEqual(calls, [1])
                    self.assertEqual(self.manager.get_task_status(task.task_id).state, "succeeded")
                    self.assertIsNotNone(self.manager._records[task.task_id].receipt_ownership_error)
                    with self.assertRaises(TaskReceiptUnavailableError):
                        self.manager.capture_task_receipt(task.task_id)

    def test_unsupported_outer_completion_interrupt_and_error_payloads_keep_verdict(self):
        class CustomDict(dict):
            pass
        def callback():
            pass
        callback.value = 1
        for commit in (False, True):
            entered, release = self.gate()
            calls = []
            def runner(context, commit=commit, calls=calls, entered=entered, release=release):
                calls.append(1)
                if commit:
                    context.commit_success(CustomDict(proof="offline-commit", callback=callback))
                entered.set()
                self.require(release)
                if not commit:
                    context.acknowledge_interrupt_barrier(CustomDict(proof="offline-stop", callback=callback))
                return {"returned": True}
            task = self.manager.submit("offline_task", runner, deferred_interrupt=True)
            self.require(entered)
            self.assertEqual(self.manager.cancel_task(task.task_id), not commit)
            release.set()
            self.manager.wait_for_task(task.task_id, timeout=2)
            self.assertEqual(calls, [1])
            self.assertEqual(self.manager.get_task_status(task.task_id).state, "succeeded" if commit else "cancelled")
            with self.assertRaises(TaskReceiptUnavailableError):
                self.manager.capture_task_receipt(task.task_id)
        calls = []
        def failed(context):
            calls.append(1)
            self.manager._set_terminal_state(context.task_id, TaskState.FAILED, "offline failure",
                                             error_payload=CustomDict(code="offline", callback=callback))
            return {"known_return": True}
        task = self.manager.submit("offline_task", failed)
        self.manager.wait_for_task(task.task_id, timeout=2)
        self.assertEqual(calls, [1])
        self.assertEqual(self.manager.get_task_status(task.task_id).state, "failed")
        with self.assertRaises(TaskReceiptUnavailableError):
            self.manager.capture_task_receipt(task.task_id)

    def test_unsupported_outer_quarantine_proof_preserves_exact_safety_result(self):
        class CustomDict(dict):
            pass
        def callback():
            pass
        callback.value = 1
        entered, release = self.gate()
        calls = []
        def runner(context):
            calls.append(1)
            entered.set()
            self.require(release)
            return {"returned": True}
        task = self.manager.submit("activate_map", runner, input_payload={"goal_id": "offline-goal"},
                                   required_resources=["map_workspace"], deferred_interrupt=True)
        self.require(entered)
        self.manager.cancel_task(task.task_id)
        release.set()
        with self.assertRaises(TaskSafetyBarrierError):
            self.manager.wait_for_task(task.task_id, timeout=2)
        before = self.manager.get_task_status(task.task_id)
        correlation = {"ownership_domain": "navigation", "goal_id": "offline-goal"}
        proof = CustomDict(correlation, callback=callback)
        self.assertTrue(self.manager.release_resource_quarantine(task.task_id, proof=proof, correlation=correlation))
        self.assertEqual(calls, [1])
        self.assertEqual(self.manager.get_task_status(task.task_id), before)
        self.assertEqual(self.locks.held_resources(task.task_id), ())
        with self.assertRaises(TaskReceiptUnavailableError):
            self.manager.capture_task_receipt(task.task_id)

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
        result["nested"][1]["key"] = "runner-alias"
        self.manager.get_task_result(task_id)["nested"][1]["key"] = "reader-alias"
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
        updates["map_asset_finalize"]["status"] = "mutated"
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
        payload["nested"]["value"][0] = 9
        event.payload["nested"]["value"] = [8]
        snapshot = self.manager.capture_task_receipt(task_id).snapshot()
        self.assertEqual(tuple(snapshot["events"][-1].payload["nested"]["value"]), (1,))

    def test_source_changes_while_copying_capture(self):
        from core.task_manager import _copy_task_value
        entered, release = self.gate()
        task_id = self.finish({"known": True})
        capture = self.manager.capture_task_receipt(task_id)
        outputs = []
        def copy_gate(value, *args, **kwargs):
            self.assertFalse(self.manager._lock._is_owned())
            if threading.current_thread() is not threading.main_thread() and isinstance(value, dict) and value.get("task_id") == task_id:
                entered.set()
                self.require(release)
            return _copy_task_value(value, *args, **kwargs)
        with patch("core.task_manager._copy_task_value", copy_gate):
            thread = threading.Thread(target=lambda: outputs.append(capture.snapshot()))
            thread.start()
            self.require(entered)
            self.manager.update_task_metadata(task_id, {"later": True})
            release.set()
            thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertNotIn("later", outputs[0]["task"].metadata)
        self.assertGreater(self.manager.capture_task_receipt(task_id).revision, capture.revision)

    def test_uncopyable_return_preserves_success_and_original_value(self):
        calls = []
        manager = self.manager
        class Uncopyable:
            def __deepcopy__(self, memo):
                self.lock_was_held = manager._lock._is_owned()
                raise TypeError("离线不可复制")
        value = Uncopyable()
        task_id = self.manager.submit("activate_map", lambda context: calls.append(1) or value).task_id
        self.assertIs(self.manager.wait_for_task(task_id, timeout=2), value)
        self.assertEqual(calls, [1])
        self.assertEqual(self.manager.get_task_status(task_id).state, "succeeded")
        self.assertIs(self.manager._records[task_id].result, value)
        self.assertFalse(value.lock_was_held)
        with self.assertRaises(TaskReceiptUnavailableError):
            self.manager.capture_task_receipt(task_id)

    def test_opaque_metadata_preserves_execution_and_copyable_model_shape(self):
        class Opaque:
            def __deepcopy__(self, memo):
                raise TypeError("不透明元数据")
        opaque = Opaque()
        calls = []
        spec = self.manager.submit("activate_map", lambda context: calls.append(1) or {"done": True},
                                   metadata={"opaque": opaque, "list_value": [1]})
        self.manager.wait_for_task(spec.task_id, timeout=2)
        self.assertEqual(calls, [1])
        self.assertEqual(self.manager.get_task_status(spec.task_id).state, "succeeded")
        copied = self.manager.get_task_spec(spec.task_id)
        self.assertIs(copied.metadata["opaque"], opaque)
        self.assertIsInstance(copied.metadata["list_value"], list)
        with self.assertRaises(TaskReceiptUnavailableError):
            self.manager.capture_task_receipt(spec.task_id).snapshot()
        status, event = self.manager.capture_task_status_event(spec.task_id)
        self.assertEqual(status.task_id, spec.task_id)
        self.assertEqual(event.event_type, "task.succeeded")

    def test_copyable_opaque_metadata_event_cannot_mutate_source(self):
        class Box:
            def __init__(self):
                self.values = [1]
        task_id = self.finish({"done": True})
        observed = []
        def mutate(event):
            event.payload["metadata_updates"]["opaque"].values.append(9)
            observed.append(1)
        self.bus.subscribe(mutate, event_types=("task.metadata_updated",))
        self.manager.update_task_metadata(task_id, {"opaque": Box()})
        self.assertEqual(observed, [1])
        self.assertEqual(self.manager.get_task_spec(task_id).metadata["opaque"].values, [1])
        with self.assertRaises(TaskReceiptUnavailableError):
            self.manager.capture_task_receipt(task_id)

    def test_cyclic_metadata_does_not_reject_action(self):
        metadata = {}
        metadata["cycle"] = metadata
        calls = []
        task_id = self.manager.submit("activate_map", lambda context: calls.append(1), metadata=metadata).task_id
        self.manager.wait_for_task(task_id, timeout=2)
        self.assertEqual(calls, [1])
        self.assertEqual(self.manager.get_task_status(task_id).state, "succeeded")

    def test_fast_async_acceptance_never_copies_result_or_loses_task_id(self):
        class Opaque:
            def __deepcopy__(self, memo):
                raise TypeError("离线不透明值")
        for opaque_context, project_result in ((False, False), (False, True), (True, False), (True, True)):
            runtime = self.runtime()
            runtime.robot = NS(manifest=NS(robot_model="offline"))
            runtime._refresh_dynamic_capability_matrix = lambda **kwargs: None
            runtime._requires_motion_admission = lambda descriptor: False
            runtime._ensure_geometry_runtime_for_invoke = lambda name: None
            calls = []
            def handler(payload, *, requested_by, request_context, opaque_context=opaque_context, calls=calls):
                metadata = {"request_context": {"opaque": Opaque()}} if opaque_context else {}
                task = self.manager.submit("offline_task", lambda context: calls.append(1) or Opaque(), metadata=metadata)
                self.manager.wait_for_task(task.task_id, timeout=2)
                return task
            descriptor = NS(name="offline_task", display_name="离线任务", execution_mode=CapabilityExecutionMode.ASYNC_TASK)
            runtime.capability_registry = NS(get_registration=lambda name, descriptor=descriptor, handler=handler: NS(descriptor=descriptor, handler=handler),
                                             get_descriptor=lambda name, descriptor=descriptor: descriptor, assert_supported=lambda *a, **kw: None)
            with patch.object(self.manager, "capture_task_receipt", side_effect=AssertionError("受理不能要求完整回执")):
                response = runtime.invoke_capability("offline_task", requested_by="local", project_result=project_result)
            self.assertEqual(calls, [1])
            accepted_id = response["task"].get("task_id") if isinstance(response["task"], dict) else response["task"].task_id
            self.assertEqual(response["task_id"], accepted_id)
            self.assertEqual(response["status"].state, "succeeded")
            if opaque_context:
                self.assertIsNone(response["feedback"])
            with self.assertRaises(TaskReceiptUnavailableError):
                self.manager.capture_task_receipt(response["task_id"])

    def test_slim_acceptance_encodes_only_retained_fields(self):
        class Opaque:
            pass
        task_id = "task_offline_task_20261002t000000z_12345678"
        spec = TaskSpec(task_id=task_id, capability_name="offline_task",
                        input_payload={"map_name": "offline", "max_duration_sec": Decimal("2.5"), "unused": Opaque()},
                        metadata={"unused": Opaque(), "polling_guidance": {"hint": "wait", "delay": Decimal("1.25")}})
        status = TaskStatus(task_id=task_id, state=TaskState.ACCEPTED)
        actual = GatewayRuntime._slim_task_acceptance(spec, status)
        self.assertEqual(actual["task_id"], task_id)
        self.assertEqual(actual["input_payload"], {"map_name": "offline", "max_duration_sec": "2.5"})
        self.assertEqual(actual["polling_guidance"], {"hint": "wait", "delay": "1.25"})
        self.assertEqual(actual["status"]["updated_at"], status.model_dump(mode="json")["updated_at"])
        self.assertEqual(set(actual), {"task_id", "capability_name", "input_payload", "status", "polling_guidance", "next_step_hint"})

    def test_deferred_interrupt_without_proof_still_quarantines(self):
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
        self.assertTrue(self.manager._records[task_id].resource_quarantined)
        self.assertEqual(self.locks.held_resources(task_id), ("map_workspace",))

    def test_quarantine_release_still_requires_exact_original_correlation(self):
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
        before = self.manager.get_task_status(task_id)
        correlation = {"ownership_domain": "navigation", "goal_id": "offline-goal"}
        with self.assertRaises(ValueError):
            self.manager.release_resource_quarantine(task_id, proof={"ownership_domain": "navigation", "goal_id": "other"},
                                                     correlation=correlation)
        self.assertEqual(self.locks.held_resources(task_id), ("map_workspace",))
        self.assertTrue(self.manager.release_resource_quarantine(task_id, proof=correlation, correlation=correlation))
        self.assertEqual(self.manager.get_task_status(task_id), before)
        self.assertEqual(self.locks.held_resources(task_id), ())
        self.assertEqual(self.manager.capture_task_receipt(task_id).snapshot()["events"][-1].event_type,
                         "task.resource_quarantine_released")

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
            self.assertFalse(self.manager._records[task_id].resource_quarantined)


    def test_snapshot_metadata_projection_matches_frozen_method(self):
        from gateways.feedback import build_mcp_tool_result

        variants = (
            {},
            {"unrelated_%d" % index: {"values": [index, "offline"]} for index in range(512)},
            {"exploration_checkpoint": {"stage": "saved", "nested": {"values": [1]}},
             "map_asset_finalize": {"committed": True, "nested": {"values": [2]}}},
            {"exploration_checkpoint": None, "map_asset_finalize": ["not a mapping"]},
            {"exploration_checkpoint": ["not a mapping"], "map_asset_finalize": "pending"},
        )
        for metadata in variants:
            for result in (None, {}, {"finalize_pending": True, "nested": [3]}, ["result"]):
                with self.subTest(metadata_keys=len(metadata), result=result):
                    task_id = self.finish(result)
                    self.manager.update_task_metadata(task_id, metadata)
                    capture = self.manager.capture_task_receipt(task_id)
                    runtime = self.runtime()
                    expected = historical_task_snapshot(runtime, task_id, capture)
                    actual = runtime.get_task_snapshot(task_id, captured_task=capture)
                    self.assertEqual(actual, expected)
                    self.assertIs(type(actual["task"].metadata), dict)
                    # Compare complete MCP projection, without claiming an HTTP route exercise.
                    self.assertEqual(build_mcp_tool_result(actual), build_mcp_tool_result(expected))

    def test_snapshot_metadata_selected_copies_and_revision_isolation(self):
        task_id = self.finish({"finalize_pending": True, "nested": [3]})
        self.manager.update_task_metadata(task_id, {
            "exploration_checkpoint": {"stage": "saved", "nested": {"values": [1]}},
            "map_asset_finalize": {"committed": True, "nested": {"values": [2]}},
            "unrelated": {"values": [4]},
        })
        runtime = self.runtime()
        capture = self.manager.capture_task_receipt(task_id)
        expected = historical_task_snapshot(runtime, task_id, capture)
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
        self.assertEqual(current, historical_task_snapshot(
            runtime, task_id, self.manager.capture_task_receipt(task_id)))

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
        with self.assertRaises(ValueError):
            runtime.get_task_snapshot("other-task", captured_task=capture)
        other = TaskManager()
        try:
            runtime.task_manager = other
            with self.assertRaises(ValueError):
                runtime.get_task_snapshot(task_id, captured_task=capture)
        finally:
            other.shutdown()


if __name__ == "__main__":
    unittest.main(verbosity=2)
