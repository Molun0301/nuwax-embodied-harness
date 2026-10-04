#!/usr/bin/env python3
"""任务准备期限与取消回归：真实编排、离线 RPC 边界，不启动设备或运动。"""
from __future__ import annotations

import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import grpc

from core.task_manager import TaskCancelledError, TaskExecutionContext, TaskTimeoutError
from gateways import runtime as gateway_module
from scripts.diagnostics.navigation_activation_probe import Fixture, ObservedEvent, join, spawn
from services.geometry.geometry_runtime_provider import GeometryRuntimeRpcError
from skills.handlers.localization_skills import handle_activate_map, handle_relocalize_active_map


class TaskControl:
    def __init__(self, deadline=None):
        self.cancelled = threading.Event()
        self.deadline = deadline
        self.checked = threading.Event()
        self.on_check = None

    def ensure_active(self):
        self.checked.set()
        if self.on_check is not None:
            self.on_check()
        if self.cancelled.is_set():
            raise TaskCancelledError("离线取消")
        if self.deadline is not None and time.monotonic() >= self.deadline:
            raise TaskTimeoutError("离线期限耗尽")

    def timeout_deadline_monotonic(self):
        return self.deadline


class ActivationControlChecks(unittest.TestCase):
    def setUp(self):
        self.releases, self.workers = [], []

    def tearDown(self):
        for release in self.releases:
            release.set()
        for worker in self.workers:
            worker[0].join(2)
            self.assertFalse(worker[0].is_alive())

    def gate(self):
        gate = threading.Event()
        self.releases.append(gate)
        return gate

    def worker(self, operation):
        worker = spawn(operation)
        self.workers.append(worker)
        return worker

    def require(self, event):
        self.assertTrue(event.wait(1), "同步门闩未触发")

    def event_fixture(self, level="motion"):
        fixture, event = Fixture(), ObservedEvent(level)
        fixture.runtime._geometry_runtime_start_event = event
        self.releases.append(event)
        return fixture, event

    def context(self, deadline=None):
        record = NS(completion_committed=False, timeout_triggered=False, cancel_requested=False,
                    timeout_deadline_monotonic=deadline)
        def timeout(task_id):
            record.timeout_triggered = True
        manager = NS(_lock=threading.RLock(), _records={"offline-task": record},
                     update_task=lambda *args, **kwargs: None, _handle_timeout=timeout)
        return TaskExecutionContext(manager=manager, task_id="offline-task"), record

    def skill_context(self, fixture, calls):
        workspace = NS(
            activate_map=lambda **kwargs: calls.append(kwargs) or {"offline": True},
            relocalize_active_map=lambda **kwargs: calls.append(kwargs) or (None, None),
        )
        return NS(ensure_geometry_runtime_started=fixture.runtime._ensure_geometry_runtime_started,
                  ensure_geometry_runtime_for_capability=fixture.runtime._ensure_geometry_runtime_for_capability,
                  map_workspace_service=workspace, refresh_workspace_snapshots=lambda **kwargs: None,
                  sync_perception_runtime_for_memory_state=lambda **kwargs: None,
                  publish_runtime_governance_stream_frame=lambda: None)

    def test_cancel_before_preparation_has_no_rpc_or_activation(self):
        fixture, control = Fixture(), TaskControl()
        control.cancelled.set()
        with self.assertRaises(TaskCancelledError):
            fixture.runtime._ensure_geometry_runtime_started("navigate_to_pose", check_active=control.ensure_active)
        self.assertEqual(fixture.stub.calls, [])
        self.assertEqual(fixture.activation_calls, [])
        self.assertIsNone(fixture.runtime._geometry_runtime_start_event)

    def test_cancel_immediately_before_ownership_has_no_activation(self):
        fixture, control = Fixture(), TaskControl()
        fixture.stub.GetStatus = lambda *a, **k: (control.cancelled.set() or NS(phase="READY", message="", rss_kb=1))
        with self.assertRaises(TaskCancelledError):
            fixture.runtime._ensure_geometry_runtime_started("navigate_to_pose", check_active=control.ensure_active)
        self.assertEqual(fixture.activation_calls, [])
        self.assertIsNone(fixture.runtime._geometry_runtime_start_event)

    def test_cancel_follower_leaves_shared_owner_intact_without_task_deadline(self):
        fixture, event = self.event_fixture()
        control = TaskControl()
        worker = self.worker(lambda: fixture.runtime._ensure_geometry_runtime_started(
            "navigate_to_pose", check_active=control.ensure_active))
        self.require(event.entered)
        control.cancelled.set()
        self.assertTrue(worker[1].wait(0.5))
        self.assertIsInstance(join(worker)["error"], TaskCancelledError)
        self.assertFalse(event.is_set())
        self.assertIsNone(event.error)
        self.assertIs(fixture.runtime._geometry_runtime_start_event, event)
        self.assertLessEqual(max(event.timeouts), 0.05)

    def test_closed_runtime_wakes_no_deadline_follower(self):
        fixture, event = self.event_fixture()
        worker = self.worker(lambda: fixture.runtime._ensure_geometry_runtime_started("activate_map"))
        self.require(event.entered)
        fixture.runtime._closed = True
        self.assertTrue(worker[1].wait(0.5))
        self.assertEqual(join(worker)["error"].error_code, "gateway_runtime_closed")
        self.assertFalse(event.is_set())

    def test_explicit_deadline_does_not_cancel_owner(self):
        fixture, event = self.event_fixture()
        with self.assertRaises(GeometryRuntimeRpcError) as caught:
            fixture.runtime._ensure_geometry_runtime_started("activate_map", deadline_monotonic=time.monotonic()+0.03)
        self.assertEqual(caught.exception.grpc_code, grpc.StatusCode.DEADLINE_EXCEEDED)
        self.assertFalse(event.is_set())
        self.assertIsNone(event.error)

    def test_owner_cancellation_waits_for_shared_success_without_poisoning_follower(self):
        fixture, control = Fixture(), TaskControl()
        entered, release = threading.Event(), self.gate()
        def activate(*args, **kwargs):
            fixture.activation_calls.append(args)
            entered.set()
            self.require(release)
        fixture.runtime._request_geometry_runtime_activation = activate
        with patch.object(gateway_module, "_RuntimeActivationEvent", ObservedEvent):
            owner = self.worker(lambda: fixture.runtime._ensure_geometry_runtime_started(
                "navigate_to_pose", check_active=control.ensure_active))
            self.require(entered)
            event = fixture.runtime._geometry_runtime_start_event
            follower = self.worker(lambda: fixture.runtime._ensure_geometry_runtime_started("navigate_to_pose"))
            self.require(event.entered)
            control.cancelled.set()
            self.assertFalse(owner[1].wait(0.03))
            release.set()
            self.assertIsInstance(join(owner)["error"], TaskCancelledError)
            self.assertNotIn("error", join(follower))
        self.assertIsNone(event.error)
        self.assertEqual(len(fixture.activation_calls), 1)
        self.assertIsNone(fixture.runtime._geometry_runtime_start_event)

    def test_cancelled_map_owner_finishes_shared_activation_but_never_dispatches_map(self):
        fixture, control, calls = Fixture(), TaskControl(), []
        entered, release = threading.Event(), self.gate()
        original_status = fixture.stub.GetStatus
        first_probe = True
        def status(*args, **kwargs):
            nonlocal first_probe
            if first_probe:
                first_probe = False
                return NS(phase="FAILED", message="离线冷启动", rss_kb=1)
            return original_status(*args, **kwargs)
        fixture.stub.GetStatus = status
        def activate(*args, **kwargs):
            entered.set()
            self.require(release)
        fixture.runtime._request_geometry_runtime_activation = activate
        with patch.object(gateway_module, "_RuntimeActivationEvent", ObservedEvent):
            owner = self.worker(lambda: handle_activate_map(self.skill_context(fixture, calls),
                {"map_name": "offline"}, execution_context=control))
            self.require(entered)
            event = fixture.runtime._geometry_runtime_start_event
            follower = self.worker(lambda: fixture.runtime._ensure_geometry_runtime_started("activate_map"))
            self.require(event.entered)
            control.cancelled.set()
            self.assertFalse(owner[1].wait(0.03))
            release.set()
            self.assertIsInstance(join(owner)["error"], TaskCancelledError)
            self.assertNotIn("error", join(follower))
        self.assertEqual(calls, [])
        self.assertIsNone(event.error)

    def test_actual_failure_shared_and_later_retry_allowed(self):
        fixture = Fixture()
        failure = GeometryRuntimeRpcError(rpc_name="RuntimeActivation", grpc_code=grpc.StatusCode.UNAVAILABLE,
                                          grpc_details="离线共享失败")
        entered, release = threading.Event(), self.gate()
        def fail(*args, **kwargs):
            entered.set()
            self.require(release)
            raise failure
        fixture.runtime._request_geometry_runtime_activation = fail
        with patch.object(gateway_module, "_RuntimeActivationEvent", ObservedEvent):
            owner = self.worker(lambda: fixture.runtime._ensure_geometry_runtime_started("navigate_to_pose"))
            self.require(entered)
            event = fixture.runtime._geometry_runtime_start_event
            follower = self.worker(lambda: fixture.runtime._ensure_geometry_runtime_started("navigate_to_pose"))
            self.require(event.entered)
            release.set()
            self.assertIs(join(owner)["error"], failure)
            self.assertIs(join(follower)["error"], failure)
        fixture.runtime._request_geometry_runtime_activation = fixture.activate
        fixture.runtime._ensure_geometry_runtime_started("navigate_to_pose")
        self.assertEqual(len(fixture.activation_calls), 1)

    def test_initial_provider_lock_honors_deadline_before_rpc(self):
        fixture = Fixture()
        with fixture.platform._runtime_state_lock:
            worker = self.worker(lambda: fixture.runtime._geometry_runtime_provider_ready(
                deadline_monotonic=time.monotonic()+0.03))
            self.assertTrue(worker[1].wait(0.5), "提供器初始锁未服从期限")
        self.assertEqual(join(worker)["error"].grpc_code, grpc.StatusCode.DEADLINE_EXCEEDED)
        self.assertEqual(fixture.stub.calls, [])

    def test_initial_provider_lock_honors_cancel_without_deadline(self):
        fixture, control = Fixture(), TaskControl()
        with fixture.platform._runtime_state_lock:
            worker = self.worker(lambda: fixture.runtime._geometry_runtime_provider_ready(check_active=control.ensure_active))
            self.require(control.checked)
            control.cancelled.set()
            self.assertTrue(worker[1].wait(0.5))
        self.assertIsInstance(join(worker)["error"], TaskCancelledError)
        self.assertEqual(fixture.stub.calls, [])

    def test_geometry_coordination_lock_wait_is_cancellable(self):
        fixture, control = Fixture(), TaskControl()
        with fixture.runtime._geometry_runtime_lock:
            worker = self.worker(lambda: fixture.runtime._ensure_geometry_runtime_started(
                "navigate_to_pose", check_active=control.ensure_active))
            self.require(control.checked)
            control.cancelled.set()
            self.assertTrue(worker[1].wait(0.5))
        self.assertIsInstance(join(worker)["error"], TaskCancelledError)
        self.assertIsNone(fixture.runtime._geometry_runtime_start_event)

    def test_task_checks_do_not_run_under_geometry_or_provider_lock(self):
        fixture, control = Fixture(), TaskControl()
        def assert_unlocked():
            self.assertFalse(fixture.runtime._geometry_runtime_lock._is_owned())
            self.assertFalse(fixture.platform._runtime_state_lock._is_owned())
        control.on_check = assert_unlocked
        fixture.runtime._ensure_geometry_runtime_started("navigate_to_pose", check_active=control.ensure_active)

    def test_changed_provider_health_is_not_reused(self):
        fixture = Fixture()
        def swap(*args, **kwargs):
            fixture.platform._remote_provider = NS(is_connected=True)
            return NS(phase="READY", message="", rss_kb=1)
        fixture.stub.GetStatus = swap
        ready, message = fixture.runtime._geometry_runtime_provider_ready(deadline_monotonic=time.monotonic()+1)
        self.assertFalse(ready)
        self.assertIn("更换", message)

    def test_legacy_no_keyword_health_callbacks_work_without_budget(self):
        fixture = Fixture()
        fixture.runtime.platform_geometry_runtime = NS(
            get_geometry_runtime_provider=lambda: fixture.provider,
            probe_geometry_runtime_health=lambda: (True, "legacy-read"),
        )
        self.assertEqual(fixture.runtime._geometry_runtime_provider_ready(), (True, "legacy-read"))

    def test_legacy_getter_cannot_silently_remove_requested_budget(self):
        fixture = Fixture()
        calls = []
        def old_getter():
            calls.append("unbounded-getter")
            return fixture.provider
        fixture.runtime.platform_geometry_runtime = NS(
            get_geometry_runtime_provider=old_getter,
            probe_geometry_runtime_health=lambda: (True, "legacy-read"),
        )
        with self.assertRaisesRegex(TypeError, "deadline_monotonic"):
            fixture.runtime._geometry_runtime_provider_ready(deadline_monotonic=time.monotonic()+1)
        self.assertEqual(calls, [])

    def test_legacy_probe_keyword_error_is_not_unavailable(self):
        fixture = Fixture()
        fixture.runtime.platform_geometry_runtime = NS(
            get_geometry_runtime_provider=lambda **kwargs: fixture.provider,
            probe_geometry_runtime_health=lambda: (True, "legacy-read"),
        )
        with self.assertRaisesRegex(TypeError, "deadline_monotonic"):
            fixture.runtime._geometry_runtime_provider_ready(deadline_monotonic=time.monotonic()+1)

    def test_rpc_error_codes_unchanged(self):
        fixture = Fixture()
        fixture.stub.error = grpc.StatusCode.DEADLINE_EXCEEDED
        with self.assertRaises(GeometryRuntimeRpcError) as caught:
            fixture.runtime._geometry_runtime_provider_ready(deadline_monotonic=time.monotonic()+1)
        self.assertEqual(caught.exception.grpc_code, grpc.StatusCode.DEADLINE_EXCEEDED)
        fixture.stub.error = grpc.StatusCode.UNAVAILABLE
        ready, message = fixture.runtime._geometry_runtime_provider_ready(deadline_monotonic=time.monotonic()+1)
        self.assertFalse(ready)
        self.assertIn("UNAVAILABLE", message)

    def test_map_handler_cancelled_follower_never_dispatches_backend(self):
        for handler in (handle_activate_map, handle_relocalize_active_map):
            with self.subTest(handler=handler.__name__):
                fixture, event = self.event_fixture()
                context, record = self.context(time.monotonic()+5)
                calls = []
                worker = self.worker(lambda handler=handler, fixture=fixture, calls=calls, context=context:
                    handler(self.skill_context(fixture, calls), {"map_name":"offline"}, execution_context=context))
                self.require(event.entered)
                record.cancel_requested = True
                self.assertTrue(worker[1].wait(0.5))
                self.assertIsInstance(join(worker)["error"], TaskCancelledError)
                self.assertEqual(calls, [])
                self.assertFalse(event.is_set())

    def test_map_handler_deadline_is_propagated(self):
        fixture, event = self.event_fixture()
        context, _ = self.context(time.monotonic()+0.03)
        calls = []
        with self.assertRaises(TaskTimeoutError):
            handle_activate_map(self.skill_context(fixture, calls), {"map_name":"offline"}, execution_context=context)
        self.assertEqual(calls, [])
        self.assertFalse(event.is_set())

    def test_cancel_after_shared_success_before_map_dispatch(self):
        for handler, callback in ((handle_activate_map, "ensure_geometry_runtime_started"),
                                  (handle_relocalize_active_map, "ensure_geometry_runtime_for_capability")):
            with self.subTest(handler=handler.__name__):
                fixture, calls, control = Fixture(), [], TaskControl()
                ctx = self.skill_context(fixture, calls)
                setattr(ctx, callback, lambda *a, control=control, **k: control.cancelled.set())
                with self.assertRaises(TaskCancelledError):
                    handler(ctx, {"map_name":"offline"}, execution_context=control)
                self.assertEqual(calls, [])

    def test_direct_handlers_and_relocalization_logical_timeout_unchanged(self):
        fixture, calls = Fixture(), []
        ctx = self.skill_context(fixture, calls)
        # 旧直接调用只提供单参数回调，仍须兼容。
        ctx.ensure_geometry_runtime_started = lambda capability: None
        ctx.ensure_geometry_runtime_for_capability = lambda capability: None
        handle_activate_map(ctx, {"map_name":"offline"})
        handle_relocalize_active_map(ctx, {"map_name":"offline", "timeout_sec":17})
        self.assertEqual(len(calls), 2)
        self.assertEqual(calls[1]["metadata"]["logical_timeout_ms"], 10000)
        self.assertNotIn("execution_context", calls[1]["metadata"])

    def test_geometry_runner_preserves_known_receipt_after_late_cancel(self):
        fixture = Fixture()
        runtime, captured = fixture.runtime, {}
        context, record = self.context(time.monotonic()+5)
        def submit(capability, runner, **kwargs):
            captured.update(runner=runner, options=kwargs)
            return "offline-task"
        context.manager.submit = submit
        runtime.task_manager = context.manager
        runtime.capability_registry = NS(get_descriptor=lambda name: NS(required_resources=[]))
        runtime._resolve_async_timeout = lambda *args, **kwargs: 5
        runtime._build_task_request_metadata = lambda **kwargs: {}
        def handler(payload, *, requested_by, execution_context):
            self.assertIs(execution_context, context)
            self.assertEqual(payload, {"map_name":"offline"})
            record.cancel_requested = True
            return {"fake_committed": True}
        runtime._submit_geometry_skill_task(capability_name="activate_map", skill_handler=handler,
            arguments={"map_name":"offline"}, requested_by=None, request_context=None)
        self.assertEqual(captured["options"]["input_payload"], {"map_name":"offline"})
        self.assertEqual(captured["runner"](context), {"fake_committed": True})

    def test_motion_admission_cancel_does_not_query_physical_guard(self):
        fixture, event = self.event_fixture()
        control = TaskControl()
        worker = self.worker(lambda: fixture.runtime._admit_motion_request("navigate_to_pose",
            request_id="offline", check_active=control.ensure_active))
        self.require(event.entered)
        control.cancelled.set()
        self.assertTrue(worker[1].wait(0.5))
        self.assertIsInstance(join(worker)["error"], TaskCancelledError)
        self.assertEqual(fixture.readiness_calls, [])

    def test_no_activation_dedup_or_guard_readiness_change_in_this_patch(self):
        fixture = Fixture()
        admission = fixture.runtime._admit_motion_request("navigate_to_pose", request_id="offline")
        fixture.runtime._ensure_motion_runtime_for_backend_call("navigate_to_pose", admission)
        self.assertEqual(len(fixture.activation_calls), 2)
        self.assertEqual(len(fixture.stub.calls), 6)
        self.assertEqual(len(fixture.readiness_calls), 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
