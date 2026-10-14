#!/usr/bin/env python3
"""按需启动延迟的离线诊断；真实编排方法、假 RPC 边界，无设备与运动。

这里固定当前行为作为基线，不把已观察到的等待/取消缺陷当成目标合同。
计时仅为注入延迟的合成结果，不能解释实机 CPU 或原生规划器耗时。
"""
from __future__ import annotations

import json
import sys
import threading
import time
from contextlib import nullcontext
from functools import partial
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import grpc

from core.errors import GatewayError
from core.task_manager import TaskExecutionContext
from gateways import runtime as gateway_module
from gateways.runtime import GatewayRuntime, _RuntimeActivationEvent
from services.geometry.geometry_runtime_provider import GeometryRuntimeProvider, GeometryRuntimeRpcError
from services.geometry.runtime import PlatformGeometryRuntime
from services.navigation.motion_guard_client import ReadinessStatus
from skills.handlers.localization_skills import handle_activate_map


class FakeRpcError(grpc.RpcError):
    def __init__(self, status):
        self.status = status

    def code(self):
        return self.status

    def details(self):
        return "离线注入的 RPC 失败"


class FakeStub:
    def __init__(self, delay=0.0):
        self.delay, self.calls, self.error = delay, [], None

    def GetStatus(self, request, timeout):
        self.calls.append(timeout)
        if self.delay:
            threading.Event().wait(self.delay)
        if self.error:
            raise FakeRpcError(self.error)
        return NS(phase="READY", message="离线健康状态", rss_kb=1)


class ObservedEvent(_RuntimeActivationEvent):
    def __init__(self, level):
        super().__init__(level)
        self.entered, self.timeouts = threading.Event(), []

    def wait(self, timeout=None):
        self.timeouts.append(timeout)
        self.entered.set()
        return super().wait(timeout)


class Fixture:
    def __init__(self, delay=0.0):
        self.stub = FakeStub(delay)
        self.activation_calls, self.readiness_calls, self.start_calls = [], [], []
        provider = object.__new__(GeometryRuntimeProvider)
        provider._stub, provider._connected, provider._stopping = self.stub, True, False
        platform = object.__new__(PlatformGeometryRuntime)
        platform._remote_provider, platform._runtime_state_lock = provider, threading.RLock()
        runtime = object.__new__(GatewayRuntime)
        runtime.platform_geometry_runtime = platform
        runtime._geometry_runtime_lock, runtime._lifecycle_lock = threading.RLock(), threading.RLock()
        runtime._geometry_runtime_start_event, runtime._closed = None, False
        runtime.config = NS(geometry=NS(enabled=True, pipeline="slam_2d", mapping=NS(profile_enabled=True)))
        runtime.robot = NS(providers=None)
        runtime._input_bridge = NS(runtime_provider=provider, is_ready=lambda: True)
        runtime._input_bridge_started = True
        runtime._request_geometry_runtime_activation = self.activate
        runtime.safety_guard = NS(begin_motion_admission=lambda: 0, motion_submission=lambda generation: nullcontext())
        runtime.motion_guard = NS(start=self.guard_start, readiness=self.readiness, started=True)
        self.runtime, self.provider, self.platform = runtime, provider, platform

    def activate(self, capability, level, *, deadline_monotonic=None):
        self.activation_calls.append((capability, level, deadline_monotonic))
        if self.stub.delay:
            threading.Event().wait(self.stub.delay)

    def guard_start(self):
        self.start_calls.append(True)

    def readiness(self, source):
        self.readiness_calls.append(source)
        return ReadinessStatus(result=None, ready=True, actuator_configured=True,
                               epoch_durable=True, startup_zero_confirmed=True,
                               reason="离线准入", generation=1, boot_nonce=b"x" * 16,
                               motion_stop_generation=0)


def spawn(operation):
    done, result = threading.Event(), {}
    def run():
        try:
            result["value"] = operation()
        except BaseException as error:  # noqa: BLE001 - 完整转交诊断线程异常。
            result["error"] = error
        finally:
            done.set()
    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread, done, result


def join(worker):
    worker[0].join(2)
    assert not worker[0].is_alive(), "诊断线程未退出"
    return worker[2]


def warm_navigation():
    fixture = Fixture(delay=0.01)
    runtime = fixture.runtime
    deadline = time.monotonic() + 2
    before = time.monotonic()
    admission = runtime._admit_motion_request("navigate_to_pose", request_id="offline-probe",
                                             deadline_monotonic=deadline)
    runtime._ensure_motion_runtime_for_backend_call("navigate_to_pose", admission,
                                                    deadline_monotonic=deadline)
    elapsed = time.monotonic() - before
    assert len(fixture.stub.calls) == 6
    assert len(fixture.activation_calls) == 2
    assert len(fixture.readiness_calls) == 1
    return {"get_status_calls": 6, "activation_calls": 2, "physical_readiness_calls": 1,
            "injected_delay_per_boundary_ms": 10, "elapsed_ms": round(elapsed * 1000, 3)}


def bounded_wait():
    fixture = Fixture()
    event = ObservedEvent("motion")
    fixture.runtime._geometry_runtime_start_event = event
    before = time.monotonic()
    try:
        fixture.runtime._ensure_geometry_runtime_started("activate_map", deadline_monotonic=before + 0.04)
    except GeometryRuntimeRpcError as error:
        assert error.grpc_code == grpc.StatusCode.DEADLINE_EXCEEDED
    else:
        raise AssertionError("等待超过父期限后没有退出")
    assert event.timeouts[0] is not None and not event.is_set()
    return {"waiter_deadline_preserved": True, "owner_event_cancelled": event.is_set(),
            "elapsed_ms": round((time.monotonic() - before) * 1000, 3)}


def unrelated_wait_and_close():
    fixture = Fixture()
    event = ObservedEvent("motion")
    fixture.runtime._geometry_runtime_start_event = event
    worker = spawn(lambda: fixture.runtime._ensure_geometry_runtime_started("activate_map"))
    try:
        assert event.entered.wait(1)
        assert event.timeouts == [None]
        fixture.runtime._closed = True
        closed_woke_waiter = worker[1].wait(0.03)
        assert not closed_woke_waiter
    finally:
        event.set()
    result = join(worker)
    assert result["error"].error_code == "gateway_runtime_closed"
    return {"healthy_geometry_waited_for_motion": True, "wait_timeout": None,
            "close_woke_waiter_without_owner_signal": closed_woke_waiter,
            "after_owner_signal_error": result["error"].error_code}


def same_level_failure_and_retry():
    fixture = Fixture()
    entered, release = threading.Event(), threading.Event()
    failure = GatewayError("离线激活失败", error_code="runtime_activation_failed")
    def fail(*args, **kwargs):
        fixture.activation_calls.append(args)
        entered.set()
        assert release.wait(1)
        raise failure
    fixture.runtime._request_geometry_runtime_activation = fail
    with patch.object(gateway_module, "_RuntimeActivationEvent", ObservedEvent):
        owner = spawn(lambda: fixture.runtime._ensure_geometry_runtime_started("navigate_to_pose"))
        assert entered.wait(1)
        event = fixture.runtime._geometry_runtime_start_event
        waiter = spawn(lambda: fixture.runtime._ensure_geometry_runtime_started("navigate_to_pose"))
        try:
            assert event.entered.wait(1)
        finally:
            release.set()
        owner_result, waiter_result = join(owner), join(waiter)
    assert owner_result["error"] is failure and waiter_result["error"] is failure
    assert len(fixture.activation_calls) == 1
    fixture.runtime._request_geometry_runtime_activation = fixture.activate
    fixture.runtime._ensure_geometry_runtime_started("navigate_to_pose")
    assert len(fixture.activation_calls) == 2
    return {"same_level_failure_shared": True, "later_request_can_retry": True}


def rpc_error_contracts():
    fixture = Fixture()
    fixture.stub.error = grpc.StatusCode.DEADLINE_EXCEEDED
    deadline = time.monotonic() + 2
    try:
        fixture.runtime._geometry_runtime_provider_ready(deadline_monotonic=deadline)
    except GeometryRuntimeRpcError as error:
        assert error.grpc_code == grpc.StatusCode.DEADLINE_EXCEEDED
    else:
        raise AssertionError("RPC 期限错误被吞掉")
    fixture.stub.error = grpc.StatusCode.UNAVAILABLE
    ready, message = fixture.runtime._geometry_runtime_provider_ready(deadline_monotonic=deadline)
    assert not ready
    return {"deadline_error_preserved": True, "unavailable_becomes_ready_false": not ready,
            "unavailable_message": message}


def deadline_before_getter_lock():
    fixture = Fixture()
    gate, release, attempted = threading.Event(), threading.Event(), threading.Event()
    def hold():
        with fixture.platform._runtime_state_lock:
            gate.set()
            assert release.wait(1)
    holder = spawn(hold)
    assert gate.wait(1)
    getter = fixture.platform.get_geometry_runtime_provider
    def observed_getter():
        attempted.set()
        return getter()
    fixture.platform.get_geometry_runtime_provider = observed_getter
    deadline = time.monotonic() + 0.03
    worker = spawn(lambda: fixture.runtime._geometry_runtime_provider_ready(deadline_monotonic=deadline))
    try:
        assert attempted.wait(1)
        completed_before_release = worker[1].wait(max(0, deadline - time.monotonic()) + 0.03)
        assert not completed_before_release
    finally:
        release.set()
    join(holder)
    result = join(worker)
    assert result["error"].grpc_code == grpc.StatusCode.DEADLINE_EXCEEDED
    return {"parent_deadline_did_not_bound_initial_provider_getter_lock": True,
            "get_status_dispatched": len(fixture.stub.calls)}


def map_task_context_loss():
    fixture = Fixture()
    runtime, event = fixture.runtime, ObservedEvent("motion")
    runtime._geometry_runtime_start_event = event
    record = NS(completion_committed=False, timeout_triggered=False, cancel_requested=False,
                timeout_deadline_monotonic=time.monotonic() + 5)
    captured, map_calls = {}, []
    def submit(capability, runner, **kwargs):
        captured.update(runner=runner, options=kwargs)
        return "offline-task"
    manager = NS(_lock=threading.RLock(), _records={"offline-task": record}, submit=submit,
                 update_task=lambda *args, **kwargs: None)
    context = TaskExecutionContext(manager=manager, task_id="offline-task")
    runtime.task_manager = manager
    runtime.capability_registry = NS(get_descriptor=lambda name: NS(required_resources=[]))
    runtime._resolve_async_timeout = lambda *args, **kwargs: 5
    runtime._build_task_request_metadata = lambda **kwargs: {}
    workspace = NS(activate_map=lambda **kwargs: map_calls.append(kwargs) or {"offline": True})
    skill_context = NS(ensure_geometry_runtime_started=runtime._ensure_geometry_runtime_started,
                       map_workspace_service=workspace, refresh_workspace_snapshots=lambda **kwargs: None,
                       sync_perception_runtime_for_memory_state=lambda **kwargs: None)
    runtime._submit_geometry_skill_task(capability_name="activate_map",
        skill_handler=partial(handle_activate_map, skill_context), arguments={"map_name": "offline"},
        requested_by=None, request_context=None)
    worker = spawn(lambda: captured["runner"](context))
    try:
        assert event.entered.wait(1)
        assert event.timeouts == [None]
        record.cancel_requested = True
        cancelled_woke_waiter = worker[1].wait(0.03)
        assert not cancelled_woke_waiter
        with runtime._geometry_runtime_lock:
            runtime._geometry_runtime_start_event = None
    finally:
        event.set()
    result = join(worker)
    assert "error" not in result, result.get("error")
    assert len(map_calls) == 1
    return {"task_has_deadline": True, "ensure_received_deadline": None,
            "task_cancel_woke_waiter": cancelled_woke_waiter,
            "fake_map_activation_after_cancel": len(map_calls)}


def main():
    output = {"scope": "offline production orchestration with fake RPC/activation/guard boundaries",
              "robot_connected": False, "motion_sent": False,
              "warm_navigation": warm_navigation(), "bounded_wait": bounded_wait(),
              "unrelated_wait_and_close": unrelated_wait_and_close(),
              "same_level_failure_and_retry": same_level_failure_and_retry(),
              "rpc_error_contracts": rpc_error_contracts(),
              "deadline_before_getter_lock": deadline_before_getter_lock(),
              "map_task_context_loss": map_task_context_loss()}
    print(json.dumps(output, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
