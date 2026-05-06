from __future__ import annotations

from datetime import timedelta
import threading
import time

import pytest

from contracts.base import utc_now
from contracts.capabilities import (
    CapabilityAvailability,
    CapabilityDescriptor,
    CapabilityExecutionMode,
    CapabilityMatrix,
    CapabilityRiskLevel,
)
from contracts.events import RuntimeEventCategory
from contracts.robot_state import RobotControlMode, RobotState
from contracts.tasks import TaskState
from core import (
    CapabilityRegistry,
    EventBus,
    ResourceConflictError,
    ResourceLockManager,
    RuntimeResource,
    SafetyGuard,
    StateNamespace,
    StateStore,
    StreamBus,
    TaskManager,
)
from typing import Dict, List, Optional, Tuple


def _wait_for_state(task_manager: TaskManager, task_id: str, target_state: TaskState, timeout_sec: float = 2.0) -> None:
    """等待任务进入目标状态。"""

    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if task_manager.get_task_status(task_id).state == target_state:
            return
        time.sleep(0.01)
    raise AssertionError(f"任务 {task_id} 未在 {timeout_sec} 秒内进入状态 {target_state}。")


def test_capability_registry_supports_matrix_and_runtime_views() -> None:
    """能力注册表应支持模式区分和机器人矩阵对接。"""

    registry = CapabilityRegistry()
    registry.register(
        CapabilityDescriptor(
            name="relative_move",
            display_name="相对移动",
            description="执行短距离相对移动。",
            execution_mode=CapabilityExecutionMode.SYNC,
            risk_level=CapabilityRiskLevel.MEDIUM,
            required_resources=["base_motion"],
            timeout_sec=10,
            cancel_supported=False,
            exposed_to_agent=True,
        ),
        handler=lambda payload: payload,
        owner="motion_skill",
    )
    registry.register(
        CapabilityDescriptor(
            name="navigate_to_pose",
            display_name="导航到位姿",
            description="执行导航任务。",
            execution_mode=CapabilityExecutionMode.ASYNC_TASK,
            risk_level=CapabilityRiskLevel.MEDIUM,
            required_resources=["base_motion", "navigation"],
            timeout_sec=300,
            cancel_supported=True,
            exposed_to_agent=True,
        ),
        handler=lambda ctx: None,
        owner="navigation_skill",
    )
    registry.bind_robot_capability_matrix(
        CapabilityMatrix(
            robot_model="go2",
            capabilities=[
                CapabilityAvailability(name="relative_move", supported=True),
                CapabilityAvailability(name="navigate_to_pose", supported=False, reason="当前未接入导航栈。"),
            ],
        )
    )

    views = registry.build_runtime_views(robot_model="go2", exposed_only=True)
    relative_move = next(view for view in views if view.name == "relative_move")
    navigate_to_pose = next(view for view in views if view.name == "navigate_to_pose")

    assert registry.supports_async("navigate_to_pose") is True
    assert relative_move.supported is True
    assert relative_move.descriptor.required_resources == ["base_motion"]
    assert navigate_to_pose.supported is False
    assert navigate_to_pose.reason == "当前未接入导航栈。"
    assert navigate_to_pose.execution_mode == CapabilityExecutionMode.ASYNC_TASK


def test_event_bus_supports_publish_subscribe_and_cursor_replay() -> None:
    """事件总线应保持顺序并支持游标续播。"""

    event_bus = EventBus(retention_limit=8)
    received: List[Tuple[Optional[int], str]] = []
    subscription_id = event_bus.subscribe(
        lambda event: received.append((event.cursor, event.event_type)),
        categories=[RuntimeEventCategory.TASK],
    )

    event_bus.publish(
        event_bus.build_event("task.accepted", category=RuntimeEventCategory.TASK, source="test_runtime")
    )
    event_bus.publish(
        event_bus.build_event("task.progress", category=RuntimeEventCategory.TASK, source="test_runtime")
    )
    event_bus.publish(
        event_bus.build_event("safety.stop_triggered", category=RuntimeEventCategory.SAFETY, source="test_runtime")
    )

    replayed = event_bus.replay(after_cursor=1)
    event_bus.unsubscribe(subscription_id)
    event_bus.publish(
        event_bus.build_event("task.succeeded", category=RuntimeEventCategory.TASK, source="test_runtime")
    )

    assert received == [(1, "task.accepted"), (2, "task.progress")]
    assert [event.event_type for event in replayed] == ["task.progress", "safety.stop_triggered"]
    assert event_bus.latest_cursor() == 4


def test_stream_bus_latest_frames_survive_retention_window() -> None:
    """流总线应能按 topic 取最新帧，即使该帧已被历史窗口淘汰。"""

    stream_bus = StreamBus(retention_limit=2)
    navigation_frame = stream_bus.publish("navigation.state", source="unit_test", payload={"status": "idle"})
    stream_bus.publish("map.frame", source="unit_test", payload={"revision": 1}, revision="1")
    stream_bus.publish("localization.pose", source="unit_test", payload={"x": 0.0})
    latest_map_frame = stream_bus.publish("map.frame", source="unit_test", payload={"revision": 2}, revision="2")

    retained_topics = [frame.topic for frame in stream_bus.replay(after_cursor=0)]
    latest_topics = [
        frame.topic
        for frame in stream_bus.latest_frames(topics=("navigation.state", "map.frame"))
    ]

    assert retained_topics == ["localization.pose", "map.frame"]
    assert latest_topics == ["navigation.state", "map.frame"]
    assert stream_bus.latest_frame("navigation.state") == navigation_frame
    assert stream_bus.latest_frame("map.frame") == latest_map_frame
    assert stream_bus.oldest_cursor() == 3


def test_task_manager_runs_successful_task_and_emits_events() -> None:
    """任务管理器应完成标准 accepted->running->progress->succeeded 流程。"""

    event_bus = EventBus()
    task_manager = TaskManager(event_bus=event_bus)

    def runner(context) -> Dict[str, bool]:
        context.update(progress=0.5, stage="moving", message="正在执行移动。")
        return {"ok": True}

    spec = task_manager.submit(
        "relative_move",
        runner,
        input_payload={"vx": 0.1},
        required_resources=[RuntimeResource.BASE_MOTION.value],
    )
    result = task_manager.wait_for_task(spec.task_id, timeout=2.0)
    status = task_manager.get_task_status(spec.task_id)
    event_types = [event.event_type for event in task_manager.get_task_events(spec.task_id)]
    replayed = event_bus.replay(after_cursor=0, categories=[RuntimeEventCategory.TASK])

    assert result == {"ok": True}
    assert status.state == TaskState.SUCCEEDED
    assert status.progress == 1.0
    assert event_types == ["task.accepted", "task.started", "task.progress", "task.succeeded"]
    assert [event.event_type for event in replayed] == event_types

    task_manager.shutdown()


def test_task_manager_handles_cancel_and_timeout() -> None:
    """任务管理器应支持取消和超时。"""

    task_manager = TaskManager()
    cancel_started = threading.Event()
    timeout_started = threading.Event()

    def cancellable_runner(context) -> None:
        cancel_started.set()
        while True:
            time.sleep(0.01)
            context.ensure_active()

    def timeout_runner(context) -> None:
        timeout_started.set()
        while True:
            time.sleep(0.01)
            context.ensure_active()

    cancel_spec = task_manager.submit("navigate_to_pose", cancellable_runner, timeout_sec=1.0)
    assert cancel_started.wait(timeout=1.0) is True
    assert task_manager.cancel_task(cancel_spec.task_id) is True
    task_manager.wait_for_task(cancel_spec.task_id, timeout=1.0)
    _wait_for_state(task_manager, cancel_spec.task_id, TaskState.CANCELLED)

    timeout_spec = task_manager.submit("explore_area", timeout_runner, timeout_sec=0.1)
    assert timeout_started.wait(timeout=1.0) is True
    task_manager.wait_for_task(timeout_spec.task_id, timeout=1.0)
    _wait_for_state(task_manager, timeout_spec.task_id, TaskState.TIMEOUT)

    assert task_manager.get_task_status(cancel_spec.task_id).state == TaskState.CANCELLED
    assert task_manager.get_task_status(timeout_spec.task_id).state == TaskState.TIMEOUT

    task_manager.shutdown()


def test_resource_conflict_and_safety_guard_work_together() -> None:
    """资源冲突应被拒绝，安全停机应释放运动资源。"""

    event_bus = EventBus()
    lock_manager = ResourceLockManager()
    safety_guard = SafetyGuard(event_bus=event_bus, resource_lock_manager=lock_manager)
    stop_reasons: List[Optional[str]] = []

    lock_manager.acquire("task_a", [RuntimeResource.BASE_MOTION.value])
    with pytest.raises(ResourceConflictError):
        lock_manager.acquire("task_b", [RuntimeResource.BASE_MOTION.value])

    safety_guard.register_stop_handler("base_motion_stop", lambda reason: stop_reasons.append(reason))
    safety_guard.stop_all_motion("手动触发安全停机。")

    assert stop_reasons == ["手动触发安全停机。"]
    assert lock_manager.is_locked(RuntimeResource.BASE_MOTION.value) is False
    assert event_bus.replay(after_cursor=0)[0].event_type == "safety.stop_triggered"


def test_watchdog_and_state_store_behave_as_expected() -> None:
    """看门狗和状态存储应具备最小可用语义。"""

    event_bus = EventBus()
    safety_guard = SafetyGuard(event_bus=event_bus)
    state_store = StateStore()
    stop_reasons: List[Optional[str]] = []

    safety_guard.register_stop_handler("noop", lambda reason: stop_reasons.append(reason))
    safety_guard.register_watchdog("perception_loop", timeout_sec=0.01, metadata={"component": "perception"})
    stale = safety_guard.check_watchdogs(now=utc_now() + timedelta(seconds=0.02))

    robot_state = RobotState(robot_id="go2", frame_id="world/go2/base", mode=RobotControlMode.HIGH_LEVEL)
    state_store.write_from_provider("go2_provider", StateNamespace.ROBOT_STATE, "go2", robot_state)
    latest = state_store.read_latest(StateNamespace.ROBOT_STATE)

    assert len(stale) == 1
    assert stop_reasons and "watchdog 超时" in (stop_reasons[0] or "")
    assert latest is not None
    assert latest.source == "go2_provider"
    assert latest.value.robot_id == "go2"
    assert event_bus.replay(after_cursor=0)[0].category == RuntimeEventCategory.SAFETY
