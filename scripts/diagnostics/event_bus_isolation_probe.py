#!/usr/bin/env python3
"""小规模、无设备的 EventBus 因果探针，不是负载基准或性能达标检查。

使用真实 EventBus、StreamBus、GatewayRuntime 方法及反馈投影；只替换日志
输出边界，并以线程门闩证明依赖。溢出场景把原队列容量临时减至 3，避免
大量事件。结果描述当前实现，后续修复应更新相应观测值。

运行：.nuwax-test-env/bin/python scripts/diagnostics/event_bus_isolation_probe.py
"""
from __future__ import annotations

import collections
import json
import logging
import sys
import threading
import time
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from contracts.events import RuntimeEventCategory
from contracts.naming import build_task_id
from core.capability_registry import CapabilityRegistry
from core.event_bus import EventBus
from core.logging_utils import LogRateLimiter
from core.stream_bus import StreamBus
from gateways.runtime import GatewayRuntime
import gateways.runtime as runtime_module


def event(bus, name="task.succeeded", **kwargs):
    return bus.build_event(name, category=RuntimeEventCategory.TASK,
                           source="isolation_probe", **kwargs)


def require(signal, label):
    if not signal.wait(2.0):
        raise AssertionError("等待失败：" + label)


def spawn(function):
    errors = []
    done = threading.Event()

    def run():
        try:
            function()
        except BaseException as exc:
            errors.append(exc)
        finally:
            done.set()

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread, done, errors


def finish(worker):
    thread, done, errors = worker
    require(done, "工作线程结束")
    thread.join(2.0)
    if errors:
        raise errors[0]


class GatedHandler(logging.Handler):
    def __init__(self):
        super().__init__()
        self.entered = threading.Event()
        self.release_gate = threading.Event()
        self.records = []

    def emit(self, record):
        self.records.append(record)
        self.entered.set()
        require(self.release_gate, "释放日志输出")


def runtime_logging_dependency():
    """真实投影已完成后，独立日志输出仍阻止发布者和同步等待者。"""
    bus, streams = EventBus(), StreamBus()
    runtime = object.__new__(GatewayRuntime)
    runtime.event_bus, runtime.stream_bus = bus, streams
    runtime.capability_registry = CapabilityRegistry()
    runtime._runtime_event_log_limiter = LogRateLimiter()
    sink = GatedHandler()
    feedback_logger = logging.Logger("probe.feedback", logging.INFO)
    feedback_logger.addHandler(sink)
    quiet_logger = logging.Logger("probe.runtime", logging.CRITICAL)
    woke = threading.Event()
    task_id = build_task_id("probe")
    bus.subscribe(runtime._log_runtime_event)
    bus.subscribe(lambda item: woke.set() if item.task_id == task_id else None)
    publisher = other = None
    timings = {}
    try:
        with patch.object(runtime_module, "FEEDBACK_LOGGER", feedback_logger), \
                patch.object(runtime_module, "RUNTIME_LOGGER", quiet_logger):
            def publish():
                started = time.monotonic()
                bus.publish(event(bus, task_id=task_id, payload={"state": "succeeded"}))
                timings["publisher_total_ms_including_injected_gate"] = round(
                    (time.monotonic() - started) * 1000, 3)

            publisher = spawn(publish)
            require(sink.entered, "日志输出已进入")
            frame = streams.latest_frame("tasks.state")
            assert frame is not None and frame.payload.task_id == task_id
            assert frame.metadata["feedback"]["state"] == "succeeded"
            assert not woke.is_set() and not publisher[1].is_set()

            reserved = bus.prepare(event(bus, "probe.unrelated"))
            other = spawn(lambda: bus.dispatch_prepared(reserved))
            # 门闩未释放前，同步后继游标绝不可能完成；不依赖定时睡眠。
            with bus._lock:
                pending_cursor = reserved.stored_event.cursor
                assert bus._next_dispatch_cursor == 1
            assert not other[1].is_set()
            observed = {
                "real_task_stream_projection_present_before_log_release": True,
                "waiter_woke_before_log_release": woke.is_set(),
                "publisher_finished_before_log_release": publisher[1].is_set(),
                "unrelated_prepared_cursor": pending_cursor,
                "unrelated_publisher_finished_before_log_release": other[1].is_set(),
            }
            sink.release_gate.set()
            finish(publisher)
            finish(other)
            require(woke, "日志释放后等待者唤醒")
            observed.update(timings)
            observed["feedback_records_written"] = len(sink.records)
            return observed
    finally:
        sink.release_gate.set()
        for worker in (publisher, other):
            if worker is not None:
                worker[0].join(2.0)
        bus.shutdown()
        streams.shutdown()


def async_isolation_and_unsubscribe():
    bus = EventBus()
    entered, release, second, sync = (threading.Event() for _ in range(4))
    seen = []

    def slow(item):
        entered.set()
        require(release, "释放异步订阅者")
        seen.append("slow")

    try:
        bus.subscribe(slow, async_dispatch=True)
        pending = bus.subscribe(lambda item: (seen.append("second"), second.set()),
                                async_dispatch=True)
        bus.subscribe(lambda item: sync.set())
        bus.publish(event(bus))
        require(entered, "异步消费者已进入")
        assert sync.is_set() and not second.is_set()
        bus.unsubscribe(pending)
        release.set()
        require(second, "取消订阅后的已排队回调仍执行")
        return {"sync_delivered_while_async_blocked": True,
                "second_async_delivered_while_first_blocked": False,
                "queued_callback_executed_after_unsubscribe": True,
                "callback_order": seen}
    finally:
        release.set()
        bus.shutdown()


def silent_overflow():
    bus = EventBus(retention_limit=20)
    entered, release, done = (threading.Event() for _ in range(3))
    seen = []
    # 同一生产队列/分发逻辑，小容量只用于确定性触发边界，不做负载测试。
    bus._async_queue = collections.deque(maxlen=3)

    def receive(item):
        if item.payload["index"] == 0:
            entered.set()
            require(release, "释放溢出消费者")
        seen.append(item.payload["index"])
        if item.payload["index"] == 4:
            done.set()

    try:
        bus.subscribe(receive, async_dispatch=True)
        bus.publish(event(bus, payload={"index": 0}))
        require(entered, "溢出消费者已暂停")
        for index in range(1, 5):
            bus.publish(event(bus, payload={"index": index}))
        assert len(bus._async_queue) == 3
        release.set()
        require(done, "溢出队列排空")
        assert seen == [0, 2, 3, 4]
        replay = bus.replay()
        assert [item.payload["index"] for item in replay] == list(range(5))
        return {"test_capacity": 3, "terminal_events_published": 5,
                "terminal_indexes_received": seen,
                "terminal_indexes_retained_in_replay": list(range(5)),
                "overflow_diagnostic_events": sum(
                    item.event_type.startswith("event_bus.") for item in replay)}
    finally:
        release.set()
        bus.shutdown()


def ordering_and_failures():
    bus = EventBus()
    seen = []

    def reentrant(item):
        seen.append(["first", item.cursor])
        if item.event_type == "probe.outer":
            bus.publish(event(bus, "probe.inner"))

    try:
        bus.subscribe(reentrant)
        bus.subscribe(lambda item: seen.append(["second", item.cursor]))
        bus.publish(event(bus, "probe.outer"))
        assert seen == [["first", 1], ["second", 1], ["first", 2], ["second", 2]]

        def fail(item):
            raise RuntimeError("injected")

        broken = bus.subscribe(fail)
        bus.publish(event(bus, "probe.failure"))
        bus.unsubscribe(broken)
        diagnostics = [item for item in bus.replay()
                       if item.event_type == "event_bus.subscription_failed"]
        assert len(diagnostics) == 1
        assert all(cursor != diagnostics[0].cursor for _, cursor in seen)
        return {"reentrant_callbacks_ordered_by_cursor": True,
                "synchronous_failure_propagated_to_publisher": False,
                "failure_diagnostic_retained": True,
                "failure_diagnostic_delivered_to_live_subscribers": False}
    finally:
        bus.shutdown()


def reserved_cursor_and_close():
    bus = EventBus()
    sync_seen, async_seen = [], []
    first = bus.prepare(event(bus, "probe.reserved"))
    second = bus.prepare(event(bus, "probe.queued"))
    worker = spawn(lambda: bus.dispatch_prepared(second))
    try:
        assert not worker[1].is_set()
        assert bus.abandon_prepared(first)
        finish(worker)
        bus.subscribe(lambda item: sync_seen.append(item.event_type))
        bus.subscribe(lambda item: async_seen.append(item.event_type), async_dispatch=True)
        bus.shutdown()
        bus.publish(event(bus, "probe.after_shutdown"))
        assert sync_seen == ["probe.after_shutdown"]
        assert async_seen == [] and len(bus._async_queue) == 1
        return {"abandon_unblocks_later_reserved_cursor": True,
                "publish_after_shutdown_accepted": True,
                "sync_callback_after_shutdown": sync_seen,
                "undeliverable_async_items_after_shutdown": len(bus._async_queue)}
    finally:
        bus.abandon_prepared(first)
        worker[0].join(2.0)
        bus.shutdown()


def main():
    result = {
        "evidence_kind": "production_methods_with_deterministic_injected_boundaries",
        "is_load_benchmark": False,
        "hardware_or_model_used": False,
        "runtime_logging_dependency": runtime_logging_dependency(),
        "async_isolation_and_unsubscribe": async_isolation_and_unsubscribe(),
        "silent_overflow": silent_overflow(),
        "ordering_and_failures": ordering_and_failures(),
        "reserved_cursor_and_close": reserved_cursor_and_close(),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
