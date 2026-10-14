#!/usr/bin/env python3
"""真实运行时方法的日志隔离与有界队列验证，不连接设备、网络或模型。"""
from __future__ import annotations

import logging
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contracts.events import RuntimeEventCategory
from contracts.naming import build_task_id
from core.capability_registry import CapabilityRegistry
from core.event_bus import EventBus
from core.logging_utils import CorrelationContextFilter, LogRateLimiter, StreamBusLogHandler, correlation_context
from core.stream_bus import StreamBus
from gateways import feedback_messages
from gateways import runtime as runtime_module
from gateways.runtime import GatewayRuntime
from gateways.runtime_event_log import RuntimeEventLog


class GateHandler(logging.Handler):
    def __init__(self):
        super().__init__()
        self.entered = threading.Event()
        self.resume = threading.Event()
        self.records = []
        self.addFilter(CorrelationContextFilter())

    def emit(self, record):
        self.records.append(record)
        self.entered.set()
        if not self.resume.wait(3):
            raise AssertionError("日志输出门闩没有释放")


class EventLogIsolation(unittest.TestCase):
    def setUp(self):
        self.owners, self.buses, self.streams, self.gates, self.workers = [], [], [], [], []

    def tearDown(self):
        for gate in self.gates:
            gate.set()
        for thread, failures, _ in self.workers:
            thread.join(3)
            self.assertFalse(thread.is_alive())
            self.assertEqual(failures, [])
        for owner in self.owners:
            self.assertTrue(owner.close(), owner.status())
        for bus in self.buses:
            self.assertTrue(bus.shutdown())
        for stream in self.streams:
            stream.shutdown()

    def gate(self):
        gate = threading.Event()
        self.gates.append(gate)
        return gate

    def require(self, gate):
        self.assertTrue(gate.wait(2), "门闩没有触发")

    def spawn(self, call):
        failures, results = [], []
        def run():
            try:
                results.append(call())
            except BaseException as error:  # noqa: BLE001 - 测试线程完整转交异常。
                failures.append(error)
        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        worker = thread, failures, results
        self.workers.append(worker)
        return worker

    def join(self, worker):
        worker[0].join(2)
        self.assertFalse(worker[0].is_alive(), "发布者被日志输出阻塞")
        self.assertEqual(worker[1], [])
        return worker[2][0]

    def owner(self, writer, **kwargs):
        owner = RuntimeEventLog(writer, **kwargs)
        self.owners.append(owner)
        return owner

    def bus(self):
        bus = EventBus()
        self.buses.append(bus)
        return bus

    def event(self, index=1, *, name="task.succeeded", payload=None, task_id=None):
        # 只构造契约，不为辅助样本创建运行时或启动线程。
        from contracts.events import RuntimeEvent
        from contracts.naming import build_event_id
        return RuntimeEvent(event_id=build_event_id(name), event_type=name,
                            category=RuntimeEventCategory.TASK, source="isolation_check",
                            cursor=index, task_id=task_id, payload=payload or {"index": index})

    def runtime(self, **owner_options):
        runtime = object.__new__(GatewayRuntime)
        runtime.event_bus = self.bus()
        runtime.stream_bus = StreamBus()
        self.streams.append(runtime.stream_bus)
        runtime.capability_registry = CapabilityRegistry()
        runtime._runtime_event_log_limiter = LogRateLimiter()
        runtime._runtime_event_log = self.owner(runtime._write_runtime_event_log,
                                              gap_writer=runtime._write_runtime_event_log_gap,
                                              **owner_options)
        runtime._event_log_subscription_id = runtime.event_bus.subscribe(runtime._log_runtime_event)
        runtime._owns_event_bus = runtime._owns_stream_bus = True
        runtime._dispatch_shutdown_complete = False
        return runtime

    def blocked_owner(self, **kwargs):
        entered, resume = self.gate(), self.gate()
        seen = []
        def writer(event, feedback):
            if not seen:
                entered.set()
                self.require(resume)
            seen.append((event, feedback))
        owner = self.owner(writer, **kwargs)
        self.assertTrue(owner.submit(self.event(1)))
        self.require(entered)
        return owner, resume, seen

    def test_real_bridge_sink_block_does_not_block_publisher_waiter_or_terminal_consumer(self):
        runtime = self.runtime()
        sink = GateHandler()
        self.gates.append(sink.resume)
        feedback_logger = logging.Logger("test.feedback", logging.INFO)  # noqa: LOG001 - 不修改全局日志注册表。
        feedback_logger.addHandler(sink)
        quiet = logging.Logger("test.runtime", logging.CRITICAL)  # noqa: LOG001 - 独立测试日志。
        task_id = build_task_id("capture_image")
        woke, terminal = self.gate(), self.gate()
        observed_frames = []
        def wait_callback(event):
            if event.task_id == task_id:
                observed_frames.append(runtime.stream_bus.latest_frame("tasks.state"))
                woke.set()
        runtime.event_bus.subscribe(wait_callback)
        runtime.event_bus.subscribe(lambda e: terminal.set(), event_types=["task.succeeded"], async_dispatch=True)
        with patch.object(runtime_module, "FEEDBACK_LOGGER", feedback_logger), \
                patch.object(runtime_module, "RUNTIME_LOGGER", quiet), \
                patch.object(feedback_messages, "build_feedback_message", wraps=feedback_messages.build_feedback_message) as project:
            published = self.spawn(lambda: runtime.event_bus.publish(
                self.event(task_id=task_id, payload={"state": "succeeded"})))
            self.require(sink.entered)
            stored = self.join(published)
            self.require(woke)
            self.require(terminal)
            self.assertEqual(project.call_count, 1)
            frame = observed_frames[0]
            self.assertIs(frame.payload, stored)
            self.assertEqual(frame.sequence, stored.cursor)
            self.assertEqual(frame.metadata["feedback"]["state"], "succeeded")
            self.assertEqual(runtime.get_event_log_status()["in_flight_cursor"], stored.cursor)
            unrelated = self.spawn(lambda: runtime.event_bus.publish(self.event(2, name="robot.state_refreshed")))
            self.join(unrelated)
            sink.resume.set()
            self.assertTrue(runtime._runtime_event_log.close())
            self.assertEqual(sink.records[0].feedback, frame.metadata["feedback"])
            self.assertEqual(runtime._runtime_event_log.status()["writer_failed"], 0)

    def test_count_bound_includes_blocked_inflight_and_gap_is_observable(self):
        gaps = []
        owner, resume, seen = self.blocked_owner(max_items=2, reserved_items=0, reserved_bytes=0, gap_writer=gaps.append)
        self.assertTrue(owner.submit(self.event(2)))
        self.assertFalse(owner.submit(self.event(3)))
        status = owner.status()
        self.assertEqual(status["outstanding_items"], 2)
        self.assertEqual(status["protected_dropped"], 1)
        self.assertEqual(status["pending_gap"]["first_missing_cursor"], 3)
        self.assertTrue(status["degraded"])
        self.assertFalse(owner.close(0.01))
        self.assertEqual(owner.status()["outstanding_items"], 2)
        resume.set()
        self.assertTrue(owner.close())
        self.assertEqual([event.cursor for event, _ in seen], [1, 2])
        self.assertEqual(gaps[0]["missing_count"], 1)
        self.assertEqual(owner.status()["outstanding_bytes"], 0)

    def test_byte_bound_includes_blocked_inflight(self):
        first, second = self.event(1), self.event(2)
        cap = len(RuntimeEventLog._capture(first, None)) + len(RuntimeEventLog._capture(second, None))
        entered, resume = self.gate(), self.gate()
        owner = self.owner(lambda e, f: (entered.set(), self.require(resume)),
                           max_items=8, max_bytes=cap, reserved_bytes=0, reserved_items=0)
        self.assertTrue(owner.submit(first))
        self.require(entered)
        self.assertTrue(owner.submit(second))
        self.assertEqual(owner.status()["outstanding_bytes"], cap)
        self.assertFalse(owner.submit(self.event(3)))
        self.assertLessEqual(owner.status()["high_water_bytes"], cap)
        resume.set()

    def test_telemetry_capacity_reserves_room_for_terminal_without_reordering(self):
        entered, resume = self.gate(), self.gate()
        seen = []
        def writer(event, feedback):
            entered.set()
            self.require(resume)
            seen.append(event.cursor)
        owner = self.owner(writer, max_items=3, reserved_items=1, reserved_bytes=0)
        self.assertTrue(owner.submit(self.event(1, name="localization.snapshot_ready")))
        self.require(entered)
        self.assertTrue(owner.submit(self.event(2, name="mapping.snapshot_ready")))
        self.assertFalse(owner.submit(self.event(3, name="robot.state_refreshed")))
        self.assertTrue(owner.submit(self.event(4)))
        self.assertFalse(owner.submit(self.event(5)))
        self.assertEqual(owner.status()["telemetry_dropped"], 1)
        self.assertEqual(owner.status()["protected_dropped"], 1)
        resume.set()
        self.assertTrue(owner.close())
        self.assertEqual(seen, [1, 2, 4])

    def test_payload_snapshot_is_bounded_redacted_and_not_mutable_by_publisher(self):
        owner, resume, seen = self.blocked_owner()
        event = self.event(2, payload={"stage": "before", "password": "secret", "values": list(range(5000))})
        feedback = {"state": "succeeded", "message_id": "message-2", "message": "完成"}
        self.assertTrue(owner.submit(event, feedback))
        event.payload["stage"] = "after"
        feedback["message"] = "changed"
        self.assertLessEqual(owner.status()["outstanding_bytes"], 2 * owner.status()["max_item_bytes"])
        resume.set()
        self.assertTrue(owner.close())
        copied, copied_feedback = seen[1]
        self.assertEqual(copied.payload["stage"], "before")
        self.assertEqual(copied.payload["password"], "********")
        self.assertLessEqual(len(copied.payload["values"]), 32)
        self.assertEqual(copied_feedback["message"], "完成")

    def test_long_multibyte_message_cannot_evict_event_identity(self):
        seen = []
        owner = self.owner(lambda event, feedback: seen.append(event))
        event = self.event(42, name="robot.state_refreshed").model_copy(update={"message": "中文消息" * 1000})
        self.assertTrue(owner.submit(event))
        self.assertTrue(owner.close())
        saved = seen[0]
        for name in ("event_id", "event_type", "cursor", "source", "subject_id", "task_id"):
            self.assertEqual(getattr(saved, name), getattr(event, name))
        self.assertEqual(saved.severity, str(event.severity))
        self.assertEqual(saved.timestamp, event.timestamp.isoformat())
        self.assertLessEqual(len(saved.message), 1800)

    def test_unbounded_identity_is_rejected_explicitly(self):
        seen = []
        owner = self.owner(lambda event, feedback: seen.append(event))
        event = self.event().model_copy(update={"source": "component" * 1000})
        self.assertFalse(owner.submit(event))
        self.assertTrue(owner.close())
        self.assertEqual(seen, [])
        self.assertEqual(owner.status()["preparation_failed"], 1)

    def test_correlation_is_captured_without_context_leak(self):
        captures = []
        def writer(event, feedback):
            record = SimpleNamespace()
            CorrelationContextFilter().filter(record)
            captures.append(vars(record))
        owner = self.owner(writer)
        with correlation_context(request_id="request-1", task_id="task-1", capability_name="capture_image"):
            self.assertTrue(owner.submit(self.event(1)))
        self.assertTrue(owner.submit(self.event(2)))
        self.assertTrue(owner.close())
        self.assertEqual(captures[0]["request_id"], "request-1")
        self.assertEqual(captures[0]["task_id"], "task-1")
        self.assertEqual(captures[0]["capability_name"], "capture_image")
        self.assertEqual(captures[1], {})

    def test_oversized_item_is_explicitly_rejected(self):
        owner = self.owner(lambda e, f: None, max_item_bytes=10)
        self.assertFalse(owner.submit(self.event()))
        self.assertTrue(owner.close())
        self.assertEqual(owner.status()["oversize_rejected"], 1)
        self.assertEqual(owner.status()["accepted"], 0)

    def test_failed_writer_and_gap_writer_do_not_recurse_or_kill_worker(self):
        seen = []
        def writer(event, feedback):
            if event.cursor == 1:
                raise RuntimeError("输出失败")
            seen.append(event.cursor)
        def gap_writer(gap):
            raise RuntimeError("诊断输出也失败")
        owner = self.owner(writer, gap_writer=gap_writer)
        owner.submit(self.event(1))
        owner.submit(self.event(2))
        self.assertTrue(owner.close())
        status = owner.status()
        self.assertEqual(seen, [2])
        self.assertEqual(status["writer_failed"], 1)
        self.assertGreaterEqual(status["gap_writer_failed"], 1)
        self.assertEqual(status["missing_count"], 1)
        self.assertEqual(status["outstanding_items"], 0)

    def test_close_from_writer_returns_false_and_admitted_jobs_drain(self):
        entered, resume = self.gate(), self.gate()
        seen, closes = [], []
        owner = None
        def writer(event, feedback):
            if event.cursor == 1:
                entered.set()
                self.require(resume)
                closes.append(owner.close())
            seen.append(event.cursor)
        owner = self.owner(writer)
        owner.submit(self.event(1))
        self.require(entered)
        owner.submit(self.event(2))
        resume.set()
        self.assertTrue(owner.close())
        self.assertEqual(closes, [False])
        self.assertEqual(seen, [1, 2])

    def test_reject_after_close_does_not_leave_undeliverable_gap_work(self):
        owner = self.owner(lambda e, f: None)
        self.assertTrue(owner.close())
        self.assertFalse(owner.submit(self.event()))
        self.assertEqual(owner.status()["closed_rejected"], 1)
        self.assertIsNone(owner.status()["pending_gap"])
        self.assertTrue(owner.status()["closed"])

    def test_runtime_close_keeps_log_dependencies_until_blocked_writer_finishes(self):
        runtime = self.runtime()
        sink = GateHandler()
        self.gates.append(sink.resume)
        logger = logging.Logger("test.feedback", logging.INFO)  # noqa: LOG001 - 独立测试日志。
        logger.addHandler(sink)
        runtime._log_stream_handler = StreamBusLogHandler(runtime.stream_bus)
        logging.getLogger().addHandler(runtime._log_stream_handler)
        self.addCleanup(logging.getLogger().removeHandler, runtime._log_stream_handler)
        real_close = runtime._runtime_event_log.close
        with patch.object(runtime_module, "FEEDBACK_LOGGER", logger), \
                patch.object(runtime_module, "RUNTIME_LOGGER", logging.Logger("quiet", logging.CRITICAL)):  # noqa: LOG001 - 独立测试日志。
            runtime.event_bus.publish(self.event(task_id=build_task_id("capture_image")))
            self.require(sink.entered)
            with patch.object(runtime._runtime_event_log, "close", side_effect=lambda *a, **kw: real_close(0.01)):
                self.assertFalse(runtime._close_dispatch_resources())
            self.assertIn(runtime._log_stream_handler, logging.getLogger().handlers)
            self.assertTrue(runtime.stream_bus._consumer_thread.is_alive())
            self.assertTrue(runtime.event_bus.status()["accepting"])
            self.assertFalse(runtime._dispatch_shutdown_complete)
            sink.resume.set()
            self.assertTrue(runtime._close_dispatch_resources())
            self.assertNotIn(runtime._log_stream_handler, logging.getLogger().handlers)
            self.assertFalse(runtime.stream_bus._consumer_thread.is_alive())
            self.assertTrue(runtime._dispatch_shutdown_complete)

    def test_shared_bus_close_waits_for_claimed_but_not_entered_bridge(self):
        runtime = self.runtime()
        runtime._owns_event_bus = False
        runtime.event_bus.unsubscribe(runtime._event_log_subscription_id)
        entered, resume = self.gate(), self.gate()
        def claimed_bridge(event):
            entered.set()
            self.require(resume)
            runtime._log_runtime_event(event)
        identity = runtime._event_log_subscription_id = runtime.event_bus.subscribe(claimed_bridge)
        publisher = self.spawn(lambda: runtime.event_bus.publish(self.event()))
        self.require(entered)
        self.assertFalse(runtime._close_dispatch_resources(timeout_sec=0.01))
        self.assertEqual(runtime._closing_event_log_subscription_id, identity)
        self.assertTrue(runtime.stream_bus._consumer_thread.is_alive())
        self.assertTrue(runtime._runtime_event_log.status()["accepting"])
        self.assertEqual(runtime.get_event_log_status()["dispatch_state"], "stopping")
        resume.set()
        stored = self.join(publisher)
        self.assertIs(runtime.stream_bus.latest_frame("tasks.state").payload, stored)
        self.assertEqual(runtime._runtime_event_log.status()["closed_rejected"], 0)
        self.assertTrue(runtime._close_dispatch_resources())
        self.assertFalse(runtime.stream_bus._consumer_thread.is_alive())
        self.assertTrue(runtime.event_bus.status()["accepting"])
        self.assertEqual(runtime.event_bus.status()["callback_failures"], 0)
        self.assertEqual(runtime.get_event_log_status()["dispatch_state"], "stopped")
        self.assertIsNone(runtime._closing_event_log_subscription_id)

    def test_bridge_reentrant_close_keeps_dependencies_and_returns_false(self):
        runtime = self.runtime()
        closed = []
        project = runtime._feedback_for_event
        def projection(event):
            closed.append(runtime._close_dispatch_resources(timeout_sec=1.0))
            return project(event)
        with patch.object(runtime, "_feedback_for_event", side_effect=projection):
            stored = runtime.event_bus.publish(self.event())
        self.assertEqual(closed, [False])
        self.assertIs(runtime.stream_bus.latest_frame("tasks.state").payload, stored)
        self.assertTrue(runtime._close_dispatch_resources())

    def test_projection_failure_is_visible_without_running_log_handler_in_publisher(self):
        runtime = self.runtime()
        seen = []
        runtime.event_bus.subscribe(seen.append)
        with patch.object(runtime, "_feedback_for_event", side_effect=ValueError("投影失败")):
            stored = runtime.event_bus.publish(self.event())
        self.assertEqual([event.cursor for event in seen], [stored.cursor])
        self.assertEqual(runtime._runtime_event_log.status()["preparation_failed"], 1)
        self.assertEqual(len(runtime.event_bus.replay(event_types=["event_bus.subscription_failed"])), 1)

    def test_status_exposes_log_gap_to_runtime_reader(self):
        runtime = self.runtime(max_item_bytes=1)
        runtime.mapping_service = SimpleNamespace(notification_status=lambda: {})
        runtime.event_bus.publish(self.event())
        status = runtime.get_live_map_stream_status()["event_log"]
        self.assertTrue(status["degraded"])
        self.assertEqual(status["oversize_rejected"], 1)

    def test_close_timeout_validation(self):
        owner = self.owner(lambda e, f: None)
        for value in (-1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                owner.close(value)
        self.assertTrue(owner.status()["accepting"])

    def test_uninitialized_status_does_not_invent_zero_success_counters(self):
        runtime = object.__new__(GatewayRuntime)
        self.assertEqual(runtime.get_event_log_status(),
                         {"state": "not_started", "accepting": False, "closed": False})

    def test_closed_projection_failure_cannot_enqueue_unserviceable_gap(self):
        owner = self.owner(lambda e, f: None)
        self.assertTrue(owner.close())
        owner.note_projection_failure(4, "ValueError")
        self.assertIsNone(owner.status()["pending_gap"])
        self.assertEqual(owner.status()["closed_rejected"], 1)

    def test_stop_retries_incomplete_dispatch_close_without_changing_memory_close(self):
        runtime = object.__new__(GatewayRuntime)
        runtime._lifecycle_lock = threading.RLock()
        runtime._started, runtime._closed = False, True
        runtime._dispatch_shutdown_complete = False
        runtime.mapping_recorder = None
        actions = []
        runtime.task_manager = SimpleNamespace(shutdown=lambda **kw: actions.append("tasks"))
        runtime._join_exploration_finalizers = lambda: actions.append("finalizers")
        runtime.memory_service = SimpleNamespace(close=lambda: actions.append("memory") or True)
        results = [False, True]
        def close_dispatch():
            actions.append("dispatch")
            result = results.pop(0)
            runtime._dispatch_shutdown_complete = result
            return result
        runtime._close_dispatch_resources = close_dispatch
        with self.assertRaisesRegex(RuntimeError, "事件分发或日志输出尚未关闭"):
            runtime.stop()
        self.assertFalse(runtime._dispatch_shutdown_complete)
        self.assertTrue(runtime._memory_shutdown_complete)
        self.assertTrue(runtime._closed)
        with self.assertRaisesRegex(RuntimeError, "已关闭"):
            runtime.start()
        runtime.stop()
        self.assertTrue(runtime._dispatch_shutdown_complete)
        self.assertTrue(runtime._memory_shutdown_complete)
        self.assertEqual(actions.count("dispatch"), 2)
        self.assertEqual(actions.count("memory"), 2)

    def test_close_during_snapshot_preparation_rejects_without_orphan_gap(self):
        owner = self.owner(lambda e, f: None)
        entered, resume = self.gate(), self.gate()
        capture = owner._capture
        def slow_capture(event, feedback):
            entered.set()
            self.require(resume)
            return capture(event, feedback)
        with patch.object(owner, "_capture", side_effect=slow_capture):
            submit = self.spawn(lambda: owner.submit(self.event()))
            self.require(entered)
            self.assertTrue(owner.close())
            resume.set()
            self.assertFalse(self.join(submit))
        self.assertIsNone(owner.status()["pending_gap"])
        self.assertEqual(owner.status()["closed_rejected"], 1)

    def test_saturated_log_queue_does_not_hide_live_terminal_event(self):
        runtime = self.runtime(max_items=1, reserved_items=0)
        entered, resume = self.gate(), self.gate()
        original = runtime._runtime_event_log._writer
        def gated_writer(event, feedback):
            entered.set()
            self.require(resume)
            original(event, feedback)
        runtime._runtime_event_log._writer = gated_writer
        seen = []
        runtime.event_bus.subscribe(seen.append)
        with patch.object(runtime_module, "FEEDBACK_LOGGER", logging.getLogger("check.silent.feedback")), \
                patch.object(runtime_module, "RUNTIME_LOGGER", logging.getLogger("check.silent.runtime")):
            runtime.event_bus.publish(self.event(1, name="task.accepted"))
            self.require(entered)
            terminal = runtime.event_bus.publish(self.event(2, payload={"state": "succeeded"}))
            self.assertEqual(seen[-1].event_type, "task.succeeded")
            self.assertIs(runtime.stream_bus.latest_frame("tasks.state").payload, terminal)
            self.assertEqual(runtime.get_event_log_status()["protected_dropped"], 1)
            self.assertEqual(runtime.get_event_log_status()["last_missing_cursor"], terminal.cursor)
            resume.set()

    def test_blocked_gap_writer_does_not_hold_queue_or_health_lock(self):
        entered, resume = self.gate(), self.gate()
        gaps = []
        def gap_writer(gap):
            gaps.append(gap)
            entered.set()
            self.require(resume)
        owner = self.owner(lambda e, f: None, gap_writer=gap_writer, max_item_bytes=1)
        self.assertFalse(owner.submit(self.event(1)))
        self.require(entered)
        self.assertFalse(owner.submit(self.event(2)))
        self.assertFalse(owner.submit(self.event(3)))
        status = owner.status()
        self.assertTrue(status["gap_in_flight"])
        self.assertEqual(status["pending_gap"]["missing_count"], 2)
        self.assertEqual(status["pending_gap"]["first_missing_cursor"], 2)
        self.assertEqual(status["pending_gap"]["last_missing_cursor"], 3)
        self.assertFalse(owner.close(0.01))
        resume.set()
        self.assertTrue(owner.close())
        self.assertEqual([gap["missing_count"] for gap in gaps], [1, 2])


if __name__ == "__main__":
    unittest.main(verbosity=2)
