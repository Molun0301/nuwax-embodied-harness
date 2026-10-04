#!/usr/bin/env python3
"""离线验证事件总线的游标、缺口、取消订阅与有界关闭契约。

仅使用小容量队列及线程门闩，不连接设备、模型或网络，不是负载基准。
运行：.nuwax-test-env/bin/python scripts/check_event_bus_contracts.py
"""
from __future__ import annotations

import sys
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contracts.events import RuntimeEventCategory
from core.event_bus import EventBus
from core.task_manager import TaskManager


class EventBusContracts(unittest.TestCase):
    def setUp(self):
        self.buses = []
        self.gates = []
        self.workers = []

    def tearDown(self):
        for gate in self.gates:
            gate.set()
        for thread, errors, _ in self.workers:
            thread.join(3)
            self.assertFalse(thread.is_alive(), "辅助线程未退出")
            self.assertEqual(errors, [])
        for bus in self.buses:
            self.assertTrue(bus.shutdown(), bus.status())
            self.assertFalse(bus._consumer_thread.is_alive())

    def bus(self, **kwargs):
        bus = EventBus(**kwargs)
        self.buses.append(bus)
        return bus

    def gate(self):
        gate = threading.Event()
        self.gates.append(gate)
        return gate

    def require(self, event):
        self.assertTrue(event.wait(2), "线程门闩未触发")

    def spawn(self, call):
        errors, result = [], []

        def run():
            try:
                result.append(call())
            except BaseException as exc:  # noqa: BLE001 - 测试线程完整转交异常。
                errors.append(exc)

        thread = threading.Thread(target=run, daemon=True)
        worker = (thread, errors, result)
        self.workers.append(worker)
        thread.start()
        return worker

    def join(self, worker):
        thread, errors, result = worker
        thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        return result[0] if result else None

    def event(self, bus, index=0, name="task.succeeded", **kwargs):
        return bus.build_event(name, category=RuntimeEventCategory.TASK,
                               source="contract_check", payload={"index": index}, **kwargs)

    def blocked_async(self, bus, *, on_gap=None, callback=None):
        entered, release = self.gate(), self.gate()
        seen = []

        def receive(event):
            if event.payload["index"] == 0:
                entered.set()
                self.require(release)
            seen.append(event.payload["index"])
            if callback:
                callback(event)

        identity = bus.subscribe(receive, async_dispatch=True, on_gap=on_gap)
        bus.publish(self.event(bus))
        self.require(entered)
        return identity, release, seen

    def test_normal_filters_and_replay(self):
        bus = self.bus()
        seen = []
        bus.subscribe(seen.append, categories=[RuntimeEventCategory.TASK], event_types=["task.succeeded"])
        bus.publish(self.event(bus))
        bus.publish(self.event(bus, 1, "task.progress"))
        self.assertEqual([e.cursor for e in seen], [1])
        self.assertEqual([e.cursor for e in bus.replay(after_cursor=1)], [2])

    def test_reentrant_delivery_order(self):
        bus = self.bus()
        seen = []

        def first(event):
            seen.append(("first", event.cursor))
            if event.cursor == 1:
                bus.publish(self.event(bus, 1))

        bus.subscribe(first)
        bus.subscribe(lambda e: seen.append(("second", e.cursor)))
        bus.publish(self.event(bus))
        self.assertEqual(seen, [("first", 1), ("second", 1), ("first", 2), ("second", 2)])

    def test_concurrent_publisher_order(self):
        bus = self.bus()
        entered, release = self.gate(), self.gate()
        seen = []

        def callback(event):
            seen.append(event.cursor)
            if event.cursor == 1:
                entered.set()
                self.require(release)

        bus.subscribe(callback)
        first = self.spawn(lambda: bus.publish(self.event(bus)))
        self.require(entered)
        prepared = bus.prepare(self.event(bus, 1))
        second = self.spawn(lambda: bus.dispatch_prepared(prepared))
        self.assertTrue(second[0].is_alive())
        release.set()
        self.join(first)
        self.join(second)
        self.assertEqual(seen, [1, 2])

    def test_abandon_releases_later_reservation(self):
        bus = self.bus()
        first = bus.prepare(self.event(bus))
        second = bus.prepare(self.event(bus, 1))
        worker = self.spawn(lambda: bus.dispatch_prepared(second))
        self.assertTrue(bus.abandon_prepared(first))
        self.assertIs(self.join(worker), second.stored_event)
        self.assertFalse(bus.abandon_prepared(first))
        self.assertEqual(bus.status()["prepared_publications"], 0)

    def test_atomic_replay_live_boundary_for_reserved_event(self):
        bus = self.bus()
        first = bus.prepare(self.event(bus))
        seen = []
        _, snapshot = bus.subscribe_with_replay(seen.append)
        self.assertEqual([e.cursor for e in snapshot.events], [1])
        bus.dispatch_prepared(first)
        bus.publish(self.event(bus, 1))
        self.assertEqual([e.cursor for e in seen], [2])

    def test_replay_gap_contract(self):
        bus = self.bus(retention_limit=2)
        for index in range(4):
            bus.publish(self.event(bus, index))
        _, expired = bus.subscribe_with_replay(lambda e: None, after_cursor=1)
        self.assertEqual(expired.gap_reason, "cursor_expired")
        self.assertEqual([e.cursor for e in expired.events], [3, 4])
        _, ahead = bus.subscribe_with_replay(lambda e: None, after_cursor=20)
        self.assertEqual(ahead.gap_reason, "cursor_ahead_of_stream")
        self.assertEqual(ahead.effective_after_cursor, 0)

    def test_unsubscribe_suppresses_captured_sync_callback(self):
        bus = self.bus()
        seen = []
        identity = bus.subscribe(seen.append)
        prepared = bus.prepare(self.event(bus))
        bus.unsubscribe(identity)
        bus.dispatch_prepared(prepared)
        self.assertEqual(seen, [])

    def test_status_reports_oldest_pending_cursor_and_age(self):
        bus = self.bus()
        _, release, _ = self.blocked_async(bus)
        bus.publish(self.event(bus, 1))
        prepared = bus.prepare(self.event(bus, 2))
        state = bus.status()
        self.assertEqual(state["oldest_async_cursor"], 2)
        self.assertEqual(state["oldest_prepared_cursor"], 3)
        self.assertGreaterEqual(state["oldest_async_age_sec"], 0)
        self.assertGreaterEqual(state["oldest_prepared_age_sec"], 0)
        bus.abandon_prepared(prepared)
        release.set()
        self.assertTrue(bus.shutdown())
        self.assertIsNone(bus.status()["oldest_async_cursor"])
        self.assertIsNone(bus.status()["oldest_prepared_cursor"])

    def test_callback_can_unsubscribe_next_callback(self):
        bus = self.bus()
        seen = []
        identities = []
        bus.subscribe(lambda event: bus.unsubscribe(identities[0]))
        identities.append(bus.subscribe(seen.append))
        bus.publish(self.event(bus))
        self.assertEqual(seen, [])

    def test_unsubscribe_discards_pending_async_and_retains_inflight(self):
        bus = self.bus()
        identity, release, seen = self.blocked_async(bus)
        bus.publish(self.event(bus, 1))
        bus.unsubscribe(identity)
        state = bus.status()
        self.assertEqual(state["async_cancelled"], 1)
        self.assertEqual(state["in_flight_callbacks"], {identity: 1})
        self.assertEqual(state["async_queued"], 0)
        release.set()
        self.assertTrue(bus.shutdown())
        self.assertEqual(seen, [0])

    def test_overflow_coalesces_exact_gap_before_remaining_payloads(self):
        bus = self.bus(async_queue_limit=2)
        order = []
        done = self.gate()
        identity, release, seen = self.blocked_async(
            bus, on_gap=lambda gap: order.append(("gap", gap, threading.get_ident())),
            callback=lambda e: (order.append(("event", e.cursor)), done.set() if e.payload["index"] == 5 else None))
        for index in range(1, 6):
            bus.publish(self.event(bus, index))
        state = bus.status()
        self.assertEqual(state["async_dropped"], 3)
        self.assertEqual(state["async_terminal_dropped"], 3)
        self.assertEqual(state["async_high_water"], 2)
        self.assertEqual(len(state["pending_gaps"]), 1)
        gap = state["pending_gaps"][0]
        self.assertEqual((gap["first_dropped_cursor"], gap["last_dropped_cursor"], gap["dropped_count"]), (2, 4, 3))
        self.assertEqual(gap["subscription_id"], identity)
        gap["dropped_count"] = 999
        self.assertEqual(bus.status()["pending_gaps"][0]["dropped_count"], 3)
        release.set()
        self.require(done)
        self.assertEqual(seen, [0, 4, 5])
        self.assertEqual([item[0] for item in order], ["event", "gap", "event", "event"])
        self.assertEqual(order[1][2], bus._consumer_thread.ident)
        diagnostics = bus.replay(event_types=["event_bus.async_dispatch_overflow"])
        self.assertEqual(len(diagnostics), 1)
        self.assertEqual(diagnostics[0].payload["dropped_count"], 3)

    def test_overflow_is_observable_without_gap_callback(self):
        bus = self.bus(async_queue_limit=1)
        done = self.gate()
        _, release, _ = self.blocked_async(bus, callback=lambda e: done.set() if e.payload["index"] == 2 else None)
        bus.publish(self.event(bus, 1))
        bus.publish(self.event(bus, 2))
        self.assertEqual(bus.status()["async_dropped"], 1)
        release.set()
        self.require(done)
        self.assertEqual(len(bus.replay(event_types=["event_bus.async_dispatch_overflow"])), 1)

    def test_blocked_gap_callback_gets_one_more_coalesced_notice(self):
        bus = self.bus(async_queue_limit=1)
        gap_entered, gap_release, done = self.gate(), self.gate(), self.gate()
        notices = []

        def on_gap(notice):
            notices.append(notice)
            if len(notices) == 1:
                gap_entered.set()
                self.require(gap_release)

        _, release, seen = self.blocked_async(bus, on_gap=on_gap,
            callback=lambda e: done.set() if e.payload["index"] == 4 else None)
        bus.publish(self.event(bus, 1))
        bus.publish(self.event(bus, 2))
        release.set()
        self.require(gap_entered)
        bus.publish(self.event(bus, 3))
        bus.publish(self.event(bus, 4))
        pending = bus.status()["pending_gaps"]
        self.assertEqual(len(pending), 1)
        self.assertEqual(pending[0]["dropped_count"], 2)
        self.assertEqual(pending[0]["first_dropped_cursor"], 3)
        # 首次缺口诊断也占一个公共游标，因此第二个新丢失载荷游标为 5。
        self.assertEqual(pending[0]["last_dropped_cursor"], 5)
        gap_release.set()
        self.require(done)
        self.assertEqual([notice["dropped_count"] for notice in notices], [1, 2])
        self.assertEqual(seen, [0, 4])

    def test_filtered_telemetry_does_not_fill_terminal_queue(self):
        bus = self.bus(async_queue_limit=1)
        entered, release = self.gate(), self.gate()
        bus.subscribe(lambda e: (entered.set(), self.require(release)),
                      event_types=["task.succeeded"], async_dispatch=True)
        bus.publish(self.event(bus))
        self.require(entered)
        for index in range(5):
            bus.publish(self.event(bus, index, "localization.snapshot_ready"))
        self.assertEqual(bus.status()["async_enqueued"], 1)
        self.assertEqual(bus.status()["async_dropped"], 0)
        self.assertEqual(bus.status()["async_queued"], 0)
        release.set()

    def test_overflow_notice_count_is_bounded_by_subscriptions(self):
        bus = self.bus(async_queue_limit=2)
        _, release, _ = self.blocked_async(bus)
        bus.subscribe(lambda e: None, async_dispatch=True)
        for index in range(1, 7):
            bus.publish(self.event(bus, index))
        state = bus.status()
        self.assertEqual(len(state["pending_gaps"]), 2)
        self.assertEqual(state["async_dropped"], 10)
        self.assertEqual(sum(g["dropped_count"] for g in state["pending_gaps"]), 10)
        release.set()

    def test_erroring_gap_callback_is_isolated(self):
        bus = self.bus(async_queue_limit=1)
        done = self.gate()

        def gap(_):
            raise RuntimeError("故意模拟缺口处理失败")

        _, release, seen = self.blocked_async(bus, on_gap=gap,
            callback=lambda e: done.set() if e.payload["index"] == 2 else None)
        bus.publish(self.event(bus, 1))
        bus.publish(self.event(bus, 2))
        release.set()
        self.require(done)
        self.assertEqual(seen, [0, 2])
        self.assertEqual(bus.status()["gap_callback_failures"], 1)
        self.assertEqual(len(bus.replay(event_types=["event_bus.gap_callback_failed"])), 1)
        self.assertEqual(bus.status()["async_enqueued"], 3)

    def test_gap_callback_can_close_reentrantly(self):
        bus = self.bus(async_queue_limit=1)
        closed = []
        _, release, seen = self.blocked_async(bus, on_gap=lambda _: closed.append(bus.shutdown()))
        bus.publish(self.event(bus, 1))
        bus.publish(self.event(bus, 2))
        release.set()
        bus._consumer_thread.join(2)
        self.assertEqual(closed, [False])
        self.assertEqual(seen, [0, 2])
        self.assertTrue(bus.shutdown())

    def test_unsubscribe_cancels_pending_gap_without_erasing_loss_counters(self):
        bus = self.bus(async_queue_limit=1)
        gaps = []
        identity, release, seen = self.blocked_async(bus, on_gap=gaps.append)
        bus.publish(self.event(bus, 1))
        bus.publish(self.event(bus, 2))
        bus.unsubscribe(identity)
        self.assertEqual(bus.status()["pending_gaps"], ())
        self.assertEqual(bus.status()["async_dropped"], 1)
        self.assertEqual(bus.status()["first_dropped_cursor"], 2)
        release.set()
        self.assertTrue(bus.shutdown())
        self.assertEqual(gaps, [])
        self.assertEqual(seen, [0])

    def test_async_failure_diagnostic_never_waits_on_unrelated_reservation(self):
        bus = self.bus()
        entered, release, delivered = self.gate(), self.gate(), self.gate()

        def failing(_):
            entered.set()
            self.require(release)
            raise RuntimeError("故意失败")

        bus.subscribe(failing, async_dispatch=True)
        bus.subscribe(lambda e: delivered.set(), async_dispatch=True)
        bus.publish(self.event(bus))
        self.require(entered)
        reservation = bus.prepare(self.event(bus, 1))
        release.set()
        self.require(delivered)
        self.assertEqual(bus.status()["callback_failures"], 1)
        self.assertEqual(len(bus.replay(event_types=["event_bus.subscription_failed"])), 1)
        self.assertTrue(bus.abandon_prepared(reservation))

    def test_sync_failure_does_not_propagate_or_recurse(self):
        bus = self.bus()
        seen = []
        bus.subscribe(lambda e: (_ for _ in ()).throw(RuntimeError("故意失败")))
        bus.subscribe(seen.append)
        stored = bus.publish(self.event(bus))
        self.assertEqual(stored.cursor, 1)
        self.assertEqual([e.cursor for e in seen], [1])
        self.assertEqual(bus.status()["callback_failures"], 1)
        self.assertEqual([e.event_type for e in bus.replay()], ["task.succeeded", "event_bus.subscription_failed"])

    def test_diagnostic_failure_does_not_fail_committed_publication(self):
        bus = self.bus()

        class BadError(Exception):
            def __str__(self):
                raise RuntimeError("错误文本本身失败")

        bus.subscribe(lambda e: (_ for _ in ()).throw(BadError()))
        stored = bus.publish(self.event(bus))
        self.assertEqual(stored.cursor, 1)
        self.assertEqual(bus.status()["diagnostic_failures"], 1)
        self.assertEqual(bus.status()["next_dispatch_cursor"], 2)

    def test_closed_admission_rejected_before_cursor_commit(self):
        bus = self.bus()
        bus.publish(self.event(bus))
        self.assertTrue(bus.shutdown())
        for call in (lambda: bus.publish(self.event(bus)), lambda: bus.prepare(self.event(bus)),
                     lambda: bus.subscribe(lambda e: None),
                     lambda: bus.subscribe_with_replay(lambda e: None)):
            with self.assertRaises(RuntimeError):
                call()
        self.assertEqual(bus.latest_cursor(), 1)
        self.assertTrue(bus.shutdown())

    def test_shutdown_drains_admitted_async_work_without_clearing_on_timeout(self):
        bus = self.bus()
        _, release, seen = self.blocked_async(bus)
        bus.publish(self.event(bus, 1))
        started = time.monotonic()
        self.assertFalse(bus.shutdown(timeout_sec=0.01))
        self.assertLess(time.monotonic() - started, 0.5)
        self.assertEqual(bus.status()["async_queued"], 1)
        self.assertFalse(bus.status()["shutdown_complete"])
        release.set()
        self.assertTrue(bus.shutdown())
        self.assertEqual(seen, [0, 1])
        self.assertTrue(bus.status()["shutdown_complete"])

    def test_sync_callback_shutdown_does_not_self_wait(self):
        bus = self.bus()
        result, after = [], []
        bus.subscribe(lambda e: result.append(bus.shutdown()))
        bus.subscribe(after.append)
        stored = bus.publish(self.event(bus))
        self.assertEqual(result, [False])
        self.assertEqual(after, [])
        self.assertEqual(stored.cursor, 1)
        self.assertTrue(bus.shutdown())
        self.assertEqual(bus.status()["shutdown_discarded_callbacks"], 1)

    def test_async_callback_shutdown_does_not_self_join(self):
        bus = self.bus()
        result = []
        called = self.gate()
        bus.subscribe(lambda e: (result.append(bus.shutdown()), called.set()), async_dispatch=True)
        bus.publish(self.event(bus))
        self.require(called)
        self.assertEqual(result, [False])
        self.assertTrue(bus.shutdown())
        self.assertEqual(bus.status()["callback_failures"], 0)

    def test_reentrant_publication_then_shutdown_is_still_committed(self):
        bus = self.bus()
        nested = []

        def receive(event):
            nested.append(bus.publish(self.event(bus, 1)))
            self.assertFalse(bus.shutdown())

        bus.subscribe(receive)
        bus.publish(self.event(bus))
        self.assertEqual([event.cursor for event in nested], [2])
        self.assertEqual(bus.status()["shutdown_discarded_publications"], 1)
        self.assertEqual([event.cursor for event in bus.replay()], [1, 2])
        self.assertTrue(bus.shutdown())

    def test_shutdown_unblocks_reserved_cursor_waiter_as_committed(self):
        bus = self.bus()
        first = bus.prepare(self.event(bus))
        second = bus.prepare(self.event(bus, 1))
        worker = self.spawn(lambda: bus.dispatch_prepared(second))
        self.assertTrue(bus.shutdown())
        self.assertIs(self.join(worker), second.stored_event)
        self.assertIs(bus.dispatch_prepared(first), first.stored_event)
        self.assertEqual(bus.status()["shutdown_discarded_publications"], 2)
        self.assertEqual(bus.status()["prepared_publications"], 0)
        self.assertEqual(len(bus.replay()), 2)

    def test_shutdown_reports_running_sync_callback_and_unblocks_other_publishers(self):
        bus = self.bus()
        entered, release = self.gate(), self.gate()
        seen = []

        def callback(event):
            seen.append(event.cursor)
            entered.set()
            self.require(release)

        bus.subscribe(callback)
        first = self.spawn(lambda: bus.publish(self.event(bus)))
        self.require(entered)
        second = bus.prepare(self.event(bus, 1))
        waiting = self.spawn(lambda: bus.dispatch_prepared(second))
        self.assertFalse(bus.shutdown(timeout_sec=0.01))
        self.assertIs(self.join(waiting), second.stored_event)
        self.assertTrue(bus.status()["in_flight_callbacks"])
        release.set()
        self.join(first)
        self.assertTrue(bus.shutdown())
        self.assertEqual(seen, [1])

    def test_shutdown_timeout_validation(self):
        bus = self.bus()
        for value in (-1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                bus.shutdown(value)
        self.assertTrue(bus.status()["accepting"])

    def test_subscription_failure_during_shutdown_remains_observable(self):
        bus = self.bus()
        entered, release = self.gate(), self.gate()

        def fail(_):
            entered.set()
            self.require(release)
            raise RuntimeError("关闭中的预期失败")

        bus.subscribe(fail, async_dispatch=True)
        bus.publish(self.event(bus))
        self.require(entered)
        self.assertFalse(bus.shutdown(0))
        release.set()
        self.assertTrue(bus.shutdown())
        self.assertEqual(bus.status()["callback_failures"], 1)
        self.assertEqual(bus.status()["diagnostics_suppressed_after_shutdown"], 1)

    def test_real_task_manager_preserves_accepted_started_terminal_order(self):
        bus = self.bus()
        manager = TaskManager(event_bus=bus, max_workers=1)
        entered, release, ran = self.gate(), self.gate(), self.gate()
        events = []

        def observe(event):
            events.append(event.event_type)
            if event.event_type == "task.accepted":
                entered.set()
                self.require(release)

        bus.subscribe(observe)
        try:
            def runner(ctx):
                ran.set()
                return {"ok": True}

            submitted = self.spawn(lambda: manager.submit("capture_image", runner))
            self.require(entered)
            self.assertFalse(ran.is_set())
            release.set()
            spec = self.join(submitted)
            self.assertEqual(manager.wait_for_task(spec.task_id, 2), {"ok": True})
            self.assertEqual(events, ["task.accepted", "task.started", "task.succeeded"])
        finally:
            release.set()
            manager.shutdown()

    def test_dispatch_boundary_failure_cannot_relabel_committed_action(self):
        bus = self.bus()
        with patch.object(bus, "_deliver_publication", side_effect=RuntimeError("分发边界失败")):
            stored = bus.publish(self.event(bus))
        self.assertEqual(stored.cursor, 1)
        self.assertEqual(bus.status()["dispatch_failures"], 1)
        self.assertEqual(bus.status()["prepared_publications"], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
