#!/usr/bin/env python3
"""Run only the six cancellation and six prepared-publication regressions.

Use the real project source next to this script. No transport, process, or
other test classes are admitted by this runner.
"""
# ruff: noqa: E402

from contextlib import ExitStack
import importlib.util
from pathlib import Path
import socket
import subprocess
import sys
import threading
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from contracts.streams import StreamFrame
from core import StreamBus
import core.stream_bus as streams

FOCUSED = (
    "test_attempted_item_is_not_replayed_and_next_publication_progresses",
    "test_concurrent_accepted_publisher_is_released",
    "test_reentrant_publication_keeps_fifo_without_duplicate_delivery",
    "test_first_interruption_survives_later_interruptions_and_ordinary_error",
    "test_shutdown_releases_queued_waiter_without_delivering_it",
    "test_old_rethrow_cannot_clear_a_new_drainer_after_empty_handoff",
)
PREPARED = (
    "test_no_prepare_callbacks_and_abandoned_work_adopted_by_next_dispatch",
    "test_shutdown_completes_pending_and_foreign_owner_rejected",
    "test_unclaimed_preparations_bounded_before_failed_cursor_allocation",
    "test_cross_thread_dispatch_and_reentrant_publish_preserve_order",
    "test_pending_shutdown_releases_waiters_without_running_their_callbacks",
    "test_async_latest_only_coalescing_and_gap_reporting_survive_prepare",
)


class PreparedChecks(unittest.TestCase):
    def setUp(self):
        self.bus = StreamBus()
        self.addCleanup(self.bus.shutdown)
        self.seen = []
        self.bus.subscribe(lambda f: self.seen.append(f.revision), topics=("synthetic",))

    def frame(self, seq):
        return StreamFrame(topic="synthetic", source="owner", revision=str(seq))

    def test_no_prepare_callbacks_and_abandoned_work_adopted_by_next_dispatch(self):
        p = self.bus.prepare_publication(self.frame(1))
        self.assertEqual(self.seen, [])
        self.bus.publish_frame(self.frame(2))
        self.assertEqual(self.seen, ["1", "2"])
        self.bus.dispatch_prepared(p)
        self.assertEqual(self.seen, ["1", "2"])

    def test_shutdown_completes_pending_and_foreign_owner_rejected(self):
        p = self.bus.prepare_publication(self.frame(1))
        other = StreamBus()
        self.addCleanup(other.shutdown)
        with self.assertRaises(ValueError):
            other.dispatch_prepared(p)
        self.bus.shutdown()
        self.assertEqual(self.bus.dispatch_prepared(p).revision, "1")
        self.assertEqual(self.seen, [])
        with self.assertRaises(RuntimeError):
            self.bus.prepare_publication(self.frame(2))

    def test_unclaimed_preparations_bounded_before_failed_cursor_allocation(self):
        with patch.object(streams, "_PREPARED_DISPATCH_QUEUE_MAXSIZE", 2):
            self.bus.prepare_publication(self.frame(1))
            self.bus.prepare_publication(self.frame(2))
            with self.assertRaises(RuntimeError):
                self.bus.prepare_publication(self.frame(3))
            self.assertEqual(self.bus.latest_cursor(), 2)

    def test_cross_thread_dispatch_and_reentrant_publish_preserve_order(self):
        def callback(f):
            if f.revision == "1":
                self.bus.publish_frame(self.frame(3))

        self.bus.subscribe(callback, topics=("synthetic",))
        p = self.bus.prepare_publication(self.frame(1))
        self.bus.prepare_publication(self.frame(2))
        worker = threading.Thread(target=lambda: self.bus.dispatch_prepared(p))
        worker.start()
        worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(self.seen, ["1", "2", "3"])

    def test_pending_shutdown_releases_waiters_without_running_their_callbacks(self):
        entered = threading.Event()
        release = threading.Event()
        finished = threading.Event()
        self.addCleanup(release.set)

        def blocker(frame):
            if frame.revision == "1":
                entered.set()
                release.wait(3)

        self.bus.subscribe(blocker, topics=("synthetic",))
        first = threading.Thread(target=lambda: self.bus.publish_frame(self.frame(1)))
        first.start()
        self.assertTrue(entered.wait(2))
        prepared = self.bus.prepare_publication(self.frame(2))
        second = threading.Thread(target=lambda: (self.bus.dispatch_prepared(prepared), finished.set()))
        second.start()
        self.bus.shutdown()
        self.assertTrue(finished.wait(1))
        release.set()
        first.join(2)
        second.join(2)
        self.assertEqual(self.seen, ["1"])

    def test_async_latest_only_coalescing_and_gap_reporting_survive_prepare(self):
        entered = threading.Event()
        release = threading.Event()
        received = threading.Event()
        notices = []
        seen = []
        self.addCleanup(release.set)

        def callback(frame):
            seen.append(frame.revision)
            if frame.revision == "0":
                entered.set()
                release.wait(3)
            if frame.revision == "3":
                received.set()

        self.bus.subscribe(
            callback, topics=("async",), async_dispatch=True, latest_only_topics=("async",), on_gap=notices.append
        )
        self.bus.publish("async", source="owner", revision="0")
        self.assertTrue(entered.wait(2))
        for revision in ["1", "2", "3"]:
            token = self.bus.prepare_publication(StreamFrame(topic="async", source="owner", revision=revision))
            self.bus.dispatch_prepared(token)
        release.set()
        self.assertTrue(received.wait(2))
        self.assertEqual(seen, ["0", "3"])
        self.assertEqual(notices, [])
        gap_entered = threading.Event()
        gap_release = threading.Event()
        gap_seen = threading.Event()
        self.addCleanup(gap_release.set)

        def event_callback(frame):
            if frame.revision == "0":
                gap_entered.set()
                gap_release.wait(3)

        def gap_callback(value):
            notices.append(value)
            gap_seen.set()

        self.bus.subscribe(event_callback, topics=("events",), async_dispatch=True, on_gap=gap_callback)
        self.bus.publish("events", source="owner", revision="0")
        self.assertTrue(gap_entered.wait(2))
        with patch.object(streams, "_ASYNC_DISPATCH_QUEUE_MAXSIZE", 2):
            for revision in ["1", "2", "3"]:
                self.bus.publish("events", source="owner", revision=revision)
        gap_release.set()
        self.assertTrue(gap_seen.wait(2))
        self.assertTrue(notices)


class NoSocket(socket.socket):
    def __new__(cls, *args, **kwargs):
        raise AssertionError("socket construction forbidden in this component suite")


def verify_imports():
    for name, module in tuple(sys.modules.items()):
        filename = getattr(module, "__file__", None)
        if not filename:
            continue
        declared = ROOT.joinpath(*name.split("."))
        candidates = (declared.with_suffix(".py"), declared / "__init__.py")
        expected = next((path for path in candidates if path.is_file()), None)
        if expected is not None and Path(filename).resolve() != expected.resolve():
            raise AssertionError("repository module imported from wrong tree: " + name)
    actual = Path(sys.modules["core.stream_bus"].__file__).resolve()
    if actual != ROOT / "core/stream_bus.py":
        raise AssertionError("real restored StreamBus was not imported")
    print("SOURCE", actual, flush=True)


def load(name, relative):
    path = ROOT / relative
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    if Path(module.__file__).resolve() != path.resolve():
        raise AssertionError("test source path changed")
    return module


def main():
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(ROOT))
    print("INTERPRETER", sys.executable, sys.version, flush=True)
    with ExitStack() as guards:
        guards.enter_context(patch.object(socket, "socket", NoSocket))
        guards.enter_context(patch.object(subprocess, "Popen", side_effect=AssertionError("process forbidden")))
        import grpc
        import lcm

        for name in ("insecure_channel", "secure_channel"):
            guards.enter_context(patch.object(grpc, name, side_effect=AssertionError("real grpc forbidden")))
        guards.enter_context(patch.object(lcm, "LCM", side_effect=AssertionError("real LCM forbidden")))
        focused = load("stream_cancellation_fixture", "tests/test_stream_bus_cancellation.py")
        suite = unittest.TestSuite()
        for module, cls_name, methods in (
            (focused, "StreamBusCancellationTests", FOCUSED),
            (sys.modules[__name__], "PreparedChecks", PREPARED),
        ):
            case = getattr(module, cls_name)
            for method in methods:
                suite.addTest(case(method))
        if suite.countTestCases() != 12:
            raise AssertionError("exact test selection changed")
        verify_imports()
        result = unittest.TextTestRunner(verbosity=2).run(suite)
        verify_imports()
        return int(not result.wasSuccessful() or bool(result.skipped) or result.testsRun != 12)


if __name__ == "__main__":
    raise SystemExit(main())
