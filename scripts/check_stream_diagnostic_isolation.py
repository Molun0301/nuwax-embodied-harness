#!/usr/bin/env python3
"""Offline causal checks for StreamBus diagnostic output ownership.

Small queues and thread gates only; no devices, models, sockets or benchmarks.
Python 3.8 compatible. Run with the project interpreter from this checkout.
"""
import io
import itertools
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core import stream_bus as module
from core.logging_utils import CorrelationContextFilter, JsonLineFormatter, LogRateLimiter, correlation_context
from core.stream_bus import StreamBus


class Sink(logging.Handler):
    def __init__(self, action=None):
        super().__init__()
        self.records = []
        self.action = action

    def emit(self, record):
        self.records.append(record)
        if self.action:
            self.action(record)


class StreamDiagnosticChecks(unittest.TestCase):
    def setUp(self):
        self.buses, self.gates, self.workers, self.errors = [], [], [], []
        self.logger = logging.Logger('core.stream_bus', logging.DEBUG)
        self.logger.propagate = False
        self.logger_patch = patch.object(module, '_logger', self.logger)
        self.limiter_patch = patch.object(module, '_DIAGNOSTIC_LOG_LIMITER', LogRateLimiter())
        self.logger_patch.start()
        self.limiter_patch.start()

    def tearDown(self):
        for gate in self.gates:
            gate.set()
        for worker in self.workers:
            worker.join(3)
            self.assertFalse(worker.is_alive(), 'fixture publisher did not exit')
        for bus in self.buses:
            self.assertTrue(bus.shutdown(timeout_sec=3), bus.status())
        self.limiter_patch.stop()
        self.logger_patch.stop()
        self.assertEqual(self.errors, [])

    def bus(self, **options):
        bus = StreamBus(**options)
        self.buses.append(bus)
        return bus

    def gate(self):
        gate = threading.Event()
        self.gates.append(gate)
        return gate

    def require(self, gate):
        self.assertTrue(gate.wait(3), 'fixture gate did not arrive')

    def wait_for(self, predicate):
        deadline = time.monotonic() + 3
        while not predicate():
            self.assertLess(time.monotonic(), deadline, 'condition did not settle')
            time.sleep(0.005)

    def start(self, action):
        done = self.gate()
        def run():
            try:
                action()
            except BaseException as error:
                self.errors.append(repr(error))
            finally:
                done.set()
        worker = threading.Thread(target=run, daemon=True)
        self.workers.append(worker)
        worker.start()
        return done

    def blocked_sink(self):
        entered, resume = self.gate(), self.gate()
        def block(record):
            if len(sink.records) == 1:
                entered.set()
                self.require(resume)
        sink = Sink(block)
        self.logger.addHandler(sink)
        return sink, entered, resume

    @staticmethod
    def emit(bus, message, key=None, **kwargs):
        return bus._log_diagnostic(logging.WARNING, message,
                                   key=key or message[:40], interval_sec=0, **kwargs)

    @staticmethod
    def state(bus):
        return bus.status()['diagnostic_log']

    def test_real_slow_dispatch_warning_does_not_wait_for_sink(self):
        sink, entered, resume = self.blocked_sink()
        bus = self.bus()
        seen = []
        bus.subscribe(lambda frame: seen.append(frame.topic))
        with patch('time.perf_counter', side_effect=itertools.count(0.0, 0.01)):
            first = self.start(lambda: bus.publish('map.frame', source='check'))
            self.require(entered)
            self.require(first)
            other = self.start(lambda: bus.publish('unrelated.state', source='check'))
            self.require(other)
        self.assertEqual(seen, ['map.frame', 'unrelated.state'])
        self.assertEqual(sink.records[0].name, 'core.stream_bus')
        self.assertTrue(sink.records[0].getMessage().startswith('[PERF] 帧分发耗时:'))
        self.assertTrue(sink.records[0].stream_bus_internal)
        resume.set()

    def test_blocked_sink_preserves_reentry_fifo_and_async_progress(self):
        _, entered, resume = self.blocked_sink()
        bus = self.bus()
        seen, async_done = [], self.gate()
        def first(frame):
            seen.append(('first', frame.topic))
            if frame.topic == 'one':
                self.emit(bus, 'callback diagnostic')
                bus.publish('two', source='check')
        bus.subscribe(first)
        bus.subscribe(lambda frame: seen.append(('second', frame.topic)))
        bus.subscribe(lambda frame: async_done.set(), async_dispatch=True)
        published = self.start(lambda: bus.publish('one', source='check'))
        self.require(entered)
        self.require(published)
        self.require(async_done)
        self.assertEqual(seen, [('first', 'one'), ('second', 'one'),
                                ('first', 'two'), ('second', 'two')])
        resume.set()

    def test_file_console_exception_and_correlation_survive_handoff(self):
        bus = self.bus()
        capture = Sink()
        self.logger.addHandler(capture)
        console = io.StringIO()
        console_handler = logging.StreamHandler(console)
        console_handler.setFormatter(JsonLineFormatter())
        console_handler.addFilter(CorrelationContextFilter())
        self.logger.addHandler(console_handler)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'diagnostics.jsonl'
            file_handler = RotatingFileHandler(str(path), maxBytes=8192, backupCount=1)
            file_handler.setFormatter(JsonLineFormatter())
            file_handler.addFilter(CorrelationContextFilter())
            self.logger.addHandler(file_handler)
            try:
                with correlation_context(request_id='request-a', task_id='task-a', capability_name='camera-a'):
                    try:
                        raise ValueError('exception detail')
                    except ValueError:
                        self.emit(bus, 'boundary failed', exc_info=True)
                self.emit(bus, 'later independent message')
                self.assertTrue(bus.shutdown(timeout_sec=3))
                self.assertIn('boundary failed', path.read_text())
                self.assertIn('exception detail', path.read_text())
                self.assertIn('boundary failed', console.getvalue())
                self.assertIn('exception detail', console.getvalue())
                first = capture.records[0]
                self.assertEqual(first.request_id, 'request-a')
                self.assertEqual(first.task_id, 'task-a')
                self.assertEqual(first.capability_name, 'camera-a')
                self.assertIsNone(first.exc_info, 'traceback/frame objects must not cross owner boundary')
                self.assertIsInstance(first.exc_text, str)
                self.assertFalse(getattr(capture.records[-1], 'request_id', ''))
            finally:
                self.logger.removeHandler(file_handler)
                file_handler.close()

    def test_rate_limit_is_decided_at_occurrence_not_writer_drain(self):
        sink, entered, resume = self.blocked_sink()
        bus = self.bus()
        clock = [0.0]
        limiter = LogRateLimiter(clock=lambda: clock[0])
        with patch.object(module, '_DIAGNOSTIC_LOG_LIMITER', limiter):
            bus._log_diagnostic(logging.WARNING, 'first', key='same', interval_sec=5)
            self.require(entered)
            clock[0] = 1.0
            bus._log_diagnostic(logging.WARNING, 'suppressed', key='same', interval_sec=5)
            clock[0] = 6.0
            bus._log_diagnostic(logging.WARNING, 'third', key='same', interval_sec=5)
            self.assertEqual(self.state(bus)['accepted'], 2)
            resume.set()
            self.assertTrue(bus.shutdown(timeout_sec=3))
        self.assertEqual([record.getMessage() for record in sink.records], ['first', 'third'])
        self.assertEqual(sink.records[-1].suppressed_count, 1)
        self.assertEqual(sink.records[-1].rate_limit_window_sec, 5)

    def test_count_capacity_includes_inflight_and_reports_gap(self):
        sink, entered, resume = self.blocked_sink()
        bus = self.bus(diagnostic_max_items=2)
        self.emit(bus, 'first')
        self.require(entered)
        self.emit(bus, 'second')
        self.emit(bus, 'third rejected')
        state = self.state(bus)
        self.assertEqual(state['outstanding_items'], 2)
        self.assertEqual(state['capacity_rejected'], 1)
        self.assertEqual(state['dropped'], 1)
        self.assertIsNotNone(state['pending_gap'])
        resume.set()
        self.assertTrue(bus.shutdown(timeout_sec=3))
        messages = [record.getMessage() for record in sink.records]
        self.assertEqual(messages[0], 'first')
        self.assertEqual(messages[-1], 'second')
        self.assertEqual(len(messages), 3)
        self.assertEqual(self.state(bus)['gap_reports'], 1)
        self.assertEqual(self.state(bus)['outstanding_items'], 0)

    def test_byte_capacity_includes_inflight(self):
        _, entered, resume = self.blocked_sink()
        bus = self.bus(diagnostic_max_items=10, diagnostic_max_bytes=8192)
        self.emit(bus, 'A' * 4000, key='first-large')
        self.require(entered)
        self.emit(bus, 'B' * 4000, key='second-large')
        state = self.state(bus)
        self.assertEqual(state['accepted'], 1)
        self.assertEqual(state['capacity_rejected'], 1)
        self.assertLessEqual(state['outstanding_bytes'], 8192)
        resume.set()

    def test_oversize_rejection_is_explicit_and_gap_is_written(self):
        sink = Sink()
        self.logger.addHandler(sink)
        bus = self.bus(diagnostic_max_bytes=1024)
        self.emit(bus, 'X' * 8192, key='oversize')
        self.assertTrue(bus.shutdown(timeout_sec=3))
        state = self.state(bus)
        self.assertEqual(state['oversize_rejected'], 1)
        self.assertEqual(state['dropped'], 1)
        self.assertEqual(state['gap_reports'], 1)
        self.assertTrue(sink.records)

    def test_sink_failure_is_counted_and_next_record_progresses(self):
        def fail_first(record):
            if record.getMessage() == 'bad sink':
                raise OSError('sink unavailable')
        sink = Sink(fail_first)
        self.logger.addHandler(sink)
        bus = self.bus()
        self.emit(bus, 'bad sink')
        self.emit(bus, 'next record')
        self.assertTrue(bus.shutdown(timeout_sec=3))
        state = self.state(bus)
        self.assertEqual(state['writer_failed'], 1)
        self.assertIn('next record', [record.getMessage() for record in sink.records])
        self.assertLessEqual(len(sink.records), 3, 'failure must not recursively enqueue itself')

    def test_gap_sink_failure_does_not_loop_or_hide_later_payload(self):
        entered, resume = self.gate(), self.gate()
        def act(record):
            message = record.getMessage()
            if message == 'first':
                entered.set()
                self.require(resume)
            elif message != 'second':
                raise OSError('gap sink unavailable')
        sink = Sink(act)
        self.logger.addHandler(sink)
        bus = self.bus(diagnostic_max_items=2)
        self.emit(bus, 'first')
        self.require(entered)
        self.emit(bus, 'second')
        self.emit(bus, 'third rejected')
        resume.set()
        self.assertTrue(bus.shutdown(timeout_sec=3))
        self.assertEqual(self.state(bus)['gap_writer_failed'], 1)
        self.assertEqual([record.getMessage() for record in sink.records][-1], 'second')
        self.assertEqual(len(sink.records), 3)

    def test_shutdown_is_bounded_retryable_and_drains_accepted_logs(self):
        sink, entered, resume = self.blocked_sink()
        bus = self.bus()
        self.emit(bus, 'first')
        self.require(entered)
        self.emit(bus, 'second')
        self.assertFalse(bus.shutdown(timeout_sec=0.01))
        state = self.state(bus)
        self.assertFalse(state['closed'])
        self.assertTrue(state['worker_alive'])
        resume.set()
        self.assertTrue(bus.shutdown(timeout_sec=3))
        self.assertEqual([record.getMessage() for record in sink.records], ['first', 'second'])
        self.assertTrue(self.state(bus)['closed'])
        self.assertFalse(self.state(bus)['worker_alive'])
        self.assertTrue(bus.shutdown(timeout_sec=0))

    def test_shutdown_fences_inflight_callback_before_diagnostic_close(self):
        sink = Sink()
        self.logger.addHandler(sink)
        bus = self.bus()
        entered, resume = self.gate(), self.gate()
        def callback(frame):
            entered.set()
            self.require(resume)
            self.emit(bus, 'late callback diagnostic')
        bus.subscribe(callback)
        done = self.start(lambda: bus.publish('state', source='check'))
        self.require(entered)
        self.assertFalse(bus.shutdown(timeout_sec=0.01))
        self.assertTrue(self.state(bus)['accepting'])
        resume.set()
        self.require(done)
        self.assertTrue(bus.shutdown(timeout_sec=3))
        self.assertIn('late callback diagnostic', [record.getMessage() for record in sink.records])
        self.assertEqual(self.state(bus)['closed_rejected'], 0)

    def test_real_async_failure_logging_does_not_hold_async_consumer(self):
        sink, entered, resume = self.blocked_sink()
        bus = self.bus()
        delivered = self.gate()
        def callback(frame):
            if frame.topic == 'bad':
                raise ValueError('async callback failure')
            delivered.set()
        bus.subscribe(callback, async_dispatch=True)
        bus.publish('bad', source='check')
        self.require(entered)
        bus.publish('good', source='check')
        self.require(delivered)
        self.assertIn('async callback failure', sink.records[0].getMessage())
        resume.set()

    def test_real_async_gap_logging_does_not_hold_async_consumer(self):
        sink, log_entered, log_resume = self.blocked_sink()
        bus = self.bus()
        callback_entered, callback_resume, last = self.gate(), self.gate(), self.gate()
        def callback(frame):
            if frame.topic == 'first':
                callback_entered.set()
                self.require(callback_resume)
            elif frame.topic == 'third':
                last.set()
        bus.subscribe(callback, async_dispatch=True)
        bus.publish('first', source='check')
        self.require(callback_entered)
        with patch.object(module, '_ASYNC_DISPATCH_QUEUE_MAXSIZE', 1):
            bus.publish('second', source='check')
            bus.publish('third', source='check')
            callback_resume.set()
            self.require(log_entered)
            self.require(last)
        self.assertEqual(sink.records[0].stream_gap['dropped_count'], 1)
        self.assertTrue(sink.records[0].stream_bus_internal)
        log_resume.set()

    def test_shutdown_called_from_writer_is_nonblocking_and_retryable(self):
        bus = self.bus()
        returned, results = self.gate(), []
        def close_from_writer(record):
            results.append(bus.shutdown(timeout_sec=0.01))
            returned.set()
        self.logger.addHandler(Sink(close_from_writer))
        self.emit(bus, 'writer requests close')
        self.require(returned)
        self.assertEqual(results, [False])
        self.assertTrue(bus.shutdown(timeout_sec=3))

    def test_idle_bus_does_not_start_a_diagnostic_thread(self):
        bus = self.bus()
        self.assertFalse(self.state(bus)['worker_alive'])
        self.assertTrue(bus.shutdown(timeout_sec=3))
        self.assertTrue(self.state(bus)['closed'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
