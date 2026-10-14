"""Private stream diagnostics output owner; never runs a handler on a publisher."""
from __future__ import annotations

from collections import deque
import json
import logging
import threading


class _StreamDiagnosticWriter:
    def __init__(self, logger, *, max_items=64, max_bytes=256 * 1024):
        self._logger = logger
        self._max_items = max(1, int(max_items))
        self._max_bytes = max(1, int(max_bytes))
        self._condition = threading.Condition()
        self._queue = deque()
        self._bytes = 0
        self._active = False
        self._closing = False
        self._thread = None
        self._gap = 0
        self._gap_active = False
        self._last_error_type = None
        self._counts = dict.fromkeys(("accepted", "delivered", "dropped", "capacity_rejected",
                                     "oversize_rejected", "closed_rejected", "preparation_failed",
                                     "writer_failed", "gap_reports", "gap_writer_failed"), 0)

    def _start_locked(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="StreamBus-diagnostic-writer", daemon=True)
            self._thread.start()

    def _reject_locked(self, reason):
        self._counts[reason] += 1
        self._counts["dropped"] += 1
        if not self._closing:
            self._gap += 1
            self._start_locked()
            self._condition.notify_all()

    def preparation_failed(self, error):
        with self._condition:
            self._last_error_type = type(error).__name__
            self._reject_locked("preparation_failed")

    def submit(self, record):
        # LogRecord contains only captured scalars/JSON extras here, never a
        # traceback, exception, frame payload or publisher Context object.
        try:
            body = json.dumps(record.__dict__, ensure_ascii=False, allow_nan=False).encode("utf-8")
        except Exception as error:
            self.preparation_failed(error)
            return False
        with self._condition:
            if self._closing:
                self._reject_locked("closed_rejected")
                return False
            if len(body) > self._max_bytes:
                self._reject_locked("oversize_rejected")
                return False
            if len(self._queue) + int(self._active) >= self._max_items or self._bytes + len(body) > self._max_bytes:
                self._reject_locked("capacity_rejected")
                return False
            self._queue.append(body)
            self._bytes += len(body)
            self._counts["accepted"] += 1
            self._start_locked()
            self._condition.notify_all()
            return True

    def _run(self):
        while True:
            with self._condition:
                while not self._queue and not self._gap and not self._closing:
                    self._condition.wait()
                if not self._queue and not self._gap:
                    return
                gap, self._gap = self._gap, 0
                self._gap_active = bool(gap)
                body = self._queue.popleft() if self._queue else None
                self._active = body is not None
            # Take a gap and a queued record together: a stream of new rejected
            # diagnostics cannot starve records already accepted before the gap.
            if gap:
                try:
                    self._logger.warning("流诊断日志输出缺口: dropped=%d", gap,
                                         extra={"stream_bus_internal": True, "diagnostic_dropped": gap})
                    with self._condition:
                        self._counts["gap_reports"] += 1
                except BaseException as error:
                    with self._condition:
                        self._last_error_type = type(error).__name__
                        self._counts["gap_writer_failed"] += 1
                finally:
                    with self._condition:
                        self._gap_active = False
            if body is not None:
                try:
                    self._logger.handle(logging.makeLogRecord(json.loads(body)))
                    with self._condition:
                        self._counts["delivered"] += 1
                except BaseException as error:
                    # A broken sink is observable, but cannot recursively log
                    # its own failure or stop subsequent diagnostic delivery.
                    with self._condition:
                        self._last_error_type = type(error).__name__
                        self._counts["writer_failed"] += 1
                        self._counts["dropped"] += 1
                        self._gap += 1
                finally:
                    with self._condition:
                        self._bytes -= len(body)
                        self._active = False
                        self._condition.notify_all()

    def status(self):
        with self._condition:
            alive = self._thread is not None and self._thread.is_alive()
            return {"accepting": not self._closing, "closed": self._closing and not alive,
                    "worker_alive": alive, "outstanding_items": len(self._queue) + int(self._active),
                    "outstanding_bytes": self._bytes, "max_items": self._max_items,
                    "max_bytes": self._max_bytes, "pending_gap": self._gap,
                    "gap_in_flight": self._gap_active,
                    "last_error_type": self._last_error_type, **self._counts}

    def close(self, timeout_sec=2.0):
        with self._condition:
            self._closing = True
            self._condition.notify_all()
            thread = self._thread
        if thread is None:
            return True
        if thread is threading.current_thread():
            return False
        thread.join(max(0.0, timeout_sec))
        return not thread.is_alive()
