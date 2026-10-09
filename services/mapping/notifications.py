"""Completion ownership only; accepted payloads remain in their original buses."""
from __future__ import annotations

import threading


class MappingNotifications:
    def __init__(self, event_bus, stream_bus):
        self._event_bus, self._stream_bus = event_bus, stream_bus
        self._condition = threading.Condition()
        self._event = self._stream = None
        self._active = False
        self._sealed = False
        self._gaps = {"event": 0, "map.frame": 0, "map.heartbeat": 0}
        self._dispatch_failures = 0
        self._initial_outcomes = stream_bus.status() if stream_bus is not None else {}
        self._initial_outcomes = {key: self._initial_outcomes.get(key, 0) for key in (
            "map_frame_interrupted_attempts", "map_frame_cancelled_attempts")}
        self._last_failure = None
        self._thread = None

    def handoff(self, *, event=None, stream=None):
        # Called under MappingService's publication lock, after bus admission.
        # Only fences are compressed. Each bus still owns every accepted item.
        with self._condition:
            if event is not None:
                self._event = event
            if stream is not None and stream.dispatch_item is not None:
                self._stream = stream
            if self._event is None and self._stream is None:
                return
            self._start_locked()
            self._condition.notify_all()

    def _start_locked(self):
        if self._thread is not None or not self._pending_locked():
            return
        worker = threading.Thread(target=self._run, name="mapping-notifications", daemon=True)
        try:
            worker.start()
        except RuntimeError as error:
            # The buses already accepted these items. Keep their fences so a
            # later handoff/flush can retry the owner, never the map commit.
            self._last_failure = {"channel": "owner", "error_type": type(error).__name__}
        else:
            self._thread = worker

    def gap(self, channel, error):
        # No inline warning: the installed log handler synchronously publishes
        # logs.entry and would put provider ingress back behind a held observer.
        with self._condition:
            self._gaps[channel] += 1
            self._last_failure = {"channel": channel, "error_type": type(error).__name__}

    def status(self):
        stream_status = self._stream_bus.status() if self._stream_bus is not None else {}
        outcomes = {key: stream_status.get(key, 0) - initial for key, initial in self._initial_outcomes.items()}
        with self._condition:
            return {"accepting": not self._sealed, "pending": self._pending_locked(),
                    "closed": self._sealed and not self._pending_locked(),
                    "event_gaps": self._gaps["event"],
                    "map_frame_admission_gaps": self._gaps["map.frame"],
                    "heartbeat_admission_gaps": self._gaps["map.heartbeat"],
                    **outcomes,
                    "dispatch_failures": self._dispatch_failures,
                    "incomplete": any(self._gaps.values()) or any(outcomes.values()) or bool(self._dispatch_failures),
                    "last_failure": dict(self._last_failure) if self._last_failure else None}

    def seal(self):
        with self._condition:
            self._sealed = True
            self._condition.notify_all()

    def _pending_locked(self):
        return self._active or self._event is not None or self._stream is not None

    def is_dispatch_thread(self):
        return (self._thread is threading.current_thread()
                or any(bus is not None and bus.is_dispatch_thread()
                       for bus in (self._event_bus, self._stream_bus)))

    def flush(self, timeout):
        # An unrelated publisher may be draining a mapping item on its own
        # thread. Checking only this owner's worker would miss that self-wait.
        if self.is_dispatch_thread():
            return False
        with self._condition:
            self._start_locked()
            return self._condition.wait_for(lambda: not self._pending_locked(), timeout=max(0.0, timeout))

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._event is not None or self._stream is not None or self._sealed)
                event, stream = self._event, self._stream
                self._event = self._stream = None
                if event is None and stream is None:
                    return
                self._active = True
            try:
                for channel, bus, publication in (("event", self._event_bus, event),
                                                   ("stream", self._stream_bus, stream)):
                    if publication is not None:
                        try:
                            bus.dispatch_prepared(publication)
                        except BaseException as error:
                            # StreamBus has completed the attempted item and
                            # later FIFO work before rethrowing. Never replay it.
                            with self._condition:
                                self._dispatch_failures += 1
                                self._last_failure = {"channel": channel, "error_type": type(error).__name__}
            finally:
                with self._condition:
                    self._active = False
                    self._condition.notify_all()
