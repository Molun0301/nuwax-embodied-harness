"""One lifecycle owner for restartable, process-isolated inference."""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable, Dict, Optional

from services.perception.inference_worker import YoloInferenceWorker


class InferenceProcessOwner:
    """Recover one worker generation off the request path, until explicitly closed."""

    def __init__(
        self,
        model_source: str,
        *,
        retry_backoff_sec: float,
        worker_factory: Callable[..., Any] = YoloInferenceWorker,
        **worker_kwargs,
    ) -> None:
        self._backoff = float(retry_backoff_sec)
        if not math.isfinite(self._backoff) or self._backoff <= 0:
            raise ValueError("inference recovery backoff must be positive and finite")
        self._source, self._factory, self._kwargs = model_source, worker_factory, worker_kwargs
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._thread = None
        self._worker = None
        self._faulted = False
        self._state = "new"
        self._failure = None
        self._last_inference_ms = None
        self._next_start_at = 0.0
        self._attempts = 0

    def start_nonblocking(self) -> bool:
        with self._condition:
            if self._stop.is_set():
                return False
            if self._thread is not None:
                return True
            if time.monotonic() < self._next_start_at:
                return False
            thread = threading.Thread(target=self._run, name="nuwax-inference-owner", daemon=True)
            self._thread = thread
            self._state = "starting"
            try:
                thread.start()
            except Exception as exc:  # noqa: BLE001 - Thread creation failure is retryable.
                self._thread = None
                self._state = "backoff"
                self._failure = str(exc)
                self._next_start_at = time.monotonic() + self._backoff
                return False
            return True

    def status(self) -> Dict[str, object]:
        with self._condition:
            return {
                "state": self._state,
                "failure_reason": self._failure,
                "start_attempts": self._attempts,
                "worker_owned": self._worker is not None,
                "lifecycle_alive": bool(self._thread is not None and self._thread.is_alive()),
            }

    @property
    def failure_reason(self) -> Optional[str]:
        with self._condition:
            return self._failure

    @property
    def last_inference_ms(self) -> Optional[float]:
        with self._condition:
            return self._last_inference_ms

    def predict(self, image_bgr, *, timeout_sec=10.0):
        with self._condition:
            self._last_inference_ms = None
            worker = self._worker
            if self._stop.is_set() or self._state != "ready" or worker is None:
                return None
        # Shared-memory ownership stays with the worker's existing prediction lock.
        # Reaping is handed to this lifecycle owner rather than the request executor.
        try:
            result = worker.predict(image_bgr, timeout_sec=timeout_sec, cleanup_on_failure=False)
            failure = worker.failure_reason or "worker_prediction_unavailable"
        except Exception as exc:  # noqa: BLE001 - Lifecycle recovery owns backend failures.
            result, failure = None, str(exc)
        with self._condition:
            # A result from an old/closing generation cannot update the new owner's state.
            if self._worker is not worker or self._stop.is_set() or self._state != "ready":
                return None
            if result is None:
                self._faulted = True
                self._state = "recovering"
                self._failure = failure
                self._condition.notify_all()
            else:
                self._last_inference_ms = worker.last_inference_ms
            return result

    def update_thresholds(self, *, confidence_threshold, iou_threshold, max_detections) -> None:
        with self._condition:
            self._kwargs.update(
                confidence_threshold=confidence_threshold, iou_threshold=iou_threshold, max_detections=max_detections
            )
            worker = self._worker
        if worker is not None:
            worker.update_thresholds(
                confidence_threshold=confidence_threshold, iou_threshold=iou_threshold, max_detections=max_detections
            )

    def close(self, timeout=5.0) -> bool:
        self._stop.set()
        with self._condition:
            self._state = "stopping" if self._thread is not None else "stopped"
            self._condition.notify_all()
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(max(0.0, float(timeout)))
        with self._condition:
            complete = self._worker is None and (thread is None or not thread.is_alive())
            if complete:
                self._state = "stopped"
            return complete

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                worker = None
                try:
                    with self._condition:
                        if self._stop.is_set():
                            break
                        self._state = "starting"
                        self._attempts += 1
                        worker = self._factory(self._source, **self._kwargs)
                        self._worker = worker
                        self._faulted = False
                    ready = worker.start_nonblocking() and worker.wait_until_ready(cancel_event=self._stop)
                    with self._condition:
                        if ready and not self._stop.is_set():
                            self._state = "ready"
                            self._failure = None
                            self._condition.notify_all()
                            while not self._stop.is_set() and not self._faulted:
                                self._condition.wait_for(
                                    lambda: self._stop.is_set() or self._faulted,
                                    timeout=self._backoff,
                                )
                                if not self._stop.is_set() and not self._faulted and not worker.is_ready:
                                    self._faulted = True
                                    self._state = "recovering"
                                    self._failure = worker.failure_reason or "worker_exited_while_idle"
                        elif not self._stop.is_set():
                            self._failure = worker.failure_reason or "worker_start_failed"
                except Exception as exc:  # noqa: BLE001 - Expose failure and retry on the same owner.
                    with self._condition:
                        self._failure = str(exc)
                finally:
                    if worker is not None:
                        # Never replace a generation until all its process/IPC resources are reaped.
                        while True:
                            try:
                                worker.stop()
                                if not worker.has_pending_resources:
                                    break
                            except Exception as exc:  # noqa: BLE001 - Retain ownership on cleanup failure.
                                with self._condition:
                                    self._failure = str(exc)
                            with self._condition:
                                self._state = "stopping" if self._stop.is_set() else "recovering"
                                # Even terminal cleanup retries must not spin on a set stop event.
                                retry_at = time.monotonic() + self._backoff
                                while time.monotonic() < retry_at:
                                    self._condition.wait(timeout=max(0.0, retry_at - time.monotonic()))
                    with self._condition:
                        self._worker = None
                if self._stop.is_set():
                    break
                with self._condition:
                    self._state = "backoff"
                    self._condition.wait_for(self._stop.is_set, timeout=self._backoff)
        finally:
            with self._condition:
                self._state = "stopped"
                self._condition.notify_all()
