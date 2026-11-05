"""临时抓图的后台保留策略；关闭时排空最后一次维护并回收线程。"""
from __future__ import annotations

import threading
from typing import Optional

from contracts.artifacts import ArtifactRetentionPolicy
from contracts.base import utc_now
from services.artifact_store import LocalArtifactStore


class ArtifactMaintenanceWorker:
    def __init__(self, store: LocalArtifactStore, *, policy: ArtifactRetentionPolicy) -> None:
        self._store = store
        self._policy = policy.model_copy(deep=True)
        self._condition = threading.Condition()
        self._pending = None
        self._active = False
        self._closing = False
        self._thread: Optional[threading.Thread] = None
        self._last_error = None

    def start(self) -> None:
        with self._condition:
            self._start_locked()

    def _start_locked(self) -> None:
        if self._closing:
            raise RuntimeError("制品维护已关闭。")
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="nuwax_artifact_maintenance", daemon=True)
            try:
                self._thread.start()
            except BaseException as error:
                self._thread = None
                self._last_error = str(error)
                raise

    def request(self) -> None:
        with self._condition:
            self._start_locked()
            self._pending = self._policy.model_copy(deep=True), utc_now()
            self._condition.notify_all()

    def update_policy(self, policy: ArtifactRetentionPolicy) -> None:
        with self._condition:
            self._policy = policy.model_copy(deep=True)

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._closing:
                    self._condition.wait()
                if self._pending is None:
                    return
                policy, as_of = self._pending
                self._pending = None
                self._active = True
            error = None
            try:
                self._store.cleanup(policy, as_of=as_of)
            except Exception as failure:
                error = str(failure)
            finally:
                with self._condition:
                    self._last_error = error
                    self._active = False
                    self._condition.notify_all()

    def status(self) -> dict:
        with self._condition:
            return {"thread_alive": self._thread is not None and self._thread.is_alive(),
                    "closing": self._closing, "pending": self._pending is not None,
                    "active": self._active, "error": self._last_error}

    def close(self) -> None:
        with self._condition:
            self._closing = True
            self._condition.notify_all()
            thread = self._thread
        if thread is not None:
            thread.join()
