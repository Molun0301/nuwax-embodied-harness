from __future__ import annotations

from dataclasses import dataclass, field
import queue
import threading
from typing import Optional, Union

from services.navigation.proto_gen import motion_guard_pb2 as pb

from .constants import ACTOR_POLL_INTERVAL_SECONDS
from .core import MotionGuardCore
from .peer_auth import PeerCredentials


@dataclass
class _ActorCall:
    request: pb.MotionGuardRequest
    peer: PeerCredentials
    completed: threading.Event = field(default_factory=threading.Event)
    response: Optional[pb.MotionGuardResponse] = None
    error: Optional[BaseException] = None


@dataclass
class _FenceCall:
    completed: threading.Event = field(default_factory=threading.Event)
    error: Optional[BaseException] = None


_STOP = object()


class GuardActor:
    """Owns all mutable guard state and actuator calls on one thread."""

    def __init__(self, core: MotionGuardCore) -> None:
        self._core = core
        self._queue: queue.Queue[Union[_ActorCall, _FenceCall, object]] = queue.Queue()
        self._thread: Optional[threading.Thread] = None
        self._started = threading.Event()
        self.thread_id: Optional[int] = None

    def start(self) -> None:
        if self._thread is not None:
            return
        thread = threading.Thread(
            target=self._run,
            name="nuwax-motion-guard-actor",
            daemon=True,
        )
        self._thread = thread
        thread.start()
        if not self._started.wait(timeout=2.0):
            raise RuntimeError("motion guard actor failed to start")

    def call(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        *,
        timeout_seconds: float,
    ) -> pb.MotionGuardResponse:
        thread = self._thread
        if thread is None or not thread.is_alive():
            raise RuntimeError("motion guard actor is not running")
        call = _ActorCall(request=request, peer=peer)
        self._queue.put(call)
        if not call.completed.wait(timeout_seconds):
            raise TimeoutError("motion guard actor request timed out")
        if call.error is not None:
            raise call.error
        if call.response is None:
            raise RuntimeError("motion guard actor returned no response")
        return call.response

    def stop(self) -> None:
        thread, self._thread = self._thread, None
        if thread is None:
            return
        self._queue.put(_STOP)
        thread.join(timeout=2.0)
        if thread.is_alive():
            raise RuntimeError("motion guard actor did not stop")
        self.thread_id = None
        self._started.clear()

    def fence(self, *, timeout_seconds: float = 3.0) -> None:
        thread = self._thread
        if thread is None or not thread.is_alive():
            raise RuntimeError("motion guard actor is not running")
        call = _FenceCall()
        self._queue.put(call)
        if not call.completed.wait(timeout_seconds):
            raise TimeoutError("motion guard graceful fence timed out")
        if call.error is not None:
            raise call.error

    def _run(self) -> None:
        self.thread_id = threading.get_ident()
        self._started.set()
        while True:
            self._core.tick()
            try:
                item = self._queue.get(timeout=ACTOR_POLL_INTERVAL_SECONDS)
            except queue.Empty:
                continue
            if item is _STOP:
                return
            if isinstance(item, _FenceCall):
                try:
                    self._core.force_safe_fence()
                except BaseException as exc:
                    item.error = exc
                finally:
                    item.completed.set()
                continue
            assert isinstance(item, _ActorCall)
            try:
                item.response = self._core.handle(item.request, item.peer)
            except BaseException as exc:
                item.error = exc
            finally:
                self._core.tick()
                item.completed.set()
