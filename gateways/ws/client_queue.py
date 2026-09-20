from __future__ import annotations

import asyncio
from collections import deque
import threading
from typing import Any, Callable, Optional


class ClientQueue:
    """跨线程有界展示队列；事件循环停顿时只保留一次唤醒。"""

    def __init__(self, loop, *, maxsize: int, is_terminal: Optional[Callable[[Any], bool]] = None):
        self._loop = loop
        self._items = deque()
        self._maxsize = max(1, int(maxsize))
        self._is_terminal = is_terminal
        self._lock = threading.Lock()
        self._ready = asyncio.Event()
        self._notification_pending = False
        self._closed = False
        self._drops = None

    def put(self, item: Any) -> None:
        with self._lock:
            if self._closed:
                return
            if len(self._items) == self._maxsize:
                dropped = self._items.popleft()
                cursor = dropped.get("cursor") if isinstance(dropped, dict) else getattr(dropped, "cursor", None)
                if self._drops is None:
                    self._drops = {"dropped_count": 0, "first_dropped_cursor": cursor, "terminal_dropped": False}
                self._drops["dropped_count"] += 1
                self._drops["last_dropped_cursor"] = cursor
                if self._is_terminal is not None and self._is_terminal(dropped):
                    self._drops["terminal_dropped"] = True
            self._items.append(item)
            if self._notification_pending:
                return
            self._notification_pending = True
        try:
            self._loop.call_soon_threadsafe(self._wake)
        except RuntimeError:
            self.close()

    def _wake(self) -> None:
        with self._lock:
            self._notification_pending = False
        self._ready.set()

    async def get(self) -> Any:
        while True:
            with self._lock:
                if self._closed:
                    raise asyncio.CancelledError
                if self._items:
                    return self._items.popleft()
                self._ready.clear()
            await self._ready.wait()

    def take_drops(self):
        with self._lock:
            drops, self._drops = self._drops, None
            return drops

    def close(self) -> None:
        with self._lock:
            self._closed = True
            self._items.clear()
            self._drops = None
