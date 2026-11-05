"""库切换、另存意图与唯一持久写线程。"""
from __future__ import annotations

from concurrent.futures import Future
from collections import deque
import logging
from pathlib import Path
import shutil
import threading

from services.memory.store import MemorySnapshot, MemoryStore

LOGGER = logging.getLogger(__name__)


class LibraryWriter:
    def __init__(self, maps_root, default_root, *, publish, changed):
        self.maps_root = Path(maps_root)
        self.default_root = Path(default_root)
        self.store = None
        self.expected_rename = None
        self.last_map_name = object()
        self._publish = publish
        self._changed = changed
        self._queue = deque()
        self._condition = threading.Condition()
        self._pending_observation = None
        self._thread = threading.Thread(target=self._run, name="memory-writer")
        self._thread.start()

    def call(self, function, *, wait=True, transaction=True, publish=True):
        if threading.current_thread() is self._thread:
            return function(self.store)
        future = Future()
        with self._condition:
            self._queue.append((function, future, transaction, publish))
            self._condition.notify()
        return future.result() if wait else future

    def enqueue_observation(self, function, *, keyframe):
        """待写观察只留最新项；关键帧不会被非关键帧替换。"""
        with self._condition:
            pending = self._pending_observation
            if pending is not None:
                if pending[1] and not keyframe:
                    return
                self._queue.remove(pending[0])
            item = (function, Future(), True, True)
            self._pending_observation = (item, keyframe)
            # 替换项排到本次入队位置，不能越过已经入队的地图切换。
            self._queue.append(item)
            self._condition.notify()

    def directory(self, name, *, default=False):
        return self.default_root if default else self.maps_root / name / "memory"

    def switch(self, name, *, default=False):
        if self.last_map_name == name:
            return False
        self.last_map_name = name
        old = self.store.name if self.store is not None else None
        if self.expected_rename == (old, name):
            self.rename(old, name)
            self.expected_rename = None
            return True
        if self.store is not None:
            self.store.close()
            self.store = None
        if name:
            self.store = MemoryStore(self.directory(name, default=default), name=name)
        return True

    def rename(self, old, new):
        current = self.store is not None and self.store.name == old
        instances = self.store.records("instance") if current else None
        if current:
            self.store.close()
            self.store = None
        destination = self.directory(new)
        destination.parent.mkdir(parents=True, exist_ok=True)
        self.directory(old).rename(destination)
        renamed = MemoryStore(destination, name=new)
        renamed.rename_records(old, new, instances=instances)
        if current:
            self.store = renamed
            self.last_map_name = new
        else:
            renamed.close()

    def expect_rename(self, old, new):
        self.expected_rename = (old, new)

    def clear_expected_rename(self):
        self.expected_rename = None

    def _run(self):
        while True:
            with self._condition:
                while not self._queue:
                    self._condition.wait()
                item = self._queue.popleft()
                if self._pending_observation is not None and self._pending_observation[0] is item:
                    self._pending_observation = None
            if item is None:
                if self.store is not None:
                    self.store.close()
                    self.store = None
                return
            function, future, transaction, publish = item
            previous_store = self.store
            previous_instances = dict(self.store._instances) if self.store is not None else None
            try:
                if transaction and self.store is not None:
                    with self.store.connection:
                        result = function(self.store)
                else:
                    result = function(self.store)
                if not publish or (not transaction and result is False):
                    future.set_result(result)
                    continue
                snapshot = self.store.snapshot(self.store.name) if self.store is not None else MemorySnapshot.empty()
                changed = tuple(self.store.changed) if self.store is not None else ()
                if self.store is not None:
                    self.store.changed.clear()
                self._publish(snapshot)
                if changed:
                    self._changed(snapshot.name, changed)
                future.set_result(result)
            except BaseException as error:
                if self.store is not None and self.store is previous_store:
                    self.store._instances = previous_instances
                    self.store.changed.clear()
                future.set_exception(error)
                LOGGER.exception("记忆写入失败")

    def close(self):
        with self._condition:
            self._queue.append(None)
            self._condition.notify()
        self._thread.join()
