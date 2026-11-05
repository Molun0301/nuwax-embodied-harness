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
    def __init__(self, maps_root, default_root, *, publish, changed, indexer=None, opened=None, failed=None):
        self.maps_root = Path(maps_root)
        self.default_root = Path(default_root)
        self.store = None
        self.expected_rename = None
        self.last_map_name = None
        self.requested_library = object()
        self.error = None
        self._indexer, self._opened, self._failed = indexer, opened, failed
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

    def open_store(self, directory, name):
        return MemoryStore(directory, name=name, indexer=self._indexer)

    def switch(self, name, *, default=False):
        if self.requested_library == name:
            return False
        self.requested_library = name
        old = self.store.name if self.store is not None else None
        try:
            if self.expected_rename == (old, name):
                try:
                    self.rename(old, name)
                finally:
                    self.expected_rename = None
            else:
                if self.store is not None:
                    self.store.close()
                    self.store = None
                if name:
                    self.store = self.open_store(self.directory(name, default=default), name)
            self.last_map_name = name
            self.error = None
            if self._opened is not None:
                self._opened(self.store)
            return True
        except Exception as error:
            self.error = {'operation': 'rename' if self.store is not None and self.store.name == old else 'open',
                          'library_name': name, 'reason': str(error)}
            raise

    def rename(self, old, new):
        current = self.store is not None and self.store.name == old
        instances = self.store.records('instance') if current else None
        source, destination = self.directory(old), self.directory(new)
        if current:
            self.store.close()
            self.store = None
        moved, renamed = False, None
        try:
            destination.parent.mkdir(parents=True, exist_ok=True)
            source.rename(destination)
            moved = True
            renamed = self.open_store(destination, new)
            renamed.rename_records(old, new, instances=instances)
            if current:
                self.store = renamed
                self.last_map_name = new
            else:
                renamed.close()
        except Exception as error:
            if renamed is not None:
                renamed.close()
            if moved:
                destination.rename(source)
            if current:
                self.store = self.open_store(source, old)
                # 未确认实例只属于当前库内存，恢复原库时一并保留。
                for record in instances:
                    if not self.store.has('instance', record.instance_id):
                        self.store.put_live_instance(record, persist=False)
                self.store.commit_cache()
            self.error = {'operation': 'rename', 'library_name': new, 'reason': str(error)}
            raise

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
            try:
                if transaction and self.store is not None:
                    with self.store.connection:
                        result = function(self.store)
                    self.store.commit_cache()
                else:
                    result = function(self.store)
                    if self.store is not None:
                        self.store.commit_cache()
            except BaseException as error:
                if self.store is not None and self.store is previous_store:
                    self.store.rollback_cache()
                if self.error is not None and self._failed is not None:
                    try:
                        self._failed(self.error)
                    except Exception:
                        LOGGER.exception("记忆库失败状态发布失败")
                future.set_exception(error)
                LOGGER.exception('记忆写入失败')
                function = previous_store = item = None
                continue
            if publish and not (not transaction and result is False):
                snapshot = self.store.snapshot() if self.store is not None else MemorySnapshot.empty()
                changed = tuple(self.store.changed) if self.store is not None else ()
                if self.store is not None:
                    self.store.changed.clear()
                # SQL已经提交；发布失败只能记录，不能回滚已提交缓存或伪报业务失败。
                try:
                    self._publish(snapshot)
                except Exception:
                    LOGGER.exception('记忆提交后的快照发布失败')
                if changed:
                    try:
                        self._changed(snapshot.name, changed)
                    except Exception:
                        LOGGER.exception('记忆提交后的向量排队失败')
            future.set_result(result)
            # 阻塞等待下一项前释放已切换库的局部引用。
            function = previous_store = snapshot = result = item = None

    def close(self):
        with self._condition:
            self._queue.append(None)
            self._condition.notify()
        self._thread.join()
