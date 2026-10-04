"""Explicit shared SQLite transaction ownership for memory backends.

Sharing this *object*, not merely its path, provides atomic source/outbox/vector
writes and a genuinely shared ``:memory:`` database. Existing backends are not
changed by this module. Injected external backends need their own explicit
protocol; a matching filename does not make them transaction participants.
"""

from __future__ import annotations

import re
import sqlite3
import threading

from services.memory.scene_revisions import compare_scene_timestamps
from collections import deque
from contextlib import contextmanager
from pathlib import Path
from typing import Callable, Iterator, List, Optional, Sequence, Tuple

_TRANSACTION_SQL = re.compile(
    r"\A(?:\s|--[^\n]*(?:\n|$)|/\*.*?\*/)*(BEGIN|COMMIT|END|ROLLBACK|SAVEPOINT|RELEASE)\b",
    re.IGNORECASE | re.DOTALL,
)


class SQLiteCompletion:
    """最外层写事务的只读结束证明，可在作用域退出后继续保留。

    settled 为 True 仅证明该事务已不可能再通过此连接提交，不代表提交或
    回滚结果已知。读取只访问已捕获的状态，绝不查询连接或获取写锁。
    回滚失败后，只有 owner 后续写边界确认原事务结束或实际关闭连接成功，
    证明才变为 True；独立 WAL 读取不会为此重新获取写锁。
    """

    __slots__ = ("_settled",)

    def __init__(self) -> None:
        self._settled = False

    @property
    def settled(self) -> bool:
        return self._settled


class SQLiteTransaction:
    """Scoped SQL facade, deliberately without commit/rollback/executescript.

    SQL transaction control is rejected by the facade. This is an internal API,
    not a sandbox for untrusted SQL. Cursors are
    for fetching results, not for retaining/accessing their raw connection.
    A facade cannot be used outside its scope or from a different thread.
    """

    def __init__(self, owner: SQLiteStorage, connection: sqlite3.Connection, *, writable: bool,
                 completion: Optional[SQLiteCompletion] = None) -> None:
        self.owner = owner
        self._connection = connection
        self._thread = threading.get_ident()
        self._active = True
        self._writable = writable
        self._callbacks: List[Tuple[Callable[[], None], bool]] = []
        self._callback_ids = set()
        self._completion_callbacks: List[Tuple[Callable[[bool], None], bool]] = []
        self._completion = completion if completion is not None else SQLiteCompletion()

    @property
    def completion(self) -> SQLiteCompletion:
        """当前最外层写事务的结束证明；嵌套作用域共享同一个对象。"""
        if not self._writable:
            raise RuntimeError("Completion proof requires a writer transaction")
        return self._completion

    def _check(self) -> None:
        if not self._active or self._thread != threading.get_ident():
            raise RuntimeError("SQLite transaction is outside its owning thread/scope")
        if not self._connection.in_transaction:
            # SQLite may roll back the ENTIRE transaction on OR ROLLBACK,
            # trigger RAISE(ROLLBACK), disk-full, or I/O failure. Even if a caller
            # catches that error, no facade may execute into autocommit mode.
            raise RuntimeError("SQLite transaction was rolled back; the whole scope is unusable")

    def execute(self, sql: str, parameters: Sequence[object] = ()) -> sqlite3.Cursor:
        self._check()
        if _TRANSACTION_SQL.match(sql):
            raise ValueError("Transaction control belongs to SQLiteStorage")
        return self._connection.execute(sql, parameters)

    def executemany(self, sql: str, parameters: object) -> sqlite3.Cursor:
        self._check()
        if _TRANSACTION_SQL.match(sql):
            raise ValueError("Transaction control belongs to SQLiteStorage")
        return self._connection.executemany(sql, parameters)

    def after_commit(self, callback: Callable[[], None], *, deduplicate: bool = False) -> None:
        """Register a notification; optionally coalesce one identical wake hint.

        Deduplication is opt-in and spans enclosing savepoints. It never changes
        ordinary callbacks that require one invocation per registered operation.
        """
        self._check()
        if not self._writable:
            raise RuntimeError("Post-commit hooks require a writer transaction")
        if deduplicate:
            # A descendant's registration can still roll back independently;
            # only this scope and its ancestors can cover this notification.
            for frame in getattr(self.owner._local, "frames", ()):
                if id(callback) in frame._callback_ids:
                    return
                if frame is self:
                    break
        self._callbacks.append((callback, deduplicate))
        self._callback_ids.add(id(callback))

    def after_completion(self, callback: Callable[[bool], None]) -> None:
        """在最外层事务退出且释放写锁后，每次注册恰好通知一次。

        成功的保存点随父事务提交；已回滚子作用域的通知仍等待最外层退出，
        并收到 False。True 仅表示本作用域及最外层 COMMIT 均已确认成功。
        False 也包含 COMMIT 结果未确认的异常，不能据此断言数据库已回滚。
        回滚本身失败时 completion.settled 可能仍为 False；此时不能仅凭
        回调已执行就释放事务仍可能提交的数据所依赖的外部资源。
        通知异常（包括 BaseException）只记录诊断，不覆盖事务结果。
        """
        self._check()
        if not self._writable:
            raise RuntimeError("Completion hooks require a writer transaction")
        # 每次注册可能对应独立资源的释放，不能像唤醒提示一样合并。
        self._completion_callbacks.append((callback, True))


class SQLiteStorage:
    """One writer connection, RLock, nested savepoints, and optional WAL reads.

    Helpers nest with SAVEPOINT, never commit an outer transaction. Inner
    rollback discards its commit hooks; successful inner commit hooks run only
    after outer commit, outside the lock. Completion hooks run after outer exit,
    before commit hooks, including for rolled-back savepoints. Hooks are
    notifications, not durable work.

    ``read_snapshot`` reads its own uncommitted writes on the current thread.
    Otherwise file-backed reads use independent query-only connections without
    taking the writer lock; in-memory reads share the writer/lock. This preserves
    a route for WAL read concurrency when existing stores are integrated later.
    One reused query-only connection serializes independent readers, not writers.
    All participant writes must use this owner's transaction facade.
    """

    def __init__(self, db_path: str = ":memory:", *, timeout: float = 5.0) -> None:
        self.db_path = str(db_path).strip() or ":memory:"
        self._timeout = timeout
        self._lock = threading.RLock()
        self._local = threading.local()
        self._reader_lock = threading.RLock()
        self._reader = None
        self._notification_errors = deque(maxlen=16)
        self.notification_failure_count = 0
        self._completion_errors = deque(maxlen=16)
        self._completion_error_lock = threading.Lock()
        self.completion_failure_count = 0
        self._rollback_errors = deque(maxlen=16)
        self.rollback_failure_count = 0
        self._unsettled_completion: Optional[SQLiteCompletion] = None
        self._closed = False
        self._savepoint = 0
        if self.db_path != ":memory:":
            self.db_path = str(Path(self.db_path).resolve())
            Path(self.db_path).parent.mkdir(parents=True, exist_ok=True)
        self._connection = self._connect()
        if self.db_path != ":memory:":
            self._connection.execute("PRAGMA journal_mode=WAL")
        # FULL is intentional for acknowledged file-backed admissions. This is
        # still subject to the filesystem/device honoring SQLite's fsync calls.
        self._connection.execute("PRAGMA synchronous=FULL")
        self._connection.execute("PRAGMA foreign_keys=ON")
        if self.db_path != ":memory:":
            self._reader = self._connect()
            self._reader.execute("PRAGMA query_only=ON")

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(
            self.db_path,
            timeout=self._timeout,
            isolation_level=None,
            check_same_thread=False,
        )
        connection.row_factory = sqlite3.Row
        connection.create_collation("NEH_SCENE_TIMESTAMP", compare_scene_timestamps)
        return connection

    @staticmethod
    def _control(connection: sqlite3.Connection, statement: str) -> None:
        connection.execute(statement)

    @property
    def notification_errors(self):
        """Bounded post-commit diagnostics; errors never undo accepted work.

        Only exception types are retained, not arbitrary potentially sensitive
        callback messages/tracebacks. Durable receipts remain the authority.
        """
        with self._lock:
            return tuple(self._notification_errors)

    @property
    def completion_errors(self):
        """完成通知的有界异常类型诊断；不混入既有提交通知计数。"""
        with self._completion_error_lock:
            return tuple(self._completion_errors)

    @property
    def rollback_errors(self):
        """回滚清理失败的有界诊断；原始事务异常仍然向调用方传播。"""
        with self._lock:
            return tuple(self._rollback_errors)

    @property
    def current_transaction(self) -> Optional[SQLiteTransaction]:
        """Current-thread writer facade; never another thread's transaction."""
        frames = getattr(self._local, "frames", ())
        return frames[-1] if frames else None

    @property
    def in_transaction(self) -> bool:
        return self.current_transaction is not None

    def _connection_is_settled(self) -> bool:
        try:
            return not self._connection.in_transaction
        except BaseException:  # noqa: BLE001 - 结束证明采集失败时保守保留，不能覆盖事务异常
            return False

    def _reconcile_completion_locked(self) -> None:
        # 未结束的 SQLite 事务会阻止下一次 BEGIN，因此最多只需保留一个证明。
        # 直接操作原始连接不受 facade 支持；后续 owner 写边界仍可确认其已结束。
        pending = self._unsettled_completion
        if pending is not None and self._connection_is_settled():
            pending._settled = True
            self._unsettled_completion = None

    @contextmanager
    def transaction(self) -> Iterator[SQLiteTransaction]:
        """BEGIN IMMEDIATE at the outer boundary; SAVEPOINT when nested."""
        callbacks: List[Tuple[Callable[[], None], bool]] = []
        completions: List[Tuple[Callable[[bool], None], bool]] = []
        if getattr(self._local, "reader", None) is not None:
            raise RuntimeError("Cannot upgrade a separate read snapshot to a writer transaction")
        try:
            with self._lock:
                if self._closed:
                    raise RuntimeError("SQLite storage is closed")
                self._reconcile_completion_locked()
                frames = getattr(self._local, "frames", None)
                if frames is None:
                    frames = self._local.frames = []
                nested = bool(frames)
                if nested:
                    frames[-1]._check()
                self._savepoint += 1
                name = f"memory_sp_{self._savepoint}"
                self._control(self._connection, "SAVEPOINT " + name if nested else "BEGIN IMMEDIATE")
                completion = frames[-1]._completion if nested else SQLiteCompletion()
                frame = SQLiteTransaction(self, self._connection, writable=True, completion=completion)
                frames.append(frame)
                succeeded = False
                try:
                    yield frame
                    frame._check()
                    self._control(self._connection, "RELEASE " + name if nested else "COMMIT")
                    succeeded = True
                except BaseException:
                    try:
                        if self._connection.in_transaction:
                            if nested:
                                self._control(self._connection, "ROLLBACK TO " + name)
                                self._control(self._connection, "RELEASE " + name)
                            else:
                                self._control(self._connection, "ROLLBACK")
                    except BaseException as error:  # noqa: BLE001 - 清理失败不能覆盖原始事务异常
                        self.rollback_failure_count += 1
                        self._rollback_errors.append(type(error).__name__[:128])
                    raise
                else:
                    if nested:
                        parent = frames[-2]
                        for callback, deduplicate in frame._callbacks:
                            if deduplicate and id(callback) in parent._callback_ids:
                                continue
                            parent._callbacks.append((callback, deduplicate))
                            parent._callback_ids.add(id(callback))
                    else:
                        callbacks = frame._callbacks
                finally:
                    frame._active = False
                    frames.pop()
                    # 子事务失败也保留释放通知；递归写锁彻底退出前绝不调用。
                    completed = [(callback, eligible and succeeded)
                                 for callback, eligible in frame._completion_callbacks]
                    if nested:
                        frames[-1]._completion_callbacks.extend(completed)
                    else:
                        # 失败的 ROLLBACK 可能留下可提交事务；通知结束不等于 SQL 已结束。
                        if self._connection_is_settled():
                            completion._settled = True
                        else:
                            self._unsettled_completion = completion
                        completions = completed
        finally:
            # 必须位于 with _lock 外层的 finally，异常展开路径同样释放持有资源。
            for callback, committed in completions:
                try:
                    callback(committed)
                except BaseException as error:  # noqa: BLE001 - 资源释放通知不能覆盖 SQL 结果
                    # 诊断也不重取写锁，避免其他写者等待后续资源释放时形成死锁。
                    with self._completion_error_lock:
                        self.completion_failure_count += 1
                        self._completion_errors.append(type(error).__name__[:128])
        for callback, _deduplicate in callbacks:
            try:
                callback()
            except Exception as error:  # noqa: BLE001 - notification failure cannot undo a durable commit
                with self._lock:
                    self.notification_failure_count += 1
                    self._notification_errors.append(type(error).__name__[:128])

    @contextmanager
    def read_snapshot(self) -> Iterator[SQLiteTransaction]:
        """Read queries only; reuse a WAL reader without taking the writer lock.

        Independent file reads enforce query_only and cannot upgrade to writers.
        Read-your-own-writes and :memory: reads reuse the serialized writer
        facade, so those internal callers must honor the read-only contract.
        """
        current = self.current_transaction
        if current is not None:
            yield current
            return
        if self.db_path == ":memory:":
            with self.transaction() as transaction:
                yield transaction
            return
        existing = getattr(self._local, "reader", None)
        if existing is not None:
            yield existing
            return
        with self._reader_lock:
            if self._closed:
                raise RuntimeError("SQLite storage is closed")
            connection = self._reader
            frame = SQLiteTransaction(self, connection, writable=False)
            self._control(connection, "BEGIN")
            self._local.reader = frame
            try:
                yield frame
            finally:
                frame._active = False
                self._local.reader = None
                if connection.in_transaction:
                    self._control(connection, "ROLLBACK")

    def close(self) -> None:
        if self.in_transaction or getattr(self._local, "reader", None) is not None:
            raise RuntimeError("Cannot close SQLite storage inside a transaction/snapshot")
        with self._reader_lock, self._lock:
            if not self._closed:
                self._connection.close()
                # 只有实际关闭成功才确认残留事务不可能再提交。
                if self._unsettled_completion is not None:
                    self._unsettled_completion._settled = True
                    self._unsettled_completion = None
                if self._reader is not None:
                    self._reader.close()
                self._closed = True
