"""独立终态回执文件；作用域授权始终由调用意图库提供，只恢复记录不重放动作。"""
from __future__ import annotations

import os
import sqlite3
import time
from contextlib import closing, contextmanager
from pathlib import Path

from core.errors import GatewayError

TERMINAL_FIELDS = ("task_id", "scope", "gateway_instance_id", "snapshot_json", "updated_at",
                   "receipt_source_id", "receipt_revision")
TERMINAL_SCHEMA = """CREATE TABLE agent_task_terminals (
    task_id TEXT PRIMARY KEY, scope TEXT NOT NULL, gateway_instance_id TEXT NOT NULL,
    snapshot_json TEXT NOT NULL, updated_at TEXT NOT NULL,
    receipt_source_id TEXT, receipt_revision INTEGER)"""


def pair_conflict():
    return GatewayError("回执文件与原调用账本的配对不一致；请核对原文件，不得重发动作。",
                        error_code="agent_terminal_pair_conflict", http_status=409)


def sync_directory(path):
    fd = os.open(str(Path(path)), os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def initialize_terminal_file(path, ledger_id, pair_id):
    """仅初始化已明确指定的配对。现有不匹配文件不能被认领或清空。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    try:
        fd = os.open(str(path), os.O_CREAT | os.O_EXCL | os.O_RDWR, 0o600)
    except FileExistsError:
        # 同一新账本的并发构造可能正在提交文件头；仅短暂等待，不猜测其身份。
        deadline = time.monotonic() + 5.0
        while True:
            with closing(sqlite3.connect(path.resolve().as_uri() + "?mode=rw", uri=True, timeout=5.0)) as db:
                table = db.execute("SELECT name FROM sqlite_master WHERE name='terminal_store_identity'").fetchone()
                if table:
                    row = db.execute("SELECT ledger_id,pair_id FROM terminal_store_identity WHERE singleton=1").fetchone()
                    if row != (ledger_id, pair_id):
                        raise pair_conflict()
                    sync_directory(path.parent)
                    return
                tables = db.execute("SELECT name FROM sqlite_master WHERE type='table'").fetchone()
                if tables or time.monotonic() >= deadline:
                    raise pair_conflict()
            time.sleep(0.025)
    else:
        os.close(fd)
    db = sqlite3.connect(str(path), timeout=5.0, isolation_level=None)
    try:
        db.execute("PRAGMA synchronous=FULL")
        db.execute("BEGIN IMMEDIATE")
        db.execute("CREATE TABLE terminal_store_identity (singleton INTEGER PRIMARY KEY CHECK(singleton=1),"
                   "ledger_id TEXT NOT NULL,pair_id TEXT NOT NULL)")
        db.execute("INSERT INTO terminal_store_identity VALUES (1,?,?)", (ledger_id, pair_id))
        db.execute(TERMINAL_SCHEMA)
        db.execute("CREATE INDEX agent_terminals_scope ON agent_task_terminals(scope)")
        db.commit()
    except BaseException:
        db.rollback()
        raise
    finally:
        db.close()
    sync_directory(path.parent)


class AgentTerminalStore:
    """配对元数据和不可变旧表使首次播种/CAS不依赖跨库事务。"""

    def __init__(self, authority, layout):
        self.authority = authority
        self.path = Path(layout["terminal_path"])
        self.ledger_id = layout["ledger_id"]
        self.pair_id = layout["pair_id"]

    @contextmanager
    def _connection(self):
        # 活跃文件丢失时绝不新建空库；读不到目标不等于目标没有该任务。
        connection = sqlite3.connect(self.path.resolve().as_uri() + "?mode=rw", uri=True,
                                     timeout=5.0, isolation_level=None)
        connection.row_factory = sqlite3.Row
        try:
            connection.execute("PRAGMA synchronous=FULL")
            try:
                row = connection.execute("SELECT ledger_id,pair_id FROM terminal_store_identity WHERE singleton=1").fetchone()
            except sqlite3.OperationalError as error:
                if "no such table" in str(error):
                    raise pair_conflict() from error
                raise
            if row is None or (row["ledger_id"], row["pair_id"]) != (self.ledger_id, self.pair_id):
                raise pair_conflict()
            yield connection
        finally:
            connection.close()

    def _target_rows(self, ids, *, full):
        fields = "*" if full else "task_id,scope,receipt_source_id,receipt_revision"
        with self._connection() as db:
            return {row["task_id"]: dict(row) for row in db.execute(
                "SELECT " + fields + " FROM agent_task_terminals WHERE task_id IN ("
                + ",".join("?" for _ in ids) + ")", ids)}

    def rows(self, ids, *, full=True):
        rows = self._target_rows(ids, full=full)
        missing = tuple(task_id for task_id in ids if task_id not in rows)
        if missing:
            # 旧表已封存；此连接在任何目标写事务前关闭。
            rows.update(self.authority._legacy_terminal_rows(missing, full=full))
        return rows

    def authorize(self, rows, *, error_code):
        with self.authority._read_connection() as db:
            for row in rows:
                self.authority._assert_scope(db, row["scope"], error_code=error_code,
                                             message="任务历史回执属于另一作用域。")

    def versions(self, ids):
        rows = self.rows(ids, full=False)
        versions = {}
        with self.authority._read_connection() as db:
            for row in rows.values():
                error_code = None
                try:
                    self.authority._assert_scope(db, row["scope"], error_code="agent_terminal_scope_conflict",
                                                 message="任务历史回执属于另一作用域。")
                except GatewayError as error:
                    error_code = error.error_code
                if error_code is None and (row["receipt_source_id"] is None or row["receipt_revision"] is None):
                    error_code = "agent_terminal_legacy_conflict"
                versions[row["task_id"]] = {"source_id": row["receipt_source_id"],
                                            "revision": row["receipt_revision"], "error_code": error_code}
        return versions

    def read(self, task_id):
        row = self.rows((task_id,)).get(task_id)
        if row is not None:
            self.authorize((row,), error_code="not_found")
        return row

    def _seed(self, task_id):
        # 目标存在即已通过所有入口的播种不变量，无需再次复制旧 JSON。
        current = self._target_rows((task_id,), full=False).get(task_id)
        if current is not None:
            self.authorize((current,), error_code="agent_terminal_scope_conflict")
            return current["scope"]
        legacy = self.authority._legacy_terminal_rows((task_id,), full=True).get(task_id)
        if legacy is not None:
            self.authorize((legacy,), error_code="agent_terminal_scope_conflict")
        # 授权读取已关闭；并发播种只允许第一次插入，绝不覆盖已有目标。
        if legacy is not None:
            with self._connection() as db:
                db.execute("BEGIN IMMEDIATE")
                try:
                    row = db.execute("SELECT scope FROM agent_task_terminals WHERE task_id=?", (task_id,)).fetchone()
                    if row is None:
                        db.execute("INSERT INTO agent_task_terminals (" + ",".join(TERMINAL_FIELDS)
                                   + ") VALUES (?,?,?,?,?,?,?)", tuple(legacy[key] for key in TERMINAL_FIELDS))
                    db.commit()
                except BaseException:
                    db.rollback()
                    raise
            return legacy["scope"]
        return self.authority.scope

    def save(self, prepared):
        allowed_scope = self._seed(prepared.task_id)
        for _ in range(3):
            with self._connection() as db:
                db.execute("BEGIN IMMEDIATE")
                try:
                    row = db.execute("SELECT * FROM agent_task_terminals WHERE task_id=?", (prepared.task_id,)).fetchone()
                    if row is not None and row["scope"] != allowed_scope:
                        # 仅重试持久化授权，不在目标写锁下打开意图库。
                        db.rollback()
                    else:
                        result = self.authority._write_terminal_row(db, row, prepared)
                        db.commit()
                        return result
                except BaseException:
                    db.rollback()
                    raise
            self.authorize((row,), error_code="agent_terminal_scope_conflict")
            allowed_scope = row["scope"]
        raise sqlite3.OperationalError("terminal scope changed during bounded persistence retry")
