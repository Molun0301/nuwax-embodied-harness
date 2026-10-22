"""独立终态回执文件；作用域授权始终由调用意图库提供，只恢复记录不重放动作。"""
from __future__ import annotations

import os
import sqlite3
from contextlib import contextmanager
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
    """由调用库启动锁串行初始化配对；未提交的空文件可继续完成。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    fd = os.open(str(path), os.O_CREAT | os.O_RDWR, 0o600)
    os.close(fd)
    db = sqlite3.connect(str(path), timeout=5.0, isolation_level=None)
    try:
        db.execute("PRAGMA synchronous=FULL")
        db.execute("BEGIN IMMEDIATE")
        tables = {row[0] for row in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
        if tables:
            if "terminal_store_identity" not in tables:
                raise pair_conflict()
            row = db.execute("SELECT ledger_id,pair_id FROM terminal_store_identity WHERE singleton=1").fetchone()
            if row != (ledger_id, pair_id):
                raise pair_conflict()
            db.execute("SELECT task_id FROM agent_task_terminals LIMIT 0")
        else:
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
    path.chmod(0o600)
    sync_directory(path.parent)


class AgentTerminalStore:
    """唯一终态存储；作用域授权由调用库提供，版本写入在本库事务完成。"""

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
        return self._target_rows(ids, full=full)

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

    def import_legacy_rows(self, source):
        """启动时逐批搬运原列；已存在的较新目标保留，冲突保留原库并报错。"""
        if source.execute("SELECT 1 FROM sqlite_master WHERE name='agent_task_terminals' AND type='table'").fetchone() is None:
            return
        cursor = source.execute("SELECT " + ",".join(TERMINAL_FIELDS) + " FROM agent_task_terminals ORDER BY task_id")
        with self._connection() as target:
            target.execute("BEGIN IMMEDIATE")
            try:
                while rows := cursor.fetchmany(64):
                    for row in rows:
                        current = target.execute("SELECT * FROM agent_task_terminals WHERE task_id=?", (row["task_id"],)).fetchone()
                        if current is not None:
                            if current["scope"] != row["scope"]:
                                raise GatewayError("导入的历史回执作用域冲突。", error_code="agent_terminal_scope_conflict", http_status=409)
                            if current["receipt_source_id"] != row["receipt_source_id"]:
                                raise GatewayError("导入的历史回执源身份冲突。", error_code="agent_terminal_source_conflict", http_status=409)
                            revision, previous = row["receipt_revision"], current["receipt_revision"]
                            if (revision is None) != (previous is None):
                                raise GatewayError("导入的历史回执版本冲突。", error_code="agent_terminal_legacy_conflict", http_status=409)
                            if revision == previous:
                                if current["snapshot_json"] != row["snapshot_json"]:
                                    raise GatewayError("同一历史回执版本内容冲突。", error_code="agent_terminal_revision_conflict", http_status=409)
                                continue
                            if previous > revision:
                                continue
                        target.execute("INSERT OR REPLACE INTO agent_task_terminals (" + ",".join(TERMINAL_FIELDS)
                                       + ") VALUES (?,?,?,?,?,?,?)", tuple(row[key] for key in TERMINAL_FIELDS))
                target.commit()
            except BaseException:
                target.rollback()
                raise

    def save(self, prepared):
        current = self._target_rows((prepared.task_id,), full=False).get(prepared.task_id)
        if current is not None:
            self.authorize((current,), error_code="agent_terminal_scope_conflict")
        allowed_scope = current["scope"] if current is not None else self.authority.scope
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
