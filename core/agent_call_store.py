"""智能体调用的持久记录；不负责恢复或重新执行机器人任务。"""

from __future__ import annotations

import json
import os
import sqlite3
import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional, Tuple
from uuid import uuid4

from core.agent_terminal_store import AgentTerminalStore, initialize_terminal_file, pair_conflict, sync_directory
from core.errors import GatewayError

_SCHEMA = """
                CREATE TABLE IF NOT EXISTS agent_calls (
                    request_id TEXT PRIMARY KEY, scope TEXT NOT NULL,
                    gateway_instance_id TEXT NOT NULL, tool_name TEXT NOT NULL,
                    capability_name TEXT NOT NULL, arguments_json TEXT NOT NULL,
                    fingerprint TEXT NOT NULL, delivery_state TEXT NOT NULL,
                    response_json TEXT, error_json TEXT, task_id TEXT,
                    created_at TEXT NOT NULL, updated_at TEXT NOT NULL
                );
                CREATE INDEX IF NOT EXISTS agent_calls_task ON agent_calls(scope, task_id);
                CREATE INDEX IF NOT EXISTS agent_calls_by_task ON agent_calls(task_id, created_at);
                CREATE TABLE IF NOT EXISTS agent_scope_owners (
                    scope_id TEXT PRIMARY KEY, identity_json TEXT NOT NULL UNIQUE
                );
                CREATE TABLE IF NOT EXISTS agent_scope_bindings (
                    scope TEXT PRIMARY KEY, owner_scope TEXT NOT NULL
                );
                CREATE INDEX IF NOT EXISTS agent_scope_bindings_owner ON agent_scope_bindings(owner_scope);
                CREATE TABLE IF NOT EXISTS agent_task_terminals (
                    task_id TEXT PRIMARY KEY, scope TEXT NOT NULL,
                    gateway_instance_id TEXT NOT NULL, snapshot_json TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS agent_scope_tokens (scope TEXT PRIMARY KEY);
                CREATE TABLE IF NOT EXISTS agent_terminal_layout (
                    singleton INTEGER PRIMARY KEY CHECK(singleton=1), ledger_id TEXT NOT NULL,
                    origin TEXT NOT NULL, layout_state TEXT NOT NULL,
                    terminal_path TEXT NOT NULL, pair_id TEXT
                );
"""

def _json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)


@dataclass(frozen=True)
class PreparedTerminalReceipt:
    task_id: str
    source_id: str
    revision: int
    encoded: str
    encoded_bytes: int
    _owner_token: Any = field(default=None, init=False, repr=False, compare=False)


class AgentCallStore:
    """以请求标识原子占位，同键重试永不重复提交设备动作。"""

    def __init__(self, path: Path, *, gateway_instance_id: str, scope: Optional[str] = None,
                 robot_identity: Optional[Dict[str, Any]] = None, artifact_dir: Optional[Path] = None,
                 fresh_layout: str = "split") -> None:
        if fresh_layout not in ("combined", "split"):
            raise ValueError("新账本布局只允许 combined 或 split。")
        if robot_identity is not None:
            if artifact_dir is None or scope is not None:
                raise ValueError("身份作用域需要 artifact_dir，且不能同时传入原始 scope。")
            scope_identity = self._identity_json(robot_identity, artifact_dir)
        else:
            if scope is None:
                raise ValueError("必须指定作用域或完整机器人身份。")
            scope_identity = None
        self.path = Path(path).expanduser().resolve()
        self.scope = scope or ""
        self._registered_scope = scope_identity is not None
        self.gateway_instance_id = gateway_instance_id
        self._terminal_preparation_token = object()
        self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        try:
            fd = os.open(str(self.path), os.O_CREAT | os.O_EXCL | os.O_RDWR, 0o600)
            fresh = True
        except FileExistsError:
            fd = os.open(str(self.path), os.O_RDWR)
            fresh = False
        os.close(fd)
        self.path.chmod(0o600)
        if not fresh:
            # 不把并发初始化中的空文件或未知现有文件猜作全新账本。
            deadline = time.monotonic() + 5.0
            while True:
                with self._read_connection() as db:
                    known = db.execute("SELECT name FROM sqlite_master WHERE name IN "
                                       "('agent_calls','agent_task_terminals','agent_terminal_layout')").fetchone()
                if known:
                    break
                if time.monotonic() >= deadline:
                    raise GatewayError("调用账本尚未完成初始化；请核对原文件，不能猜测其身份。",
                                       error_code="agent_terminal_cutover_required", http_status=409)
                time.sleep(0.025)
        with self._connection() as connection:
            connection.execute("BEGIN IMMEDIATE")
            try:
                for statement in _SCHEMA.split(";"):
                    if statement.strip():
                        connection.execute(statement)
                columns = {row["name"] for row in connection.execute("PRAGMA table_info(agent_task_terminals)")}
                for name, sql_type in (("receipt_source_id", "TEXT"), ("receipt_revision", "INTEGER")):
                    if name not in columns:
                        connection.execute(f"ALTER TABLE agent_task_terminals ADD COLUMN {name} {sql_type}")
                layout = connection.execute("SELECT * FROM agent_terminal_layout WHERE singleton=1").fetchone()
                if layout is None:
                    ledger_id = uuid4().hex
                    split_new = fresh and fresh_layout == "split"
                    target = str(self.path.with_name(self.path.stem + ".terminals.sqlite3")) if split_new else str(self.path)
                    connection.execute("INSERT INTO agent_terminal_layout "
                                       "(singleton,ledger_id,origin,layout_state,terminal_path,pair_id) VALUES (1,?,?,?,?,?)",
                                       (ledger_id, "fresh" if fresh else "legacy", "pending" if split_new else "combined",
                                        target, uuid4().hex if split_new else None))
                # 原始 token 的保留不是身份绑定；只防止后续分配与它碰撞。
                if scope_identity is None and self.scope:
                    connection.execute("INSERT OR IGNORE INTO agent_scope_tokens(scope) VALUES (?)", (self.scope,))
                if scope_identity is not None:
                    self.scope = self._resolve_owner(connection, scope_identity)
                connection.commit()
            except BaseException:
                connection.rollback()
                raise
        if fresh:
            sync_directory(self.path.parent)
        layout = self.terminal_storage_status(self.path)
        if layout["origin"] == "fresh" and layout["layout_state"] == "pending":
            self.activate_terminal_split(self.path, expected_ledger_id=layout["ledger_id"],
                                         terminal_path=Path(layout["terminal_path"]), writers_quiesced=True)

    @contextmanager
    def _read_connection(self):
        connection = sqlite3.connect(self.path.as_uri() + "?mode=ro", uri=True, timeout=5.0, isolation_level=None)
        connection.row_factory = sqlite3.Row
        try:
            yield connection
        finally:
            connection.close()

    @staticmethod
    def _seal_sql(operation):
        name = "agent_terminal_legacy_seal_" + operation.lower()
        return name, ("CREATE TRIGGER " + name + " BEFORE " + operation + " ON agent_task_terminals "
                      "BEGIN SELECT RAISE(ABORT,'legacy terminal store sealed'); END")

    @classmethod
    def terminal_storage_status(cls, path):
        """只读布局状态；不创建账本、绑定身份、修复或迁移。"""
        db = sqlite3.connect(Path(path).expanduser().resolve().as_uri() + "?mode=ro", uri=True,
                             timeout=5.0, isolation_level=None)
        db.row_factory = sqlite3.Row
        try:
            exists = db.execute("SELECT name FROM sqlite_master WHERE name='agent_terminal_layout'").fetchone()
            row = db.execute("SELECT * FROM agent_terminal_layout WHERE singleton=1").fetchone() if exists else None
            if row is None:
                raise GatewayError("旧账本尚未由当前版本初始化布局标识；请由维护方核对原文件。",
                                   error_code="agent_terminal_cutover_required", http_status=409)
            result = dict(row)
            result.pop("singleton", None)
            expected = dict(cls._seal_sql(operation) for operation in ("INSERT", "UPDATE", "DELETE"))
            seals = {item["name"]: item["sql"] for item in db.execute(
                "SELECT name,sql FROM sqlite_master WHERE type='trigger' AND name IN (?,?,?)", tuple(expected))}
            result["legacy_sealed"] = seals == expected
            return result
        finally:
            db.close()

    @classmethod
    def activate_terminal_split(cls, path, *, expected_ledger_id, terminal_path, writers_quiesced):
        """可信维护入口。调用方必须获授权并确认所有旧写入者已排空；不会停止机器人。"""
        if writers_quiesced is not True:
            raise GatewayError("必须由获授权维护方确认旧账本写入者及在途调用已排空。",
                               error_code="agent_terminal_cutover_required", http_status=409)
        path, target = Path(path).expanduser().resolve(), Path(terminal_path).expanduser().resolve()
        if path == target or (target.exists() and os.path.samefile(path, target)):
            raise pair_conflict()
        # 仅既有意图库；不调用构造器、不分配作用域、更不借迁移认领旧任务。
        db = sqlite3.connect(path.as_uri() + "?mode=rw", uri=True, timeout=5.0, isolation_level=None)
        db.row_factory = sqlite3.Row
        try:
            db.execute("PRAGMA synchronous=FULL")
            db.execute("BEGIN IMMEDIATE")
            row = db.execute("SELECT * FROM agent_terminal_layout WHERE singleton=1").fetchone()
            if row is None or row["ledger_id"] != expected_ledger_id:
                raise pair_conflict()
            if row["layout_state"] == "combined":
                pair_id = uuid4().hex
                db.execute("UPDATE agent_terminal_layout SET layout_state='pending',terminal_path=?,pair_id=? "
                           "WHERE singleton=1", (str(target), pair_id))
            elif row["layout_state"] in {"pending", "active"} and row["terminal_path"] == str(target):
                pair_id = row["pair_id"]
            else:
                raise pair_conflict()
            db.commit()
        except BaseException:
            db.rollback()
            raise
        finally:
            db.close()
        sync_directory(path.parent)
        if row["layout_state"] == "active":
            # 已激活文件丢失不能通过再次调用维护入口创建空替代库。
            with AgentTerminalStore(None, dict(row))._connection():
                pass
        else:
            initialize_terminal_file(target, expected_ledger_id, pair_id)
        if os.path.samefile(path, target):
            raise pair_conflict()
        db = sqlite3.connect(path.as_uri() + "?mode=rw", uri=True, timeout=5.0, isolation_level=None)
        db.row_factory = sqlite3.Row
        try:
            db.execute("PRAGMA synchronous=FULL")
            db.execute("BEGIN IMMEDIATE")
            row = db.execute("SELECT * FROM agent_terminal_layout WHERE singleton=1").fetchone()
            if row is None or (row["ledger_id"], row["pair_id"], row["terminal_path"]) != (
                    expected_ledger_id, pair_id, str(target)) or row["layout_state"] not in {"pending", "active"}:
                raise pair_conflict()
            for operation in ("INSERT", "UPDATE", "DELETE"):
                name, sql = cls._seal_sql(operation)
                previous = db.execute("SELECT sql FROM sqlite_master WHERE name=?", (name,)).fetchone()
                if previous is None:
                    if row["layout_state"] == "active":
                        raise pair_conflict()
                    db.execute(sql)
                elif previous["sql"] != sql:
                    raise pair_conflict()
            db.execute("UPDATE agent_terminal_layout SET layout_state='active' WHERE singleton=1")
            db.commit()
        except BaseException:
            db.rollback()
            raise
        finally:
            db.close()
        return cls.terminal_storage_status(path)

    def _terminal_backend(self):
        layout = self.terminal_storage_status(self.path)
        if layout["layout_state"] in {"combined", "pending"}:
            return None
        if layout["layout_state"] != "active" or not layout["legacy_sealed"] or not layout["pair_id"]:
            raise pair_conflict()
        return AgentTerminalStore(self, layout)

    def _legacy_terminal_rows(self, ids, *, full):
        fields = "*" if full else "task_id,scope,receipt_source_id,receipt_revision"
        with self._read_connection() as db:
            return {row["task_id"]: dict(row) for row in db.execute(
                "SELECT " + fields + " FROM agent_task_terminals WHERE task_id IN ("
                + ",".join("?" for _ in ids) + ")", ids)}

    @staticmethod
    def _terminal_unavailable():
        return GatewayError("终态回执存储暂不可读取或写入；已知动作结果不因此改变，请核对原任务。",
                            error_code="agent_terminal_store_unavailable", http_status=503)

    @staticmethod
    def _identity_json(robot_identity: Dict[str, Any], artifact_dir: Path) -> str:
        # 与旧作用域的原始输入完全一致；不重新计算或验证任何历史派生标识。
        directory = str(Path(artifact_dir).expanduser().resolve())
        return json.dumps([robot_identity, directory], sort_keys=True)

    @classmethod
    def for_identity(cls, path: Path, *, robot_identity: Dict[str, Any], artifact_dir: Path,
                     gateway_instance_id: str, fresh_layout: str = "split"):
        """按精确身份选择持久分配的作用域；旧作用域只能显式核对绑定。"""
        return cls(path, gateway_instance_id=gateway_instance_id,
                   robot_identity=robot_identity, artifact_dir=artifact_dir, fresh_layout=fresh_layout)

    @staticmethod
    def _resolve_owner(connection, identity_json: str) -> str:
        row = connection.execute("SELECT scope_id FROM agent_scope_owners WHERE identity_json=?",
                                 (identity_json,)).fetchone()
        if row is not None:
            return row["scope_id"]
        for _ in range(8):
            scope = "scope_" + uuid4().hex
            occupied = connection.execute(
                "SELECT scope_id FROM agent_scope_owners WHERE scope_id=? UNION ALL "
                "SELECT scope FROM agent_scope_bindings WHERE scope=? UNION ALL "
                "SELECT scope FROM agent_calls WHERE scope=? UNION ALL "
                "SELECT scope FROM agent_task_terminals WHERE scope=? UNION ALL "
                "SELECT scope FROM agent_scope_tokens WHERE scope=? LIMIT 1",
                (scope, scope, scope, scope, scope)).fetchone()
            if occupied is not None:
                continue
            connection.execute("INSERT INTO agent_scope_owners(scope_id,identity_json) VALUES (?,?)",
                               (scope, identity_json))
            connection.execute("INSERT INTO agent_scope_bindings(scope,owner_scope) VALUES (?,?)", (scope, scope))
            return scope
        raise GatewayError("无法分配独立调用作用域；原调用记录保持不变。",
                           error_code="agent_scope_identity_conflict", http_status=409)

    @classmethod
    def list_legacy_scopes(cls, path: Path) -> Tuple[str, ...]:
        """只读维护清单；仅列出尚未绑定的原始标识，不推断其机器人身份。"""
        connection = sqlite3.connect(Path(path).resolve().as_uri() + "?mode=ro", uri=True)
        try:
            tables = {row[0] for row in connection.execute("SELECT name FROM sqlite_master WHERE type='table'")}
            sources = ["SELECT scope FROM " + name for name in ("agent_calls", "agent_task_terminals") if name in tables]
            if not sources:
                return ()
            sql = "SELECT scope FROM (" + " UNION ".join(sources) + ")"
            if "agent_scope_bindings" in tables:
                sql += " WHERE scope NOT IN (SELECT scope FROM agent_scope_bindings)"
            scopes = {row[0] for row in connection.execute(sql)}
            if "agent_terminal_layout" in tables:
                layout = cls.terminal_storage_status(path)
                if layout["layout_state"] == "active":
                    if not layout["legacy_sealed"]:
                        raise pair_conflict()
                    with AgentTerminalStore(None, layout)._connection() as target:
                        target_scopes = {row[0] for row in target.execute("SELECT DISTINCT scope FROM agent_task_terminals")}
                    bindings = {row[0] for row in connection.execute("SELECT scope FROM agent_scope_bindings")}
                    scopes.update(target_scopes - bindings)
            return tuple(sorted(scopes))
        finally:
            connection.close()

    @classmethod
    def bind_verified_legacy_scope(cls, path: Path, *, legacy_scope: str,
                                   robot_identity: Dict[str, Any], artifact_dir: Path) -> str:
        """可信维护入口：调用方须已核实指定身份；单凭历史标识不构成授权。"""
        store = cls(path, scope="", gateway_instance_id="scope-binding-maintenance")
        identity_json = cls._identity_json(robot_identity, artifact_dir)
        target_exists = False
        backend = store._terminal_backend()
        if backend is not None:
            with backend._connection() as target:
                target_exists = target.execute("SELECT 1 FROM agent_task_terminals WHERE scope=? LIMIT 1",
                                               (legacy_scope,)).fetchone() is not None
        with store._connection() as connection:
            connection.execute("BEGIN IMMEDIATE")
            try:
                binding = connection.execute(
                    "SELECT b.owner_scope,o.identity_json FROM agent_scope_bindings b "
                    "JOIN agent_scope_owners o ON o.scope_id=b.owner_scope WHERE b.scope=?",
                    (legacy_scope,)).fetchone()
                if binding is not None:
                    if binding["identity_json"] != identity_json:
                        raise GatewayError("历史作用域已绑定另一身份；必须核对原记录，不能重新认领。",
                                           error_code="agent_scope_binding_conflict", http_status=409)
                    connection.commit()
                    return binding["owner_scope"]
                exists = connection.execute(
                    "SELECT scope FROM agent_calls WHERE scope=? UNION ALL "
                    "SELECT scope FROM agent_task_terminals WHERE scope=? LIMIT 1",
                    (legacy_scope, legacy_scope)).fetchone()
                if exists is None and not target_exists:
                    raise GatewayError("没有待绑定的历史作用域记录。", error_code="not_found", http_status=404)
                owner = cls._resolve_owner(connection, identity_json)
                connection.execute("INSERT INTO agent_scope_bindings(scope,owner_scope) VALUES (?,?)", (legacy_scope, owner))
                connection.commit()
                return owner
            except BaseException:
                connection.rollback()
                raise

    def _assert_scope(self, connection, scope: str, *, error_code: str, message: str) -> None:
        if scope == self.scope:
            return
        if self._registered_scope:
            row = connection.execute("SELECT owner_scope FROM agent_scope_bindings WHERE scope=?", (scope,)).fetchone()
            if row is not None and row["owner_scope"] == self.scope:
                return
            if row is None:
                raise GatewayError("原记录的历史作用域尚未核实绑定；原请求不能被重新认领或重放。",
                                   error_code="agent_scope_binding_required", http_status=409)
        raise GatewayError(message, error_code=error_code, http_status=404 if error_code == "not_found" else 409)

    @contextmanager
    def _connection(self):
        # 每次操作独立连接并及时关闭，数据库事务绝不跨越机器人调用。
        connection = sqlite3.connect(self.path.as_uri() + "?mode=rw", uri=True, timeout=5.0, isolation_level=None)
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA synchronous=FULL")
        try:
            yield connection
        finally:
            connection.close()

    def begin(self, request_id: str, tool_name: str, capability_name: str,
              arguments: Dict[str, Any]) -> Tuple[Dict[str, Any], bool]:
        arguments_json = _json(arguments)
        now = datetime.now(timezone.utc).isoformat()
        with self._connection() as connection:
            connection.execute("BEGIN IMMEDIATE")
            try:
                row = connection.execute("SELECT * FROM agent_calls WHERE request_id=?", (request_id,)).fetchone()
                if row is not None:
                    self._assert_scope(connection, row["scope"], error_code="agent_request_conflict",
                                       message="请求标识已用于另一机器人，请核对原调用。")
                    if row["tool_name"] != tool_name or row["arguments_json"] != arguments_json:
                        raise GatewayError("请求标识已用于其他参数或机器人，请核对原调用。",
                                           error_code="agent_request_conflict", http_status=409)
                    connection.commit()
                    return self._decode(row), False
                # fingerprint 仅为旧表结构保留；不再计算、读取比较或改写历史值。
                connection.execute(
                    "INSERT INTO agent_calls (request_id,scope,gateway_instance_id,tool_name,capability_name,"
                    "arguments_json,fingerprint,delivery_state,response_json,error_json,task_id,created_at,updated_at) "
                    "VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)",
                    (request_id, self.scope, self.gateway_instance_id, tool_name, capability_name,
                     arguments_json, "", "pending", None, None, None, now, now),
                )
                row = connection.execute("SELECT * FROM agent_calls WHERE request_id=?", (request_id,)).fetchone()
                connection.commit()
                return self._decode(row), True
            except BaseException:
                connection.rollback()
                raise

    def finish(self, request_id: str, response: Any, error: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        response_json = _json(response)
        task_id = response.get("task_id") if isinstance(response, dict) else None
        with self._connection() as connection:
            cursor = connection.execute(
                "UPDATE agent_calls SET response_json=?,error_json=?,task_id=?,delivery_state='recorded',updated_at=? "
                "WHERE request_id=? AND scope=? AND gateway_instance_id=? AND delivery_state='pending'",
                (response_json, _json(error) if error else None, task_id,
                 datetime.now(timezone.utc).isoformat(), request_id, self.scope, self.gateway_instance_id),
            )
            if cursor.rowcount != 1:
                raise GatewayError("调用记录的拥有方或状态已改变，执行结果需核对。",
                                   error_code="agent_call_outcome_unknown", http_status=409)
        return self.get(request_id)

    def mark_unknown(self, request_id: str) -> None:
        with self._connection() as connection:
            connection.execute(
                "UPDATE agent_calls SET delivery_state='unknown',updated_at=? "
                "WHERE request_id=? AND scope=? AND gateway_instance_id=? AND delivery_state='pending'",
                (datetime.now(timezone.utc).isoformat(), request_id, self.scope, self.gateway_instance_id),
            )

    def get(self, request_id: str) -> Dict[str, Any]:
        with self._connection() as connection:
            row = connection.execute("SELECT * FROM agent_calls WHERE request_id=?", (request_id,)).fetchone()
            if row is not None:
                self._assert_scope(connection, row["scope"], error_code="not_found", message="当前机器人没有该调用记录。")
        if row is None:
            raise GatewayError("当前机器人没有该调用记录。", error_code="not_found", http_status=404)
        return self._decode(row)

    def find_by_task(self, task_id: str) -> Optional[Dict[str, Any]]:
        with self._connection() as connection:
            row = connection.execute(
                "SELECT * FROM agent_calls WHERE task_id=? AND (scope=? OR "
                "(? AND scope IN (SELECT scope FROM agent_scope_bindings WHERE owner_scope=?))) "
                "ORDER BY created_at LIMIT 1",
                (task_id, self.scope, self._registered_scope, self.scope),
            ).fetchone()
        return self._decode(row) if row else None

    def find_task_call_identity(self, task_id: str) -> Optional[Dict[str, str]]:
        """查询任务回执需要的调用身份，不加载历史参数和结果。"""
        with self._connection() as connection:
            row = connection.execute(
                "SELECT request_id,tool_name FROM agent_calls WHERE task_id=? AND (scope=? OR "
                "(? AND scope IN (SELECT scope FROM agent_scope_bindings WHERE owner_scope=?))) "
                "ORDER BY created_at LIMIT 1",
                (task_id, self.scope, self._registered_scope, self.scope),
            ).fetchone()
        return dict(row) if row else None

    def terminal_versions(self, task_ids):
        """有界查询身份/版本，恢复扫描不读取或解码历史 JSON。"""
        ids = tuple(task_ids)
        if len(ids) > 64:
            raise ValueError("回执版本查询每批最多 64 个任务。")
        if not ids:
            return {}
        backend = self._terminal_backend()
        if backend is not None:
            return backend.versions(ids)
        with self._connection() as connection:
            rows = connection.execute(
                "SELECT task_id,scope,receipt_source_id,receipt_revision FROM agent_task_terminals "
                "WHERE task_id IN (" + ",".join("?" for _ in ids) + ")", ids).fetchall()
            versions = {}
            for row in rows:
                error_code = None
                try:
                    self._assert_scope(connection, row["scope"], error_code="agent_terminal_scope_conflict",
                                       message="任务历史回执属于另一作用域。")
                except GatewayError as error:
                    error_code = error.error_code
                if error_code is None and (row["receipt_source_id"] is None or row["receipt_revision"] is None):
                    error_code = "agent_terminal_legacy_conflict"
                versions[row["task_id"]] = {"source_id": row["receipt_source_id"],
                                            "revision": row["receipt_revision"], "error_code": error_code}
            return versions

    def prepare_terminal(self, task_id: str, snapshot: Dict[str, Any], *,
                      source_id: Optional[str] = None, revision: Optional[int] = None) -> PreparedTerminalReceipt:
        """同一任务源按版本单调写入；较旧快照不覆盖，旧无版本行只读保留。"""
        if (not isinstance(source_id, str) or not source_id or len(source_id) > 128
                or not isinstance(revision, int) or isinstance(revision, bool) or revision < 1):
            raise GatewayError("任务回执缺少有效的源身份与版本；不得重放动作补写。",
                               error_code="agent_terminal_version_required", http_status=409)
        if snapshot.get("task_id") != task_id:
            raise GatewayError("任务回执身份与目标不一致。",
                               error_code="agent_terminal_identity_conflict", http_status=409)
        encoded = _json(snapshot)
        prepared = PreparedTerminalReceipt(task_id, source_id, revision, encoded, len(encoded.encode("utf-8")))
        object.__setattr__(prepared, "_owner_token", self._terminal_preparation_token)
        return prepared

    def save_terminal(self, task_id: str, snapshot: Dict[str, Any], *,
                      source_id: Optional[str] = None, revision: Optional[int] = None) -> Dict[str, Any]:
        prepared = self.prepare_terminal(task_id, snapshot, source_id=source_id, revision=revision)
        try:
            return self.save_prepared_terminal(prepared)
        except (sqlite3.Error, OSError) as error:
            raise self._terminal_unavailable() from error

    def save_prepared_terminal(self, prepared: PreparedTerminalReceipt) -> Dict[str, Any]:
        # 只接受本存储边界已验证的不可变准备结果，不重新遍历 JSON。
        if type(prepared) is not PreparedTerminalReceipt or prepared._owner_token is not self._terminal_preparation_token:
            raise GatewayError("回执准备结果不属于此存储拥有者。",
                               error_code="agent_terminal_identity_conflict", http_status=409)
        backend = self._terminal_backend()
        if backend is not None:
            return backend.save(prepared)
        with self._connection() as connection:
            connection.execute("BEGIN IMMEDIATE")
            try:
                row = connection.execute("SELECT * FROM agent_task_terminals WHERE task_id=?", (prepared.task_id,)).fetchone()
                if row is not None:
                    self._assert_scope(connection, row["scope"], error_code="agent_terminal_scope_conflict",
                                       message="任务历史回执属于另一作用域。")
                result = self._write_terminal_row(connection, row, prepared)
                connection.commit()
                return result
            except BaseException:
                connection.rollback()
                raise

    def _write_terminal_row(self, connection, row, prepared):
        # 调用方已授权；此方法不打开其他库、不复制/投影/编码源内容。
        task_id, source_id, revision, encoded = prepared.task_id, prepared.source_id, prepared.revision, prepared.encoded
        now = datetime.now(timezone.utc).isoformat()
        if row is None:
            connection.execute(
                "INSERT INTO agent_task_terminals "
                "(task_id,scope,gateway_instance_id,snapshot_json,updated_at,receipt_source_id,receipt_revision) "
                "VALUES (?,?,?,?,?,?,?)", (task_id, self.scope, self.gateway_instance_id, encoded, now, source_id, revision))
            disposition, stored_revision = "inserted", revision
        else:
            if row["receipt_source_id"] is None or row["receipt_revision"] is None:
                raise GatewayError("已有旧版历史回执没有可验证的源版本，仅允许读取，不能自动认领或改写。",
                                   error_code="agent_terminal_legacy_conflict", http_status=409)
            if row["receipt_source_id"] != source_id:
                raise GatewayError("任务回执属于另一任务管理器源，不能重新标记历史拥有方。",
                                   error_code="agent_terminal_source_conflict", http_status=409)
            stored_revision = row["receipt_revision"]
            if revision < stored_revision:
                disposition = "stale"
            elif revision == stored_revision:
                if row["snapshot_json"] != encoded:
                    raise GatewayError("同一任务回执版本出现不同内容，原记录已保留。",
                                       error_code="agent_terminal_revision_conflict", http_status=409)
                disposition = "unchanged"
            else:
                connection.execute(
                    "UPDATE agent_task_terminals SET snapshot_json=?,updated_at=?,receipt_revision=? "
                    "WHERE task_id=? AND scope=? AND receipt_source_id=? AND receipt_revision<?",
                    (encoded, now, revision, task_id, row["scope"], source_id, revision))
                disposition, stored_revision = "updated", revision
        return {"disposition": disposition, "source_id": source_id, "revision": stored_revision}

    def get_terminal(self, task_id: str) -> Dict[str, Any]:
        try:
            backend = self._terminal_backend()
            if backend is not None:
                row = backend.read(task_id)
            else:
                with self._connection() as connection:
                    row = connection.execute("SELECT * FROM agent_task_terminals WHERE task_id=?", (task_id,)).fetchone()
                    if row is not None:
                        self._assert_scope(connection, row["scope"], error_code="not_found",
                                           message="当前机器人没有该任务终态记录。")
        except (sqlite3.Error, OSError) as error:
            raise self._terminal_unavailable() from error
        if row is None:
            raise GatewayError("没有已保存的任务终态记录。", error_code="not_found", http_status=404)
        return {"gateway_instance_id": row["gateway_instance_id"],
                "snapshot": json.loads(row["snapshot_json"]), "recorded_at": row["updated_at"]}

    def _decode(self, row) -> Dict[str, Any]:
        record = dict(row)
        response_json = record.pop("response_json")
        error_json = record.pop("error_json")
        record["response"] = json.loads(response_json) if response_json else None
        record["error"] = json.loads(error_json) if error_json else None
        record.pop("arguments_json", None)
        record.pop("fingerprint", None)
        # 旧实例的未完成意图只能恢复查看，不能由新实例自动认领执行。
        if record["delivery_state"] == "pending" and record["gateway_instance_id"] != self.gateway_instance_id:
            record["delivery_state"] = "unknown"
        return record
