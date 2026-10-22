"""智能体调用的持久记录；不负责恢复或重新执行机器人任务。"""

from __future__ import annotations

import json
import os
import sqlite3
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
                CREATE TABLE IF NOT EXISTS agent_terminal_layout (
                    singleton INTEGER PRIMARY KEY CHECK(singleton=1), ledger_id TEXT NOT NULL,
                    origin TEXT NOT NULL, layout_state TEXT NOT NULL,
                    terminal_path TEXT NOT NULL, pair_id TEXT, import_version INTEGER NOT NULL DEFAULT 0
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

    def __init__(self, path: Path, *, gateway_instance_id: str,
                 robot_identity: Dict[str, Any], artifact_dir: Path) -> None:
        scope_identity = self._identity_json(robot_identity, artifact_dir)
        self.path = Path(path).expanduser().resolve()
        self.gateway_instance_id = gateway_instance_id
        self._terminal_preparation_token = object()
        self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        fd = os.open(str(self.path), os.O_CREAT | os.O_RDWR, 0o600)
        os.close(fd)
        self.path.chmod(0o600)
        with self._connection() as connection:
            connection.execute("BEGIN IMMEDIATE")
            try:
                tables = {row[0] for row in connection.execute("SELECT name FROM sqlite_master WHERE type='table'")}
                if tables and not tables.intersection({'agent_calls', 'agent_task_terminals', 'agent_terminal_layout'}):
                    raise pair_conflict()
                for statement in _SCHEMA.split(";"):
                    if statement.strip():
                        connection.execute(statement)
                if 'agent_task_terminals' in tables:
                    columns = {row["name"] for row in connection.execute("PRAGMA table_info(agent_task_terminals)")}
                    for name, sql_type in (("receipt_source_id", "TEXT"), ("receipt_revision", "INTEGER")):
                        if name not in columns:
                            connection.execute(f"ALTER TABLE agent_task_terminals ADD COLUMN {name} {sql_type}")
                columns = {row["name"] for row in connection.execute("PRAGMA table_info(agent_terminal_layout)")}
                if "import_version" not in columns:
                    connection.execute("ALTER TABLE agent_terminal_layout ADD COLUMN import_version INTEGER NOT NULL DEFAULT 0")
                layout = connection.execute("SELECT * FROM agent_terminal_layout WHERE singleton=1").fetchone()
                if layout is None:
                    connection.execute("INSERT INTO agent_terminal_layout "
                        "(singleton,ledger_id,origin,layout_state,terminal_path,pair_id) VALUES (1,?,?,?,?,?)",
                        (uuid4().hex, "legacy" if tables else "fresh", "split",
                         str(self.path.with_name(self.path.stem + ".terminals.sqlite3")), uuid4().hex))
                elif layout["layout_state"] == "combined" and Path(layout["terminal_path"]).resolve() == self.path:
                    # 旧布局只在本次向前导入边界解释，运行期没有合库后端。
                    connection.execute("UPDATE agent_terminal_layout SET terminal_path=?,pair_id=? WHERE singleton=1",
                        (str(self.path.with_name(self.path.stem + ".terminals.sqlite3")), uuid4().hex))
                self.scope = self._resolve_owner(connection, scope_identity)
                connection.commit()
            except BaseException:
                connection.rollback()
                raise
        sync_directory(self.path.parent)
        self._prepare_terminal_storage()

    def _prepare_terminal_storage(self):
        """在接收动作前完成一次导入；目标提交后才记录完成，重启可原样重试。"""
        with self._connection() as connection:
            connection.execute("BEGIN IMMEDIATE")
            try:
                layout = dict(connection.execute("SELECT * FROM agent_terminal_layout WHERE singleton=1").fetchone())
                target = Path(layout["terminal_path"]).resolve()
                if (target == self.path or (target.exists() and os.path.samefile(target, self.path))
                        or not layout["pair_id"] or layout["layout_state"] not in {'split', 'combined', 'pending', 'active'}
                        or layout["import_version"] not in (0, 1)):
                    raise pair_conflict()
                self._terminal_store = AgentTerminalStore(self, layout)
                if layout["import_version"] == 1 or layout["layout_state"] == "active":
                    # 已服务的分库丢失必须报错，不能用旧表造一个空替代库。
                    with self._terminal_store._connection() as target_connection:
                        target_connection.execute("SELECT task_id FROM agent_task_terminals LIMIT 0")
                else:
                    initialize_terminal_file(target, layout["ledger_id"], layout["pair_id"])
                if layout["import_version"] == 0:
                    self._terminal_store.import_legacy_rows(connection)
                    for operation in ("insert", "update", "delete"):
                        connection.execute("DROP TRIGGER IF EXISTS agent_terminal_legacy_seal_" + operation)
                    connection.execute("UPDATE agent_terminal_layout SET layout_state='split',import_version=1 WHERE singleton=1")
                connection.commit()
            except BaseException:
                connection.rollback()
                raise

    @contextmanager
    def _read_connection(self):
        connection = sqlite3.connect(self.path.as_uri() + "?mode=ro", uri=True, timeout=5.0, isolation_level=None)
        connection.row_factory = sqlite3.Row
        try:
            yield connection
        finally:
            connection.close()


    @classmethod
    def terminal_storage_status(cls, path):
        """只读配对及导入版本，不创建文件或执行迁移。"""
        db = sqlite3.connect(Path(path).expanduser().resolve().as_uri() + "?mode=ro", uri=True,
                             timeout=5.0, isolation_level=None)
        db.row_factory = sqlite3.Row
        try:
            row = db.execute("SELECT * FROM agent_terminal_layout WHERE singleton=1").fetchone()
            if row is None:
                raise pair_conflict()
            result = dict(row)
            result.pop("singleton", None)
            return result
        finally:
            db.close()




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
                     gateway_instance_id: str):
        """按精确机器人身份选择已持久分配的作用域。"""
        return cls(path, gateway_instance_id=gateway_instance_id,
                   robot_identity=robot_identity, artifact_dir=artifact_dir)

    @staticmethod
    def _resolve_owner(connection, identity_json: str) -> str:
        row = connection.execute("SELECT scope_id FROM agent_scope_owners WHERE identity_json=?",
                                 (identity_json,)).fetchone()
        if row is not None:
            return row["scope_id"]
        has_legacy_terminals = connection.execute(
            "SELECT 1 FROM sqlite_master WHERE name='agent_task_terminals' AND type='table'").fetchone() is not None
        for _ in range(8):
            scope = "scope_" + uuid4().hex
            occupied = connection.execute(
                "SELECT scope_id FROM agent_scope_owners WHERE scope_id=? UNION ALL "
                "SELECT scope FROM agent_scope_bindings WHERE scope=? UNION ALL "
                "SELECT scope FROM agent_calls WHERE scope=? LIMIT 1",
                (scope, scope, scope)).fetchone()
            if occupied is None and has_legacy_terminals:
                occupied = connection.execute("SELECT scope FROM agent_task_terminals WHERE scope=? LIMIT 1", (scope,)).fetchone()
            if occupied is not None:
                continue
            connection.execute("INSERT INTO agent_scope_owners(scope_id,identity_json) VALUES (?,?)",
                               (scope, identity_json))
            connection.execute("INSERT INTO agent_scope_bindings(scope,owner_scope) VALUES (?,?)", (scope, scope))
            return scope
        raise GatewayError("无法分配独立调用作用域；原调用记录保持不变。",
                           error_code="agent_scope_identity_conflict", http_status=409)


    def _assert_scope(self, connection, scope: str, *, error_code: str, message: str) -> None:
        if scope == self.scope:
            return
        row = connection.execute("SELECT owner_scope FROM agent_scope_bindings WHERE scope=?", (scope,)).fetchone()
        if row is not None and row["owner_scope"] == self.scope:
            return
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
                "scope IN (SELECT scope FROM agent_scope_bindings WHERE owner_scope=?)) "
                "ORDER BY created_at LIMIT 1",
                (task_id, self.scope, self.scope),
            ).fetchone()
        return self._decode(row) if row else None

    def find_task_call_identity(self, task_id: str) -> Optional[Dict[str, str]]:
        """查询任务回执需要的调用身份，不加载历史参数和结果。"""
        with self._connection() as connection:
            row = connection.execute(
                "SELECT request_id,tool_name FROM agent_calls WHERE task_id=? AND (scope=? OR "
                "scope IN (SELECT scope FROM agent_scope_bindings WHERE owner_scope=?)) "
                "ORDER BY created_at LIMIT 1",
                (task_id, self.scope, self.scope),
            ).fetchone()
        return dict(row) if row else None

    def terminal_versions(self, task_ids):
        """有界查询身份/版本，恢复扫描不读取或解码历史 JSON。"""
        ids = tuple(task_ids)
        if len(ids) > 64:
            raise ValueError("回执版本查询每批最多 64 个任务。")
        if not ids:
            return {}
        return self._terminal_store.versions(ids)

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
        return self._terminal_store.save(prepared)

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
            row = self._terminal_store.read(task_id)
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
