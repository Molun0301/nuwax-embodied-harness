"""智能体调用的持久记录；不负责恢复或重新执行机器人任务。"""

from __future__ import annotations

from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sqlite3
from typing import Any, Dict, Optional, Tuple

from core.errors import GatewayError


def _json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)


class AgentCallStore:
    """以请求标识原子占位，同键重试永不重复提交设备动作。"""

    def __init__(self, path: Path, *, scope: str, gateway_instance_id: str) -> None:
        self.path = Path(path)
        self.scope = scope
        self.gateway_instance_id = gateway_instance_id
        self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        fd = os.open(str(self.path), os.O_CREAT | os.O_RDWR, 0o600)
        os.close(fd)
        self.path.chmod(0o600)
        with self._connection() as connection:
            connection.executescript("""
                CREATE TABLE IF NOT EXISTS agent_calls (
                    request_id TEXT PRIMARY KEY, scope TEXT NOT NULL,
                    gateway_instance_id TEXT NOT NULL, tool_name TEXT NOT NULL,
                    capability_name TEXT NOT NULL, arguments_json TEXT NOT NULL,
                    fingerprint TEXT NOT NULL, delivery_state TEXT NOT NULL,
                    response_json TEXT, error_json TEXT, task_id TEXT,
                    created_at TEXT NOT NULL, updated_at TEXT NOT NULL
                );
                CREATE INDEX IF NOT EXISTS agent_calls_task ON agent_calls(scope, task_id);
                CREATE TABLE IF NOT EXISTS agent_task_terminals (
                    task_id TEXT PRIMARY KEY, scope TEXT NOT NULL,
                    gateway_instance_id TEXT NOT NULL, snapshot_json TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );
            """)

    @contextmanager
    def _connection(self):
        # 每次操作独立连接并及时关闭，数据库事务绝不跨越机器人调用。
        connection = sqlite3.connect(str(self.path), timeout=5.0, isolation_level=None)
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA synchronous=FULL")
        try:
            yield connection
        finally:
            connection.close()

    def begin(self, request_id: str, tool_name: str, capability_name: str,
              arguments: Dict[str, Any]) -> Tuple[Dict[str, Any], bool]:
        arguments_json = _json(arguments)
        fingerprint = hashlib.sha256(_json([tool_name, arguments]).encode("utf-8")).hexdigest()
        now = datetime.now(timezone.utc).isoformat()
        with self._connection() as connection:
            connection.execute("BEGIN IMMEDIATE")
            try:
                row = connection.execute("SELECT * FROM agent_calls WHERE request_id=?", (request_id,)).fetchone()
                if row is not None:
                    if row["scope"] != self.scope or row["fingerprint"] != fingerprint:
                        raise GatewayError("请求标识已用于其他参数或机器人，请核对原调用。",
                                           error_code="agent_request_conflict", http_status=409)
                    connection.commit()
                    return self._decode(row), False
                connection.execute(
                    "INSERT INTO agent_calls VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)",
                    (request_id, self.scope, self.gateway_instance_id, tool_name, capability_name,
                     arguments_json, fingerprint, "pending", None, None, None, now, now),
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
            row = connection.execute("SELECT * FROM agent_calls WHERE request_id=? AND scope=?",
                                     (request_id, self.scope)).fetchone()
        if row is None:
            raise GatewayError("当前机器人没有该调用记录。", error_code="not_found", http_status=404)
        return self._decode(row)

    def find_by_task(self, task_id: str) -> Optional[Dict[str, Any]]:
        with self._connection() as connection:
            row = connection.execute(
                "SELECT * FROM agent_calls WHERE scope=? AND task_id=? ORDER BY created_at LIMIT 1",
                (self.scope, task_id),
            ).fetchone()
        return self._decode(row) if row else None

    def save_terminal(self, task_id: str, snapshot: Dict[str, Any]) -> None:
        encoded = _json(snapshot)
        with self._connection() as connection:
            connection.execute(
                "INSERT OR REPLACE INTO agent_task_terminals VALUES (?,?,?,?,?)",
                (task_id, self.scope, self.gateway_instance_id, encoded, datetime.now(timezone.utc).isoformat()),
            )

    def get_terminal(self, task_id: str) -> Dict[str, Any]:
        with self._connection() as connection:
            row = connection.execute(
                "SELECT * FROM agent_task_terminals WHERE scope=? AND task_id=?", (self.scope, task_id)
            ).fetchone()
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
