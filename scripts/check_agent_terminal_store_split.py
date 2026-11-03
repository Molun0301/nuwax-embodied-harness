#!/usr/bin/env python3
"""终态唯一分库回归；仅使用自行创建的数据库和动作计数。

子进程退出验证重启收敛，不代表断电或介质持久性验证。
"""
from __future__ import annotations
import json
import os
import sqlite3
import subprocess
import sys
import tempfile
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core.agent_call_store import AgentCallStore
from core.agent_terminal_store import TERMINAL_FIELDS
from core.errors import GatewayError

CONNECT = sqlite3.connect
IDENTITY = {"robot_model": "offline", "robot_id": "one"}
EXIT_CODE = 71
FIELDS = TERMINAL_FIELDS

def snapshot(task="task", revision=8):
    return {"task_id": task, "status": {"state": "succeeded"}, "result": {"revision": revision}}



def store(path, gateway="owner"):
    return AgentCallStore(path, robot_identity=IDENTITY, artifact_dir=Path(path).parent / "artifacts",
                          gateway_instance_id=gateway)

def state(path):
    return AgentCallStore.terminal_storage_status(path)

def target_path(path):
    return Path(state(path)["terminal_path"])

def raw_rows(path):
    with CONNECT(str(path)) as db:
        return db.execute("SELECT " + ",".join(FIELDS) + " FROM agent_task_terminals ORDER BY task_id").fetchall()


@contextmanager
def observed_connections(intent, terminal, *, before_commit=None, after_commit=None, before_begin=None,
                         reject_intent_writes=False, timeout=None):
    """Instrument real SQLite calls without replacing transaction semantics."""
    intent, terminal = Path(intent).resolve(), Path(terminal).resolve()
    events, open_intent, writers = [], set(), set()
    guard = threading.RLock()

    class Observed(sqlite3.Connection):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            actual = Path(super().execute("PRAGMA database_list").fetchone()[2]).resolve()
            self.role = "intent" if actual == intent else "terminal" if actual == terminal else "other"
            self.sql = []
            self.closed = False
            with guard:
                if self.role == "intent":
                    open_intent.add(id(self))
            self.set_trace_callback(self.trace)

        def trace(self, sql):
            with guard:
                events.append((self.role, sql))
                self.sql.append(sql)

        def execute(self, sql, *args, **kwargs):
            normalized = sql.lstrip().upper()
            mutating = normalized.startswith(("BEGIN", "INSERT", "UPDATE", "DELETE", "CREATE", "ALTER", "DROP", "REPLACE"))
            if self.role == "intent" and reject_intent_writes and mutating:
                raise AssertionError("terminal operation attempted intent mutation: " + sql)
            if normalized.startswith("BEGIN") and self.role == "terminal":
                with guard:
                    if reject_intent_writes and open_intent:
                        raise AssertionError("intent connection held at terminal BEGIN")
                if before_begin:
                    before_begin(self)
            result = super().execute(sql, *args, **kwargs)
            if normalized.startswith("BEGIN") and self.role == "terminal":
                writers.add(id(self))
            return result

        def executescript(self, sql):
            if self.role == "intent" and reject_intent_writes:
                raise AssertionError("terminal operation attempted intent schema initialization")
            return super().executescript(sql)

        def commit(self):
            if before_commit:
                before_commit(self)
            result = super().commit()
            writers.discard(id(self))
            if after_commit:
                after_commit(self)
            self.sql.clear()
            return result

        def rollback(self):
            result = super().rollback()
            writers.discard(id(self))
            return result

        def close(self):
            try:
                return super().close()
            finally:
                self.closed = True
                with guard:
                    open_intent.discard(id(self))
                    writers.discard(id(self))

    def connect(*args, **kwargs):
        kwargs["factory"] = Observed
        if timeout is not None:
            kwargs["timeout"] = timeout
        return CONNECT(*args, **kwargs)

    with patch.object(sqlite3, "connect", connect):
        yield {"events": events, "open_intent": open_intent, "writers": writers}


class SplitChecks(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="agent-forward-import-", dir=os.environ.get("NEH_TERMINAL_SPLIT_TMP"))
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.path = self.root / "agent_calls.sqlite3"

    def save(self, service, revision, task="task", *, source="source", value=None):
        return service.save_terminal(task, snapshot(task, revision if value is None else value), source_id=source, revision=revision)

    def error(self, code, callback):
        with self.assertRaises(GatewayError) as caught:
            callback()
        self.assertEqual(caught.exception.error_code, code)

    def test_fresh_has_only_split_terminal_table(self):
        service = store(self.path)
        self.assertEqual(state(self.path)["layout_state"], "split")
        self.assertEqual(state(self.path)["layout_state"], "split")
        with CONNECT(self.path) as db:
            self.assertIsNone(db.execute("SELECT name FROM sqlite_master WHERE name='agent_task_terminals'").fetchone())
        self.assertEqual(self.save(service, 1)["disposition"], "inserted")
        self.assertEqual(raw_rows(target_path(self.path))[0][-1], 1)
        with CONNECT(target_path(self.path)) as db:
            self.assertEqual({row[0] for row in db.execute("SELECT name FROM sqlite_master WHERE type='table'")},
                             {"terminal_store_identity", "agent_task_terminals"})
        self.assertEqual(self.path.stat().st_mode & 0o777, 0o600)
        self.assertEqual(target_path(self.path).stat().st_mode & 0o777, 0o600)








    def test_missing_completed_target_is_not_recreated(self):
        service = store(self.path)
        self.save(service, 8)
        target = target_path(self.path)
        target.unlink()
        with self.assertRaises((sqlite3.Error, OSError, GatewayError)):
            store(self.path)
        self.assertFalse(target.exists())

    def test_foreign_target_is_not_claimed(self):
        service = store(self.path)
        self.save(service, 8)
        target = target_path(self.path)
        with CONNECT(target) as db:
            db.execute("UPDATE terminal_store_identity SET pair_id='different'")
        before = raw_rows(target)
        self.error("agent_terminal_pair_conflict", lambda: store(self.path))
        self.assertEqual(raw_rows(target), before)


    def test_concurrent_constructors_share_one_pair_and_owner(self):
        with ThreadPoolExecutor(max_workers=6) as pool:
            stores = list(pool.map(lambda index: store(self.path, str(index)), range(6)))
        self.assertEqual(len({item.scope for item in stores}), 1)
        self.assertEqual(len({item._terminal_store.pair_id for item in stores}), 1)
        self.assertEqual(state(self.path)["layout_state"], "split")

    def test_monotonic_versions_and_original_scope_survive_concurrent_writes(self):
        service = store(self.path)
        self.save(service, 8)
        original = raw_rows(target_path(self.path))[0]
        stores = [store(self.path, str(index)) for index in range(4)]
        with ThreadPoolExecutor(max_workers=4) as pool:
            results = list(pool.map(lambda pair: self.save(*pair), zip(stores, (7, 9, 9, 8))))
        self.assertTrue(all(item["revision"] in (8, 9) for item in results))
        self.assertEqual(stores[0].terminal_versions(["task"])["task"]["revision"], 9)
        self.assertEqual(raw_rows(target_path(self.path))[0][:3], original[:3])
        self.error("agent_terminal_revision_conflict", lambda: self.save(stores[0], 9, value="different"))
        self.error("agent_terminal_source_conflict", lambda: self.save(stores[0], 10, source="another"))

    def test_versions_do_not_decode_json_and_prepared_receipts_remain_owned(self):
        first, second = store(self.path), store(self.path, "other")
        prepared = first.prepare_terminal("task", snapshot(), source_id="source", revision=8)
        self.error("agent_terminal_identity_conflict", lambda: second.save_prepared_terminal(prepared))
        first.save_prepared_terminal(prepared)
        with patch.object(json, "loads", side_effect=AssertionError("版本查询不可解码结果正文")):
            self.assertEqual(first.terminal_versions(["task"])["task"]["revision"], 8)
        self.assertEqual(first.terminal_versions([]), {})
        with self.assertRaises(ValueError):
            first.terminal_versions([str(index) for index in range(65)])

    def test_terminal_writer_keeps_intent_transactions_independent(self):
        for mode in ("delete", "wal"):
            with self.subTest(mode=mode):
                folder = self.root / mode
                folder.mkdir()
                path = folder / "agent_calls.sqlite3"
                service = store(path)
                target = target_path(path)
                for file in (path, target):
                    with CONNECT(file) as db:
                        self.assertEqual(db.execute("PRAGMA journal_mode=" + mode).fetchone()[0], mode)
                service.begin("begun", "tool", "cap", {})
                entered, release = threading.Event(), threading.Event()
                def blocked(db):
                    if db.role == "terminal" and any("INSERT INTO agent_task_terminals" in sql for sql in db.sql):
                        entered.set()
                        if not release.wait(3):
                            raise AssertionError("测试未释放终态事务")
                with observed_connections(path, target, before_commit=blocked, timeout=.05), ThreadPoolExecutor(max_workers=1) as pool:
                    pending = pool.submit(self.save, service, 1)
                    try:
                        self.assertTrue(entered.wait(2))
                        self.assertEqual(service.finish("begun", {})["delivery_state"], "recorded")
                        self.assertTrue(service.begin("new", "tool", "cap", {})[1])
                        service.finish("new", {"task_id": "task"})
                    finally:
                        release.set()
                    self.assertEqual(pending.result()["revision"], 1)
                self.assertFalse(store(path, "restarted").begin("new", "tool", "cap", {})[1])


    def test_process_exit_never_replays_recorded_or_uncertain_actions(self):
        for phase, count, delivery in (("after_begin", 0, "unknown"), ("after_action", 1, "unknown"),
                                       ("after_unknown", 1, "unknown"), ("after_finish", 1, "recorded")):
            with self.subTest(phase=phase):
                folder = self.root / phase
                folder.mkdir()
                self.run_child("intent", phase, folder)
                service = store(folder / "agent_calls.sqlite3", "restarted")
                record, acquired = service.begin("request", "tool", "cap", {})
                self.assertFalse(acquired)
                self.assertEqual(record["delivery_state"], delivery)
                action = folder / "action-count"
                self.assertEqual(int(action.read_text()) if action.exists() else 0, count)

    def run_child(self, operation, phase, folder):
        result = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--child", operation, phase, str(folder)],
                                capture_output=True, text=True, timeout=12)
        self.assertEqual(result.returncode, EXIT_CODE, result.stdout + result.stderr)
        self.assertEqual((folder / "reached").read_text(), phase)


def child(operation, phase, folder):
    folder = Path(folder)
    path = folder / "agent_calls.sqlite3"
    def stop():
        (folder / "reached").write_text(phase)
        os._exit(EXIT_CODE)
    if operation == "intent":
        service = store(path)
        service.begin("request", "tool", "cap", {})
        if phase == "after_begin": stop()
        (folder / "action-count").write_text("1")
        if phase == "after_action": stop()
        if phase == "after_unknown":
            service.mark_unknown("request")
            stop()
        service.finish("request", {"task_id": "task"})
        if phase == "after_finish": stop()
    raise AssertionError("没有到达中断点：" + phase)

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--child":
        child(*sys.argv[2:])
    else:
        unittest.main(verbosity=2)
