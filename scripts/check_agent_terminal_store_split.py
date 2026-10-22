#!/usr/bin/env python3
"""终态唯一分库与一次性前向导入回归；仅使用自行创建的数据库和动作计数。

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
from core.agent_terminal_store import initialize_terminal_file, TERMINAL_FIELDS
from core.errors import GatewayError
import core.agent_call_store as call_module

CONNECT = sqlite3.connect
IDENTITY = {"robot_model": "offline", "robot_id": "one"}
EXIT_CODE = 71
FIELDS = TERMINAL_FIELDS

def snapshot(task="task", revision=8):
    return {"task_id": task, "status": {"state": "succeeded"}, "result": {"revision": revision}}

def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)

def legacy_row(task="task", scope="opaque", revision=8, *, encoded=None):
    return (task, scope, "original-gateway", encoded or canonical(snapshot(task, revision)),
            "original-time", "source" if revision is not None else None, revision)

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

def legacy_database(path, rows=(), *, versioned=True):
    with CONNECT(str(path)) as db:
        db.execute("CREATE TABLE agent_task_terminals (task_id TEXT PRIMARY KEY, scope TEXT NOT NULL, "
                   "gateway_instance_id TEXT NOT NULL, snapshot_json TEXT NOT NULL, updated_at TEXT NOT NULL"
                   + (", receipt_source_id TEXT, receipt_revision INTEGER" if versioned else "") + ")")
        db.executemany("INSERT INTO agent_task_terminals VALUES (" + ",".join("?" for _ in range(7 if versioned else 5)) + ")",
                       [tuple(row[:7 if versioned else 5]) for row in rows])
        db.execute("CREATE TABLE agent_scope_owners (scope_id TEXT PRIMARY KEY, identity_json TEXT NOT NULL UNIQUE)")
        db.execute("CREATE TABLE agent_scope_bindings (scope TEXT PRIMARY KEY, owner_scope TEXT NOT NULL)")
        identity = json.dumps([IDENTITY, str((Path(path).parent / 'artifacts').resolve())], sort_keys=True)
        db.execute("INSERT INTO agent_scope_owners VALUES ('existing-owner',?)", (identity,))
        db.execute("INSERT INTO agent_scope_bindings VALUES ('existing-owner','existing-owner')")
        db.execute("INSERT INTO agent_scope_bindings VALUES ('opaque','existing-owner')")

def old_split_layout(path, *, layout="active", target_rows=()):
    target = Path(path).with_name("historical-terminals.sqlite3")
    initialize_terminal_file(target, "ledger", "pair")
    with CONNECT(str(target)) as db:
        db.executemany("INSERT INTO agent_task_terminals VALUES (?,?,?,?,?,?,?)", target_rows)
    with CONNECT(str(path)) as db:
        db.execute("CREATE TABLE agent_terminal_layout (singleton INTEGER PRIMARY KEY, ledger_id TEXT NOT NULL, "
                   "origin TEXT NOT NULL, layout_state TEXT NOT NULL, terminal_path TEXT NOT NULL, pair_id TEXT)")
        db.execute("INSERT INTO agent_terminal_layout VALUES (1,'ledger','legacy',?,?, 'pair')", (layout, str(target)))
        if layout == "active":
            for operation in ("INSERT", "UPDATE", "DELETE"):
                db.execute("CREATE TRIGGER agent_terminal_legacy_seal_" + operation.lower()
                           + " BEFORE " + operation + " ON agent_task_terminals BEGIN SELECT RAISE(ABORT,'sealed'); END")
    return target
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
        self.assertEqual(state(self.path)["import_version"], 1)
        with CONNECT(self.path) as db:
            self.assertIsNone(db.execute("SELECT name FROM sqlite_master WHERE name='agent_task_terminals'").fetchone())
        self.assertEqual(self.save(service, 1)["disposition"], "inserted")
        self.assertEqual(raw_rows(target_path(self.path))[0][-1], 1)
        with CONNECT(target_path(self.path)) as db:
            self.assertEqual({row[0] for row in db.execute("SELECT name FROM sqlite_master WHERE type='table'")},
                             {"terminal_store_identity", "agent_task_terminals"})
        self.assertEqual(self.path.stat().st_mode & 0o777, 0o600)
        self.assertEqual(target_path(self.path).stat().st_mode & 0o777, 0o600)

    def test_unversioned_original_bytes_and_binding_survive_import(self):
        original = legacy_row(revision=None, encoded='{ "task_id": "task", "result": "历史原文" }')
        legacy_database(self.path, [original], versioned=False)
        service = store(self.path)
        self.assertEqual(service.scope, "existing-owner")
        self.assertEqual(raw_rows(self.path), [original])
        self.assertEqual(raw_rows(target_path(self.path)), [original])
        self.assertEqual(service.get_terminal("task")["gateway_instance_id"], "original-gateway")
        self.error("agent_terminal_legacy_conflict", lambda: self.save(service, 1))
        self.assertEqual(service.terminal_versions(["task"])["task"]["error_code"], "agent_terminal_legacy_conflict")

    def test_old_combined_preserves_call_bytes_and_unknown_without_replay(self):
        original = legacy_row(encoded='{ "task_id": "task", "result": "历史原文" }')
        legacy_database(self.path, [original])
        with CONNECT(self.path) as db:
            db.execute("CREATE TABLE agent_terminal_layout (singleton INTEGER PRIMARY KEY,ledger_id TEXT NOT NULL,"
                       "origin TEXT NOT NULL,layout_state TEXT NOT NULL,terminal_path TEXT NOT NULL,pair_id TEXT)")
            db.execute("INSERT INTO agent_terminal_layout VALUES (1,'old-ledger','legacy','combined',?,NULL)", (str(self.path),))
            db.execute("CREATE TABLE agent_calls (request_id TEXT PRIMARY KEY,scope TEXT NOT NULL,gateway_instance_id TEXT NOT NULL,"
                       "tool_name TEXT NOT NULL,capability_name TEXT NOT NULL,arguments_json TEXT NOT NULL,fingerprint TEXT NOT NULL,"
                       "delivery_state TEXT NOT NULL,response_json TEXT,error_json TEXT,task_id TEXT,created_at TEXT NOT NULL,updated_at TEXT NOT NULL)")
            rows = [("recorded", "opaque", "old-gateway", "tool", "cap", '{"n":1}', "legacy-fingerprint", "recorded",
                     '{ "result": "调用原文" }', None, "task", "created", "updated"),
                    ("pending", "opaque", "old-gateway", "tool", "cap", '{}', "legacy-fingerprint", "pending",
                     None, None, None, "created", "updated")]
            db.executemany("INSERT INTO agent_calls VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)", rows)
        service = store(self.path, "restarted")
        self.assertFalse(service.begin("recorded", "tool", "cap", {"n": 1})[1])
        pending, acquired = service.begin("pending", "tool", "cap", {})
        self.assertFalse(acquired)
        self.assertEqual(pending["delivery_state"], "unknown")
        with CONNECT(self.path) as db:
            self.assertEqual(db.execute("SELECT * FROM agent_calls ORDER BY request_id").fetchall(), sorted(rows))
        self.assertEqual(state(self.path)["ledger_id"], "old-ledger")
        self.assertEqual(raw_rows(target_path(self.path)), [original])
        self.assertEqual(raw_rows(self.path), [original])

    def test_old_pending_without_target_completes_the_recorded_pair(self):
        legacy_database(self.path, [legacy_row()])
        target = old_split_layout(self.path, layout="pending")
        target.unlink()
        service = store(self.path)
        self.assertEqual(state(self.path)["pair_id"], "pair")
        self.assertEqual(state(self.path)["import_version"], 1)
        self.assertEqual(service.get_terminal("task")["snapshot"], snapshot())
        self.assertEqual(raw_rows(target), [legacy_row()])

    def test_old_active_imports_unseeded_rows_and_preserves_newer_target(self):
        originals = [legacy_row("a"), legacy_row("b")]
        legacy_database(self.path, originals)
        newer = legacy_row("a", revision=9)
        target = old_split_layout(self.path, target_rows=[newer])
        service = store(self.path)
        self.assertEqual(raw_rows(target), [newer, originals[1]])
        self.assertEqual(raw_rows(self.path), originals)
        self.assertEqual(service.get_terminal("a")["snapshot"], snapshot("a", 9))
        with CONNECT(self.path) as db:
            self.assertEqual(db.execute("SELECT count(*) FROM sqlite_master WHERE type='trigger'").fetchone()[0], 0)

    def test_same_version_conflict_keeps_both_original_files(self):
        legacy_database(self.path, [legacy_row("a"), legacy_row("b")])
        conflict = legacy_row("b", encoded='{"conflict":true}')
        target = old_split_layout(self.path, target_rows=[conflict])
        before_source, before_target = raw_rows(self.path), raw_rows(target)
        self.error("agent_terminal_revision_conflict", lambda: store(self.path))
        self.assertEqual(raw_rows(self.path), before_source)
        self.assertEqual(raw_rows(target), before_target)
        self.assertEqual(state(self.path)["import_version"], 0)

    def test_unknown_scope_is_copied_without_becoming_authorized(self):
        original = legacy_row(scope="unbound-opaque")
        legacy_database(self.path, [original])
        service = store(self.path)
        self.assertEqual(raw_rows(target_path(self.path)), [original])
        self.error("not_found", lambda: service.get_terminal("task"))
        self.error("agent_terminal_scope_conflict", lambda: self.save(service, 9))

    def test_runtime_never_reads_or_writes_legacy_results(self):
        legacy_database(self.path, [legacy_row()])
        service = store(self.path)
        with CONNECT(self.path) as db:
            db.execute("DELETE FROM agent_task_terminals")
            db.execute("INSERT INTO agent_task_terminals VALUES (?,?,?,?,?,?,?)", legacy_row("old-only"))
        with observed_connections(self.path, target_path(self.path), reject_intent_writes=True) as observed:
            self.assertEqual(service.get_terminal("task")["snapshot"], snapshot())
            self.save(service, 9)
            service.terminal_versions(["task"])
            self.error("not_found", lambda: service.get_terminal("old-only"))
        self.assertFalse(any(role == "intent" and "agent_task_terminals" in sql for role, sql in observed["events"]))
        self.assertEqual(raw_rows(self.path), [legacy_row("old-only")])

    def test_missing_completed_or_old_active_target_is_not_recreated(self):
        for old in (False, True):
            with self.subTest(old=old):
                path = self.root / ("old.sqlite3" if old else "new.sqlite3")
                if old:
                    legacy_database(path, [legacy_row()])
                    target = old_split_layout(path)
                else:
                    service = store(path)
                    self.save(service, 8)
                    target = target_path(path)
                target.unlink()
                with self.assertRaises((sqlite3.Error, OSError, GatewayError)):
                    store(path)
                self.assertFalse(target.exists())

    def test_partial_null_version_rows_remain_read_only(self):
        rows = [list(legacy_row("source-null")), list(legacy_row("revision-null"))]
        rows[0][5] = None
        rows[1][6] = None
        legacy_database(self.path, rows)
        service = store(self.path)
        for row in rows:
            self.error("agent_terminal_legacy_conflict", lambda row=row: self.save(service, 9, row[0]))
        self.assertEqual(raw_rows(target_path(self.path)), sorted(tuple(row) for row in rows))

    def test_foreign_target_is_not_claimed(self):
        legacy_database(self.path, [legacy_row()])
        target = old_split_layout(self.path)
        with CONNECT(target) as db:
            db.execute("UPDATE terminal_store_identity SET pair_id='different'")
        before = raw_rows(self.path), raw_rows(target)
        self.error("agent_terminal_pair_conflict", lambda: store(self.path))
        self.assertEqual((raw_rows(self.path), raw_rows(target)), before)

    def test_scope_allocation_collision_does_not_claim_unbound_historical_terminals(self):
        from types import SimpleNamespace
        raw_scope = "scope_" + "c" * 32
        legacy_database(self.path, [legacy_row(scope=raw_scope)])
        store(self.path)
        with patch.object(call_module, "uuid4", side_effect=[SimpleNamespace(hex="c" * 32), SimpleNamespace(hex="d" * 32)]):
            other = AgentCallStore(self.path, robot_identity={**IDENTITY, "robot_id": "other"},
                                   artifact_dir=self.root / "artifacts", gateway_instance_id="other")
        self.assertEqual(other.scope, "scope_" + "d" * 32)
        self.error("not_found", lambda: other.get_terminal("task"))
        self.assertEqual(raw_rows(target_path(self.path))[0][1], raw_scope)

    def test_concurrent_constructors_share_one_pair_and_owner(self):
        with ThreadPoolExecutor(max_workers=6) as pool:
            stores = list(pool.map(lambda index: store(self.path, str(index)), range(6)))
        self.assertEqual(len({item.scope for item in stores}), 1)
        self.assertEqual(len({item._terminal_store.pair_id for item in stores}), 1)
        self.assertEqual(state(self.path)["import_version"], 1)

    def test_monotonic_versions_and_original_scope_survive_concurrent_writes(self):
        original = legacy_row()
        legacy_database(self.path, [original])
        stores = [store(self.path, str(index)) for index in range(4)]
        with ThreadPoolExecutor(max_workers=4) as pool:
            results = list(pool.map(lambda pair: self.save(*pair), zip(stores, (7, 9, 9, 8))))
        self.assertTrue(all(item["revision"] in (8, 9) for item in results))
        self.assertEqual(stores[0].terminal_versions(["task"])["task"]["revision"], 9)
        self.assertEqual(raw_rows(target_path(self.path))[0][:3], original[:3])
        self.error("agent_terminal_revision_conflict", lambda: self.save(stores[0], 9, value="different"))
        self.error("agent_terminal_source_conflict", lambda: self.save(stores[0], 10, source="another"))
        self.assertEqual(raw_rows(self.path), [original])

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

    def test_process_exit_during_import_retries_exact_original_rows(self):
        for phase in ("before_pair", "after_pair", "during_copy", "after_copy", "before_marker", "after_marker"):
            with self.subTest(phase=phase):
                folder = self.root / phase
                folder.mkdir()
                path = folder / "agent_calls.sqlite3"
                rows = [legacy_row(f"task-{index:04d}") for index in range(130)]
                legacy_database(path, rows)
                self.run_child("import", phase, folder)
                service = store(path, "restarted")
                self.assertEqual(raw_rows(path), rows)
                self.assertEqual(raw_rows(target_path(path)), rows)
                self.assertEqual(state(path)["import_version"], 1)
                self.assertEqual(service.get_terminal(rows[0][0])["recorded_at"], "original-time")

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
    else:
        target = path.with_name(path.stem + ".terminals.sqlite3")
        def commit(db, when):
            pair = any("CREATE TABLE terminal_store_identity" in sql for sql in db.sql)
            copied = any("INSERT OR REPLACE INTO agent_task_terminals" in sql for sql in db.sql)
            marked = any("import_version=1" in sql for sql in db.sql)
            if ((phase == when + "_pair" and db.role == "terminal" and pair)
                    or (phase == when + "_copy" and db.role == "terminal" and copied)
                    or (phase == when + "_marker" and db.role == "intent" and marked)):
                stop()
        with observed_connections(path, target, before_commit=lambda db: commit(db, "before"),
                                  after_commit=lambda db: commit(db, "after")):
            if phase == "during_copy":
                calls = [0]
                original_import = call_module.AgentTerminalStore.import_legacy_rows
                class WrappedSource:
                    def __init__(self, source): self.source = source
                    def execute(self, sql, *args):
                        cursor = self.source.execute(sql, *args)
                        if sql.startswith("SELECT task_id,scope,"):
                            class Cursor:
                                def fetchmany(self, count):
                                    calls[0] += 1
                                    if calls[0] == 2: stop()
                                    return cursor.fetchmany(count)
                            return Cursor()
                        return cursor
                def interrupted(owner, source):
                    return original_import(owner, WrappedSource(source))
                with patch.object(call_module.AgentTerminalStore, "import_legacy_rows", interrupted):
                    store(path)
            else:
                store(path)
    raise AssertionError("没有到达中断点：" + phase)

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--child":
        child(*sys.argv[2:])
    else:
        unittest.main(verbosity=2)
