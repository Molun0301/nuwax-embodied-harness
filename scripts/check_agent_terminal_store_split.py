#!/usr/bin/env python3
"""Offline split-store gates using synthetic actions and self-created SQLite files.

Run with PYTHONDONTWRITEBYTECODE=1 and optionally NEH_TERMINAL_SPLIT_TMP pointing
at an existing scratch directory. Child os._exit fixtures prove process-reopen
behavior only, never power-loss or filesystem-media durability.
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
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import core.agent_call_store as call_module
from core.agent_call_store import AgentCallStore
from core.errors import GatewayError
from gateways.feedback import build_public_error_payload

FIELDS = ("task_id", "scope", "gateway_instance_id", "snapshot_json", "updated_at",
          "receipt_source_id", "receipt_revision")
IDENTITY = {"robot_model": "offline", "robot_name": "split-fixture", "entrypoint": "fake", "robot_id": "one"}
CONNECT = sqlite3.connect
EXIT_CODE = 71


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)


def snapshot(task="task", revision=8):
    return {"task_id": task, "status": {"state": "succeeded"}, "result": {"revision": revision}}


def legacy_database(path, rows=()):
    """Recognizable pre-split schema; never infer age from an empty file."""
    with CONNECT(str(path)) as db:
        db.execute("CREATE TABLE agent_task_terminals (task_id TEXT PRIMARY KEY, scope TEXT NOT NULL, "
                   "gateway_instance_id TEXT NOT NULL, snapshot_json TEXT NOT NULL, updated_at TEXT NOT NULL, "
                   "receipt_source_id TEXT, receipt_revision INTEGER)")
        db.executemany("INSERT INTO agent_task_terminals VALUES (?,?,?,?,?,?,?)", rows)


def legacy_row(task="task", scope="scope", revision=8, *, encoded=None):
    return (task, scope, "original-gateway", encoded or canonical(snapshot(task, revision)),
            "original-time", "source" if revision is not None else None, revision)


def raw_rows(path):
    with CONNECT(str(path)) as db:
        return db.execute("SELECT " + ",".join(FIELDS) + " FROM agent_task_terminals ORDER BY task_id").fetchall()


def status(path):
    return AgentCallStore.terminal_storage_status(path)


def target_path(path):
    return Path(status(path)["terminal_path"])


def activate(path, terminal=None, **overrides):
    values = {"expected_ledger_id": status(path)["ledger_id"],
              "terminal_path": terminal or path.with_name("separate-terminals.sqlite3"),
              "writers_quiesced": True}
    values.update(overrides)
    return AgentCallStore.activate_terminal_split(path, **values)


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


class SplitFixture(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="neh-terminal-split-", dir=os.environ.get("NEH_TERMINAL_SPLIT_TMP"))
        self.root = Path(self.temp.name)
        self.path = self.root / "agent_calls.sqlite3"
        self.artifacts = self.root / "artifacts"

    def tearDown(self):
        self.temp.cleanup()

    def store(self, gateway="owner", *, identity=False, path=None, scope="scope"):
        if identity:
            return AgentCallStore.for_identity(path or self.path, robot_identity=IDENTITY,
                                              artifact_dir=self.artifacts, gateway_instance_id=gateway)
        return AgentCallStore(path or self.path, scope=scope, gateway_instance_id=gateway)

    def old(self, rows=None, *, identity=False):
        legacy_database(self.path, (legacy_row(),) if rows is None else rows)
        return self.store(identity=identity)

    def split(self, rows=None, *, identity=False):
        store = self.old(rows, identity=identity)
        activate(self.path)
        return store

    def error(self, code, fn):
        with self.assertRaises(GatewayError) as caught:
            fn()
        self.assertEqual(caught.exception.error_code, code)
        return caught.exception

    def storage_error(self, fn):
        with self.assertRaises((GatewayError, sqlite3.Error, OSError)) as caught:
            fn()
        if isinstance(caught.exception, GatewayError):
            self.assertNotEqual(caught.exception.error_code, "not_found", "storage failure was reported as healthy absence")
        return caught.exception

    def save(self, store, revision, task="task", *, source="source", value=None):
        return store.save_terminal(task, snapshot(task, revision if value is None else value),
                                   source_id=source, revision=revision)

    def bind(self, scope, *, identity=None):
        return AgentCallStore.bind_verified_legacy_scope(self.path, legacy_scope=scope,
                                                         robot_identity=identity or IDENTITY, artifact_dir=self.artifacts)


class LayoutAndPairingChecks(SplitFixture):
    def test_fresh_defaults_split_and_terminal_has_no_scope_registry(self):
        store = self.store()
        state = status(self.path)
        self.assertEqual(state["layout_state"], "active")
        self.assertTrue(state["ledger_id"] and state["pair_id"])
        terminal = target_path(self.path)
        self.assertNotEqual(terminal.resolve(), self.path.resolve())
        self.assertEqual(self.save(store, 1)["disposition"], "inserted")
        self.assertEqual(raw_rows(self.path), [])
        self.assertEqual(raw_rows(terminal)[0][-1], 1)
        with CONNECT(str(terminal)) as db:
            tables = {row[0] for row in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
        self.assertNotIn("agent_scope_owners", tables)
        self.assertNotIn("agent_scope_bindings", tables)
        self.assertEqual(self.path.stat().st_mode & 0o777, 0o600)
        self.assertEqual(terminal.stat().st_mode & 0o777, 0o600)

    def test_existing_schema_stays_combined_until_explicit_activation(self):
        store = self.old()
        self.assertEqual(status(self.path)["layout_state"], "combined")
        self.assertEqual(self.save(store, 9)["disposition"], "updated")
        self.assertEqual(raw_rows(self.path)[0][-1], 9)
        activate(self.path)
        self.assertEqual(status(self.path)["layout_state"], "active")
        self.assertEqual(store.get_terminal("task")["snapshot"], snapshot(revision=9))
        self.assertEqual(self.save(store, 10)["revision"], 10)
        self.assertEqual(raw_rows(self.path)[0][-1], 9)
        self.assertEqual(raw_rows(target_path(self.path))[0][-1], 10)

    def test_status_is_readonly_and_missing_does_not_create(self):
        self.storage_error(lambda: status(self.path))
        self.assertFalse(self.path.exists())
        legacy_database(self.path, [legacy_row()])
        before = self.path.read_bytes()
        try:
            result = status(self.path)
            self.assertNotEqual(result.get("layout_state"), "active")
        except (GatewayError, sqlite3.Error):
            pass  # Uninitialized legacy metadata may be explicitly reported.
        self.assertEqual(self.path.read_bytes(), before)
        with CONNECT(str(self.path)) as db:
            self.assertIsNone(db.execute("SELECT name FROM sqlite_master WHERE name='agent_scope_owners'").fetchone())

    def test_unrecognized_empty_file_not_adopted_as_fresh(self):
        self.path.touch()
        self.storage_error(self.store)
        self.assertFalse(self.path.with_name("agent_calls.terminals.sqlite3").exists())

    def test_concurrent_fresh_constructors_choose_one_pair_and_owner(self):
        barrier = threading.Barrier(4)
        def construct(index):
            barrier.wait(3)
            store = self.store("gateway-" + str(index), identity=True)
            return store.scope, status(self.path), store.begin("one", "tool", "cap", {})[1]
        with ThreadPoolExecutor(max_workers=4) as pool:
            results = list(pool.map(construct, range(4)))
        self.assertEqual(len({row[0] for row in results}), 1)
        self.assertEqual(len({row[1]["ledger_id"] for row in results}), 1)
        self.assertEqual(len({row[1]["pair_id"] for row in results}), 1)
        self.assertEqual(len({row[1]["terminal_path"] for row in results}), 1)
        self.assertEqual(sum(row[2] for row in results), 1)

    def test_raw_token_reservation_blocks_assigned_collision_without_granting_binding(self):
        raw = self.store(scope="scope_reserved")
        self.save(raw, 8)
        self.store(scope="scope_unused")
        self.assertEqual(raw_rows(self.path), [])
        self.assertEqual(raw_rows(target_path(self.path))[0][1], "scope_reserved")
        self.assertEqual(AgentCallStore.list_legacy_scopes(self.path), ("scope_reserved",))
        self.error("not_found", lambda: self.bind("scope_unused"))
        with patch.object(call_module, "uuid4", side_effect=(SimpleNamespace(hex="reserved"),
                                                            SimpleNamespace(hex="allocated"))):
            assigned = self.store(identity=True)
        self.assertEqual(assigned.scope, "scope_allocated")
        self.error("agent_scope_binding_required", lambda: assigned.get_terminal("task"))
        self.assertEqual(AgentCallStore.list_legacy_scopes(self.path), ("scope_reserved",))
        with CONNECT(str(self.path)) as db:
            self.assertIsNone(db.execute("SELECT owner_scope FROM agent_scope_bindings WHERE scope='scope_unused'").fetchone())
            self.assertEqual(db.execute("SELECT COUNT(*) FROM agent_scope_owners").fetchone()[0], 1)

    def test_cutover_requires_quiescence_and_exact_ledger(self):
        self.old()
        self.storage_error(lambda: activate(self.path, writers_quiesced=False))
        self.storage_error(lambda: activate(self.path, expected_ledger_id="wrong-ledger"))
        self.assertEqual(status(self.path)["layout_state"], "combined")
        activate(self.path)
        before = status(self.path)
        activate(self.path, terminal=target_path(self.path))
        self.assertEqual(status(self.path), before)
        self.storage_error(lambda: activate(self.path, terminal=self.root / "different.sqlite3"))

    def test_same_file_symlink_hardlink_and_unrelated_target_rejected(self):
        self.old()
        self.storage_error(lambda: activate(self.path, terminal=self.path))
        symlink = self.root / "same-symbolic.sqlite3"
        symlink.symlink_to(self.path)
        self.storage_error(lambda: activate(self.path, terminal=symlink))
        hardlink = self.root / "same-hard.sqlite3"
        os.link(self.path, hardlink)
        self.storage_error(lambda: activate(self.path, terminal=hardlink))
        unrelated = self.root / "unrelated.sqlite3"
        with CONNECT(str(unrelated)) as db:
            db.execute("CREATE TABLE unrelated (value TEXT)")
            db.execute("INSERT INTO unrelated VALUES ('preserve')")
        before = unrelated.read_bytes()
        self.storage_error(lambda: activate(self.path, terminal=unrelated))
        self.assertEqual(unrelated.read_bytes(), before)
        self.assertNotEqual(status(self.path)["layout_state"], "active")

    def test_other_ledger_target_pair_cannot_be_adopted(self):
        self.old()
        other = self.root / "other.sqlite3"
        self.store(path=other)
        paired = target_path(other)
        before = paired.read_bytes()
        self.storage_error(lambda: activate(self.path, terminal=paired))
        self.assertEqual(paired.read_bytes(), before)


class LegacyMergeChecks(SplitFixture):
    def test_first_stale_seed_preserves_all_seven_fields_and_equal_later_cas(self):
        original = legacy_row()
        store = self.split([original])
        terminal = target_path(self.path)
        self.assertEqual(raw_rows(terminal), [])
        self.assertEqual(store.get_terminal("task")["recorded_at"], "original-time")
        self.assertEqual(store.terminal_versions(["task"])["task"]["revision"], 8)
        self.assertEqual(self.save(store, 7), {"disposition": "stale", "source_id": "source", "revision": 8})
        self.assertEqual(raw_rows(terminal), [original])
        self.assertEqual(self.save(store, 8)["disposition"], "unchanged")
        self.error("agent_terminal_revision_conflict", lambda: self.save(store, 8, value="different"))
        self.error("agent_terminal_source_conflict", lambda: self.save(store, 9, source="different"))
        self.assertEqual(raw_rows(terminal), [original])
        self.assertEqual(self.save(store, 9)["disposition"], "updated")
        row = raw_rows(terminal)[0]
        self.assertEqual(row[0:3], original[0:3])
        self.assertEqual(row[5:], ("source", 9))
        self.assertEqual(raw_rows(self.path), [original])
        other = self.store("second-gateway")
        self.assertEqual(self.save(other, 9)["disposition"], "unchanged")
        self.assertEqual(self.save(other, 7)["revision"], 9)
        self.assertEqual(other.get_terminal("task")["gateway_instance_id"], "original-gateway")

    def test_noncanonical_legacy_bytes_and_null_versions_are_not_adopted(self):
        original = legacy_row(revision=None, encoded='{ "task_id": "task", "result": {"value": "原样"} }')
        store = self.split([original])
        before = store.get_terminal("task")
        self.error("agent_terminal_legacy_conflict", lambda: self.save(store, 1))
        self.assertEqual(store.get_terminal("task"), before)
        self.assertEqual(store.terminal_versions(["task"])["task"],
                         {"source_id": None, "revision": None, "error_code": "agent_terminal_legacy_conflict"})
        self.assertEqual(raw_rows(self.path), [original])
        rows = raw_rows(target_path(self.path))
        self.assertIn(rows, ([], [original]), "rejected candidate must leave absent or exact legacy seed")

    def test_partially_null_legacy_source_or_revision_also_blocks_adoption(self):
        source_null = list(legacy_row("source-null"))
        source_null[5] = None
        revision_null = list(legacy_row("revision-null"))
        revision_null[6] = None
        rows = [tuple(source_null), tuple(revision_null)]
        store = self.split(rows)
        for row in rows:
            with self.subTest(task=row[0]):
                self.error("agent_terminal_legacy_conflict", lambda row=row: self.save(store, 9, row[0]))
                self.assertEqual(store.terminal_versions([row[0]])[row[0]]["error_code"], "agent_terminal_legacy_conflict")
        self.assertEqual(raw_rows(self.path), sorted(rows))
        self.assertTrue(all(row in rows for row in raw_rows(target_path(self.path))))

    def test_multiple_opaque_aliases_preserve_original_scope_and_gateway(self):
        rows = [legacy_row("a", "opaque-a"), legacy_row("b", "opaque-b")]
        store = self.old(rows, identity=True)
        owner = self.bind("opaque-a")
        self.assertEqual(self.bind("opaque-b"), owner)
        self.assertEqual(self.bind("opaque-a"), owner)
        self.error("agent_scope_binding_conflict", lambda: self.bind("opaque-a", identity=dict(IDENTITY, robot_id="two")))
        activate(self.path)
        for task in ("a", "b"):
            self.assertEqual(self.save(store, 9, task)["revision"], 9)
            self.assertEqual(store.get_terminal(task)["gateway_instance_id"], "original-gateway")
        self.assertEqual([(r[0], r[1], r[2]) for r in raw_rows(target_path(self.path))],
                         [(r[0], r[1], r[2]) for r in rows])
        wrong = AgentCallStore.for_identity(self.path, robot_identity=dict(IDENTITY, robot_id="two"),
                                            artifact_dir=self.artifacts, gateway_instance_id="wrong")
        self.error("not_found", lambda: wrong.get_terminal("a"))
        self.error("agent_terminal_scope_conflict", lambda: self.save(wrong, 10, "a"))
        self.assertEqual(wrong.terminal_versions(["a"])["a"]["error_code"], "agent_terminal_scope_conflict")

    def test_unbound_terminal_only_scope_stays_blocked(self):
        store = self.split([legacy_row(scope="opaque-unbound")], identity=True)
        self.error("agent_scope_binding_required", lambda: store.get_terminal("task"))
        self.error("agent_scope_binding_required", lambda: self.save(store, 9))
        self.assertEqual(store.terminal_versions(["task"])["task"]["error_code"], "agent_scope_binding_required")
        self.assertEqual(raw_rows(target_path(self.path)), [])
        self.assertEqual(AgentCallStore.list_legacy_scopes(self.path), ("opaque-unbound",))

    def test_split_terminal_only_raw_scope_can_be_discovered_and_explicitly_bound(self):
        raw = self.store(scope="terminal-only-opaque")
        self.save(raw, 8)
        self.assertEqual(raw_rows(self.path), [])
        self.assertEqual(AgentCallStore.list_legacy_scopes(self.path), ("terminal-only-opaque",))
        registered = self.store(identity=True)
        self.error("agent_scope_binding_required", lambda: registered.get_terminal("task"))
        self.assertEqual(self.bind("terminal-only-opaque"), registered.scope)
        self.assertEqual(self.bind("terminal-only-opaque"), registered.scope)
        self.assertEqual(AgentCallStore.list_legacy_scopes(self.path), ())
        self.assertEqual(registered.get_terminal("task")["snapshot"], snapshot())
        self.assertEqual(self.save(registered, 9)["revision"], 9)
        self.assertEqual(raw_rows(target_path(self.path))[0][1], "terminal-only-opaque")
        self.error("agent_scope_binding_conflict", lambda: self.bind(
            "terminal-only-opaque", identity=dict(IDENTITY, robot_id="wrong")))

    def test_concurrent_first_seed_reverse_revision_and_equal_content_services(self):
        self.split()
        stores = [self.store("service-" + str(index)) for index in range(4)]
        barrier = threading.Barrier(4)
        def save(index):
            barrier.wait(3)
            return self.save(stores[index], (7, 9, 9, 8)[index])
        with ThreadPoolExecutor(max_workers=4) as pool:
            results = list(pool.map(save, range(4)))
        self.assertTrue(all(result["revision"] in (8, 9) for result in results))
        self.assertEqual(stores[0].terminal_versions(["task"])["task"]["revision"], 9)
        row = raw_rows(target_path(self.path))[0]
        self.assertEqual(row[0:3], legacy_row()[0:3])
        self.assertEqual(row[-1], 9)
        self.assertEqual(raw_rows(self.path), [legacy_row()])

    def test_versions_are_bounded_and_do_not_decode_snapshot_json(self):
        store = self.split()
        self.save(store, 1, "new")
        with patch.object(json, "loads", side_effect=AssertionError("version query decoded snapshot JSON")):
            result = store.terminal_versions(["task", "new", "absent"])
        self.assertEqual({task: row["revision"] for task, row in result.items()}, {"task": 8, "new": 1})
        self.assertEqual(store.terminal_versions([]), {})
        with self.assertRaises(ValueError):
            store.terminal_versions([str(i) for i in range(65)])

    def test_prepared_receipt_is_facade_owned(self):
        first = self.store()
        second = self.store("other")
        prepared = first.prepare_terminal("task", snapshot(), source_id="source", revision=8)
        self.error("agent_terminal_identity_conflict", lambda: second.save_prepared_terminal(prepared))
        self.assertEqual(first.save_prepared_terminal(prepared)["revision"], 8)


class StorageFailureAndSealChecks(SplitFixture):
    def test_storage_errors_wrap_direct_reads_but_remain_retryable_inside_owner(self):
        store = self.split()
        terminal = target_path(self.path)
        prepared = store.prepare_terminal("task", snapshot(revision=9), source_id="source", revision=9)
        def unavailable(*args, **kwargs):
            if terminal.name in str(args[0]):
                raise PermissionError("synthetic target permission failure")
            return CONNECT(*args, **kwargs)
        with patch.object(sqlite3, "connect", unavailable):
            self.error("agent_terminal_store_unavailable", lambda: store.get_terminal("task"))
            self.error("agent_terminal_store_unavailable", lambda: self.save(store, 9))
            with self.assertRaises(PermissionError):
                store.terminal_versions(["task"])
            with self.assertRaises(PermissionError):
                store.save_prepared_terminal(prepared)
            self.assertTrue(store.begin("still-usable", "tool", "cap", {})[1])
            self.assertEqual(store.finish("still-usable", {})["delivery_state"], "recorded")

    def test_receipt_pairing_and_storage_public_errors_keep_original_identity(self):
        for code in ("agent_terminal_pair_conflict", "agent_terminal_cutover_required", "agent_terminal_store_unavailable"):
            with self.subTest(code=code):
                result = build_public_error_payload(GatewayError("Known action outcome unchanged", error_code=code,
                                                                 http_status=503 if code.endswith("unavailable") else 409),
                                                    task_id="original-task", request_id="original-request")
                self.assertEqual(result["error_category"], "receipt")
                self.assertFalse(result["retryable"])
                self.assertEqual(result["task_id"], "original-task")
                self.assertEqual(result["request_id"], "original-request")
                self.assertIn("不要重发动作", result["next_action"])

    def test_missing_active_target_never_recreates_or_serves_stale_legacy(self):
        store = self.split()
        self.save(store, 9)
        terminal = target_path(self.path)
        moved = terminal.with_suffix(".held")
        terminal.rename(moved)
        try:
            reopened = self.store("restart")
            self.storage_error(lambda: activate(self.path, terminal=terminal))
            self.assertFalse(terminal.exists(), "repeated activation recreated an empty active target")
            for current in (store, reopened):
                self.storage_error(lambda current=current: current.get_terminal("task"))
                self.storage_error(lambda current=current: current.terminal_versions(["task"]))
                self.storage_error(lambda current=current: self.save(current, 10))
            self.assertFalse(terminal.exists())
            _, acquired = reopened.begin("usable", "tool", "cap", {})
            self.assertTrue(acquired)
            self.assertEqual(reopened.finish("usable", {"known": True})["delivery_state"], "recorded")
        finally:
            moved.rename(terminal)
        self.assertEqual(store.get_terminal("task")["snapshot"], snapshot(revision=9))

    def test_active_missing_legacy_seal_cannot_be_silently_repaired(self):
        store = self.split()
        self.save(store, 9)
        terminal = target_path(self.path)
        before = raw_rows(terminal)
        with CONNECT(str(self.path)) as db:
            db.execute("DROP TRIGGER agent_terminal_legacy_seal_update")
            db.execute("UPDATE agent_task_terminals SET receipt_revision=100, updated_at='untrusted-change'")
        self.assertFalse(status(self.path)["legacy_sealed"])
        self.error("agent_terminal_pair_conflict", lambda: activate(self.path, terminal=terminal))
        self.assertFalse(status(self.path)["legacy_sealed"], "activation silently repaired an untrusted legacy seal")
        self.error("agent_terminal_pair_conflict", lambda: store.get_terminal("task"))
        self.error("agent_terminal_pair_conflict", lambda: store.terminal_versions(["task"]))
        self.assertEqual(raw_rows(terminal), before)
        self.assertEqual(raw_rows(self.path)[0][-1], 100)
        self.assertTrue(store.begin("intent-after-broken-seal", "tool", "cap", {})[1])
        self.assertEqual(store.finish("intent-after-broken-seal", {})["delivery_state"], "recorded")

    def test_healthy_absence_allows_legacy_but_wrong_pair_and_sqlite_outage_do_not(self):
        store = self.split()
        self.assertEqual(store.get_terminal("task")["snapshot"], snapshot())
        self.error("not_found", lambda: store.get_terminal("absent"))
        self.save(store, 9)
        terminal = target_path(self.path)
        other_path = self.root / "other.sqlite3"
        self.store(path=other_path)
        other = target_path(other_path)
        moved = terminal.with_suffix(".held")
        terminal.rename(moved)
        terminal.write_bytes(other.read_bytes())
        try:
            self.storage_error(lambda: store.get_terminal("task"))
            self.storage_error(lambda: store.terminal_versions(["task"]))
        finally:
            terminal.unlink()
            moved.rename(terminal)
        def unavailable(*args, **kwargs):
            value = str(args[0])
            if terminal.name in value:
                raise sqlite3.OperationalError("synthetic terminal storage outage")
            return CONNECT(*args, **kwargs)
        with patch.object(sqlite3, "connect", unavailable):
            self.storage_error(lambda: store.get_terminal("task"))
            self.storage_error(lambda: store.terminal_versions(["task"]))
            self.assertTrue(store.begin("intent-works", "tool", "cap", {})[1])

    def test_legacy_seal_blocks_insert_update_delete_but_not_old_noop_writer(self):
        store = self.split()
        original = raw_rows(self.path)
        statements = [("INSERT INTO agent_task_terminals VALUES (?,?,?,?,?,?,?)", legacy_row("other")),
                      ("UPDATE agent_task_terminals SET updated_at='wrong' WHERE task_id='task'", ()),
                      ("DELETE FROM agent_task_terminals WHERE task_id='task'", ())]
        for sql, args in statements:
            with self.subTest(sql=sql), CONNECT(str(self.path)) as db, self.assertRaises(sqlite3.IntegrityError):
                db.execute(sql, args)
        self.assertEqual(raw_rows(self.path), original)
        with CONNECT(str(self.path), isolation_level=None) as old:
            old.execute("BEGIN IMMEDIATE")  # A seal cannot prove old writers have quiesced.
            with observed_connections(self.path, target_path(self.path), timeout=0.03), self.assertRaises(sqlite3.OperationalError):
                store.begin("blocked-by-old-noop", "tool", "cap", {})
            old.rollback()
        self.assertTrue(store.begin("works", "tool", "cap", {})[1])
        self.assertEqual(store.finish("works", {})["delivery_state"], "recorded")


class IsolationAndAuthorityChecks(SplitFixture):
    def test_terminal_writer_gate_keeps_begin_and_finish_independent_in_delete_and_wal(self):
        for mode in ("delete", "wal"):
            for combined in (False, True):
                with self.subTest(mode=mode, combined=combined):
                    folder = self.root / (mode + ("-combined" if combined else "-split"))
                    folder.mkdir()
                    path = folder / "calls.sqlite3"
                    if combined:
                        legacy_database(path)
                    store = self.store(path=path)
                    terminal = path if combined else target_path(path)
                    for file in {path, terminal}:
                        with CONNECT(str(file)) as db:
                            self.assertEqual(db.execute("PRAGMA journal_mode=" + mode).fetchone()[0].lower(), mode)
                    store.begin("already-begun", "tool", "cap", {})
                    gate, release = threading.Event(), threading.Event()
                    errors, receipts, actions = [], [], []
                    def before_commit(db, gate=gate, release=release):
                        if threading.current_thread().name == "terminal-gate" and any("INSERT INTO AGENT_TASK_TERMINALS" in sql.upper() for sql in db.sql):
                            gate.set()
                            if not release.wait(4):
                                raise AssertionError("terminal commit gate expired")
                    def save(store=store, receipts=receipts, errors=errors):
                        try:
                            receipts.append(self.save(store, 1))
                        except BaseException as error:  # noqa: BLE001 - Relay worker failure to the main test thread.
                            errors.append(error)
                    with observed_connections(path, terminal, before_commit=before_commit, timeout=0.05):
                        worker = threading.Thread(target=save, name="terminal-gate")
                        worker.start()
                        try:
                            self.assertTrue(gate.wait(3), "real terminal transaction never reached commit gate")
                            if combined:
                                with self.assertRaises(sqlite3.OperationalError):
                                    store.begin("new", "tool", "cap", {})
                                with self.assertRaises(sqlite3.OperationalError):
                                    store.finish("already-begun", {})
                            else:
                                self.assertEqual(store.finish("already-begun", {})["delivery_state"], "recorded")
                                _, acquired = store.begin("new", "tool", "cap", {})
                                self.assertTrue(acquired)
                                actions.append(1)
                                store.finish("new", {"task_id": "synthetic-action"})
                                reopened = self.store(path=path, gateway="restart")
                                self.assertEqual(reopened.get("new")["delivery_state"], "recorded")
                                self.assertFalse(release.is_set())
                        finally:
                            release.set()
                            worker.join(4)
                    self.assertFalse(worker.is_alive())
                    self.assertEqual(errors, [])
                    self.assertEqual(len(receipts), 1)
                    if combined:
                        store.finish("already-begun", {})
                        _, acquired = store.begin("new", "tool", "cap", {})
                        self.assertTrue(acquired)
                        actions.append(1)
                        store.finish("new", {})
                    self.assertFalse(store.begin("new", "tool", "cap", {})[1])
                    self.assertEqual(actions, [1])

    def test_terminal_runtime_never_opens_intent_writer_or_holds_read_at_target_begin(self):
        store = self.old([legacy_row(scope="opaque")], identity=True)
        self.bind("opaque")
        activate(self.path)
        terminal = target_path(self.path)
        encoded = []
        original_json = call_module._json
        with observed_connections(self.path, terminal, reject_intent_writes=True) as observed:
            def encode(value):
                self.assertFalse(observed["writers"], "JSON encoded inside terminal writer transaction")
                encoded.append(value)
                return original_json(value)
            with patch.object(call_module, "_json", encode), patch.object(AgentCallStore, "_resolve_owner", side_effect=AssertionError("hidden scope resolution")):
                store.get_terminal("task")
                store.terminal_versions(["task"])
                self.save(store, 9)
                store.get_terminal("task")
                store.terminal_versions(["task"])
                self.save(store, 1, "new")
        self.assertTrue(encoded)
        self.assertTrue(any(role == "intent" and "SELECT" in sql.upper() for role, sql in observed["events"]))
        self.assertFalse(observed["open_intent"])
        self.assertFalse(any(role == "intent" and sql.lstrip().upper().startswith(
            ("BEGIN", "INSERT", "UPDATE", "DELETE", "CREATE", "ALTER", "REPLACE", "ATTACH"))
            for role, sql in observed["events"]))

    def test_new_target_scope_race_rechecks_authority_outside_writer_and_finishes_bounded(self):
        store = self.old([legacy_row(scope="opaque")], identity=True)
        self.bind("opaque")
        activate(self.path)
        alias = self.store(gateway="alias-first-writer", scope="opaque")
        injected = [False]
        starts = []
        def race(db):
            starts.append(1)
            if not injected[0]:
                injected[0] = True
                self.assertEqual(self.save(alias, 1, "new")["disposition"], "inserted")
        with observed_connections(self.path, target_path(self.path), before_begin=race,
                                  reject_intent_writes=True):
            self.assertEqual(self.save(store, 2, "new")["revision"], 2)
        self.assertTrue(injected[0])
        self.assertLessEqual(len(starts), 8, "authority race retry was not bounded")
        row = next(row for row in raw_rows(target_path(self.path)) if row[0] == "new")
        self.assertEqual(row[1:3], ("opaque", "alias-first-writer"))
        self.assertEqual(row[-1], 2)


class ProcessExitChecks(SplitFixture):
    def run_child(self, operation, phase, folder=None):
        folder = folder or self.root
        result = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--child", operation,
                                 phase, str(folder)], capture_output=True, text=True, timeout=12, check=False,
                                env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"))
        self.assertEqual(result.returncode, EXIT_CODE, result.stdout + result.stderr)
        self.assertEqual((folder / "reached").read_text(), phase)

    def test_begin_action_finish_process_exit_never_replays(self):
        for phase, count, state in (("after_begin", 0, "unknown"), ("after_action", 1, "unknown"),
                                    ("after_unknown", 1, "unknown"), ("after_finish", 1, "recorded")):
            with self.subTest(phase=phase):
                folder = self.root / phase
                folder.mkdir()
                self.run_child("intent", phase, folder)
                reopened = self.store(path=folder / "agent_calls.sqlite3", gateway="restarted")
                row, acquired = reopened.begin("request", "tool", "cap", {})
                self.assertFalse(acquired)
                self.assertEqual(row["delivery_state"], state)
                action = folder / "action-count"
                self.assertEqual(int(action.read_text()) if action.exists() else 0, count)
                with CONNECT(str(reopened.path)) as db:
                    physical = db.execute("SELECT delivery_state FROM agent_calls WHERE request_id='request'").fetchone()[0]
                self.assertEqual(physical, "recorded" if phase == "after_finish" else "unknown" if phase == "after_unknown" else "pending")

    def test_cutover_process_exit_pairs_or_reports_truthful_pending_and_resumes_exact_target(self):
        for phase in ("before_pending", "after_pending", "before_pair", "after_pair", "before_activation", "after_activation"):
            with self.subTest(phase=phase):
                folder = self.root / phase
                folder.mkdir()
                path = folder / "agent_calls.sqlite3"
                legacy_database(path, [legacy_row()])
                self.store(path=path)
                self.run_child("cutover", phase, folder)
                state = status(path)
                expected_state = "combined" if phase == "before_pending" else "active" if phase == "after_activation" else "pending"
                self.assertEqual(state["layout_state"], expected_state)
                if phase in ("before_pending", "after_pending"):
                    self.assertFalse((folder / "separate-terminals.sqlite3").exists())
                reopened = self.store(path=path, gateway="restart")
                self.assertEqual(reopened.get_terminal("task")["snapshot"], snapshot())
                with CONNECT(str(path)) as db:
                    triggers = db.execute("SELECT sql FROM sqlite_master WHERE type='trigger' AND tbl_name='agent_task_terminals'").fetchall()
                self.assertEqual(len(triggers), 3 if state["layout_state"] == "active" else 0)
                if state["layout_state"] != "combined":
                    self.storage_error(lambda path=path, folder=folder: activate(path, terminal=folder / "wrong-target.sqlite3"))
                if phase == "before_pair":
                    # No committed pair identifies the interrupted empty target.
                    # Explicitly blocked recovery is safer than adopting that file.
                    self.error("agent_terminal_pair_conflict", lambda path=path, folder=folder: activate(
                        path, terminal=folder / "separate-terminals.sqlite3"))
                    self.assertEqual(status(path)["layout_state"], "pending")
                else:
                    activate(path, terminal=folder / "separate-terminals.sqlite3")
                    resumed = status(path)
                    self.assertEqual(resumed["layout_state"], "active")
                    self.assertEqual(resumed["ledger_id"], state["ledger_id"])
                    if state["layout_state"] != "combined":
                        self.assertEqual(resumed["pair_id"], state["pair_id"])
                self.assertEqual(raw_rows(path), [legacy_row()])

    def test_seed_and_candidate_commit_process_exit_reopens_monotonically(self):
        for phase in ("before_seed", "after_seed", "before_candidate", "after_candidate", "after_ack"):
            with self.subTest(phase=phase):
                folder = self.root / phase
                folder.mkdir()
                path = folder / "agent_calls.sqlite3"
                legacy_database(path, [legacy_row()])
                store = self.store(path=path)
                store.begin("request", "tool", "cap", {})
                (folder / "action-count").write_text("1")
                store.finish("request", {"task_id": "task"})
                activate(path, terminal=folder / "separate-terminals.sqlite3")
                self.run_child("save", phase, folder)
                reopened = self.store(path=path, gateway="restarted")
                self.assertFalse(reopened.begin("request", "tool", "cap", {})[1])
                expected = 9 if phase in ("after_candidate", "after_ack") else 8
                self.assertEqual(reopened.terminal_versions(["task"])["task"]["revision"], expected)
                self.assertEqual(reopened.get_terminal("task")["snapshot"], snapshot(revision=expected))
                rows = raw_rows(target_path(path))
                if phase == "before_seed":
                    self.assertEqual(rows, [])
                elif expected == 8:
                    self.assertEqual(rows, [legacy_row()])
                self.assertEqual(self.save(reopened, 9)["revision"], 9)
                self.assertEqual(self.save(reopened, 9)["disposition"], "unchanged")
                self.assertEqual((folder / "action-count").read_text(), "1")
                self.assertEqual(raw_rows(path), [legacy_row()])


def child_exit(folder, phase):
    (folder / "reached").write_text(phase)
    os._exit(EXIT_CODE)


def child(operation, phase, folder):
    folder = Path(folder)
    path = folder / "agent_calls.sqlite3"
    if operation == "intent":
        store = AgentCallStore(path, scope="scope", gateway_instance_id="original")
        assert store.begin("request", "tool", "cap", {})[1]
        if phase == "after_begin":
            child_exit(folder, phase)
        (folder / "action-count").write_text("1")
        if phase == "after_action":
            child_exit(folder, phase)
        if phase == "after_unknown":
            store.mark_unknown("request")
            child_exit(folder, phase)
        store.finish("request", {"task_id": "task"})
        child_exit(folder, phase)
    terminal = folder / "separate-terminals.sqlite3"
    commits = {"terminal": 0}
    def point(db, timing):
        if not db.sql:
            return
        sql = "\n".join(db.sql).upper()
        if operation == "cutover":
            if db.role == "terminal":
                kind = "pair"
            elif "CREATE TRIGGER" in sql:
                kind = "activation"
            elif "UPDATE AGENT_TERMINAL_LAYOUT" in sql and "LAYOUT_STATE='PENDING'" in sql:
                kind = "pending"
            else:
                kind = None
        else:
            changed = "INSERT INTO AGENT_TASK_TERMINALS" in sql or "UPDATE AGENT_TASK_TERMINALS" in sql
            kind = ("seed" if commits["terminal"] == 0 else "candidate") if db.role == "terminal" and changed else None
        if kind and phase == timing + "_" + kind:
            child_exit(folder, phase)
        if timing == "after" and db.role == "terminal" and kind:
            commits["terminal"] += 1
    # Construct before instrumenting save, so constructor verification is not a candidate.
    store = AgentCallStore(path, scope="scope", gateway_instance_id="writer") if operation == "save" else None
    with observed_connections(path, terminal, before_commit=lambda db: point(db, "before"),
                              after_commit=lambda db: point(db, "after")):
        if operation == "cutover":
            activate(path, terminal=terminal)
        else:
            store.save_terminal("task", snapshot(revision=9), source_id="source", revision=9)
            if phase == "after_ack":
                child_exit(folder, phase)
    raise AssertionError("requested child process-exit point was not reached: " + phase)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--child":
        child(*sys.argv[2:])
    else:
        unittest.main(verbosity=2)
