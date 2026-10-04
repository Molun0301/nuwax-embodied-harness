#!/usr/bin/env python3
"""离线验证调用参数直比、显式旧作用域绑定和重启不重放；仅用临时 SQLite。"""
from __future__ import annotations

import json
import sqlite3
import sys
import tempfile
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.agent_call_store import AgentCallStore
from core.errors import GatewayError
from core.event_bus import EventBus
from gateways.agent.service import AgentToolService
from gateways.feedback import build_public_error_payload


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)


class AgentScopeChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="neh-agent-scope-")
        self.root = Path(self.temp.name)
        self.path = self.root / "agent_calls.sqlite3"
        self.artifacts = self.root / "artifacts"
        self.identity = {"robot_model": "offline", "robot_name": "same-name", "entrypoint": "fake", "robot_id": "one"}

    def tearDown(self):
        self.temp.cleanup()

    def store(self, *, identity=None, gateway="new-gateway", directory=None):
        return AgentCallStore.for_identity(self.path, robot_identity=identity or self.identity,
                                          artifact_dir=directory or self.artifacts, gateway_instance_id=gateway)

    def legacy(self, scope="old-opaque-token", request="old-request", *, state="recorded", arguments=None):
        args = {"n": 1} if arguments is None else arguments
        with sqlite3.connect(str(self.path)) as db:
            db.execute("CREATE TABLE IF NOT EXISTS agent_calls (request_id TEXT PRIMARY KEY,scope TEXT NOT NULL,gateway_instance_id TEXT NOT NULL,tool_name TEXT NOT NULL,capability_name TEXT NOT NULL,arguments_json TEXT NOT NULL,fingerprint TEXT NOT NULL,delivery_state TEXT NOT NULL,response_json TEXT,error_json TEXT,task_id TEXT,created_at TEXT NOT NULL,updated_at TEXT NOT NULL)")
            db.execute("CREATE TABLE IF NOT EXISTS agent_task_terminals (task_id TEXT PRIMARY KEY,scope TEXT NOT NULL,gateway_instance_id TEXT NOT NULL,snapshot_json TEXT NOT NULL,updated_at TEXT NOT NULL)")
            db.execute("INSERT INTO agent_calls VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)", (
                request, scope, "old-gateway", "offline_tool", "offline_capability", canonical(args),
                "unread-historical-derived-value", state, canonical({"task_id": request + "-task", "ok": True}) if state == "recorded" else None,
                None, request + "-task", "old-created", "old-updated"))
            db.execute("INSERT INTO agent_task_terminals VALUES (?,?,?,?,?)", (
                request + "-task", scope, "old-gateway", canonical({"task_id": request + "-task", "status": {"state": "succeeded"}}), "old-updated"))

    def bind(self, scope="old-opaque-token", *, identity=None):
        return AgentCallStore.bind_verified_legacy_scope(self.path, legacy_scope=scope,
                                                         robot_identity=identity or self.identity, artifact_dir=self.artifacts)

    def error(self, code, operation):
        with self.assertRaises(GatewayError) as caught:
            operation()
        self.assertEqual(caught.exception.error_code, code)
        return caught.exception

    def rows(self):
        with sqlite3.connect(str(self.path)) as db:
            return db.execute("SELECT * FROM agent_calls ORDER BY request_id").fetchall()

    def test_readonly_discovery_does_not_create_registry_or_guess_single_scope(self):
        self.legacy()
        before = self.rows()
        self.assertEqual(AgentCallStore.list_legacy_scopes(self.path), ("old-opaque-token",))
        with sqlite3.connect(str(self.path)) as db:
            self.assertIsNone(db.execute("SELECT name FROM sqlite_master WHERE name='agent_scope_owners'").fetchone())
        store = self.store()
        self.error("agent_scope_binding_required", lambda: store.get("old-request"))
        self.error("agent_scope_binding_required", lambda: store.get_terminal("old-request-task"))
        self.error("agent_scope_binding_required", lambda: store.begin("old-request", "offline_tool", "offline_capability", {"n": 1}))
        self.assertEqual(self.rows(), before)
        self.assertEqual(AgentCallStore.list_legacy_scopes(self.path), ("old-opaque-token",))

    def test_readonly_discovery_never_creates_missing_database(self):
        with self.assertRaises(sqlite3.OperationalError):
            AgentCallStore.list_legacy_scopes(self.path)
        self.assertFalse(self.path.exists())

    def test_bound_old_rows_preserve_exact_fields_and_never_acquire_again(self):
        self.legacy()
        before = self.rows()
        owner = self.bind()
        self.assertEqual(self.bind(), owner)
        for gateway in ("old-gateway", "restarted-gateway"):
            store = self.store(gateway=gateway)
            self.assertEqual(store.scope, owner)
            record, acquired = store.begin("old-request", "offline_tool", "changed-catalog-capability", {"n": 1})
            self.assertFalse(acquired)
            self.assertEqual(record["scope"], "old-opaque-token")
            self.assertEqual(record["capability_name"], "offline_capability")
            self.assertEqual(store.find_by_task("old-request-task")["request_id"], "old-request")
            self.assertEqual(store.get_terminal("old-request-task")["gateway_instance_id"], "old-gateway")
        self.assertEqual(self.rows(), before)
        self.assertEqual(AgentCallStore.list_legacy_scopes(self.path), ())

    def test_pending_and_unknown_never_reset_or_gain_new_owner(self):
        self.legacy(request="old-pending", state="pending")
        self.legacy(request="old-unknown", state="unknown")
        before = self.rows()
        self.bind()
        for request in ("old-pending", "old-unknown"):
            store = self.store()
            record, acquired = store.begin(request, "offline_tool", "offline_capability", {"n": 1})
            self.assertFalse(acquired)
            self.assertEqual(record["delivery_state"], "unknown")
            self.error("agent_call_outcome_unknown", lambda store=store, request=request: store.finish(request, {"ok": True}))
        self.assertEqual(self.store(gateway="old-gateway").get("old-pending")["delivery_state"], "pending")
        self.assertEqual(self.rows(), before)

    def test_tool_and_canonical_argument_equality_preserve_distinctions(self):
        store = self.store()
        record, acquired = store.begin("new-request", "tool", "cap", {"b": 2, "a": 1})
        actions = [1] if acquired else []
        store.finish("new-request", {"ok": True})
        again, acquired = store.begin("new-request", "tool", "changed-cap", {"a": 1, "b": 2})
        if acquired:
            actions.append(1)
        self.assertEqual(again["capability_name"], record["capability_name"])
        self.assertEqual(actions, [1])
        for tool, args in (("other", {"a": 1, "b": 2}), ("tool", {"a": 1.0, "b": 2}), ("tool", {"a": True, "b": 2})):
            self.error("agent_request_conflict", lambda tool=tool, args=args: store.begin("new-request", tool, "cap", args))
        with sqlite3.connect(str(self.path)) as db:
            self.assertEqual(db.execute("SELECT fingerprint FROM agent_calls").fetchone(), ("",))

    def test_legacy_arguments_do_not_depend_on_historical_derived_column(self):
        self.legacy()
        self.bind()
        store = self.store()
        _, acquired = store.begin("old-request", "offline_tool", "cap", {"n": 1})
        self.assertFalse(acquired)
        self.error("agent_request_conflict", lambda: store.begin("old-request", "offline_tool", "cap", {"n": 1.0}))
        with sqlite3.connect(str(self.path)) as db:
            self.assertEqual(db.execute("SELECT fingerprint FROM agent_calls").fetchone()[0], "unread-historical-derived-value")

    def test_exact_identity_and_resolved_directory_registry(self):
        first = self.store()
        self.assertEqual(first.scope, self.store(identity=dict(reversed(tuple(self.identity.items())))).scope)
        self.assertEqual(first.scope, self.store(directory=self.artifacts / ".." / "artifacts").scope)
        self.assertNotEqual(first.scope, self.store(identity=dict(self.identity, robot_id="two")).scope)
        self.assertNotEqual(first.scope, self.store(directory=self.root / "another-artifacts").scope)
        with sqlite3.connect(str(self.path)) as db:
            actual = db.execute("SELECT identity_json FROM agent_scope_owners WHERE scope_id=?", (first.scope,)).fetchone()[0]
        self.assertEqual(actual, json.dumps([self.identity, str(self.artifacts.resolve())], sort_keys=True))

    def test_concurrent_registry_and_request_admission_execute_once(self):
        barrier = threading.Barrier(4)
        def begin(index):
            barrier.wait(2)
            store = self.store(gateway="gateway-" + str(index))
            return store.scope, store.begin("one-request", "tool", "cap", {"v": 1})[1]
        with ThreadPoolExecutor(max_workers=4) as pool:
            results = list(pool.map(begin, range(4)))
        self.assertEqual(len({scope for scope, _ in results}), 1)
        self.assertEqual(sum(owner for _, owner in results), 1)

    def test_alias_cannot_be_reassigned_to_other_robot(self):
        self.legacy()
        owner = self.bind()
        self.error("agent_scope_binding_conflict", lambda: self.bind(identity=dict(self.identity, robot_id="other")))
        with sqlite3.connect(str(self.path)) as db:
            self.assertEqual(db.execute("SELECT COUNT(*) FROM agent_scope_owners").fetchone()[0], 1)
            self.assertEqual(db.execute("SELECT owner_scope FROM agent_scope_bindings WHERE scope=?", ("old-opaque-token",)).fetchone()[0], owner)
        wrong = self.store(identity=dict(self.identity, robot_id="other"))
        self.error("not_found", lambda: wrong.get("old-request"))
        self.error("agent_request_conflict", lambda: wrong.begin("old-request", "offline_tool", "cap", {"n": 1}))

    def test_multiple_verified_aliases_keep_original_row_tokens(self):
        self.legacy(scope="old-a", request="a")
        self.legacy(scope="old-b", request="b")
        before = self.rows()
        self.assertEqual(AgentCallStore.list_legacy_scopes(self.path), ("old-a", "old-b"))
        self.assertEqual(self.bind("old-a"), self.bind("old-b"))
        self.assertEqual(self.store().get("a")["scope"], "old-a")
        self.assertEqual(self.store().get("b")["scope"], "old-b")
        self.assertEqual(self.rows(), before)

    def test_terminal_revision_updates_preserve_bound_alias_and_source(self):
        legacy = AgentCallStore(self.path, scope="old-a", gateway_instance_id="old-gateway")
        snapshot = {"task_id": "task-a", "status": {"state": "cancelled"}, "result": None}
        legacy.save_terminal("task-a", snapshot, source_id="source-a", revision=1)
        self.bind("old-a")
        store = self.store()
        new = dict(snapshot, result={"known": True})
        self.assertEqual(store.save_terminal("task-a", new, source_id="source-a", revision=2)["disposition"], "updated")
        self.assertEqual(store.save_terminal("task-a", snapshot, source_id="source-a", revision=1)["disposition"], "stale")
        with sqlite3.connect(AgentCallStore.terminal_storage_status(self.path)["terminal_path"]) as db:
            self.assertEqual(db.execute("SELECT scope,gateway_instance_id,receipt_source_id,receipt_revision FROM agent_task_terminals").fetchone(), ("old-a", "old-gateway", "source-a", 2))

    def test_generated_scope_collision_never_adopts_other_owner(self):
        first = self.store()
        other_identity = dict(self.identity, robot_id="two")
        with patch("core.agent_call_store.uuid4", side_effect=(NS(hex=first.scope[6:]), NS(hex="fresh-assigned-id"))):
            other = self.store(identity=other_identity)
        self.assertEqual(other.scope, "scope_fresh-assigned-id")
        with patch("core.agent_call_store.uuid4", return_value=NS(hex=first.scope[6:])):
            self.error("agent_scope_identity_conflict", lambda: self.store(identity=dict(self.identity, robot_id="three")))
        with sqlite3.connect(str(self.path)) as db:
            self.assertEqual(db.execute("SELECT COUNT(*) FROM agent_scope_owners").fetchone()[0], 2)

    def test_binding_transaction_rolls_back_on_storage_error(self):
        self.legacy()
        AgentCallStore(self.path, scope="unbound-maintenance", gateway_instance_id="maintenance")
        with sqlite3.connect(str(self.path)) as db:
            db.execute("CREATE TRIGGER deny_alias BEFORE INSERT ON agent_scope_bindings WHEN NEW.scope='old-opaque-token' BEGIN SELECT RAISE(ABORT,'synthetic storage error'); END")
        with self.assertRaises(sqlite3.IntegrityError):
            self.bind()
        with sqlite3.connect(str(self.path)) as db:
            self.assertEqual(db.execute("SELECT COUNT(*) FROM agent_scope_owners").fetchone()[0], 0)
            self.assertEqual(db.execute("SELECT COUNT(*) FROM agent_scope_bindings").fetchone()[0], 0)
        self.assertEqual(AgentCallStore.list_legacy_scopes(self.path), ("old-opaque-token",))

    def test_concurrent_alias_binding_has_one_verified_owner(self):
        self.legacy()
        AgentCallStore(self.path, scope="maintenance", gateway_instance_id="maintenance")
        barrier = threading.Barrier(2)
        def bind(robot_id):
            barrier.wait(2)
            try:
                return self.bind(identity=dict(self.identity, robot_id=robot_id)), None
            except GatewayError as error:
                return None, error.error_code
        with ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(bind, ("one", "two")))
        self.assertEqual(sum(owner is not None for owner, _ in results), 1)
        self.assertEqual([error for _, error in results if error], ["agent_scope_binding_conflict"])
        with sqlite3.connect(str(self.path)) as db:
            self.assertEqual(db.execute("SELECT COUNT(*) FROM agent_scope_owners").fetchone()[0], 1)

    def test_service_legacy_binding_recovers_known_request_without_invocation(self):
        bus = EventBus()
        calls = []
        legacy = AgentCallStore(self.path, scope="legacy-exact-scope", gateway_instance_id="old-gateway")
        _, owner = legacy.begin("legacy-service", "offline_tool", "offline_capability", {})
        if owner:
            calls.append(1)
        legacy.finish("legacy-service", {"ok": True, "result": {"known": True}})
        runtime = NS(robot=NS(manifest=NS(robot_model="offline", robot_name="same-name", entrypoint="fake"),
                              defaults=NS(parameters={"robot_id": "one"})),
                     config=NS(gateway=NS(artifact_dir=self.artifacts)), event_bus=bus,
                     capability_registry=NS(get_descriptor=lambda name: NS(name=name)))
        runtime.list_tools = lambda **kwargs: []
        runtime.invoke_tool = lambda *args, **kwargs: calls.append(1)
        access = NS(authorize_capability=lambda descriptor: None)
        service = AgentToolService(runtime, access)
        try:
            self.error("agent_scope_binding_required", lambda: service.invoke("legacy-service", "offline_tool", {}))
            self.bind("legacy-exact-scope")
            first = service.invoke("legacy-service", "offline_tool", {})
            restarted = AgentToolService(runtime, access)
            try:
                self.assertEqual(restarted.invoke("legacy-service", "offline_tool", {}), first)
            finally:
                restarted.close()
            self.assertEqual(calls, [1])
        finally:
            service.close()
            self.assertTrue(bus.shutdown(2))

    def test_binding_errors_public_guidance_never_replays_action(self):
        for code in ("agent_scope_binding_required", "agent_scope_binding_conflict", "agent_scope_identity_conflict"):
            public = build_public_error_payload(GatewayError("原记录保持不变。", error_code=code, http_status=409), request_id="original")
            self.assertEqual(public["error_category"], "receipt")
            self.assertFalse(public["retryable"])
            self.assertEqual(public["next_action"], "由已授权维护方核对原作用域与机器人身份，明确绑定后只读查询原 request_id/task_id。不要更换请求标识重发动作，也不要取消原任务来修复记录。")

    def test_service_restart_same_request_invokes_once_and_keeps_identity(self):
        bus = EventBus()
        calls = []
        runtime = NS(robot=NS(manifest=NS(robot_model="offline", robot_name="same-name", entrypoint="fake"),
                              defaults=NS(parameters={"robot_id": "one"})),
                     config=NS(gateway=NS(artifact_dir=self.artifacts)), event_bus=bus,
                     capability_registry=NS(get_descriptor=lambda name: NS(name=name)))
        runtime.list_tools = lambda **kwargs: [NS(descriptor=NS(name="offline_tool", exposed_to_agent=True), supported=True,
                                                  runnable=True, capability_name="offline_capability", input_schema={"type": "object"})]
        runtime.invoke_tool = lambda *args, **kwargs: calls.append(1) or {"ok": True, "result": {"known": True}}
        access = NS(authorize_capability=lambda descriptor: None)
        first, second = AgentToolService(runtime, access), AgentToolService(runtime, access)
        try:
            self.assertEqual(first.invoke("service-once", "offline_tool", {}), second.invoke("service-once", "offline_tool", {}))
            self.assertEqual(first.store.scope, second.store.scope)
            self.assertEqual(calls, [1])
        finally:
            first.close()
            second.close()
            self.assertTrue(bus.shutdown(2))


if __name__ == "__main__":
    unittest.main(verbosity=2)
