#!/usr/bin/env python3
"""离线验证任务调用身份投影不读取历史载荷，完整记录与作用域合同不变。"""
from __future__ import annotations

import asyncio
import json
import sqlite3
import sys
import tempfile
import threading
import unittest
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import httpx
from fastapi import FastAPI

from contracts.building_navigation import BuildingExplorationRequest, FloorRecord
from core.agent_call_store import AgentCallStore
from tests.conftest import identity_call_store, insert_historical_scope_binding, create_isolated_gateway
from core.errors import GatewayError
from core.event_bus import EventBus
from core.task_manager import TaskManager
from gateways.agent.feedback import checked_json
from gateways.agent.router import build_agent_router
from gateways.agent.service import AgentToolService
from gateways.runtime import GatewayRuntime


@contextmanager
def forbid_historical_payload_columns(store):
    """由 SQLite 授权器证明未选取载荷列，而非只统计 Python 解码调用。"""
    connection = store._connection
    reads = []

    def authorize(action, table, column, database, trigger):
        if action == sqlite3.SQLITE_READ and table == "agent_calls":
            reads.append(column)
            if column in {"response_json", "error_json", "arguments_json", "fingerprint"}:
                return sqlite3.SQLITE_DENY
        return sqlite3.SQLITE_OK

    @contextmanager
    def guarded_connection():
        with connection() as db:
            db.set_authorizer(authorize)
            try:
                yield db
            finally:
                db.set_authorizer(None)

    with patch.object(store, "_connection", guarded_connection):
        yield reads


class TaskCallIdentityStoreTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="neh-call-identity-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root / "calls.sqlite3"
        self.store = identity_call_store(self.path, identity_name='scope-a', gateway_instance_id='gateway-a')

    def record(self, request_id="request-a", *, store=None, task_id="task-a", tool="tool-a", error=None):
        store = store or self.store
        response = {"task_id": task_id, "result": {"values": list(range(1024))}}
        store.begin(request_id, tool, "offline_task", {"arguments": list(range(1024))})
        store.finish(request_id, response, error)
        return response

    def raw_rows(self):
        with sqlite3.connect(self.path) as db:
            return db.execute("SELECT * FROM agent_calls ORDER BY request_id").fetchall()

    def test_identity_reads_no_payload_columns_or_decoder(self):
        self.record(error={"message": "recorded error"})
        before = self.raw_rows()
        with forbid_historical_payload_columns(self.store) as reads, patch.object(
                self.store, "_decode", side_effect=AssertionError("完整记录解码不应运行")):
            first = self.store.find_task_call_identity("task-a")
            self.assertEqual(first, {"request_id": "request-a", "tool_name": "tool-a"})
            first["tool_name"] = "caller mutation"
            self.assertEqual(self.store.find_task_call_identity("task-a")["tool_name"], "tool-a")
        self.assertIn("request_id", reads)
        self.assertIn("tool_name", reads)
        self.assertTrue(set(reads) <= {"request_id", "tool_name", "task_id", "scope", "created_at"})
        self.assertEqual(self.raw_rows(), before)

    def test_earliest_and_tied_records_match_unchanged_full_lookup(self):
        for request_id in ("second", "first", "tie"):
            self.record(request_id, tool=request_id)
        with sqlite3.connect(self.path) as db:
            db.execute("UPDATE agent_calls SET created_at='2026-10-07T01:00:00Z'")
            db.execute("UPDATE agent_calls SET created_at='2026-10-07T00:00:00Z' WHERE request_id='first'")
        self.assertEqual(self.store.find_task_call_identity("task-a")["request_id"], "first")
        with sqlite3.connect(self.path) as db:
            db.execute("UPDATE agent_calls SET created_at='2026-10-07T00:00:00Z' WHERE request_id='tie'")
        expected = self.store.find_by_task("task-a")
        self.assertEqual(self.store.find_task_call_identity("task-a"),
                         {key: expected[key] for key in ("request_id", "tool_name")})

    def test_missing_and_foreign_scopes_are_not_returned(self):
        other = identity_call_store(self.path, identity_name='scope-b', gateway_instance_id='gateway-b')
        self.record(store=other)
        self.assertIsNone(self.store.find_task_call_identity("task-a"))
        self.assertIsNone(self.store.find_task_call_identity("missing"))
        self.assertIsNone(self.store.find_by_task("task-a"))

    def test_existing_legacy_binding_and_restart_preserve_identity(self):
        self.record()
        identity = {"robot_model": "offline", "robot_name": "robot", "robot_id": "one"}
        options = {"robot_identity": identity, "artifact_dir": self.root / "artifacts"}
        registered = AgentCallStore.for_identity(self.path, gateway_instance_id="current", **options)
        self.assertIsNone(registered.find_task_call_identity("task-a"))
        insert_historical_scope_binding(self.path, legacy_scope=self.store.scope, owner_scope=registered.scope)
        before = self.raw_rows()
        for gateway in ("current", "restarted"):
            reader = AgentCallStore.for_identity(self.path, gateway_instance_id=gateway, **options)
            with forbid_historical_payload_columns(reader):
                self.assertEqual(reader.find_task_call_identity("task-a"),
                                 {"request_id": "request-a", "tool_name": "tool-a"})
        unrelated = AgentCallStore.for_identity(self.path, gateway_instance_id="foreign",
            robot_identity={**identity, "robot_id": "two"}, artifact_dir=self.root / "artifacts")
        self.assertIsNone(unrelated.find_task_call_identity("task-a"))
        self.assertEqual(self.raw_rows(), before)

    def test_full_record_apis_retain_complete_response_and_error(self):
        error = {"message": "historical", "details": {"values": [1, 2, 3]}}
        response = self.record(error=error)
        before = self.store.get("request-a")
        self.store.find_task_call_identity("task-a")
        self.assertEqual(self.store.find_by_task("task-a"), before)
        self.assertEqual(before["response"], response)
        self.assertEqual(before["error"], error)
        self.assertNotIn("arguments_json", before)
        self.assertNotIn("fingerprint", before)

    def test_bound_and_current_scopes_share_ordering_without_foreign_rows(self):
        self.record("bound")
        options = {"robot_identity": {"robot_model": "offline", "robot_id": "one"},
                   "artifact_dir": self.root / "artifacts"}
        registered = AgentCallStore.for_identity(self.path, gateway_instance_id="current", **options)
        self.record("current", store=registered)
        foreign = identity_call_store(self.path, identity_name='foreign', gateway_instance_id='foreign')
        self.record("foreign", store=foreign)
        insert_historical_scope_binding(self.path, legacy_scope=self.store.scope, owner_scope=registered.scope)
        with sqlite3.connect(self.path) as db:
            db.execute("UPDATE agent_calls SET created_at='2026-10-07T01:00:00Z'")
            db.execute("UPDATE agent_calls SET created_at='2026-10-07T00:00:00Z' WHERE request_id='foreign'")
        expected = registered.find_by_task("task-a")
        self.assertNotEqual(expected["request_id"], "foreign")
        self.assertEqual(registered.find_task_call_identity("task-a"),
                         {key: expected[key] for key in ("request_id", "tool_name")})

    def test_unused_malformed_json_does_not_block_identity_but_full_reads_still_fail(self):
        self.record(error={"message": "historical"})
        for column in ("response_json", "error_json"):
            with self.subTest(column=column), sqlite3.connect(self.path) as db:
                original = db.execute(f"SELECT {column} FROM agent_calls").fetchone()[0]
                db.execute(f"UPDATE agent_calls SET {column}=?", ("{broken",))
                db.commit()
                self.assertEqual(self.store.find_task_call_identity("task-a"),
                                 {"request_id": "request-a", "tool_name": "tool-a"})
                for operation in (lambda: self.store.get("request-a"), lambda: self.store.find_by_task("task-a")):
                    with self.assertRaises(json.JSONDecodeError):
                        operation()
                db.execute(f"UPDATE agent_calls SET {column}=?", (original,))


class TaskCallIdentityEndpointTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="neh-call-identity-http-")
        self.bus = EventBus()
        self.manager = TaskManager(event_bus=self.bus)
        self.release, self.entered = threading.Event(), threading.Event()
        self.actions = []
        self.allowed = True
        self.authorization_reads = 0

        def action(context):
            self.actions.append(1)
            self.entered.set()
            if not self.release.wait(10):
                raise AssertionError("离线任务门闩超时")
            return {"known": True}

        request = BuildingExplorationRequest(request_id="building-request", building_id="campus",
            topology_revision=1, floors=tuple(FloorRecord(floor_id=f"floor-{i}",
                display_name=f"楼层 {i}", map_name=f"campus-{i}") for i in range(128)))
        self.spec = self.manager.submit("explore_building", action,
            input_payload=request.model_dump(mode="json"), metadata={"request_id": "fallback-request"})
        self.task_id = self.spec.task_id
        self.assertTrue(self.entered.wait(2))
        runtime = create_isolated_gateway(Path(self.temp.name), event_bus=self.bus, task_manager=self.manager)
        runtime.capability_registry = NS(get_descriptor=lambda name: NS(display_name="offline"))

        def authorize(descriptor):
            self.assertFalse(self.manager._lock._is_owned())
            self.authorization_reads += 1
            if not self.allowed:
                raise GatewayError("拒绝", error_code="access_denied", http_status=403)

        self.service = AgentToolService(runtime, NS(authorize_capability=authorize))
        self.acceptance = checked_json({"request_id": "recorded-request", "task_id": self.task_id,
            "capability_name": "explore_building", "mode": "async_task", "task": self.spec,
            "status": self.manager.get_task_status(self.task_id), "feedback": None})
        self.service.store.begin("recorded-request", "explore_building", "explore_building", {"building_id": "campus"})
        self.service.store.finish("recorded-request", self.acceptance)
        self.app = FastAPI()
        self.app.include_router(build_agent_router(self.service))

    def tearDown(self):
        self.release.set()
        self.manager.shutdown(wait=True)
        closed = self.service.close(2, source_fenced=True)
        self.assertTrue(closed["owner_closed"], closed)
        self.service.runtime.stop()
        self.assertTrue(self.bus.shutdown(2))
        self.assertEqual(self.actions, [1])
        self.temp.cleanup()

    def get(self, suffix=""):
        async def request():
            async with httpx.AsyncClient(transport=httpx.ASGITransport(app=self.app),
                                         base_url="http://offline") as client:
                return await client.get(f"/api/agent/tasks/{self.task_id}{suffix}")
        return asyncio.run(request())

    def test_public_live_receipt_matches_full_lookup_without_reading_historical_payload(self):
        with patch.object(self.service.store, "find_task_call_identity", self.service.store.find_by_task):
            expected = self.get()
        self.assertEqual(expected.status_code, 200)
        before = self.authorization_reads
        with forbid_historical_payload_columns(self.service.store), patch.object(
                self.service.store, "_decode", side_effect=AssertionError("不得解码旧受理载荷")):
            actual = self.get()
        self.assertEqual(actual.status_code, expected.status_code)
        self.assertEqual(actual.headers, expected.headers)
        self.assertEqual(actual.content, expected.content)
        self.assertGreater(self.authorization_reads, before)
        self.assertEqual(actual.json()["status_source"], "live_task")
        self.assertEqual(actual.json()["status"]["state"], "running")
        self.assertEqual(actual.json()["request_id"], "recorded-request")

    def test_repeated_receipt_reads_do_not_decode_recorded_acceptance(self):
        loads = json.loads
        historical = []
        with sqlite3.connect(self.service.store.path) as db:
            encoded = db.execute("SELECT response_json FROM agent_calls").fetchone()[0]

        def counted(value, *args, **kwargs):
            if value == encoded:
                historical.append(len(value.encode("utf-8")))
            return loads(value, *args, **kwargs)

        snapshot = self.service._task_snapshot(self.task_id)
        with patch("core.agent_call_store.json.loads", side_effect=counted):
            receipts = [self.service._task_receipt(snapshot) for _ in range(3)]
        self.assertEqual(historical, [])
        self.assertEqual(receipts[0], receipts[1])
        self.assertEqual(receipts[1], receipts[2])
        self.assertGreater(len(encoded), 10_000)

    def test_access_denial_precedes_identity_lookup(self):
        self.allowed = False
        with patch.object(self.service.store, "find_task_call_identity",
                          side_effect=AssertionError("未授权请求不应读取调用身份")) as lookup:
            response = self.get()
        self.assertEqual(response.status_code, 403)
        self.assertEqual(response.json()["error"]["code"], "access_denied")
        lookup.assert_not_called()

    def test_missing_record_uses_live_spec_fallback(self):
        with sqlite3.connect(self.service.store.path) as db:
            db.execute("DELETE FROM agent_calls")
        response = self.get()
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["request_id"], "fallback-request")
        self.assertIsNone(response.json()["tool_name"])

    def test_unused_malformed_acceptance_does_not_replace_live_status(self):
        with sqlite3.connect(self.service.store.path) as db:
            db.execute("UPDATE agent_calls SET response_json='{broken',error_json='{broken'")
        response = self.get()
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["status"]["state"], "running")
        self.assertEqual(response.json()["request_id"], "recorded-request")
        with self.assertRaises(json.JSONDecodeError):
            self.service.store.get("recorded-request")


if __name__ == "__main__":
    unittest.main()
