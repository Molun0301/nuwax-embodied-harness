#!/usr/bin/env python3
"""离线检查终态回执迁移、单调写入、投影缓存与同源读取，不重放动作。"""
from __future__ import annotations

import json
import sqlite3
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contracts.capabilities import CapabilityExecutionMode
from contracts.tasks import TaskSpec
from core.agent_call_store import AgentCallStore
from tests.conftest import identity_call_store, insert_historical_scope_binding, create_isolated_gateway
from core.errors import GatewayError
from core.event_bus import EventBus
from core.task_manager import TaskManager
from gateways.agent.service import AgentToolService
from gateways.feedback import build_public_error_payload
from gateways.runtime import GatewayRuntime


class TerminalStoreChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="neh-terminal-cas-")
        self.path = Path(self.temp.name) / "calls.sqlite3"
        self.store = identity_call_store(self.path, identity_name='robot-a', gateway_instance_id='gateway-a')

    def tearDown(self):
        self.temp.cleanup()

    def snapshot(self, value=1):
        return {"task_id": "task-a", "status": {"state": "succeeded"}, "result": {"value": value}}

    def save(self, value, revision, source="source-a", store=None):
        return (store or self.store).save_terminal("task-a", self.snapshot(value), source_id=source, revision=revision)

    def conflict(self, code, operation):
        with self.assertRaises(GatewayError) as caught:
            operation()
        self.assertEqual(caught.exception.error_code, code)

    def test_reverse_order_same_revision_conflict_and_idempotency(self):
        self.assertEqual(self.save(2, 2)["disposition"], "inserted")
        before = self.store.get_terminal("task-a")
        self.assertEqual(self.save(1, 1), {"disposition": "stale", "source_id": "source-a", "revision": 2})
        self.assertEqual(self.save(2, 2)["disposition"], "unchanged")
        self.conflict("agent_terminal_revision_conflict", lambda: self.save(9, 2))
        self.assertEqual(self.store.get_terminal("task-a"), before)
        self.assertEqual(self.save(3, 3)["disposition"], "updated")
        self.assertEqual(self.store.get_terminal("task-a")["snapshot"]["result"], {"value": 3})

    def test_scope_source_and_missing_version_are_explicit(self):
        self.save(1, 1)
        self.conflict("agent_terminal_source_conflict", lambda: self.save(2, 2, source="other-source"))
        other = identity_call_store(self.path, identity_name='robot-b', gateway_instance_id='gateway-a')
        self.conflict("agent_terminal_scope_conflict", lambda: self.save(2, 2, store=other))
        self.conflict("agent_terminal_version_required", lambda: self.store.save_terminal("task-a", self.snapshot()))
        self.conflict("agent_terminal_identity_conflict", lambda: self.store.save_terminal("other", self.snapshot(), source_id="source-a", revision=2))
        self.assertEqual(self.store.get_terminal("task-a")["snapshot"]["result"], {"value": 1})

    def test_two_store_readers_preserve_initial_gateway_identity(self):
        self.save(1, 1)
        other = identity_call_store(self.path, identity_name='robot-a', gateway_instance_id='gateway-b')
        self.assertEqual(self.save(1, 1, store=other)["disposition"], "unchanged")
        self.assertEqual(self.save(2, 2, store=other)["disposition"], "updated")
        self.assertEqual(other.get_terminal("task-a")["gateway_instance_id"], "gateway-a")

    def test_legacy_rows_readable_but_never_adopted(self):
        legacy_path = Path(self.temp.name) / "legacy.sqlite3"
        with sqlite3.connect(str(legacy_path)) as db:
            db.execute("CREATE TABLE agent_task_terminals (task_id TEXT PRIMARY KEY, scope TEXT NOT NULL, gateway_instance_id TEXT NOT NULL, snapshot_json TEXT NOT NULL, updated_at TEXT NOT NULL)")
            db.execute("INSERT INTO agent_task_terminals VALUES (?,?,?,?,?)", ("task-a", "robot-a", "gateway-a", json.dumps(self.snapshot(9)), "legacy-time"))
        for gateway in ("gateway-a", "restarted-gateway"):
            store = identity_call_store(legacy_path, identity_name='robot-a', gateway_instance_id=gateway)
            insert_historical_scope_binding(legacy_path, legacy_scope="robot-a", owner_scope=store.scope)
            before = store.get_terminal("task-a")
            self.assertEqual(before["snapshot"]["result"], {"value": 9})
            self.conflict("agent_terminal_legacy_conflict", lambda store=store: self.save(10, 1, store=store))
            self.assertEqual(store.get_terminal("task-a"), before)
            self.assertEqual(before["gateway_instance_id"], "gateway-a")
        with sqlite3.connect(str(legacy_path)) as db:
            row = db.execute("SELECT receipt_source_id, receipt_revision FROM agent_task_terminals").fetchone()
            self.assertEqual(row, (None, None))

    def test_request_intent_idempotency_and_reopen_unchanged(self):
        _, owner = self.store.begin("request-a", "tool-a", "activate_map", {"map": "offline"})
        actions = [1] if owner else []
        self.store.finish("request-a", {"task_id": "task-a"})
        _, owner = self.store.begin("request-a", "tool-a", "activate_map", {"map": "offline"})
        if owner:
            actions.append(1)
        self.assertEqual(actions, [1])
        self.store.begin("pending", "tool-a", "activate_map", {})
        reopened = identity_call_store(self.path, identity_name='robot-a', gateway_instance_id='gateway-b')
        record, owner = reopened.begin("pending", "tool-a", "activate_map", {})
        self.assertFalse(owner)
        self.assertEqual(record["delivery_state"], "unknown")
        self.assertEqual(reopened.get("request-a")["response"], {"task_id": "task-a"})


class PublicReceiptErrorChecks(unittest.TestCase):
    def test_new_receipt_errors_never_claim_action_failed_or_invite_replay(self):
        codes = ("task_receipt_unsupported", "task_receipt_unavailable", "agent_call_result_unsupported",
                 "agent_terminal_version_required",
                 "agent_terminal_identity_conflict", "agent_terminal_scope_conflict", "agent_terminal_legacy_conflict",
                 "agent_terminal_source_conflict", "agent_terminal_revision_conflict")
        message = "回执需要核对，原任务状态没有改变。"
        next_action = ("只读核对原 task_id 的当前状态和历史回执。修复回执格式、版本或存储后，只重试读取或回执写入；"
                       "不要重发动作，也不要取消原任务来修复记录。")
        for code in codes:
            error = GatewayError(message, error_code=code, http_status=500 if code.startswith("task_") else 409,
                                 details={"task_outcome_unchanged": True, "action_replay_allowed": False})
            actual = build_public_error_payload(error, task_id="task-known", error_category="internal")
            self.assertEqual(actual, {"code": code, "message": message, "error_category": "receipt", "retryable": False,
                                      "next_action": next_action, "task_id": "task-known",
                                      "details": {"task_outcome_unchanged": True, "action_replay_allowed": False}})

    def test_unknown_call_receipt_guidance_only_queries_original_identity(self):
        error = GatewayError("调用结果未知，请核对原任务与设备状态，不要自动重发。",
                             error_code="agent_call_outcome_unknown", http_status=409)
        actual = build_public_error_payload(error, request_id="known-request")
        self.assertEqual(actual["error_category"], "receipt")
        self.assertFalse(actual["retryable"])
        self.assertEqual(actual["next_action"],
                         "先用原 request_id 查询调用记录；若已有 task_id，再只读核对原任务与设备状态。"
                         "回执未知不代表动作未执行，不要重发动作或以新请求标识重试。")
        self.assertEqual(actual["request_id"], "known-request")

    def test_existing_motion_domain_timeout_projection_unchanged(self):
        cases = (
            ("resource_conflict", 409, "capability", True, "原始领域消息",
             "查询并等待占用任务结束；必要时取消冲突任务，再重试。"),
            ("timeout", 504, "timeout", True, "原始领域消息",
             "先查询任务与后端状态；确认没有仍在执行的动作后再决定是否重试。"),
            ("parking_unconfirmed", 409, "validation", False, "原始领域消息",
             "使用 request_id 在详细日志中定位根因，修正条件后再调用。"),
            ("execution_failed", 500, "capability", False, "工具执行失败，未完成请求。",
             "使用 request_id 在详细日志中定位根因，修正条件后再调用。"),
            ("mcp_result_unavailable", 500, "internal", False, "无法完整返回MCP数据；工具可能已经执行，请勿自动重复执行操作。",
             "先核对任务/操作状态；只读查询可缩小范围，已有完整结果或制品地址可单独读取。不要自动重复执行操作。"),
        )
        for code, status, category, retryable, message, guidance in cases:
            actual = build_public_error_payload(GatewayError("原始领域消息", error_code=code, http_status=status), task_id="task-known")
            self.assertEqual(actual, {"code": code, "message": message, "error_category": category,
                                      "retryable": retryable, "next_action": guidance, "task_id": "task-known"})


class AgentReceiptChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="neh-agent-receipt-")
        self.bus = EventBus()
        self.manager = TaskManager(event_bus=self.bus)
        self.runtime = create_isolated_gateway(Path(self.temp.name), event_bus=self.bus, task_manager=self.manager)
        self.label = "标签 A"
        def descriptor(name):
            self.assertFalse(self.manager._lock._is_owned())
            return NS(display_name=self.label)
        self.runtime.capability_registry = NS(get_descriptor=descriptor)
        self.authorization_reads = 0
        self.allowed = True
        def authorize(descriptor):
            self.assertFalse(self.manager._lock._is_owned())
            self.authorization_reads += 1
            if not self.allowed:
                raise GatewayError("拒绝", error_code="forbidden", http_status=403)
        self.access = NS(authorize_capability=authorize)
        self.service = AgentToolService(self.runtime, self.access)
        self.services = [self.service]
        self.releases = []
        self.actions = []

    def tearDown(self):
        for release in self.releases:
            release.set()
        self.manager.shutdown(wait=True)
        for service in self.services:
            service.close(source_fenced=True)
        self.runtime.stop()
        self.assertTrue(self.bus.shutdown(2))
        self.temp.cleanup()

    def require(self, event):
        self.assertTrue(event.wait(2), "离线门闩超时")

    def finish(self, value=None):
        task_id = self.manager.submit("activate_map", lambda context: self.actions.append(1) or value).task_id
        self.manager.wait_for_task(task_id, timeout=2)
        return task_id

    def test_unchanged_read_cache_owned_and_authorized_every_time(self):
        task_id = self.finish({"nested": {"value": 1}})
        first = self.service.get_task_result(task_id)
        first["result"]["nested"]["value"] = 9
        before = self.authorization_reads
        with patch.object(self.runtime, "get_task_snapshot", side_effect=AssertionError("缓存命中不能重复投影")):
            second = self.service.get_task_result(task_id)
        self.assertGreater(self.authorization_reads, before)
        self.assertEqual(second["result"]["nested"]["value"], 1)
        self.assertNotIn("receipt_revision", json.loads(json.dumps(second)))
        self.allowed = False
        with self.assertRaises(GatewayError) as caught:
            self.service.get_task_result(task_id)
        self.assertEqual(caught.exception.error_code, "forbidden")
        self.assertEqual(self.actions, [1])

    def test_two_services_same_revision_and_changed_registry_label(self):
        task_id = self.finish({"value": 1})
        first = self.service.get_task_result(task_id)
        self.label = "标签 B"
        other = AgentToolService(self.runtime, self.access)
        self.services.append(other)
        second = other.get_task_result(task_id)
        self.assertEqual(first, second)
        self.assertEqual(self.actions, [1])

    def test_contract_model_result_owned_across_services(self):
        original = TaskSpec(task_id="task_offline_20261002t000000z_12345678", capability_name="offline", metadata={"values": [1]})
        task_id = self.finish(original)
        first = self.service.get_task_result(task_id)
        original.metadata["values"].append(2)
        capture = self.manager.capture_task_receipt(task_id)
        capture.snapshot()["result"].metadata["values"].append(3)
        self.manager.get_task_result(task_id).metadata["values"].append(4)
        other = AgentToolService(self.runtime, self.access)
        self.services.append(other)
        self.assertEqual(other.get_task_result(task_id), first)
        self.assertEqual(first["result"]["metadata"]["values"], [1])
        self.assertEqual(self.actions, [1])

    def test_two_services_concurrent_same_revision_read(self):
        task_id = self.finish({"value": 1})
        other = AgentToolService(self.runtime, self.access)
        self.services.append(other)
        self.assertTrue(self.bus.shutdown(2))
        for service in (self.service, other):
            with service._terminal_projection_lock:
                service._terminal_projection_cache.clear()
                service._terminal_projection_bytes = 0
        barrier = threading.Barrier(2)
        original = self.runtime.get_task_snapshot
        outputs, errors = [], []
        def simultaneous(*args, **kwargs):
            barrier.wait(2)
            return original(*args, **kwargs)
        def read(service):
            try:
                outputs.append(service.get_task_result(task_id))
            except BaseException as error:  # noqa: BLE001 - 主线程集中断言线程失败。
                errors.append(error)
        with patch.object(self.runtime, "get_task_snapshot", simultaneous):
            threads = [threading.Thread(target=read, args=(service,)) for service in (self.service, other)]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join(3)
                self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(len(outputs), 2)
        self.assertEqual(outputs[0], outputs[1])
        self.assertEqual(self.actions, [1])

    def test_old_http_read_cannot_overwrite_late_metadata(self):
        task_id = self.finish({"finalize_pending": True})
        captured, release, ended = threading.Event(), threading.Event(), threading.Event()
        self.releases.append(release)
        original = self.service.store.save_terminal
        errors = []
        def delayed(task_id, snapshot, **version):
            self.assertFalse(self.manager._lock._is_owned())
            if threading.current_thread().name == "older-read":
                captured.set()
                self.require(release)
            return original(task_id, snapshot, **version)
        def old_read():
            try:
                self.service.get_task_result(task_id)
            except BaseException as error:  # noqa: BLE001 - 收集离线线程失败并在主线程断言。
                errors.append(error)
            finally:
                ended.set()
        with patch.object(self.service.store, "save_terminal", delayed):
            thread = threading.Thread(target=old_read, name="older-read")
            thread.start()
            self.require(captured)
            self.manager.update_task_metadata(task_id, {"map_asset_finalize": {"status": "succeeded"}})
            newer = self.service.get_task_result(task_id)
            release.set()
            self.require(ended)
            thread.join(2)
        self.assertEqual(errors, [])
        recorded = self.service.get_task_record(task_id)["snapshot"]
        self.assertEqual(recorded, newer)
        self.assertEqual(recorded["map_asset_finalize"]["status"], "succeeded")
        self.assertEqual(self.actions, [1])

    def test_known_late_metadata_subscriber_updates_durable_record(self):
        task_id = self.finish({"finalize_pending": True})
        self.service.get_task_result(task_id)
        saved = threading.Event()
        original = self.service.store.save_prepared_terminal
        def observe(prepared):
            outcome = original(prepared)
            if json.loads(prepared.encoded).get("map_asset_finalize"):
                saved.set()
            return outcome
        with patch.object(self.service.store, "save_prepared_terminal", observe):
            self.manager.update_task_metadata(task_id, {"map_asset_finalize": {"status": "succeeded"}})
            self.require(saved)
        self.assertEqual(self.service.get_task_record(task_id)["snapshot"]["map_asset_finalize"]["status"], "succeeded")
        self.assertEqual(self.actions, [1])

    def test_cache_has_entry_and_serialized_byte_bounds(self):
        with patch("gateways.agent.service._TERMINAL_PROJECTION_CACHE_BYTES", 10), patch("gateways.agent.service._TERMINAL_PROJECTION_CACHE_ENTRIES", 2):
            self.service._cache_terminal_projection(("a", "s", 1), b"1234")
            self.service._cache_terminal_projection(("b", "s", 1), b"1234")
            self.service._cache_terminal_projection(("c", "s", 1), b"1234")
            self.assertEqual(len(self.service._terminal_projection_cache), 2)
            self.assertEqual(self.service._terminal_projection_bytes, 8)
            self.service._cache_terminal_projection(("d", "s", 1), b"1234567")
            self.assertEqual(self.service._terminal_projection_bytes, 7)
            self.assertEqual(len(self.service._terminal_projection_cache), 1)
            self.service._cache_terminal_projection(("large", "s", 1), b"x" * 11)
            self.assertEqual(self.service._terminal_projection_bytes, 7)

    def test_capture_and_projection_failures_never_change_outcome_or_replay(self):
        class Uncopyable:
            def __deepcopy__(self, memo):
                raise TypeError("不可复制")
        class Unserializable:
            __slots__ = ("value",)
            def __init__(self):
                self.value = object()
        for value in (Uncopyable(), Unserializable()):
            task_id = self.finish(value)
            with self.assertRaises(GatewayError):
                self.service.get_task_result(task_id)
            self.assertEqual(self.manager.get_task_status(task_id).state, "succeeded")
        self.assertEqual(self.actions, [1, 1])

    def test_web_and_agent_capture_errors_have_same_receipt_truth(self):
        class Uncopyable:
            def __deepcopy__(self, memo):
                raise TypeError("不可复制")
        task_id = self.finish(Uncopyable())
        errors = []
        for operation in (self.runtime.get_task_snapshot, self.service.get_task_result):
            with self.assertRaises(GatewayError) as caught:
                operation(task_id)
            errors.append(build_public_error_payload(caught.exception, task_id=task_id))
        self.assertEqual(errors[0], errors[1])
        self.assertEqual(errors[0]["code"], "task_receipt_unsupported")
        self.assertEqual(errors[0]["error_category"], "receipt")
        self.assertFalse(errors[0]["retryable"])
        self.assertEqual(self.manager.get_task_status(task_id).state, "succeeded")
        self.assertEqual(self.actions, [1])

    def test_agent_invoke_preserves_known_acceptance_when_full_metadata_cannot_encode(self):
        class Opaque:
            def __deepcopy__(self, memo):
                raise TypeError("unused metadata")
        capability, tool_name = "offline_task", "offline_tool"
        from contracts.capabilities import CapabilityDescriptor, CapabilityRiskLevel
        from contracts.skills import SkillDescriptor, SkillCategory
        from core.capability_registry import CapabilityRegistry
        from skills.registry import SkillRegistry
        descriptor = CapabilityDescriptor(name=capability, display_name="离线任务", description="离线任务。",
            execution_mode=CapabilityExecutionMode.ASYNC_TASK, risk_level=CapabilityRiskLevel.LOW,
            exposed_to_agent=True, input_schema={"type": "object"})
        def handler(payload, *, requested_by, request_context):
            task = self.manager.submit(capability, lambda context: self.actions.append(1) or {"known": True},
                                       metadata={"unused": Opaque()})
            self.manager.wait_for_task(task.task_id, timeout=2)
            return task
        self.runtime.capability_registry = CapabilityRegistry()
        self.runtime.capability_registry.register(descriptor, handler)
        self.runtime.capability_registry.assert_supported = lambda *args, **kwargs: None
        self.runtime.skill_registry = SkillRegistry()
        self.runtime.skill_registry.register(SkillDescriptor(name=tool_name, display_name="离线工具", description="离线任务。",
            category=SkillCategory.TASK, capability_name=capability))
        self.runtime._refresh_dynamic_capability_matrix = lambda **kwargs: None
        self.runtime._requires_motion_admission = lambda descriptor: False
        self.runtime._ensure_geometry_runtime_for_invoke = lambda name: None
        first = self.service.invoke("opaque-accepted", tool_name, {})
        repeated = self.service.invoke("opaque-accepted", tool_name, {})
        self.assertEqual(self.actions, [1])
        self.assertEqual(first, repeated)
        self.assertEqual(first["delivery_state"], "recorded")
        self.assertEqual(first["status"]["state"], "succeeded")
        self.assertEqual(first["error"]["code"], "agent_call_result_unsupported")
        self.assertFalse(first["error"]["retryable"])
        self.assertFalse(first["completeness"]["complete"])
        self.assertEqual(self.service.store.get("opaque-accepted")["task_id"], first["task_id"])
        self.assertEqual(self.manager.get_task_status(first["task_id"]).state, "succeeded")
        full = self.service.get_call_result("opaque-accepted")
        self.assertEqual(full["result"]["task_id"], first["task_id"])
        self.assertFalse(full["error"]["details"]["full_response_available"])

    def test_cross_service_identity_preserving_results_are_explicitly_unsupported(self):
        def callback():
            pass
        callback.value = 1
        class SharedBox:
            def __init__(self):
                self.values = [1]
            def __deepcopy__(self, memo):
                result = SharedBox()
                result.values = self.values
                return result
        other = AgentToolService(self.runtime, self.access)
        self.services.append(other)
        for result in (callback, {"callback": callback}, SharedBox()):
            task_id = self.finish(result)
            for service in (self.service, other):
                with self.assertRaises(GatewayError) as caught:
                    service.get_task_result(task_id)
                self.assertEqual(caught.exception.error_code, "task_receipt_unsupported")
            self.assertEqual(self.manager.get_task_status(task_id).state, "succeeded")
            self.assertEqual(self.service.store.find_by_task(task_id), None)
        self.assertEqual(self.actions, [1, 1, 1])

    def test_cancel_timeout_persist_returned_evidence_once(self):
        for timeout in (False, True):
            entered, release = threading.Event(), threading.Event()
            self.releases.append(release)
            def action(context, entered=entered, release=release):
                self.actions.append(1)
                entered.set()
                self.require(release)
                return {"returned_evidence": True}
            task_id = self.manager.submit("activate_map", action).task_id
            self.require(entered)
            if timeout:
                self.manager._handle_timeout(task_id)
            else:
                self.manager.cancel_task(task_id, "原始取消")
            before = self.manager.get_task_status(task_id)
            release.set()
            self.manager.wait_for_task(task_id, timeout=2)
            snapshot = self.service.get_task_result(task_id)
            recorded = self.service.get_task_record(task_id)["snapshot"]
            self.assertEqual(recorded, snapshot)
            self.assertEqual(snapshot["result"], {"returned_evidence": True})
            self.assertEqual(self.manager.get_task_status(task_id), before)
        self.assertEqual(self.actions, [1, 1])

    def test_transient_sqlite_failure_can_retry_snapshot_without_action(self):
        task_id = self.finish({"known": True})
        with patch.object(self.service.store, "save_terminal", side_effect=sqlite3.OperationalError("offline locked")), self.assertRaises(sqlite3.OperationalError):
            self.service.get_task_result(task_id)
        recovered = self.service.get_task_result(task_id)
        self.assertEqual(self.service.get_task_record(task_id)["snapshot"], recovered)
        self.assertEqual(self.actions, [1])


if __name__ == "__main__":
    unittest.main(verbosity=2)
