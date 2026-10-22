"""公共工具调用、任务结果与历史记录服务，不拥有对话或机器人任务执行。"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import threading
import time
import uuid
from collections import OrderedDict
from pathlib import Path
from typing import Any, Dict
from urllib.parse import quote

from starlette.concurrency import run_in_threadpool

import gateways.agent.feedback as agent_feedback
import gateways.feedback as public_feedback
from core.agent_call_store import AgentCallStore
from core.errors import GatewayError, as_gateway_error
from gateways.agent.feedback import (
    INPUT_MAX_BYTES,
    TERMINAL_STATES,
    build_receipt,
    checked_json,
)
from gateways.agent.terminal_persistence import TerminalReceiptPersistence
from gateways.feedback import _redact_text, build_mcp_tool_result, build_public_error_payload, compact_public_data
from gateways.request_context import normalize_request_id

LOGGER = logging.getLogger("nuwax_robot_bridge.agent_tools")

_TERMINAL_PROJECTION_CACHE_BYTES = 8 * 1024 * 1024
_TERMINAL_PROJECTION_CACHE_ENTRIES = 64


class _VersionedTaskSnapshot(dict):
    """身份/版本仅附着于进程内封套，不作为公共 JSON 字段。"""

    def __init__(self, content, *, source_id, revision):
        super().__init__(content)
        self.receipt_source_id = source_id
        self.receipt_revision = revision


class _AsyncTaskWakeup:
    """跨线程合并通知：每个等待者最多排队一个回调，不保留事件载荷。"""

    def __init__(self, loop) -> None:
        self.event = asyncio.Event()
        self._loop = loop
        self._lock = threading.Lock()
        self._notified = False
        self._queued = False
        self._closed = False

    def notify(self) -> None:
        with self._lock:
            if self._closed or self._notified:
                return
            self._notified = True
            if self._queued:
                return
            self._queued = True
            try:
                self._loop.call_soon_threadsafe(self._deliver)
            except RuntimeError:
                # 关闭中的事件循环不能再接收回调；不可让发布任务的线程失败。
                self._closed = True
                self._queued = False

    def _deliver(self) -> None:
        with self._lock:
            self._queued = False
            if self._notified and not self._closed:
                self.event.set()

    def clear(self) -> None:
        # 只由所属事件循环调用；先清除再读快照，防止读后清除丢失变更。
        with self._lock:
            self._notified = False
            self.event.clear()

    def close(self) -> None:
        with self._lock:
            self._closed = True


def validate_request_id(value: Any) -> str:
    if not isinstance(value, str):
        raise GatewayError("request_id 必须是稳定字符串。", error_code="invalid_params", http_status=422)
    context = normalize_request_id(value)
    if context.request_id_replaced or context.request_id != value:
        raise GatewayError("request_id 只允许 1 至 128 个字母、数字及 ._-:，不可含空白。",
                           error_code="invalid_params", http_status=422)
    return value






class AgentToolService:
    """适配统一能力入口；持久记录仅用于查询，永不自动重放动作。"""

    def __init__(self, runtime, access_manager) -> None:
        self.runtime = runtime
        self.access_manager = access_manager
        self.gateway_instance_id = uuid.uuid4().hex
        manifest = runtime.robot.manifest
        self.robot_model = str(manifest.robot_model)
        parameters = runtime.robot.defaults.parameters
        self.robot_identity = {
            "robot_model": self.robot_model,
            "robot_name": str(manifest.robot_name),
            "entrypoint": str(manifest.entrypoint),
            "robot_id": str(parameters.get("robot_id", "")),
        }
        artifact_dir = Path(runtime.config.gateway.artifact_dir).expanduser().resolve()
        self.store = AgentCallStore(
            artifact_dir.parent / "agent_calls.sqlite3", robot_identity=self.robot_identity,
            artifact_dir=artifact_dir, gateway_instance_id=self.gateway_instance_id)
        self._uncertain_calls = set()
        self._uncertain_lock = threading.Lock()
        self._waiters_lock = threading.Lock()
        self._async_waiters = {}
        self._closed = False
        self._terminal_projection_lock = threading.Lock()
        self._terminal_projection_cache = OrderedDict()
        self._terminal_projection_bytes = 0
        self._terminal_owner = TerminalReceiptPersistence(
            runtime.task_manager, self.store, self._task_snapshot, self._receipt_eligible, autostart=False)
        self._subscription_id = runtime.event_bus.subscribe(
            self._on_task_terminal, categories=("task",), async_dispatch=False)
        try:
            self._terminal_owner.start()
        except BaseException:
            runtime.event_bus.unsubscribe(self._subscription_id)
            self._subscription_id = None
            raise
        # 订阅前已存在或恰好完成的终态由源分页覆盖，不依赖总线重放保留。
        self._terminal_owner.request_reconcile()

    def close(self, timeout_sec=2.0, *, source_fenced=False):
        """关闭本服务并返回持久化覆盖情况，不停止机器人或伪造源静止。"""
        with self._waiters_lock:
            self._closed = True
            subscription = self._subscription_id
            self._subscription_id = None
            waiters = tuple(self._async_waiters.items())
            self._async_waiters.clear()
        if subscription is not None:
            self.runtime.event_bus.unsubscribe(subscription)
        for subscription, changed in waiters:
            self.runtime.event_bus.unsubscribe(subscription)
            changed.notify()
        return self._terminal_owner.close(timeout_sec, source_fenced=source_fenced)

    def _receipt_eligible(self, capability_name):
        self.access_manager.authorize_capability(self.runtime.capability_registry.get_descriptor(capability_name))
        return True

    def persistence_status(self):
        return self._terminal_owner.status()

    def discover(self) -> Dict[str, Any]:
        return {
            "schema_version": 1, "gateway_instance_id": self.gateway_instance_id,
            "robot_model": self.robot_model, "robot_identity": self.robot_identity,
            "robot_identity_source": "configured_assembly_and_artifact_directory",
            "paths": {"tools": "/api/agent/tools", "invoke": "/api/agent/invoke",
                      "calls": "/api/agent/calls", "tasks": "/api/agent/tasks",
                      "events": "/events/stream", "artifacts": "/artifacts",
                      "feedback": "/api/feedback/stream"},
            "feedback": {"transport": "sse", "resume_header": "Last-Event-ID",
                         "subscribe_before_invoke": True,
                         "agent_integration": "将反馈作为观测加入会话，attention=wake时唤醒宿主；不重复执行动作。"},
            "limits": {"invoke_max_bytes": INPUT_MAX_BYTES, "wait_max_seconds": 30},
            "terminal_persistence": self.persistence_status(),
        }

    def list_tools(self) -> Dict[str, Any]:
        views = self.runtime.list_tools(exposed_only=True, include_unsupported=False)
        allowed = [view for view in views if view.supported and view.runnable
                   and view.descriptor.exposed_to_agent
                   and self.access_manager.can_access_capability(
                       self.runtime.capability_registry.get_descriptor(view.capability_name))]
        return checked_json({"tools": allowed})

    def invoke(self, request_id: str, tool_name: str, arguments: Dict[str, Any]) -> Dict[str, Any]:
        validate_request_id(request_id)
        if not isinstance(tool_name, str) or not tool_name or len(tool_name) > 128 or not isinstance(arguments, dict):
            raise GatewayError("tool_name 必须为工具名，arguments 必须为对象。",
                               error_code="invalid_params", http_status=422)
        arguments = checked_json(arguments, max_bytes=INPUT_MAX_BYTES)
        # 先查已记录请求，使暂时不可用的工具仍能恢复查看；并发最终由数据库占位裁决。
        try:
            existing = self.store.get(request_id)
        except GatewayError as exc:
            if exc.http_status != 404:
                raise
        else:
            record, _ = self.store.begin(request_id, tool_name, existing["capability_name"], arguments)
            return self._call_receipt(record)
        try:
            descriptor = self.runtime.get_tool_descriptor(tool_name)
        except KeyError as exc:
            raise GatewayError("工具未注册，请刷新工具目录。",
                               error_code="capability_not_registered", http_status=404) from exc
        if not descriptor.exposed_to_agent:
            raise GatewayError("工具未向智能体公开。", error_code="access_denied", http_status=403)
        capability = self.runtime.capability_registry.get_descriptor(descriptor.capability_name)
        self.access_manager.authorize_capability(capability)
        record, owner = self.store.begin(request_id, tool_name, descriptor.capability_name, arguments)
        if not owner:
            return self._call_receipt(record)
        error = None
        response = None
        try:
            response = self.runtime.invoke_tool(tool_name, arguments, requested_by="local",
                                                request_context={"request_id": request_id, "tool_name": tool_name},
                                                project_result=False)
        except Exception as exc:
            error = build_public_error_payload(as_gateway_error(exc), request_id=request_id,
                                               capability_name=descriptor.capability_name)
        try:
            try:
                response = checked_json(response)
            except Exception as encoding_error:  # 已知受理身份不能因附加值不可编码而丢失。
                if error is not None:
                    raise
                response, error = self._known_task_acceptance(
                    response, request_id=request_id, capability_name=descriptor.capability_name,
                    encoding_error=encoding_error)
            record = self.store.finish(request_id, response, error)
        except Exception:
            # 命令可能已产生副作用；保存失败后只报告未知，绝不再次执行。
            LOGGER.exception("工具结果未能持久保存 request_id=%s", request_id)
            with self._uncertain_lock:
                self._uncertain_calls.add(request_id)
            try:
                self.store.mark_unknown(request_id)
            except Exception:
                LOGGER.error("工具未知状态未能持久保存 request_id=%s", request_id)
            record["delivery_state"] = "unknown"
            record["response"] = None
        return self._call_receipt(record)

    def _known_task_acceptance(self, response, *, request_id, capability_name, encoding_error):
        """仅保留运行时已经返回的受理证据；不重新查询/提交任何动作。"""
        if not isinstance(response, dict) or response.get("mode") != "async_task":
            raise encoding_error
        task_id = response.get("task_id")
        if not isinstance(task_id, str) or not task_id or response.get("capability_name") != capability_name:
            raise encoding_error
        status = response.get("status")
        status_id = status.get("task_id") if isinstance(status, dict) else getattr(status, "task_id", None)
        if status_id != task_id:
            raise encoding_error
        status = compact_public_data(status, max_depth=4, max_items=24, max_string_chars=1000,
                                     max_total_bytes=8192, max_nodes=64)
        if not isinstance(status, dict) or status.get("task_id") != task_id:
            raise encoding_error
        accepted = checked_json({
            "request_id": request_id, "task_id": task_id, "capability_name": capability_name,
            "mode": "async_task", "task": {"task_id": task_id, "capability_name": capability_name},
            "status": status, "feedback": None,
        })
        receipt_error = build_public_error_payload(GatewayError(
            "任务受理与标识已确认，但完整调用返回含无法编码的值；已保存已知任务状态，不要重发动作。",
            error_code="agent_call_result_unsupported", http_status=500,
            details={"full_response_available": False, "task_outcome_unchanged": True,
                     "action_replay_allowed": False}), request_id=request_id,
            capability_name=capability_name, task_id=task_id)
        return accepted, receipt_error

    def get_call(self, request_id: str) -> Dict[str, Any]:
        validate_request_id(request_id)
        return self._call_receipt(self.store.get(request_id))

    def get_call_result(self, request_id: str) -> Any:
        validate_request_id(request_id)
        record = self.store.get(request_id)
        self.access_manager.authorize_capability(
            self.runtime.capability_registry.get_descriptor(record["capability_name"]))
        with self._uncertain_lock:
            uncertain = request_id in self._uncertain_calls
        if record["delivery_state"] != "recorded" or uncertain:
            raise GatewayError("本次调用尚无完整确定回执；不得据此重发动作。",
                               error_code="agent_call_outcome_unknown", http_status=409)
        if record["error"]:
            return {"request_id": request_id, "capability_name": record["capability_name"],
                    "result": record["response"], "error": record["error"]}
        return record["response"]

    def _call_receipt(self, record: Dict[str, Any]) -> Dict[str, Any]:
        self.access_manager.authorize_capability(
            self.runtime.capability_registry.get_descriptor(record["capability_name"]))
        state = record["delivery_state"]
        with self._uncertain_lock:
            if record["request_id"] in self._uncertain_calls:
                state = "unknown"
        error = record.get("error")
        if state == "unknown":
            error = build_public_error_payload(GatewayError(
                "调用结果未知，请核对原任务与设备状态，不要自动重发。",
                error_code="agent_call_outcome_unknown", http_status=409), request_id=record["request_id"])
        receipt = build_receipt(request_id=record["request_id"], gateway_instance_id=record["gateway_instance_id"],
                                tool_name=record["tool_name"], capability_name=record["capability_name"],
                                response=record.get("response"), delivery_state=state, error=error,
                                full_result_path="/api/agent/calls/%s/result" % quote(record["request_id"], safe=""))
        if error and error.get("code") == "agent_call_result_unsupported":
            receipt["completeness"]["complete"] = False
            receipt["completeness"]["artifacts_complete"] = False
            receipt["summary"] = "任务标识与状态已保存，但完整调用回执无法编码。请核对原任务，不要自动重发动作。"
        return receipt

    def _cached_terminal_projection(self, key):
        with self._terminal_projection_lock:
            encoded = self._terminal_projection_cache.get(key)
            if encoded is not None:
                self._terminal_projection_cache.move_to_end(key)
            return encoded

    def _cache_terminal_projection(self, key, encoded):
        if len(encoded) > _TERMINAL_PROJECTION_CACHE_BYTES:
            return
        with self._terminal_projection_lock:
            previous = self._terminal_projection_cache.pop(key, None)
            if previous is not None:
                self._terminal_projection_bytes -= len(previous)
            self._terminal_projection_cache[key] = encoded
            self._terminal_projection_bytes += len(encoded)
            while (len(self._terminal_projection_cache) > _TERMINAL_PROJECTION_CACHE_ENTRIES
                   or self._terminal_projection_bytes > _TERMINAL_PROJECTION_CACHE_BYTES):
                _, evicted = self._terminal_projection_cache.popitem(last=False)
                self._terminal_projection_bytes -= len(evicted)

    @staticmethod
    def _projection_context():
        # 完整投影版本与当前限制共同决定可编码性；不缓存权限或动态注册表错误。
        return (1, public_feedback.MCP_RESULT_MAX_BYTES, agent_feedback.FULL_RESULT_MAX_BYTES)

    def _task_snapshot(self, task_id: str) -> _VersionedTaskSnapshot:
        manager = self.runtime.task_manager
        try:
            # 授权在每次读取执行，不能因缓存命中而继承上一次读取者的权限。
            capability = manager.get_task_capability_name(task_id)
            self.access_manager.authorize_capability(self.runtime.capability_registry.get_descriptor(capability))
            source_id, revision = manager.task_receipt_version(task_id)
            projection_context = self._projection_context()
            encoded = self._cached_terminal_projection((task_id, source_id, revision, projection_context))
            if encoded is not None:
                return _VersionedTaskSnapshot(json.loads(encoded), source_id=source_id, revision=revision)
            capture = manager.capture_task_receipt(task_id)
            failure = capture.projection_failure(projection_context)
            if failure is not None:
                raise GatewayError(**failure)
            snapshot = self.runtime.get_task_snapshot(task_id, captured_task=capture)
            try:
                public = checked_json(build_mcp_tool_result(snapshot, max_total_bytes=projection_context[1])["structuredContent"],
                                      max_bytes=projection_context[2])
            except GatewayError as error:
                if (self._projection_context() == projection_context
                        and error.error_code in {"mcp_result_unavailable", "mcp_response_too_large", "agent_payload_too_large", "agent_result_unavailable"}
                        and len(error.message) <= 512 and set(error.details) <= {"reason", "max_bytes"}
                        and all(type(value) in (str, int, bool, type(None))
                                and (not isinstance(value, str) or len(value) <= 128)
                                for value in error.details.values())):
                    capture.remember_projection_failure(projection_context, {"message": error.message, "error_code": error.error_code,
                                                        "http_status": error.http_status, "jsonrpc_code": error.jsonrpc_code,
                                                        "details": error.details})
                raise
            # 公共投影/编码都在任务锁外。缓存不可变字节，响应使用独立解码对象。
            if public["status"]["state"] in TERMINAL_STATES and self._projection_context() == projection_context:
                encoded = json.dumps(public, ensure_ascii=False, allow_nan=False,
                                     separators=(",", ":")).encode("utf-8")
                self._cache_terminal_projection((task_id, capture.source_id, capture.revision, projection_context), encoded)
            return _VersionedTaskSnapshot(public, source_id=capture.source_id, revision=capture.revision)
        except KeyError as exc:
            raise GatewayError("当前网关实例没有此任务；旧调用记录不代表当前机器人状态。",
                               error_code="not_found", http_status=404,
                               details={"status_source": "live_task", "gateway_instance_id": self.gateway_instance_id,
                                        "historical_record_path": "/api/agent/tasks/%s/record" % quote(task_id, safe="")}) from exc

    def get_task(self, task_id: str) -> Dict[str, Any]:
        snapshot = self._task_snapshot(task_id)
        self._save_terminal(snapshot)
        return self._task_receipt(snapshot)

    def get_task_result(self, task_id: str) -> Dict[str, Any]:
        snapshot = self._task_snapshot(task_id)
        self._save_terminal(snapshot)
        return snapshot

    def get_task_record(self, task_id: str) -> Dict[str, Any]:
        saved = self.store.get_terminal(task_id)
        snapshot = saved["snapshot"]
        capability_name = snapshot["task"]["capability_name"]
        self.access_manager.authorize_capability(self.runtime.capability_registry.get_descriptor(capability_name))
        return {"schema_version": 1, "record_scope": "historical_task", "status_source": "recorded_terminal",
                "current_gateway_instance_id": self.gateway_instance_id, **saved}

    def _task_receipt(self, snapshot: Dict[str, Any]) -> Dict[str, Any]:
        spec = snapshot["task"]
        record = self.store.find_task_call_identity(snapshot["task_id"])
        request_id = record["request_id"] if record else spec.get("metadata", {}).get("request_id")
        return build_receipt(request_id=request_id, gateway_instance_id=self.gateway_instance_id,
                             tool_name=record["tool_name"] if record else None,
                             capability_name=spec["capability_name"], response=snapshot, delivery_state="recorded",
                             full_result_path="/api/agent/tasks/%s/result" % quote(snapshot["task_id"], safe=""), scope="task")

    def _wait_status(self, task_id: str):
        """只读授权与有界状态；普通进度唤醒不复制历史或重做完整投影。"""
        manager = self.runtime.task_manager
        try:
            header = manager.task_receipt_header(task_id)
            self.access_manager.authorize_capability(
                self.runtime.capability_registry.get_descriptor(header["capability_name"]))
            status = manager.get_task_status(task_id)
        except KeyError as exc:
            raise GatewayError("当前网关实例没有此任务；旧调用记录不代表当前机器人状态。",
                               error_code="not_found", http_status=404,
                               details={"status_source": "live_task", "gateway_instance_id": self.gateway_instance_id,
                                        "historical_record_path": f"/api/agent/tasks/{quote(task_id, safe='')}/record"}) from exc
        state = status.get("state") if isinstance(status, dict) else status.state
        stage = status.get("stage") if isinstance(status, dict) else status.stage
        # 与完整公共投影使用同一标量脱敏，避免原始/公共阶段不同造成伪变化或忙循环。
        return {"state": state, "stage": _redact_text(stage) if isinstance(stage, str) else stage}, None

    @staticmethod
    def _wait_status_changed(status, baseline_key):
        return status["state"] in TERMINAL_STATES or (status["state"], status.get("stage")) != baseline_key

    def wait_task(self, task_id: str, timeout_sec: float = 30) -> Dict[str, Any]:
        if not math.isfinite(timeout_sec) or not 0 <= timeout_sec <= 30:
            raise GatewayError("timeout_sec 必须是 0 至 30 的有限数。", error_code="invalid_params", http_status=422)
        baseline = self._task_snapshot(task_id)
        baseline_key = (baseline["status"]["state"], baseline["status"].get("stage"))
        changed = threading.Event()
        subscription = self.runtime.event_bus.subscribe(
            lambda event: changed.set() if event.task_id == task_id else None,
        )
        deadline = time.monotonic() + timeout_sec
        timed_out = False
        try:
            while True:
                changed.clear()
                status, snapshot = self._wait_status(task_id)
                remaining = deadline - time.monotonic()
                if self._wait_status_changed(status, baseline_key) or remaining <= 0:
                    if snapshot is None:
                        snapshot = self._task_snapshot(task_id)
                    if self._wait_status_changed(snapshot["status"], baseline_key):
                        break
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        timed_out = True
                        break
                changed.wait(remaining)
            return self._finish_task_wait(snapshot, timed_out)
        finally:
            self.runtime.event_bus.unsubscribe(subscription)

    def _check_wait_open(self) -> None:
        if self._closed:
            raise GatewayError("工具服务已关闭，请重新连接网关。",
                               error_code="agent_service_closed", http_status=503)

    def _finish_task_wait(self, snapshot: Dict[str, Any], timed_out: bool) -> Dict[str, Any]:
        self._save_terminal(snapshot)
        return {**self._task_receipt(snapshot), "wait_timed_out": timed_out}

    async def wait_task_async(self, task_id: str, timeout_sec: float = 30) -> Dict[str, Any]:
        """HTTP 长轮询的等待阶段不占共享线程；快照和持久化仍在线程中执行。"""
        if not math.isfinite(timeout_sec) or not 0 <= timeout_sec <= 30:
            raise GatewayError("timeout_sec 必须是 0 至 30 的有限数。", error_code="invalid_params", http_status=422)
        self._check_wait_open()
        baseline = await run_in_threadpool(self._task_snapshot, task_id)
        baseline_key = (baseline["status"]["state"], baseline["status"].get("stage"))
        changed = _AsyncTaskWakeup(asyncio.get_running_loop())
        # 注册只修改小型内存索引；不能放进可被取消后遗弃的工作线程，否则会泄漏订阅。
        with self._waiters_lock:
            self._check_wait_open()
            subscription = self.runtime.event_bus.subscribe(
                lambda event: changed.notify() if event.task_id == task_id else None,
            )
            self._async_waiters[subscription] = changed
        deadline = time.monotonic() + timeout_sec
        timed_out = False
        try:
            while True:
                changed.clear()
                self._check_wait_open()
                # 订阅后重新读取；每次授权，但不为忽略的进度重新构建完整历史。
                status, snapshot = await run_in_threadpool(self._wait_status, task_id)
                self._check_wait_open()
                remaining = deadline - time.monotonic()
                if self._wait_status_changed(status, baseline_key) or remaining <= 0:
                    if snapshot is None:
                        snapshot = await run_in_threadpool(self._task_snapshot, task_id)
                        self._check_wait_open()
                    # 最终完整读决定返回原因；阶段回退不能产生伪变化，终态仍优先于超时。
                    if self._wait_status_changed(snapshot["status"], baseline_key):
                        break
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        timed_out = True
                        break
                try:
                    await asyncio.wait_for(changed.event.wait(), remaining)
                except asyncio.TimeoutError:
                    # 到期也读取最终快照，沿用同步接口的终态/阶段变更优先规则。
                    pass
            return await run_in_threadpool(self._finish_task_wait, snapshot, timed_out)
        finally:
            changed.close()
            with self._waiters_lock:
                owned = self._async_waiters.pop(subscription, None)
            if owned is not None:
                self.runtime.event_bus.unsubscribe(subscription)

    def cancel_task(self, task_id: str) -> Dict[str, Any]:
        self._task_snapshot(task_id)
        self.runtime._handle_cancel_task({"task_id": task_id}, requested_by="local")
        return self.get_task(task_id)

    def _save_terminal(self, snapshot: _VersionedTaskSnapshot) -> None:
        if snapshot["status"]["state"] in TERMINAL_STATES:
            self.store.save_terminal(snapshot["task_id"], snapshot,
                                     source_id=snapshot.receipt_source_id, revision=snapshot.receipt_revision)

    def _on_task_terminal(self, event) -> None:
        # 同步回调仅交付小型 ID，完整源读取、投影和 SQLite 均由独立拥有者处理。
        if event.task_id:
            self._terminal_owner.offer(event.task_id)
