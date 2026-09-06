"""公共工具调用、任务结果与历史记录服务，不拥有对话或机器人任务执行。"""

from __future__ import annotations

import hashlib
import json
import logging
import math
from pathlib import Path
import threading
import time
from typing import Any, Dict
from urllib.parse import quote
import uuid

from jsonschema.validators import validator_for
from jsonschema.exceptions import SchemaError, ValidationError

from core.agent_call_store import AgentCallStore
from core.errors import GatewayError, as_gateway_error
from gateways.agent.feedback import (
    INPUT_MAX_BYTES, TERMINAL_STATES, build_receipt, checked_json,
)
from gateways.feedback import build_public_error_payload
from gateways.request_context import normalize_request_id


LOGGER = logging.getLogger("nuwax_robot_bridge.agent_tools")


def validate_request_id(value: Any) -> str:
    if not isinstance(value, str):
        raise GatewayError("request_id 必须是稳定字符串。", error_code="invalid_params", http_status=422)
    context = normalize_request_id(value)
    if context.request_id_replaced or context.request_id != value:
        raise GatewayError("request_id 只允许 1 至 128 个字母、数字及 ._-:，不可含空白。",
                           error_code="invalid_params", http_status=422)
    return value


def validate_arguments(arguments: Dict[str, Any], schema: Dict[str, Any]) -> None:
    """使用能力真实结构验证参数；禁止结构引用触发远端请求。"""
    stack = [schema]
    while stack:
        item = stack.pop()
        if isinstance(item, dict):
            for key in ("$ref", "$dynamicRef", "$recursiveRef"):
                ref = item.get(key)
                if isinstance(ref, str) and not ref.startswith("#"):
                    raise GatewayError("工具参数结构含不允许的远端引用。",
                                       error_code="agent_schema_unsupported", http_status=500)
            stack.extend(item.values())
        elif isinstance(item, list):
            stack.extend(item)
    try:
        validator = validator_for(schema)
        validator.check_schema(schema)
        validator(schema).validate(arguments)
    except ValidationError as exc:
        raise GatewayError("参数不符合工具声明的输入结构。", error_code="invalid_params", http_status=422,
                           details={"path": list(exc.absolute_path), "validator": exc.validator}) from exc
    except SchemaError as exc:
        raise GatewayError("工具参数结构无效。", error_code="agent_schema_invalid", http_status=500) from exc


class AgentToolService:
    """适配统一能力入口；持久记录仅用于查询，永不自动重放动作。"""

    def __init__(self, runtime, access_manager) -> None:
        self.runtime = runtime
        self.access_manager = access_manager
        self.gateway_instance_id = uuid.uuid4().hex
        manifest = runtime.robot.manifest
        self.robot_model = str(manifest.robot_model)
        parameters = getattr(getattr(runtime.robot, "defaults", None), "parameters", {})
        self.robot_identity = {
            "robot_model": self.robot_model,
            "robot_name": str(getattr(manifest, "robot_name", "")),
            "entrypoint": str(getattr(manifest, "entrypoint", "")),
            "robot_id": str(parameters.get("robot_id", "")),
        }
        artifact_dir = Path(runtime.config.gateway.artifact_dir).expanduser().resolve()
        scope = hashlib.sha256(json.dumps([self.robot_identity, str(artifact_dir)],
                                          sort_keys=True).encode("utf-8")).hexdigest()
        self.store = AgentCallStore(artifact_dir.parent / "agent_calls.sqlite3", scope=scope,
                                   gateway_instance_id=self.gateway_instance_id)
        self._uncertain_calls = set()
        self._uncertain_lock = threading.Lock()
        self._subscription_id = runtime.event_bus.subscribe(
            self._on_task_terminal, event_types=tuple("task." + state for state in TERMINAL_STATES),
            async_dispatch=True,
        )

    def close(self) -> None:
        """只释放本服务拥有的事件订阅，不停止机器人运行时。"""
        if self._subscription_id is not None:
            self.runtime.event_bus.unsubscribe(self._subscription_id)
            self._subscription_id = None

    def discover(self) -> Dict[str, Any]:
        return {
            "schema_version": 1, "gateway_instance_id": self.gateway_instance_id,
            "robot_model": self.robot_model, "robot_identity": self.robot_identity,
            "robot_identity_source": "configured_assembly_and_artifact_directory",
            "paths": {"tools": "/api/agent/tools", "invoke": "/api/agent/invoke",
                      "calls": "/api/agent/calls", "tasks": "/api/agent/tasks",
                      "events": "/events/stream", "artifacts": "/artifacts"},
            "limits": {"invoke_max_bytes": INPUT_MAX_BYTES, "wait_max_seconds": 30},
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
        views = self.runtime.list_tools(exposed_only=True, include_unsupported=False)
        view = next((item for item in views if item.descriptor.name == tool_name
                     and item.supported and item.runnable and item.descriptor.exposed_to_agent), None)
        if view is None:
            raise GatewayError("工具未公开或当前不可调用，请刷新工具目录。",
                               error_code="capability_not_registered", http_status=404)
        self.access_manager.authorize_capability(self.runtime.capability_registry.get_descriptor(view.capability_name))
        validate_arguments(arguments, view.input_schema)
        record, owner = self.store.begin(request_id, tool_name, view.capability_name, arguments)
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
                                               capability_name=view.capability_name)
        try:
            response = checked_json(response)
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
        return build_receipt(request_id=record["request_id"], gateway_instance_id=record["gateway_instance_id"],
                             tool_name=record["tool_name"], capability_name=record["capability_name"],
                             response=record.get("response"), delivery_state=state, error=error,
                             full_result_path="/api/agent/calls/%s/result" % quote(record["request_id"], safe=""))

    def _task_snapshot(self, task_id: str) -> Dict[str, Any]:
        manager = self.runtime.task_manager
        try:
            # 同一锁内取得状态与结果，避免拼接出不同转换时刻的快照。
            with manager._lock:
                spec = manager.get_task_spec(task_id)
                self.access_manager.authorize_capability(self.runtime.capability_registry.get_descriptor(spec.capability_name))
                snapshot = {"task_id": task_id, "task": spec,
                            "status": manager.get_task_status(task_id),
                            "events": manager.get_task_events(task_id), "result": manager.get_task_result(task_id),
                            "error": manager.get_task_error(task_id),
                            "error_payload": manager.get_task_error_payload(task_id)}
        except KeyError as exc:
            raise GatewayError("当前网关实例没有此任务；旧调用记录不代表当前机器人状态。",
                               error_code="not_found", http_status=404,
                               details={"status_source": "live_task", "gateway_instance_id": self.gateway_instance_id,
                                        "historical_record_path": "/api/agent/tasks/%s/record" % quote(task_id, safe="")}) from exc
        return checked_json(snapshot)

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
        record = self.store.find_by_task(snapshot["task_id"])
        request_id = record["request_id"] if record else spec.get("metadata", {}).get("request_id")
        return build_receipt(request_id=request_id, gateway_instance_id=self.gateway_instance_id,
                             tool_name=record["tool_name"] if record else None,
                             capability_name=spec["capability_name"], response=snapshot, delivery_state="recorded",
                             full_result_path="/api/agent/tasks/%s/result" % quote(snapshot["task_id"], safe=""), scope="task")

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
                snapshot = self._task_snapshot(task_id)
                status = snapshot["status"]
                if status["state"] in TERMINAL_STATES or (status["state"], status.get("stage")) != baseline_key:
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    timed_out = True
                    break
                changed.wait(remaining)
            self._save_terminal(snapshot)
            return {**self._task_receipt(snapshot), "wait_timed_out": timed_out}
        finally:
            self.runtime.event_bus.unsubscribe(subscription)

    def cancel_task(self, task_id: str) -> Dict[str, Any]:
        self._task_snapshot(task_id)
        self.runtime._handle_cancel_task({"task_id": task_id}, requested_by="local")
        return self.get_task(task_id)

    def _save_terminal(self, snapshot: Dict[str, Any]) -> None:
        if snapshot["status"]["state"] in TERMINAL_STATES:
            self.store.save_terminal(snapshot["task_id"], snapshot)

    def _on_task_terminal(self, event) -> None:
        if event.task_id:
            try:
                self._save_terminal(self._task_snapshot(event.task_id))
            except Exception:
                # 保留持久化失败证据，但不能让记录消费者覆盖实际机器人终态。
                LOGGER.exception("任务终态记录失败 task_id=%s", event.task_id)
