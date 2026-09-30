#!/usr/bin/env python3
"""机器人智能体接口客户端；只依赖Python标准库，可单文件复制到智能体工作区。"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
from http.client import HTTPException
import json
import math
import os
from pathlib import Path
import re
import socket
import sys
import tempfile
import threading
from typing import Any, Dict, Optional
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlencode, urlsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener
from uuid import uuid4


JSON_LIMIT = 32 * 1024 * 1024
ARTIFACT_LIMIT = 64 * 1024 * 1024
TERMINAL_STATES = frozenset(("succeeded", "failed", "cancelled", "timeout"))


class RobotClientError(RuntimeError):
    """保留机器可读错误，传输不确定时禁止自动重发。"""

    def __init__(self, code: str, message: str, *, request_id: Optional[str] = None,
                 details: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(message)
        self.payload = {"code": code, "message": message, "retryable": False}
        if request_id:
            self.payload["request_id"] = request_id
        if details:
            self.payload["details"] = details


class _NoRedirect(HTTPRedirectHandler):
    """网关地址固定，拒绝重定向转发凭据或改变动作请求。"""

    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def _strict_constant(value: str) -> None:
    raise ValueError("不允许非有限数值：" + value)


def _json_loads(raw: bytes) -> Any:
    return json.loads(raw.decode("utf-8"), parse_constant=_strict_constant)


def _bounded_read(response, limit: int) -> bytes:
    declared = response.headers.get("Content-Length")
    expected = int(declared) if declared is not None else None
    if expected is not None and expected > limit:
        raise RobotClientError("response_too_large", "响应超过客户端读取上限，请分页或读取制品。")
    raw = response.read(limit + 1)
    if len(raw) > limit:
        raise RobotClientError("response_too_large", "响应超过客户端读取上限，请分页或读取制品。")
    if expected is not None and len(raw) != expected:
        raise RobotClientError("transport_error", "响应被截断，未收到完整回执。")
    return raw


class RobotClient:
    """使用统一回执与真实任务状态，不将HTTP成功当作业务成功。"""

    def __init__(self, base_url: str, *, timeout: float = 35.0,
                 token: Optional[str] = None) -> None:
        parsed = urlsplit(base_url)
        try:
            valid_port = parsed.port is None or 0 < parsed.port <= 65535
        except ValueError:
            valid_port = False
        if (any(char.isspace() for char in base_url) or not valid_port
                or parsed.scheme not in ("http", "https") or not parsed.hostname
                or parsed.username or parsed.password or parsed.query or parsed.fragment):
            raise RobotClientError("invalid_url", "网关地址须为无凭据、查询参数和片段的HTTP或HTTPS基地址。")
        if not math.isfinite(timeout) or timeout <= 0 or timeout > 300:
            raise RobotClientError("invalid_timeout", "连接超时须大于0且不超过300秒。")
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.token = token
        if token and any(char in token for char in ("\r", "\n")):
            raise RobotClientError("invalid_token", "认证令牌不能包含换行。")
        self._opener = build_opener(ProxyHandler({}), _NoRedirect())

    def _request(self, path: str, *, method: str = "GET", body: Any = None,
                 request_id: Optional[str] = None, timeout: Optional[float] = None) -> Dict[str, Any]:
        headers = {"Accept": "application/json"}
        if self.token:
            headers["Authorization"] = "Bearer " + self.token
        if request_id:
            headers["X-Request-ID"] = request_id
        data = None
        if body is not None:
            try:
                data = json.dumps(body, ensure_ascii=False, allow_nan=False).encode("utf-8")
            except (TypeError, ValueError) as exc:
                raise RobotClientError("invalid_arguments", "参数必须是有限数值的JSON数据。", request_id=request_id) from exc
            headers["Content-Type"] = "application/json"
        request = Request(self.base_url + path, data=data, headers=headers, method=method)
        try:
            with self._opener.open(request, timeout=timeout or self.timeout) as response:
                payload = _json_loads(_bounded_read(response, JSON_LIMIT))
        except HTTPError as exc:
            try:
                error_body = _json_loads(_bounded_read(exc, 1024 * 1024))
                error = error_body.get("error", {}) if isinstance(error_body, dict) else {}
                if not isinstance(error, dict):
                    error = {}
            except (ValueError, UnicodeError, RobotClientError):
                error = {}
            finally:
                exc.close()
            raise RobotClientError(error.get("code", "http_error"),
                                   error.get("message", "网关请求失败，HTTP状态%d。" % exc.code),
                                   request_id=request_id,
                                   details={"http_status": exc.code, "gateway_error": error}) from exc
        except (URLError, TimeoutError, socket.timeout, ConnectionError, HTTPException) as exc:
            message = "连接未完成。"
            if method != "GET":
                message += "动作是否执行尚不确定；先查询原请求或任务，不要生成新请求重复执行。"
            raise RobotClientError("transport_error", message, request_id=request_id) from exc
        except (ValueError, UnicodeError) as exc:
            raise RobotClientError("invalid_response", "网关未返回有效JSON；变更请求请先查询原请求状态。",
                                   request_id=request_id) from exc
        except RobotClientError as exc:
            if request_id:
                exc.payload["request_id"] = request_id
            if method != "GET":
                exc.payload["next_action"] = "回执未完整接收，执行结果待核对；先查询原请求或任务，不要重新提交动作。"
            raise
        if not isinstance(payload, dict):
            raise RobotClientError("invalid_response", "网关响应必须是JSON对象。", request_id=request_id)
        return payload

    def info(self) -> Dict[str, Any]:
        payload = self._request("/api/agent")
        if type(payload.get("schema_version")) is not int or payload["schema_version"] != 1:
            raise RobotClientError("unsupported_api", "网关未提供受支持的智能体接口版本1。")
        return payload

    def tools(self, name: Optional[str] = None) -> Dict[str, Any]:
        payload = self._request("/api/agent/tools")
        if name is None:
            return payload
        for item in payload.get("tools", []):
            descriptor = item.get("descriptor", {})
            if item.get("name") == name or isinstance(descriptor, dict) and descriptor.get("name") == name:
                return {"tool": item}
        raise RobotClientError("tool_not_found", "当前机器人未公开该可调用工具：" + name)

    def invoke(self, tool_name: str, arguments: Optional[Dict[str, Any]] = None,
               *, request_id: Optional[str] = None) -> Dict[str, Any]:
        request_id = request_id or "agent_" + uuid4().hex
        if not isinstance(request_id, str) or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,128}", request_id):
            raise RobotClientError("invalid_request_id", "请求标识须为1到128位字母、数字或_.:-。")
        if arguments is not None and not isinstance(arguments, dict):
            raise RobotClientError("invalid_arguments", "工具参数必须是JSON对象。", request_id=request_id)
        try:
            return self._receipt(self._request("/api/agent/invoke", method="POST", request_id=request_id,
                                              body={"request_id": request_id, "tool_name": tool_name,
                                                    "arguments": arguments if arguments is not None else {}}))
        except RobotClientError as exc:
            exc.payload["request_id"] = request_id
            raise

    @staticmethod
    def _receipt(payload: Dict[str, Any]) -> Dict[str, Any]:
        required = {"request_id", "gateway_instance_id", "capability_name", "task_id", "status",
                    "summary", "result", "error", "completeness"}
        if (not required.issubset(payload) or type(payload.get("schema_version")) is not int
                or payload["schema_version"] != 1 or payload.get("delivery_state") not in
                ("pending", "recorded", "unknown")):
            raise RobotClientError("invalid_receipt", "响应不是版本1工具执行回执；不能据此判断执行成功。")
        if (not isinstance(payload["gateway_instance_id"], str) or not payload["gateway_instance_id"]
                or not isinstance(payload["summary"], str) or not isinstance(payload["completeness"], dict)):
            raise RobotClientError("invalid_receipt", "回执缺少实例身份、摘要或完整性信息。")
        status = payload.get("status")
        if status is not None and (not isinstance(status, dict) or status.get("state") not in
                                   TERMINAL_STATES.union(("accepted", "running"))):
            raise RobotClientError("invalid_receipt", "任务状态缺失或不受支持；请核对完整结果。")
        if payload.get("task_id") and status is None and payload.get("delivery_state") == "recorded":
            raise RobotClientError("invalid_receipt", "异步任务回执缺少任务状态。")
        return payload

    def call(self, request_id: str) -> Dict[str, Any]:
        return self._receipt(self._request("/api/agent/calls/" + quote(request_id, safe="")))

    def result(self, *, request_id: Optional[str] = None,
               task_id: Optional[str] = None) -> Dict[str, Any]:
        if bool(request_id) == bool(task_id):
            raise RobotClientError("invalid_arguments", "必须且只能指定一个请求或任务标识。")
        path = "/api/agent/tasks/" + quote(task_id, safe="") if task_id else "/api/agent/calls/" + quote(request_id, safe="")
        return self._request(path + "/result")

    def task(self, task_id: str) -> Dict[str, Any]:
        return self._receipt(self._request("/api/agent/tasks/" + quote(task_id, safe="")))

    def wait(self, task_id: str, *, timeout_sec: float = 30.0) -> Dict[str, Any]:
        if not math.isfinite(timeout_sec) or not 0 <= timeout_sec <= 30:
            raise RobotClientError("invalid_timeout", "单次等待预算须在0到30秒之间。")
        return self._receipt(self._request("/api/agent/tasks/" + quote(task_id, safe="") + "/wait?" +
                                          urlencode({"timeout_sec": timeout_sec}),
                                          timeout=max(self.timeout, timeout_sec + 5)))

    def cancel(self, task_id: str) -> Dict[str, Any]:
        return self._receipt(self._request("/api/agent/tasks/" + quote(task_id, safe="") + "/cancel", method="POST", body={}))

    def feedback(self, *, after=None, request_id=None, task_id=None, capability_name=None):
        """建立就绪的订阅后再调用工具；上下文退出只关闭订阅。"""
        return FeedbackSubscription(self, after=after, request_id=request_id,
                                    task_id=task_id, capability_name=capability_name)

    def download_artifact(self, artifact_id: str, output: Path, *,
                          max_bytes: int = ARTIFACT_LIMIT) -> Dict[str, Any]:
        """按同一网关制品标识下载，原子发布且不覆盖已有文件。"""
        if max_bytes <= 0:
            raise RobotClientError("invalid_arguments", "制品下载上限必须为正数。")
        artifact_path = "/artifacts/" + quote(artifact_id, safe="")
        meta = self._request(artifact_path + "/meta")
        # 当前网关直接返回制品对象，同时允许显式artifact包装。
        artifact = meta.get("artifact", meta)
        if not isinstance(artifact, dict) or artifact.get("artifact_id") != artifact_id:
            raise RobotClientError("invalid_artifact", "制品元数据标识与请求不一致。")
        output = Path(output).expanduser().absolute()
        if output.exists() or output.is_symlink():
            raise RobotClientError("output_exists", "目标文件已存在，请选择新路径：" + str(output))
        output.parent.mkdir(parents=True, exist_ok=True)
        headers = {"Accept": "application/octet-stream"}
        if self.token:
            headers["Authorization"] = "Bearer " + self.token
        request = Request(self.base_url + artifact_path, headers=headers)
        temp_path = None
        digest = hashlib.sha256()
        total = 0
        try:
            with self._opener.open(request, timeout=self.timeout) as response:
                media_type = response.headers.get_content_type()
                declared = response.headers.get("Content-Length")
                expected_length = int(declared) if declared is not None else None
                with tempfile.NamedTemporaryFile(prefix=".robot-artifact-", dir=str(output.parent), delete=False) as stream:
                    temp_path = Path(stream.name)
                    while True:
                        chunk = response.read(min(65536, max_bytes - total + 1))
                        if not chunk:
                            break
                        total += len(chunk)
                        if total > max_bytes:
                            raise RobotClientError("artifact_too_large", "制品超过下载上限，未发布本地文件。")
                        digest.update(chunk)
                        stream.write(chunk)
                    stream.flush()
                    os.fsync(stream.fileno())
            if expected_length is not None and total != expected_length:
                raise RobotClientError("artifact_unavailable", "制品传输被截断，未发布本地文件。")
            expected = artifact.get("sha256")
            if expected and digest.hexdigest() != expected:
                raise RobotClientError("artifact_digest_mismatch", "制品摘要校验失败，未发布本地文件。")
            if artifact.get("size_bytes") is not None and total != artifact["size_bytes"]:
                raise RobotClientError("artifact_size_mismatch", "制品大小与元数据不一致，未发布本地文件。")
            # 硬链接以排他方式发布同目录临时文件，避免覆盖竞争中的用户文件。
            os.link(str(temp_path), str(output))
        except HTTPError as exc:
            exc.close()
            raise RobotClientError("artifact_unavailable", "制品内容不可读取，HTTP状态%d。" % exc.code) from exc
        except (URLError, TimeoutError, socket.timeout, ConnectionError, HTTPException) as exc:
            raise RobotClientError("artifact_unavailable", "制品下载未完成。") from exc
        except FileExistsError as exc:
            raise RobotClientError("output_exists", "目标文件已存在，未覆盖。") from exc
        finally:
            if temp_path is not None:
                temp_path.unlink(missing_ok=True)
        return {"artifact_id": artifact_id, "path": str(output), "mime_type": media_type,
                "size_bytes": total, "sha256": digest.hexdigest(), "source": artifact,
                "downloaded_at": datetime.now(timezone.utc).isoformat(),
                "visual_input_required": media_type.startswith("image/"),
                "next_action": "使用引擎图片读取工具读取path；下载成功尚不代表模型已经看到图像。"}


class FeedbackSubscription:
    """持续读取推送，断线仅续接订阅；成功处理后才推进游标。"""

    def __init__(self, client, *, after=None, **filters):
        if after and not re.fullmatch(r"[a-f0-9]{32}:[0-9]{1,20}", after):
            raise RobotClientError("invalid_cursor", "反馈续传标识无效。")
        self.client, self.last_event_id = client, after
        self.path = "/api/feedback/stream?" + urlencode({k: v for k, v in filters.items() if v})
        self._closed = threading.Event()
        self._io_lock = threading.RLock()
        self._response = None
        self._frames = None
        self._pending = []
        self._connect()

    def _connect(self):
        headers = {"Accept": "text/event-stream"}
        if self.client.token:
            headers["Authorization"] = "Bearer " + self.client.token
        if self.last_event_id:
            headers["Last-Event-ID"] = self.last_event_id
        try:
            self._response = self.client._opener.open(
                Request(self.client.base_url + self.path, headers=headers), timeout=self.client.timeout)
            if self._response.headers.get_content_type() != "text/event-stream":
                raise RobotClientError("invalid_response", "网关未返回反馈事件流。")
            self._frames = self._read_frames()
            for frame in self._frames:
                if frame["event"] == "feedback.ready":
                    return
                self._pending.append(frame)
                if len(self._pending) > 8:
                    raise RobotClientError("invalid_response", "反馈连接未报告就绪。")
            raise RobotClientError("transport_error", "反馈连接在就绪前关闭。")
        except HTTPError as exc:
            exc.close()
            raise RobotClientError("feedback_http_error", "反馈连接被拒绝。",
                                   details={"http_status": exc.code}) from exc
        except (URLError, TimeoutError, OSError, HTTPException) as exc:
            self._disconnect()
            raise RobotClientError("transport_error", "反馈连接中断。") from exc
        except BaseException:
            self._disconnect()
            raise

    def _read_frames(self):
        data, kind, identity, size = [], "message", None, 0
        while not self._closed.is_set():
            with self._io_lock:
                if self._closed.is_set() or self._response is None:
                    return
                line = self._response.readline(65537)
            if not line:
                return
            size += len(line)
            if size > 65536:
                raise RobotClientError("response_too_large", "单条反馈超过读取上限。")
            text = line.decode("utf-8").rstrip("\r\n")
            if not text:
                if data:
                    payload = _json_loads("\n".join(data).encode("utf-8"))
                    if not isinstance(payload, dict):
                        raise RobotClientError("invalid_response", "反馈须为JSON对象。")
                    yield {"event": kind, "data": payload, "id": identity}
                data, kind, identity, size = [], "message", None, 0
            elif not text.startswith(":"):
                field, _, value = text.partition(":")
                value = value[1:] if value.startswith(" ") else value
                if field == "data":
                    data.append(value)
                elif field == "event":
                    kind = value
                elif field == "id":
                    if not re.fullmatch(r"[a-f0-9]{32}:[0-9]{1,20}", value):
                        raise RobotClientError("invalid_response", "反馈事件含非法续传标识。")
                    identity = value

    def __iter__(self):
        backoff = 0.5
        while not self._closed.is_set():
            try:
                if self._frames is None:
                    self._connect()
                if self._closed.is_set():
                    return
                if self._pending:
                    frame = self._pending.pop(0)
                else:
                    frame = next(self._frames)
                if frame["event"] in {"keepalive", "feedback.ready"}:
                    continue
                yield frame
                # 宿主回调抛错时不会到这里，重连可再次交付未处理消息。
                if frame["id"]:
                    self.last_event_id = frame["id"]
                backoff = 0.5
            except (StopIteration, URLError, TimeoutError, OSError, HTTPException):
                self._disconnect()
                if self._closed.wait(backoff):
                    return
                backoff = min(backoff * 2, 30)
            except RobotClientError as exc:
                if exc.payload["code"] != "transport_error":
                    raise
                self._disconnect()
                if self._closed.wait(backoff):
                    return
                backoff = min(backoff * 2, 30)

    def listen(self, on_message, *, on_gap=None):
        """宿主回调可注入会话并唤醒模型；工具消息仍是观测。"""
        for frame in self:
            if frame["event"] == "feedback.message":
                on_message(frame["data"])
            elif frame["event"] == "feedback.gap":
                if on_gap is None:
                    raise RobotClientError("feedback_gap", frame["data"]["message"], details=frame["data"])
                on_gap(frame["data"])

    def _disconnect(self):
        with self._io_lock:
            response, self._response = self._response, None
            self._frames = None
            if response is not None:
                response.close()

    def close(self):
        self._closed.set()
        response = self._response
        if response is not None:
            # 先唤醒阻塞读取，再关闭文件对象，允许宿主从另一线程回收订阅。
            try:
                response.fp.raw._sock.shutdown(socket.SHUT_RDWR)
            except (AttributeError, OSError):
                pass
        self._disconnect()

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.close()


def receipt_exit_code(payload: Dict[str, Any]) -> int:
    """0表示本次查询/同步调用成功；3表示仍在执行；1表示错误或未确认结果。"""
    if payload.get("error") or payload.get("delivery_state") == "unknown":
        return 1
    result = payload.get("result")
    if isinstance(result, dict) and (result.get("ok") is False or result.get("success") is False
                                     or result.get("error") or result.get("error_payload")):
        return 1
    if "delivery_state" in payload and payload["delivery_state"] not in ("pending", "recorded", "unknown"):
        return 1
    status = payload.get("status")
    state = status.get("state") if isinstance(status, dict) else None
    if state in ("failed", "timeout", "cancelled"):
        return 1
    if state in ("accepted", "running") or payload.get("delivery_state") == "pending":
        return 3
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, epilog="退出码：0查询/同步操作成功；3任务仍在执行；1失败或未知；2输入错误；130客户端中断（任务未自动取消）。")
    parser.add_argument("--url", default=os.environ.get("NUWAX_ROBOT_URL", "http://127.0.0.1:8766"), help="机器人网关基地址。")
    parser.add_argument("--timeout", type=float, default=35.0, help="HTTP超时秒数。")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("info", help="检查智能体接口版本和机器人身份。")
    watch = commands.add_parser("watch", help="持续接收主动反馈，断线续接；不轮询任务。")
    for name in ("after", "request-id", "task-id", "capability-name"):
        watch.add_argument("--" + name)
    tools_parser = commands.add_parser("tools", help="读取真实工具参数和结果结构。")
    tools_parser.add_argument("--name")
    tools_parser.add_argument("--full", action="store_true", help="输出所有工具完整参数结构；默认只列目录。")
    invoke_parser = commands.add_parser("invoke", help="执行工具；不会自动重试动作。")
    invoke_parser.add_argument("tool_name")
    arguments = invoke_parser.add_mutually_exclusive_group()
    arguments.add_argument("--arguments", default=None, help="JSON对象。")
    arguments.add_argument("--arguments-file", type=Path, help="包含JSON对象的UTF-8文件。")
    invoke_parser.add_argument("--request-id", required=True, help="稳定请求标识；丢失回执时查询此标识。")
    call_parser = commands.add_parser("call", help="恢复查看调用记录。")
    call_parser.add_argument("request_id")
    result_parser = commands.add_parser("result", help="读取调用或任务的完整结果。")
    identities = result_parser.add_mutually_exclusive_group(required=True)
    identities.add_argument("--request-id")
    identities.add_argument("--task-id")
    for name in ("task", "wait", "cancel"):
        child = commands.add_parser(name)
        child.add_argument("task_id")
        if name == "wait":
            child.add_argument("--timeout-sec", type=float, default=30.0)
    artifact_parser = commands.add_parser("artifact", help="下载完整制品到工作区，不覆盖原文件。")
    artifact_parser.add_argument("artifact_id")
    artifact_parser.add_argument("--output", type=Path, required=True)
    artifact_parser.add_argument("--max-bytes", type=int, default=ARTIFACT_LIMIT)
    args = parser.parse_args(argv)
    try:
        client = RobotClient(args.url, timeout=args.timeout, token=os.environ.get("NUWAX_ROBOT_TOKEN"))
        if args.command == "info":
            payload = client.info()
        elif args.command == "watch":
            with client.feedback(after=args.after, request_id=args.request_id, task_id=args.task_id,
                                 capability_name=args.capability_name) as subscription:
                for frame in subscription:
                    print(json.dumps(frame, ensure_ascii=False, allow_nan=False), flush=True)
            return 0
        elif args.command == "tools":
            payload = client.tools(args.name)
            if args.name is None and not args.full:
                catalog = []
                for item in payload.get("tools", []):
                    descriptor = item.get("descriptor", item)
                    catalog.append({key: descriptor.get(key) for key in ("name", "display_name", "description", "capability_name")})
                payload = {"tools": catalog, "total": len(catalog), "complete": True,
                           "parameter_schemas_included": False,
                           "next_action": "用tools --name 工具名读取真实参数和结果结构。"}
        elif args.command == "invoke":
            raw = args.arguments_file.read_bytes() if args.arguments_file else (args.arguments or "{}").encode("utf-8")
            params = _json_loads(raw)
            if not isinstance(params, dict):
                raise RobotClientError("invalid_arguments", "工具参数必须是JSON对象。")
            payload = client.invoke(args.tool_name, params, request_id=args.request_id)
        elif args.command == "call":
            payload = client.call(args.request_id)
        elif args.command == "result":
            payload = client.result(request_id=args.request_id, task_id=args.task_id)
        elif args.command == "task":
            payload = client.task(args.task_id)
        elif args.command == "wait":
            payload = client.wait(args.task_id, timeout_sec=args.timeout_sec)
        elif args.command == "cancel":
            payload = client.cancel(args.task_id)
        else:
            payload = client.download_artifact(args.artifact_id, args.output, max_bytes=args.max_bytes)
        print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
        return receipt_exit_code(payload)
    except RobotClientError as exc:
        print(json.dumps({"error": exc.payload}, ensure_ascii=False, indent=2))
        return 1
    except (OSError, ValueError, UnicodeError) as exc:
        print(json.dumps({"error": {"code": "client_input_error", "message": str(exc)}}, ensure_ascii=False))
        return 2
    except KeyboardInterrupt:
        error = {"code": "client_interrupted", "message": "客户端已中断；动作执行情况待核对，机器人任务未自动取消。请按原请求或任务标识查询、取消。"}
        if getattr(args, "request_id", None):
            error["request_id"] = args.request_id
        if getattr(args, "task_id", None):
            error["task_id"] = args.task_id
        print(json.dumps({"error": error}, ensure_ascii=False))
        return 130


if __name__ == "__main__":
    sys.exit(main())
