#!/usr/bin/env python3
"""机器人智能体接口客户端，可单文件复制到智能体工作区。"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlencode
from urllib.request import ProxyHandler, Request, build_opener

TERMINAL_STATES = frozenset(("succeeded", "failed", "cancelled", "timed_out"))


class RobotClientError(RuntimeError):
    def __init__(self, code, message, *, details=None):
        super().__init__(message)
        self.payload = {"code": code, "message": message, "details": details or {}}


class RobotClient:
    def __init__(self, base_url: str, *, timeout: float = 35.0, token=None):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.token = token
        self._opener = build_opener(ProxyHandler({}))

    def _open(self, path, *, method="GET", body=None, timeout=None, headers=None):
        headers = {"Accept": "application/json", **(headers or {})}
        if self.token:
            headers["Authorization"] = "Bearer " + self.token
        data = None
        if body is not None:
            headers["Content-Type"] = "application/json"
            data = json.dumps(body, ensure_ascii=False, allow_nan=False).encode()
        try:
            return self._opener.open(Request(self.base_url + path, data=data, method=method, headers=headers),
                                     timeout=self.timeout if timeout is None else timeout)
        except HTTPError as error:
            with error:
                payload = json.load(error)
            detail = payload["error"]
            raise RobotClientError(detail["code"], detail["message"], details=detail.get("details")) from error
        except (URLError, OSError) as error:
            raise RobotClientError("transport_error", str(error)) from error

    def _request(self, path, **kwargs):
        with self._open(path, **kwargs) as response:
            return json.load(response)

    def info(self):
        return self._request("/api/agent")

    def tools(self, name=None):
        payload = self._request("/api/agent/tools")
        if name is None:
            return payload
        for item in payload["tools"]:
            if item["name"] == name:
                return {"tool": item}
        raise RobotClientError("unknown_capability", "工具不存在：" + name)

    def invoke(self, tool_name, arguments=None):
        return self._request("/api/agent/invoke", method="POST",
                             body={"tool_name": tool_name, "arguments": {} if arguments is None else arguments})

    def task(self, task_id):
        return self._request("/api/agent/tasks/" + quote(task_id, safe=""))

    def result(self, *, task_id):
        return self._request("/api/agent/tasks/" + quote(task_id, safe="") + "/result")

    def wait(self, task_id, *, timeout_sec=30):
        return self._request("/api/agent/tasks/" + quote(task_id, safe="") + "/wait?" +
                             urlencode({"timeout_sec": timeout_sec}), timeout=max(self.timeout, timeout_sec + 5))

    def cancel(self, task_id):
        return self._request("/api/agent/tasks/" + quote(task_id, safe="") + "/cancel", method="POST", body={})

    def feedback(self, *, task_id=None, capability_name=None):
        return FeedbackSubscription(self, task_id=task_id, capability_name=capability_name)

    def download_artifact(self, artifact_id, output: Path):
        path = "/artifacts/" + quote(artifact_id, safe="")
        metadata = self._request(path + "/meta")
        with self._open(path) as response, Path(output).open("xb") as target:
            while data := response.read(1024 * 1024):
                target.write(data)
        return {"artifact": metadata, "path": str(output)}


class FeedbackSubscription:
    """订阅任务状态变化；关闭连接不取消机器人任务。"""
    def __init__(self, client, **filters):
        self._response = client._open("/api/feedback/stream?" + urlencode({k: v for k, v in filters.items() if v}),
                                      headers={"Accept": "text/event-stream"})

    def __iter__(self):
        event, data = "message", []
        for raw in self._response:
            line = raw.decode().rstrip("\r\n")
            if not line:
                if data:
                    yield {"event": event, "data": json.loads("\n".join(data))}
                event, data = "message", []
            elif line.startswith("event:"):
                event = line[6:].strip()
            elif line.startswith("data:"):
                data.append(line[5:].lstrip())

    def close(self):
        self._response.close()

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.close()


def result_exit_code(payload):
    status = payload.get("task", payload)
    if status.get("state") in {"running", "cancelling"}:
        return 3
    return 1 if payload.get("error") or status.get("state") in {"failed", "cancelled", "timed_out"} else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default=os.environ.get("NUWAX_ROBOT_URL", "http://127.0.0.1:8766"))
    parser.add_argument("--timeout", type=float, default=35)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("info")
    tools = commands.add_parser("tools")
    tools.add_argument("--name")
    tools.add_argument("--full", action="store_true")
    invoke = commands.add_parser("invoke")
    invoke.add_argument("tool_name")
    arguments = invoke.add_mutually_exclusive_group()
    arguments.add_argument("--arguments", default="{}")
    arguments.add_argument("--arguments-file", type=Path)
    for name in ("task", "wait", "cancel"):
        child = commands.add_parser(name)
        child.add_argument("task_id")
        if name == "wait":
            child.add_argument("--timeout-sec", type=float, default=30)
    result = commands.add_parser("result")
    result.add_argument("--task-id", required=True)
    watch = commands.add_parser("watch")
    watch.add_argument("--task-id")
    watch.add_argument("--capability-name")
    artifact = commands.add_parser("artifact")
    artifact.add_argument("artifact_id")
    artifact.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    client = RobotClient(args.url, timeout=args.timeout, token=os.environ.get("NUWAX_ROBOT_TOKEN"))
    try:
        if args.command == "invoke":
            raw = args.arguments_file.read_text() if args.arguments_file else args.arguments
            payload = client.invoke(args.tool_name, json.loads(raw))
        elif args.command == "info":
            payload = client.info()
        elif args.command == "tools":
            payload = client.tools(args.name)
        elif args.command == "result":
            payload = client.result(task_id=args.task_id)
        elif args.command == "wait":
            payload = client.wait(args.task_id, timeout_sec=args.timeout_sec)
        elif args.command in {"task", "cancel"}:
            payload = getattr(client, args.command)(args.task_id)
        elif args.command == "watch":
            with client.feedback(task_id=args.task_id, capability_name=args.capability_name) as subscription:
                for frame in subscription:
                    print(json.dumps(frame, ensure_ascii=False), flush=True)
            return 0
        else:
            payload = client.download_artifact(args.artifact_id, args.output)
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return result_exit_code(payload)
    except RobotClientError as error:
        print(json.dumps({"error": error.payload}, ensure_ascii=False))
        return 1
    except (ValueError, OSError) as error:
        print(json.dumps({"error": {"code": "client_error", "message": str(error)}}, ensure_ascii=False))
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
