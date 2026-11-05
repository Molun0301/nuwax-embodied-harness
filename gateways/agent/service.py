"""智能体工具入口与任务账本，状态由任务执行器拥有。"""
from __future__ import annotations

import asyncio
import threading
import time

from core.errors import GatewayError
from gateways.agent.feedback import TERMINAL_STATES, task_record
from gateways.invoke import invoke
from gateways.serialization import to_jsonable


class AgentToolService:
    """工具调用共用 invoke，账本直接读取 TaskRunner（任务执行器）。"""

    def __init__(self, platform, access_manager=None) -> None:
        self.platform = platform
        self._closed = False
        self._waiters = set()
        self._lock = threading.Lock()

    def discover(self) -> dict:
        return {"robot_name": self.platform.robot.manifest.robot_name,
                "paths": {"tools": "/api/agent/tools", "invoke": "/api/agent/invoke",
                          "tasks": "/api/agent/tasks", "events": "/events/stream",
                          "feedback": "/api/feedback/stream", "artifacts": "/artifacts"},
                "task_retention": "in_memory"}

    def list_tools(self) -> dict:
        return {"tools": [{"name": cap.name, "description": cap.description,
                           "inputSchema": cap.input_schema, "kind": cap.kind}
                          for cap in self.platform.capabilities.agent_tools(
                              self.platform.robot.manifest.robot_name)]}

    def invoke(self, tool_name: str, arguments: dict) -> dict:
        names = {cap.name for cap in self.platform.capabilities.agent_tools(self.platform.robot.manifest.robot_name)}
        if tool_name not in names:
            raise GatewayError("未知智能体工具。", error_code="unknown_capability", http_status=404)
        return to_jsonable(invoke(self.platform, tool_name, arguments, "agent"))

    def list_tasks(self, limit: int = 50) -> dict:
        return {"active_tasks": to_jsonable(self.platform.tasks.active()),
                "history_tasks": to_jsonable(self.platform.tasks.history(limit))}

    def _status(self, task_id):
        status = self.platform.tasks.status(task_id)
        if status is None:
            raise GatewayError("任务不存在或已移出历史。", error_code="task_not_found", http_status=404)
        return status

    def get_task(self, task_id: str) -> dict:
        return task_record(self._status(task_id))

    def get_task_record(self, task_id: str) -> dict:
        return self.get_task(task_id)

    def get_task_result(self, task_id: str) -> dict:
        return self.get_task(task_id)

    def cancel_task(self, task_id: str) -> dict:
        return to_jsonable(invoke(self.platform, "cancel_task", {"task_id": task_id}, "agent"))

    async def wait_task_async(self, task_id: str, timeout_sec: float = 30) -> dict:
        """事件只负责唤醒，返回值始终读任务执行器。"""
        loop = asyncio.get_running_loop()
        changed = asyncio.Event()

        def receive(event):
            if event.task_id == task_id:
                loop.call_soon_threadsafe(changed.set)

        subscription = self.platform.events.subscribe(receive, event_types=("task.updated",))
        waiter = (loop, changed)
        with self._lock:
            self._waiters.add(waiter)
        deadline = time.monotonic() + timeout_sec
        try:
            while True:
                changed.clear()
                status = self._status(task_id)
                if str(status.state.value) in TERMINAL_STATES or self._closed:
                    return task_record(status)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return task_record(status)
                try:
                    await asyncio.wait_for(changed.wait(), timeout=remaining)
                except asyncio.TimeoutError:
                    return self.get_task(task_id)
        finally:
            self.platform.events.unsubscribe(subscription)
            with self._lock:
                self._waiters.discard(waiter)

    def close(self) -> None:
        with self._lock:
            self._closed = True
            waiters = tuple(self._waiters)
        for loop, changed in waiters:
            loop.call_soon_threadsafe(changed.set)
