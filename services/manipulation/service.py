"""用公共任务上下文跟踪设备操作；取消只转发一次设备取消。"""
from __future__ import annotations

import threading
import time
from contextlib import contextmanager

from .http_backend import DeviceError, HttpManipulationBackend


class ManipulationService:
    def __init__(self, config, backend=None):
        self.config = config
        self.backend = backend or (HttpManipulationBackend(config.base_url, timeout_sec=config.http_timeout_sec)
            if config.enabled and config.base_url else None)
        self._condition = threading.Condition()
        self._active_operations = 0
        self._closing = False
        self._transport_closing = False
        self._closed = False

    @contextmanager
    def _operation(self):
        with self._condition:
            if self._closing:
                raise DeviceError("操作服务正在关闭", code="manipulation_unavailable")
            self._active_operations += 1
        try:
            yield
        finally:
            with self._condition:
                self._active_operations -= 1
                self._condition.notify_all()

    def close(self, timeout=2.0):
        """等待任务/发现请求退出再关HTTP；超时保留原客户端和关闭责任。"""
        deadline = time.monotonic() + max(0.0, timeout)
        with self._condition:
            self._closing = True
            while self._active_operations or self._transport_closing:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._condition.wait(remaining)
            if self._closed:
                return True
            self._transport_closing = True
        complete = False
        try:
            complete = self.backend is None or self.backend.close() is not False
            return complete
        finally:
            with self._condition:
                self._closed = complete
                self._transport_closing = False
                self._condition.notify_all()

    def availability(self):
        if not self.config.enabled or self.backend is None:
            return False, "外部操作任务未启用或未配置设备地址"
        # 目录只声明本地接入条件；远端能力、运行状态与代次在 bind 时验证。
        with self._condition:
            if self._closing:
                return False, "操作服务正在关闭"
        return True, "设备已配置，后端能力与运行会话尚未实时验证；执行受理前检查"

    def bind(self):
        if not self.config.enabled or self.backend is None:
            raise DeviceError("操作任务未配置", code="manipulation_unavailable")
        with self._operation():
            return self.backend.discover()

    def run(self, context, binding, parameters):
        with self._operation():
            return self._run_task(context, binding, parameters)

    def _run_task(self, context, binding, parameters):
        task_id = None
        complete = False
        try:
            context.check()
            context.progress(0.0, "manipulation_submitting", "正在向设备提交操作指令。")
            accepted = self.backend.start(binding, **parameters)
            task_id = accepted["task_id"]
            while True:
                context.check()
                result = self.backend.status(binding, task_id)
                context.check()
                state = result["state"]
                context.progress(0.0, "manipulation_" + state, "设备操作状态：" + state,
                    device_task_id=task_id, device_binding=binding.fields(), device_status=result)
                if state == "succeeded":
                    physical = result.get("result")
                    if not isinstance(physical, dict) or physical.get("physics_success") is not True:
                        raise DeviceError("设备声称完成但缺少physics_success真实判定")
                    if not physical.get("model_source"):
                        raise DeviceError("设备成功结果缺少实际动作模型来源")
                    complete = True
                    return {**binding.fields(), "device_task_id": task_id, "device_status": result}
                if state in {"failed", "cancelled", "idle"}:
                    raise DeviceError("设备操作未成功：" + str(result.get("error") or state))
                context.wait(self.config.poll_interval_sec)
        finally:
            if task_id is not None and not complete:
                self.backend.cancel(binding, task_id, timeout=self.config.http_timeout_sec)
