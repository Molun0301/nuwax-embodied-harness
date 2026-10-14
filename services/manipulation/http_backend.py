"""配置化HTTP设备边界，不依赖具体仿真器或模型版本。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict
from urllib.parse import urlsplit

import httpx


class DeviceError(RuntimeError):
    def __init__(self, message, *, code="manipulation_device_error", uncertain=False):
        super().__init__(message)
        self.code, self.uncertain = code, uncertain

    def to_payload(self):
        return {"code": self.code, "message": str(self), "uncertain": self.uncertain}


@dataclass(frozen=True)
class DeviceBinding:
    session: str
    epoch: int

    def fields(self):
        return {"session": self.session, "epoch": self.epoch}


class HttpManipulationBackend:
    OPERATIONS = frozenset({"manipulate", "manipulation_status", "cancel_manipulation"})

    def __init__(self, base_url: str, *, timeout_sec: float = 3.0):
        parsed = urlsplit(base_url)
        if parsed.scheme not in {"http", "https"} or not parsed.netloc or parsed.query or parsed.fragment:
            raise ValueError("操作设备地址必须是有效HTTP(S)基础地址")
        self.timeout_sec = timeout_sec
        self.client = httpx.Client(base_url=base_url.rstrip('/')+'/', timeout=timeout_sec, trust_env=False)

    def close(self):
        self.client.close()

    def _request(self, method, path, *, body=None, timeout=None):
        try:
            response = self.client.request(method, path, json=body, timeout=timeout or self.timeout_sec)
            response.raise_for_status()
            value = response.json()
            if not isinstance(value, dict):
                raise ValueError("回执必须为对象")
            return value
        except httpx.HTTPStatusError as error:
            # 明确4xx拒绝不会被当成受理；5xx及连接中断可能发生在设备执行后。
            raise DeviceError(f"设备HTTP请求失败: {error.response.status_code}",
                uncertain=method == 'POST' and error.response.status_code >= 500) from error
        except (httpx.HTTPError, ValueError) as error:
            raise DeviceError(f"设备回执不可用: {error}", uncertain=method == 'POST') from error

    @staticmethod
    def binding(status):
        session, epoch = status.get('session'), status.get('epoch')
        if not isinstance(session, str) or not session or type(epoch) is not int or epoch < 0:
            raise DeviceError("设备缺少有效session/epoch", code="manipulation_identity_invalid")
        return DeviceBinding(session, epoch)

    def discover(self):
        caps = self._request('GET', 'api/v1/capabilities')
        operations = caps.get('operations')
        if not isinstance(operations, list) or not all(isinstance(op,str) for op in operations) or not self.OPERATIONS.issubset(operations):
            raise DeviceError("当前设备未提供完整操作任务/查询/取消能力", code="manipulation_unavailable")
        status = self._request('GET', 'api/v1/status')
        if status.get('state') not in {'running', 'paused'}:
            raise DeviceError("操作设备尚未运行", code="manipulation_unavailable")
        return self.binding(status)

    def control(self, op, params, binding, *, task_id=None, timeout=None):
        value = self._request('POST', 'api/v1/control', body={'op': op, 'params': params, **binding.fields()}, timeout=timeout)
        try:
            returned_binding = self.binding(value)
        except DeviceError as error:
            raise DeviceError(str(error), code=error.code, uncertain=True) from error
        if returned_binding != binding or (task_id is not None and value.get('task_id') != task_id):
            raise DeviceError("设备返回了不同任务或运行代次", code="manipulation_identity_mismatch", uncertain=True)
        return value

    def start(self, binding, instruction, timeout_sec, max_steps):
        value = self.control('manipulate', {'instruction': instruction, 'timeout_sec': timeout_sec, 'max_steps': max_steps}, binding)
        if value.get('accepted') is not True or not isinstance(value.get('task_id'), str) or not value['task_id']:
            raise DeviceError("操作受理回执缺少明确accepted/task_id", uncertain=True)
        return value

    def status(self, binding, task_id, *, timeout=None):
        value = self.control('manipulation_status', {'task_id': task_id}, binding, task_id=task_id, timeout=timeout)
        if value.get('state') not in {'idle', 'running', 'succeeded', 'failed', 'cancelled'}:
            raise DeviceError("未知设备任务状态", uncertain=True)
        return value

    def cancel(self, binding, task_id, *, timeout=None):
        return self.control('cancel_manipulation', {'task_id': task_id}, binding, task_id=task_id, timeout=timeout)
