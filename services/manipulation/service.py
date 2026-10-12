"""跟踪同一设备任务直到真实终态；取消等待匹配任务暂停证明。"""
from __future__ import annotations

import math
import threading
import time
from contextlib import contextmanager

from core.task_manager import TaskSafetyBarrierError
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

    @staticmethod
    def parameters(arguments):
        instruction = arguments.get('instruction')
        timeout, steps = arguments.get('timeout_sec', 120), arguments.get('max_steps', 1440)
        if not isinstance(instruction, str) or not instruction.strip() or len(instruction) > 2000:
            raise ValueError("instruction须为1至2000字符的自然语言指令")
        if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or not 1 <= timeout <= 1800:
            raise ValueError("timeout_sec须在1至1800秒之间")
        if type(steps) is not int or not 1 <= steps <= 10000:
            raise ValueError("max_steps须为1至10000的整数")
        return {'instruction': instruction.strip(), 'timeout_sec': float(timeout), 'max_steps': steps}

    def bind(self):
        if not self.config.enabled or self.backend is None:
            raise DeviceError("操作任务未配置", code="manipulation_unavailable")
        with self._operation():
            return self.backend.discover()

    def _stop(self, binding, task_id):
        deadline = time.monotonic() + self.config.stop_timeout_sec
        try:
            self.backend.cancel(binding, task_id, timeout=min(self.config.http_timeout_sec, self.config.stop_timeout_sec))
            while time.monotonic() < deadline:
                result = self.backend.status(binding, task_id, timeout=min(self.config.http_timeout_sec, max(.001, deadline-time.monotonic())))
                if result['state'] in {'succeeded', 'failed', 'cancelled'} and result.get('physics_paused') is True:
                    return {**binding.fields(), 'device_task_id': task_id, 'state': result['state'],
                            'physics_paused': True, 'stop_semantics': 'device_physics_paused_not_joint_zero_velocity'}
                time.sleep(min(self.config.poll_interval_sec, max(0, deadline-time.monotonic())))
            raise DeviceError("取消后未在时限内收到匹配任务终态及physics_paused")
        except Exception as error:
            raise TaskSafetyBarrierError("外部操作停止未确认: " + str(error), code="manipulation_stop_unconfirmed",
                details={**binding.fields(), 'device_task_id': task_id, 'physics_paused': False}) from error

    def run(self, context, binding, parameters):
        with self._operation():
            return self._run_task(context, binding, parameters)

    def _run_task(self, context, binding, parameters):
        task_id = None
        stop_proof = None
        def interrupted():
            return context.is_cancel_requested() or context.is_timed_out()
        def finish_interrupt():
            nonlocal stop_proof
            stop_proof = self._stop(binding, task_id) if task_id else {**binding.fields(), 'device_action_started': False}
            context.acknowledge_interrupt_barrier(stop_proof)
            context.ensure_active()
        try:
            if interrupted():
                finish_interrupt()
            context.update(stage='manipulation_submitting', message='正在向绑定设备提交操作指令。')
            accepted = self.backend.start(binding, **parameters)
            task_id = accepted['task_id']
            context.update_metadata({'device_task_id': task_id, 'device_binding': binding.fields()})
            while True:
                if interrupted():
                    finish_interrupt()
                result = self.backend.status(binding, task_id)
                state = result['state']
                context.update(stage='manipulation_'+state, message='设备操作状态: '+state,
                    payload={'device_task_id': task_id, 'device_status': result})
                if state == 'succeeded':
                    physical = result.get('result')
                    if not isinstance(physical, dict) or physical.get('physics_success') is not True:
                        raise DeviceError("设备声称完成但缺少physics_success真实判定")
                    if not physical.get('model_source'):
                        raise DeviceError('设备成功结果缺少实际动作模型来源')
                    context.ensure_active()
                    proof = {**binding.fields(), 'device_task_id': task_id, 'device_status': result}
                    if context.commit_success(proof):
                        return proof
                    finish_interrupt()
                if state in {'failed', 'cancelled', 'idle'}:
                    raise DeviceError('设备操作未成功: '+str(result.get('error') or state))
                time.sleep(self.config.poll_interval_sec)
        except TaskSafetyBarrierError:
            raise
        except Exception as error:
            if task_id is None:
                if isinstance(error, DeviceError) and error.uncertain:
                    raise TaskSafetyBarrierError('提交回执不确定，无法关联设备任务并确认停止', code='manipulation_stop_unconfirmed',
                        details={**binding.fields(), 'device_task_id': None, 'cause': str(error)}) from error
                if interrupted():
                    context.acknowledge_interrupt_barrier({**binding.fields(), 'device_action_started': False})
            else:
                if stop_proof is None:
                    stop_proof = self._stop(binding, task_id)
                context.update_metadata({'manipulation_stop_proof': stop_proof})
                if interrupted():
                    context.acknowledge_interrupt_barrier(stop_proof)
            raise
