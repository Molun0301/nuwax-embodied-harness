"""unitree-sim/1 的设备连接与运动执行；不导入仿真平台实现。"""
from __future__ import annotations
import asyncio
import logging
from threading import RLock, Thread
import time
from urllib.parse import urlsplit, urlunsplit

import httpx
import msgpack
import numpy as np
import websockets

from contracts.geometry import Twist
from contracts.motion_feedback import MotionExecutionFeedback
from contracts.motion_limits import MotionCommandLimits
from .clock import source_timestamp

LOGGER = logging.getLogger(__name__)


def capabilities(base_url, model):
    with httpx.Client(trust_env=False) as client:
        response = client.get(base_url.rstrip('/') + '/api/v1/capabilities')
        response.raise_for_status()
        result = response.json()
    if result['protocol'] != 'unitree-sim/1' or result['backend']['robot'] != model:
        raise ValueError('设备协议或当前机器人型号与方案不一致。')
    return result['backend']


class Device:
    provider_name = 'unitree_sim_device'
    provider_version = '1'

    def __init__(self, base_url, model, frame_id):
        self.base_url, self.model, self.frame_id = base_url.rstrip('/'), model, frame_id
        self.http = httpx.Client(base_url=self.base_url, trust_env=False)
        self.lock = RLock()
        self.scope = None
        self.backend = None
        self.feedback = None
        self.limits = None
        self.registration = None
        self.error = None
        self.running = False
        self.thread = self.loop = self.task = None

    def bind(self):
        # 显式重新启动才重新绑定设备；不续传或重放旧运动命令。
        if self.http.is_closed:
            self.http = httpx.Client(base_url=self.base_url, trust_env=False)
        self.error = self.registration = self.feedback = self.limits = None
        try:
            self.backend = capabilities(self.base_url, self.model)
            response = self.http.get('/api/v1/status')
            response.raise_for_status()
            status = response.json()
            if status['mode'] != 'physics':
                raise RuntimeError('记录回放不能作为当前物理设备接入。')
            self.scope = (status['session'], status['epoch'])
            if self.scope[0] is None:
                raise RuntimeError('仿真设备没有当前物理运行。')
            self._status(status['robot_status'], status['sim_time_ns'])
        except Exception:
            self.http.close()
            raise

    def start(self, on_event=None, on_failure=None):
        self.bind()
        self.subscribe(on_event, on_failure)

    def subscribe(self, on_event=None, on_failure=None):
        self.running = True
        self.loop = asyncio.new_event_loop()
        self.thread = Thread(target=self._run, args=(on_event, on_failure), name='unitree-sim-events', daemon=True)
        self.thread.start()

    def _run(self, on_event, on_failure):
        async def receive():
            url = urlsplit(self.base_url)
            address = urlunsplit(('wss' if url.scheme == 'https' else 'ws', url.netloc,
                                  url.path.rstrip('/') + '/api/v1/stream', '', ''))
            async with websockets.connect(address, max_size=64 * 1024 * 1024) as socket:
                async for raw in socket:
                    if not isinstance(raw, bytes):
                        continue
                    event = msgpack.unpackb(raw, raw=False)
                    self.accept(event)
                    if on_event:
                        on_event(event)
                if self.running:
                    raise RuntimeError('设备事件流已结束。')
        try:
            self.task = self.loop.create_task(receive())
            self.loop.run_until_complete(self.task)
        except asyncio.CancelledError:
            pass
        except Exception as error:
            with self.lock:
                self.error, self.feedback, self.limits = str(error), None, None
            LOGGER.error('unitree_sim 输入停止：%s', error)
            if on_failure:
                on_failure(str(error))
        finally:
            with self.lock:
                self.running = False
                self.loop.close()
                self.loop = self.task = None

    def accept(self, event):
        if event['protocol'] != 'unitree-sim/1' or (event['session'], event['epoch']) != self.scope:
            raise RuntimeError('设备会话或重置代次已变化，原连接停止。')
        payload = event['payload']
        if event['type'] == 'imu':
            with self.lock:
                if payload['registration']['frame_id'] != self.frame_id:
                    raise RuntimeError('设备参考帧与已安装方案不一致。')
                self.registration = payload['registration']
        elif event['type'] == 'status':
            self._status(payload, event['sim_time_ns'])
        elif event['type'] == 'error':
            raise RuntimeError(str(payload))

    def _status(self, payload, sim_stamp=None):
        with self.lock:
            self.feedback = self.limits = None
            mapping = payload['source_clock_mapping']
            if payload['state'] != 'running' or mapping['mode'] != 'realtime':
                return
            velocity = payload['velocity']
            if self.model == 'g1':
                if velocity['reference_frame_id'] != self.frame_id:
                    raise RuntimeError('设备实际速度参考帧与方案不一致。')
                linear, angular = velocity['reference_linear_mps'], velocity['reference_angular_rps']
                rotation = np.asarray(velocity['command_from_reference_rotation_matrix'], dtype=float)
                origin = np.zeros(3)
                limit_frame = velocity['command_frame_id']
            else:
                if self.registration is None:
                    return
                rotation = np.asarray(self.registration['root_rotation_matrix'], dtype=float)
                origin = np.asarray(self.registration['root_translation_m'], dtype=float)
                root_linear = np.asarray(velocity['body_linear_mps'], dtype=float)
                root_angular = np.asarray(velocity['body_angular_rps'], dtype=float)
                linear = rotation.T @ (root_linear + np.cross(root_angular, origin))
                angular = rotation.T @ root_angular
                limit_frame = 'base_link'
            axes = self.backend['command_limits']
            self.limits = MotionCommandLimits(
                max_linear_x_mps=float(axes['vx_mps']), max_linear_y_mps=float(axes['vy_mps']),
                max_angular_z_rps=float(axes['wz_rps']), source=axes['source'],
                input_frame_id=self.frame_id, limit_frame_id=limit_frame,
                command_from_input_rotation=tuple(tuple(float(x) for x in row[:2]) for row in rotation[:2]),
                input_origin_in_command_x_m=float(origin[0]), input_origin_in_command_y_m=float(origin[1]))
            self.feedback = MotionExecutionFeedback(
                source_timestamp_ns=source_timestamp(payload, payload['sim_time_ns'] if sim_stamp is None else sim_stamp),
                received_monotonic_ns=time.monotonic_ns(),
                frame_id=self.frame_id, linear_x_mps=float(linear[0]), linear_y_mps=float(linear[1]),
                linear_z_mps=float(linear[2]), angular_x_rps=float(angular[0]), angular_y_rps=float(angular[1]),
                angular_z_rps=float(angular[2]), source_id='unitree-sim/%s/%s/%s' % (*self.scope, mapping['clock_id']),
                orientation_wxyz=tuple(float(x) for x in velocity['reference_orientation_wxyz']))

    def is_available(self):
        with self.lock:
            return self.running and self.error is None and self.feedback is not None

    def get_motion_execution_feedback(self):
        with self.lock:
            return self.feedback

    def get_motion_command_limits(self):
        with self.lock:
            return self.limits

    def send_twist(self, twist: Twist):
        with self.lock:
            limits = self.limits
            if not self.is_available() or limits is None or twist.frame_id != self.frame_id:
                raise RuntimeError('设备运动反馈或命令参考帧尚不可用。')
            vx, vy, wz = twist.linear.x, twist.linear.y, twist.angular.z
            limits.validate_velocity(vx, vy, wz)
            (a, b), (c, d) = limits.command_from_input_rotation
            params = dict(vx=a * vx + b * vy + limits.input_origin_in_command_y_m * wz,
                          vy=c * vx + d * vy - limits.input_origin_in_command_x_m * wz,
                          wz=wz, ttl_ms=300)
        self._command('twist', params)

    def _command(self, operation, params):
        response = self.http.post('/api/v1/control', json={
            'op': operation, 'params': params, 'session': self.scope[0], 'epoch': self.scope[1]})
        response.raise_for_status()
        result = response.json()
        if result.get('accepted') is False or result.get('ok') is False:
            raise RuntimeError('设备拒绝控制请求：%s' % result)

    def stop_motion(self):
        self._command('stop', {})

    def close(self):
        with self.lock:
            self.running = False
            if self.loop is not None:
                self.loop.call_soon_threadsafe(lambda: self.task.cancel())
        if self.thread is not None:
            self.thread.join()
        with self.lock:
            self.feedback = self.limits = None
        self.http.close()


def create_motion_control():
    """由现有运动守卫在自己的进程创建，不启动另一套守卫。"""
    from core.config import value
    device = Device(value('unitree_sim.base_url'), value('unitree_sim.model'), value('robot.base_frame'))
    device.start()
    return device
