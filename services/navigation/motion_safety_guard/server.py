"""Unix（本机）套接字边界：解码一次并校验协议输入。"""
from __future__ import annotations

import math
import os
from pathlib import Path
import socket
import socketserver
import threading

from google.protobuf.message import DecodeError

from services.navigation.proto_gen import motion_guard_pb2 as pb
from .constants import WATCHDOG_INTERVAL_SECONDS
from .core import MotionGuardCore
from .framing import FramingError, recv_frame, send_frame
from .go2_actuator import _ONE_SHOT_SPORT_METHODS, _TOGGLE_SPORT_METHODS


def decode_request(payload: bytes) -> pb.GuardRequest:
    request = pb.GuardRequest.FromString(payload)
    operation = request.WhichOneof("op")
    if operation is None:
        raise ValueError("缺少运动操作")
    if operation == "acquire" and request.acquire.source not in {
        pb.SOURCE_NAV, pb.SOURCE_GATEWAY_FOLLOW, pb.SOURCE_GATEWAY_RELATIVE,
    }:
        raise ValueError("租约来源无效")
    if operation == "velocity" and not all(math.isfinite(v) for v in (
        request.velocity.vx_mps, request.velocity.vy_mps, request.velocity.wz_rps,
    )):
        raise ValueError("速度必须是有限数值")
    if operation == "sport":
        sport = request.sport
        command = sport.WhichOneof("command")
        if command == "action":
            action = sport.action.action
            if action not in _ONE_SHOT_SPORT_METHODS and action not in _TOGGLE_SPORT_METHODS and action != pb.SPORT_ACTION_STOP_MOVE:
                raise ValueError("机体动作无效")
            if action not in _TOGGLE_SPORT_METHODS and not sport.action.enabled:
                raise ValueError("一次性机体动作必须 enabled=true")
        elif command == "body_pose":
            if not all(math.isfinite(v) for v in (sport.body_pose.roll_rad, sport.body_pose.pitch_rad, sport.body_pose.yaw_rad)):
                raise ValueError("机身姿态必须是有限数值")
        elif command != "control_mode" or sport.control_mode not in {pb.CONTROL_MODE_HIGH_LEVEL, pb.CONTROL_MODE_LOW_LEVEL}:
            raise ValueError("机体动作或控制模式无效")
    if operation == "setting":
        setting = request.setting
        if not setting.HasField("speed_level") and not setting.HasField("obstacle_avoidance"):
            raise ValueError("至少提供一项机器人设置")
        if setting.HasField("speed_level") and setting.speed_level not in {1, 2, 3}:
            raise ValueError("速度档位必须在 [1, 3] 内")
    return request


class _Handler(socketserver.BaseRequestHandler):
    def finish(self) -> None:
        with self.server.connections_lock:
            self.server.connections.discard(self.request)

    def handle(self) -> None:
        while True:
            try:
                payload = recv_frame(self.request)
            except (FramingError, OSError):
                return
            try:
                request = decode_request(payload)
            except (DecodeError, ValueError) as exc:
                response = pb.GuardResponse(error=pb.Error(code=pb.ERROR_CODE_INVALID_ARGUMENT, message=str(exc)))
            else:
                response = self.server.core.handle(request)
            try:
                send_frame(self.request, response.SerializeToString())
            except (FramingError, OSError):
                return


class _Server(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = False

    def __init__(self, path, handler):
        self.connections = set()
        self.connections_lock = threading.Lock()
        super().__init__(path, handler)

    def process_request(self, request, address) -> None:
        with self.connections_lock:
            self.connections.add(request)
        try:
            super().process_request(request, address)
        except BaseException:
            with self.connections_lock:
                self.connections.discard(request)
            raise


class MotionGuardUnixServer:
    def __init__(self, *, socket_path: Path, core: MotionGuardCore, socket_gid: int) -> None:
        self.socket_path = Path(socket_path)
        self._core = core
        self._socket_gid = socket_gid
        self._server = None
        self._thread = None
        self._watchdog = None
        self._stopping = threading.Event()

    def start(self) -> None:
        self._core.initialize_zero()
        self.socket_path.parent.mkdir(parents=True, exist_ok=True)
        self.socket_path.unlink(missing_ok=True)
        server = _Server(str(self.socket_path), _Handler)
        server.core = self._core
        try:
            os.chown(self.socket_path, os.geteuid(), self._socket_gid)
            os.chmod(self.socket_path, 0o660)
        except BaseException:
            server.server_close()
            self.socket_path.unlink(missing_ok=True)
            raise
        self._server = server
        self._stopping.clear()
        self._thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
        self._watchdog = threading.Thread(target=self._watch, name="motion-guard-watchdog", daemon=True)
        self._thread.start()
        self._watchdog.start()

    def _watch(self) -> None:
        while not self._stopping.wait(WATCHDOG_INTERVAL_SECONDS):
            self._core.tick()

    def stop(self) -> None:
        self._stopping.set()
        if self._server is not None:
            self._server.shutdown()
            with self._server.connections_lock:
                connections = tuple(self._server.connections)
            for connection in connections:
                try:
                    connection.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
            self._server.server_close()
            self._thread.join()
            self._watchdog.join()
            self._server = None
            try:
                self._core.initialize_zero()
            finally:
                self.socket_path.unlink(missing_ok=True)
