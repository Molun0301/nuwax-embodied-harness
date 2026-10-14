"""网页和独立命令行共用同一配置拥有方；令牌不转换为文件摘要。"""
from __future__ import annotations

import json
import os
import signal
import socket
import subprocess
import tempfile
import time
from pathlib import Path

from . import PACKAGE_ROOT
from .common import ManagerError, file_lock, process_alive, read_json
from .runtime import clean_environment

MAX_MESSAGE = 8 * 1024 * 1024


class ConfigurationClient:
    def __init__(self, home, instance_id):
        self.home = Path(home).resolve()
        self.state_path = self.home / "config/profile-owner.json"
        self.lock_path = self.home / "config/profile-owner-start.lock"
        address = self.home / "config/profile-owner.sock"
        if len(os.fsencode(address)) >= 100:
            directory = Path(tempfile.gettempdir()) / ("neh-config-" + str(os.getuid()))
            directory.mkdir(mode=0o700, exist_ok=True)
            info = directory.lstat()
            if directory.is_symlink() or info.st_uid != os.getuid() or info.st_mode & 0o077:
                raise ManagerError("config_socket_unsafe", "配置通信临时目录权限不安全。", 500)
            address = directory / (instance_id + ".sock")
        self.address = address

    def _request(self, prefix, operation, **values):
        payload = dict(values, prefix=str(prefix), operation=operation)
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(30)
            connection.connect(str(self.address))
            connection.sendall(json.dumps(payload, ensure_ascii=False).encode() + b"\n")
            with connection.makefile("rb") as reader:
                raw = reader.readline(MAX_MESSAGE + 1)
        if len(raw) > MAX_MESSAGE or not raw.endswith(b"\n"):
            raise ManagerError("config_unavailable", "配置拥有方没有返回完整结果。", 503)
        response = json.loads(raw)
        if response.get("error"):
            error = response["error"]
            raise ManagerError(error["code"], error["message"], error.get("status", 500))
        return response["result"]

    def close(self):
        """停止本安装的配置拥有方；已有编辑令牌随后明确失效。"""
        with file_lock(self.lock_path, blocking=True):
            self._stop_locked()

    def _stop_locked(self):
        record = read_json(self.state_path, {})
        identity = record.get("process")
        if process_alive(identity):
            try:
                os.kill(identity["pid"], signal.SIGTERM)
            except ProcessLookupError:
                pass
            deadline = time.monotonic() + 35
            while process_alive(identity) and time.monotonic() < deadline:
                time.sleep(0.1)
            if process_alive(identity):
                raise ManagerError("config_owner_busy", "配置拥有方仍在保存，暂不能切换业务版本。", 409)
        self.address.unlink(missing_ok=True)
        self.state_path.unlink(missing_ok=True)

    def _ensure(self, prefix):
        prefix = Path(prefix).resolve()
        with file_lock(self.lock_path, blocking=True):
            record = read_json(self.state_path, {})
            if process_alive(record.get("process")) and record.get("prefix") == str(prefix) and record.get("protocol") == 1:
                try:
                    self._request(prefix, "ping")
                    return
                except (OSError, ValueError, ManagerError):
                    pass
            self._stop_locked()
            if prefix.parent != self.home / "releases" or (self.home / "current").resolve() != prefix:
                raise ManagerError("config_conflict", "业务版本已变化，请刷新后重试。", 409)
            environment = {key: value for key, value in clean_environment(prefix).items() if not key.startswith("NEH_MANAGER_WORKER_")}
            log_path = self.home / "logs/config-owner.log"
            descriptor = os.open(str(log_path), os.O_CREAT | os.O_APPEND | os.O_WRONLY, 0o600)
            with os.fdopen(descriptor, "ab") as log:
                process = subprocess.Popen([str(prefix / "venv/bin/python"), str(PACKAGE_ROOT / "scripts/device_manager/config_owner.py"),
                    "--home", str(self.home), "--prefix", str(prefix), "--socket", str(self.address)], cwd=prefix / "source",
                    env=environment, stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline and process.poll() is None:
                record = read_json(self.state_path, {})
                if record.get("prefix") == str(prefix) and process_alive(record.get("process")):
                    try:
                        self._request(prefix, "ping")
                        return
                    except (OSError, ValueError):
                        pass
                time.sleep(0.05)
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            raise ManagerError("config_owner_unavailable", "业务版本的共同配置拥有方无法启动；请核对版本与设备本地config-owner.log。", 503)

    def request(self, prefix, operation, **values):
        self._ensure(prefix)
        try:
            return self._request(prefix, operation, **values)
        except (OSError, ValueError) as exc:
            # 保存不自动重试：响应丢失时需重新读取，避免重复或越过陈旧检查。
            raise ManagerError("config_unavailable", "配置拥有方连接中断，请重新读取配置后重试。", 503) from exc
