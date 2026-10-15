"""可选设备密码；密码只存盐化摘要，不进入命令参数和任务日志。"""
from __future__ import annotations

import hashlib
import hmac
import secrets

from .common import ManagerError, atomic_json, file_lock, read_json

ITERATIONS = 310000


class Authentication:
    def __init__(self, engine):
        self.engine = engine
        self.path = engine.manager_home / "config/password.json"

    def enabled(self):
        return bool(read_json(self.path, {}).get("digest"))

    def epoch(self):
        return read_json(self.path, {}).get("epoch", "initial")

    def verify(self, password):
        state = read_json(self.path, {})
        if not state.get("digest"):
            return True
        actual = hashlib.pbkdf2_hmac("sha256", str(password).encode(), bytes.fromhex(state["salt"]), state["iterations"]).hex()
        return hmac.compare_digest(actual, state["digest"])

    def change(self, password, current_password=""):
        if not isinstance(password, str) or len(password) > 1024:
            raise ManagerError("invalid_password", "密码必须是最长1024字符的文本。")
        if password and len(password) < 8:
            raise ManagerError("weak_password", "密码至少需要8个字符。")
        with file_lock(self.engine.lock_path):
            if not self.verify(current_password):
                raise ManagerError("wrong_password", "当前密码不正确。", 401)
            salt = secrets.token_bytes(16)
            state = {"epoch": secrets.token_hex(16)}
            if password:
                state.update(salt=salt.hex(), iterations=ITERATIONS,
                             digest=hashlib.pbkdf2_hmac("sha256", password.encode(), salt, ITERATIONS).hex())
            atomic_json(self.path, state)
        return {"password_enabled": bool(password)}
