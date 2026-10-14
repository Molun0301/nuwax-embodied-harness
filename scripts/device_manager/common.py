"""持久文件、进程身份和跨进程事务基础。"""
from __future__ import annotations

import contextlib
import fcntl
import hashlib
import json
import os
import re
import time
import uuid
from pathlib import Path


class ManagerError(Exception):
    def __init__(self, code, message, status=400, details=None):
        super().__init__(message)
        self.code, self.message, self.status, self.details = code, message, status, details

    def as_dict(self):
        result = {"code": self.code, "message": self.message}
        if self.details is not None:
            result["details"] = self.details
        return result


def read_json(path, default=None):
    try:
        with open(path, encoding="utf-8") as handle:
            return json.load(handle)
    except FileNotFoundError:
        return default
    except (ValueError, OSError) as exc:
        raise ManagerError("state_corrupt", "持久记录无法读取：" + Path(path).name, 500) from exc


def atomic_json(path, value):
    atomic_write(path, json.dumps(value, ensure_ascii=False, indent=2).encode("utf-8"))


def atomic_write(path, content, mode=0o600):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    temporary = path.with_name("." + path.name + "." + uuid.uuid4().hex)
    try:
        descriptor = os.open(str(temporary), os.O_WRONLY | os.O_CREAT | os.O_EXCL, mode)
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(str(temporary), str(path))
        fsync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)


def fsync_directory(path):
    descriptor = os.open(str(path), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


@contextlib.contextmanager
def file_lock(path, blocking=False):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    with open(path, "a") as handle:
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | (0 if blocking else fcntl.LOCK_NB))
        except BlockingIOError as exc:
            raise ManagerError("busy", "已有管理操作正在执行，请等待当前任务完成。", 409) from exc
        try:
            yield
        finally:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)


def process_identity(pid):
    try:
        raw = Path("/proc/{}/stat".format(int(pid))).read_text()
        fields = raw[raw.rfind(")") + 2:].split()
        if fields[0] == "Z":
            return None
        return {"pid": int(pid), "start": fields[19], "boot": Path("/proc/sys/kernel/random/boot_id").read_text().strip()}
    except (OSError, ValueError, IndexError):
        return None


def process_alive(identity):
    return bool(identity and process_identity(identity.get("pid", 0)) == identity)


def digest_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def identifier(value):
    value = str(value)
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._+-]{0,127}", value) or value in (".", ".."):
        raise ManagerError("invalid_identifier", "非法的版本或任务标识。")
    return value


def timestamp():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def compare_versions(candidate, current, candidate_published=None, current_published=None):
    """可识别版本按语义排序；未知格式仅在双方有发布时间时比较。"""
    def parsed(value):
        match = re.fullmatch(r"v?(\d+)\.(\d+)(?:\.(\d+))?(?:-([0-9A-Za-z.-]+))?(?:\+[0-9A-Za-z.-]+)?", str(value or ""))
        if not match:
            return None
        core = tuple(int(item or 0) for item in match.groups()[:3])
        suffix = match.group(4)
        pre = () if suffix is None else tuple((0, int(part)) if part.isdigit() else (1, part) for part in suffix.split("."))
        return core, suffix is None, pre
    first, second = parsed(candidate), parsed(current)
    if first is not None and second is not None:
        return (first > second) - (first < second)
    if candidate == current:
        return 0
    if candidate_published and current_published:
        import datetime
        try:
            first = datetime.datetime.fromisoformat(str(candidate_published).replace("Z", "+00:00"))
            second = datetime.datetime.fromisoformat(str(current_published).replace("Z", "+00:00"))
            return (first > second) - (first < second)
        except (ValueError, TypeError):
            pass
    return None
