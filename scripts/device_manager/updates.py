"""通道准入、默认提醒与空闲自动更新；仅由管理入口的后台线程轮询。"""
from __future__ import annotations

from datetime import datetime
import json
import random
import re
import time
from urllib.parse import urlsplit

from scripts.neh_source import SourceError, source_version

from .common import ManagerError, atomic_json, file_lock, read_json, timestamp
from .releases import open_source


def channel_url(repository):
    parsed = urlsplit(repository)
    # Python3.8使用同一规则，不依赖额外字符串接口。
    path = parsed.path.strip("/")
    if path.endswith(".git"):
        path = path[:-4]
    parts = path.split("/")
    if parsed.scheme != "https" or parsed.netloc != "github.com" or len(parts) != 2 or any(not re.fullmatch(r"[\w.-]+", part) for part in parts):
        raise ManagerError("channel_unavailable", "main通道需要公开GitHub仓库；其它公开来源请选择branch:分支。", 409)
    return "https://github.com/" + path + "/releases/download/neh-prebuilt-v1/channels-main.json"


def read_channel(repository):
    try:
        with open_source(channel_url(repository)) as response:
            raw = response.read(16385)
        if len(raw) > 16384:
            raise ValueError("通道文件过大")
        value = json.loads(raw)
        if (not isinstance(value, dict) or value.get("schema") != 1 or value.get("channel") != "main"
                or type(value.get("requires_confirmation")) is not bool
                or not isinstance(value.get("minimum_manager_version"), str)):
            raise ValueError("通道结构错误")
        source_version(value["revision"])
        if value["minimum_manager_version"] != source_version(value["revision"]):
            raise ValueError("最低管理版本须对应本次通过检查的完整提交")
        datetime.fromisoformat(value["published_at"].replace("Z", "+00:00"))
        if not isinstance(value.get("version"), str) or not value["version"]:
            raise ValueError("缺少显示版本")
        return value
    except (ManagerError, SourceError, ValueError, TypeError, KeyError) as exc:
        raise ManagerError("channel_unavailable", "main通道尚未就绪或无法读取，请等待云端检查与预编译完成。", 502) from exc


def validate_window(value):
    if value and not re.fullmatch(r"(?:[01][0-9]|2[0-3]):[0-5][0-9]-(?:[01][0-9]|2[0-3]):[0-5][0-9]", value):
        raise ManagerError("invalid_settings", "维护窗口格式为HH:MM-HH:MM，留空表示全天。")
    return value


def in_window(value, now=None):
    if not value:
        return True
    now = now or datetime.now()
    current = now.hour * 60 + now.minute
    start, end = (int(item[:2]) * 60 + int(item[3:]) for item in value.split("-"))
    return start <= current < end if start < end else (current >= start or current < end) if start > end else True


def tick(engine, *, now=None):
    settings = engine.settings()
    if settings.get("auto_update", "notify") == "off":
        return
    now = time.time() if now is None else now
    path = engine.manager_home / "config/update-check.json"
    state = read_json(path, {})
    if now < state.get("next_check_at", 0):
        return
    try:
        with file_lock(engine.manager_home / "config/update-check.lock"):
            # 先持久化下一轮时间，即使下载失败也不能忙轮询。
            state.update(next_check_at=now + random.uniform(0.9, 1.1) * settings.get("check_interval_minutes", 30) * 60,
                         checked_at=timestamp(), error=None)
            atomic_json(path, state)
            try:
                engine.releases(refresh=True)
                status = engine.status()
                if status["update"].get("error"):
                    raise ManagerError("update_check", status["update"]["error"], 502)
                state.update(revision=status["update"].get("revision"), relation=status["update"].get("relation"))
                if (settings.get("auto_update") == "idle" and status["update"]["available"]
                        and not status["update"].get("requires_confirmation") and not engine.active_job()
                        and in_window(settings.get("maintenance_window", ""))):
                    from .update_flow import begin_update
                    state["job_id"] = begin_update(engine, automatic=True)["id"]
            except (ManagerError, OSError, RuntimeError, ValueError, KeyError, TypeError) as exc:
                state["error"] = getattr(exc, "message", str(exc))
            atomic_json(path, state)
    except ManagerError as exc:
        if exc.code != "busy":
            raise
