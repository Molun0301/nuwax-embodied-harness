"""统一日志历史查询与导出。磁盘扫描在线程中执行，按记录游标翻页。"""
from __future__ import annotations

from datetime import datetime, timezone
import base64
import heapq
from copy import deepcopy
from functools import lru_cache
import json
import os
from pathlib import Path
import re
from typing import Iterator, Optional

from fastapi import APIRouter, Query
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import Response

from core.logging_utils import _redact_sensitive_text, _is_sensitive_key

_TEXT_LINE = re.compile(r"^(?P<timestamp>\d{4}-\d\d-\d\d[ T]\d\d:\d\d:\d\d[,.]\d+)\s+\[(?P<level>\w+)\]\s+\[(?P<component>[^]]+)\]\s*(?P<message>.*)$")
_RUST_LINE = re.compile(r"^(?P<timestamp>\d{4}-\d\d-\d\dT\S+)\s+(?P<level>TRACE|DEBUG|INFO|WARN|ERROR)\s+(?P<message>.*)$")
_ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;]*m")


def _reverse_lines(path: Path) -> Iterator[tuple[int, str, os.stat_result]]:
    """从文件尾部分块读取；历史大文件不会被一次读入内存。"""
    try:
        with path.open("rb") as handle:
            opened_stat = os.fstat(handle.fileno())
            position = handle.seek(0, 2)
            pending = b""
            while position:
                size = min(position, 64 * 1024)
                position -= size
                handle.seek(position)
                block = handle.read(size) + pending
                parts = block.split(b"\n")
                pending = parts.pop(0)
                offset = position + len(block)
                for raw in reversed(parts):
                    offset -= len(raw) + 1
                    if raw.strip():
                        yield offset + 1, raw.decode("utf-8", errors="replace"), opened_stat
            if pending.strip():
                yield 0, pending.decode("utf-8", errors="replace"), opened_stat
    except FileNotFoundError:
        # 容量轮转恰好发生时，下一轮查询会从当前文件列表继续。
        return


def _parse_entry(raw: str, *, identity: str, fallback_time: str) -> Optional[dict]:
    text = _ANSI_ESCAPE.sub("", raw).replace("\x00", "").strip()
    if not text:
        return None
    try:
        data = json.loads(text)
    except ValueError:
        data = None
    if isinstance(data, dict) and any(key in data for key in ("message", "msg", "fields")):
        fields = data.get("details") or data.get("fields") or {}
        fields = dict(fields) if isinstance(fields, dict) else {}
        rust = "fields" in data
        for span in [*(data.get("spans") or []), data.get("span") or {}]:
            if isinstance(span, dict):
                for key in ("request_id", "task_id", "capability_name"):
                    if span.get(key):
                        fields.setdefault(key, span[key])
        entry = {
            "id": data.get("id") or identity,
            "timestamp": data.get("timestamp") or data.get("ts") or fallback_time,
            "level": "warning" if str(data.get("level") or "").lower() == "warn" else str(data.get("level") or "info").lower(),
            "source": data.get("source") or ("rust" if rust else "python"),
            "component": data.get("component") or fields.get("component") or data.get("target") or data.get("logger") or "unknown",
            "event": data.get("event") or fields.get("event"),
            "message": data.get("message") or data.get("msg") or fields.pop("message", ""),
            "request_id": data.get("request_id") or fields.get("request_id"),
            "task_id": data.get("task_id") or fields.get("task_id"),
            "capability_name": data.get("capability_name") or fields.get("capability_name"),
            "exception": data.get("exception"),
            "details": fields,
        }
        # 兼容迁移前的 JSON 扩展字段，不把它们丢弃为一段消息。
        if "details" not in data and not rust:
            for key, value in data.items():
                if key not in entry and key not in {"ts", "msg", "logger"}:
                    fields.setdefault(key, value)
        return entry
    match = _TEXT_LINE.match(text)
    rust_match = _RUST_LINE.match(text)
    if match:
        entry = match.groupdict()
        entry["timestamp"] = datetime.fromisoformat(entry["timestamp"].replace(",", ".")).astimezone(timezone.utc).isoformat()
        entry["level"] = entry["level"].lower()
        source = "native" if "go2_sh" in entry["component"] else "python"
    elif rust_match:
        entry = rust_match.groupdict()
        entry["component"] = "rust"
        entry["level"] = "warning" if entry["level"] == "WARN" else entry["level"].lower()
        source = "rust"
    elif raw[:1].isspace() or text.startswith(("Traceback (", "During handling of", "The above exception")) or re.match(r"^[\w.]+(?:Error|Exception|Interrupt|Exit|Warning):", text):
        return None
    else:
        entry = {"timestamp": fallback_time, "level": "info", "component": "process", "message": text}
        source = "native"
    entry.update(id=identity, source=source, event=None, request_id=None, task_id=None,
                 capability_name=None, exception=None, details={"legacy_format": True})
    return entry


def _redact_entry(value):
    if isinstance(value, dict):
        return {key: "********" if _is_sensitive_key(key) else _redact_entry(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_redact_entry(item) for item in value]
    return _redact_sensitive_text(value) if isinstance(value, str) else value


def _log_paths(log_file: str, log_dir: str = "") -> list[list[Path]]:
    main = Path(log_file).expanduser()
    directory = Path(log_dir).expanduser() if log_dir else main.parent
    bases = {main, *directory.glob("nuwax-*.log")}
    # 组件当前文件尚未重建时，也包含已有轮转文件。
    for candidate in directory.glob("nuwax-*.log.[0-9]*"):
        base, _, suffix = str(candidate).rpartition(".")
        if suffix.isdigit():
            bases.add(Path(base))
    families = []
    for base in sorted(bases):
        rotations = []
        for candidate in base.parent.glob(base.name + ".*"):
            suffix = candidate.name[len(base.name) + 1:]
            if suffix.isdigit():
                rotations.append((int(suffix), candidate))
        families.append([base, *(item[1] for item in sorted(rotations))])
    return families


def _time_key(value: str) -> str:
    try:
        timestamp = datetime.fromisoformat(str(value).replace("Z", "+00:00"))
        return timestamp.astimezone(timezone.utc).isoformat(timespec="microseconds")
    except (ValueError, TypeError):
        return str(value)


def _entry_key(entry: dict) -> tuple[str, str]:
    return _time_key(entry["timestamp"]), str(entry["id"])


def _component_entries(paths: list[Path], seen_files: set):
    for path in paths:
        fallback_time = None
        continuation = []
        for offset, raw, stat in _reverse_lines(path):
            if fallback_time is None:
                identity = (stat.st_dev, stat.st_ino)
                if identity in seen_files:
                    break
                seen_files.add(identity)
                fallback_time = datetime.fromtimestamp(stat.st_mtime, timezone.utc).isoformat()
            identity = f"{stat.st_dev:020}:{stat.st_ino:020}:{offset:020}"
            entry = _parse_entry(raw, identity=identity, fallback_time=fallback_time)
            if entry is None:
                continuation.append(raw)
                continue
            if continuation:
                entry["exception"] = "\n".join(reversed(continuation))
                continuation.clear()
            yield entry


def _encode_cursor(entry: dict) -> str:
    value = json.dumps(_entry_key(entry), separators=(",", ":")).encode("utf-8")
    return "v1." + base64.urlsafe_b64encode(value).decode("ascii").rstrip("=")


def _decode_cursor(before: str):
    if not before.startswith("v1."):
        return None
    try:
        value = before[3:]
        timestamp, identity = json.loads(base64.urlsafe_b64decode(value + "=" * (-len(value) % 4)))
        return str(timestamp), str(identity)
    except (ValueError, TypeError):
        return None


def _scan_logs(log_file: str, *, log_dir: str = "", limit: int = 200, before: str = "", level: str = "", source: str = "",
               component: str = "", query: str = "", request_id: str = "", task_id: str = "") -> dict:
    """按源时间和标识全局排序，仅保留页容量；线程写入造成的文件内乱序也不会漏页。"""
    sources = set()
    cursor = _decode_cursor(before)
    timestamp_before = _time_key(before) if before and re.match(r"^\d{4}-\d\d-\d\dT", before) else ""
    if before and cursor is None and not timestamp_before:
        return {"entries": [], "next_before": None, "has_more": False, "sources": [], "cursor_expired": True}
    levels = {"warning" if item.strip().lower() == "warn" else item.strip().lower() for item in level.split(",") if item.strip()}
    search = query.casefold()

    def candidates():
        seen_files = set()
        for paths in _log_paths(log_file, log_dir):
            for entry in _component_entries(paths, seen_files):
                key = _entry_key(entry)
                if cursor and key >= cursor:
                    continue
                if timestamp_before and key[0] >= timestamp_before:
                    continue
                sources.add(str(entry["source"]))
                if levels and entry["level"] not in levels:
                    continue
                if source and entry["source"] != source:
                    continue
                if component and component.casefold() not in str(entry["component"]).casefold():
                    continue
                if request_id and entry["request_id"] != request_id:
                    continue
                if task_id and entry["task_id"] != task_id:
                    continue
                if search and search not in json.dumps(_redact_entry(entry), ensure_ascii=False).casefold():
                    continue
                yield entry

    # 不假定多线程写入的单个文件严格有序，选择最近一页而不把全量日志载入内存。
    entries = heapq.nlargest(limit + 1, candidates(), key=_entry_key)
    has_more = len(entries) > limit
    entries = [_redact_entry(entry) for entry in entries[:limit]]
    return {"entries": entries, "next_before": _encode_cursor(entries[-1]) if entries else None,
            "has_more": has_more, "sources": sorted(sources), "cursor_expired": False}


@lru_cache(maxsize=4)
def _cached_log_page(signature, log_file, log_dir, limit, before, level, source, component, query, request_id, task_id):
    return _scan_logs(log_file, log_dir=log_dir, limit=limit, before=before, level=level, source=source,
                      component=component, query=query, request_id=request_id, task_id=task_id)


def query_logs(log_file: str, *, log_dir: str = "", limit: int = 200, before: str = "", level: str = "", source: str = "",
               component: str = "", query: str = "", request_id: str = "", task_id: str = "") -> dict:
    """文件未变化时复用最近四个小页面；写入、轮转和新组件文件都会自然失效。"""
    if limit > 200:
        return _scan_logs(log_file, log_dir=log_dir, limit=limit, before=before, level=level, source=source,
                          component=component, query=query, request_id=request_id, task_id=task_id)
    signature = []
    for paths in _log_paths(log_file, log_dir):
        for path in paths:
            try:
                stat = path.stat()
                signature.append((str(path), stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns))
            except FileNotFoundError:
                continue
    return deepcopy(_cached_log_page(tuple(signature), log_file, log_dir, limit, before, level, source,
                                     component, query, request_id, task_id))


def build_logs_router(runtime, access_manager=None) -> APIRouter:
    """沿用控制台访问边界，不额外增加一套日志权限状态。"""
    router = APIRouter()

    @router.get("/api/logs")
    async def logs(limit: int = Query(200, ge=1, le=1000), before: str = "", level: str = "", source: str = "",
                   component: str = "", query: str = "", request_id: str = "", task_id: str = ""):
        return await run_in_threadpool(query_logs, runtime.config.logging.log_file, log_dir=runtime.config.logging.log_dir, limit=limit, before=before,
                                       level=level, source=source, component=component, query=query,
                                       request_id=request_id, task_id=task_id)

    @router.get("/api/logs/export")
    async def export_logs(limit: int = Query(10000, ge=1, le=100000), before: str = "", level: str = "", source: str = "",
                          component: str = "", query: str = "", request_id: str = "", task_id: str = "",
                          format: str = Query("jsonl", pattern="^(jsonl|text)$")):
        def render():
            result = query_logs(runtime.config.logging.log_file, log_dir=runtime.config.logging.log_dir, limit=limit, before=before, level=level, source=source,
                                component=component, query=query, request_id=request_id, task_id=task_id)
            rows = reversed(result["entries"])
            if format == "jsonl":
                content = "".join(json.dumps(entry, ensure_ascii=False) + "\n" for entry in rows)
            else:
                content = "".join(f'{entry["timestamp"]} [{entry["level"]}] [{entry["component"]}] {entry["message"]}\n'
                                  + json.dumps({key: value for key, value in entry.items() if key not in {"timestamp", "level", "component", "message", "exception"}}, ensure_ascii=False)
                                  + "\n" + (entry.get("exception") or "") + "\n" for entry in rows)
            return content, result
        content, result = await run_in_threadpool(render)
        return Response(content, media_type="application/x-ndjson" if format == "jsonl" else "text/plain",
                        headers={"Content-Disposition": f'attachment; filename="nuwax-logs.{"jsonl" if format == "jsonl" else "txt"}"',
                                 "X-Logs-Has-More": str(result["has_more"]).lower(), "X-Logs-Next-Before": result["next_before"] or ""})
    return router
