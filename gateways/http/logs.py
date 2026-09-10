"""统一日志历史查询与导出。磁盘扫描在线程中执行，按记录游标翻页。"""
from __future__ import annotations

from datetime import datetime, timezone
from contextlib import asynccontextmanager
import asyncio
import multiprocessing
import base64
from collections import OrderedDict
import heapq
from itertools import islice
import json
import os
from pathlib import Path
import re
import sys
import threading
from typing import Iterator, Optional

from fastapi import APIRouter, Query
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import Response

from core.logging_utils import _redact_sensitive_text, _is_sensitive_key
from core.process_title import set_process_title

_TEXT_LINE = re.compile(r"^(?P<timestamp>\d{4}-\d\d-\d\d[ T]\d\d:\d\d:\d\d[,.]\d+)\s+\[(?P<level>\w+)\]\s+\[(?P<component>[^]]+)\]\s*(?P<message>.*)$")
_RUST_LINE = re.compile(r"^(?P<timestamp>\d{4}-\d\d-\d\dT\S+)\s+(?P<level>TRACE|DEBUG|INFO|WARN|ERROR)\s+(?P<message>.*)$")
_ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;]*m")


def _reverse_open_lines(handle, start: int, end: int):
    """仅逆序读取打开文件的固定字节范围，并保留物理行是否已有换行符。"""
    position = end
    pending = b""
    while position > start:
        size = min(position - start, 64 * 1024)
        position -= size
        handle.seek(position)
        block = handle.read(size) + pending
        parts = block.split(b"\n")
        pending = parts.pop(0)
        offset = position + len(block)
        for raw in reversed(parts):
            offset -= len(raw) + 1
            if raw.strip():
                yield offset + 1, raw.decode("utf-8", errors="replace"), offset + 1 + len(raw) < end
    if pending.strip():
        yield start, pending.decode("utf-8", errors="replace"), start + len(pending) < end


def _reverse_lines(path: Path) -> Iterator[tuple[int, str, os.stat_result]]:
    """从文件尾部分块读取；历史大文件不会被一次读入内存。"""
    try:
        with path.open("rb") as handle:
            opened_stat = os.fstat(handle.fileno())
            for offset, raw, _complete in _reverse_open_lines(handle, 0, opened_stat.st_size):
                yield offset, raw, opened_stat
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


class _LogFilter:
    def __init__(self, before, level, source, component, query, request_id, task_id):
        self.cursor = _decode_cursor(before)
        self.timestamp_before = _time_key(before) if before and re.match(r"^\d{4}-\d\d-\d\dT", before) else ""
        self.valid = not before or self.cursor is not None or bool(self.timestamp_before)
        self.levels = {"warning" if item.strip().lower() == "warn" else item.strip().lower() for item in level.split(",") if item.strip()}
        self.source, self.component, self.search = source, component.casefold(), query.casefold()
        self.request_id, self.task_id = request_id, task_id

    def before_key(self, key):
        return not ((self.cursor and key >= self.cursor) or (self.timestamp_before and key[0] >= self.timestamp_before))

    def matches(self, entry, redacted_entry=None):
        return not (
            (self.levels and entry["level"] not in self.levels)
            or (self.source and entry["source"] != self.source)
            or (self.component and self.component not in str(entry["component"]).casefold())
            or (self.request_id and entry["request_id"] != self.request_id)
            or (self.task_id and entry["task_id"] != self.task_id)
            or (self.search and self.search not in json.dumps(
                _redact_entry(entry) if redacted_entry is None else redacted_entry,
                ensure_ascii=False).casefold())
        )


def _page_result(entries, sources, limit, *, cursor_expired=False, already_redacted=False):
    has_more = len(entries) > limit
    entries = entries[:limit] if already_redacted else [_redact_entry(entry) for entry in entries[:limit]]
    return {"entries": entries, "next_before": _encode_cursor(entries[-1]) if entries else None,
            "has_more": has_more, "sources": sorted(sources), "cursor_expired": cursor_expired}


def _scan_logs(log_file: str, *, log_dir: str = "", limit: int = 200, before: str = "", level: str = "", source: str = "",
               component: str = "", query: str = "", request_id: str = "", task_id: str = "") -> dict:
    """按源时间和标识全局排序，仅保留页容量；线程写入造成的文件内乱序也不会漏页。"""
    sources = set()
    matcher = _LogFilter(before, level, source, component, query, request_id, task_id)
    if not matcher.valid:
        return _page_result([], set(), limit, cursor_expired=True)

    def candidates():
        seen_files = set()
        for paths in _log_paths(log_file, log_dir):
            for entry in _component_entries(paths, seen_files):
                key = _entry_key(entry)
                if not matcher.before_key(key):
                    continue
                sources.add(str(entry["source"]))
                if matcher.matches(entry):
                    yield entry

    # 不假定多线程写入的单个文件严格有序，选择最近一页而不把全量日志载入内存。
    entries = heapq.nlargest(limit + 1, candidates(), key=_entry_key)
    return _page_result(entries, sources, limit)


_LOG_CACHE_MAX_QUERIES = 4
_LOG_CACHE_MAX_FILES = 64
_LOG_CACHE_MAX_BYTES = 16 * 1024 * 1024
_LOG_CACHE_PAGE_SIZE = 200
_LOG_CACHE_ANCHOR_BYTES = 256
_LOG_QUERY_CACHE = OrderedDict()
_LOG_QUERY_CACHE_LOCK = threading.Lock()


class _LogSnapshotChanged(Exception):
    """读取期间旧字节范围发生变化，交回完整扫描处理。"""


class _LogCacheFull(Exception):
    """候选索引已达到保留容量，交回原有完整扫描。"""


class _LogCacheBudget:
    def __init__(self, existing_bytes):
        # 包含查询登记容器；旧快照在事务提交前仍由读者持有。
        self.used = existing_bytes + 16 * 1024

    def retain(self, size):
        if self.used + size > _LOG_CACHE_MAX_BYTES:
            raise _LogCacheFull()
        self.used += size

    def release(self, size):
        self.used -= size


def _file_anchors(handle, size):
    """核对旧文件头和追加边界，识别通常的覆盖及截断后增长。"""
    length = min(size, _LOG_CACHE_ANCHOR_BYTES)
    handle.seek(0)
    prefix = handle.read(length)
    handle.seek(size - length)
    return prefix, handle.read(length)


class _FileLogIndex:
    def __init__(self):
        self.stat = None
        self.path = None
        self.tail_start = 0
        self.mutable = []
        self.heap = []
        self.sources = set()
        self.mutable_sources = set()
        self.ordered = ()
        self.heap_bytes = 0
        self.source_bytes = 0
        self.cache_bytes = 0
        self.dynamic_time = False
        self.anchors = (b"", b"")

    def refresh(self, handle, stat, path, matcher, capacity, budget=None):
        old = self.stat
        if old is not None and (old.st_size, old.st_mtime_ns, old.st_ctime_ns) == (stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns):
            self.path = path
            return False
        unchanged_content = bool(old and old.st_size == stat.st_size and old.st_mtime_ns == stat.st_mtime_ns and self.path != path)
        append = bool(old and stat.st_size > old.st_size and not self.dynamic_time)
        if (append or unchanged_content) and _file_anchors(handle, old.st_size) != self.anchors:
            append = unchanged_content = False
        if unchanged_content:
            self.stat, self.path = stat, path
            return False
        budget = budget or _LogCacheBudget(0)
        # 为局部堆、来源集合和容量记账容器预留空间，不等全部文件完成才限额。
        budget.retain(16 * 1024)
        retained_sizes = {}
        anchors = _file_anchors(handle, stat.st_size)
        start = self.tail_start if append else 0
        heap = list(self.heap) if append else []
        heap_bytes = self.heap_bytes if append else 0
        sources = set(self.sources) if append else set()
        budget.retain(sys.getsizeof(sources))
        source_bytes = self.source_bytes if append else 0
        dynamic_time = self.dynamic_time if append else False
        mutable = []
        mutable_sources = set()
        mutable_bytes = 0
        mutable_source_bytes = 0
        fallback_time = datetime.fromtimestamp(stat.st_mtime, timezone.utc).isoformat()
        continuation = []
        complete_header_seen = False
        next_tail_start = 0
        for offset, raw, complete in _reverse_open_lines(handle, start, stat.st_size):
            identity = f"{stat.st_dev:020}:{stat.st_ino:020}:{offset:020}"
            entry = _parse_entry(raw, identity=identity, fallback_time=fallback_time)
            if entry is None:
                continuation.append(raw)
                continue
            if continuation:
                entry["exception"] = "\n".join(reversed(continuation))
                continuation.clear()
            key = _entry_key(entry)
            before = matcher.before_key(key)
            if not complete_header_seen:
                # 无换行末行以后可能变成异常续行，连同前一个完整头一起回读。
                if before:
                    source = str(entry["source"])
                    if source not in mutable_sources:
                        previous_container_size = sys.getsizeof(mutable_sources)
                        mutable_sources.add(source)
                        size = sys.getsizeof(source)
                        budget.retain(size + sys.getsizeof(mutable_sources) - previous_container_size)
                        mutable_source_bytes += size
                    item = _matched_cached_item(entry, key, offset, matcher)
                    if item is not None:
                        budget.retain(item[3])
                        mutable_bytes += item[3]
                        mutable.append(item)
                if complete:
                    complete_header_seen = True
                    next_tail_start = offset
                continue
            dynamic_time = dynamic_time or entry["timestamp"] == fallback_time
            if before:
                source = str(entry["source"])
                if source not in sources:
                    previous_container_size = sys.getsizeof(sources)
                    sources.add(source)
                    size = sys.getsizeof(source)
                    budget.retain(size + sys.getsizeof(sources) - previous_container_size)
                    source_bytes += size
                if len(heap) < capacity or (key, offset) > heap[0][:2]:
                    item = _matched_cached_item(entry, key, offset, matcher)
                    if item is None:
                        continue
                    size = item[3]
                    if len(heap) < capacity:
                        budget.retain(size)
                        retained_sizes[offset] = size
                        heapq.heappush(heap, item)
                        heap_bytes += size
                    else:
                        # 本轮新候选被淘汰即退回预算；旧快照对象仍保守计入。
                        budget.release(retained_sizes.pop(heap[0][1], 0))
                        budget.retain(size)
                        retained_sizes[offset] = size
                        replaced = heapq.heapreplace(heap, item)
                        heap_bytes += size - replaced[3]
        finished = os.fstat(handle.fileno())
        if finished.st_size < stat.st_size:
            raise _LogSnapshotChanged("文件在索引期间被截断")
        if finished.st_size == stat.st_size and (finished.st_mtime_ns, finished.st_ctime_ns) != (stat.st_mtime_ns, stat.st_ctime_ns):
            raise _LogSnapshotChanged("固定大小文件的修改/元数据时间在索引期间变化")
        if _file_anchors(handle, stat.st_size) != anchors:
            raise _LogSnapshotChanged("文件头或读取边界在索引期间被覆盖")
        ordered = tuple(sorted(heap + mutable, key=lambda item: item[:2], reverse=True))
        cache_bytes = (16 * 1024 + heap_bytes + source_bytes + mutable_bytes + mutable_source_bytes
                       + sys.getsizeof(sources) + sys.getsizeof(mutable_sources))
        self.heap, self.sources, self.mutable = heap, sources, mutable
        self.mutable_sources = mutable_sources
        self.ordered = ordered
        self.heap_bytes, self.source_bytes = heap_bytes, source_bytes
        self.cache_bytes = cache_bytes
        self.dynamic_time = dynamic_time
        self.tail_start = next_tail_start
        self.anchors = anchors
        self.stat, self.path = stat, path
        return True

    def candidates(self, matcher, sources, budget=None):
        for contribution in (self.sources, self.mutable_sources):
            previous_size = sys.getsizeof(sources)
            sources.update(contribution)
            if budget is not None:
                budget.retain(sys.getsizeof(sources) - previous_size)
        return self.ordered


def _cache_size(value):
    """估算实际保留对象容量；不把文件大小或记录条数当作内存容量。"""
    pending, seen, size = [value], set(), 0
    while pending:
        item = pending.pop()
        if id(item) in seen:
            continue
        seen.add(id(item))
        size += sys.getsizeof(item)
        if isinstance(item, dict):
            pending.extend(item.keys())
            pending.extend(item.values())
        elif isinstance(item, (tuple, list, set)):
            pending.extend(item)
        elif isinstance(item, _FileLogIndex):
            pending.append(vars(item))
    return size


def _matched_cached_item(entry, key, offset, matcher):
    """只缓存脱敏后的紧凑字节，避免各查询重复保留大量字典与字符串对象。"""
    redacted = _redact_entry(entry) if matcher.search else None
    if not matcher.matches(entry, redacted):
        return None
    if redacted is None:
        redacted = _redact_entry(entry)
    encoded = json.dumps(redacted, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    size = _cache_size((key, offset, encoded)) + 64
    return key, offset, encoded, size


def _encode_cached_page(items, sources):
    """拼接已脱敏候选，不重新解析、脱敏或深拷贝历史记录。"""
    chunks = [b'{"entries":[']
    for index, item in enumerate(items):
        if index:
            chunks.append(b",")
        chunks.append(item[2])
    chunks.extend((b'],"sources":', json.dumps(sorted(sources), ensure_ascii=False).encode("utf-8"), b"}"))
    return b"".join(chunks)


class _LogQueryIndex:
    def __init__(self):
        self.lock = threading.Lock()
        self.files = {}
        self.order = ()
        self.response = None
        self.files_bytes = 0
        self.response_bytes = 0
        self.cache_bytes = 0

    def read(self, log_file, log_dir, limit, matcher):
        with self.lock:
            try:
                return self._read_files(log_file, log_dir, limit, matcher)
            except _LogCacheFull:
                self.files.clear()
                self.order = ()
                self.response = None
                self.files_bytes = self.response_bytes = 0
                self.cache_bytes = 0
                return None

    def _read_files(self, log_file, log_dir, limit, matcher):
        cached_response = self.response
        cached_response_bytes = self.response_bytes
        # 某文件刷新后其它文件可能失败；已持有本索引的等待者也须重建页面。
        self.response = None
        self.response_bytes = 0
        budget = _LogCacheBudget(self.files_bytes + cached_response_bytes)
        seen, order, changed = set(), [], False
        for paths in _log_paths(log_file, log_dir):
            for path in paths:
                try:
                    with path.open("rb") as handle:
                        stat = os.fstat(handle.fileno())
                        identity = (stat.st_dev, stat.st_ino)
                        if identity in seen:
                            continue
                        seen.add(identity)
                        order.append(identity)
                        if len(seen) > _LOG_CACHE_MAX_FILES:
                            self.files.clear()
                            self.response = None
                            self.files_bytes = self.response_bytes = 0
                            self.cache_bytes = 0
                            return None
                        index = self.files.setdefault(identity, _FileLogIndex())
                        previous_bytes = index.cache_bytes
                        previous_budget = budget.used
                        try:
                            changed = index.refresh(handle, stat, str(path), matcher, _LOG_CACHE_PAGE_SIZE + 1, budget) or changed
                        finally:
                            self.files_bytes += index.cache_bytes - previous_bytes
                            self.cache_bytes = self.files_bytes
                            # refresh返回后旧局部堆已释放；缓存页独立持有字节，不引用旧候选。
                            # 只在单文件构建时保留新旧峰值，不能把已释放快照累计到后续文件。
                            budget.used = previous_budget - previous_bytes + index.cache_bytes
                except FileNotFoundError:
                    continue
        changed = changed or tuple(order) != self.order
        removed_bytes = sum(index.cache_bytes for identity, index in self.files.items() if identity not in seen)
        self.files_bytes -= removed_bytes
        self.files = {identity: self.files[identity] for identity in order}
        budget.release(removed_bytes)
        self.order = tuple(order)
        if changed or cached_response is None:
            sources = set()
            # 先汇总全部文件来源，再惰性取一页；未取到的文件不能丢失来源贡献。
            candidates = [self.files[identity].candidates(matcher, sources, budget) for identity in order]
            selected = list(islice(heapq.merge(*candidates, key=lambda item: item[0], reverse=True), _LOG_CACHE_PAGE_SIZE + 1))
            # 缓存最大普通页的紧凑字节；数量变化共用文件索引，返回时才解码独立对象。
            self.response_bytes = 16 * 1024 + sum(len(item[2]) + 1 for item in selected) + sum(sys.getsizeof(source) + 64 for source in sources)
            budget.retain(2 * self.response_bytes)
            self.response = _encode_cached_page(selected, sources)
        else:
            self.response = cached_response
            self.response_bytes = cached_response_bytes
        self.cache_bytes = 16 * 1024 + self.files_bytes + self.response_bytes
        decoded = json.loads(self.response)
        return _page_result(decoded["entries"], decoded["sources"], limit, already_redacted=True)


def query_logs(log_file: str, *, log_dir: str = "", limit: int = 200, before: str = "", level: str = "", source: str = "",
               component: str = "", query: str = "", request_id: str = "", task_id: str = "") -> dict:
    """最近四种普通页查询按文件增量更新；追加不再使全部历史失去缓存。"""
    if limit > _LOG_CACHE_PAGE_SIZE or limit < 1:
        return _scan_logs(log_file, log_dir=log_dir, limit=limit, before=before, level=level, source=source,
                          component=component, query=query, request_id=request_id, task_id=task_id)
    matcher = _LogFilter(before, level, source, component, query, request_id, task_id)
    if not matcher.valid:
        return _page_result([], set(), limit, cursor_expired=True)
    signature = (log_file, log_dir, before, level, source, component, query, request_id, task_id)
    with _LOG_QUERY_CACHE_LOCK:
        index = _LOG_QUERY_CACHE.setdefault(signature, _LogQueryIndex())
        _LOG_QUERY_CACHE.move_to_end(signature)
        while len(_LOG_QUERY_CACHE) > _LOG_CACHE_MAX_QUERIES:
            _LOG_QUERY_CACHE.popitem(last=False)
    try:
        result = index.read(log_file, log_dir, limit, matcher)
    except _LogSnapshotChanged:
        with _LOG_QUERY_CACHE_LOCK:
            if _LOG_QUERY_CACHE.get(signature) is index:
                _LOG_QUERY_CACHE.pop(signature)
        result = None
    except Exception:
        with _LOG_QUERY_CACHE_LOCK:
            if _LOG_QUERY_CACHE.get(signature) is index:
                _LOG_QUERY_CACHE.pop(signature)
        raise
    with _LOG_QUERY_CACHE_LOCK:
        while _LOG_QUERY_CACHE and sum(item.cache_bytes for item in _LOG_QUERY_CACHE.values()) > _LOG_CACHE_MAX_BYTES:
            _LOG_QUERY_CACHE.popitem(last=False)
    if result is None:
        return _scan_logs(log_file, log_dir=log_dir, limit=limit, before=before, level=level, source=source,
                          component=component, query=query, request_id=request_id, task_id=task_id)
    return result


def _render_log_response(log_file, options, export_format):
    result = query_logs(log_file, **options)
    if export_format is None:
        return json.dumps(result, ensure_ascii=False, separators=(",", ":")).encode("utf-8"), {}
    rows = reversed(result["entries"])
    if export_format == "jsonl":
        content = "".join(json.dumps(entry, ensure_ascii=False) + "\n" for entry in rows)
    else:
        content = "".join(f'{entry["timestamp"]} [{entry["level"]}] [{entry["component"]}] {entry["message"]}\n'
            + json.dumps({key: value for key, value in entry.items() if key not in
                {"timestamp", "level", "component", "message", "exception"}}, ensure_ascii=False)
            + "\n" + (entry.get("exception") or "") + "\n" for entry in rows)
    return content.encode("utf-8"), {"X-Logs-Has-More": str(result["has_more"]).lower(),
                                    "X-Logs-Next-Before": result["next_before"] or ""}


def _log_query_process(connection):
    """只读取日志并复用原索引；不构造设备、网关运行时或业务服务。"""
    set_process_title("nuwax-log-query")
    if hasattr(os, "nice"):
        os.nice(10)  # 历史扫描让出调度优先级，给实时采集和交付保留余量。
    try:
        while True:
            request = connection.recv()
            if request is None:
                return
            try:
                connection.send((True, _render_log_response(*request)))
            except Exception as error:
                connection.send((False, f"{type(error).__name__}: {error}"))
    except (EOFError, BrokenPipeError):
        pass
    finally:
        connection.close()


class _LogQueryWorker:
    """路由拥有的单查询进程；取消不释放尚在处理的请求槽，关闭回收只读计算。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._stopped = threading.Event()
        self._process = None
        self._connection = None
        self._slot = None

    def _execute(self, log_file, options, export_format):
        with self._lock:
            if self._stopped.is_set():
                raise RuntimeError("日志查询拥有方已关闭")
            if self._process is not None and not self._process.is_alive():
                self._process.join()
                self._connection.close()
                self._process = self._connection = None
            if self._process is None:
                context = multiprocessing.get_context("spawn")
                # 生产启动器使用 exec -a 命名，Python 3.8 的 sys.executable 可能为空。
                # 使用当前真实解释器，不能让子进程尝试执行显示名称或空路径。
                if not sys.executable and sys.platform.startswith("linux"):
                    context.set_executable(os.readlink("/proc/self/exe"))
                parent, child = context.Pipe()
                process = context.Process(target=_log_query_process, args=(child,), name="nuwax-log-query")
                try:
                    process.start()
                except BaseException:
                    parent.close()
                    child.close()
                    raise
                child.close()
                self._process, self._connection = process, parent
            process, connection = self._process, self._connection
        connection.send((log_file, options, export_format))
        while not self._stopped.is_set():
            if connection.poll(.1):
                ok, result = connection.recv()
                if not ok:
                    raise RuntimeError(result)
                return result
            if not process.is_alive():
                raise RuntimeError(f"日志查询进程已退出：{process.exitcode}")
        raise RuntimeError("日志查询已随网关关闭")

    async def query(self, log_file, options, export_format=None):
        if self._slot is None:
            self._slot = asyncio.Semaphore(1)
        await self._slot.acquire()
        try:
            future = asyncio.get_running_loop().run_in_executor(None, self._execute,
                                                                log_file, options, export_format)
        except BaseException:
            self._slot.release()
            raise
        def completed(done):
            self._slot.release()
            # 调用者已经断开时也领取真实错误，不遗留未消费的任务异常。
            if not done.cancelled():
                done.exception()
        future.add_done_callback(completed)
        return await asyncio.shield(future)

    def close(self):
        with self._lock:
            self._stopped.set()
            process, connection = self._process, self._connection
        if process is None:
            return
        # 进程只做查询；服务关闭不等待任意大小历史扫描完成。
        if process.is_alive():
            process.terminate()
        process.join(2)
        if process.is_alive():
            process.kill()
            process.join(1)
        if process.is_alive():
            raise RuntimeError("日志查询进程尚未退出")
        connection.close()
        with self._lock:
            self._process = self._connection = None


def build_logs_router(runtime, access_manager=None) -> APIRouter:
    """沿用控制台访问边界，独立查询进程随路由生命周期关闭。"""
    worker = _LogQueryWorker()
    @asynccontextmanager
    async def lifespan(app):
        try:
            yield
        finally:
            await run_in_threadpool(worker.close)
    router = APIRouter(lifespan=lifespan)

    def options(limit, before, level, source, component, query, request_id, task_id):
        return dict(log_dir=runtime.config.logging.log_dir, limit=limit, before=before,
                    level=level, source=source, component=component, query=query,
                    request_id=request_id, task_id=task_id)

    @router.get("/api/logs")
    async def logs(limit: int = Query(200, ge=1, le=1000), before: str = "", level: str = "", source: str = "",
                   component: str = "", query: str = "", request_id: str = "", task_id: str = ""):
        content, _ = await worker.query(runtime.config.logging.log_file,
            options(limit, before, level, source, component, query, request_id, task_id))
        return Response(content, media_type="application/json")

    @router.get("/api/logs/export")
    async def export_logs(limit: int = Query(10000, ge=1, le=100000), before: str = "", level: str = "", source: str = "",
                          component: str = "", query: str = "", request_id: str = "", task_id: str = "",
                          format: str = Query("jsonl", pattern="^(jsonl|text)$")):
        content, headers = await worker.query(runtime.config.logging.log_file,
            options(limit, before, level, source, component, query, request_id, task_id), format)
        headers["Content-Disposition"] = f'attachment; filename="nuwax-logs.{"jsonl" if format == "jsonl" else "txt"}"'
        return Response(content, media_type="application/x-ndjson" if format == "jsonl" else "text/plain",
                        headers=headers)
    return router
