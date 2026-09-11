"""查询进程拥有的持久日志索引；保存脱敏投影，原日志始终是权威数据。"""
from __future__ import annotations

import hashlib
import json
import logging
import os
from pathlib import Path
import sqlite3
import stat
import time
from datetime import datetime, timezone

_INDEX_VERSION = 1
_INDEX_MAX_BYTES = 2 * 1024 * 1024 * 1024
_LOGGER = logging.getLogger("nuwax.log_index")


class _UnsupportedProjection(Exception):
    """敏感排序或组件字段无法无损地持久索引，须沿原扫描路径查询。"""


class _WorkBudget:
    def __init__(self):
        self.started = time.monotonic()

    def yield_if_due(self):
        if time.monotonic() - self.started >= 0.008:
            time.sleep(0.002)
            self.started = time.monotonic()
        return 0


def _equality_key(value):
    # 精确筛选需要原值；仅保存类型敏感散列，不存储可能含凭据的原文。
    encoded = json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _database_path(log_file, log_dir):
    main = Path(log_file).expanduser().absolute()
    directory = Path(log_dir).expanduser().absolute() if log_dir else main.parent
    cache = directory / ".nuwax-history-index"
    cache.mkdir(mode=0o700, exist_ok=True)
    info = cache.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
        raise OSError("索引目录必须为当前用户独占的普通目录")
    digest = hashlib.sha256((str(main) + "\0" + str(directory)).encode()).hexdigest()[:24]
    path = cache / f"history-v{_INDEX_VERSION}-{digest}.sqlite3"
    try:
        descriptor = os.open(str(path), os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, "O_NOFOLLOW", 0), 0o600)
    except FileExistsError:
        info = path.lstat()
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
            raise OSError("索引文件必须为当前用户独占的普通文件")
    else:
        os.close(descriptor)
    return path


class PersistentLogIndex:
    """全部筛选共享一套文件索引；单线程使用，调用者负责关闭。"""

    def __init__(self, log_file, log_dir=""):
        self.log_file, self.log_dir = log_file, log_dir
        self.path = _database_path(log_file, log_dir)
        self.connection = None
        self.stats = {}
        db = sqlite3.connect(str(self.path), timeout=0.5, isolation_level=None)
        try:
            db.execute("PRAGMA foreign_keys=ON")
            db.execute("PRAGMA journal_mode=DELETE")
            db.execute("PRAGMA synchronous=NORMAL")
            db.execute("PRAGMA cache_size=-8192")
            db.execute("PRAGMA temp_store=FILE")
            db.execute("PRAGMA mmap_size=0")
            page_size = db.execute("PRAGMA page_size").fetchone()[0]
            db.execute(f"PRAGMA max_page_count={_INDEX_MAX_BYTES // page_size}")
            version = db.execute("PRAGMA user_version").fetchone()[0]
            if version not in (0, _INDEX_VERSION):
                raise _UnsupportedProjection("日志索引版本不匹配")
            self.connection = db
            self._create_schema()
        except BaseException:
            db.close()
            self.connection = None
            raise

    def _create_schema(self):
        db = self.connection
        db.execute("BEGIN IMMEDIATE")
        try:
            db.execute("""CREATE TABLE IF NOT EXISTS files(
                file_key TEXT PRIMARY KEY, path TEXT NOT NULL, size INTEGER NOT NULL,
                mtime_ns INTEGER NOT NULL, ctime_ns INTEGER NOT NULL, tail_start INTEGER NOT NULL,
                dynamic_time INTEGER NOT NULL, prefix BLOB NOT NULL, suffix BLOB NOT NULL,
                file_rank INTEGER NOT NULL)""")
            db.execute("""CREATE TABLE IF NOT EXISTS entries(
                rowid INTEGER PRIMARY KEY,
                file_key TEXT NOT NULL REFERENCES files(file_key) ON DELETE CASCADE,
                byte_offset INTEGER NOT NULL, time_key TEXT NOT NULL, identity TEXT NOT NULL,
                level TEXT NOT NULL, source TEXT NOT NULL, source_key TEXT NOT NULL,
                component TEXT NOT NULL, request_key TEXT NOT NULL, task_key TEXT NOT NULL,
                encoded TEXT NOT NULL, search_text TEXT NOT NULL,
                UNIQUE(file_key,byte_offset))""")
            for name, columns in (("order", "time_key DESC,identity DESC"),
                                  ("request", "request_key,time_key DESC,identity DESC"),
                                  ("task", "task_key,time_key DESC,identity DESC"),
                                  ("level", "level,time_key DESC,identity DESC"),
                                  ("source", "source,time_key,identity")):
                db.execute(f"CREATE INDEX IF NOT EXISTS entry_{name} ON entries({columns})")
            exists = db.execute("SELECT 1 FROM sqlite_master WHERE name='entry_text'").fetchone()
            self.trigram = bool(exists)
            if not exists:
                try:
                    db.execute("CREATE VIRTUAL TABLE entry_text USING fts5(search_text,content='entries',"
                               "content_rowid='rowid',columnsize=0,detail=none,tokenize='trigram case_sensitive 1')")
                except sqlite3.OperationalError as error:
                    if not any(text in str(error).lower() for text in
                               ("no such module", "no such tokenizer", "error in tokenizer")):
                        raise
                else:
                    self.trigram = True
                    db.execute("INSERT INTO entry_text(entry_text) VALUES('rebuild')")
            if self.trigram:
                db.execute("""CREATE TRIGGER IF NOT EXISTS entry_insert AFTER INSERT ON entries BEGIN
                    INSERT INTO entry_text(rowid,search_text) VALUES(new.rowid,new.search_text); END""")
                db.execute("""CREATE TRIGGER IF NOT EXISTS entry_delete AFTER DELETE ON entries BEGIN
                    INSERT INTO entry_text(entry_text,rowid,search_text)
                    VALUES('delete',old.rowid,old.search_text); END""")
            db.execute(f"PRAGMA user_version={_INDEX_VERSION}")
            db.execute("COMMIT")
        except BaseException:
            if db.in_transaction:
                db.execute("ROLLBACK")
            raise

    def _refresh_file(self, handle, current, path, file_key, rank, budget):
        from . import logs as protocol
        db = self.connection
        old = db.execute("SELECT path,size,mtime_ns,ctime_ns,tail_start,dynamic_time,prefix,suffix "
                         "FROM files WHERE file_key=?", (file_key,)).fetchone()
        size, mtime, ctime = current.st_size, current.st_mtime_ns, current.st_ctime_ns
        if old and (old[1], old[2], old[3]) == (size, mtime, ctime):
            db.execute("UPDATE files SET path=?,file_rank=? WHERE file_key=?", (path, rank, file_key))
            self.stats["reused_files"] += 1
            return
        append = bool(old and size > old[1] and not old[5])
        rename = bool(old and old[0] != path and (size, mtime) == (old[1], old[2]))
        if (append or rename) and self._anchors(handle, old[1]) != (old[6], old[7]):
            append = rename = False
        if rename:
            db.execute("UPDATE files SET path=?,ctime_ns=?,file_rank=? WHERE file_key=?",
                       (path, ctime, rank, file_key))
            self.stats["reused_files"] += 1
            return
        start = old[4] if append else 0
        anchors = self._anchors(handle, size)
        db.execute("INSERT OR IGNORE INTO files VALUES(?,?,?,?,?,?,?,?,?,?)",
                   (file_key, path, size, mtime, ctime, 0, 0, *anchors, rank))
        # 仅撤销缓存表中该文件的可变末尾；从不修改实际日志文件。
        db.execute("DELETE FROM entries WHERE file_key=? AND byte_offset>=?", (file_key, start))
        continuation = []
        tail_start, complete_seen, dynamic_time = 0, False, False
        fallback = datetime.fromtimestamp(current.st_mtime, timezone.utc).isoformat()
        rows, row_bytes = [], 0
        for offset, raw, complete in protocol._reverse_open_lines(handle, start, size):
            identity = f"{current.st_dev:020}:{current.st_ino:020}:{offset:020}"
            entry = protocol._parse_entry(raw, identity=identity, fallback_time=fallback)
            if entry is None:
                continuation.append(raw)
                continue
            if continuation:
                entry["exception"] = "\n".join(reversed(continuation))
                continuation.clear()
            if not complete_seen and complete:
                tail_start, complete_seen = offset, True
            dynamic_time = dynamic_time or entry["timestamp"] == fallback
            key = protocol._entry_key(entry)
            redacted = protocol._redact_entry(entry)
            if (protocol._entry_key(redacted) != key or redacted["level"] != entry["level"]
                    or str(redacted["component"]) != str(entry["component"])
                    or str(redacted["source"]) != str(entry["source"])):
                raise _UnsupportedProjection("敏感日志排序或组件字段需要非持久扫描")
            encoded = json.dumps(redacted, ensure_ascii=False, separators=(",", ":"))
            search_text = json.dumps(redacted, ensure_ascii=False).casefold()
            rows.append((file_key, offset, *key, entry["level"], str(entry["source"]),
                         _equality_key(entry["source"]), str(entry["component"]).casefold(),
                         _equality_key(entry["request_id"]), _equality_key(entry["task_id"]), encoded, search_text))
            row_bytes += len(encoded.encode("utf-8")) + len(search_text.encode("utf-8"))
            self.stats["parsed_entries"] += 1
            if len(rows) >= 128 or row_bytes >= 1024 * 1024:
                self._insert(rows)
                rows, row_bytes = [], 0
                budget.yield_if_due()
        self._insert(rows)
        finished = os.fstat(handle.fileno())
        if (finished.st_size < size or
                (finished.st_size == size and (finished.st_mtime_ns, finished.st_ctime_ns) != (mtime, ctime)) or
                self._anchors(handle, size) != anchors):
            raise protocol._LogSnapshotChanged("日志文件在持久索引读取期间改变")
        db.execute("UPDATE files SET path=?,size=?,mtime_ns=?,ctime_ns=?,tail_start=?,dynamic_time=?,"
                   "prefix=?,suffix=?,file_rank=? WHERE file_key=?",
                   (path, size, mtime, ctime, tail_start, int(dynamic_time), *anchors, rank, file_key))
        self.stats["read_bytes"] += size - start

    @staticmethod
    def _anchors(handle, size):
        from . import logs as protocol
        # 文件边界可能含敏感原文，只保留摘要，不把原始片段写入缓存。
        return tuple(hashlib.sha256(part).digest() for part in protocol._file_anchors(handle, size))

    def _insert(self, rows):
        self.connection.executemany(
            "INSERT INTO entries(file_key,byte_offset,time_key,identity,level,source,source_key,"
            "component,request_key,task_key,encoded,search_text) VALUES(?,?,?,?,?,?,?,?,?,?,?,?)", rows)

    def _refresh(self, budget):
        from . import logs as protocol
        seen = set()
        for family in protocol._log_paths(self.log_file, self.log_dir):
            for path in family:
                try:
                    handle = path.open("rb")
                except FileNotFoundError:
                    continue
                with handle:
                    current = os.fstat(handle.fileno())
                    if not stat.S_ISREG(current.st_mode):
                        raise _UnsupportedProjection("日志索引只接受普通日志文件")
                    key = f"{current.st_dev}:{current.st_ino}"
                    if key in seen:
                        continue
                    rank = len(seen)
                    seen.add(key)
                    self._refresh_file(handle, current, str(path), key, rank, budget)
                budget.yield_if_due()
        for (key,) in self.connection.execute("SELECT file_key FROM files").fetchall():
            if key not in seen:
                self.connection.execute("DELETE FROM files WHERE file_key=?", (key,))

    def query(self, log_file, *, log_dir="", limit=200, before="", level="", source="",
              component="", query="", request_id="", task_id=""):
        from . import logs as protocol
        if (log_file, log_dir) != (self.log_file, self.log_dir):
            raise ValueError("索引不属于当前日志目录")
        matcher = protocol._LogFilter(before, level, source, component, query, request_id, task_id)
        if not matcher.valid:
            return protocol._page_result([], set(), limit, cursor_expired=True)
        db = self.connection
        if db is None:
            raise RuntimeError("日志持久索引已经关闭")
        self.stats = {"parsed_entries": 0, "read_bytes": 0, "reused_files": 0, "trigram": False}
        budget = _WorkBudget()
        db.set_progress_handler(budget.yield_if_due, 20000)
        try:
            db.execute("BEGIN IMMEDIATE")
            self._refresh(budget)
            conditions, parameters = [], []
            if matcher.cursor:
                conditions.append("(e.time_key,e.identity)<(?,?)")
                parameters.extend(matcher.cursor)
            if matcher.timestamp_before:
                conditions.append("e.time_key<?")
                parameters.append(matcher.timestamp_before)
            cursor_sql = " AND ".join(conditions) or "1"
            # 原来源列表涵盖游标前的全部记录，不限于本次筛选命中。
            sources = [row[0] for row in db.execute("SELECT DISTINCT e.source FROM entries e WHERE " + cursor_sql, parameters)]
            if matcher.levels:
                conditions.append("e.level IN (" + ",".join("?" for _ in matcher.levels) + ")")
                parameters.extend(sorted(matcher.levels))
            for value, column in ((source, "source_key"), (request_id, "request_key"), (task_id, "task_key")):
                if value:
                    conditions.append(f"e.{column}=?")
                    parameters.append(_equality_key(value))
            if matcher.component:
                conditions.append("instr(e.component,?)>0")
                parameters.append(matcher.component)
            if matcher.search:
                if self.trigram and len(matcher.search) >= 3 and "\x00" not in matcher.search:
                    conditions.append("e.rowid IN (SELECT rowid FROM entry_text WHERE entry_text MATCH ?)")
                    # 不保存每个三元词的全部位置，避免索引远大于原日志。
                    # 至多八个三字符必要条件取交集，最终instr仍严格核对完整子串。
                    length = len(matcher.search) - 2
                    offsets = {i * (length - 1) // 7 for i in range(8)}
                    terms = {matcher.search[offset:offset + 3] for offset in offsets}
                    parameters.append(" AND ".join('"' + term.replace('"', '""') + '"' for term in sorted(terms)))
                    self.stats["trigram"] = True
                # 三元索引只缩小候选，最终仍按原casefold（Unicode大小写折叠）字面子串复核。
                conditions.append("instr(e.search_text,?)>0")
                parameters.append(matcher.search)
            where = " AND ".join(conditions) or "1"
            rows = db.execute("SELECT e.encoded FROM entries e JOIN files f USING(file_key) WHERE " + where +
                              " ORDER BY e.time_key DESC,e.identity DESC,f.file_rank,e.byte_offset DESC LIMIT ?",
                              [*parameters, max(0, limit) + 1])
            entries = [json.loads(row[0]) for row in rows]
            db.execute("COMMIT")
            return protocol._page_result(entries, sources, limit, already_redacted=True)
        except BaseException:
            if db.in_transaction:
                db.execute("ROLLBACK")
            raise
        finally:
            db.set_progress_handler(None, 0)

    def close(self):
        if self.connection is not None:
            self.connection.close()
            self.connection = None


class LogHistoryReader:
    """既有查询进程的单索引拥有方；缓存失败时完整扫描，不返回不完整结果。"""

    def __init__(self):
        self.index = None
        self.scope = None
        self.failed_scope = None

    def query(self, log_file, **options):
        from . import logs as protocol
        scope = (log_file, options.get("log_dir", ""))
        try:
            if scope != self.scope:
                self.close()
                self.scope = scope
                self.failed_scope = None
            if self.failed_scope != scope:
                if self.index is None:
                    self.index = PersistentLogIndex(*scope)
                return self.index.query(log_file, **options)
        except (OSError, sqlite3.Error, _UnsupportedProjection, protocol._LogSnapshotChanged) as error:
            self.close()
            # 不记录原文、路径或SQL参数；此失败只影响可重建的缓存。
            _LOGGER.warning("历史日志持久索引不可用，使用完整扫描：%s", type(error).__name__)
            if not isinstance(error, protocol._LogSnapshotChanged):
                self.failed_scope = scope
        return protocol.query_logs(log_file, **options)

    def close(self):
        if self.index is not None:
            self.index.close()
            self.index = None
