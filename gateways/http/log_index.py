"""查询进程拥有的持久日志索引；保存脱敏投影，原日志始终是权威数据。"""
from __future__ import annotations

from collections import OrderedDict
import json
import logging
import os
from pathlib import Path
import sqlite3
import stat
import time
from datetime import datetime, timezone

_INDEX_VERSION = 2
_ANCHOR_CACHE_FILES = 256
_RAW_READER_FILES = 8
_INDEX_MAX_BYTES = 2 * 1024 * 1024 * 1024
_LOGGER = logging.getLogger("nuwax.log_index")


class _UnsupportedProjection(Exception):
    """敏感排序或组件字段无法无损地持久索引，须沿原扫描路径查询。"""


def _sqlite_contention(error):
    if not isinstance(error, sqlite3.OperationalError):
        return False
    code = getattr(error, "sqlite_errorcode", None)
    if code is not None:
        # SQLite's stable primary result codes; Python 3.8 does not export
        # the named constants or attach sqlite_errorcode to its exceptions.
        return code & 0xff in (5, 6)  # SQLITE_BUSY, SQLITE_LOCKED
    message = str(error)
    return (message in ("database is locked", "database table is locked", "database schema is locked")
            or message.startswith(("database table is locked: ", "database schema is locked: ")))


class _WorkBudget:
    def __init__(self):
        self.started = time.monotonic()

    def yield_if_due(self):
        if time.monotonic() - self.started >= 0.008:
            time.sleep(0.002)
            self.started = time.monotonic()
        return 0


def _equality_key(value):
    # 调用方只把脱敏值写入数据库；保留原有类型敏感的精确筛选语义。
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def _database_path(log_file, log_dir):
    main = Path(log_file).expanduser().absolute()
    directory = Path(log_dir).expanduser().absolute() if log_dir else main.parent
    cache = directory / ".nuwax-history-index"
    cache.mkdir(mode=0o700, exist_ok=True)
    info = cache.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
        raise OSError("索引目录必须为当前用户独占的普通目录")
    path = cache / f"history-v{_INDEX_VERSION}.sqlite3"
    try:
        descriptor = os.open(str(path), os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, "O_NOFOLLOW", 0), 0o600)
    except FileExistsError:
        info = path.lstat()
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
            raise OSError("索引文件必须为当前用户独占的普通文件")
    else:
        os.close(descriptor)
    return path


class _RawCandidateReader:
    """一次查询内的有限打开文件；只按索引时的范围核对脱敏碰撞候选。"""

    def __init__(self, index):
        self.index = index
        self.files = OrderedDict()

    def __enter__(self):
        return self

    def _verify(self, record):
        from . import logs as protocol
        handle, file_key, version, anchors = record
        current = os.fstat(handle.fileno())
        if (not stat.S_ISREG(current.st_mode) or f"{current.st_dev}:{current.st_ino}" != file_key
                or (current.st_size, current.st_mtime_ns, current.st_ctime_ns) != version
                or self.index._anchors(handle, version[0]) != anchors):
            raise protocol._LogSnapshotChanged("原日志已改变，不能沿用旧候选偏移")

    def _opened(self, row):
        from . import logs as protocol
        path, file_key = row[6], row[7]
        version = tuple(row[8:11])
        record = self.files.get(file_key)
        if record is not None:
            if record[2] != version:
                raise protocol._LogSnapshotChanged("候选索引来源版本不一致")
            self.files.move_to_end(file_key)
            return record[0]
        if len(self.files) >= _RAW_READER_FILES:
            _old_key, old = self.files.popitem(last=False)
            try:
                self._verify(old)
            finally:
                old[0].close()
        try:
            handle = Path(path).open("rb")
        except FileNotFoundError as error:
            raise protocol._LogSnapshotChanged("原日志已轮转，不能沿用旧候选路径") from error
        try:
            current = os.fstat(handle.fileno())
            if (not stat.S_ISREG(current.st_mode) or f"{current.st_dev}:{current.st_ino}" != file_key
                    or (current.st_size, current.st_mtime_ns, current.st_ctime_ns) != version):
                raise protocol._LogSnapshotChanged("候选文件与索引来源版本不一致")
            anchors = self.index._cached_anchors(file_key, version)
            if anchors is None:
                anchors = self.index._anchors(handle, version[0])
            record = (handle, file_key, version, anchors)
            self._verify(record)
        except BaseException:
            handle.close()
            raise
        self.files[file_key] = record
        return handle

    def read(self, row):
        from . import logs as protocol
        handle = self._opened(row)
        offset, size = row[5], row[8]
        if not 0 <= offset < size:
            raise protocol._LogSnapshotChanged("候选偏移不在索引字节范围内")
        handle.seek(offset)
        raw = handle.readline(size - offset)
        self.index.stats["raw_candidates"] += 1
        self.index.stats["raw_candidate_bytes"] += len(raw)
        current = os.fstat(handle.fileno())
        identity = f"{current.st_dev:020}:{current.st_ino:020}:{offset:020}"
        fallback = datetime.fromtimestamp(current.st_mtime, timezone.utc).isoformat()
        entry = protocol._parse_entry(raw.decode("utf-8", errors="replace"), identity=identity, fallback_time=fallback)
        if entry is None or protocol._entry_key(entry) != tuple(row[11:13]):
            raise protocol._LogSnapshotChanged("候选偏移的记录身份或排序值已改变")
        return entry

    def __exit__(self, exc_type, exc, tb):
        failure = None
        for record in self.files.values():
            try:
                if exc_type is None:
                    self._verify(record)
            except BaseException as error:
                if failure is None:
                    failure = error
            finally:
                record[0].close()
        self.files.clear()
        if failure is not None:
            raise failure


class PersistentLogIndex:
    """全部筛选共享一套文件索引；单线程使用，调用者负责关闭。"""

    def __init__(self, log_file, log_dir=""):
        self.log_file, self.log_dir = log_file, log_dir
        self.path = _database_path(log_file, log_dir)
        self.connection = None
        self.stats = {}
        self._anchor_cache = OrderedDict()
        main = Path(log_file).expanduser().absolute()
        directory = Path(log_dir).expanduser().absolute() if log_dir else main.parent
        self.literal_scope = (str(main), str(directory))
        self.scope_id = None
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
            db.execute("""CREATE TABLE IF NOT EXISTS scopes(
                scope_id INTEGER PRIMARY KEY, main_path TEXT NOT NULL, directory TEXT NOT NULL,
                UNIQUE(main_path,directory))""")
            db.execute("INSERT OR IGNORE INTO scopes(main_path,directory) VALUES(?,?)", self.literal_scope)
            self.scope_id = db.execute("SELECT scope_id FROM scopes WHERE main_path=? AND directory=?", self.literal_scope).fetchone()[0]
            db.execute("""CREATE TABLE IF NOT EXISTS files(
                scope_id INTEGER NOT NULL REFERENCES scopes(scope_id), file_key TEXT NOT NULL,
                path TEXT NOT NULL, size INTEGER NOT NULL, mtime_ns INTEGER NOT NULL,
                ctime_ns INTEGER NOT NULL, tail_start INTEGER NOT NULL, dynamic_time INTEGER NOT NULL,
                file_rank INTEGER NOT NULL, PRIMARY KEY(scope_id,file_key))""")
            db.execute("""CREATE TABLE IF NOT EXISTS entries(
                rowid INTEGER PRIMARY KEY, scope_id INTEGER NOT NULL, file_key TEXT NOT NULL,
                byte_offset INTEGER NOT NULL, time_key TEXT NOT NULL, identity TEXT NOT NULL,
                level TEXT NOT NULL, source TEXT NOT NULL, source_key TEXT NOT NULL,
                component TEXT NOT NULL, request_key TEXT NOT NULL, task_key TEXT NOT NULL,
                request_raw_check INTEGER NOT NULL, task_raw_check INTEGER NOT NULL,
                encoded TEXT NOT NULL, search_text TEXT NOT NULL,
                FOREIGN KEY(scope_id,file_key) REFERENCES files(scope_id,file_key) ON DELETE CASCADE,
                UNIQUE(scope_id,file_key,byte_offset))""")
            for name, columns in (("order", "scope_id,time_key DESC,identity DESC"),
                                  ("request", "scope_id,request_key,time_key DESC,identity DESC"),
                                  ("task", "scope_id,task_key,time_key DESC,identity DESC"),
                                  ("level", "scope_id,level,time_key DESC,identity DESC"),
                                  ("source", "scope_id,source,time_key,identity")):
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
        old = db.execute("SELECT path,size,mtime_ns,ctime_ns,tail_start,dynamic_time "
                         "FROM files WHERE scope_id=? AND file_key=?", (self.scope_id, file_key)).fetchone()
        size, mtime, ctime = current.st_size, current.st_mtime_ns, current.st_ctime_ns
        if old and (old[1], old[2], old[3]) == (size, mtime, ctime):
            db.execute("UPDATE files SET path=?,file_rank=? WHERE scope_id=? AND file_key=?", (path, rank, self.scope_id, file_key))
            if self._cached_anchors(file_key, (size, mtime, ctime)) is None:
                anchors = self._anchors(handle, size)
                finished = os.fstat(handle.fileno())
                if (finished.st_dev, finished.st_ino, finished.st_size, finished.st_mtime_ns, finished.st_ctime_ns) != (
                        current.st_dev, current.st_ino, size, mtime, ctime):
                    raise protocol._LogSnapshotChanged("日志在建立私有边界快照时改变")
                self._remember_anchors(file_key, (size, mtime, ctime), anchors)
            self.stats["reused_files"] += 1
            return
        previous_anchors = self._cached_anchors(file_key, tuple(old[1:4])) if old else None
        append = bool(old and previous_anchors is not None and size > old[1] and not old[5])
        rename = bool(old and previous_anchors is not None and old[0] != path and (size, mtime) == (old[1], old[2]))
        if (append or rename) and self._anchors(handle, old[1]) != previous_anchors:
            append = rename = False
        if rename:
            db.execute("UPDATE files SET path=?,ctime_ns=?,file_rank=? WHERE scope_id=? AND file_key=?",
                       (path, ctime, rank, self.scope_id, file_key))
            self._remember_anchors(file_key, (size, mtime, ctime), previous_anchors)
            self.stats["reused_files"] += 1
            return
        start = old[4] if append else 0
        anchors = self._anchors(handle, size)
        db.execute("INSERT OR IGNORE INTO files VALUES(?,?,?,?,?,?,?,?,?)",
                   (self.scope_id, file_key, path, size, mtime, ctime, 0, 0, rank))
        # 仅撤销缓存表中该文件的可变末尾；从不修改实际日志文件。
        db.execute("DELETE FROM entries WHERE scope_id=? AND file_key=? AND byte_offset>=?", (self.scope_id, file_key, start))
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
            request_key, task_key = _equality_key(redacted["request_id"]), _equality_key(redacted["task_id"])
            request_changed = redacted["request_id"] is not entry["request_id"] and request_key != _equality_key(entry["request_id"])
            task_changed = redacted["task_id"] is not entry["task_id"] and task_key != _equality_key(entry["task_id"])
            rows.append((self.scope_id, file_key, offset, *key, entry["level"], str(entry["source"]),
                         _equality_key(redacted["source"]), str(entry["component"]).casefold(),
                         request_key, task_key, int(request_changed), int(task_changed), encoded, search_text))
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
                   "file_rank=? WHERE scope_id=? AND file_key=?",
                   (path, size, mtime, ctime, tail_start, int(dynamic_time), rank, self.scope_id, file_key))
        self._remember_anchors(file_key, (size, mtime, ctime), anchors)
        self.stats["read_bytes"] += size - start

    @staticmethod
    def _anchors(handle, size):
        from . import logs as protocol
        # 原始边界仅在拥有方私有RAM中保留，绝不写入派生数据库。
        return protocol._file_anchors(handle, size)

    def _cached_anchors(self, file_key, version):
        value = self._anchor_cache.get(file_key)
        if value is None or value[0] != version:
            return None
        self._anchor_cache.move_to_end(file_key)
        return value[1]

    def _remember_anchors(self, file_key, version, anchors):
        self._anchor_cache[file_key] = (version, anchors)
        self._anchor_cache.move_to_end(file_key)
        while len(self._anchor_cache) > _ANCHOR_CACHE_FILES:
            self._anchor_cache.popitem(last=False)

    def _insert(self, rows):
        self.connection.executemany(
            "INSERT INTO entries(scope_id,file_key,byte_offset,time_key,identity,level,source,source_key,"
            "component,request_key,task_key,request_raw_check,task_raw_check,encoded,search_text) "
            "VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)", rows)

    def _refresh(self, budget):
        from . import logs as protocol
        seen = set()
        for family in protocol._log_paths(*self.literal_scope):
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
        previous_key = ""
        while True:
            page = self.connection.execute("SELECT file_key FROM files WHERE scope_id=? AND file_key>? ORDER BY file_key LIMIT 256",
                                           (self.scope_id, previous_key)).fetchall()
            if not page:
                break
            for (key,) in page:
                if key not in seen:
                    self.connection.execute("DELETE FROM files WHERE scope_id=? AND file_key=?", (self.scope_id, key))
                    self._anchor_cache.pop(key, None)
            previous_key = page[-1][0]

    def _sources_before(self, matcher, budget):
        db = self.connection
        rows = db.execute("SELECT source,time_key,identity FROM entries WHERE scope_id=?"
                          " ORDER BY source,time_key,identity LIMIT 32", (self.scope_id,)).fetchall()
        sources = []
        while rows:
            # 短样本已跨来源时逐来源查询反而更贵；余下来源保留DISTINCT。
            # 后续也检查相邻记录，避免首个大来源掩盖大量单记录来源。
            if rows[0][0] != rows[-1][0]:
                conditions, parameters = ["scope_id=?", "source>=?"], [self.scope_id, rows[0][0]]
                if matcher.cursor:
                    conditions.append("(time_key,identity)<(?,?)")
                    parameters.extend(matcher.cursor)
                if matcher.timestamp_before:
                    conditions.append("time_key<?")
                    parameters.append(matcher.timestamp_before)
                sources.extend(row[0] for row in db.execute("SELECT DISTINCT source FROM entries WHERE " +
                                                           " AND ".join(conditions), parameters))
                return sources
            # entry_source中每个来源的最早记录即可判断游标前是否存在该来源。
            row = rows[0]
            if matcher.before_key(row[1:]):
                sources.append(row[0])
            rows = db.execute("SELECT source,time_key,identity FROM entries WHERE scope_id=? AND source>?"
                              " ORDER BY source,time_key,identity LIMIT 2", (self.scope_id, row[0])).fetchall()
            budget.yield_if_due()
        return sources

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
        self.stats = {"parsed_entries": 0, "read_bytes": 0, "reused_files": 0, "trigram": False,
                      "raw_candidates": 0, "raw_candidate_bytes": 0}
        budget = _WorkBudget()
        db.set_progress_handler(budget.yield_if_due, 20000)
        try:
            db.execute("BEGIN IMMEDIATE")
            self._refresh(budget)
            conditions, parameters = ["e.scope_id=?"], [self.scope_id]
            if matcher.cursor:
                conditions.append("(e.time_key,e.identity)<(?,?)")
                parameters.extend(matcher.cursor)
            if matcher.timestamp_before:
                conditions.append("e.time_key<?")
                parameters.append(matcher.timestamp_before)
            # 原来源列表涵盖游标前的全部记录，不限于本次筛选命中。
            sources = self._sources_before(matcher, budget)
            if matcher.levels:
                conditions.append("e.level IN (" + ",".join("?" for _ in matcher.levels) + ")")
                parameters.extend(sorted(matcher.levels))
            for value, column in ((source, "source_key"), (request_id, "request_key"), (task_id, "task_key")):
                if value:
                    redacted_key = _equality_key(protocol._redact_entry(value))
                    conditions.append(f"e.{column}=?")
                    parameters.append(redacted_key)
                    if column == "source_key" and redacted_key != _equality_key(value):
                        # 改变来源本身的脱敏记录不进入持久投影，不能误命中相同掩码。
                        conditions.append("0")
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
            filters = [("request_id", request_id, _equality_key(request_id), 1, 3),
                       ("task_id", task_id, _equality_key(task_id), 2, 4)]
            active = [item for item in filters if item[1]]
            # 未筛选敏感关联字段时无需候选复核，保留原SQL的有界页选择。
            limit_sql = "" if active else " LIMIT ?"
            row_parameters = parameters if active else [*parameters, max(0, limit) + 1]
            rows = db.execute("SELECT e.encoded,e.request_key,e.task_key,e.request_raw_check,e.task_raw_check,"
                              "e.byte_offset,f.path,f.file_key,f.size,f.mtime_ns,f.ctime_ns,e.time_key,e.identity "
                              "FROM entries e JOIN files f ON e.scope_id=f.scope_id AND e.file_key=f.file_key WHERE " + where +
                              " ORDER BY e.time_key DESC,e.identity DESC,f.file_rank,e.byte_offset DESC" + limit_sql, row_parameters)
            entries = []
            with _RawCandidateReader(self) as raw_reader:
                for row in rows:
                    if any(not row[flag] and row[key] != expected for _name, _value, expected, key, flag in active):
                        budget.yield_if_due()
                        continue
                    if any(row[flag] for _name, _value, _expected, _key, flag in active):
                        raw_entry = raw_reader.read(row)
                        if any(_equality_key(raw_entry[name]) != expected for name, _value, expected, _key, _flag in active):
                            budget.yield_if_due()
                            continue
                    entries.append(json.loads(row[0]))
                    if len(entries) >= max(0, limit) + 1:
                        break
                    budget.yield_if_due()
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
        self._anchor_cache.clear()


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
            transient_sqlite = _sqlite_contention(error)
            if not isinstance(error, protocol._LogSnapshotChanged) and not transient_sqlite:
                self.failed_scope = scope
        return protocol.query_logs(log_file, **options)

    def close(self):
        if self.index is not None:
            self.index.close()
            self.index = None
