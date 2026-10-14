"""业务标识的可逆文件路径，以及仅用于旧文件定位的派生索引。"""
from __future__ import annotations

from contextlib import contextmanager
import fcntl
import os
from pathlib import Path
import re
import sqlite3
import stat
import tempfile
from urllib.parse import quote, unquote


MAX_RECORD_BYTES = 4 * 1024 * 1024
_LEGACY_NAME = re.compile(r"[0-9a-f]{64}\.json\Z")
_BATCH_SIZE = 256


def read_record_text(path: Path) -> str:
    """只读取本文件，保留四兆预算并拒绝链接和非普通记录。"""
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(descriptor, "rb") as stream:
        status = os.fstat(stream.fileno())
        if not stat.S_ISREG(status.st_mode):
            raise ValueError("目录记录必须是普通文件。")
        if status.st_size > MAX_RECORD_BYTES:
            raise ValueError("目录记录超过四兆字节预算。")
        payload = stream.read(MAX_RECORD_BYTES + 1)
        if len(payload) > MAX_RECORD_BYTES:
            raise ValueError("目录记录超过四兆字节预算。")
    return payload.decode("utf-8")


class LegacyFileIndex:
    """旧ID到旧文件名的可重建磁盘索引；新ID始终直接寻址。

    发现只发生在首次准备或显式刷新，不在普通查询未命中时发生。旧写者
    必须在切换前停止；外部导入旧文件后显式刷新，不推断外部写入事件。
    """

    def __init__(self, root: Path, read_identity, *, discovery_forbidden=lambda: False):
        self.root = root
        self._read_identity = read_identity
        self._discovery_forbidden = discovery_forbidden
        self._index_path = root / "legacy-paths.sqlite3"
        self._ready = False

    def direct_path(self, identity: str) -> Path:
        if not isinstance(identity, str) or not 1 <= len(identity) <= 128:
            raise ValueError("目录业务标识必须为一到一百二十八个字符。")
        encoded = quote(identity, safe="")
        return self.root / "by-id-v1" / Path(*("id-" + encoded[i:i + 177] for i in range(0, len(encoded), 177))) / "record.json"

    def _check_components(self, path: Path) -> None:
        try:
            root_mode = self.root.lstat().st_mode
        except FileNotFoundError:
            return
        if not stat.S_ISDIR(root_mode):
            raise ValueError("目录命名空间必须是普通目录，不能是符号链接。")
        current = self.root
        for part in path.relative_to(self.root).parts:
            current = current / part
            try:
                mode = current.lstat().st_mode
            except FileNotFoundError:
                break
            if stat.S_ISLNK(mode):
                raise ValueError("目录身份路径不能经过符号链接。")
            if current != path and not stat.S_ISDIR(mode):
                raise ValueError("目录身份路径被非目录记录占用。")

    @contextmanager
    def _discovery_owner(self):
        if self._discovery_forbidden():
            raise RuntimeError("请先在地图激活事务之外准备或刷新旧目录索引。")
        self._check_components(self.root)
        try:
            self.root.mkdir()
        except FileExistsError:
            pass
        # 另一进程可能刚创建目录但尚未同步父目录；发布索引前必须完成此步。
        descriptor = os.open(self.root.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        with self._lock_stream() as stream:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(stream.fileno(), fcntl.LOCK_UN)

    @contextmanager
    def _lock_stream(self):
        self._check_components(self.root)
        descriptor = os.open(self.root / ".legacy-discovery.lock",
            os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW | os.O_NONBLOCK, 0o600)
        with os.fdopen(descriptor, "r+b") as stream:
            if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
                raise ValueError("旧目录发现锁必须是普通文件。")
            yield stream

    @contextmanager
    def _reader(self):
        self._check_components(self._index_path)
        connection = sqlite3.connect(self._index_path.absolute().as_uri() + "?mode=ro", uri=True)
        try:
            yield connection
        finally:
            connection.close()

    def prepare(self) -> None:
        if self._ready:
            return
        if self._index_path.exists():
            self._accept_completed_index()
            return
        with self._discovery_owner():
            if self._index_path.exists():
                self._accept_completed_index()
            else:
                self._build_locked()
            self._ready = True

    def _accept_completed_index(self) -> None:
        with self._reader() as connection:
            if connection.execute("SELECT version, complete FROM discovery").fetchall() != [(1, 1)]:
                raise ValueError("旧目录索引尚未完成；请显式刷新。")
        self._ready = True

    @contextmanager
    def write_guard(self):
        """先取得发现共享租约，再由业务层进入地图事务，防止刷新与新写入交错。"""
        self.prepare()
        if self._discovery_forbidden():
            raise RuntimeError("目录写入必须先取得发现租约，再进入地图激活事务。")
        with self._lock_stream() as stream:
            fcntl.flock(stream.fileno(), fcntl.LOCK_SH)
            try:
                yield
            finally:
                fcntl.flock(stream.fileno(), fcntl.LOCK_UN)

    def refresh(self) -> None:
        """显式发现外部旧文件；发布前失败保留旧索引，不发布半成品。

        replace 后目录同步失败会报错，但新完整索引可能已可见。
        """
        with self._discovery_owner():
            self._build_locked()
            self._ready = True

    def _build_locked(self) -> None:
        descriptor, name = tempfile.mkstemp(prefix=".legacy-paths-", suffix=".sqlite3", dir=self.root)
        os.close(descriptor)
        temporary = Path(name)
        connection = None
        try:
            connection = sqlite3.connect(temporary)
            connection.execute("PRAGMA journal_mode=DELETE")
            connection.execute("CREATE TABLE paths (identity TEXT PRIMARY KEY COLLATE BINARY, basename TEXT NOT NULL UNIQUE)")
            connection.execute("CREATE TABLE discovery (version INTEGER NOT NULL, complete INTEGER NOT NULL)")
            count = 0
            with os.scandir(self.root) as entries:
                for entry in entries:
                    if not entry.name.endswith(".json"):
                        continue
                    if not _LEGACY_NAME.fullmatch(entry.name) or not entry.is_file(follow_symlinks=False):
                        raise ValueError("旧目录记录文件名或文件类型无效：" + entry.name)
                    identity = self._read_identity(Path(entry.path))
                    direct = self.direct_path(identity)
                    self._check_components(direct)
                    if direct.exists():
                        raise ValueError("同一业务标识同时存在旧路径和新路径：" + repr(identity))
                    try:
                        connection.execute("INSERT INTO paths VALUES (?, ?)", (identity, entry.name))
                    except sqlite3.IntegrityError as error:
                        raise ValueError("多个旧目录文件声明了同一业务标识：" + repr(identity)) from error
                    count += 1
                    if count % _BATCH_SIZE == 0:
                        connection.commit()
            connection.execute("INSERT INTO discovery VALUES (1, 1)")
            connection.commit()
            connection.close()
            connection = None
            with temporary.open("rb") as stream:
                os.fsync(stream.fileno())
            os.replace(temporary, self._index_path)
            directory = os.open(self.root, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            if connection is not None:
                connection.close()
            temporary.unlink(missing_ok=True)

    def resolve(self, identity: str) -> Path:
        direct = self.direct_path(identity)
        self.prepare()
        self._check_components(direct)
        with self._reader() as connection:
            row = connection.execute("SELECT basename FROM paths WHERE identity = ?", (identity,)).fetchone()
        if row is None:
            return direct
        if not _LEGACY_NAME.fullmatch(row[0]):
            raise ValueError("旧目录索引路径无效。")
        if direct.exists():
            raise ValueError("同一业务标识同时存在旧路径和新路径：" + repr(identity))
        legacy = self.root / row[0]
        self._check_components(legacy)
        if not legacy.exists():
            raise FileNotFoundError("已登记旧目录文件缺失；不会另建同标识记录：" + row[0])
        return legacy

    def prepare_parent(self, path: Path) -> None:
        self._check_components(path)
        current = self.root
        for part in path.parent.relative_to(self.root).parts:
            parent, current = current, current / part
            try:
                current.mkdir()
            except FileExistsError:
                pass
            # 也覆盖上次 mkdir 后同步失败、或另一进程尚未同步的已有组件。
            descriptor = os.open(parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
        self._check_components(path)

    def _identity_from_direct_path(self, path: Path) -> str:
        parts = path.relative_to(self.root / "by-id-v1").parts
        if parts[-1] != "record.json" or not 2 <= len(parts) <= 10 or any(not p.startswith("id-") for p in parts[:-1]):
            raise ValueError("新目录身份路径格式无效。")
        identity = unquote("".join(p[3:] for p in parts[:-1]), encoding="utf-8", errors="strict")
        if self.direct_path(identity) != path:
            raise ValueError("新目录身份路径不是规范编码。")
        return identity

    def iter_paths(self):
        """逐项列举，索引游标分批读取；调用者自己决定结果/最近条目预算。"""
        self.prepare()
        with self._reader() as connection:
            cursor = connection.execute("SELECT identity, basename FROM paths ORDER BY basename")
            while rows := cursor.fetchmany(_BATCH_SIZE):
                for identity, basename in rows:
                    if not _LEGACY_NAME.fullmatch(basename):
                        raise ValueError("旧目录索引路径无效。")
                    path = self.root / basename
                    self._check_components(path)
                    direct = self.direct_path(identity)
                    self._check_components(direct)
                    if direct.exists():
                        raise ValueError("同一业务标识同时存在旧路径和新路径：" + repr(identity))
                    yield path
            base = self.root / "by-id-v1"
            self._check_components(base)
            if not base.exists():
                return
            for path in self._iter_direct_files(base):
                self._check_components(path)
                identity = self._identity_from_direct_path(path)
                if connection.execute("SELECT 1 FROM paths WHERE identity = ?", (identity,)).fetchone():
                    raise ValueError("同一业务标识同时存在旧路径和新路径：" + repr(identity))
                yield path

    def _iter_direct_files(self, directory: Path, depth: int = 0):
        """限定编码深度，逐目录项迭代；不把整个新目录的文件名列表装入内存。"""
        if depth > 9:
            raise ValueError("新目录身份路径层级过深。")
        with os.scandir(directory) as entries:
            for entry in entries:
                if entry.is_dir(follow_symlinks=False):
                    if not entry.name.startswith("id-"):
                        raise ValueError("新目录身份路径无效。")
                    yield from self._iter_direct_files(Path(entry.path), depth + 1)
                elif entry.name.startswith(".") and entry.name.endswith(".tmp") and entry.is_file(follow_symlinks=False):
                    continue
                elif entry.is_file(follow_symlinks=False):
                    yield Path(entry.path)
                else:
                    raise ValueError("新目录身份路径必须是普通文件或目录。")
