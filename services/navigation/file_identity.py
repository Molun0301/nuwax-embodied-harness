"""业务标识的可逆文件路径与直接目录遍历。"""
from __future__ import annotations

import os
from pathlib import Path
import stat
from urllib.parse import quote, unquote


MAX_RECORD_BYTES = 4 * 1024 * 1024


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


class DirectFileStore:
    """业务标识直接寻址，不读取旧文件名或派生索引。"""

    def __init__(self, root: Path):
        self.root = root

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

    def prepare_parent(self, path: Path) -> None:
        self._check_components(path)
        current = self.root.parent
        for part in path.parent.relative_to(self.root.parent).parts:
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
        """仅遍历现行直接寻址目录。"""
        base = self.root / "by-id-v1"
        self._check_components(base)
        if not base.exists():
            return
        for path in self._iter_direct_files(base):
            self._check_components(path)
            self._identity_from_direct_path(path)
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
