"""运动守卫单实例锁，进程退出后由内核释放。"""
from __future__ import annotations

import errno
import fcntl
import os
from pathlib import Path
import stat
from typing import Optional


class SingleInstanceError(RuntimeError):
    pass


class SingleInstanceLock:
    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self._fd: Optional[int] = None

    def acquire(self) -> None:
        if self._fd is not None:
            return
        self.path.parent.mkdir(mode=0o750, parents=True, exist_ok=True)
        flags = os.O_RDWR | os.O_CREAT | os.O_CLOEXEC
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        try:
            fd = os.open(self.path, flags, 0o600)
        except OSError as exc:
            if exc.errno in {errno.ELOOP, errno.ENOTDIR}:
                raise SingleInstanceError(f"lock path must be a non-symlink regular file: {self.path}") from exc
            raise SingleInstanceError(f"cannot open guard lock {self.path}") from exc
        try:
            metadata = os.fstat(fd)
            if not stat.S_ISREG(metadata.st_mode):
                raise SingleInstanceError(f"lock path is not a regular file: {self.path}")
            if metadata.st_uid != os.geteuid():
                raise SingleInstanceError(f"lock owner UID {metadata.st_uid} does not match guard UID {os.geteuid()}")
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            os.close(fd)
            raise SingleInstanceError(f"another motion guard owns {self.path}") from exc
        except BaseException:
            os.close(fd)
            raise
        self._fd = fd

    def release(self) -> None:
        if self._fd is None:
            return
        fd, self._fd = self._fd, None
        try:
            fcntl.flock(fd, fcntl.LOCK_UN)
        finally:
            os.close(fd)

    def __enter__(self) -> "SingleInstanceLock":
        self.acquire()
        return self

    def __exit__(self, *_exc_info: object) -> None:
        self.release()
