from __future__ import annotations

import fcntl
import os
from pathlib import Path
import stat
from typing import Optional, TextIO


class InstallerLock:
    def __init__(self, path: Path = Path("/run/nuwax-installer/installer.lock")) -> None:
        self.path = path
        self._handle: Optional[TextIO] = None

    def __enter__(self) -> "InstallerLock":
        self.path.parent.mkdir(parents=True, mode=0o700, exist_ok=True)
        parent_metadata = self.path.parent.lstat()
        if (
            self.path.parent.is_symlink()
            or not stat.S_ISDIR(parent_metadata.st_mode)
            or parent_metadata.st_mode & (stat.S_IRWXG | stat.S_IRWXO)
            or os.geteuid() == 0
            and parent_metadata.st_uid != 0
        ):
            raise RuntimeError(f"deployment lock directory is unsafe: {self.path.parent}")
        os.chmod(self.path.parent, 0o700)
        flags = os.O_RDWR | os.O_CREAT | os.O_CLOEXEC | getattr(os, "O_NOFOLLOW", 0)
        try:
            descriptor = os.open(self.path, flags, 0o600)
        except OSError as exc:
            raise RuntimeError(f"deployment lock path is unsafe: {self.path}") from exc
        metadata = os.fstat(descriptor)
        if (
            not stat.S_ISREG(metadata.st_mode)
            or metadata.st_nlink != 1
            or metadata.st_mode & (stat.S_IRWXG | stat.S_IRWXO)
            or os.geteuid() == 0
            and metadata.st_uid != 0
        ):
            os.close(descriptor)
            raise RuntimeError(f"deployment lock file is unsafe: {self.path}")
        os.fchmod(descriptor, 0o600)
        handle = os.fdopen(descriptor, "r+", encoding="utf-8")
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            handle.close()
            raise RuntimeError(f"another nuwax deployment is active: {self.path}")
        handle.seek(0)
        handle.truncate()
        handle.write(f"{os.getpid()}\n")
        handle.flush()
        self._handle = handle
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        del exc_type, exc, tb
        if self._handle is None:
            return
        fcntl.flock(self._handle.fileno(), fcntl.LOCK_UN)
        self._handle.close()
        self._handle = None
