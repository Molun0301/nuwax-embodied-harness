"""installer 共享 gateway lock 封装。

与 ``scripts/start_gateway.sh:86-96`` (``acquire_script_lock``) 共用 flock 协议，
锁文件路径硬编码 ``runtime_data/.gateway.lock``（**禁止**引入
``runtime_data/.installer.lock`` —— D4 / iter2 P0-3 PASS / RK18 统一吸纳 RK7）。
跨进程互斥：同一 lock 文件上 Python ``fcntl.flock`` 与 bash ``flock -n 9``
走内核级 BSD lock，可跨语言互斥（见 ``start_gateway.sh:86-96`` 的
``exec {LOCK_FD}>\"${lock_path}\"`` + ``flock -n \"${LOCK_FD}\"`` 模式）。

设计要点（与 bash 端完全等价）：

* ``mkdir -p $(dirname)`` —— ``_ensure_lockfile`` 用 ``os.makedirs`` 兜底
* ``: >> $LOCK_FILE`` —— 路径存在但 0 字节也能 flock（flock 锁文件大小无关）
* lock file mode 0o644（与 bash 端 ``exec FD>FILE`` 默认 umask 一致）
* ``fcntl.flock LOCK_EX | LOCK_NB`` —— 非阻塞，timeout=0 直接抛 ``LockBusy``

调用方契约：

* ``acquire_lock(timeout=0)`` —— 非阻塞获取，失败抛 ``LockBusy``
* ``release_lock()`` —— 释放 + 关 FD，幂等
* ``is_held()`` —— 当前 PID 是否持有（fuser/proc 兜底，防 FD 失效误判）

不在本模块引入 ``state.json`` 写入逻辑（plan RK7 mitigation 收敛到
``runtime_data/.gateway.lock`` + ``os.replace`` 原子写，本模块只负责锁）。
"""

from __future__ import annotations

import errno
import fcntl
import os
from pathlib import Path

# 锁文件路径硬编码 —— 与 scripts/start_gateway.sh:484 默认值不同
# （bash 端默认 ``runtime_data/gateway/start_gateway.lock``），
# 但按 openspec/changes/installer-redesign/specs/installer/spec.md
# Requirement "与 start_gateway.sh 共享 flock 锁" 与 plan 4.1 L127 +
# RK18 统一吸纳 RK7，installer 侧统一走 ``runtime_data/.gateway.lock``。
# 如需切换到 bash 端默认值，须同步修改 spec.md 与 plan，不在本任务范围。
LOCK_FILE = Path("runtime_data/.gateway.lock")
LOCK_FILE_MODE = 0o644  # 与 bash 端 ``exec FD>FILE`` 默认 umask 一致


class LockBusy(RuntimeError):
    """锁被其他进程持有（非阻塞获取失败）。

    携带其他进程的 PID（若能解析），便于 errors.InstallerError(code=4)
    渲染中文 remediation。``acquire_lock(timeout=0)`` 唯一抛出场景。
    """

    def __init__(self, lock_path: Path, holder_pid: int | None = None) -> None:
        self.lock_path = lock_path
        self.holder_pid = holder_pid
        super().__init__(
            f"gateway lock busy: {lock_path} (holder_pid={holder_pid})"
        )


# 模块级 FD，避免 acquire/release 跨调用句柄漂移
_fd: int | None = None


def _ensure_lockfile(lock_path: Path) -> None:
    """确保锁文件存在并设 mode 0o644。

    等价 bash 端 ``mkdir -p $(dirname) && : >> $LOCK_FILE``：目录兜底 + 文件兜底。
    已存在文件不动 mode（避免破坏 bash 端持有的 inode 状态）。
    """
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    if not lock_path.exists():
        # ``open(... 'a')`` 等价 ``: >> FILE``，创建 0 字节文件
        fd = os.open(str(lock_path), os.O_CREAT | os.O_RDWR, LOCK_FILE_MODE)
        os.close(fd)
    elif lock_path.stat().st_mode & 0o777 != LOCK_FILE_MODE:
        # 仅在文件未被任何进程持有时调整 mode（避免惊动 flock 持有者）
        try:
            os.chmod(lock_path, LOCK_FILE_MODE)
        except PermissionError:
            pass


def acquire_lock(timeout: float = 0.0) -> None:
    """非阻塞（或有限阻塞）获取 ``runtime_data/.gateway.lock``。

    参数
    ----
    timeout: 阻塞等待秒数，默认 0（非阻塞）。非 0 时简单 ``sleep`` 轮询，
        不引入信号量复杂度（本任务仅要求 ``timeout=0`` 契约）。

    异常
    ----
    LockBusy: 其他进程持有锁（与 bash ``flock -n`` 失败等价）。
    """
    global _fd
    if _fd is not None:
        # 本进程已持有，幂等返回（防止 health phase 多次调用）
        return

    _ensure_lockfile(LOCK_FILE)

    # ``open`` 必须 O_RDWR：flock 在只读 FD 上行为不可移植
    fd = os.open(str(LOCK_FILE), os.O_RDWR)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as exc:
        os.close(fd)
        if exc.errno not in (errno.EWOULDBLOCK, errno.EAGAIN):
            raise
        holder_pid = _try_read_pid(LOCK_FILE)
        raise LockBusy(LOCK_FILE, holder_pid) from exc

    # 写 PID 到锁文件头，便于其他进程（bash 端）解析持有者
    # 与 health phase 的 PID 比对共用同一文件 —— RK18 / D4 强制
    try:
        os.ftruncate(fd, 0)
        os.write(fd, f"{os.getpid()}\n".encode("utf-8"))
    except OSError:
        # 写 PID 失败不阻断持锁（lock 状态由 flock 保证）
        pass

    _fd = fd


def release_lock() -> None:
    """释放锁 + 关闭 FD。幂等（重复调用不抛错）。

    与 bash 端 ``flock -u`` 等价；FD 关闭时内核自动释放 BSD lock。
    """
    global _fd
    if _fd is None:
        return
    try:
        fcntl.flock(_fd, fcntl.LOCK_UN)
    except OSError:
        pass
    try:
        os.close(_fd)
    except OSError:
        pass
    _fd = None


def is_held() -> bool:
    """当前进程是否持有锁。

    通过模块级 FD 判定：若本进程已 ``acquire_lock`` 且未 ``release_lock``，
    返回 True；否则 False（不查其他进程持有的状态 —— 那是 health phase
    通过 ``fuser``/proc 做的事）。
    """
    return _fd is not None


def current_holder_pid() -> int | None:
    """读锁文件首行解析持有者 PID（供 health phase 比对）。

    与 ``acquire_lock`` 写入的 PID 同源；其他进程（bash 端）持锁时
    锁文件可能为空或含 stale PID —— 调用方须配合 ``is_pid_alive`` 使用。
    """
    return _try_read_pid(LOCK_FILE)


def _try_read_pid(lock_path: Path) -> int | None:
    """从锁文件首行尝试解析 PID；解析失败返回 None。"""
    try:
        with open(lock_path, "r", encoding="utf-8") as fp:
            line = fp.readline().strip()
        return int(line) if line else None
    except (OSError, ValueError):
        return None
