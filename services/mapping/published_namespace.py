from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
import errno
import fcntl
import json
import math
import os
from pathlib import Path
import stat
import time
import uuid
from typing import Dict, Iterator, Mapping, Optional, Tuple


_PROTOCOL = "nuwax.mapping.published-namespace.v1"
_SCHEMA_VERSION = 1
_INITIAL_GENERATION = 2
_MAX_GENERATION = (1 << 63) - 1
_LOCK_FILE_NAME = ".published_namespace.lock"
_STATE_FILE_NAME = ".published_namespace_state.json"
_MARKER_FILE_NAME = ".published_namespace_protocol.json"
_MAX_DOCUMENT_BYTES = 64 * 1024
_FILE_MODE = 0o600
_ALLOWED_OPERATIONS = frozenset(
    {"bootstrap", "finalize", "delete", "legacy_finalize", "bootstrap_recovery"}
)
_ALLOWED_STABLE_OUTCOMES = frozenset(
    {"initialized", "committed", "aborted", "recovered"}
)


class PublishedNamespaceError(RuntimeError):
    """published namespace 协议拒绝继续操作。"""


class PublishedNamespaceUnavailableError(PublishedNamespaceError):
    """协议尚未 bootstrap，或持久协议文件缺失。"""


class PublishedNamespaceIntegrityError(PublishedNamespaceError):
    """catalog root、锁文件或 journal 身份/内容不可信。"""


class PublishedNamespaceLeaseTimeoutError(PublishedNamespaceError):
    """未能在有界时间内取得协作式 advisory lease。"""


class PublishedNamespaceRecoveryRequiredError(PublishedNamespaceError):
    """持久 generation 为奇数，必须由显式 recovery owner 恢复。"""


class PublishedNamespaceGenerationExhaustedError(PublishedNamespaceError):
    """signed i64 generation 已无下一组 odd/even 值可用。"""


class PublishedNamespaceTransactionError(PublishedNamespaceError):
    """mutation journal 的阶段、事务身份或调用顺序非法。"""


@dataclass(frozen=True)
class FileIdentity:
    """安全打开文件的可审计身份。"""

    device: int
    inode: int
    uid: int
    gid: int
    mode: int
    link_count: int
    file_type: str


@dataclass(frozen=True)
class PublishedNamespaceState:
    """严格 v1 journal；偶数为 stable，奇数为 mutating。"""

    schema_version: int
    protocol: str
    generation: int
    phase: str
    transaction_id: str
    operation: str
    map_name: str
    before_identity: str
    after_identity: str
    outcome: str
    writer_instance_id: str
    updated_at_ns: int

    @property
    def is_stable(self) -> bool:
        return self.phase == "stable" and self.generation % 2 == 0

    @property
    def is_mutating(self) -> bool:
        return self.phase == "mutating" and self.generation % 2 == 1

    def canonical_bytes(self) -> bytes:
        payload = {
            "after_identity": self.after_identity,
            "before_identity": self.before_identity,
            "generation": self.generation,
            "map_name": self.map_name,
            "operation": self.operation,
            "outcome": self.outcome,
            "phase": self.phase,
            "protocol": self.protocol,
            "schema_version": self.schema_version,
            "transaction_id": self.transaction_id,
            "updated_at_ns": self.updated_at_ns,
            "writer_instance_id": self.writer_instance_id,
        }
        return _canonical_json_bytes(payload)

    @classmethod
    def from_bytes(cls, raw: bytes) -> "PublishedNamespaceState":
        payload = _load_strict_json_object(raw, "published namespace state")
        expected = {
            "schema_version",
            "protocol",
            "generation",
            "phase",
            "transaction_id",
            "operation",
            "map_name",
            "before_identity",
            "after_identity",
            "outcome",
            "writer_instance_id",
            "updated_at_ns",
        }
        if set(payload) != expected:
            raise PublishedNamespaceIntegrityError(
                "published namespace state 字段集合不符合 schema v1。"
            )
        state = cls(
            schema_version=_require_int(payload, "schema_version", 1, 1),
            protocol=_require_text(payload, "protocol", allow_empty=False),
            generation=_require_int(payload, "generation", 0, _MAX_GENERATION),
            phase=_require_text(payload, "phase", allow_empty=False),
            transaction_id=_require_text(payload, "transaction_id", allow_empty=False),
            operation=_require_text(payload, "operation", allow_empty=False),
            map_name=_require_text(payload, "map_name", allow_empty=True),
            before_identity=_require_text(payload, "before_identity", allow_empty=True),
            after_identity=_require_text(payload, "after_identity", allow_empty=True),
            outcome=_require_text(payload, "outcome", allow_empty=False),
            writer_instance_id=_require_text(
                payload, "writer_instance_id", allow_empty=False
            ),
            updated_at_ns=_require_int(
                payload, "updated_at_ns", 0, _MAX_GENERATION
            ),
        )
        state._validate()
        if state.canonical_bytes() != raw:
            raise PublishedNamespaceIntegrityError(
                "published namespace state 不是 canonical JSON bytes。"
            )
        return state

    def _validate(self) -> None:
        if self.schema_version != _SCHEMA_VERSION or self.protocol != _PROTOCOL:
            raise PublishedNamespaceIntegrityError(
                "published namespace state 协议版本不受支持。"
            )
        if self.generation < _INITIAL_GENERATION:
            raise PublishedNamespaceIntegrityError(
                "published namespace generation 尚未初始化。"
            )
        if self.operation not in _ALLOWED_OPERATIONS:
            raise PublishedNamespaceIntegrityError(
                "published namespace operation 不受支持。"
            )
        _require_uuid(self.transaction_id, "transaction_id")
        _require_uuid(self.writer_instance_id, "writer_instance_id")
        if self.phase == "stable":
            if self.generation % 2 != 0 or self.outcome not in _ALLOWED_STABLE_OUTCOMES:
                raise PublishedNamespaceIntegrityError(
                    "stable state 必须使用偶数 generation 与 stable outcome。"
                )
        elif self.phase == "mutating":
            if self.generation % 2 != 1 or self.outcome != "in_progress":
                raise PublishedNamespaceIntegrityError(
                    "mutating state 必须使用奇数 generation 与 in_progress。"
                )
        else:
            raise PublishedNamespaceIntegrityError(
                "published namespace phase 不受支持。"
            )


@dataclass(frozen=True)
class PublishedNamespaceMutation:
    """一次已 durable begin 的 mutation token。"""

    generation: int
    transaction_id: str
    operation: str
    map_name: str
    before_identity: str
    writer_instance_id: str


class PublishedNamespaceLease:
    """持有 catalog-root 固定 inode 上的 SH 或 EX advisory lease。

    威胁模型仅覆盖所有 writer 都协作使用同一固定 lock inode 的进程；拥有同等
    文件权限且绕过 flock 的进程不在模型内。调用方必须遵守固定锁序：
    map_activation_guard -> repository lease -> runtime/owner read。禁止 SH->EX 升级。
    """

    def __init__(
        self,
        protocol: "PublishedNamespaceProtocol",
        *,
        root_descriptor: int,
        lock_descriptor: int,
        mode: str,
        root_identity: FileIdentity,
        lock_identity: FileIdentity,
    ) -> None:
        self._protocol = protocol
        self._root_descriptor = root_descriptor
        self._lock_descriptor = lock_descriptor
        self.mode = mode
        self.root_identity = root_identity
        self.lock_identity = lock_identity
        self._closed = False

    def read_state(self) -> PublishedNamespaceState:
        self._require_open()
        return self._protocol._read_state(
            self._root_descriptor,
            self.root_identity,
        )

    def read_stable_state(self) -> PublishedNamespaceState:
        state = self.read_state()
        if not state.is_stable:
            raise PublishedNamespaceRecoveryRequiredError(
                "published namespace generation 为奇数；reader 不得自行升级锁恢复。"
            )
        return state

    def begin_mutation(
        self,
        *,
        operation: str,
        map_name: str,
        before_identity: str,
        writer_instance_id: str,
        transaction_id: Optional[str] = None,
        updated_at_ns: Optional[int] = None,
    ) -> PublishedNamespaceMutation:
        self._require_exclusive()
        stable = self.read_stable_state()
        if stable.generation > _MAX_GENERATION - 2:
            raise PublishedNamespaceGenerationExhaustedError(
                "published namespace generation 已耗尽，writer 永久 fail closed。"
            )
        if operation not in _ALLOWED_OPERATIONS - {"bootstrap"}:
            raise PublishedNamespaceTransactionError("mutation operation 不受支持。")
        resolved_map_name = str(map_name or "").strip()
        if not resolved_map_name:
            raise PublishedNamespaceTransactionError("mutation map_name 不能为空。")
        resolved_before = str(before_identity or "").strip()
        resolved_writer = str(writer_instance_id or "").strip()
        _require_uuid(resolved_writer, "writer_instance_id")
        resolved_transaction = str(transaction_id or uuid.uuid4())
        _require_uuid(resolved_transaction, "transaction_id")
        odd_generation = stable.generation + 1
        state = PublishedNamespaceState(
            schema_version=_SCHEMA_VERSION,
            protocol=_PROTOCOL,
            generation=odd_generation,
            phase="mutating",
            transaction_id=resolved_transaction,
            operation=operation,
            map_name=resolved_map_name,
            before_identity=resolved_before,
            after_identity="",
            outcome="in_progress",
            writer_instance_id=resolved_writer,
            updated_at_ns=_resolve_updated_at_ns(updated_at_ns),
        )
        state._validate()
        self._protocol._replace_state(
            self._root_descriptor,
            self.root_identity,
            state,
        )
        return PublishedNamespaceMutation(
            generation=odd_generation,
            transaction_id=resolved_transaction,
            operation=operation,
            map_name=resolved_map_name,
            before_identity=resolved_before,
            writer_instance_id=resolved_writer,
        )

    def commit_mutation(
        self,
        mutation: PublishedNamespaceMutation,
        *,
        after_identity: str,
        updated_at_ns: Optional[int] = None,
    ) -> PublishedNamespaceState:
        return self._finish_mutation(
            mutation,
            after_identity=after_identity,
            outcome="committed",
            updated_at_ns=updated_at_ns,
        )

    def abort_mutation(
        self,
        mutation: PublishedNamespaceMutation,
        *,
        after_identity: Optional[str] = None,
        updated_at_ns: Optional[int] = None,
    ) -> PublishedNamespaceState:
        return self._finish_mutation(
            mutation,
            after_identity=(
                mutation.before_identity
                if after_identity is None
                else str(after_identity).strip()
            ),
            outcome="aborted",
            updated_at_ns=updated_at_ns,
        )

    def recover_mutation(
        self,
        *,
        expected_transaction_id: str,
        after_identity: str,
        outcome: str,
        updated_at_ns: Optional[int] = None,
    ) -> PublishedNamespaceState:
        """将可证明的 odd residue 前推到 even；本 helper 不猜测 namespace 结果。"""

        self._require_exclusive()
        if outcome not in {"aborted", "recovered"}:
            raise PublishedNamespaceTransactionError(
                "recovery outcome 只能为 aborted 或 recovered。"
            )
        odd = self.read_state()
        if not odd.is_mutating:
            raise PublishedNamespaceTransactionError(
                "只有 odd mutating residue 可以 recovery。"
            )
        expected = str(expected_transaction_id or "").strip()
        if odd.transaction_id != expected:
            raise PublishedNamespaceTransactionError(
                "recovery transaction_id 与 durable residue 不一致。"
            )
        return self._write_stable_successor(
            odd,
            after_identity=str(after_identity or "").strip(),
            outcome=outcome,
            updated_at_ns=updated_at_ns,
        )

    def _finish_mutation(
        self,
        mutation: PublishedNamespaceMutation,
        *,
        after_identity: str,
        outcome: str,
        updated_at_ns: Optional[int],
    ) -> PublishedNamespaceState:
        self._require_exclusive()
        odd = self.read_state()
        if not odd.is_mutating or (
            odd.generation,
            odd.transaction_id,
            odd.operation,
            odd.map_name,
            odd.before_identity,
            odd.writer_instance_id,
        ) != (
            mutation.generation,
            mutation.transaction_id,
            mutation.operation,
            mutation.map_name,
            mutation.before_identity,
            mutation.writer_instance_id,
        ):
            raise PublishedNamespaceTransactionError(
                "mutation token 与当前 durable odd journal 不一致。"
            )
        return self._write_stable_successor(
            odd,
            after_identity=str(after_identity or "").strip(),
            outcome=outcome,
            updated_at_ns=updated_at_ns,
        )

    def _write_stable_successor(
        self,
        odd: PublishedNamespaceState,
        *,
        after_identity: str,
        outcome: str,
        updated_at_ns: Optional[int],
    ) -> PublishedNamespaceState:
        if odd.generation >= _MAX_GENERATION:
            raise PublishedNamespaceGenerationExhaustedError(
                "odd generation 无可表示的 even successor。"
            )
        stable = PublishedNamespaceState(
            schema_version=_SCHEMA_VERSION,
            protocol=_PROTOCOL,
            generation=odd.generation + 1,
            phase="stable",
            transaction_id=odd.transaction_id,
            operation=odd.operation,
            map_name=odd.map_name,
            before_identity=odd.before_identity,
            after_identity=after_identity,
            outcome=outcome,
            writer_instance_id=odd.writer_instance_id,
            updated_at_ns=_resolve_updated_at_ns(updated_at_ns),
        )
        stable._validate()
        self._protocol._replace_state(
            self._root_descriptor,
            self.root_identity,
            stable,
        )
        verified = self.read_state()
        if verified != stable:
            raise PublishedNamespaceIntegrityError(
                "stable journal 持久化后复读不一致。"
            )
        return verified

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            fcntl.flock(self._lock_descriptor, fcntl.LOCK_UN)
        finally:
            os.close(self._lock_descriptor)
            os.close(self._root_descriptor)

    def _require_open(self) -> None:
        if self._closed:
            raise PublishedNamespaceTransactionError("repository lease 已关闭。")

    def _require_exclusive(self) -> None:
        self._require_open()
        if self.mode != "exclusive":
            raise PublishedNamespaceTransactionError(
                "mutation journal primitive 必须持有 exclusive lease。"
            )


class PublishedNamespaceProtocol:
    """catalog-root 级 published namespace 持久协议。

    本 R0 helper 不自动 bootstrap，也不接入任何 writer/resolver。marker/state 缺失、
    odd residue、未知 schema、inode 漂移、owner/mode/type/nlink 非法均 fail closed。
    """

    def __init__(self, catalog_root: str, *, lease_timeout_seconds: float = 5.0) -> None:
        self._catalog_root = Path(catalog_root).expanduser()
        self._lease_timeout_seconds = _require_timeout(lease_timeout_seconds)
        root_descriptor, identity = _open_catalog_root(self._catalog_root)
        os.close(root_descriptor)
        self._catalog_root_identity = identity

    @property
    def catalog_root_identity(self) -> FileIdentity:
        return self._catalog_root_identity

    @property
    def lock_order(self) -> Tuple[str, str, str]:
        return (
            "map_activation_guard",
            "published_namespace_lease",
            "runtime_owner_read",
        )

    def bootstrap(self, *, writer_instance_id: str) -> PublishedNamespaceState:
        """在已停写且版本锁步的部署窗口显式创建 v1 协议。

        调用方负责先扫描 current/staging/legacy namespace；本函数故意不在普通读取时
        自动初始化，也不把不完整 repository 猜成安全状态。
        """

        resolved_writer = str(writer_instance_id or "").strip()
        _require_uuid(resolved_writer, "writer_instance_id")
        root_descriptor = self._open_verified_root()
        lock_descriptor = -1
        try:
            lock_descriptor = _open_or_create_lock(root_descriptor)
            _acquire_flock(
                lock_descriptor,
                exclusive=True,
                timeout_seconds=self._lease_timeout_seconds,
            )
            _verify_lock_entry(root_descriptor, lock_descriptor)
            marker_exists = _entry_exists(root_descriptor, _MARKER_FILE_NAME)
            state_exists = _entry_exists(root_descriptor, _STATE_FILE_NAME)
            if marker_exists or state_exists:
                if not marker_exists or not state_exists:
                    raise PublishedNamespaceIntegrityError(
                        "bootstrap residue 不完整；禁止覆盖或猜测恢复。"
                    )
                self._read_marker(root_descriptor, self._catalog_root_identity)
                return self._read_state(root_descriptor, self._catalog_root_identity)
            marker = _canonical_json_bytes(_marker_payload())
            _atomic_replace_bytes(
                root_descriptor,
                _MARKER_FILE_NAME,
                marker,
                replace_existing=False,
            )
            initial = PublishedNamespaceState(
                schema_version=_SCHEMA_VERSION,
                protocol=_PROTOCOL,
                generation=_INITIAL_GENERATION,
                phase="stable",
                transaction_id=str(uuid.uuid4()),
                operation="bootstrap",
                map_name="",
                before_identity="",
                after_identity="",
                outcome="initialized",
                writer_instance_id=resolved_writer,
                updated_at_ns=time.time_ns(),
            )
            initial._validate()
            _atomic_replace_bytes(
                root_descriptor,
                _STATE_FILE_NAME,
                initial.canonical_bytes(),
                replace_existing=False,
            )
            self._verify_root_descriptor(root_descriptor, self._catalog_root_identity)
            return self._read_state(root_descriptor, self._catalog_root_identity)
        finally:
            if lock_descriptor >= 0:
                try:
                    fcntl.flock(lock_descriptor, fcntl.LOCK_UN)
                finally:
                    os.close(lock_descriptor)
            os.close(root_descriptor)

    @contextmanager
    def shared_lease(
        self, *, timeout_seconds: Optional[float] = None
    ) -> Iterator[PublishedNamespaceLease]:
        with self._lease(exclusive=False, timeout_seconds=timeout_seconds) as lease:
            yield lease

    @contextmanager
    def exclusive_lease(
        self, *, timeout_seconds: Optional[float] = None
    ) -> Iterator[PublishedNamespaceLease]:
        with self._lease(exclusive=True, timeout_seconds=timeout_seconds) as lease:
            yield lease

    @contextmanager
    def _lease(
        self,
        *,
        exclusive: bool,
        timeout_seconds: Optional[float],
    ) -> Iterator[PublishedNamespaceLease]:
        timeout = (
            self._lease_timeout_seconds
            if timeout_seconds is None
            else _require_timeout(timeout_seconds)
        )
        root_descriptor = self._open_verified_root()
        lock_descriptor = -1
        lease: Optional[PublishedNamespaceLease] = None
        try:
            self._read_marker(root_descriptor, self._catalog_root_identity)
            lock_descriptor = _open_existing_file(
                root_descriptor,
                _LOCK_FILE_NAME,
                flags=os.O_RDWR,
            )
            _acquire_flock(
                lock_descriptor,
                exclusive=exclusive,
                timeout_seconds=timeout,
            )
            lock_identity = _verify_lock_entry(root_descriptor, lock_descriptor)
            self._verify_root_descriptor(root_descriptor, self._catalog_root_identity)
            lease = PublishedNamespaceLease(
                self,
                root_descriptor=root_descriptor,
                lock_descriptor=lock_descriptor,
                mode="exclusive" if exclusive else "shared",
                root_identity=self._catalog_root_identity,
                lock_identity=lock_identity,
            )
            root_descriptor = -1
            lock_descriptor = -1
            yield lease
        finally:
            if lease is not None:
                lease.close()
            else:
                if lock_descriptor >= 0:
                    os.close(lock_descriptor)
                if root_descriptor >= 0:
                    os.close(root_descriptor)

    def _open_verified_root(self) -> int:
        descriptor, identity = _open_catalog_root(self._catalog_root)
        if not _same_pinned_identity(identity, self._catalog_root_identity):
            os.close(descriptor)
            raise PublishedNamespaceIntegrityError(
                "catalog root 固定 inode/owner/mode 已漂移。"
            )
        return descriptor

    def _verify_root_descriptor(
        self,
        descriptor: int,
        expected: FileIdentity,
    ) -> None:
        actual = _identity_from_stat(os.fstat(descriptor), expected_type="directory")
        path_stat = os.stat(self._catalog_root, follow_symlinks=False)
        path_identity = _identity_from_stat(path_stat, expected_type="directory")
        if not _same_pinned_identity(actual, expected) or not _same_pinned_identity(
            path_identity, expected
        ):
            raise PublishedNamespaceIntegrityError(
                "catalog root 路径与持有 dirfd 不再指向固定可信 inode。"
            )

    def _read_marker(self, root_descriptor: int, root_identity: FileIdentity) -> None:
        raw = _read_stable_regular_file(
            root_descriptor,
            _MARKER_FILE_NAME,
            expected_mode=_FILE_MODE,
        )
        payload = _load_strict_json_object(raw, "published namespace marker")
        if payload != _marker_payload() or _canonical_json_bytes(payload) != raw:
            raise PublishedNamespaceIntegrityError(
                "published namespace marker schema/canonical bytes 非法。"
            )
        self._verify_root_descriptor(root_descriptor, root_identity)

    def _read_state(
        self,
        root_descriptor: int,
        root_identity: FileIdentity,
    ) -> PublishedNamespaceState:
        self._read_marker(root_descriptor, root_identity)
        raw = _read_stable_regular_file(
            root_descriptor,
            _STATE_FILE_NAME,
            expected_mode=_FILE_MODE,
        )
        state = PublishedNamespaceState.from_bytes(raw)
        self._verify_root_descriptor(root_descriptor, root_identity)
        return state

    def _replace_state(
        self,
        root_descriptor: int,
        root_identity: FileIdentity,
        state: PublishedNamespaceState,
    ) -> None:
        state._validate()
        self._read_marker(root_descriptor, root_identity)
        _atomic_replace_bytes(
            root_descriptor,
            _STATE_FILE_NAME,
            state.canonical_bytes(),
            replace_existing=True,
        )
        self._verify_root_descriptor(root_descriptor, root_identity)


def _marker_payload() -> Dict[str, object]:
    return {
        "generation_encoding": "signed-i64-even-stable-odd-mutating-no-wrap",
        "lock_file": _LOCK_FILE_NAME,
        "lock_order": "map_activation_guard->repository_lease->runtime_owner_read",
        "protocol": _PROTOCOL,
        "schema_version": _SCHEMA_VERSION,
        "state_file": _STATE_FILE_NAME,
        "threat_model": "cooperative-cross-process-advisory-flock",
    }


def _open_catalog_root(path: Path) -> Tuple[int, FileIdentity]:
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(str(path), flags)
    except OSError as exc:
        raise PublishedNamespaceUnavailableError(
            "catalog root 必须预先存在且不可为 symlink。"
        ) from exc
    try:
        identity = _identity_from_stat(os.fstat(descriptor), expected_type="directory")
        path_identity = _identity_from_stat(
            os.stat(path, follow_symlinks=False), expected_type="directory"
        )
        if not _same_pinned_identity(identity, path_identity):
            raise PublishedNamespaceIntegrityError(
                "catalog root open 前后身份不稳定。"
            )
        if identity.uid != os.geteuid() or identity.mode & 0o022:
            raise PublishedNamespaceIntegrityError(
                "catalog root 必须由当前服务账号拥有且不可 group/world writable。"
            )
        return descriptor, identity
    except Exception:
        os.close(descriptor)
        raise


def _identity_from_stat(result: os.stat_result, *, expected_type: str) -> FileIdentity:
    if expected_type == "directory":
        valid_type = stat.S_ISDIR(result.st_mode)
        minimum_links = 2
    elif expected_type == "regular":
        valid_type = stat.S_ISREG(result.st_mode)
        minimum_links = 1
    else:
        raise AssertionError("unsupported expected_type")
    if not valid_type or result.st_nlink < minimum_links:
        raise PublishedNamespaceIntegrityError(
            "协议对象 type/nlink 非法：%s。" % expected_type
        )
    return FileIdentity(
        device=int(result.st_dev),
        inode=int(result.st_ino),
        uid=int(result.st_uid),
        gid=int(result.st_gid),
        mode=stat.S_IMODE(result.st_mode),
        link_count=int(result.st_nlink),
        file_type=expected_type,
    )


def _same_pinned_identity(left: FileIdentity, right: FileIdentity) -> bool:
    return (
        left.device,
        left.inode,
        left.uid,
        left.gid,
        left.mode,
        left.file_type,
    ) == (
        right.device,
        right.inode,
        right.uid,
        right.gid,
        right.mode,
        right.file_type,
    ) and left.link_count >= (2 if left.file_type == "directory" else 1)


def _open_or_create_lock(root_descriptor: int) -> int:
    flags = os.O_RDWR | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    created = False
    try:
        descriptor = os.open(
            _LOCK_FILE_NAME,
            flags | os.O_CREAT | os.O_EXCL,
            _FILE_MODE,
            dir_fd=root_descriptor,
        )
        created = True
    except FileExistsError:
        descriptor = _open_existing_file(
            root_descriptor,
            _LOCK_FILE_NAME,
            flags=os.O_RDWR,
        )
    try:
        if created:
            os.fchmod(descriptor, _FILE_MODE)
            os.fsync(descriptor)
            os.fsync(root_descriptor)
        _verify_lock_entry(root_descriptor, descriptor)
        return descriptor
    except Exception:
        os.close(descriptor)
        raise


def _open_existing_file(root_descriptor: int, name: str, *, flags: int) -> int:
    resolved_flags = flags | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        return os.open(name, resolved_flags, dir_fd=root_descriptor)
    except FileNotFoundError as exc:
        raise PublishedNamespaceUnavailableError(
            "published namespace 协议文件缺失：%s。" % name
        ) from exc
    except OSError as exc:
        raise PublishedNamespaceIntegrityError(
            "published namespace 协议文件无法安全打开：%s。" % name
        ) from exc


def _verify_lock_entry(root_descriptor: int, descriptor: int) -> FileIdentity:
    opened = _identity_from_stat(os.fstat(descriptor), expected_type="regular")
    entry = _identity_from_stat(
        os.stat(_LOCK_FILE_NAME, dir_fd=root_descriptor, follow_symlinks=False),
        expected_type="regular",
    )
    if (
        not _same_pinned_identity(opened, entry)
        or opened.link_count != 1
        or opened.mode != _FILE_MODE
        or opened.uid != os.geteuid()
    ):
        raise PublishedNamespaceIntegrityError(
            "published namespace lock owner/mode/type/nlink/dev/ino 非法。"
        )
    return opened


def _read_stable_regular_file(
    root_descriptor: int,
    name: str,
    *,
    expected_mode: int,
) -> bytes:
    descriptor = _open_existing_file(root_descriptor, name, flags=os.O_RDONLY)
    try:
        before = _identity_from_stat(os.fstat(descriptor), expected_type="regular")
        if (
            before.link_count != 1
            or before.mode != expected_mode
            or before.uid != os.geteuid()
        ):
            raise PublishedNamespaceIntegrityError(
                "%s owner/mode/type/nlink 非法。" % name
            )
        chunks = []
        total = 0
        while True:
            chunk = os.read(descriptor, min(8192, _MAX_DOCUMENT_BYTES + 1 - total))
            if not chunk:
                break
            chunks.append(chunk)
            total += len(chunk)
            if total > _MAX_DOCUMENT_BYTES:
                raise PublishedNamespaceIntegrityError("%s 超过大小上限。" % name)
        after = _identity_from_stat(os.fstat(descriptor), expected_type="regular")
        entry = _identity_from_stat(
            os.stat(name, dir_fd=root_descriptor, follow_symlinks=False),
            expected_type="regular",
        )
        if before != after or not _same_pinned_identity(after, entry):
            raise PublishedNamespaceIntegrityError("%s 读取前后身份不稳定。" % name)
        return b"".join(chunks)
    finally:
        os.close(descriptor)


def _atomic_replace_bytes(
    root_descriptor: int,
    name: str,
    payload: bytes,
    *,
    replace_existing: bool,
) -> None:
    temporary_name = ".%s.%s.tmp" % (name, uuid.uuid4().hex)
    flags = (
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptor = os.open(temporary_name, flags, _FILE_MODE, dir_fd=root_descriptor)
    try:
        os.fchmod(descriptor, _FILE_MODE)
        view = memoryview(payload)
        written = 0
        while written < len(view):
            count = os.write(descriptor, view[written:])
            if count <= 0:
                raise PublishedNamespaceIntegrityError(
                    "published namespace journal write 未取得进展。"
                )
            written += count
        os.fsync(descriptor)
        identity = _identity_from_stat(os.fstat(descriptor), expected_type="regular")
        if identity.link_count != 1 or identity.mode != _FILE_MODE:
            raise PublishedNamespaceIntegrityError(
                "published namespace 临时 journal 身份非法。"
            )
    finally:
        os.close(descriptor)
    try:
        if not replace_existing and _entry_exists(root_descriptor, name):
            raise PublishedNamespaceIntegrityError("协议文件已存在，禁止 bootstrap 覆盖。")
        os.replace(
            temporary_name,
            name,
            src_dir_fd=root_descriptor,
            dst_dir_fd=root_descriptor,
        )
        os.fsync(root_descriptor)
    finally:
        try:
            os.unlink(temporary_name, dir_fd=root_descriptor)
        except FileNotFoundError:
            pass


def _entry_exists(root_descriptor: int, name: str) -> bool:
    try:
        os.stat(name, dir_fd=root_descriptor, follow_symlinks=False)
    except FileNotFoundError:
        return False
    return True


def _acquire_flock(
    descriptor: int,
    *,
    exclusive: bool,
    timeout_seconds: float,
) -> None:
    operation = (fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH) | fcntl.LOCK_NB
    deadline = time.monotonic() + timeout_seconds
    while True:
        try:
            fcntl.flock(descriptor, operation)
            return
        except OSError as exc:
            if exc.errno not in {errno.EACCES, errno.EAGAIN}:
                raise
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise PublishedNamespaceLeaseTimeoutError(
                    "repository %s lease 获取超时。"
                    % ("exclusive" if exclusive else "shared")
                ) from exc
            time.sleep(min(0.02, remaining))


def _canonical_json_bytes(payload: Mapping[str, object]) -> bytes:
    return json.dumps(
        payload,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _load_strict_json_object(raw: bytes, label: str) -> Dict[str, object]:
    if not raw or len(raw) > _MAX_DOCUMENT_BYTES:
        raise PublishedNamespaceIntegrityError("%s 大小非法。" % label)
    try:
        text = raw.decode("utf-8")
        pairs = json.loads(
            text,
            object_pairs_hook=lambda values: values,
            parse_constant=lambda value: (_raise_invalid_number(value)),
        )
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
        raise PublishedNamespaceIntegrityError("%s 不是严格 JSON。" % label) from exc
    if not isinstance(pairs, list):
        raise PublishedNamespaceIntegrityError("%s 顶层必须为 object。" % label)
    payload: Dict[str, object] = {}
    for item in pairs:
        if not isinstance(item, tuple) or len(item) != 2 or not isinstance(item[0], str):
            raise PublishedNamespaceIntegrityError("%s object 结构非法。" % label)
        key, value = item
        if key in payload:
            raise PublishedNamespaceIntegrityError("%s 包含重复字段。" % label)
        payload[key] = value
    return payload


def _raise_invalid_number(value: str) -> object:
    raise ValueError("invalid JSON number: %s" % value)


def _require_text(
    payload: Mapping[str, object],
    key: str,
    *,
    allow_empty: bool,
) -> str:
    value = payload.get(key)
    if not isinstance(value, str) or (not allow_empty and not value):
        raise PublishedNamespaceIntegrityError("state.%s 必须为 string。" % key)
    if value != value.strip() or "\x00" in value or len(value.encode("utf-8")) > 4096:
        raise PublishedNamespaceIntegrityError("state.%s 文本非法。" % key)
    return value


def _require_int(
    payload: Mapping[str, object],
    key: str,
    minimum: int,
    maximum: int,
) -> int:
    value = payload.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise PublishedNamespaceIntegrityError("state.%s 必须为 integer。" % key)
    if value < minimum or value > maximum:
        raise PublishedNamespaceIntegrityError("state.%s 超出范围。" % key)
    return value


def _require_uuid(value: str, label: str) -> None:
    try:
        parsed = uuid.UUID(value)
    except (ValueError, AttributeError) as exc:
        raise PublishedNamespaceIntegrityError("%s 必须为 canonical UUID。" % label) from exc
    if str(parsed) != value:
        raise PublishedNamespaceIntegrityError("%s 必须为 canonical UUID。" % label)


def _require_timeout(value: float) -> float:
    resolved = float(value)
    if not math.isfinite(resolved) or resolved <= 0:
        raise ValueError("lease timeout 必须为有限正数。")
    return resolved


def _resolve_updated_at_ns(value: Optional[int]) -> int:
    resolved = time.time_ns() if value is None else value
    if isinstance(resolved, bool) or not isinstance(resolved, int):
        raise PublishedNamespaceTransactionError("updated_at_ns 必须为 integer。")
    if resolved < 0 or resolved > _MAX_GENERATION:
        raise PublishedNamespaceTransactionError("updated_at_ns 超出 signed i64。")
    return resolved
