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
_ALLOWED_MUTATIONS = frozenset({"finalize", "delete", "legacy_finalize"})
_PROTOCOL_NAMES = frozenset({_LOCK_FILE_NAME, _STATE_FILE_NAME, _MARKER_FILE_NAME})


class PublishedNamespaceError(RuntimeError):
    """published namespace 协议拒绝继续操作。"""


class PublishedNamespaceUnavailableError(PublishedNamespaceError):
    """协议尚未 bootstrap，或持久协议文件缺失。"""


class PublishedNamespaceIntegrityError(PublishedNamespaceError):
    """catalog root、锁文件或 journal 身份/内容不可信。"""


class PublishedNamespaceLeaseTimeoutError(PublishedNamespaceError):
    """未能在有界时间内取得协作式 advisory lease。"""


class PublishedNamespaceRecoveryRequiredError(PublishedNamespaceError):
    """持久 residue 必须由显式 recovery owner 恢复。"""


class PublishedNamespaceGenerationExhaustedError(PublishedNamespaceError):
    """signed i64 generation 已无下一组 odd/even 值可用。"""


class PublishedNamespaceTransactionError(PublishedNamespaceError):
    """mutation journal 的阶段、事务身份或调用顺序非法。"""


@dataclass(frozen=True)
class FileIdentity:
    """安全打开文件的可审计身份。"""

    device: int
    inode: int
    size: int
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
    lock_path: str
    lock_device: int
    lock_inode: int
    lock_size: int
    lock_mode: int
    lock_uid: int
    lock_gid: int
    lock_nlink: int
    root_device: int
    root_inode: int
    root_mode: int
    root_uid: int
    root_gid: int
    root_nlink: int

    @property
    def is_stable(self) -> bool:
        return self.phase == "stable" and self.generation % 2 == 0

    @property
    def is_mutating(self) -> bool:
        return self.phase == "mutating" and self.generation % 2 == 1

    def canonical_bytes(self) -> bytes:
        return _canonical_json_bytes(self.__dict__)

    @classmethod
    def from_bytes(cls, raw: bytes) -> "PublishedNamespaceState":
        payload = _load_strict_json_object(raw, "published namespace state")
        expected = set(cls.__dataclass_fields__)
        if set(payload) != expected:
            raise PublishedNamespaceIntegrityError(
                "published namespace state 字段集合不符合 schema v1。"
            )
        text_fields = {
            "protocol", "phase", "transaction_id", "operation", "map_name",
            "before_identity", "after_identity", "outcome", "writer_instance_id",
            "lock_path",
        }
        empty_fields = {"map_name", "before_identity", "after_identity"}
        values = {}
        for name in expected:
            if name in text_fields:
                values[name] = _require_text(
                    payload, name, allow_empty=name in empty_fields
                )
            else:
                values[name] = _require_int(payload, name, 0, _MAX_GENERATION)
        state = cls(**values)
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
        _require_uuid(self.transaction_id, "transaction_id")
        _require_uuid(self.writer_instance_id, "writer_instance_id")
        if self.lock_path != _LOCK_FILE_NAME or self.lock_nlink != 1:
            raise PublishedNamespaceIntegrityError("state lock 持久绑定非法。")
        if self.lock_mode != _FILE_MODE or self.lock_size != 0:
            raise PublishedNamespaceIntegrityError("state lock mode/size 非法。")
        if self.root_device < 0 or self.root_inode < 1:
            raise PublishedNamespaceIntegrityError("state root identity 非法。")
        _validate_state_matrix(self)


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
    """持有 catalog-root 固定 inode 上的 SH 或 EX advisory lease。"""

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
            self._root_descriptor, self.root_identity, self.lock_identity
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
        if operation not in _ALLOWED_MUTATIONS:
            raise PublishedNamespaceTransactionError("mutation operation 不受支持。")
        resolved_map_name = str(map_name or "").strip()
        if not resolved_map_name:
            raise PublishedNamespaceTransactionError("mutation map_name 不能为空。")
        resolved_before = str(before_identity or "").strip()
        resolved_writer = str(writer_instance_id or "").strip()
        _require_uuid(resolved_writer, "writer_instance_id")
        resolved_transaction = str(transaction_id or uuid.uuid4())
        _require_uuid(resolved_transaction, "transaction_id")
        state = self._protocol._state_with_binding(
            generation=stable.generation + 1,
            phase="mutating",
            transaction_id=resolved_transaction,
            operation=operation,
            map_name=resolved_map_name,
            before_identity=resolved_before,
            after_identity="",
            outcome="in_progress",
            writer_instance_id=resolved_writer,
            updated_at_ns=_resolve_updated_at_ns(updated_at_ns),
            lock_identity=self.lock_identity,
        )
        self._protocol._replace_state(
            self._root_descriptor, self.root_identity, self.lock_identity, state
        )
        return PublishedNamespaceMutation(
            generation=state.generation,
            transaction_id=state.transaction_id,
            operation=state.operation,
            map_name=state.map_name,
            before_identity=state.before_identity,
            writer_instance_id=state.writer_instance_id,
        )

    def commit_mutation(
        self,
        mutation: PublishedNamespaceMutation,
        *,
        after_identity: str,
        updated_at_ns: Optional[int] = None,
    ) -> PublishedNamespaceState:
        return self._finish_mutation(
            mutation, after_identity=after_identity, outcome="committed",
            updated_at_ns=updated_at_ns
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
                if after_identity is None else str(after_identity).strip()
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
        """只前推一个可证明的 odd residue；不重置 generation 或恢复额度。"""

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
        if odd.transaction_id != str(expected_transaction_id or "").strip():
            raise PublishedNamespaceTransactionError(
                "recovery transaction_id 与 durable residue 不一致。"
            )
        return self._write_stable_successor(
            odd, after_identity=str(after_identity or "").strip(), outcome=outcome,
            updated_at_ns=updated_at_ns
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
            odd.generation, odd.transaction_id, odd.operation, odd.map_name,
            odd.before_identity, odd.writer_instance_id,
        ) != (
            mutation.generation, mutation.transaction_id, mutation.operation,
            mutation.map_name, mutation.before_identity, mutation.writer_instance_id,
        ):
            raise PublishedNamespaceTransactionError(
                "mutation token 与当前 durable odd journal 不一致。"
            )
        return self._write_stable_successor(
            odd, after_identity=str(after_identity or "").strip(), outcome=outcome,
            updated_at_ns=updated_at_ns
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
        stable = self._protocol._state_with_binding(
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
            lock_identity=self.lock_identity,
        )
        self._protocol._replace_state(
            self._root_descriptor, self.root_identity, self.lock_identity, stable
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
    """catalog-root 级 published namespace 持久协议。"""

    def __init__(self, catalog_root: str, *, lease_timeout_seconds: float = 5.0) -> None:
        self._catalog_root = Path(catalog_root).expanduser()
        self._lease_timeout_seconds = _require_timeout(lease_timeout_seconds)
        descriptor, identity = _open_catalog_root(self._catalog_root)
        self._catalog_root_descriptor = descriptor
        self._catalog_root_identity = identity

    def __del__(self) -> None:
        descriptor = getattr(self, "_catalog_root_descriptor", -1)
        if descriptor >= 0:
            try:
                os.close(descriptor)
            except OSError:
                pass
            self._catalog_root_descriptor = -1

    @property
    def catalog_root_identity(self) -> FileIdentity:
        return self._catalog_root_identity

    @property
    def lock_order(self) -> Tuple[str, str, str]:
        return (
            "map_activation_guard", "published_namespace_lease", "runtime_owner_read"
        )

    def bootstrap(self, *, writer_instance_id: str) -> PublishedNamespaceState:
        """只在无 marker/state/lock/temp 的全新目录初始化协议。"""

        resolved_writer = str(writer_instance_id or "").strip()
        _require_uuid(resolved_writer, "writer_instance_id")
        root_descriptor = self._open_verified_root()
        lock_descriptor = -1
        try:
            residues = _protocol_residues(root_descriptor)
            if residues:
                raise PublishedNamespaceRecoveryRequiredError(
                    "bootstrap 发现协议 residue；只能调用 recover_bootstrap 人工恢复：%s。"
                    % ",".join(residues)
                )
            lock_descriptor = _create_lock(root_descriptor)
            _acquire_flock(
                lock_descriptor, exclusive=True,
                timeout_seconds=self._lease_timeout_seconds
            )
            lock_identity = _verify_lock_entry(root_descriptor, lock_descriptor)
            transaction_id = str(uuid.uuid4())
            marker = self._marker_payload(
                lock_identity=lock_identity,
                transaction_id=transaction_id,
                writer_instance_id=resolved_writer,
                operation="bootstrap",
            )
            _atomic_replace_bytes(
                root_descriptor, _MARKER_FILE_NAME,
                _canonical_json_bytes(marker), replace_existing=False
            )
            initial = self._state_with_binding(
                generation=_INITIAL_GENERATION,
                phase="stable",
                transaction_id=transaction_id,
                operation="bootstrap",
                map_name="", before_identity="", after_identity="",
                outcome="initialized",
                writer_instance_id=resolved_writer,
                updated_at_ns=time.time_ns(),
                lock_identity=lock_identity,
            )
            _atomic_replace_bytes(
                root_descriptor, _STATE_FILE_NAME, initial.canonical_bytes(),
                replace_existing=False
            )
            return self._read_state(
                root_descriptor, self._catalog_root_identity, lock_identity
            )
        finally:
            if lock_descriptor >= 0:
                try:
                    fcntl.flock(lock_descriptor, fcntl.LOCK_UN)
                finally:
                    os.close(lock_descriptor)
            os.close(root_descriptor)

    def recover_bootstrap(
        self,
        *,
        writer_instance_id: str,
        expected_root_identity: FileIdentity,
        expected_lock_identity: FileIdentity,
    ) -> PublishedNamespaceState:
        """人工完成 bootstrap crash residue；这是唯一允许的恢复合同。

        调用方必须从停写窗口的审计收据传入 root/lock 完整身份。该操作只完成
        generation=2，不回退已有 generation，也不创建缺失 lock。
        """

        resolved_writer = str(writer_instance_id or "").strip()
        _require_uuid(resolved_writer, "writer_instance_id")
        if not _same_pinned_identity(expected_root_identity, self._catalog_root_identity):
            raise PublishedNamespaceIntegrityError("人工恢复的 root 身份收据不匹配。")
        root_descriptor = self._open_verified_root()
        lock_descriptor = -1
        try:
            lock_descriptor = _open_existing_file(
                root_descriptor, _LOCK_FILE_NAME, flags=os.O_RDWR
            )
            _acquire_flock(
                lock_descriptor, exclusive=True,
                timeout_seconds=self._lease_timeout_seconds
            )
            lock_identity = _verify_lock_entry(root_descriptor, lock_descriptor)
            if not _same_pinned_identity(lock_identity, expected_lock_identity):
                raise PublishedNamespaceIntegrityError("人工恢复的 lock 身份收据不匹配。")
            marker_exists = _entry_exists(root_descriptor, _MARKER_FILE_NAME)
            state_exists = _entry_exists(root_descriptor, _STATE_FILE_NAME)
            if state_exists:
                if not marker_exists:
                    raise PublishedNamespaceIntegrityError(
                        "state durable 但 marker 缺失，无法证明协议来源。"
                    )
                state = self._read_state(
                    root_descriptor, self._catalog_root_identity, lock_identity
                )
                if state.generation != _INITIAL_GENERATION:
                    raise PublishedNamespaceIntegrityError(
                        "recover_bootstrap 禁止重置已有 generation。"
                    )
                _remove_protocol_temporaries(root_descriptor)
                os.fsync(root_descriptor)
                return state
            transaction_id = str(uuid.uuid4())
            if marker_exists:
                marker = self._read_marker(
                    root_descriptor, self._catalog_root_identity, lock_identity
                )
                transaction_id = marker["transaction_id"]
            _remove_protocol_temporaries(root_descriptor)
            recovery_marker = self._marker_payload(
                lock_identity=lock_identity,
                transaction_id=transaction_id,
                writer_instance_id=resolved_writer,
                operation="bootstrap_recovery",
            )
            _atomic_replace_bytes(
                root_descriptor, _MARKER_FILE_NAME,
                _canonical_json_bytes(recovery_marker),
                replace_existing=marker_exists,
            )
            recovered = self._state_with_binding(
                generation=_INITIAL_GENERATION,
                phase="stable",
                transaction_id=transaction_id,
                operation="bootstrap_recovery",
                map_name="", before_identity="", after_identity="",
                outcome="recovered",
                writer_instance_id=resolved_writer,
                updated_at_ns=time.time_ns(),
                lock_identity=lock_identity,
            )
            _atomic_replace_bytes(
                root_descriptor, _STATE_FILE_NAME, recovered.canonical_bytes(),
                replace_existing=False
            )
            os.fsync(root_descriptor)
            return self._read_state(
                root_descriptor, self._catalog_root_identity, lock_identity
            )
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
        self, *, exclusive: bool, timeout_seconds: Optional[float]
    ) -> Iterator[PublishedNamespaceLease]:
        timeout = (
            self._lease_timeout_seconds if timeout_seconds is None
            else _require_timeout(timeout_seconds)
        )
        root_descriptor = self._open_verified_root()
        lock_descriptor = -1
        lease = None
        try:
            lock_descriptor = _open_existing_file(
                root_descriptor, _LOCK_FILE_NAME, flags=os.O_RDWR
            )
            _acquire_flock(
                lock_descriptor, exclusive=exclusive, timeout_seconds=timeout
            )
            lock_identity = _verify_lock_entry(root_descriptor, lock_descriptor)
            self._read_marker(
                root_descriptor, self._catalog_root_identity, lock_identity
            )
            self._verify_root_descriptor(
                root_descriptor, self._catalog_root_identity
            )
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
        try:
            descriptor = os.dup(self._catalog_root_descriptor)
        except OSError as exc:
            raise PublishedNamespaceUnavailableError(
                "catalog root 稳定 dirfd 已关闭。"
            ) from exc
        self._verify_root_descriptor(descriptor, self._catalog_root_identity)
        return descriptor

    def _verify_root_descriptor(
        self, descriptor: int, expected: FileIdentity
    ) -> None:
        actual = _identity_from_stat(os.fstat(descriptor), expected_type="directory")
        try:
            path_stat = os.stat(self._catalog_root, follow_symlinks=False)
            path_identity = _identity_from_stat(path_stat, expected_type="directory")
        except OSError as exc:
            raise PublishedNamespaceIntegrityError(
                "catalog root 当前路径无法验证。"
            ) from exc
        if not _same_pinned_identity(actual, expected) or not _same_pinned_identity(
            path_identity, expected
        ):
            raise PublishedNamespaceIntegrityError(
                "catalog root 路径与稳定 dirfd 不再指向持久绑定 inode。"
            )

    def _marker_payload(
        self,
        *,
        lock_identity: FileIdentity,
        transaction_id: str,
        writer_instance_id: str,
        operation: str,
    ) -> Dict[str, object]:
        payload = {
            "generation_encoding": "signed-i64-even-stable-odd-mutating-no-wrap",
            "lock_order": "map_activation_guard->repository_lease->runtime_owner_read",
            "protocol": _PROTOCOL,
            "schema_version": _SCHEMA_VERSION,
            "state_file": _STATE_FILE_NAME,
            "threat_model": "cooperative-cross-process-advisory-flock",
            "transaction_id": transaction_id,
            "writer_instance_id": writer_instance_id,
            "operation": operation,
        }
        payload.update(_binding_payload(self._catalog_root_identity, lock_identity))
        return payload

    def _read_marker(
        self,
        root_descriptor: int,
        root_identity: FileIdentity,
        lock_identity: FileIdentity,
    ) -> Dict[str, object]:
        raw = _read_stable_regular_file(
            root_descriptor, _MARKER_FILE_NAME, expected_mode=_FILE_MODE
        )
        payload = _load_strict_json_object(raw, "published namespace marker")
        expected = set(self._marker_payload(
            lock_identity=lock_identity,
            transaction_id=str(uuid.uuid4()),
            writer_instance_id=str(uuid.uuid4()),
            operation="bootstrap",
        ))
        if set(payload) != expected:
            raise PublishedNamespaceIntegrityError(
                "published namespace marker 字段集合非法。"
            )
        if payload["protocol"] != _PROTOCOL or payload["schema_version"] != 1:
            raise PublishedNamespaceIntegrityError("marker 协议版本非法。")
        if payload["operation"] not in {"bootstrap", "bootstrap_recovery"}:
            raise PublishedNamespaceIntegrityError("marker operation 非法。")
        _require_uuid(_require_text(payload, "transaction_id", allow_empty=False), "transaction_id")
        _require_uuid(_require_text(payload, "writer_instance_id", allow_empty=False), "writer_instance_id")
        expected_binding = _binding_payload(root_identity, lock_identity)
        for key, value in expected_binding.items():
            if payload.get(key) != value:
                raise PublishedNamespaceIntegrityError(
                    "marker 持久 root/lock 身份绑定不匹配。"
                )
        fixed = self._marker_payload(
            lock_identity=lock_identity,
            transaction_id=payload["transaction_id"],
            writer_instance_id=payload["writer_instance_id"],
            operation=payload["operation"],
        )
        if payload != fixed or _canonical_json_bytes(payload) != raw:
            raise PublishedNamespaceIntegrityError(
                "published namespace marker schema/canonical bytes 非法。"
            )
        self._verify_root_descriptor(root_descriptor, root_identity)
        return payload

    def _read_state(
        self,
        root_descriptor: int,
        root_identity: FileIdentity,
        lock_identity: FileIdentity,
    ) -> PublishedNamespaceState:
        marker = self._read_marker(root_descriptor, root_identity, lock_identity)
        raw = _read_stable_regular_file(
            root_descriptor, _STATE_FILE_NAME, expected_mode=_FILE_MODE
        )
        state = PublishedNamespaceState.from_bytes(raw)
        _verify_state_binding(state, root_identity, lock_identity)
        if state.operation in {"bootstrap", "bootstrap_recovery"} and (
            state.transaction_id != marker["transaction_id"]
            or state.operation != marker["operation"]
            or state.writer_instance_id != marker["writer_instance_id"]
        ):
            raise PublishedNamespaceIntegrityError(
                "bootstrap state 与 marker 恢复合同不一致。"
            )
        self._verify_root_descriptor(root_descriptor, root_identity)
        return state

    def _replace_state(
        self,
        root_descriptor: int,
        root_identity: FileIdentity,
        lock_identity: FileIdentity,
        state: PublishedNamespaceState,
    ) -> None:
        state._validate()
        self._read_marker(root_descriptor, root_identity, lock_identity)
        _verify_state_binding(state, root_identity, lock_identity)
        _atomic_replace_bytes(
            root_descriptor, _STATE_FILE_NAME, state.canonical_bytes(),
            replace_existing=True
        )
        self._verify_root_descriptor(root_descriptor, root_identity)

    def _state_with_binding(self, *, lock_identity: FileIdentity, **values: object) -> PublishedNamespaceState:
        values.update(_binding_payload(self._catalog_root_identity, lock_identity))
        state = PublishedNamespaceState(
            schema_version=_SCHEMA_VERSION, protocol=_PROTOCOL, **values
        )
        state._validate()
        return state


def _binding_payload(root: FileIdentity, lock: FileIdentity) -> Dict[str, object]:
    return {
        "lock_path": _LOCK_FILE_NAME,
        "lock_device": lock.device,
        "lock_inode": lock.inode,
        "lock_size": lock.size,
        "lock_mode": lock.mode,
        "lock_uid": lock.uid,
        "lock_gid": lock.gid,
        "lock_nlink": lock.link_count,
        "root_device": root.device,
        "root_inode": root.inode,
        "root_mode": root.mode,
        "root_uid": root.uid,
        "root_gid": root.gid,
        "root_nlink": root.link_count,
    }


def _verify_state_binding(
    state: PublishedNamespaceState,
    root: FileIdentity,
    lock: FileIdentity,
) -> None:
    expected = _binding_payload(root, lock)
    for key, value in expected.items():
        if getattr(state, key) != value:
            raise PublishedNamespaceIntegrityError(
                "state 持久 root/lock 身份绑定不匹配。"
            )


def _validate_state_matrix(state: PublishedNamespaceState) -> None:
    bootstrap = state.operation in {"bootstrap", "bootstrap_recovery"}
    if bootstrap:
        expected_outcome = (
            "initialized" if state.operation == "bootstrap" else "recovered"
        )
        if (
            state.phase != "stable" or state.generation != _INITIAL_GENERATION
            or state.outcome != expected_outcome or state.map_name
            or state.before_identity or state.after_identity
        ):
            raise PublishedNamespaceIntegrityError(
                "bootstrap operation/outcome/map/identity 组合非法。"
            )
        return
    if state.operation not in _ALLOWED_MUTATIONS or not state.map_name:
        raise PublishedNamespaceIntegrityError(
            "state operation/map_name 组合非法。"
        )
    if state.phase == "mutating":
        if state.generation % 2 != 1 or state.outcome != "in_progress" or state.after_identity:
            raise PublishedNamespaceIntegrityError(
                "mutating operation/outcome/identity 组合非法。"
            )
        return
    if state.phase != "stable" or state.generation % 2 != 0:
        raise PublishedNamespaceIntegrityError("state phase/generation 组合非法。")
    if state.outcome not in {"committed", "aborted", "recovered"}:
        raise PublishedNamespaceIntegrityError("stable outcome 组合非法。")
    if state.operation == "delete":
        if state.outcome == "committed" and (not state.before_identity or state.after_identity):
            raise PublishedNamespaceIntegrityError("delete committed identity 组合非法。")
        if state.outcome == "aborted" and state.after_identity != state.before_identity:
            raise PublishedNamespaceIntegrityError("delete aborted identity 组合非法。")
    elif state.outcome == "committed" and not state.after_identity:
        raise PublishedNamespaceIntegrityError("finalize committed 必须有 after_identity。")
    elif state.outcome == "aborted" and state.after_identity != state.before_identity:
        raise PublishedNamespaceIntegrityError("finalize aborted identity 组合非法。")


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
            raise PublishedNamespaceIntegrityError("catalog root open 前后身份不稳定。")
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
        device=int(result.st_dev), inode=int(result.st_ino), size=int(result.st_size),
        uid=int(result.st_uid), gid=int(result.st_gid),
        mode=stat.S_IMODE(result.st_mode), link_count=int(result.st_nlink),
        file_type=expected_type,
    )


def _same_pinned_identity(left: FileIdentity, right: FileIdentity) -> bool:
    return (
        left.device, left.inode, left.size, left.uid, left.gid, left.mode,
        left.link_count, left.file_type,
    ) == (
        right.device, right.inode, right.size, right.uid, right.gid, right.mode,
        right.link_count, right.file_type,
    )


def _create_lock(root_descriptor: int) -> int:
    flags = (
        os.O_RDWR | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    try:
        descriptor = os.open(
            _LOCK_FILE_NAME, flags, _FILE_MODE, dir_fd=root_descriptor
        )
    except FileExistsError as exc:
        raise PublishedNamespaceRecoveryRequiredError(
            "lock 已存在；bootstrap 禁止自动复用或重建。"
        ) from exc
    try:
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
    try:
        entry_stat = os.stat(
            _LOCK_FILE_NAME, dir_fd=root_descriptor, follow_symlinks=False
        )
    except OSError as exc:
        raise PublishedNamespaceIntegrityError(
            "published namespace lock 当前目录项缺失。"
        ) from exc
    entry = _identity_from_stat(entry_stat, expected_type="regular")
    if (
        not _same_pinned_identity(opened, entry) or opened.link_count != 1
        or opened.mode != _FILE_MODE or opened.uid != os.geteuid()
        or opened.size != 0
    ):
        raise PublishedNamespaceIntegrityError(
            "published namespace lock owner/mode/type/nlink/dev/ino/size 非法。"
        )
    return opened


def _read_stable_regular_file(
    root_descriptor: int, name: str, *, expected_mode: int
) -> bytes:
    descriptor = _open_existing_file(root_descriptor, name, flags=os.O_RDONLY)
    try:
        before = _identity_from_stat(os.fstat(descriptor), expected_type="regular")
        if before.link_count != 1 or before.mode != expected_mode or before.uid != os.geteuid():
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
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0)
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
        if replace_existing:
            os.replace(
                temporary_name, name,
                src_dir_fd=root_descriptor, dst_dir_fd=root_descriptor
            )
        else:
            try:
                os.link(
                    temporary_name, name,
                    src_dir_fd=root_descriptor, dst_dir_fd=root_descriptor,
                    follow_symlinks=False,
                )
            except FileExistsError as exc:
                raise PublishedNamespaceIntegrityError(
                    "协议文件已存在，禁止 bootstrap 覆盖。"
                ) from exc
            os.unlink(temporary_name, dir_fd=root_descriptor)
        os.fsync(root_descriptor)
    finally:
        try:
            os.unlink(temporary_name, dir_fd=root_descriptor)
            os.fsync(root_descriptor)
        except FileNotFoundError:
            pass


def _protocol_residues(root_descriptor: int) -> Tuple[str, ...]:
    names = []
    for name in os.listdir(root_descriptor):
        if name in _PROTOCOL_NAMES or _is_protocol_temporary(name):
            names.append(name)
    return tuple(sorted(names))


def _is_protocol_temporary(name: str) -> bool:
    return any(
        name.startswith(".%s." % target) and name.endswith(".tmp")
        for target in (_MARKER_FILE_NAME, _STATE_FILE_NAME)
    )


def _remove_protocol_temporaries(root_descriptor: int) -> None:
    removed = False
    for name in os.listdir(root_descriptor):
        if _is_protocol_temporary(name):
            try:
                identity = _identity_from_stat(
                    os.stat(name, dir_fd=root_descriptor, follow_symlinks=False),
                    expected_type="regular",
                )
            except OSError as exc:
                raise PublishedNamespaceIntegrityError(
                    "bootstrap temp residue 无法验证。"
                ) from exc
            if identity.uid != os.geteuid() or identity.link_count != 1:
                raise PublishedNamespaceIntegrityError(
                    "bootstrap temp residue 身份非法。"
                )
            os.unlink(name, dir_fd=root_descriptor)
            removed = True
    if removed:
        os.fsync(root_descriptor)


def _entry_exists(root_descriptor: int, name: str) -> bool:
    try:
        os.stat(name, dir_fd=root_descriptor, follow_symlinks=False)
    except FileNotFoundError:
        return False
    return True


def _acquire_flock(
    descriptor: int, *, exclusive: bool, timeout_seconds: float
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
        payload, ensure_ascii=False, allow_nan=False, sort_keys=True,
        separators=(",", ":")
    ).encode("utf-8")


def _load_strict_json_object(raw: bytes, label: str) -> Dict[str, object]:
    if not raw or len(raw) > _MAX_DOCUMENT_BYTES:
        raise PublishedNamespaceIntegrityError("%s 大小非法。" % label)
    try:
        text = raw.decode("utf-8")
        pairs = json.loads(
            text, object_pairs_hook=lambda values: values,
            parse_constant=lambda value: _raise_invalid_number(value)
        )
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
        raise PublishedNamespaceIntegrityError("%s 不是严格 JSON。" % label) from exc
    if not isinstance(pairs, list):
        raise PublishedNamespaceIntegrityError("%s 顶层必须为 object。" % label)
    payload = {}
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
    payload: Mapping[str, object], key: str, *, allow_empty: bool
) -> str:
    value = payload.get(key)
    if not isinstance(value, str) or (not allow_empty and not value):
        raise PublishedNamespaceIntegrityError("state.%s 必须为 string。" % key)
    if value != value.strip() or "\x00" in value or len(value.encode("utf-8")) > 4096:
        raise PublishedNamespaceIntegrityError("state.%s 文本非法。" % key)
    return value


def _require_int(
    payload: Mapping[str, object], key: str, minimum: int, maximum: int
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
