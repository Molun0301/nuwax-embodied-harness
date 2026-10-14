"""本地制品存储；侧车文件是权威数据，维护使用单次扫描的时间点视图。"""

from __future__ import annotations

import os
import re
import threading
import time
import weakref
from collections import OrderedDict
from contextlib import contextmanager
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from itertools import islice
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, Iterator, List, Optional, Tuple

from contracts.artifacts import (
    ArtifactCleanupResult,
    ArtifactKind,
    ArtifactRef,
    ArtifactRetentionPolicy,
    ArtifactStorageSummary,
)
from contracts.base import utc_now
from contracts.naming import ARTIFACT_ID_PATTERN, build_artifact_id
from core.errors import ArtifactNotFoundError, GatewayError


class _RootState:
    def __init__(self) -> None:
        self.mutation_lock = threading.RLock()
        self.cleanup_lock = threading.Lock()
        self.destructive_epoch = 0
        self.reference_epoch = 0
        self.owner_lock = threading.Lock()
        self.maintenance_owner = None
        self.pins: Dict[str, int] = {}
        self.holders: Dict[object, object] = {}
        self.local_pins: Dict[str, int] = {}
        self.deleted_while_held = set()
        self.owned_ids: OrderedDict = OrderedDict()
        self.reference_sources: Dict[object, object] = {}
        self.local = threading.local()


_METADATA_CACHE_LIMIT = 16384
_METADATA_CACHE_MAX_BYTES = 16 * 1024 * 1024
_OWNED_ID_LIMIT = 16384
_ARTIFACT_FILENAME = re.compile(r"^(art_[a-z0-9_]+_[0-9]{8}T[0-9]{6}Z_[a-f0-9]{8})")
_ROOT_STATES_LOCK = threading.Lock()
_ROOT_STATES: weakref.WeakValueDictionary = weakref.WeakValueDictionary()


def _root_state(directory: Path) -> _RootState:
    key = (os.getpid(), str(directory.resolve()))
    with _ROOT_STATES_LOCK:
        state = _ROOT_STATES.get(key)
        if state is None:
            state = _RootState()
            _ROOT_STATES[key] = state
        return state


def _identity(path: Path) -> Tuple[int, int, int, int, int]:
    stat = path.stat()
    return stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns


def compact_maintenance_status(value):
    """健康端点只复制固定字段；不遍历整个制品/保护 ID 列表。"""
    import copy
    fields = ("status", "reason", "pending_count", "pending_bytes", "pending_expired", "error_code",
              "error_type", "summary_scope", "request_sequence", "triggered_at", "policy_revision",
              "trigger_artifact_id")
    result = {key: value[key] for key in fields if key in value}
    if "errors" in value:
        result["errors"] = list(value["errors"][:16])
    cleanup = value.get("result")
    if isinstance(cleanup, dict):
        result["result"] = {key: copy.deepcopy(cleanup[key]) for key in
                            ("removed_count", "freed_bytes", "protected_count", "summary") if key in cleanup}
    return result


@dataclass(frozen=True)
class _ArtifactEntry:
    ref: ArtifactRef
    meta_path: Path
    meta_identity: Tuple[int, int, int, int, int]
    file_path: Optional[Path]
    file_identity: Optional[Tuple[int, int, int, int, int]]


@dataclass(frozen=True)
class ArtifactDeletionPermit:
    """外部保护器检查引用后返回的不可变令牌；最终校验不调用外部代码。"""

    root_token: object
    reference_epoch: int


class ArtifactHold:
    """内部拥有方句柄；显式释放，不把仍在运行的工作按时钟强制失效。

    句柄不进入 ArtifactRef。保留量随实际操作、有限队列和已发布槽位增长；
    不合作的拥有方保留原句柄并在状态中可见，不创建续租线程或历史账本。
    """

    def __init__(self, store, owner: str) -> None:
        self._store = store
        self._token = object()
        self.owner = str(owner)[:128]
        self.artifact_ids = set()
        self.local_ids = set()
        self.missing_local_ids = set()
        self.unresolved_ids = set()
        self._released = False
        self.started_at = time.monotonic()

    def _add_locked(self, artifact_id, *, local=False, missing=False) -> None:
        if self._released:
            raise RuntimeError("制品拥有方已释放")
        state = self._store._state
        if artifact_id not in self.artifact_ids:
            if not self.artifact_ids:
                self.started_at = time.monotonic()
            self.artifact_ids.add(artifact_id)
            state.pins[artifact_id] = state.pins.get(artifact_id, 0) + 1
            state.holders[self._token] = self
        if local and artifact_id not in self.local_ids:
            self.local_ids.add(artifact_id)
            state.local_pins[artifact_id] = state.local_pins.get(artifact_id, 0) + 1
        if missing:
            self.missing_local_ids.add(artifact_id)
        if not local:
            self.unresolved_ids.add(artifact_id)

    def validate_introduced(self, existing_ids=()) -> None:
        """源 SQL 前仅检查已取得的证明；本方法不做文件、仓储或模型 I/O。"""
        with self._store.maintenance_fence():
            if self._released:
                raise GatewayError("制品发布拥有方已结束。", error_code="artifact_owner_closed", http_status=409)
            missing = (self.missing_local_ids | (self.local_ids & self._store._state.deleted_while_held)) - set(existing_ids)
            if missing:
                raise GatewayError("新引用的本地制品已不存在。", error_code="artifact_publication_missing", http_status=409,
                                   details={"artifact_ids": sorted(missing)})

    def retain(self, *, owner: str):
        """在旧拥有方仍有效时创建独立槽位；无查盘、无无保护转移窗口。"""
        with self._store.maintenance_fence():
            self.validate_introduced(self.artifact_ids)
            other = ArtifactHold(self._store, owner)
            for artifact_id in self.artifact_ids:
                other._add_locked(artifact_id, local=artifact_id in self.local_ids,
                                  missing=artifact_id in self.missing_local_ids)
            return other

    def add_existing_references(self, artifact_ids) -> None:
        """接管权威旧记录引用；不把已有断链升级为新写入失败，不做 I/O。"""
        with self._store.maintenance_fence():
            state = self._store._state
            for artifact_id in artifact_ids:
                self._add_locked(artifact_id, local=(artifact_id in state.owned_ids or artifact_id in state.local_pins))

    def release(self, *, reference_changed: bool = False) -> None:
        """完成源事务时在同一短栅栏内推进代际并释放；重复释放无副作用。"""
        with self._store.maintenance_fence():
            if self._released:
                return
            state = self._store._state
            if reference_changed:
                state.reference_epoch += 1
            self._released = True
            state.holders.pop(self._token, None)
            for artifact_id in self.artifact_ids:
                remaining = state.pins[artifact_id] - 1
                if remaining:
                    state.pins[artifact_id] = remaining
                else:
                    state.pins.pop(artifact_id, None)
                    state.deleted_while_held.discard(artifact_id)
                if artifact_id in self.local_ids:
                    local_remaining = state.local_pins[artifact_id] - 1
                    if local_remaining:
                        state.local_pins[artifact_id] = local_remaining
                    else:
                        state.local_pins.pop(artifact_id, None)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.release()


@dataclass(frozen=True)
class ArtifactCleanupContext:
    """后台维护显式契约；未提供 prepare_delete 时绝不允许删除。

    prepare_delete 仅在文件锁外运行，返回不可变引用代际令牌。
    is_current 必须是拥有方提供的短、无 I/O 状态检查。令牌本身不替代真实
    引用检查；生产调用方必须接入引用桥。outcome 保存本次请求独立的维护结果。
    """

    as_of: datetime
    is_current: Callable[[], bool]
    prepare_delete: Optional[Callable[[str], Optional[ArtifactDeletionPermit]]] = None
    outcome: Dict[str, object] = field(default_factory=dict)

    def check_current(self) -> None:
        if not self.is_current():
            raise GatewayError("制品维护请求已失效。", error_code="artifact_maintenance_cancelled", http_status=409)


class LocalArtifactStore:
    """宿主机本地制品存储；同进程同目录实例共享文件变更锁及临时保护。

    外部进程的文件变更在每次扫描可见，但扫描并不是跨进程或跨记忆库事务。
    引用提供器在文件变更锁外调用，避免与调用方的仓储锁形成倒序锁依赖。
    """

    def __init__(self, base_dir: str, public_base_url: str) -> None:
        self._base_dir = Path(base_dir)
        self._base_dir.mkdir(parents=True, exist_ok=True)
        self._public_base_url = public_base_url.rstrip("/")
        self._cache_lock = threading.RLock()
        self._metadata_cache: OrderedDict = OrderedDict()
        self._metadata_cache_bytes = 0
        self._paths: Dict[str, Path] = {}
        self._reference_provider: Optional[Callable[[], object]] = None
        self._state = _root_state(self._base_dir)
        self._maintenance_status: Dict[str, object] = {"status": "not_run"}

    @property
    def base_dir(self) -> Path:
        """返回制品根目录。"""
        return self._base_dir

    def set_reference_provider(self, provider: Optional[Callable[[], object]]) -> None:
        """设置跨服务制品引用索引提供器。"""
        with self.maintenance_fence():
            self._reference_provider = provider
            self._state.reference_epoch += 1

    def register_reference_source(self, token, snapshot, lookup) -> None:
        """登记合作仓储；根目录的所有实例均使用该来源的最终删除校验。"""
        with self.maintenance_fence():
            self._state.reference_sources[token] = (snapshot, lookup)
            self._state.reference_epoch += 1

    def unregister_reference_source(self, token) -> None:
        with self.maintenance_fence():
            if self._state.reference_sources.pop(token, None) is not None:
                self._state.reference_epoch += 1

    def prepare_registered_deletion(self, artifact_id: str) -> ArtifactDeletionPermit:
        """快照之前捕获代际；所有外部查引用均在文件变更锁外。"""
        with self.maintenance_fence():
            generation = self._state.reference_epoch
            sources = tuple(self._state.reference_sources.values())
        if not sources:
            raise GatewayError("尚未登记制品引用来源。", error_code="artifact_guard_unavailable", http_status=503)
        if getattr(self._state.local, "registered_check_active", False):
            raise GatewayError("制品引用检查不可递归调用。", error_code="artifact_reference_unavailable", http_status=503)
        self._state.local.registered_check_active = True
        try:
            for _snapshot, lookup in sources:
                references = lookup(artifact_id)
                if references:
                    raise GatewayError("制品仍有仓储引用。", error_code="artifact_in_use", http_status=409,
                                       details={"artifact_id": artifact_id, "references": list(references)})
        except GatewayError:
            raise
        except Exception as exc:
            raise GatewayError("制品引用状态不可用。", error_code="artifact_reference_unavailable", http_status=503) from exc
        finally:
            self._state.local.registered_check_active = False
        return self.deletion_permit(generation)

    def _remember_owned_locked(self, artifact_id, present=True) -> None:
        self._state.owned_ids[artifact_id] = bool(present)
        self._state.owned_ids.move_to_end(artifact_id)
        while len(self._state.owned_ids) > _OWNED_ID_LIMIT:
            self._state.owned_ids.popitem(last=False)

    def acquire_artifacts(self, artifact_ids=(), *, owner: str, local_ids=()) -> ArtifactHold:
        """查验可证明属于本根目录的制品，再原子取得保护。

        未知/外部标识保留为未解析引用；仅凭 art_ 命名不能证明所有权。
        目录查找和元数据解析不持有文件变更锁。最终身份复核仅针对候选文件。
        local_ids 只由内部已证明来源的句柄/ArtifactRef 调用方提供。
        """
        hold = ArtifactHold(self, owner)
        proven = set(local_ids)
        try:
            for artifact_id in dict.fromkeys(str(value).strip() for value in artifact_ids if str(value).strip()):
                with self.maintenance_fence():
                    known = (artifact_id in proven or artifact_id in self._state.owned_ids
                             or artifact_id in self._state.local_pins)
                entry = None
                if len(artifact_id) <= 255 and ARTIFACT_ID_PATTERN.fullmatch(artifact_id):
                    meta_path = self._find_meta_path(artifact_id)
                    if meta_path is not None:
                        known = True
                        try:
                            cached = self._paths.get(artifact_id)
                            entry = self._read_entry(meta_path, cached if cached is not None and cached.is_file()
                                                     else self._find_file_path(artifact_id))
                        except Exception:  # noqa: BLE001 - 已有损坏侧车是诊断欠账，新引用由准入校验拒绝
                            entry = None
                with self.maintenance_fence():
                    present = entry is not None and entry.file_path is not None and entry.file_identity is not None
                    if present:
                        try:
                            present = (_identity(entry.meta_path) == entry.meta_identity
                                       and _identity(entry.file_path) == entry.file_identity)
                        except FileNotFoundError:
                            present = False
                    if known:
                        self._remember_owned_locked(artifact_id, present)
                    if present:
                        self._paths[artifact_id] = entry.file_path
                    hold._add_locked(artifact_id, local=known, missing=known and not present)
            return hold
        except BaseException:
            hold.release()
            raise

    def hold_proven_artifacts(self, artifact_ids=(), *, owner: str) -> ArtifactHold:
        """外层 SQL 已打开时只接管内存证明；未知 ID 留作未解析诊断。

        已知本地保存/删除凭据及已有拥有方足以防止合作删除竞态。此路径不
        查盘、不推断命名空间，也不宣称检测了非合作的外部文件变更。
        """
        hold = ArtifactHold(self, owner)
        with self.maintenance_fence():
            state = self._state
            for artifact_id in dict.fromkeys(str(value).strip() for value in artifact_ids if str(value).strip()):
                local = artifact_id in state.owned_ids or artifact_id in state.local_pins
                missing = local and (state.owned_ids.get(artifact_id) is False or artifact_id in state.deleted_while_held)
                hold._add_locked(artifact_id, local=local, missing=missing)
        return hold

    @contextmanager
    def publication_scope(self, *, owner: str) -> Iterator[ArtifactHold]:
        """本线程新保存的制品在同一次发布中被句柄接管；退出必定释放。"""
        hold = ArtifactHold(self, owner)
        scopes = getattr(self._state.local, "hold_scopes", None)
        if scopes is None:
            scopes = self._state.local.hold_scopes = []
        scopes.append(hold)
        try:
            yield hold
        finally:
            scopes.pop()
            hold.release()

    def get_holder_status(self) -> Dict[str, object]:
        """只读计数及有界拥有方诊断；不扫描文件，也不暴露私有句柄。"""
        with self.maintenance_fence():
            oldest = next(iter(self._state.holders.values()), None)
            return {"holder_count": len(self._state.holders), "protected_artifact_count": len(self._state.pins),
                    "owned_receipt_count": len(self._state.owned_ids), "owned_receipt_limit": _OWNED_ID_LIMIT,
                    "oldest_holder_age_sec": time.monotonic() - oldest.started_at if oldest is not None else 0.0,
                    "owners": [item.owner for item in islice(self._state.holders.values(), 16)],
                    "reference_source_count": len(self._state.reference_sources)}

    @contextmanager
    def read_artifact(self, artifact_id: str):
        """保护元数据/文件打开到字节读取结束；返回已打开流，读取不持有变更锁。"""
        with self.acquire_artifacts((artifact_id,), owner="artifact.reader"):
            ref = self.get_ref(artifact_id)
            with self.maintenance_fence():
                stream = self.get_file_path(artifact_id).open("rb")
            try:
                yield ref, stream
            finally:
                stream.close()

    def read_bytes(self, artifact_id: str, *, max_bytes=None) -> bytes:
        """将文件复制到调用方拥有的字节后立即释放读取保护。"""
        with self.read_artifact(artifact_id) as (_ref, stream):
            data = stream.read() if max_bytes is None else stream.read(max(0, int(max_bytes)) + 1)
            if max_bytes is not None and len(data) > max_bytes:
                raise GatewayError("制品超过读取大小上限。", error_code="artifact_too_large", http_status=413)
            return data

    def reference_epoch(self) -> int:
        with self._state.mutation_lock:
            return self._state.reference_epoch

    def reference_changed(self) -> int:
        """未来引用桥的代际推进；此调用本身不登记引用或保证发布原子性。

        调用方须先持有覆盖源事务的发布保护，并在同一个 maintenance_fence
        内推进代际、释放保护。先无保护地发布、再调用本方法仍存在删除窗口；
        不应以把慢源事务塞进此短栅栏来替代发布保护。当前模块未接入仓储引用桥。
        """
        with self._state.mutation_lock:
            self._state.reference_epoch += 1
            return self._state.reference_epoch

    def deletion_permit(self, checked_epoch: int) -> ArtifactDeletionPermit:
        """封装引用快照之前读取的代际；不得用快照之后的新代际掩盖并发发布。

        此方法不查询引用，也不把无条件颁发令牌变成安全的生产保护器。
        """
        return ArtifactDeletionPermit(self._state, checked_epoch)

    def _permit_is_current(self, permit: ArtifactDeletionPermit) -> bool:
        return permit.root_token is self._state and permit.reference_epoch == self._state.reference_epoch

    @contextmanager
    def maintenance_fence(self) -> Iterator[None]:
        """仅供短拥有方/策略状态变更；不得包裹扫描、提供器或模型调用。"""
        with self._state.mutation_lock:
            yield

    def claim_maintenance_owner(self, token: object) -> bool:
        """同进程同目录至多一个后台拥有方；不改变手动清理接口。"""
        with self._state.owner_lock:
            if self._state.maintenance_owner is not None and self._state.maintenance_owner is not token:
                return False
            self._state.maintenance_owner = token
            return True

    def is_maintenance_owner(self, token: object) -> bool:
        with self._state.owner_lock:
            return self._state.maintenance_owner is token

    def release_maintenance_owner(self, token: object) -> None:
        with self._state.owner_lock:
            if self._state.maintenance_owner is token:
                self._state.maintenance_owner = None

    @contextmanager
    def protect_saved_artifacts(self) -> Iterator[None]:
        """临时保护本线程在作用域内发布的制品，退出时释放，不留下永久保留集。"""
        scopes = getattr(self._state.local, "save_scopes", None)
        if scopes is None:
            scopes = self._state.local.save_scopes = []
        saved: List[str] = []
        scopes.append(saved)
        try:
            yield
        finally:
            scopes.pop()
            with self._state.mutation_lock:
                for artifact_id in saved:
                    remaining = self._state.pins[artifact_id] - 1
                    if remaining:
                        self._state.pins[artifact_id] = remaining
                    else:
                        self._state.pins.pop(artifact_id)
                        self._state.deleted_while_held.discard(artifact_id)

    def save_bytes(
        self,
        *,
        kind: ArtifactKind,
        mime_type: str,
        data: bytes,
        metadata: Optional[Dict[str, object]] = None,
        extension: Optional[str] = None,
    ) -> ArtifactRef:
        """保存二进制制品；元数据最后发布，未增加 fsync 持久性承诺。"""
        artifact_id = build_artifact_id(kind.value)
        suffix = extension or self._guess_extension(mime_type)
        directory = self._base_dir / kind.value
        file_path = directory / f"{artifact_id}{suffix}"
        meta_path = directory / f"{artifact_id}.meta.json"
        tmp_file_path = directory / f".{artifact_id}{suffix}.tmp"
        tmp_meta_path = directory / f".{artifact_id}.meta.json.tmp"
        ref = ArtifactRef(
            artifact_id=artifact_id,
            kind=kind,
            mime_type=mime_type,
            size_bytes=len(data),
            uri=f"{self._public_base_url}/artifacts/{artifact_id}",
            metadata=dict(metadata or {}),
        )
        with self._state.mutation_lock:
            directory.mkdir(parents=True, exist_ok=True)
            if any(path.exists() for path in (file_path, meta_path, tmp_file_path, tmp_meta_path)):
                raise GatewayError("制品标识碰撞，请重试保存。", error_code="artifact_id_collision", http_status=409,
                                   details={"retryable": True})
            data_published = False
            try:
                tmp_file_path.write_bytes(data)
                tmp_meta_path.write_text(ref.model_dump_json(indent=2), encoding="utf-8")
                os.replace(tmp_file_path, file_path)
                data_published = True
                os.replace(tmp_meta_path, meta_path)
            except Exception:
                for path in (tmp_file_path, tmp_meta_path) + ((file_path,) if data_published else ()):
                    try:
                        path.unlink(missing_ok=True)
                    except OSError:
                        pass
                raise
            self._paths[artifact_id] = file_path
            self._remember_owned_locked(artifact_id)
            for hold in getattr(self._state.local, "hold_scopes", ()):
                hold._add_locked(artifact_id, local=True)
            for scope in getattr(self._state.local, "save_scopes", ()):
                scope.append(artifact_id)
                self._state.pins[artifact_id] = self._state.pins.get(artifact_id, 0) + 1
        return ref

    def get_ref(self, artifact_id: str) -> ArtifactRef:
        """读取当前侧车文件，避免返回外部删除/修改前的缓存元数据。"""
        with self._state.mutation_lock:
            meta_path = self._find_meta_path(artifact_id)
            if meta_path is None:
                raise ArtifactNotFoundError(artifact_id)
            return self._read_entry(meta_path, None).ref.model_copy(deep=True)

    def get_file_path(self, artifact_id: str) -> Path:
        """解析当前制品文件路径，缓存命中也验证文件仍存在。"""
        with self._state.mutation_lock:
            cached = self._paths.get(artifact_id)
            if cached is not None and cached.is_file():
                return cached
            self._paths.pop(artifact_id, None)
            artifact_path = self._find_file_path(artifact_id)
            if artifact_path is None:
                raise ArtifactNotFoundError(artifact_id)
            self._paths[artifact_id] = artifact_path
            return artifact_path

    def _read_entry(self, meta_path: Path, file_path: Optional[Path]) -> _ArtifactEntry:
        before = _identity(meta_path)
        with self._cache_lock:
            cached = self._metadata_cache.get(meta_path)
            if cached is not None and cached[0] == before:
                ref = cached[1]
                self._metadata_cache.move_to_end(meta_path)
            else:
                ref = None
        if ref is None:
            # 打开、读取、解析、stat 都不持有缓存锁或扫描级文件变更锁。
            ref = self._parse_metadata(meta_path)
            if _identity(meta_path) != before or meta_path.name != f"{ref.artifact_id}.meta.json":
                raise GatewayError("制品元数据已变化或标识不匹配。", error_code="artifact_changed", http_status=409)
            with self._cache_lock:
                self._drop_cached_metadata(meta_path)
                if before[2] <= _METADATA_CACHE_MAX_BYTES:
                    self._metadata_cache[meta_path] = (before, ref)
                    self._metadata_cache_bytes += before[2]
                    while (len(self._metadata_cache) > _METADATA_CACHE_LIMIT
                           or self._metadata_cache_bytes > _METADATA_CACHE_MAX_BYTES):
                        _, removed = self._metadata_cache.popitem(last=False)
                        self._metadata_cache_bytes -= removed[0][2]
        try:
            file_identity = _identity(file_path) if file_path is not None else None
        except FileNotFoundError:
            file_identity = None
        return _ArtifactEntry(ref, meta_path, before, file_path, file_identity)

    def _drop_cached_metadata(self, meta_path: Path) -> None:
        with self._cache_lock:
            previous = self._metadata_cache.pop(meta_path, None)
            if previous is not None:
                self._metadata_cache_bytes -= previous[0][2]

    def _parse_metadata(self, meta_path: Path) -> ArtifactRef:
        return ArtifactRef.model_validate_json(meta_path.read_text(encoding="utf-8"))

    def _scan_entries(self) -> Tuple[_ArtifactEntry, ...]:
        """仅遍历一次目录；同时捕获侧车及同目录数据路径，保留外部修改可见性。"""
        entries: List[_ArtifactEntry] = []
        seen = set()

        def scan_error(error: OSError) -> None:
            raise error

        for directory, dirs, files in os.walk(self._base_dir, onerror=scan_error):
            dirs.sort()
            files.sort()
            parent = Path(directory)
            payloads: Dict[str, Path] = {}
            for name in files:
                match = _ARTIFACT_FILENAME.match(name)
                if match is not None and not name.endswith(".meta.json"):
                    artifact_id = match.group(1)
                    if artifact_id in payloads:
                        raise GatewayError("同一制品存在多个数据文件。", error_code="artifact_inventory_invalid", http_status=409)
                    payloads[artifact_id] = parent / name
            for name in files:
                if not name.endswith(".meta.json"):
                    continue
                artifact_id = name[:-len(".meta.json")]
                file_path = payloads.get(artifact_id)
                if artifact_id in seen:
                    raise GatewayError("存在重复的制品标识。", error_code="artifact_inventory_invalid", http_status=409)
                seen.add(artifact_id)
                entries.append(self._read_entry(parent / name, file_path))
                if file_path is not None:
                    with self._state.mutation_lock:
                        self._paths[artifact_id] = file_path
        entries.sort(key=lambda item: (item.ref.timestamp, str(item.meta_path)))
        return tuple(entries)

    def list_refs(self, *, kind: Optional[ArtifactKind] = None) -> Tuple[ArtifactRef, ...]:
        """列出当前全部制品引用。"""
        return tuple(entry.ref.model_copy(deep=True) for entry in self._scan_entries()
                     if kind is None or entry.ref.kind == kind)

    def build_summary(self, refs: Optional[Iterable[ArtifactRef]] = None) -> ArtifactStorageSummary:
        """构造给定视图的存储摘要；省略 refs 时执行一次权威扫描。"""
        items = list(refs if refs is not None else (entry.ref for entry in self._scan_entries()))
        by_kind: Dict[str, int] = {}
        total_size_bytes = 0
        for ref in items:
            by_kind[ref.kind.value] = by_kind.get(ref.kind.value, 0) + 1
            total_size_bytes += int(ref.size_bytes or 0)
        return ArtifactStorageSummary(
            artifact_count=len(items), total_size_bytes=total_size_bytes, by_kind=by_kind,
            oldest_artifact_id=items[0].artifact_id if items else None,
            newest_artifact_id=items[-1].artifact_id if items else None,
        )

    def delete_artifact(self, artifact_id: str, *, force: bool = False) -> ArtifactRef:
        """显式删除；force 跳过引用/临时保护，普通删除在实际删除前重新查引用。"""
        with self._state.mutation_lock:
            meta_path = self._find_meta_path(artifact_id)
            if meta_path is None:
                raise ArtifactNotFoundError(artifact_id)
            entry = self._read_entry(meta_path, self.get_file_path(artifact_id))
        return self._delete_entry(entry, force=force).model_copy(deep=True)

    def _delete_entry(
        self, entry: _ArtifactEntry, *, force: bool = False,
        expected_epoch: Optional[int] = None, context: Optional[ArtifactCleanupContext] = None,
    ) -> ArtifactRef:
        ref = entry.ref
        file_path = entry.file_path
        if file_path is None or entry.file_identity is None:
            raise ArtifactNotFoundError(ref.artifact_id)
        # 不在文件锁下调用跨服务提供器；这仍不等于仓储引用事务。
        reference_generation = self.reference_epoch()
        if context is not None:
            context.check_current()
        if not force:
            reference_index = self._build_reference_index(include_registered=False)
            if ref.artifact_id in reference_index:
                raise GatewayError(
                    f"制品 {ref.artifact_id} 仍被引用，拒绝删除。", error_code="artifact_in_use",
                    http_status=409, jsonrpc_code=-32009,
                    details={"artifact_id": ref.artifact_id, "references": reference_index[ref.artifact_id]},
                )
        registered_permit = None
        with self.maintenance_fence():
            has_registered_sources = bool(self._state.reference_sources)
        if not force and has_registered_sources:
            registered_permit = self.prepare_registered_deletion(ref.artifact_id)
        permit = None
        if context is not None:
            try:
                permit = context.prepare_delete(ref.artifact_id) if context.prepare_delete is not None else None
            except Exception as exc:
                raise GatewayError("制品删除许可不可用。", error_code="artifact_guard_unavailable", http_status=503) from exc
            if not isinstance(permit, ArtifactDeletionPermit):
                raise GatewayError("尚未接入制品删除许可。", error_code="artifact_guard_unavailable", http_status=503)
        with self._state.mutation_lock:
            if registered_permit is not None and not self._permit_is_current(registered_permit):
                raise GatewayError("制品引用许可已变化。", error_code="artifact_guard_changed", http_status=409)
            if context is not None:
                context.check_current()
                if not self._permit_is_current(permit):
                    raise GatewayError("制品引用许可已变化。", error_code="artifact_guard_changed", http_status=409)
            if not force and reference_generation != self._state.reference_epoch:
                raise GatewayError("制品引用许可已变化。", error_code="artifact_guard_changed", http_status=409)
            if expected_epoch is not None and self._state.destructive_epoch != expected_epoch:
                raise GatewayError("扫描后出现其他删除或替换，推迟剩余清理。", error_code="artifact_snapshot_stale", http_status=409)
            if not force and ref.artifact_id in self._state.pins:
                raise GatewayError("制品尚在当前保存操作中。", error_code="artifact_in_use", http_status=409)
            if _identity(entry.meta_path) != entry.meta_identity or _identity(file_path) != entry.file_identity:
                raise GatewayError("制品在扫描后已变化，跳过删除。", error_code="artifact_changed", http_status=409)
            # stat 也可能阻塞；返回后再次确认关闭/策略栅栏，之后才承认本次删除。
            if context is not None:
                context.check_current()
                if not self._permit_is_current(permit):
                    raise GatewayError("制品引用许可已变化。", error_code="artifact_guard_changed", http_status=409)
            if not force and reference_generation != self._state.reference_epoch:
                raise GatewayError("制品引用许可已变化。", error_code="artifact_guard_changed", http_status=409)
            # 即使只完成了数据删除，也必须使其他扫描的配额判断失效。
            self._state.destructive_epoch += 1
            try:
                file_path.unlink()
                # 只在数据确实删除后写缺失凭据；权限错误不能把完好制品变成断链。
                self._remember_owned_locked(ref.artifact_id, False)
                if ref.artifact_id in self._state.pins:
                    self._state.deleted_while_held.add(ref.artifact_id)
                entry.meta_path.unlink()
            finally:
                # 部分删除也不能留下可命中的旧路径/元数据缓存。
                self._drop_cached_metadata(entry.meta_path)
                self._paths.pop(ref.artifact_id, None)
        return ref

    def get_maintenance_status(self, *, compact: bool = False) -> Dict[str, object]:
        """返回最近维护结果与未满足的保留策略；摘要是扫描时间点视图。"""
        import copy
        with self._state.mutation_lock:
            return (compact_maintenance_status(self._maintenance_status) if compact
                    else copy.deepcopy(self._maintenance_status))

    def _publish_maintenance_status(self, status, context):
        with self._state.mutation_lock:
            retired = self._maintenance_status
            self._maintenance_status = status
            if context is not None:
                context.outcome.update(status)
        # 最后一个旧大列表引用的释放也不应占用保存/状态读取的短锁。
        del retired

    @contextmanager
    def _maintenance_scope(self) -> Iterator[bool]:
        # 非阻塞准入：仓储锁持有者不得等待另一维护中持有的引用提供器。
        if getattr(self._state.local, "maintenance_active", False):
            raise GatewayError("制品维护不可递归调用。", error_code="artifact_maintenance_reentrant", http_status=409)
        admitted = self._state.cleanup_lock.acquire(blocking=False)
        if not admitted:
            yield False
            return
        self._state.local.maintenance_active = True
        try:
            yield True
        finally:
            self._state.local.maintenance_active = False
            self._state.cleanup_lock.release()

    def cleanup(
        self, policy: ArtifactRetentionPolicy, *, context: Optional[ArtifactCleanupContext] = None,
    ) -> ArtifactCleanupResult:
        """按一次扫描视图清理；仍有 O(N) 枚举/stat，未变更元数据使用有界缓存。"""
        with self._maintenance_scope() as admitted:
            with self._state.mutation_lock:
                expected_epoch = self._state.destructive_epoch
            try:
                entries = self._scan_entries()
            except Exception as exc:
                self._publish_maintenance_status(
                    {"status": "failed", "error_code": "artifact_inventory_unavailable",
                     "error_type": type(exc).__name__, "summary": None}, context,
                )
                raise
            removed: List[ArtifactRef] = []
            removed_ids = set()
            protected_ids = set()
            skipped_ids = set()
            errors: List[str] = []
            as_of = context.as_of if context is not None else utc_now()
            expire_before = as_of - timedelta(days=policy.retention_days) if policy.retention_days else None
            retained_count = len(entries)
            retained_bytes = sum(int(entry.ref.size_bytes or 0) for entry in entries)

            def attempt(entry: _ArtifactEntry) -> bool:
                nonlocal retained_count, retained_bytes, expected_epoch
                if entry.ref.artifact_id in initial_references:
                    protected_ids.add(entry.ref.artifact_id)
                    return True
                try:
                    removed.append(self._delete_entry(entry, expected_epoch=expected_epoch, context=context))
                except GatewayError as exc:
                    if exc.error_code == "artifact_in_use":
                        protected_ids.add(entry.ref.artifact_id)
                    else:
                        skipped_ids.add(entry.ref.artifact_id)
                        if len(errors) < 16:
                            errors.append(exc.error_code)
                    return exc.error_code == "artifact_in_use"
                except (OSError, ArtifactNotFoundError) as exc:
                    skipped_ids.add(entry.ref.artifact_id)
                    if len(errors) < 16:
                        errors.append(type(exc).__name__)
                    return False
                expected_epoch += 1
                removed_ids.add(entry.ref.artifact_id)
                retained_count -= 1
                retained_bytes -= int(entry.ref.size_bytes or 0)
                return True

            proceed = admitted
            initial_references = set()
            needs_cleanup = (
                (policy.max_count is not None and retained_count > policy.max_count)
                or (policy.max_total_bytes is not None and retained_bytes > policy.max_total_bytes)
                or (expire_before is not None and any(entry.ref.timestamp < expire_before for entry in entries))
            )
            if needs_cleanup and admitted:
                try:
                    if context is not None:
                        context.check_current()
                        if context.prepare_delete is None:
                            raise GatewayError("尚未接入制品删除许可。", error_code="artifact_guard_unavailable", http_status=503)
                    initial_references = set(self._build_reference_index())
                except GatewayError as exc:
                    errors.append(exc.error_code)
                    proceed = False
            for entry in entries:
                if len(removed) >= policy.cleanup_batch_size or not proceed:
                    break
                if expire_before is not None and entry.ref.timestamp < expire_before:
                    proceed = attempt(entry)
            if proceed:
                for entry in entries:
                    if len(removed) >= policy.cleanup_batch_size:
                        break
                    over_count = policy.max_count is not None and retained_count > policy.max_count
                    over_bytes = policy.max_total_bytes is not None and retained_bytes > policy.max_total_bytes
                    if not over_count and not over_bytes:
                        break
                    if (entry.ref.artifact_id in removed_ids or entry.ref.artifact_id in protected_ids
                            or entry.ref.artifact_id in skipped_ids):
                        continue
                    if not attempt(entry):
                        break
            retained = [entry.ref for entry in entries if entry.ref.artifact_id not in removed_ids]
            result = ArtifactCleanupResult(
                removed_count=len(removed), freed_bytes=sum(int(ref.size_bytes or 0) for ref in removed),
                removed_artifact_ids=[ref.artifact_id for ref in removed],
                protected_count=len(protected_ids), protected_artifact_ids=sorted(protected_ids),
                summary=self.build_summary(retained),
            )
            debt_count = max(0, retained_count - policy.max_count) if policy.max_count is not None else 0
            debt_bytes = max(0, retained_bytes - policy.max_total_bytes) if policy.max_total_bytes is not None else 0
            debt_expired = sum(ref.timestamp < expire_before for ref in retained) if expire_before else 0
            # 排序和可能很大的 ID 列表序列化留在锁外；保存/健康只共享最后赋值。
            maintenance_status = {
                "status": "failed" if errors else "deferred" if not admitted or debt_count or debt_bytes or debt_expired else "completed",
                "reason": "maintenance_busy" if not admitted else "retention_policy",
                "pending_count": debt_count, "pending_bytes": debt_bytes, "pending_expired": debt_expired,
                "errors": errors, "skipped_artifact_ids": sorted(skipped_ids),
                "summary_scope": "scan_snapshot_minus_confirmed_deletions",
                "result": result.model_dump(mode="json"),
            }
            self._publish_maintenance_status(maintenance_status, context)
            return result

    def clear_all(self, *, force: bool = False) -> ArtifactCleanupResult:
        """显式清空；非强制清理逐项重新查引用，失败向调用方报告。"""
        with self._maintenance_scope() as admitted:
            if not admitted:
                raise GatewayError("另一制品维护正在执行，请在维护结束后重试。", error_code="artifact_maintenance_busy",
                                   http_status=409, details={"retryable": True, "reason": "maintenance_busy", "operation": "clear_all"})
            entries = self._scan_entries()
            removed: List[ArtifactRef] = []
            protected: List[str] = []
            for entry in entries:
                try:
                    removed.append(self._delete_entry(entry, force=force))
                except GatewayError as exc:
                    if exc.error_code != "artifact_in_use":
                        raise
                    protected.append(entry.ref.artifact_id)
            with self._state.mutation_lock:
                self._remove_empty_dirs()
                self._base_dir.mkdir(parents=True, exist_ok=True)
            removed_ids = {ref.artifact_id for ref in removed}
            return ArtifactCleanupResult(
                removed_count=len(removed), freed_bytes=sum(int(ref.size_bytes or 0) for ref in removed),
                removed_artifact_ids=[ref.artifact_id for ref in removed], protected_count=len(protected),
                protected_artifact_ids=sorted(protected),
                summary=self.build_summary(entry.ref for entry in entries if entry.ref.artifact_id not in removed_ids),
            )

    def diagnose_reference_links(self) -> Dict[str, object]:
        """诊断引用索引中的制品断链。"""
        reference_index = self._build_reference_index()
        missing: List[Dict[str, object]] = []
        for artifact_id, references in sorted(reference_index.items()):
            try:
                self.get_ref(artifact_id)
                self.get_file_path(artifact_id)
            except Exception as exc:  # noqa: BLE001 - 每条断链独立报告，包括格式或文件错误
                missing.append({"artifact_id": artifact_id, "error": str(exc), "references": references})
        existing_ids = {ref.artifact_id for ref in self.list_refs()}
        return {"referenced_artifact_count": len(reference_index), "existing_artifact_count": len(existing_ids),
                "missing_artifact_count": len(missing), "missing_artifacts": missing,
                "unreferenced_artifact_ids": sorted(existing_ids - set(reference_index))}

    def _build_reference_index(self, *, include_registered=True) -> Dict[str, List[Dict[str, Any]]]:
        with self.maintenance_fence():
            providers = ([self._reference_provider] if self._reference_provider is not None else [])
            if include_registered:
                providers.extend(source[0] for source in self._state.reference_sources.values())
        if not providers:
            return {}
        if getattr(self._state.local, "reference_check_active", False):
            raise GatewayError("制品引用检查不可递归调用。", error_code="artifact_reference_unavailable", http_status=503)
        self._state.local.reference_check_active = True
        try:
            index: Dict[str, List[Dict[str, Any]]] = {}
            for provider in providers:
                payload = provider()
                if isinstance(payload, dict):
                    for artifact_id, references in payload.items():
                        resolved_id = str(artifact_id or "").strip()
                        if resolved_id:
                            index.setdefault(resolved_id, []).extend(
                                [dict(item) for item in references if isinstance(item, dict)]
                                if isinstance(references, (list, tuple)) else [{"source": str(references)}])
                elif isinstance(payload, Iterable) and not isinstance(payload, (str, bytes)):
                    for item in payload:
                        resolved_id = str(item or "").strip()
                        if resolved_id:
                            index.setdefault(resolved_id, []).append({"source": "unknown"})
                else:
                    raise TypeError("引用提供器必须返回索引或制品标识集合")
            return index
        except Exception as exc:
            raise GatewayError("制品引用状态不可用，保留现有制品。", error_code="artifact_reference_unavailable",
                               http_status=503) from exc
        finally:
            self._state.local.reference_check_active = False

    def _remove_empty_dirs(self) -> None:
        for child in sorted(self._base_dir.glob("**/*"), key=lambda item: len(item.parts), reverse=True):
            if child != self._base_dir and child.is_dir():
                try:
                    child.rmdir()
                except OSError:
                    pass

    def _find_meta_path(self, artifact_id: str) -> Optional[Path]:
        cached = self._paths.get(artifact_id)
        if cached is not None:
            meta_path = cached.parent / f"{artifact_id}.meta.json"
            if meta_path.is_file():
                return meta_path
        # 保存布局可由既有 ID 契约定位；旧布局/移动后的侧车仍走原查找。
        if ARTIFACT_ID_PATTERN.fullmatch(artifact_id):
            try:
                kind = ArtifactKind(artifact_id[4:].rsplit("_", 2)[0])
            except ValueError:
                pass
            else:
                meta_path = self._base_dir / kind.value / f"{artifact_id}.meta.json"
                try:
                    if meta_path.is_file():
                        return meta_path
                except OSError:
                    pass
        matches = list(self._base_dir.glob(f"**/{artifact_id}.meta.json"))
        return matches[0] if matches else None

    def _find_file_path(self, artifact_id: str) -> Optional[Path]:
        for candidate in self._base_dir.glob(f"**/{artifact_id}*"):
            if not candidate.name.endswith(".meta.json") and candidate.is_file():
                return candidate
        return None

    def _guess_extension(self, mime_type: str) -> str:
        return {"image/jpeg": ".jpg", "image/png": ".png", "application/json": ".json",
                "audio/wav": ".wav"}.get(mime_type.lower().strip(), ".bin")
