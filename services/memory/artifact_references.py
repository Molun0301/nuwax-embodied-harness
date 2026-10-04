"""合作记忆仓储与本地制品的引用发布桥；不参与模型、向量或查询排名。

源表仍为权威。只缓存当前引用代际下的有界快照，不建立持久化触发账本。
所有文件检查在源 SQL 之前，所有释放在真正外层事务完成之后。
"""
from __future__ import annotations

import copy
import json
import threading
from collections import deque
from contextlib import contextmanager

from core.errors import GatewayError
from services.artifact_store import ArtifactHold

_SOURCES = {
    "memory": ("semantic_memories", "memory_id"),
    "instance": ("semantic_instances", "instance_id"),
    "event": ("observation_events", "event_id"),
    "scene": ("scene_spaces", "scene_space_id"),
}
_CACHE_REFERENCE_LIMIT = 16384
_CACHE_BYTES_LIMIT = 8 * 1024 * 1024


def artifact_ids(kind, record):
    """源引用投影与现有记忆接口一致；不推断外部 ID 的文件所有权。"""
    if kind not in _SOURCES:
        return ()
    payload = record if isinstance(record, dict) else record.model_dump(mode="json")
    values = payload.get("artifact_ids", ()) if kind != "scene" else (
        value for sample in payload.get("observation_samples", ()) for value in sample.get("artifact_ids", ()))
    return tuple(dict.fromkeys(str(value).strip() for value in values if str(value).strip()))


class MemoryArtifactReferences:
    """可选仓储适配器；独立仓储不附加时保留原有行为。

    新引用只验证已证明属于此根目录的 ID；旧断链和未知/外部 ID 保留并报告。
    保护针对本进程已登记的合作源操作。任意绕过仓储的 SQL 或外部文件修改
    不因此获得跨进程原子性；下一次权威扫描仍能观察外部文件变化。
    """

    def __init__(self, repository, store) -> None:
        self.repository = repository
        self.storage = repository.storage
        self.store = store
        self._token = object()
        self._lock = threading.RLock()
        self._cache = None
        self._cache_key = None
        self._cache_reference_count = 0
        self._pending = {}
        self._closed = False
        self._errors = deque(maxlen=16)
        self._counts = {"admitted": 0, "completed": 0, "completed_without_commit_confirmation": 0,
                        "unresolved_references": 0, "legacy_missing_references": 0,
                        "completion_failures": 0, "snapshot_builds": 0}
        repository.attach_artifact_references(self)
        store.register_reference_source(self._token, self.snapshot, self.references_for)

    def _check_open(self):
        with self._lock:
            if self._closed:
                raise GatewayError("制品引用仓储已关闭。", error_code="artifact_reference_unavailable", http_status=503)

    @contextmanager
    def mutation(self, kind, record, context, *, payload_json=None):
        """先取得新增候选保护，再进入原协调器；CAS/持久队列顺序保持不变。"""
        if kind not in _SOURCES:
            with context as transaction:
                yield transaction
            return
        self._check_open()
        acquire = self.store.hold_proven_artifacts if self.storage.in_transaction else self.store.acquire_artifacts
        # 仓储即将写入的序列化快照才是保护对象；不重新借用可变调用方记录。
        payload = json.loads(payload_json) if payload_json is not None else record
        hold = acquire(artifact_ids(kind, payload), owner="memory.source." + kind)
        transferred = False
        try:
            with context as transaction:
                table, column = _SOURCES[kind]
                row = transaction.execute(f"SELECT payload_json FROM {table} WHERE {column}=?",
                                          (getattr(record, column),)).fetchone()
                existing = artifact_ids(kind, json.loads(row[0])) if row else ()
                hold.validate_introduced(existing)
                # 旧引用在未提交的删除/替换视图里可能已不可见，回滚前仍须保护。
                hold.add_existing_references(existing)
                transferred = self._bind(transaction, hold)
                yield transaction
        finally:
            if not transferred:
                hold.release()

    @contextmanager
    def deletion(self, kind, ids, context):
        if kind not in _SOURCES:
            with context as transaction:
                yield transaction
            return
        self._check_open()
        hold = ArtifactHold(self.store, "memory.delete." + kind)
        transferred = False
        try:
            with context as transaction:
                table, column = _SOURCES[kind]
                for point_id in ids:
                    row = transaction.execute(f"SELECT payload_json FROM {table} WHERE {column}=?", (point_id,)).fetchone()
                    if row:
                        hold.add_existing_references(artifact_ids(kind, json.loads(row[0])))
                transferred = self._bind(transaction, hold)
                yield transaction
        finally:
            if not transferred:
                hold.release()

    def _bind(self, transaction, hold):
        if not hold.artifact_ids:
            return False
        with self._lock:
            self._check_open()
            # 与 close 串行准入，不能在登记回调和记录未决拥有方之间移除来源。
            transaction.after_completion(lambda committed: self._complete(hold, committed))
            self._pending[hold._token] = (hold, None, transaction.completion)
            self._counts["admitted"] += 1
            self._counts["unresolved_references"] += len(hold.unresolved_ids)
            self._counts["legacy_missing_references"] += len(hold.missing_local_ids)
        return True

    def _complete(self, hold, committed):
        with self._lock:
            pending = self._pending.get(hold._token)
            if pending is None:
                return
            self._pending[hold._token] = (hold, bool(committed), pending[2])
            if not pending[2].settled:
                # ROLLBACK 本身失败时仍可能晚提交。保留原句柄，等待拥有方
                # 确认连接事务结束；False 通知绝不等同已回滚。
                return
        try:
            hold.release(reference_changed=True)
        except Exception as error:  # noqa: BLE001 - 通知失败须保留已提交结果及可重试保护
            # 源提交结果不能因维护通知失败被改写；保留保护并提供重试路径。
            with self._lock:
                self._counts["completion_failures"] += 1
                self._errors.append(type(error).__name__)
            return
        with self._lock:
            if self._pending.pop(hold._token, None) is not None:
                self._counts["completed" if committed else "completed_without_commit_confirmation"] += 1
            self._cache = None
            self._cache_key = None
            self._cache_reference_count = 0

    def retry_completed_holds(self):
        """重试已知完成事务的短释放；仍在 SQL 中的拥有方绝不按超时丢弃。"""
        with self._lock:
            completed = tuple(value for value in self._pending.values() if value[1] is not None)
        for hold, committed, _completion in completed:
            self._complete(hold, committed)

    def _read_index(self):
        # 同一源快照覆盖所有记忆库和四类来源。只在 SQL 范围内复制原始行；
        # JSON 解析及索引构造在释放仓储锁后执行。
        rows = []
        with self.storage.read_snapshot() as transaction:
            for kind, (table, column) in _SOURCES.items():
                rows.extend((kind, row[0], row[1], row[2]) for row in transaction.execute(
                    f"SELECT {column},library_name,payload_json FROM {table}").fetchall())
        index = {}
        total = 0
        byte_count = 0
        for kind, point_id, library, raw in rows:
            for artifact_id in artifact_ids(kind, json.loads(raw)):
                index.setdefault(artifact_id, []).append({"source": "memory." + kind, "record_kind": kind,
                                                         "record_id": point_id, "library_name": library})
                total += 1
                byte_count += len(artifact_id.encode("utf-8")) + len(str(point_id).encode("utf-8")) + len(str(library).encode("utf-8")) + 128
        return index, total, byte_count

    def _index(self):
        self._check_open()
        self.retry_completed_holds()
        generation = self.store.reference_epoch()
        # 不缓存当前线程未提交视图，避免污染另一个线程的权威读。
        cache_allowed = not self.storage.in_transaction
        with self._lock:
            if cache_allowed and self._cache_key == generation and self._cache is not None:
                return self._cache
        index, total, byte_count = self._read_index()
        current = self.store.reference_epoch()
        with self._lock:
            self._counts["snapshot_builds"] += 1
            if (cache_allowed and current == generation and total <= _CACHE_REFERENCE_LIMIT
                    and byte_count <= _CACHE_BYTES_LIMIT):
                self._cache = index
                self._cache_key = generation
                self._cache_reference_count = total
        return index

    def snapshot(self):
        return copy.deepcopy(self._index())

    def references_for(self, artifact_id):
        return copy.deepcopy(self._index().get(artifact_id, ()))

    def status(self):
        with self._lock:
            return {"closed": self._closed, "pending_transactions": sum(value[1] is None for value in self._pending.values()),
                    "pending_completions": sum(value[1] is not None and value[2].settled for value in self._pending.values()),
                    "unresolved_transactions": sum(value[1] is not None and not value[2].settled for value in self._pending.values()),
                    "cache_reference_count": self._cache_reference_count, "cache_reference_limit": _CACHE_REFERENCE_LIMIT,
                    "counts": dict(self._counts), "errors": list(self._errors)}

    def close(self):
        """维护拥有方停止后调用；未决源事务保留桥和保护，允许稍后再次关闭。"""
        self.retry_completed_holds()
        with self._lock:
            if self._pending:
                return False
            self._closed = True
            self._cache = None
            self._cache_key = None
        self.store.unregister_reference_source(self._token)
        return True
