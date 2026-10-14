from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from contracts.compat import StrEnum
import threading
from typing import Dict, Optional, Tuple, Any

from contracts.base import MetadataDict, utc_now


class StateNamespace(StrEnum):
    """状态存储命名空间。"""

    ROBOT_STATE = "robot_state"
    LOCALIZATION = "localization"
    OBSERVATION = "observation"
    PERCEPTION = "perception"
    NAVIGATION = "navigation"
    MAP = "map"
    MEMORY = "memory"
    SAFETY = "safety"
    SYSTEM = "system"


@dataclass(frozen=True)
class StateEntry:
    """状态存储条目。"""

    namespace: StateNamespace
    key: str
    source: str
    updated_at: datetime
    value: Any
    metadata: MetadataDict = field(default_factory=dict)


class StateStore:
    """线程安全的最新状态缓存。"""

    def __init__(self) -> None:
        self._store: Dict[StateNamespace, Dict[str, StateEntry]] = {namespace: {} for namespace in StateNamespace}
        # per-namespace 锁，__init__ 时预创建，避免运行时竞态
        self._locks: Dict[StateNamespace, threading.RLock] = {ns: threading.RLock() for ns in StateNamespace}
        # 每个 namespace 最新条目缓存，write 时更新，read_latest 直接返回
        self._latest_by_namespace: Dict[StateNamespace, Optional[StateEntry]] = {ns: None for ns in StateNamespace}

    def write(
        self,
        namespace: StateNamespace,
        key: str,
        value: Any,
        *,
        source: str,
        metadata: Optional[MetadataDict] = None,
        updated_at: Optional[datetime] = None,
    ) -> StateEntry:
        """写入状态条目。"""

        entry = StateEntry(
            namespace=namespace,
            key=key,
            source=source,
            updated_at=updated_at or utc_now(),
            value=value,
            metadata=dict(metadata or {}),
        )
        with self._locks[namespace]:
            # copy-on-write：替换整个 namespace dict 引用，snapshot 可无锁读取
            self._store[namespace] = {**self._store[namespace], key: entry}
            # 更新最新条目缓存
            latest = self._latest_by_namespace[namespace]
            if latest is None or entry.updated_at >= latest.updated_at:
                self._latest_by_namespace[namespace] = entry
            elif latest.key == key:
                # Replacing the cached entry with an older timestamp invalidates
                # that entry. Scan only this exceptional path; prefer the new
                # write on an equal timestamp, as on the normal path above.
                self._latest_by_namespace[namespace] = max(
                    self._store[namespace].values(),
                    key=lambda candidate: (candidate.updated_at, candidate is entry),
                )
        return entry

    def discard_if_value(self, namespace: StateNamespace, key: str, expected_value: Any) -> bool:
        """仅撤销已被后继拥有方取代的原对象；不递归触发 write 扩展回调。"""
        with self._locks[namespace]:
            current = self._store[namespace].get(key)
            if current is None or current.value is not expected_value:
                return False
            remaining = dict(self._store[namespace])
            del remaining[key]
            self._store[namespace] = remaining
            if self._latest_by_namespace[namespace] is current:
                self._latest_by_namespace[namespace] = max(
                    remaining.values(), key=lambda entry: entry.updated_at, default=None)
            return True

    def write_from_provider(
        self,
        provider_name: str,
        namespace: StateNamespace,
        key: str,
        value: Any,
        *,
        metadata: Optional[MetadataDict] = None,
    ) -> StateEntry:
        """按提供器路径写入状态。"""

        return self.write(namespace, key, value, source=provider_name, metadata=metadata)

    def read(self, namespace: StateNamespace, key: str) -> Optional[StateEntry]:
        """读取单条状态。"""

        with self._locks[namespace]:
            return self._store[namespace].get(key)

    def read_latest(self, namespace: StateNamespace) -> Optional[StateEntry]:
        """读取某命名空间最新条目，直接返回缓存值。"""

        return self._latest_by_namespace[namespace]

    def snapshot(self, namespace: StateNamespace) -> Tuple[StateEntry, ...]:
        """读取某命名空间完整快照（copy-on-write，读取引用即可）。"""

        # write 时已替换整个 dict 引用，此处拿到的是不可变快照
        ns_dict = self._store[namespace]
        entries = sorted(ns_dict.values(), key=lambda item: item.updated_at)
        return tuple(entries)
