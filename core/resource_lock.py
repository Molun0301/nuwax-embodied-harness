from __future__ import annotations

from contracts.compat import StrEnum
import threading

from pydantic import Field

from contracts.base import ContractModel, MetadataDict, utc_now
from typing import Dict, List, Optional, Tuple, Union


class RuntimeResource(StrEnum):
    """首版运行时资源。"""

    BASE_MOTION = "base_motion"
    NAVIGATION = "navigation"
    AUDIO_OUT = "audio_out"
    CAMERA_OBSERVATION = "camera_observation"


class ResourceLease(ContractModel):
    """资源占用记录。"""

    resource: str = Field(description="资源名称。")
    task_id: str = Field(description="占用任务。")
    acquired_at: str = Field(default_factory=lambda: utc_now().isoformat(), description="占用时间。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class ResourceConflict(ContractModel):
    """资源冲突信息。"""

    resource: str = Field(description="冲突资源。")
    holder_task_id: str = Field(description="当前持有者。")
    requester_task_id: str = Field(description="请求者。")
    message: str = Field(description="冲突说明。")


class ResourceConflictError(RuntimeError):
    """资源冲突异常。"""

    def __init__(self, conflicts: List[ResourceConflict]) -> None:
        self.conflicts = conflicts
        summary = ", ".join(f"{item.resource}->{item.holder_task_id}" for item in conflicts)
        super().__init__(f"资源冲突: {summary}")


class ResourceLockManager:
    """运行时资源锁。"""

    def __init__(self) -> None:
        self._leases: Dict[str, ResourceLease] = {}
        self._lock = threading.RLock()

    def acquire(
        self,
        task_id: str,
        resources: Union[List[str], Tuple[str, ...]],
        *,
        metadata: Optional[MetadataDict] = None,
    ) -> Tuple[ResourceLease, ...]:
        """申请一组资源。"""

        normalized = [self._normalize_resource(resource) for resource in resources]
        with self._lock:
            conflicts: List[ResourceConflict] = []
            for resource in normalized:
                lease = self._leases.get(resource)
                if lease is not None and lease.task_id != task_id:
                    conflicts.append(
                        ResourceConflict(
                            resource=resource,
                            holder_task_id=lease.task_id,
                            requester_task_id=task_id,
                            message=f"资源 {resource} 当前由任务 {lease.task_id} 持有。",
                        )
                    )
            if conflicts:
                raise ResourceConflictError(conflicts)

            leases: List[ResourceLease] = []
            for resource in normalized:
                existing = self._leases.get(resource)
                if existing is None:
                    existing = ResourceLease(resource=resource, task_id=task_id, metadata=dict(metadata or {}))
                    self._leases[resource] = existing
                leases.append(existing)
            return tuple(leases)

    def release(self, task_id: str, resources: Union[List[str], Tuple[str, ...], None] = None) -> None:
        """释放某任务持有的资源。"""

        with self._lock:
            if resources is None:
                resources = [resource for resource, lease in self._leases.items() if lease.task_id == task_id]
            for resource in resources:
                normalized = self._normalize_resource(resource)
                lease = self._leases.get(normalized)
                if lease is not None and lease.task_id == task_id:
                    self._leases.pop(normalized, None)

    def force_release(self, resources: Union[List[str], Tuple[str, ...]]) -> None:
        """无条件释放指定资源。"""

        with self._lock:
            for resource in resources:
                self._leases.pop(self._normalize_resource(resource), None)

    def release_all(self) -> None:
        """释放全部资源。"""

        with self._lock:
            self._leases.clear()

    def snapshot(self) -> Tuple[ResourceLease, ...]:
        """返回当前全部资源占用快照。"""

        with self._lock:
            return tuple(self._leases[resource] for resource in sorted(self._leases))

    def held_resources(self, task_id: str) -> Tuple[str, ...]:
        """列出某任务当前持有的资源。"""

        with self._lock:
            return tuple(sorted(resource for resource, lease in self._leases.items() if lease.task_id == task_id))

    def is_locked(self, resource: str) -> bool:
        """判断资源是否被占用。"""

        with self._lock:
            return self._normalize_resource(resource) in self._leases

    def _normalize_resource(self, resource: Union[str, RuntimeResource]) -> str:
        if isinstance(resource, RuntimeResource):
            return resource.value
        return str(resource)
