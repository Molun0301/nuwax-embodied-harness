"""制品维护的独立后台拥有方；尚未接入运行时自动保存链路。

至多一个活动请求和一个最新待处理请求。合并不累积删除额度，空闲时不主动
推进过期时间。不使用共享 executor；删除必须显式注入具备最终校验的保护器。
"""
from __future__ import annotations

import copy
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Callable, Dict, Optional

from contracts.artifacts import ArtifactRetentionPolicy
from contracts.base import utc_now
from services.artifact_store import (
    ArtifactCleanupContext,
    ArtifactDeletionPermit,
    LocalArtifactStore,
    compact_maintenance_status,
)

DeletionGuard = Callable[[str], Optional[ArtifactDeletionPermit]]


@dataclass(frozen=True)
class _Request:
    sequence: int
    policy_revision: int
    policy: ArtifactRetentionPolicy
    triggered_at: datetime
    artifact_id: Optional[str]


class ArtifactMaintenanceWorker:
    """显式启动、显式关闭的单线程维护基础设施。

    deletion_guard 是未来引用桥的入口：仅在文件锁外准备不可变引用代际许可。
    默认 None 会留下明确的 guard-unavailable 欠账，不能删除。
    本类不默认宣称任何仓储引用安全，不持有图像字节，也不建立持久触发账本。
    """

    def __init__(
        self, store: LocalArtifactStore, *, policy: ArtifactRetentionPolicy,
        deletion_guard: Optional[DeletionGuard] = None,
    ) -> None:
        self._store = store
        self._policy = policy.model_copy(deep=True)
        self._guard = deletion_guard
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._owner_token = object()
        self._policy_revision = 0
        self._sequence = 0
        self._pending: Optional[_Request] = None
        self._active: Optional[_Request] = None
        self._thread: Optional[threading.Thread] = None
        self._started = False
        self._starting = False
        self._closing = False
        self._phase = "not_started"
        self._last_outcome: Dict[str, object] = {}
        self._counts = {key: 0 for key in (
            "requested", "started", "completed", "failed", "cancelled", "coalesced", "rejected",
        )}

    def start(self) -> bool:
        """显式申请根目录拥有权；关闭无需等待正在申请拥有权的线程。"""
        with self._condition:
            if self._closing or self._starting:
                return False
            if self._started:
                return self._thread is not None and self._thread.is_alive()
            self._starting = True
            self._phase = "starting"
        claimed = False
        started = False
        try:
            claimed = self._store.claim_maintenance_owner(self._owner_token)
            with self._condition:
                if self._closing:
                    self._phase = "closed"
                    return False
                if not claimed:
                    self._phase = "owner_busy"
                    return False
                thread = threading.Thread(target=self._run, name="nuwax_artifact_maintenance", daemon=True)
                self._thread = thread
                self._started = True
                self._phase = "idle"
                try:
                    thread.start()
                except BaseException:
                    self._started = False
                    self._thread = None
                    self._phase = "start_failed"
                    raise
                started = True
                return True
        finally:
            if claimed and not started:
                self._store.release_maintenance_owner(self._owner_token)
            with self._condition:
                self._starting = False
                self._condition.notify_all()

    def request(self, *, triggered_at: Optional[datetime] = None, artifact_id: Optional[str] = None) -> Optional[int]:
        """提交一个冻结策略/时间的触发；不执行存储、引用查询、扫描或等待工作者。"""
        when = triggered_at or utc_now()
        if when.tzinfo is None or when.utcoffset() is None:
            raise ValueError("维护触发时间必须包含时区")
        with self._condition:
            if (not self._started or self._closing or self._thread is None
                    or not self._thread.is_alive()):
                self._counts["rejected"] += 1
                return None
            self._sequence += 1
            self._counts["requested"] += 1
            if self._pending is not None:
                self._counts["coalesced"] += 1
            self._pending = _Request(self._sequence, self._policy_revision,
                                     self._policy.model_copy(deep=True), when, artifact_id)
            self._condition.notify()
            return self._sequence

    def update_policy(self, policy: ArtifactRetentionPolicy) -> None:
        """使旧请求失效，但不产生新维护触发或空闲清理。"""
        replacement = policy.model_copy(deep=True)
        # 与最终候选删除共享短栅栏：更新返回后不能再按旧策略删除。
        with self._store.maintenance_fence(), self._condition:
            self._policy = replacement
            self._policy_revision += 1
            if self._pending is not None:
                self._pending = None
                self._counts["cancelled"] += 1
            self._condition.notify_all()

    def _is_current(self, request: _Request) -> bool:
        # 此函数可能在 store 的短文件变更区内运行；绝不执行文件或模型 I/O。
        if self._stop.is_set():
            return False
        if not self._store.is_maintenance_owner(self._owner_token):
            return False
        with self._condition:
            return not self._closing and request.policy_revision == self._policy_revision

    def _run(self) -> None:
        try:
            while True:
                with self._condition:
                    while self._pending is None and not self._closing:
                        self._condition.wait()
                    if self._closing:
                        return
                    request = self._pending
                    self._pending = None
                    self._active = request
                    self._phase = "maintenance"
                    self._counts["started"] += 1
                context = ArtifactCleanupContext(
                    as_of=request.triggered_at,
                    is_current=lambda request=request: self._is_current(request),
                    prepare_delete=self._guard,
                )
                outcome: Dict[str, object] = {}
                try:
                    if self._is_current(request):
                        self._store.cleanup(request.policy, context=context)
                        outcome = dict(context.outcome)
                    else:
                        outcome = {"status": "cancelled", "reason": "request_invalidated"}
                except Exception as error:  # noqa: BLE001 - 隔离一次维护故障，不把线程失败变成保存失败
                    outcome = {**context.outcome, "status": "failed", "error_type": type(error).__name__}
                still_current = self._is_current(request)
                with self._condition:
                    retired_outcome = self._last_outcome
                    if not still_current or self._closing or request.policy_revision != self._policy_revision:
                        self._counts["cancelled"] += 1
                        outcome = {**outcome, "status": "cancelled", "reason": "request_invalidated"}
                    elif outcome.get("status") == "failed":
                        self._counts["failed"] += 1
                    else:
                        self._counts["completed"] += 1
                    self._last_outcome = {**outcome, "request_sequence": request.sequence,
                                          "triggered_at": request.triggered_at.isoformat(),
                                          "policy_revision": request.policy_revision,
                                          "trigger_artifact_id": request.artifact_id}
                    self._active = None
                    self._phase = "stopping" if self._closing else "idle"
                    self._condition.notify_all()
                del retired_outcome
        except BaseException as error:  # noqa: BLE001 - 线程终止必须发布失败、释放拥有权及唤醒等待者
            with self._condition:
                self._counts["failed"] += 1
                self._last_outcome = {"status": "failed", "error_type": type(error).__name__,
                                      "reason": "worker_terminated"}
        finally:
            # 不合作回调尚未返回时不会走到这里，拥有权及线程引用必须仍保留。
            self._store.release_maintenance_owner(self._owner_token)
            with self._condition:
                if self._pending is not None:
                    self._pending = None
                    self._counts["cancelled"] += 1
                self._active = None
                self._phase = "closed" if self._closing else "failed"
                self._condition.notify_all()

    def status(self, *, compact: bool = False) -> Dict[str, object]:
        with self._condition:
            return {
                "phase": self._phase, "started": self._started, "starting": self._starting, "closing": self._closing,
                "thread_alive": self._thread is not None and self._thread.is_alive(),
                "pending_requests": int(self._pending is not None),
                "active_requests": int(self._active is not None),
                "pending_sequence": self._pending.sequence if self._pending is not None else None,
                "active_sequence": self._active.sequence if self._active is not None else None,
                "policy_revision": self._policy_revision, "guard_configured": self._guard is not None,
                "counts": dict(self._counts), "last_outcome": (compact_maintenance_status(self._last_outcome)
                                                             if compact else copy.deepcopy(self._last_outcome)),
            }

    def wait_idle(self, timeout: float = 1.0) -> bool:
        """管理/测试辅助；保存路径不可调用，超时不会取消线程。"""
        deadline = time.monotonic() + max(0.0, timeout)
        with self._condition:
            while self._active is not None or self._pending is not None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._condition.wait(remaining)
            return True

    def close(self, *, timeout: float = 1.0) -> Dict[str, object]:
        """有界关闭；回调阻塞时保留拥有权，晚返回不得继续删除。"""
        self._stop.set()
        with self._condition:
            self._closing = True
            if self._pending is not None:
                self._pending = None
                self._counts["cancelled"] += 1
            self._phase = "stopping" if self._thread is not None and self._thread.is_alive() else "closed"
            self._condition.notify_all()
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(0.0, timeout))
        result = self.status()
        result["stopped"] = not result["thread_alive"]
        return result
