from __future__ import annotations

import logging
import threading
from typing import Dict, Optional, Tuple

from contracts.artifacts import (
    ArtifactCleanupResult,
    ArtifactKind,
    ArtifactRef,
    ArtifactRetentionPolicy,
    ArtifactStorageSummary,
)
from contracts.image import ImageEncoding, ImageFrame
from core.errors import GatewayError
from services.artifact_maintenance import ArtifactMaintenanceWorker
from services.artifact_store import LocalArtifactStore


class ArtifactService:
    """制品服务。"""

    def __init__(
        self,
        store: LocalArtifactStore,
        *,
        retention_policy: ArtifactRetentionPolicy,
    ) -> None:
        self._store = store
        self._retention_policy = retention_policy.model_copy(deep=True)
        self._lifecycle_lock = threading.RLock()
        self._maintenance_worker = None
        self._async_maintenance = False
        self._closing = False
        self._active_saves = 0
        self._accepted_saves = 0
        self._failed_saves = 0
        self._unserviced_saves = 0
        self._last_summary = None

    def start_maintenance(self) -> bool:
        """引用桥装配完成后显式启动；启动失败也不得回退为捕获线程全量清理。"""
        with self._lifecycle_lock:
            if self._closing:
                return False
            self._async_maintenance = True
            if self._maintenance_worker is None:
                self._maintenance_worker = ArtifactMaintenanceWorker(
                    self._store, policy=self._retention_policy,
                    deletion_guard=self._store.prepare_registered_deletion,
                )
            worker = self._maintenance_worker
        if not self._store.get_holder_status()["reference_source_count"]:
            return False
        return worker.start()

    def publication_scope(self, *, owner: str):
        return self._store.publication_scope(owner=owner)

    def acquire_artifacts(self, artifact_ids=(), *, owner: str, local_ids=()):
        return self._store.acquire_artifacts(artifact_ids, owner=owner, local_ids=local_ids)

    def hold_proven_artifacts(self, artifact_ids=(), *, owner: str):
        return self._store.hold_proven_artifacts(artifact_ids, owner=owner)

    def update_retention_policy(self, policy: ArtifactRetentionPolicy) -> None:
        """替换完整策略，下一次正常清理使用新值；保存设置不主动清理数据。"""
        replacement = policy.model_copy(deep=True)
        with self._lifecycle_lock:
            worker = self._maintenance_worker
            if worker is not None:
                worker.update_policy(replacement)
            self._retention_policy = replacement

    def save_image_frame(
        self,
        image_frame: ImageFrame,
        *,
        metadata: Optional[Dict[str, object]] = None,
    ) -> ArtifactRef:
        """保存图像帧；已装配运行时只排入有界维护信号，不查引用或扫描。"""
        with self._lifecycle_lock:
            if self._closing:
                raise GatewayError("制品服务正在关闭。", error_code="artifact_service_closed", http_status=409)
            self._active_saves += 1
        try:
            result = self._save_image_frame(image_frame, metadata=metadata)
        except BaseException:
            with self._lifecycle_lock:
                self._failed_saves += 1
            raise
        else:
            with self._lifecycle_lock:
                self._accepted_saves += 1
            return result
        finally:
            with self._lifecycle_lock:
                self._active_saves -= 1

    def _save_image_frame(self, image_frame, *, metadata=None):

        if image_frame.data is None:
            raise GatewayError("当前图像帧未包含内联数据，无法保存为宿主机制品。")

        mime_type, extension = self._resolve_image_mime(image_frame.encoding)
        with self._store.protect_saved_artifacts():
            artifact = self._store.save_bytes(
                kind=ArtifactKind.IMAGE,
                mime_type=mime_type,
                data=image_frame.data,
                metadata={
                    **dict(image_frame.metadata),
                    **dict(metadata or {}),
                    "camera_id": image_frame.camera_id,
                    "frame_id": image_frame.frame_id,
                    "encoding": image_frame.encoding.value,
                    "width_px": image_frame.width_px,
                    "height_px": image_frame.height_px,
                    # 保留输入帧时间，不能用制品保存时间冒充采集时间。
                    "frame_timestamp": image_frame.timestamp.isoformat(),
                    "frame_timestamp_origin": "image_frame.timestamp",
                },
                extension=extension,
            )
            with self._lifecycle_lock:
                asynchronous = self._async_maintenance
                worker = self._maintenance_worker
            if asynchronous:
                try:
                    accepted = worker is not None and worker.request(artifact_id=artifact.artifact_id) is not None
                except Exception:
                    accepted = False
                    logging.getLogger(__name__).exception("图像已保存，但制品维护触发未被接收。")
                if not accepted:
                    with self._lifecycle_lock:
                        self._unserviced_saves += 1
                return artifact
            try:
                self.cleanup_if_needed()
                status = self.get_maintenance_status()
                if status.get("status") == "failed":
                    logging.getLogger(__name__).warning("图像已保存，但制品维护存在未完成项：%s", status.get("errors"))
            except Exception:
                # 已提交的新图像仍可返回；维护故障由状态接口及日志明确暴露。
                logging.getLogger(__name__).exception("图像已保存，但制品维护失败。")
            return artifact

    def get_summary(self) -> ArtifactStorageSummary:
        """返回当前制品存储摘要。"""

        summary = self._store.build_summary()
        with self._lifecycle_lock:
            self._last_summary = summary.model_copy(deep=True)
        return summary

    def get_cached_summary(self) -> Optional[ArtifactStorageSummary]:
        """健康入口只读取上次扫描摘要；尚未扫描时返回未知而不是虚构零。"""
        status = self._store.get_maintenance_status(compact=True)
        raw = status.get("result", {}).get("summary")
        if raw is not None:
            return ArtifactStorageSummary.model_validate(raw)
        with self._lifecycle_lock:
            return self._last_summary.model_copy(deep=True) if self._last_summary is not None else None

    def diagnose_links(self) -> Dict[str, object]:
        """诊断制品引用索引中的断链。"""

        return self._store.diagnose_reference_links()

    def cleanup_if_needed(self) -> ArtifactCleanupResult:
        """按策略执行制品清理。"""

        with self._lifecycle_lock:
            policy = self._retention_policy.model_copy(deep=True)
        return self._store.cleanup(policy)

    def get_maintenance_status(self) -> Dict[str, object]:
        """只复制有界缓存状态；不触发文件扫描、仓储查询或提供器工作。"""
        status = self._store.get_maintenance_status(compact=True)
        with self._lifecycle_lock:
            worker = self._maintenance_worker
            status.update({"asynchronous": self._async_maintenance, "closing": self._closing,
                           "active_saves": self._active_saves, "accepted_saves": self._accepted_saves,
                           "failed_saves": self._failed_saves, "unserviced_saves": self._unserviced_saves})
        status["worker"] = worker.status(compact=True) if worker is not None else None
        worker_status = status["worker"] or {}
        status["maintenance_pending"] = bool(worker_status.get("pending_requests") or worker_status.get("active_requests"))
        status["holders"] = self._store.get_holder_status()
        return status

    def close(self, *, timeout: float = 1.0) -> bool:
        """先封闭保存/维护准入，再有界等待；不合作线程的句柄和依赖仍保留。"""
        with self._lifecycle_lock:
            self._closing = True
            worker = self._maintenance_worker
        stopped = worker is None or worker.close(timeout=timeout)["stopped"]
        with self._lifecycle_lock:
            return stopped and self._active_saves == 0

    def _resolve_image_mime(self, encoding: ImageEncoding) -> Tuple[str, str]:
        if encoding == ImageEncoding.JPEG:
            return "image/jpeg", ".jpg"
        if encoding == ImageEncoding.PNG:
            return "image/png", ".png"
        return "application/octet-stream", ".bin"
