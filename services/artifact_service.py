"""临时图像制品与保留策略入口。"""
from __future__ import annotations

import logging
import threading
from typing import Optional

from contracts.artifacts import ArtifactCleanupResult, ArtifactKind, ArtifactRef, ArtifactRetentionPolicy, ArtifactStorageSummary
from contracts.image import ImageEncoding, ImageFrame
from core.errors import GatewayError
from services.artifact_maintenance import ArtifactMaintenanceWorker
from services.artifact_store import LocalArtifactStore


class ArtifactService:
    def __init__(self, store: LocalArtifactStore, *, retention_policy: ArtifactRetentionPolicy) -> None:
        self._store = store
        self._retention_policy = retention_policy.model_copy(deep=True)
        self._condition = threading.Condition(threading.RLock())
        self._maintenance_worker = ArtifactMaintenanceWorker(store, policy=retention_policy)
        self._closing = False
        self._active_saves = 0
        self._last_summary = None

    def start_maintenance(self) -> None:
        with self._condition:
            self._ensure_open()
            self._maintenance_worker.start()

    def _ensure_open(self) -> None:
        if self._closing:
            raise GatewayError("制品服务正在关闭。", error_code="artifact_service_closed", http_status=409)

    def update_retention_policy(self, policy: ArtifactRetentionPolicy) -> None:
        with self._condition:
            self._retention_policy = policy.model_copy(deep=True)
            self._maintenance_worker.update_policy(policy)

    def save_image_frame(self, image_frame: ImageFrame, *, metadata: Optional[dict] = None) -> ArtifactRef:
        with self._condition:
            self._ensure_open()
            self._active_saves += 1
        try:
            if image_frame.data is None:
                raise GatewayError("当前图像帧未包含内联数据，无法保存为宿主机制品。")
            mime_type, extension = self._resolve_image_mime(image_frame.encoding)
            artifact = self._store.save_bytes(kind=ArtifactKind.IMAGE, mime_type=mime_type, data=image_frame.data,
                metadata={**dict(image_frame.metadata), **dict(metadata or {}), "camera_id": image_frame.camera_id,
                    "frame_id": image_frame.frame_id, "encoding": image_frame.encoding.value,
                    "width_px": image_frame.width_px, "height_px": image_frame.height_px,
                    "frame_timestamp": image_frame.timestamp.isoformat(), "frame_timestamp_origin": "image_frame.timestamp"},
                extension=extension)
            try:
                self._maintenance_worker.request()
            except Exception:
                logging.getLogger(__name__).exception("图像已保存，但临时制品维护未能启动。")
            return artifact
        finally:
            with self._condition:
                self._active_saves -= 1
                self._condition.notify_all()

    def read_bytes(self, artifact_id: str) -> bytes:
        return self._store.read_bytes(artifact_id)

    def get_summary(self) -> ArtifactStorageSummary:
        summary = self._store.build_summary()
        with self._condition:
            self._last_summary = summary.model_copy(deep=True)
        return summary

    def get_cached_summary(self) -> Optional[ArtifactStorageSummary]:
        raw = self._store.get_maintenance_status().get("result", {}).get("summary")
        if raw is not None:
            return ArtifactStorageSummary.model_validate(raw)
        with self._condition:
            return self._last_summary.model_copy(deep=True) if self._last_summary is not None else None

    def cleanup_if_needed(self) -> ArtifactCleanupResult:
        with self._condition:
            policy = self._retention_policy
        return self._store.cleanup(policy)

    def get_maintenance_status(self) -> dict:
        status = self._store.get_maintenance_status()
        worker = self._maintenance_worker.status()
        with self._condition:
            status.update(closing=self._closing, active_saves=self._active_saves, worker=worker,
                          maintenance_pending=worker["pending"] or worker["active"])
        return status

    def close(self) -> None:
        with self._condition:
            self._closing = True
            while self._active_saves:
                self._condition.wait()
        self._maintenance_worker.close()

    @staticmethod
    def _resolve_image_mime(encoding: ImageEncoding) -> tuple[str, str]:
        if encoding == ImageEncoding.JPEG:
            return "image/jpeg", ".jpg"
        if encoding == ImageEncoding.PNG:
            return "image/png", ".png"
        return "application/octet-stream", ".bin"
