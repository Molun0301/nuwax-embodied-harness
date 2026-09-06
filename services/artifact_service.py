from __future__ import annotations

from contracts.artifacts import ArtifactCleanupResult, ArtifactKind, ArtifactRef, ArtifactRetentionPolicy, ArtifactStorageSummary
from contracts.image import ImageEncoding, ImageFrame
from services.artifact_store import LocalArtifactStore
from core.errors import GatewayError
from typing import Dict, Optional, Tuple


class ArtifactService:
    """制品服务。"""

    def __init__(
        self,
        store: LocalArtifactStore,
        *,
        retention_policy: ArtifactRetentionPolicy,
    ) -> None:
        self._store = store
        self._retention_policy = retention_policy

    def save_image_frame(
        self,
        image_frame: ImageFrame,
        *,
        metadata: Optional[Dict[str, object]] = None,
    ) -> ArtifactRef:
        """保存图像帧为制品。"""

        if image_frame.data is None:
            raise GatewayError("当前图像帧未包含内联数据，无法保存为宿主机制品。")

        mime_type, extension = self._resolve_image_mime(image_frame.encoding)
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
        self.cleanup_if_needed()
        return artifact

    def get_summary(self) -> ArtifactStorageSummary:
        """返回当前制品存储摘要。"""

        return self._store.build_summary()

    def diagnose_links(self) -> Dict[str, object]:
        """诊断制品引用索引中的断链。"""

        return self._store.diagnose_reference_links()

    def cleanup_if_needed(self) -> ArtifactCleanupResult:
        """按策略执行制品清理。"""

        return self._store.cleanup(self._retention_policy)

    def _resolve_image_mime(self, encoding: ImageEncoding) -> Tuple[str, str]:
        if encoding == ImageEncoding.JPEG:
            return "image/jpeg", ".jpg"
        if encoding == ImageEncoding.PNG:
            return "image/png", ".png"
        return "application/octet-stream", ".bin"
