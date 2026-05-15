from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import threading
import time
from typing import Any, Dict, Optional

from contracts.image import ImageEncoding, ImageFrame


@dataclass(frozen=True)
class ReceivedVideoFrame:
    """平台从端侧推流接收到的 latest-only 视频帧。"""

    camera_id: str
    frame_id: str
    width_px: int
    height_px: int
    encoding: str
    data: bytes
    sequence: int
    timestamp: datetime
    metadata: Dict[str, Any]

    @property
    def payload_bytes(self) -> int:
        return len(self.data)


class VideoStreamReceiver:
    """平台视频 ingest 接收器。

    该类只接收 Go2 端侧已经推上来的 JPEG 视频帧，并维护 latest-only 缓冲；
    它不触发机器人图像采集，也不调用任何相机 SDK。
    """

    source_kind = "platform_video_ingest_stream"

    def __init__(self, *, max_frame_age_sec: float = 2.0) -> None:
        self._condition = threading.Condition()
        self._latest_frame: Optional[ReceivedVideoFrame] = None
        self._sequence = 0
        self._max_frame_age_sec = max(0.1, float(max_frame_age_sec))
        self._connected = False
        self._camera_id = "front_camera"
        self._frame_id = "go2/front_camera"
        self._width_px = 1280
        self._height_px = 720
        self._encoding = "jpeg"
        self._last_client_id: Optional[str] = None
        self._last_frame_at: Optional[datetime] = None
        self._last_frame_monotonic: Optional[float] = None
        self._fps_estimate = 0.0
        self._total_payload_bytes = 0
        self._frame_count = 0
        self._connection_count = 0
        self._disconnect_count = 0
        self._last_disconnect_at: Optional[datetime] = None
        self._last_error: Optional[str] = None
        self._stream_metadata: Dict[str, Any] = {}

    def reset_stream(
        self,
        *,
        client_id: str,
        reason: str = "reset",
        error: Optional[str] = None,
    ) -> Dict[str, Any]:
        with self._condition:
            if client_id == self._last_client_id:
                self._connected = False
                self._latest_frame = None
                self._last_frame_at = None
                self._last_frame_monotonic = None
                self._fps_estimate = 0.0
                self._frame_count = 0
                self._total_payload_bytes = 0
                self._last_error = str(error) if error else None
                self._stream_metadata = {**dict(self._stream_metadata), "reset_reason": reason}
                self._condition.notify_all()
            return self.status_snapshot()

    def start_stream(
        self,
        *,
        client_id: str,
        camera_id: Optional[str] = None,
        frame_id: Optional[str] = None,
        width_px: Optional[int] = None,
        height_px: Optional[int] = None,
        encoding: str = "jpeg",
        metadata: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        normalized_encoding = str(encoding or "jpeg").strip().lower()
        if normalized_encoding in {"jpg", "image/jpeg"}:
            normalized_encoding = "jpeg"
        with self._condition:
            self._connected = True
            self._last_client_id = str(client_id or "")
            self._camera_id = str(camera_id or self._camera_id or "front_camera")
            self._frame_id = str(frame_id or self._frame_id or self._camera_id)
            self._width_px = max(1, int(width_px or self._width_px or 1280))
            self._height_px = max(1, int(height_px or self._height_px or 720))
            self._encoding = normalized_encoding
            self._connection_count += 1
            self._last_error = None
            self._stream_metadata = dict(metadata or {})
            self._latest_frame = None
            self._last_frame_at = None
            self._last_frame_monotonic = None
            self._fps_estimate = 0.0
            self._frame_count = 0
            self._total_payload_bytes = 0
            self._condition.notify_all()
            return self.status_snapshot()

    def mark_disconnected(self, *, client_id: Optional[str] = None, error: Optional[str] = None) -> Dict[str, Any]:
        with self._condition:
            if client_id is None or client_id == self._last_client_id:
                self._connected = False
                self._disconnect_count += 1
                self._last_disconnect_at = datetime.now(timezone.utc)
                if error:
                    self._last_error = str(error)
            self._condition.notify_all()
            return self.status_snapshot()

    def ingest_jpeg(
        self,
        payload: bytes,
        *,
        client_id: str,
        camera_id: Optional[str] = None,
        frame_id: Optional[str] = None,
        width_px: Optional[int] = None,
        height_px: Optional[int] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> ReceivedVideoFrame:
        jpeg_payload = bytes(payload or b"")
        if not self._looks_like_jpeg(jpeg_payload):
            raise ValueError("video ingest 只接受完整 JPEG 帧。")
        now = datetime.now(timezone.utc)
        now_monotonic = time.monotonic()
        with self._condition:
            normalized_client_id = str(client_id or self._last_client_id or "")
            if self._last_client_id and normalized_client_id and normalized_client_id != self._last_client_id:
                raise ValueError("视频 ingest 帧来自非当前活动推流客户端。")
            resolved_camera_id = str(camera_id or self._camera_id or "front_camera")
            resolved_frame_id = str(frame_id or self._frame_id or resolved_camera_id)
            resolved_width = max(1, int(width_px or self._width_px or 1280))
            resolved_height = max(1, int(height_px or self._height_px or 720))
            self._sequence += 1
            if self._last_frame_monotonic is not None:
                delta_sec = max(1e-6, now_monotonic - self._last_frame_monotonic)
                instant_fps = 1.0 / delta_sec
                self._fps_estimate = instant_fps if self._fps_estimate <= 0 else (self._fps_estimate * 0.8 + instant_fps * 0.2)
            self._last_frame_monotonic = now_monotonic
            self._last_frame_at = now
            self._frame_count += 1
            self._total_payload_bytes += len(jpeg_payload)
            self._connected = True
            self._last_client_id = normalized_client_id
            self._camera_id = resolved_camera_id
            self._frame_id = resolved_frame_id
            self._width_px = resolved_width
            self._height_px = resolved_height
            merged_metadata = {
                **dict(self._stream_metadata),
                **dict(metadata or {}),
                "source_kind": self.source_kind,
                "upstream_source_kind": str((metadata or {}).get("source_kind") or self._stream_metadata.get("source_kind") or "go2_side_stream_push"),
                "client_id": self._last_client_id,
                "ingest_sequence": self._sequence,
                "transport": "websocket_jpeg_ingest",
            }
            frame = ReceivedVideoFrame(
                camera_id=resolved_camera_id,
                frame_id=resolved_frame_id,
                width_px=resolved_width,
                height_px=resolved_height,
                encoding="jpeg",
                data=jpeg_payload,
                sequence=self._sequence,
                timestamp=now,
                metadata=merged_metadata,
            )
            self._latest_frame = frame
            self._condition.notify_all()
            return frame

    def latest_frame(self, camera_id: Optional[str] = None) -> Optional[ReceivedVideoFrame]:
        with self._condition:
            if not self._is_current_frame_available_locked():
                return None
            frame = self._latest_frame
            if frame is None:
                return None
            if camera_id and frame.camera_id != camera_id:
                return None
            return frame

    def wait_for_frame(
        self,
        after_sequence: int,
        *,
        timeout_sec: float,
        camera_id: Optional[str] = None,
    ) -> Optional[ReceivedVideoFrame]:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        with self._condition:
            while True:
                frame = self._latest_frame
                if (
                    frame is not None
                    and self._is_current_frame_available_locked()
                    and frame.sequence > int(after_sequence or 0)
                    and (not camera_id or frame.camera_id == camera_id)
                ):
                    return frame
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(timeout=remaining)

    def get_latest_image(self, camera_id: Optional[str] = None) -> Optional[ImageFrame]:
        frame = self.latest_frame(camera_id)
        if frame is None:
            return None
        return ImageFrame(
            camera_id=frame.camera_id,
            frame_id=frame.frame_id,
            width_px=frame.width_px,
            height_px=frame.height_px,
            encoding=ImageEncoding.JPEG,
            data=bytes(frame.data),
            timestamp=frame.timestamp,
            metadata=dict(frame.metadata),
        )

    def status_snapshot(self) -> Dict[str, Any]:
        with self._condition:
            latest = self._latest_frame
            age_sec = self._last_frame_age_sec_locked()
            available = self._is_current_frame_available_locked()
            return {
                "available": available,
                "connected": self._connected,
                "camera_id": self._camera_id,
                "frame_id": self._frame_id,
                "width_px": self._width_px,
                "height_px": self._height_px,
                "encoding": self._encoding,
                "frame_count": self._frame_count,
                "last_frame_at": self._last_frame_at,
                "last_frame_age_sec": age_sec,
                "max_frame_age_sec": self._max_frame_age_sec,
                "last_client_id": self._last_client_id,
                "fps_estimate": self._fps_estimate,
                "payload_bytes": latest.payload_bytes if latest is not None else 0,
                "cached_frame_available": latest is not None,
                "total_payload_bytes": self._total_payload_bytes,
                "sequence": latest.sequence if latest is not None else 0,
                "stale": bool(latest is not None and not available),
                "source_kind": self.source_kind,
                "transport": "websocket_jpeg_ingest",
                "connection_count": self._connection_count,
                "disconnect_count": self._disconnect_count,
                "last_disconnect_at": self._last_disconnect_at,
                "last_error": self._last_error,
                "stream_metadata": dict(self._stream_metadata),
            }

    @staticmethod
    def _looks_like_jpeg(payload: bytes) -> bool:
        if len(payload) < 4:
            return False
        return payload.startswith(b"\xff\xd8") and payload.endswith(b"\xff\xd9")

    def _is_current_frame_available_locked(self) -> bool:
        if self._latest_frame is None or not self._connected:
            return False
        age_sec = self._last_frame_age_sec_locked()
        if age_sec is None:
            return False
        return age_sec <= self._max_frame_age_sec

    def _last_frame_age_sec_locked(self) -> Optional[float]:
        if self._last_frame_monotonic is None:
            return None
        return max(0.0, time.monotonic() - self._last_frame_monotonic)
