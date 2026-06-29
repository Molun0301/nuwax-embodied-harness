#!/usr/bin/env python3
"""Host-side bridge: web_video_server snapshot JPEG -> Nuwax video SHM."""

from __future__ import annotations

import logging
import mmap
import os
import struct
import sys
import time
from pathlib import Path
from typing import Optional
from urllib.error import URLError
from urllib.request import urlopen


LOGGER = logging.getLogger("puppypi_web_video_snapshot_to_shm")

SNAPSHOT_URL = os.environ.get(
    "PUPPYPI_WEB_VIDEO_SNAPSHOT_URL",
    "http://127.0.0.1:8080/snapshot?topic=/usb_cam/image_raw",
).strip()
SHM_PATH = os.environ.get("PUPPYPI_SHM_PATH", os.environ.get("NUWAX_VIDEO_SHM_PATH", "/tmp/nuwax_video.shm")).strip()
FPS = float(os.environ.get("PUPPYPI_VIDEO_FPS", "10"))
WIDTH = int(os.environ.get("PUPPYPI_VIDEO_WIDTH", "640"))
HEIGHT = int(os.environ.get("PUPPYPI_VIDEO_HEIGHT", "480"))
REQUEST_TIMEOUT_SEC = float(os.environ.get("PUPPYPI_SNAPSHOT_TIMEOUT_SEC", "2.0"))

SHM_MAGIC = 0x4E55_5741_5856_4944
SHM_VERSION = 1
HEADER_SIZE = 64
MAX_FRAME_SIZE = 512 * 1024
BUFFER_SIZE = HEADER_SIZE + MAX_FRAME_SIZE * 2


class VideoShmWriter:
    def __init__(self, shm_path: str) -> None:
        self._shm_path = shm_path
        self._file = None
        self._mm: Optional[mmap.mmap] = None
        self._sequence = 0

    def open(self) -> None:
        if self._mm is not None:
            return
        path = Path(self._shm_path)
        if path.is_dir():
            raise IsADirectoryError(f"SHM path is a directory, remove it first: {self._shm_path}")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch(exist_ok=True)
        self._file = open(self._shm_path, "r+b")
        self._file.truncate(BUFFER_SIZE)
        self._mm = mmap.mmap(self._file.fileno(), BUFFER_SIZE, access=mmap.ACCESS_WRITE)
        self._mm[:HEADER_SIZE] = b"\x00" * HEADER_SIZE
        struct.pack_into("<QI", self._mm, 0, SHM_MAGIC, SHM_VERSION)
        self._mm.flush()

    def write_frame(self, jpeg_data: bytes) -> bool:
        self.open()
        if len(jpeg_data) == 0 or len(jpeg_data) > MAX_FRAME_SIZE:
            LOGGER.warning("JPEG frame size invalid: %s bytes", len(jpeg_data))
            return False

        mm = self._mm
        assert mm is not None
        seq = self._sequence + 1
        active_buffer = seq % 2
        buf_offset = HEADER_SIZE + active_buffer * MAX_FRAME_SIZE
        mm[buf_offset:buf_offset + len(jpeg_data)] = jpeg_data
        struct.pack_into("<IHHQI", mm, 24, len(jpeg_data), WIDTH, HEIGHT, time.time_ns(), active_buffer)
        struct.pack_into("<Q", mm, 16, seq)
        self._sequence = seq
        mm.flush()
        return True

    def close(self) -> None:
        if self._mm is not None:
            self._mm.close()
            self._mm = None
        if self._file is not None:
            self._file.close()
            self._file = None


def fetch_snapshot() -> bytes:
    with urlopen(SNAPSHOT_URL, timeout=REQUEST_TIMEOUT_SEC) as response:
        payload = response.read()
    if not payload.startswith(b"\xff\xd8"):
        raise ValueError(f"snapshot response is not JPEG: {len(payload)} bytes")
    return payload


def main() -> int:
    logging.basicConfig(level=os.environ.get("PUPPYPI_PROXY_LOG_LEVEL", "INFO"))
    if not SNAPSHOT_URL:
        LOGGER.error("PUPPYPI_WEB_VIDEO_SNAPSHOT_URL is empty")
        return 1
    if not SHM_PATH:
        LOGGER.error("PUPPYPI_SHM_PATH/NUWAX_VIDEO_SHM_PATH is empty")
        return 1

    writer = VideoShmWriter(SHM_PATH)
    writer.open()
    interval = 1.0 / max(1.0, FPS)
    frame_count = 0
    LOGGER.info("Bridging snapshot %s -> %s @ %.1f fps", SNAPSHOT_URL, SHM_PATH, FPS)

    while True:
        started_at = time.monotonic()
        try:
            frame = fetch_snapshot()
            writer.write_frame(frame)
            frame_count += 1
            if frame_count == 1 or frame_count % 100 == 0:
                LOGGER.info("Wrote snapshot frame count=%s bytes=%s", frame_count, len(frame))
        except Exception:
            LOGGER.warning("snapshot fetch/write failed", exc_info=True)
            time.sleep(1.0)
            continue

        elapsed = time.monotonic() - started_at
        time.sleep(max(0.0, interval - elapsed))


if __name__ == "__main__":
    sys.exit(main())
