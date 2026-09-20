"""由记忆变更唤醒的实例快照推送；所有网页共享一份有界快照。"""

from __future__ import annotations

import logging
import threading
import time


LOGGER = logging.getLogger(__name__)


def _snapshot_frame_id(payload):
    # 实例可携带不同坐标系；只有整份快照明确同系时才声明共同坐标身份。
    frames = {row.get("frame_id") for row in payload.get("instances", ())}
    if len(frames) != 1:
        return None
    frame = next(iter(frames))
    return frame if isinstance(frame, str) and frame.strip() else None


class SemanticInstancePublisher:
    def __init__(self, stream_bus, snapshot_reader) -> None:
        self._stream_bus = stream_bus
        self._snapshot_reader = snapshot_reader
        self._condition = threading.Condition()
        self._dirty = True
        self._stopping = False
        self._thread = None

    def start(self) -> None:
        if self._stream_bus is None:
            return
        with self._condition:
            if self._thread is not None and self._thread.is_alive():
                if self._stopping:
                    raise RuntimeError("空间记忆推送仍在关闭。")
                return
            self._stopping = False
            self._dirty = True
            self._thread = threading.Thread(target=self._run, name="memory-instance-stream", daemon=True)
            self._thread.start()

    def notify(self) -> None:
        # 通知只占一个槽位，不在检测/关联线程查询数据库或编码全量记录。
        with self._condition:
            self._dirty = True
            self._condition.notify()

    def stop(self) -> None:
        with self._condition:
            self._stopping = True
            self._condition.notify_all()
            thread = self._thread
        if thread is not None:
            thread.join(timeout=3.0)
            if thread.is_alive():
                raise RuntimeError("空间记忆推送线程未完成关闭。")
        with self._condition:
            if self._thread is thread:
                self._thread = None

    def _run(self) -> None:
        next_update_at = None
        next_publish_at = 0.0
        while True:
            with self._condition:
                while not self._dirty and not self._stopping:
                    timeout = None if next_update_at is None else max(0.0, next_update_at - time.time())
                    if timeout == 0.0:
                        break
                    self._condition.wait(timeout=timeout)
                if self._stopping:
                    return
                # 高频观察合并到最多每秒十份快照，通知不能不断延后发布期限。
                while not self._stopping and time.monotonic() < next_publish_at:
                    self._condition.wait(timeout=next_publish_at - time.monotonic())
                if self._stopping:
                    return
                self._dirty = False
            frame_id = None
            try:
                payload = self._snapshot_reader()
                next_update_at = payload.pop("_next_update_at", None)
                frame_id = _snapshot_frame_id(payload)
                status = "available"
            except Exception:
                LOGGER.exception("读取空间记忆实时快照失败。")
                payload = {"available": False, "message": "空间记忆快照读取失败。"}
                next_update_at = None
                status = "error"
            with self._condition:
                if self._stopping:
                    return
            self._stream_bus.publish(
                "memory.instances", source="memory_service", payload=payload,
                frame_id=frame_id, status=status,
            )
            next_publish_at = time.monotonic() + 0.1
