"""视频消费者用内核共享锁登记需求，进程退出即自动释放。"""
from contextlib import contextmanager
import fcntl
import os
import threading


VIDEO_SHM_PATH = "/tmp/nuwax_video.shm"


@contextmanager
def video_consumer(path=VIDEO_SHM_PATH):
    descriptor = os.open(str(path) + ".demand", os.O_CREAT | os.O_RDWR | os.O_CLOEXEC, 0o600)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_SH)
        yield
    finally:
        os.close(descriptor)


class VideoDemandMonitor:
    """写者只探测是否存在共享锁，不持有消费者的资源。"""

    def __init__(self, path=VIDEO_SHM_PATH):
        self._descriptor = os.open(str(path) + ".demand", os.O_CREAT | os.O_RDWR | os.O_CLOEXEC, 0o600)
        self._lock = threading.Lock()

    def has_consumers(self):
        with self._lock:
            try:
                fcntl.flock(self._descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                return True
            fcntl.flock(self._descriptor, fcntl.LOCK_UN)
            return False

    def close(self):
        with self._lock:
            os.close(self._descriptor)
