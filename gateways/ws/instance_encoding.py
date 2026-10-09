"""实例公开快照的路由内 WS JSON 复用；不拥有连接或源载荷。"""
from __future__ import annotations

from collections import OrderedDict
import sys
import weakref

from contracts.streams import InstanceSnapshotFrame


class InstanceSnapshotEncoder:
    """只在路由的事件循环中使用；编码完成后连接才各自 await 发送。

    一个拥有方只对应现有公开实例视图及固定 WS JSON 编码选项，不能用于
    SSE、用户私有投影或其他编码配置。条目和字节上限只约束保留的 str；
    不约束源快照、首次编码的临时内存或套接字已经持有的文本。
    """

    def __init__(self, encode_frame, *, max_entries=8, max_bytes=4 * 1024 * 1024):
        self._encode_frame = encode_frame
        self._max_entries = max(0, int(max_entries))
        self._max_bytes = max(0, int(max_bytes))
        self._entries = OrderedDict()
        self._retained_bytes = 0
        self._closed = False

    def encode(self, frame):
        # 资格来自生产者的所有权移交类型，不根据 topic/source 猜测可变载荷。
        if type(frame) is not InstanceSnapshotFrame or frame.cursor is None or self._closed:
            return self._encode_frame(frame)
        key = id(frame)
        cached = self._entries.get(key)
        if cached is not None:
            if cached[0]() is frame:
                self._entries.move_to_end(key)
                return cached[1]
            self._retained_bytes -= self._entries.pop(key)[2]
        encoded = self._encode_frame(frame)
        size = sys.getsizeof(encoded)  # CPython str 大小为 O(1)，不编码第二遍或扫描文本。
        if type(encoded) is not str or not self._max_entries or size > self._max_bytes:
            return encoded
        while self._entries and (len(self._entries) >= self._max_entries
                                 or self._retained_bytes + size > self._max_bytes):
            _, retired = self._entries.popitem(last=False)
            self._retained_bytes -= retired[2]
        self._entries[key] = (weakref.ref(frame), encoded, size)
        self._retained_bytes += size
        return encoded

    def close(self):
        self._closed = True
        self._entries.clear()
        self._retained_bytes = 0
