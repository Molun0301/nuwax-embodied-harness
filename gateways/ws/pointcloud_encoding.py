"""有界的规范点云传输复用；不持有共享内存视图或改变编码合同。"""
from __future__ import annotations

import asyncio
import weakref
from collections import OrderedDict
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from typing import Optional

from contracts.streams import StreamFrame, StreamPayloadKind

_CACHE_BYTES = 32 * 1024 * 1024
_CACHE_ENTRIES = 8
_IDENTITY_CHAR_LIMIT = 64 * 1024
_METADATA_FIELDS = (
    "source", "source_epoch", "stream_generation", "revision", "revision_ordinal",
    "frame_id", "source_timestamp_ns", "point_count", "format",
)


@dataclass
class _Job:
    encode: object
    frame: Optional[StreamFrame]


def _run_job(job, work):
    try:
        if work.set_running_or_notify_cancel():
            try:
                encoded = job.encode(job.frame)
            except BaseException as error:  # noqa: BLE001 - 与 Future 执行边界相同，向全部等待者交付。
                work.set_exception(error)
            else:
                work.set_result(encoded)
    finally:
        job.frame = None


@dataclass
class _Entry:
    raw: bytes
    charge: int
    result: asyncio.Future
    finished: asyncio.Future
    job: _Job
    work: Optional[Future] = None
    submission: Optional[Future] = None
    waiters: int = 0


def _capture(frame):
    """仅快照有界的内建标量；完整验证及序列化仍由原编码器在工作线程完成。"""
    if type(frame) is not StreamFrame:
        return None
    raw = frame.payload
    source_metadata = frame.metadata
    topic = frame.topic
    if (topic != "pointcloud_map.snapshot" or frame.payload_kind != StreamPayloadKind.BINARY
            or type(raw) is not bytes or type(source_metadata) is not dict):
        return None
    metadata = {name: source_metadata.get(name) for name in _METADATA_FIELDS}
    identity = (topic, frame.cursor, frame.revision, frame.frame_id,
                *(metadata[name] for name in _METADATA_FIELDS))
    chars = 0
    for value in identity:
        if type(value) is str:
            chars += len(value)
        elif value is not None and (type(value) is not int or value.bit_length() > 64):
            return None
    if chars > _IDENTITY_CHAR_LIMIT:
        return None
    # 输入 Unicode 最多4字节/字符，JSON最多6字节/字符；另留容器/整数/帧头余量。
    # 输出与原始 XYZ 均计费，即使后者同时由来源/客户端队列持有。
    charge = len(raw) * 2 + chars * 12 + 8192
    key = ("NUWAXPC-v1", id(raw), identity)
    return key, raw, metadata, charge


class CanonicalPointCloudEncoder:
    """一个专用工作线程，异步共享结果；不同规范帧明确串行编码。

    字节/条目预算包括在途工作；饱和时等待异步准入，不溢出到通用线程池。
    等待者只持有发送路径原已领取的帧；预算不代表所有连接内存的总上限。
    可变、超预算、非规范输入仍走原线程池，故这些兼容回退并非完全隔离。
    """

    def __init__(self, encode_frame, run_blocking, *, max_bytes=_CACHE_BYTES, max_entries=_CACHE_ENTRIES):
        self._encode_frame = encode_frame
        self._run_blocking = run_blocking
        self._max_bytes = max(0, int(max_bytes))
        self._max_entries = max(0, int(max_entries))
        self._entries = OrderedDict()
        self._charged_bytes = 0
        self._loop = None
        self._changed = None
        self._executor = None
        self._closed = False

    def _notify_capacity(self):
        if self._changed is not None:
            old, self._changed = self._changed, asyncio.Event()
            old.set()

    def _remove(self, key, entry):
        if self._entries.get(key) is entry:
            del self._entries[key]
            self._charged_bytes -= entry.charge
            entry.raw = b""
            self._notify_capacity()

    def _make_room(self, charge):
        for key, entry in tuple(self._entries.items()):
            if self._charged_bytes + charge <= self._max_bytes and len(self._entries) < self._max_entries:
                return True
            if entry.finished.done():
                self._remove(key, entry)
        return self._charged_bytes + charge <= self._max_bytes and len(self._entries) < self._max_entries

    @staticmethod
    def _observe_completion(result):
        if not result.cancelled():
            result.exception()

    def _finished(self, key, entry, work):
        entry.finished.set_result(None)
        if work.cancelled():
            self._remove(key, entry)
            if not entry.result.done():
                entry.result.cancel()
            return
        try:
            encoded = work.result()
        except BaseException as error:  # noqa: BLE001 - 保持原编码器异常，清除缓存拥有权。
            self._remove(key, entry)
            entry.result.set_exception(error)
        else:
            if type(encoded) is not bytes or self._closed or not entry.waiters:
                self._remove(key, entry)
            entry.result.set_result(encoded)
        # 完成后即使保留热条目，也要唤醒等待者，使其能驱逐完成条目后准入。
        self._notify_capacity()

    def _cancel_queued(self, key, entry):
        if entry.work is not None and entry.work.cancel():
            # 只取消控制 Future；提交项仍会被工作线程领取并立刻跳过编码。
            # 在 finished 确认领取前保留条目/预算，避免取消风暴堆积无界 tombstone。
            entry.job.frame = None
            if not entry.result.done():
                entry.result.cancel()
            return True
        return False

    def _submit(self, key, raw, metadata, charge):
        identity = key[-1]
        frozen = StreamFrame.model_construct(
            topic=identity[0], cursor=identity[1], revision=identity[2], frame_id=identity[3],
            payload_kind=StreamPayloadKind.BINARY, payload=raw, metadata=metadata,
            source="canonical_ws_encoding",
        )
        result = self._loop.create_future()
        result.add_done_callback(self._observe_completion)
        entry = _Entry(raw=raw, charge=charge, result=result, finished=self._loop.create_future(),
                       job=_Job(self._encode_frame, frozen), work=Future())
        self._entries[key] = entry
        self._charged_bytes += charge
        try:
            if self._executor is None:
                self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="nuwax-cloud-encode")
            entry.submission = self._executor.submit(_run_job, entry.job, entry.work)
        except BaseException:
            # submit 可能已入队，却在启动首个线程时失败。取消控制票据并退役该池，
            # 不能反复向无工作线程的队列累积未计费项，或以后调用 encode(None)。
            entry.work.cancel()
            entry.job.frame = None
            failed_executor, self._executor = self._executor, None
            if failed_executor is not None:
                failed_executor.shutdown(wait=False)
            self._remove(key, entry)
            result.cancel()
            entry.finished.cancel()
            raise
        # Future 不得经回调形成 owner/entry/大载荷的环状保留。
        owner_ref, entry_ref = weakref.ref(self), weakref.ref(entry)
        loop = self._loop

        def completed(_submission):
            owner, current = owner_ref(), entry_ref()
            if owner is not None and current is not None:
                loop.call_soon_threadsafe(owner._finished, key, current, current.work)

        entry.submission.add_done_callback(completed)
        return entry

    async def encode(self, frame):
        loop = asyncio.get_running_loop()
        if self._closed:
            raise RuntimeError("点云编码服务已关闭。")
        if self._loop is not None and self._loop is not loop:
            return await self._run_blocking(self._encode_frame, frame)
        if self._loop is None:
            self._loop, self._changed = loop, asyncio.Event()
        captured = _capture(frame)
        if captured is None:
            return await self._run_blocking(self._encode_frame, frame)
        key, raw, metadata, charge = captured
        if charge > self._max_bytes or not self._max_entries:
            return await self._run_blocking(self._encode_frame, frame)
        while True:
            if self._closed:
                raise RuntimeError("点云编码服务已关闭。")
            entry = self._entries.get(key)
            if entry is not None:
                if not entry.result.cancelled():
                    break
                await self._changed.wait()
                continue
            if self._make_room(charge):
                entry = self._submit(key, raw, metadata, charge)
                break
            # 不提交额外工作，不占用任何共享线程；取消准入等待无需清理任务。
            await self._changed.wait()
        self._entries.move_to_end(key)
        entry.waiters += 1
        try:
            return await asyncio.shield(entry.result)
        finally:
            entry.waiters -= 1
            if not entry.waiters and not entry.result.done():
                self._cancel_queued(key, entry)

    async def close(self):
        if self._loop is not None and self._loop is not asyncio.get_running_loop():
            raise RuntimeError("点云编码拥有方必须在接纳工作的事件循环中关闭。")
        self._closed = True
        self._notify_capacity()
        for key, entry in tuple(self._entries.items()):
            self._cancel_queued(key, entry)
        if self._executor is not None:
            # 不在事件循环 join 线程；禁止新任务，线程完成当前纯编码后自行退出。
            self._executor.shutdown(wait=False)
        results = [entry.finished for entry in self._entries.values()]
        cancelled = False
        try:
            if results:
                drain = asyncio.gather(*results, return_exceptions=True)
                while not drain.done():
                    try:
                        await asyncio.shield(drain)
                    except asyncio.CancelledError:
                        # 不伪装成已停止：只等待本拥有方已经运行的纯编码，不等通用池。
                        cancelled = True
                drain.result()
        finally:
            self._entries.clear()
            self._charged_bytes = 0
            self._loop = None
        if cancelled:
            raise asyncio.CancelledError
