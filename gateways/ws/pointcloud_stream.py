"""按页面修订读取的独立点云通道；每连接只钉住一个一致性事务。"""
from __future__ import annotations

import asyncio
import json
import struct
import threading
from dataclasses import dataclass
from typing import Optional

from starlette.websockets import WebSocket, WebSocketDisconnect

_PREFIX = struct.Struct('<8sBI')
_MAGIC = b'NUWAXPC\0'
_PAGE_POINTS = 1024
_PAGES_PER_PACKET = 4
_MAX_POINTS = (32 * 1024 * 1024) // 12
_MAX_U64 = (1 << 64) - 1
_IDENTITY_FIELDS = ('source', 'source_epoch', 'stream_generation', 'layout_generation', 'frame_id')


def _u64(value):
    if type(value) not in (int, str):
        raise ValueError('invalid pointcloud integer')
    if isinstance(value, str) and (not value.isascii() or not value.isdecimal() or (len(value) > 1 and value[0] == '0')):
        raise ValueError('invalid pointcloud integer')
    number = int(value)
    if number < 0 or number > _MAX_U64:
        raise ValueError('pointcloud integer out of range')
    return str(number)


def snapshot_identity(snapshot):
    return {'source': snapshot.source, 'source_epoch': snapshot.source_epoch,
            'stream_generation': _u64(snapshot.stream_generation),
            'layout_generation': _u64(getattr(snapshot, 'layout_generation', 0)),
            'frame_id': snapshot.frame_id, 'revision_ordinal': _u64(snapshot.revision_ordinal)}


def parse_client_base(value):
    if value is None:
        return None
    if not isinstance(value, dict):
        raise TypeError('invalid pointcloud base')
    result = {}
    for name in ('source', 'source_epoch', 'frame_id'):
        text = value.get(name)
        if not isinstance(text, str) or not text or len(text) > 4096:
            raise ValueError('invalid pointcloud base identity')
        result[name] = text
    for name in ('stream_generation', 'layout_generation', 'revision_ordinal'):
        result[name] = _u64(value.get(name))
    return result


@dataclass(frozen=True)
class _LegacyPage:
    chunk_index: int
    revision_ordinal: int
    point_count: int
    xyz_bytes: object


class PointCloudTransfer:
    """固定事务读取不可变页面；每次仅拼装最多四页，不合并完整 XYZ。"""
    def __init__(self, snapshot, base, transfer_id):
        self.snapshot = snapshot
        self.identity = snapshot_identity(snapshot)
        self.reset = base is None or any(base.get(key) != self.identity[key] for key in _IDENTITY_FIELDS)
        if not self.reset and int(base['revision_ordinal']) > snapshot.revision_ordinal:
            self.reset = True
        self.base_revision = '0' if self.reset else base['revision_ordinal']
        self.transfer_id = str(transfer_id)
        self.next_index = 0
        count = snapshot.point_count
        self.page_points = getattr(snapshot, 'chunk_size_points', 0) or _PAGE_POINTS
        if not isinstance(count, int) or count < 0 or count > _MAX_POINTS or self.page_points != _PAGE_POINTS:
            raise ValueError('unsupported pointcloud size or page layout')
        self.chunked = bool(getattr(snapshot, 'chunked', False))
        if self.chunked:
            self.chunks = tuple(chunk for chunk in snapshot.chunks
                                if self.reset or chunk.revision_ordinal > int(self.base_revision))
        else:
            # 已保存/旧生产者完整来源只在来源修订变化时分段；不伪称源头增量读取。
            self.reset = True
            self.base_revision = '0'
            raw = memoryview(snapshot.xyz_bytes)
            self.chunks = tuple(_LegacyPage(index, snapshot.revision_ordinal,
                                min(_PAGE_POINTS, count - index * _PAGE_POINTS),
                                raw[index * _PAGE_POINTS * 12:(index + 1) * _PAGE_POINTS * 12])
                                for index in range((count + _PAGE_POINTS - 1) // _PAGE_POINTS))
        self.packet_count = max(1, (len(self.chunks) + _PAGES_PER_PACKET - 1) // _PAGES_PER_PACKET)
        self.encoded_bytes = 0
        self.encoded_xyz_bytes = 0

    @property
    def complete(self):
        return self.next_index >= self.packet_count

    def next_packet(self):
        if self.complete:
            return None
        packet_index = self.next_index
        start = packet_index * _PAGES_PER_PACKET
        chunks = self.chunks[start:start + _PAGES_PER_PACKET]
        snapshot = self.snapshot
        header = {**self.identity, 'topic': 'pointcloud_map.pages', 'cursor': '0',
                  'revision': snapshot.revision, 'source_timestamp_ns': _u64(snapshot.source_timestamp_ns),
                  'format': 'float32-xyz-le', 'point_count': snapshot.point_count,
                  'chunk_size_points': self.page_points, 'base_revision_ordinal': self.base_revision,
                  'reset': self.reset, 'transfer_id': self.transfer_id,
                  'packet_index': packet_index, 'packet_count': self.packet_count,
                  'changed_chunk_count': len(self.chunks), 'source_incremental': self.chunked,
                  'mapping_source_epoch': str(snapshot.provenance.get('mapping_source_epoch') or ''),
                  'chunks': [{'chunk_index': item.chunk_index, 'revision_ordinal': _u64(item.revision_ordinal),
                              'point_count': item.point_count} for item in chunks],
                  'payload_length': sum(len(item.xyz_bytes) for item in chunks)}
        encoded = json.dumps(header, ensure_ascii=False, separators=(',', ':')).encode('utf8')
        # 头部尾随 JSON 空白允许浏览器直接创建对齐 Float32Array 视图。
        encoded += b' ' * (-(len(encoded) + _PREFIX.size) % 4)
        if len(encoded) > 64 * 1024:
            raise ValueError('pointcloud header too large')
        result = b''.join((_PREFIX.pack(_MAGIC, 2, len(encoded)), encoded, *(item.xyz_bytes for item in chunks)))
        self.next_index += 1
        self.encoded_bytes += len(result)
        self.encoded_xyz_bytes += header['payload_length']
        return result

    def accepts_ack(self, message):
        return (self.complete and message.get('type') == 'pointcloud.ack'
                and message.get('transfer_id') == self.transfer_id
                and message.get('base') == self.identity)


class _LatestWake:
    """在线程端先合并通知，不向事件循环堆积 callback 或持有历史快照。"""
    def __init__(self, loop):
        self.loop = loop
        self.event = asyncio.Event()
        self.lock = threading.Lock()
        self.pending = False
        self.closed = False

    def notify(self, _frame=None):
        with self.lock:
            if self.closed or self.pending:
                return
            self.pending = True
        try:
            self.loop.call_soon_threadsafe(self._deliver)
        except RuntimeError:
            self.close()

    def _deliver(self):
        with self.lock:
            self.pending = False
            if self.closed:
                return
        self.event.set()

    async def wait(self):
        await self.event.wait()
        self.event.clear()

    def close(self):
        with self.lock:
            self.closed = True
            self.pending = False


def install_pointcloud_stream(router, runtime):
    @router.websocket('/streams/pointcloud/ws')
    async def pointcloud_pages(websocket: WebSocket):
        # 此 router 与现有 WS 共用应用鉴权中间件，不另创设备/账号检查。
        await websocket.accept()
        await websocket.send_json({'type': 'pointcloud.ready', 'version': 2})
        wake = _LatestWake(asyncio.get_running_loop())
        subscription = runtime.stream_bus.subscribe(wake.notify, topics=('pointcloud_map.snapshot',))
        receive = asyncio.create_task(websocket.receive())
        wake_task = None
        base = None
        transfer: Optional[PointCloudTransfer] = None
        initialized = False
        serial = 0
        ready = True
        required_map_epoch = ''
        try:
            while True:
                if receive.done():
                    event = receive.result()
                    if event.get('type') == 'websocket.disconnect':
                        return
                    raw = event.get('text')
                    if not isinstance(raw, str) or len(raw) > 16384:
                        await websocket.close(code=4400, reason='invalid pointcloud control')
                        return
                    try:
                        control = json.loads(raw)
                        if not isinstance(control, dict):
                            raise TypeError('invalid pointcloud control')
                        if control.get('type') in ('pointcloud.resume', 'pointcloud.resync', 'pointcloud.map'):
                            requested_map = control.get('mapping_source_epoch', required_map_epoch)
                            if not isinstance(requested_map, str) or len(requested_map) > 4096:
                                raise ValueError('invalid map source identity')
                            required_map_epoch = requested_map
                        if control.get('type') == 'pointcloud.resume' and not initialized:
                            base = parse_client_base(control.get('base'))
                            initialized, ready = True, True
                        elif control.get('type') in ('pointcloud.resync', 'pointcloud.map'):
                            # 一次重建请求使旧事务失效；后续旧 ACK 不能推进新基线。
                            base, transfer, ready = None, None, True
                        elif transfer is not None and transfer.accepts_ack(control):
                            base, transfer, ready = transfer.identity, None, True
                    except (TypeError, ValueError, OverflowError):
                        await websocket.close(code=4400, reason='invalid pointcloud identity')
                        return
                    receive = asyncio.create_task(websocket.receive())
                if initialized and ready and transfer is None:
                    snapshot = runtime.get_pointcloud_map_snapshot()
                    ready = False
                    source_map = '' if snapshot is None else str(snapshot.provenance.get('mapping_source_epoch') or '')
                    map_matches = not required_map_epoch or not source_map or required_map_epoch == source_map
                    if snapshot is not None and map_matches and snapshot_identity(snapshot) != base:
                        serial += 1
                        transfer = PointCloudTransfer(snapshot, base, serial)
                if transfer is not None and not transfer.complete:
                    await websocket.send_bytes(transfer.next_packet())
                    # 即使 ASGI 立即返回，也在有界页面间让出事件循环。
                    await asyncio.sleep(0)
                    continue
                if wake_task is None:
                    wake_task = asyncio.create_task(wake.wait())
                done, _ = await asyncio.wait((receive, wake_task), timeout=30,
                                            return_when=asyncio.FIRST_COMPLETED)
                if wake_task in done:
                    wake_task = None
                    ready = True
                if not done:
                    await websocket.send_json({'type': 'pointcloud.keepalive'})
        except (WebSocketDisconnect, RuntimeError):
            return
        finally:
            wake.close()
            runtime.stream_bus.unsubscribe(subscription)
            transfer = None
            pending = [task for task in (receive, wake_task) if task is not None]
            for task in pending:
                task.cancel()
            if pending:
                await asyncio.gather(*pending, return_exceptions=True)
