from __future__ import annotations

import asyncio
import base64
import collections
from collections.abc import Mapping
from contextlib import asynccontextmanager
from dataclasses import asdict, is_dataclass
from datetime import date, datetime, timezone
from decimal import Decimal
from enum import Enum
import json
import logging
import math
import os
from pathlib import Path
import struct
import threading
import time
from typing import Any, Dict, Iterable, Optional, Tuple, Union
from uuid import UUID
from urllib.parse import quote

_logger = logging.getLogger(__name__)

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse, StreamingResponse
import numpy as np
from pydantic_core import to_json as _model_json_bytes
from starlette.websockets import WebSocket, WebSocketDisconnect

from contracts.streams import StreamFrame, StreamPayloadKind
from contracts.occupancy_cells import PackedOccupancyCells
from core.logging_utils import LogRateLimiter
from core.stream_coalescing import stream_replacement_key
from gateways.auth import GatewayAccessManager
from core.errors import GatewayError
from gateways.feedback import build_public_runtime_event
from gateways.agent.feedback import TERMINAL_STATES
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from gateways.platform import Platform
from gateways.ws.console_projection import console_payload, field_value, selected_fields
from gateways.ws.instance_encoding import InstanceSnapshotEncoder
from gateways.ws.pointcloud_encoding import CanonicalPointCloudEncoder
from gateways.ws.pointcloud_stream import install_pointcloud_stream

try:
    from services.geometry.video_shm_reader import VideoShmReader
except ImportError:
    VideoShmReader = None


_STREAM_QUEUE_MAXSIZE = 200
_DIAGNOSTIC_LOG_LIMITER = LogRateLimiter()
_DIAGNOSTIC_LOG_INTERVAL_SEC = 5.0
_STREAM_REPLAY_MAX_FRAMES = 200
_STREAM_JSON_MAX_OCCUPANCY_CELLS = 2_000_000
_POINTCLOUD_BINARY_MAGIC = b"NUWAXPC\x00"
_POINTCLOUD_BINARY_VERSION = 1
_POINTCLOUD_BINARY_PREFIX = struct.Struct("<8sBI")
_POINTCLOUD_BINARY_MAX_HEADER_BYTES = 64 * 1024
_MAX_U64 = (1 << 64) - 1
_POINTCLOUD_BINARY_TOPICS = frozenset(
    {"pointcloud_map.snapshot", "map.frame"}
)
_WEBSOCKET_CURSOR_STATUS_FIELDS = frozenset(
    {
        "requested_after_cursor",
        "effective_after_cursor",
        "oldest_cursor",
        "latest_cursor",
        "first_dropped_cursor",
        "last_dropped_cursor",
    }
)
_WEBSOCKET_CURSOR_MAP_FIELDS = frozenset(
    {"topic_oldest_cursors", "topic_evicted_through_cursors"}
)
_LATEST_ONLY_STREAM_TOPICS = frozenset(
    {
        "map.frame",
        "pointcloud_map.snapshot",
        "robot.state",  # 机器人状态只需要最新帧
        "pointcloud.frame",  # 点云也只需要最新帧
        "localization.pose",  # 定位姿态只需要最新帧
        "localization.frame_tree",  # 坐标树只需要最新帧
        "localization.trajectory",  # 轨迹只需要最新帧
        "map.heartbeat",  # 地图心跳只需要最新帧
        "memory.instances",  # 物品完整视图可替代，慢连接不积压逐次观察。
        "navigation.state",  # 仅共享判定认可的无目标守卫心跳，业务状态不合并。
    }
)
# 连接恢复总要携带当前状态，事件历史仍按游标续传；不在回放完成后另读新快照。
_REPLAY_SNAPSHOT_TOPICS = _LATEST_ONLY_STREAM_TOPICS | frozenset({
    "runtime.governance", "pointcloud_map.meta", "exploration.state", "system.metrics", "settings.display",
})
def _stream_frame_values(frame: StreamFrame) -> Dict[str, object]:
    model_fields = getattr(type(frame), "model_fields", {})
    values = {
        str(field_name): getattr(frame, field_name)
        for field_name in model_fields
        if field_name not in {"payload", "metadata"}
    }
    if frame.cursor is not None:
        values["cursor"] = _pointcloud_identity_integer(frame.cursor, "cursor")
    # 先保护信封中的状态与身份，载荷耗尽预算不能把 available 改成截断字符串。
    values["payload"] = (
        build_public_runtime_event(frame.payload)
        if frame.topic == "tasks.state"
        else console_payload(frame.topic, frame.payload)
    )
    values["metadata"] = frame.metadata
    return values


def _strict_json_dumps(value: object, *, compact: bool = False) -> str:
    """完整编码网页载荷，不建立独立结果投影。"""
    return _stream_json_dumps(value)


def _stream_json_default(value):
    """规范实时载荷直接使用模型序列化，不逐字段重复做预算、反射与字符串编码。"""
    if isinstance(value, StreamFrame):
        return _stream_frame_values(value)
    if is_dataclass(value):
        return asdict(value)
    if isinstance(value, PackedOccupancyCells):
        return value.to_list()
    if isinstance(value, (bytes, bytearray, memoryview)):
        return {"$type": "bytes", "encoding": "base64", "data": base64.b64encode(value).decode("ascii")}
    if hasattr(value, "model_dump"):
        return value.model_dump(mode="json")
    if isinstance(value, (datetime, date)):
        return value.isoformat()
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, (Path, UUID, Decimal)):
        return str(value)
    raise TypeError(type(value).__name__)


class _DiagnosticJSONEncoder(json.JSONEncoder):
    """非有限诊断值沿用旧字符串表示，完整保留消息字段。"""

    def iterencode(self, value, _one_shot=False):
        def float_value(number):
            if math.isnan(number):
                return '"NaN"'
            if math.isinf(number):
                return '"Infinity"' if number > 0 else '"-Infinity"'
            return float.__repr__(number)
        encoder = json.encoder.encode_basestring_ascii if self.ensure_ascii else json.encoder.encode_basestring
        encode = json.encoder._make_iterencode({} if self.check_circular else None,
            self.default, encoder, self.indent, float_value, self.key_separator,
            self.item_separator, self.sort_keys, self.skipkeys, False)
        return encode(value, 0)


def _stream_json_dumps(value, *, ensure_ascii: bool = False) -> str:
    try:
        return json.dumps(value, default=_stream_json_default, ensure_ascii=ensure_ascii,
                          allow_nan=False, separators=(",", ":"))
    except ValueError:
        # 正常消息保留原生快速编码，诊断消息仅转换非有限数，不恢复旧预算投影。
        return json.dumps(value, default=_stream_json_default, ensure_ascii=ensure_ascii,
            allow_nan=False, separators=(",", ":"), cls=_DiagnosticJSONEncoder)


async def _send_ws_json(websocket: WebSocket, payload: object) -> None:
    await websocket.send_text(_stream_json_dumps(payload))


def _parse_topics(values: Optional[str]) -> Optional[Tuple[str, ...]]:
    if not values:
        return None
    topics = tuple(item.strip() for item in values.split(",") if item.strip())
    return topics if topics else None


def _normalize_after_cursor(after_cursor: int, latest_cursor: int) -> int:
    if after_cursor <= 0:
        return 0
    if after_cursor > latest_cursor:
        return 0
    return after_cursor


def _build_replay_batch(
    runtime: Platform,
    *,
    after_cursor: int,
    topics: Optional[Iterable[str]],
    limit: int = _STREAM_REPLAY_MAX_FRAMES,
    snapshot=None,
) -> Tuple[Dict[str, Any], Tuple[StreamFrame, ...]]:
    topic_filter = tuple(str(topic) for topic in topics) if topics is not None else None
    if snapshot is None:
        snapshot = runtime.streams.stream_bus.replay_snapshot(
            after_cursor=after_cursor,
            topics=topic_filter,
        )
    latest_cursor = int(snapshot.latest_cursor)
    oldest_cursor = int(snapshot.oldest_cursor)
    retained_frames = tuple(snapshot.frames)
    topic_oldest_cursors = dict(snapshot.topic_oldest_cursors)
    topic_evicted_through_cursors = dict(snapshot.topic_evicted_through_cursors)
    requested_after_cursor = max(0, int(after_cursor or 0))
    normalized_after_cursor = _normalize_after_cursor(requested_after_cursor, latest_cursor)
    gap_reason: Optional[str] = None
    if requested_after_cursor > 0 and requested_after_cursor > latest_cursor:
        gap_reason = "cursor_ahead_of_stream"
    elif requested_after_cursor > 0 and any(
        requested_after_cursor < evicted_through_cursor
        for evicted_through_cursor in topic_evicted_through_cursors.values()
    ):
        gap_reason = "cursor_expired"

    latest_states = tuple(
        frame for frame in snapshot.latest_frames
        if frame.topic in _REPLAY_SNAPSHOT_TOPICS
        and (topic_filter is None or frame.topic in topic_filter)
    )
    frames = _coalesce_latest_topic_frames(tuple(sorted(
        {frame.cursor: frame for frame in (*retained_frames, *latest_states)}.values(),
        key=lambda frame: frame.cursor or 0,
    )))

    frame_limit = max(1, int(limit))
    truncated = len(frames) > frame_limit
    if truncated:
        if requested_after_cursor <= 0 or requested_after_cursor > latest_cursor:
            # 首次订阅先保留各状态主题，日志填充剩余容量；历史仍可用日志接口继续翻页。
            states = [frame for frame in frames if frame.topic != "logs.entry"][-frame_limit:]
            remaining = frame_limit - len(states)
            logs = [frame for frame in frames if frame.topic == "logs.entry"][-remaining:] if remaining else []
            frames = tuple(sorted([*states, *logs], key=lambda frame: frame.cursor or 0))
        else:
            frames = frames[-frame_limit:]

    frames = tuple(
        frame
        for frame in frames
        if frame.cursor is None or frame.cursor <= latest_cursor
    )
    status: Dict[str, Any] = {
        "requested_after_cursor": requested_after_cursor,
        "effective_after_cursor": normalized_after_cursor,
        "oldest_cursor": oldest_cursor,
        "topic_oldest_cursors": topic_oldest_cursors,
        "topic_evicted_through_cursors": topic_evicted_through_cursors,
        "latest_cursor": latest_cursor,
        "replay_gap": gap_reason is not None,
        "gap_reason": gap_reason,
        "resync_required": gap_reason is not None,
        "truncated": truncated,
        "frame_count": len(frames),
    }
    return status, frames


def _coalesce_latest_topic_frames(
    frames: Iterable[StreamFrame],
) -> Tuple[StreamFrame, ...]:
    frame_list = tuple(frames)
    keys = tuple(stream_replacement_key(frame, _LATEST_ONLY_STREAM_TOPICS) if frame.topic in _LATEST_ONLY_STREAM_TOPICS else None for frame in frame_list)
    latest_index_by_topic = {key: index for index, key in enumerate(keys) if key is not None}
    return tuple(
        frame
        for index, frame in enumerate(frame_list)
        if keys[index] is None or latest_index_by_topic.get(keys[index]) == index
    )


async def _run_blocking(callable_obj, *args, **kwargs):
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(None, lambda: callable_obj(*args, **kwargs))


def _frame_event_type(frame: StreamFrame) -> str:
    payload = getattr(frame, "payload", None)
    if isinstance(payload, dict):
        return str(payload.get("event_type") or payload.get("runtime_event_type") or "")
    return str(getattr(payload, "event_type", "") or "")


def _frame_priority(frame: StreamFrame) -> int:
    event_type = _frame_event_type(frame)
    payload = frame.payload
    status = field_value(payload, "payload", payload)
    if frame.topic == "tasks.state" and field_value(status, "state") in TERMINAL_STATES:
        return 100
    if frame.topic == "logs.entry":
        payload = frame.payload if isinstance(frame.payload, dict) else {}
        severity = str(payload.get("severity") or payload.get("level") or "").lower()
        return 80 if severity in {"error", "critical"} else 20
    if frame.topic == "tasks.state":
        return 60
    if frame.payload_kind == StreamPayloadKind.DIAGNOSTIC:
        return 70
    if frame.payload_kind == StreamPayloadKind.BINARY:
        return 5
    return 40


class _ClientStreamGap(Exception):
    """只有丢帧通知而没有数据帧时唤醒发送循环。"""


class _ClientStreamClosed(Exception):
    """客户端队列已关闭，不再接收旧订阅回调。"""


class _ClientStreamQueue:
    """单客户端发送队列：按槽位替换最新帧，按优先级分桶丢弃。

    替代原来的 `asyncio.Queue`。原实现每收到一帧 latest-only/BINARY 帧就要排空
    整个队列过滤同主题旧帧再逐项塞回，队列满时再排空一次做全扫描找最低优先级项，
    容量 200 时点云和视频每帧都是 O(n)。这里改为：

    - 交付顺序仍是严格 FIFO，优先级只用于选择丢弃对象，不改变发送次序；
    - latest-only/BINARY 帧按槽位键直接覆盖，覆盖后移到队尾（与原实现一致）；
    - 每个优先级维护一个插入有序的键索引，满时取最低优先级里最早的一项，
      优先级取值只有 7 种，因此选择丢弃对象是 O(1)。
    """

    def __init__(self, maxsize: int, *, preserve_topics: Iterable[str] = ()) -> None:
        self._maxsize = int(maxsize)
        self._preserve_topics = frozenset(preserve_topics)
        # 槽位键 -> (帧, 优先级)，插入序即交付序。
        self._items = collections.OrderedDict()
        # 优先级 -> 该优先级下的槽位键（插入有序），用于 O(1) 选丢弃对象。
        self._keys_by_priority = {}
        self._sequence = 0
        self._waiter = None
        self._lock = threading.RLock()
        self._notification_pending = False
        self._closed = False
        self._nuwax_drop_state = None

    def _slot_key(self, frame: StreamFrame) -> Optional[Tuple[str, str]]:
        """返回可被后续同类帧覆盖的槽位键；None 表示该帧独占一个位置。

        与原过滤条件等价：latest-only 主题按主题整体收敛；其余主题只有 BINARY
        帧之间互相收敛，同主题的非 BINARY 帧不受影响。
        """

        if frame.topic in self._preserve_topics:
            return None
        return stream_replacement_key(frame, _LATEST_ONLY_STREAM_TOPICS)

    def _discard_key(self, key: object, priority: int) -> None:
        self._items.pop(key, None)
        bucket = self._keys_by_priority.get(priority)
        if bucket is not None:
            bucket.pop(key, None)
            if not bucket:
                del self._keys_by_priority[priority]

    def _lowest_priority_key(self) -> Tuple[object, int]:
        priority = min(self._keys_by_priority)
        return next(iter(self._keys_by_priority[priority])), priority

    def put(self, frame: StreamFrame, runtime: Platform) -> None:
        with self._lock:
            if self._closed:
                return
            self._put_locked(frame, runtime)
        self._wake_waiter()

    def put_threadsafe(self, loop: asyncio.AbstractEventLoop, frame: StreamFrame, runtime: Platform) -> None:
        """先在有界槽位中合并，再向事件循环合并发送一次唤醒。"""
        with self._lock:
            if self._closed:
                return
            self._put_locked(frame, runtime)
        self._schedule_wakeup(loop)

    def note_gap_threadsafe(self, loop: asyncio.AbstractEventLoop, notice: Dict[str, Any]) -> None:
        with self._lock:
            if self._closed:
                return
            _merge_queue_gap(self, notice)
        self._schedule_wakeup(loop)

    def _schedule_wakeup(self, loop: asyncio.AbstractEventLoop) -> None:
        with self._lock:
            if self._closed or self._notification_pending:
                return
            self._notification_pending = True
        try:
            loop.call_soon_threadsafe(self._wake_waiter)
        except RuntimeError:
            # 事件循环已关闭；不保留断开连接的帧或反复向旧循环调度。
            with self._lock:
                self._closed = True
                self._items.clear()
                self._keys_by_priority.clear()
                self._waiter = None
                self._notification_pending = False

    def _wake_waiter(self) -> None:
        with self._lock:
            self._notification_pending = False
            waiter = self._waiter
        if waiter is not None and not waiter.done():
            waiter.set_result(None)

    def _put_locked(self, frame: StreamFrame, runtime: Platform) -> None:
        priority = _frame_priority(frame)
        key = self._slot_key(frame)
        if key is None:
            self._sequence += 1
            key = ("seq", self._sequence)
        else:
            existing = self._items.get(key)
            if existing is not None:
                # 被新帧取代的旧帧不是拥塞丢弃，不计入 gap 通知。
                self._discard_key(key, existing[1])

        if len(self._items) >= self._maxsize:
            drop_key, drop_priority = self._lowest_priority_key()
            if drop_priority > priority:
                # 队列里全是更高优先级的帧，改为丢弃新到帧。
                _record_queue_drop(self, frame)
                return
            _record_queue_drop(self, self._items[drop_key][0])
            self._discard_key(drop_key, drop_priority)

        self._items[key] = (frame, priority)
        self._keys_by_priority.setdefault(priority, collections.OrderedDict())[key] = None

    def empty(self) -> bool:
        with self._lock:
            return not self._items

    def get_nowait(self) -> StreamFrame:
        with self._lock:
            if not self._items:
                raise asyncio.QueueEmpty
            key, (frame, priority) = self._items.popitem(last=False)
            self._discard_key(key, priority)
            return frame

    async def get(self) -> StreamFrame:
        while True:
            with self._lock:
                if self._items:
                    return self.get_nowait()
                if self._closed:
                    raise _ClientStreamClosed()
                if self._nuwax_drop_state:
                    raise _ClientStreamGap()
                waiter = asyncio.get_running_loop().create_future()
                self._waiter = waiter
            try:
                await waiter
            finally:
                with self._lock:
                    if self._waiter is waiter:
                        self._waiter = None

    def close(self) -> None:
        with self._lock:
            self._closed = True
            self._items.clear()
            self._keys_by_priority.clear()
            self._nuwax_drop_state = None
        self._wake_waiter()


def _merge_queue_gap(queue: _ClientStreamQueue, notice: Dict[str, Any]) -> None:
    state = getattr(queue, "_nuwax_drop_state", None)
    if not isinstance(state, dict):
        state = {
            "dropped_count": 0,
            "first_dropped_cursor": None,
            "last_dropped_cursor": None,
            "topics": set(),
            "reasons": set(),
        }
        setattr(queue, "_nuwax_drop_state", state)
    state["dropped_count"] += int(notice["dropped_count"])
    for name, pick in (("first_dropped_cursor", min), ("last_dropped_cursor", max)):
        cursor = notice.get(name)
        if cursor is not None:
            state[name] = cursor if state[name] is None else pick(state[name], cursor)
    state["topics"].update(notice.get("topics") or ())
    state["reasons"].add(notice.get("reason", "client_queue_overflow"))


def _record_queue_drop(queue: _ClientStreamQueue, frame: StreamFrame) -> None:
    _merge_queue_gap(queue, {
        "reason": "client_queue_overflow", "dropped_count": 1,
        "first_dropped_cursor": frame.cursor, "last_dropped_cursor": frame.cursor,
        "topics": (frame.topic,),
    })


def _take_queue_gap_notice(queue: _ClientStreamQueue, runtime: Platform) -> Optional[Dict[str, Any]]:
    with queue._lock:
        state = getattr(queue, "_nuwax_drop_state", None)
        setattr(queue, "_nuwax_drop_state", None)
    if not isinstance(state, dict) or not state.get("dropped_count"):
        return None
    notice = {
        "reason": next(iter(state["reasons"])) if len(state["reasons"]) == 1 else "multiple_queue_overflows",
        "dropped_count": int(state["dropped_count"]),
        "first_dropped_cursor": state.get("first_dropped_cursor"),
        "last_dropped_cursor": state.get("last_dropped_cursor"),
        "topics": sorted(state.get("topics") or ()),
        "latest_cursor": runtime.streams.stream_bus.latest_cursor(),
        "resync_required": True,
        "next_action": "重新获取任务快照；日志等增量数据从最新游标继续。",
    }
    return notice


def _log_frame_delay(frame: StreamFrame, *, stage: str, threshold_ms: float) -> None:
    """传输延迟按阶段周期汇总，保留文件诊断而不回注同一实时流。"""

    if hasattr(frame, 'timestamp') and frame.timestamp:
        delay_ms = (datetime.now(timezone.utc) - frame.timestamp).total_seconds() * 1000
        if delay_ms > threshold_ms:
            _DIAGNOSTIC_LOG_LIMITER.log(
                _logger,
                logging.WARNING,
                "[PERF] %s延迟过高: %.1fms topic=%s cursor=%s",
                stage,
                delay_ms,
                frame.topic,
                getattr(frame, 'cursor', None),
                key=stage,
                interval_sec=_DIAGNOSTIC_LOG_INTERVAL_SEC,
                extra={"stream_bus_internal": True},
            )


def _safe_enqueue(
    loop: asyncio.AbstractEventLoop,
    queue: _ClientStreamQueue,
    frame: StreamFrame,
    runtime: Platform,
) -> None:
    _log_frame_delay(frame, stage="入队", threshold_ms=50)

    queue.put_threadsafe(loop, frame, runtime)


def _pointcloud_identity_integer(value: object, field_name: str) -> str:
    if isinstance(value, bool):
        raise ValueError("%s 必须是 0..2^64-1 的整数。" % field_name)
    if isinstance(value, int):
        parsed = value
    elif (
        isinstance(value, str)
        and value.isascii()
        and (value == "0" or (value[:1] in "123456789" and value.isdigit()))
    ):
        parsed = int(value)
    else:
        raise ValueError("%s 必须是规范十进制 u64 字符串或整数。" % field_name)
    if parsed < 0 or parsed > _MAX_U64:
        raise ValueError("%s 必须位于 0..2^64-1。" % field_name)
    return str(parsed)


def _external_cursor_status(payload: Mapping[str, Any]) -> Dict[str, Any]:
    result = dict(payload)
    for field_name in _WEBSOCKET_CURSOR_STATUS_FIELDS:
        value = result.get(field_name)
        if value is not None:
            result[field_name] = _pointcloud_identity_integer(value, field_name)
    for field_name in _WEBSOCKET_CURSOR_MAP_FIELDS:
        values = result.get(field_name)
        if isinstance(values, Mapping):
            result[field_name] = {
                str(topic): _pointcloud_identity_integer(cursor, field_name)
                for topic, cursor in values.items()
            }
    return result


def _encode_pointcloud_binary_frame(frame: StreamFrame) -> bytes:
    """把统一点云 XYZ 编码为版本化、自描述的 WebSocket 信封。"""

    if frame.topic not in _POINTCLOUD_BINARY_TOPICS:
        raise ValueError("不是受支持的点云二进制主题。")
    if not isinstance(frame.payload, (bytes, bytearray)):
        raise ValueError("点云二进制载荷必须是 bytes。")
    metadata = dict(frame.metadata or {})
    xyz_bytes = bytes(frame.payload)
    source = str(metadata.get("source") or "")
    source_epoch = str(metadata.get("source_epoch") or "")
    revision = str(metadata.get("revision") or frame.revision or "")
    frame_id = str(metadata.get("frame_id") or frame.frame_id or "")
    point_format = str(metadata.get("format") or "")
    if source not in {"live", "saved"}:
        raise ValueError("点云 source 必须是 live 或 saved。")
    if not source_epoch or not revision or not frame_id:
        raise ValueError("点云信封身份字段不能为空。")
    if point_format != "float32-xyz-le":
        raise ValueError("点云信封仅支持 float32-xyz-le。")
    point_count_text = _pointcloud_identity_integer(metadata.get("point_count"), "point_count")
    point_count = int(point_count_text)
    if len(xyz_bytes) != point_count * 12:
        raise ValueError("点云 XYZ 长度与 point_count 不一致。")
    # 只读小端视图批量校验全部坐标，避免逐点解释执行挤占惯性读取线程。
    xyz = np.frombuffer(xyz_bytes, dtype="<f4")
    if not np.isfinite(xyz).all():
        raise ValueError("点云 XYZ 坐标必须全部为有限 float32。")

    header = {
        "topic": str(frame.topic),
        "cursor": _pointcloud_identity_integer(frame.cursor, "cursor"),
        "source": source,
        "source_epoch": source_epoch,
        "stream_generation": _pointcloud_identity_integer(
            metadata.get("stream_generation"),
            "stream_generation",
        ),
        "revision": revision,
        "revision_ordinal": _pointcloud_identity_integer(
            metadata.get("revision_ordinal"),
            "revision_ordinal",
        ),
        "frame_id": frame_id,
        "source_timestamp_ns": _pointcloud_identity_integer(
            metadata.get("source_timestamp_ns"),
            "source_timestamp_ns",
        ),
        "point_count": point_count,
        "format": point_format,
        "payload_length": len(xyz_bytes),
    }
    header_bytes = json.dumps(
        header,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    if len(header_bytes) > _POINTCLOUD_BINARY_MAX_HEADER_BYTES:
        raise ValueError("点云信封 JSON header 超过上限。")
    return (
        _POINTCLOUD_BINARY_PREFIX.pack(
            _POINTCLOUD_BINARY_MAGIC,
            _POINTCLOUD_BINARY_VERSION,
            len(header_bytes),
        )
        + header_bytes
        + xyz_bytes
    )


_MAP_BINARY_MAGIC = b"NUWAXMAP"
_MAP_BINARY_VERSION = 4


def _encode_map_binary_frame(frame: StreamFrame) -> bytes:
    """版本4直接编码新地图流：栅格元数据、有效栅格、物理栅格。"""
    update = frame.payload
    grid = asdict(update.grid)
    effective = grid.pop("data")
    payload = {"grid": grid, "raw_data_length": len(update.raw_data)}
    header_frame = frame.model_copy(update={"payload": payload})
    header = _stream_json_dumps({"type": "stream.frame", "frame": header_frame}).encode("utf-8")
    return (struct.pack("<8sBI", _MAP_BINARY_MAGIC, _MAP_BINARY_VERSION, len(header))
            + header + effective + update.raw_data)


def _encode_ws_frame(frame: StreamFrame, *, map_binary_version=4) -> Union[str, bytes]:
    """回放与实时发送使用同一编码；大载荷的降级也留在编码线程内。"""
    if frame.payload_kind == StreamPayloadKind.BINARY and isinstance(frame.payload, (bytes, bytearray)):
        if frame.topic not in _POINTCLOUD_BINARY_TOPICS:
            return bytes(frame.payload)
        try:
            return _encode_pointcloud_binary_frame(frame)
        except (TypeError, ValueError) as exc:
            return _strict_json_dumps({
                "type": "stream.binary_rejected", "topic": frame.topic,
                "cursor": _pointcloud_identity_integer(frame.cursor, "cursor"),
                "reason": str(exc),
            }, compact=True)
    if frame.topic == "map.frame" and frame.payload is not None and map_binary_version == 4:
        return _encode_map_binary_frame(frame)
    if frame.topic in {"localization.pose", "localization.trajectory", "localization.frame_tree"}:
        # 规范定位对象由原生序列化器直接编码，避免每个轨迹点先转字典再走 Python JSON。
        # 外部诊断和非有限值继续走原有严格投影，不改变其错误显示语义。
        try:
            values = _stream_frame_values(frame)
            values["timestamp"] = frame.timestamp.isoformat()
            encoded = _model_json_bytes({"type": "stream.frame", "frame": values},
                                        fallback=_stream_json_default, inf_nan_mode="constants")
            # 模型默认会将非有限向量写为 null；沿原严格诊断显示原值，不能伪装成缺失坐标。
            if not any(marker in encoded for marker in
                       (b"NaN", b"Infinity", b'"x":null', b'"y":null', b'"z":null', b'"w":null')):
                return encoded.decode("utf-8")
        except (TypeError, ValueError):
            pass
    return _stream_json_dumps({"type": "stream.frame", "frame": frame})


def _pointcloud_snapshot_frame(frame, snapshot):
    """为快照通道生成完整载荷。"""
    if snapshot.provenance.get("assigned_source") is not None:
        # 新保存来源只走有界页面；不能在旧适配器重新拼接整图或创建摘要别名。
        return frame.model_copy(update={"payload_kind": StreamPayloadKind.JSON,
            "status": "unavailable", "payload": {"reason": "assigned_saved_requires_page_stream",
                "stream_path": "/streams/pointcloud/ws"},
            "metadata": {"source": "saved", "assigned_source": snapshot.provenance["assigned_source"]}})
    metadata = {**frame.metadata, "source": snapshot.source, "source_epoch": snapshot.source_epoch,
                "revision": snapshot.revision, "revision_ordinal": snapshot.revision_ordinal,
                "frame_id": snapshot.frame_id, "source_timestamp_ns": snapshot.source_timestamp_ns,
                "point_count": snapshot.point_count, "format": snapshot.format,
                "stream_generation": snapshot.stream_generation}
    raw = b"".join(chunk.xyz_bytes for chunk in snapshot.chunks) if getattr(snapshot, "chunked", False) else snapshot.xyz_bytes
    return frame.model_copy(update={"payload_kind": StreamPayloadKind.BINARY, "payload": raw,
                                    "metadata": metadata, "frame_id": snapshot.frame_id,
                                    "revision": snapshot.revision})


async def _send_ws_frame(websocket: WebSocket, frame: StreamFrame, *, pointcloud_encoder=None, map_binary_version=None, runtime=None, instance_encoder=None) -> None:
    if frame.topic == "pointcloud_map.snapshot" and frame.metadata.get("chunked"):
        snapshot = runtime.streams.get_pointcloud_map_snapshot() if runtime is not None else None
        if snapshot is None:
            return
        frame = await _run_blocking(_pointcloud_snapshot_frame, frame, snapshot)
    # 地图复制、完整栅格校验和点云编码不能占用所有连接共享的事件循环。
    # 轻量位姿直接编码，避免在工作线程繁忙时跟随地图排队。
    if (pointcloud_encoder is not None and frame.topic == "pointcloud_map.snapshot"
            and frame.payload_kind == StreamPayloadKind.BINARY):
        payload = await pointcloud_encoder.encode(frame)
    elif frame.topic == "map.frame":
        payload = await _run_blocking(_encode_ws_frame, frame, map_binary_version=map_binary_version)
    elif frame.payload_kind == StreamPayloadKind.BINARY:
        payload = await _run_blocking(_encode_ws_frame, frame)
    else:
        payload = instance_encoder.encode(frame) if instance_encoder is not None else _encode_ws_frame(frame)
    if isinstance(payload, bytes):
        await websocket.send_bytes(payload)
    else:
        await websocket.send_text(payload)


async def _format_sse_frame(frame: StreamFrame, *, runtime=None) -> str:
    if runtime is not None and frame.topic == "pointcloud_map.snapshot" and frame.metadata.get("chunked"):
        snapshot = runtime.streams.get_pointcloud_map_snapshot()
        if snapshot is not None:
            frame = await _run_blocking(_pointcloud_snapshot_frame, frame, snapshot)
    if frame.topic == "map.frame" or frame.payload_kind == StreamPayloadKind.BINARY:
        return await _run_blocking(_format_sse_payload, frame)
    return _format_sse_payload(frame)


def build_data_stream_router(runtime: Platform, access_manager: GatewayAccessManager) -> APIRouter:
    """构造实时数据流路由。"""

    pointcloud_encoder = None
    instance_encoder = None

    @asynccontextmanager
    async def lifespan(_app):
        nonlocal pointcloud_encoder, instance_encoder
        owner = CanonicalPointCloudEncoder(_encode_ws_frame, _run_blocking)
        instances = InstanceSnapshotEncoder(_encode_ws_frame)
        pointcloud_encoder, instance_encoder = owner, instances
        try:
            yield
        finally:
            pointcloud_encoder = instance_encoder = None
            instances.close()
            await owner.close()

    router = APIRouter(lifespan=lifespan)
    install_pointcloud_stream(router, runtime)

    @router.get("/streams/sse")
    async def sse_streams(
        request: Request,
        topics: Optional[str] = None,
        after_cursor: int = 0,
        max_frames: Optional[int] = None,
    ) -> StreamingResponse:
        topic_filter = _parse_topics(topics)
        resume_after_cursor = int(after_cursor or 0)
        if resume_after_cursor <= 0:
            resume_after_cursor = _positive_int(request.headers.get("last-event-id"), 0)

        async def frame_generator():
            loop = asyncio.get_running_loop()
            queue = _ClientStreamQueue(maxsize=_STREAM_QUEUE_MAXSIZE)
            emitted = 0
            subscription_id, replay_snapshot = runtime.streams.stream_bus.subscribe_with_replay(
                lambda frame: _safe_enqueue(loop, queue, frame, runtime),
                after_cursor=resume_after_cursor,
                topics=topic_filter,
                async_dispatch=True,
                latest_only_topics=_LATEST_ONLY_STREAM_TOPICS,
                on_gap=lambda notice: queue.note_gap_threadsafe(loop, notice),
            )
            try:
                replay_status, replay_frames = _build_replay_batch(
                    runtime,
                    after_cursor=resume_after_cursor,
                    topics=topic_filter,
                    snapshot=replay_snapshot,
                )
                replay_latest_cursor = replay_status["latest_cursor"]
                for frame in replay_frames:
                    yield await _format_sse_frame(frame, runtime=runtime)
                    emitted += 1
                    if max_frames is not None and emitted >= max_frames:
                        return
                yield _format_sse("stream.replay", _external_cursor_status(replay_status))
                while True:
                    if await request.is_disconnected():
                        break
                    try:
                        frame = await asyncio.wait_for(queue.get(), timeout=runtime.config.gateway.sse_keepalive_sec)

                        _log_frame_delay(frame, stage="SSE发送", threshold_ms=100)

                        if frame.cursor is not None and frame.cursor <= replay_latest_cursor:
                            continue
                        gap_notice = _take_queue_gap_notice(queue, runtime)
                        if gap_notice is not None:
                            yield _format_sse(
                                "stream.gap",
                                _external_cursor_status(gap_notice),
                            )
                        yield await _format_sse_frame(frame, runtime=runtime)
                        emitted += 1
                        if max_frames is not None and emitted >= max_frames:
                            return
                    except _ClientStreamGap:
                        gap_notice = _take_queue_gap_notice(queue, runtime)
                        if gap_notice is not None:
                            yield _format_sse("stream.gap", _external_cursor_status(gap_notice))
                    except _ClientStreamClosed:
                        return
                    except asyncio.TimeoutError:
                        yield _format_sse_keepalive(runtime.streams.stream_bus.latest_cursor())
            finally:
                queue.close()
                runtime.streams.stream_bus.unsubscribe(subscription_id)

        return StreamingResponse(frame_generator(), media_type="text/event-stream")

    @router.get("/streams/video.mjpeg")
    async def video_mjpeg_stream(
        request: Request,
        camera_id: Optional[str] = None,
        fps: Optional[float] = None,
    ):
        source_status = runtime.perception.video_frame_source_status()
        source_camera_id = str(source_status.get("camera_id") or "front_camera")
        if camera_id is not None and camera_id != source_camera_id:
            return JSONResponse(status_code=404, content={
                "error": "video_camera_unavailable",
                "message": "当前视频源未绑定请求的相机。",
                "camera_id": camera_id,
                "available_camera_id": source_camera_id,
            })
        # 首帧及写者重启由共享读取拥有方等待，瞬时空发布头不结束网页视频连接。
        # 默认随输入到达交付；显式请求的帧率仅作为发送上限。
        requested_fps = float(fps) if fps is not None else 0.0
        if not math.isfinite(requested_fps) or requested_fps < 0:
            requested_fps = 0.0
        frame_interval_sec = 1.0 / min(max(requested_fps, 1.0), 60.0) if requested_fps else 0.0

        async def frame_generator():
            from services.perception.video_demand import video_consumer
            with video_consumer():
                shm_path = "/tmp/nuwax_video.shm"
                attached = _SHARED_MJPEG_SOURCE.acquire(shm_path, camera_id=source_camera_id)
                last_sent_sequence = -1
                next_frame_at = 0.0
                try:
                    if not attached:
                        return
                    while True:
                        multipart, sequence = await _SHARED_MJPEG_SOURCE.wait_for_frame(last_sent_sequence)
                        if multipart is None:
                            return
                        remaining = next_frame_at - time.monotonic()
                        if remaining > 0:
                            await asyncio.sleep(remaining)
                            # 等待发送限额时只领取最新帧，不补发已经过时的候选。
                            multipart, sequence = _SHARED_MJPEG_SOURCE.latest_multipart()
                        last_sent_sequence = sequence
                        next_frame_at = time.monotonic() + frame_interval_sec
                        yield multipart
                finally:
                    if attached:
                        # 读者退出由拥有方等待，不能阻塞网页事件循环。
                        await _run_blocking(_SHARED_MJPEG_SOURCE.release)

        return StreamingResponse(
            frame_generator(),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={"Cache-Control": "no-store"},
        )

    @router.websocket("/streams/ws")
    async def websocket_streams(websocket: WebSocket) -> None:
        try:
            topic_filter = _parse_topics(websocket.query_params.get("topics"))
            after_cursor = int(websocket.query_params.get("after_cursor", "0"))
            map_binary_version = 4 if websocket.query_params.get("map_binary_version") == "4" else None
        except Exception as exc:
            error = exc if isinstance(exc, GatewayError) else GatewayError(f"数据流订阅初始化失败: {exc}")
            await websocket.close(code=4403, reason=error.message)
            return

        await websocket.accept()
        loop = asyncio.get_running_loop()
        queue = _ClientStreamQueue(
            maxsize=_STREAM_QUEUE_MAXSIZE,
        )
        subscription_id: Optional[str] = None
        receive_task = None
        frame_task = None
        pointcloud_detached = websocket.query_params.get("pointcloud_transport") == "pages"

        try:
            subscription_id, replay_snapshot = runtime.streams.stream_bus.subscribe_with_replay(
                lambda frame: _safe_enqueue(loop, queue, frame, runtime),
                # 身份通道只有可替代状态，每次连接从同一原子边界重建地图与位姿。
                after_cursor=after_cursor,
                topics=topic_filter,
                async_dispatch=True,
                latest_only_topics=_LATEST_ONLY_STREAM_TOPICS,
                on_gap=lambda notice: queue.note_gap_threadsafe(loop, notice),
            )
            replay_status, replay_frames = _build_replay_batch(
                runtime,
                after_cursor=after_cursor,
                topics=topic_filter,
                snapshot=replay_snapshot,
            )
            replay_latest_cursor = replay_status["latest_cursor"]
            for frame in replay_frames:
                if pointcloud_detached and frame.topic == "pointcloud_map.snapshot":
                    continue
                if getattr(frame, 'payload_kind', None) == StreamPayloadKind.BINARY:
                    continue  # binary 帧不回放
                await _send_ws_frame(websocket, frame, pointcloud_encoder=pointcloud_encoder, map_binary_version=map_binary_version, runtime=runtime, instance_encoder=instance_encoder)
            await _send_ws_json(
                websocket,
                {"type": "stream.replay", **_external_cursor_status(replay_status)},
            )
            receive_task = asyncio.create_task(websocket.receive())
            while True:
                try:
                    if frame_task is None:
                        frame_task = asyncio.create_task(queue.get())
                    done, _ = await asyncio.wait(
                        (frame_task, receive_task),
                        timeout=runtime.config.gateway.ws_ping_interval_sec,
                        return_when=asyncio.FIRST_COMPLETED,
                    )
                    if not done:
                        raise asyncio.TimeoutError
                    if receive_task in done:
                        message = receive_task.result()
                        if message.get("type") == "websocket.disconnect":
                            return
                        # 往返探针只沿当前发送拥有方应答，不并发写套接字或挤入业务游标。
                        raw = message.get("text")
                        if isinstance(raw, str) and len(raw) <= 256:
                            try:
                                probe = json.loads(raw)
                            except (ValueError, TypeError):
                                probe = None
                            if isinstance(probe, dict) and probe.get("type") == "pointcloud.detach":
                                pointcloud_detached = True
                            if isinstance(probe, dict) and probe.get("type") == "stream.ping":
                                request_id = probe.get("request_id")
                                if isinstance(request_id, str) and 0 < len(request_id) <= 64:
                                    await _send_ws_json(websocket, {"type": "stream.pong", "request_id": request_id,
                                                                   "server_timestamp_ns": str(time.time_ns())})
                        receive_task = asyncio.create_task(websocket.receive())
                    if frame_task not in done:
                        continue
                    current_frame_task, frame_task = frame_task, None
                    frame = current_frame_task.result()

                    _log_frame_delay(frame, stage="WS发送", threshold_ms=100)

                    if frame.cursor is not None and frame.cursor <= replay_latest_cursor:
                        continue
                    gap_notice = _take_queue_gap_notice(queue, runtime)
                    if gap_notice is not None:
                        await _send_ws_json(
                            websocket,
                            {"type": "stream.gap", **_external_cursor_status(gap_notice)},
                        )
                    if pointcloud_detached and frame.topic == "pointcloud_map.snapshot":
                        continue
                    await _send_ws_frame(websocket, frame, pointcloud_encoder=pointcloud_encoder, map_binary_version=map_binary_version, runtime=runtime, instance_encoder=instance_encoder)
                except _ClientStreamGap:
                    gap_notice = _take_queue_gap_notice(queue, runtime)
                    if gap_notice is not None:
                        await _send_ws_json(websocket, {"type": "stream.gap", **_external_cursor_status(gap_notice)})
                except _ClientStreamClosed:
                    return
                except asyncio.TimeoutError:
                    await _send_ws_json(
                        websocket,
                        {
                            "type": "keepalive",
                            "cursor": _pointcloud_identity_integer(
                                runtime.streams.stream_bus.latest_cursor(),
                                "cursor",
                            ),
                        },
                    )
        except WebSocketDisconnect:
            return
        finally:
            queue.close()
            if subscription_id is not None:
                runtime.streams.stream_bus.unsubscribe(subscription_id)
            pending_tasks = [task for task in (receive_task, frame_task) if task is not None]
            for task in pending_tasks:
                task.cancel()
            if pending_tasks:
                await asyncio.gather(*pending_tasks, return_exceptions=True)

    return router


def _positive_int(value: object, default: int) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return int(default)
    return parsed if parsed > 0 else int(default)


class _SharedMjpegFrameSource:
    """单个读线程共享最新视频帧；连接等待通知，不各自定时读共享内存。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._reader = None
        self._shm_path = None
        self._consumers = 0
        self._cached_multipart = None
        self._cached_sequence = -1
        self._camera_id = ""
        self._worker = None
        self._stop = None
        self._waiters = set()

    def acquire(self, shm_path: str, *, camera_id: str = "front_camera") -> bool:
        if VideoShmReader is None:
            return False
        with self._lock:
            if self._reader is not None:
                if self._shm_path != shm_path or self._camera_id != camera_id:
                    return False
            else:
                reader = VideoShmReader(shm_path)
                stop = threading.Event()
                worker = threading.Thread(target=self._read_loop, args=(reader, stop, camera_id),
                                          name="video-shm-reader", daemon=True)
                self._reader, self._stop, self._worker = reader, stop, worker
                self._shm_path, self._camera_id = shm_path, camera_id
                self._cached_multipart, self._cached_sequence = None, -1
                try:
                    worker.start()
                except BaseException:
                    self._reader = self._stop = self._worker = None
                    reader.close()
                    raise
            self._consumers += 1
            return True

    @staticmethod
    def _notify(waiter) -> None:
        if not waiter.done():
            waiter.set_result(None)

    def _read_loop(self, reader, stop, camera_id) -> None:
        try:
            while not stop.is_set():
                result = reader.read_latest()
                if result is not None:
                    data, sequence, timestamp_ns = result
                    multipart = _format_mjpeg_frame(data, camera_id=camera_id,
                                                    sequence=sequence, timestamp_ns=timestamp_ns)
                    with self._lock:
                        if self._reader is not reader or stop.is_set():
                            return
                        self._cached_multipart = multipart
                        self._cached_sequence += 1
                        waiters = tuple(self._waiters)
                        self._waiters.clear()
                    for loop, waiter in waiters:
                        try:
                            loop.call_soon_threadsafe(self._notify, waiter)
                        except RuntimeError:
                            pass  # 已关闭的连接循环不再接收通知。
                # 共享内存尚无通知描述符，只有拥有方做短轮询，不给每个连接新增定时器。
                stop.wait(0.003)
        except Exception:
            _logger.exception("视频共享读取失败，结束当前视频连接。")
        finally:
            reader.close()
            with self._lock:
                if self._reader is reader:
                    self._cached_multipart = None
                    stop.set()
                    waiters = tuple(self._waiters)
                    self._waiters.clear()
                else:
                    waiters = ()
            for loop, waiter in waiters:
                try:
                    loop.call_soon_threadsafe(self._notify, waiter)
                except RuntimeError:
                    pass

    def latest_multipart(self) -> Tuple[Optional[bytes], int]:
        with self._lock:
            return self._cached_multipart, self._cached_sequence

    async def wait_for_frame(self, after_sequence: int) -> Tuple[Optional[bytes], int]:
        loop = asyncio.get_running_loop()
        while True:
            waiter = loop.create_future()
            entry = (loop, waiter)
            with self._lock:
                if self._stop is None or self._stop.is_set():
                    return None, self._cached_sequence
                if self._cached_multipart is not None and self._cached_sequence != after_sequence:
                    return self._cached_multipart, self._cached_sequence
                self._waiters.add(entry)
            try:
                await waiter
            finally:
                with self._lock:
                    self._waiters.discard(entry)

    def release(self) -> None:
        with self._lock:
            self._consumers = max(0, self._consumers - 1)
            if self._consumers or self._reader is None:
                return
            stop, worker = self._stop, self._worker
            stop.set()
            # 最后一个消费者关闭时撤销本代；新连接可以创建自己的读取拥有方。
            self._reader = self._worker = self._stop = None
            self._cached_multipart = None
            waiters = tuple(self._waiters)
            self._waiters.clear()
        for loop, waiter in waiters:
            try:
                loop.call_soon_threadsafe(self._notify, waiter)
            except RuntimeError:
                pass
        worker.join()


_SHARED_MJPEG_SOURCE = _SharedMjpegFrameSource()


def _format_mjpeg_frame(data: bytes, *, camera_id: str = "", sequence=None, timestamp_ns: int = 0) -> bytes:
    identity_headers = b""
    if camera_id:
        identity_headers += f"X-Camera-Id: {quote(camera_id, safe='')}\r\n".encode("ascii")
    if sequence is not None:
        identity_headers += f"X-Frame-Sequence: {int(sequence)}\r\n".encode("ascii")
    if timestamp_ns > 0:
        timestamp = datetime.fromtimestamp(timestamp_ns // 1_000_000_000, tz=timezone.utc).replace(
            microsecond=(timestamp_ns % 1_000_000_000) // 1000,
        )
        identity_headers += f"X-Frame-Timestamp: {timestamp.isoformat()}\r\n".encode("ascii")
        identity_headers += b"X-Frame-Timestamp-Source: video_relay.write_time\r\n"
    headers = (
        b"--frame\r\n"
        b"Content-Type: image/jpeg\r\n"
        + f"Content-Length: {len(data)}".encode("ascii")
        + b"\r\n" + identity_headers + b"\r\n"
    )
    return b"".join((headers, data, b"\r\n"))


def _format_sse_payload(frame: StreamFrame) -> str:
    return _format_sse("stream.frame", frame, cursor=frame.cursor)


def _format_sse_keepalive(cursor: int) -> str:
    return _format_sse(
        "keepalive",
        {"cursor": _pointcloud_identity_integer(cursor, "cursor")},
    )


def _format_sse(event_name: str, payload: object, *, cursor: Optional[object] = None) -> str:
    lines = []
    if cursor is not None:
        lines.append(
            "id: %s" % _pointcloud_identity_integer(cursor, "cursor")
        )
    lines.append(f"event: {event_name}")
    encoded = _strict_json_dumps(payload)
    lines.append(f"data: {encoded}")
    lines.append("")
    return "\n".join(lines) + "\n"
