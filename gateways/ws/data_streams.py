from __future__ import annotations

import asyncio
import base64
import collections
from collections.abc import Mapping
from contextlib import asynccontextmanager
from dataclasses import fields as dataclass_fields, is_dataclass
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
from contracts.runtime_views import MapSnapshot
from contracts.occupancy_cells import PackedOccupancyCells
from core.logging_utils import LogRateLimiter
from core.stream_coalescing import stream_replacement_key
from gateways.auth import GatewayAccessManager
from gateways.errors import GatewayError
from gateways.feedback import build_public_runtime_event
from gateways.runtime import GatewayRuntime
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
_STREAM_JSON_MAX_DEPTH = 12
_STREAM_JSON_MAX_NODES = 2048
_STREAM_JSON_MAX_ENTRIES = 4096
_STREAM_JSON_MAX_CONTAINER_ITEMS = 256
_STREAM_JSON_MAX_STRING_CHARS = 8192
_STREAM_JSON_MAX_TOTAL_STRING_CHARS = 64 * 1024
_STREAM_JSON_MAX_BYTES = 16 * 1024
_STREAM_JSON_MAX_INTEGER_BITS = 256
_STREAM_JSON_MAX_OCCUPANCY_CELLS = 2_000_000
_POINTCLOUD_BINARY_MAGIC = b"NUWAXPC\x00"
_POINTCLOUD_BINARY_VERSION = 1
_POINTCLOUD_BINARY_PREFIX = struct.Struct("<8sBI")
_POINTCLOUD_BINARY_MAX_HEADER_BYTES = 64 * 1024
_MAX_U64 = (1 << 64) - 1
_POINTCLOUD_BINARY_TOPICS = frozenset(
    {"pointcloud_map.snapshot", "pointcloud_map.live_chunk", "map.frame"}
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
        "pointcloud_map.live_chunk",
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
_TERMINAL_TASK_EVENT_TYPES = frozenset(
    {
        "task.succeeded",
        "task.failed",
        "task.cancelled",
        "task.timeout",
        "task.resource_quarantined",
    }
)


class _JsonProjectionBudget:
    def __init__(self) -> None:
        self.nodes_remaining = _STREAM_JSON_MAX_NODES
        self.entries_remaining = _STREAM_JSON_MAX_ENTRIES
        self.text_chars_remaining = _STREAM_JSON_MAX_TOTAL_STRING_CHARS
        self.bytes_remaining = _STREAM_JSON_MAX_BYTES

    def take_node(self) -> bool:
        if self.nodes_remaining <= 0:
            return False
        self.nodes_remaining -= 1
        return True

    def take_entry(self) -> bool:
        if self.entries_remaining <= 0:
            return False
        self.entries_remaining -= 1
        return True


def _bounded_json_text(
    value: str,
    budget: _JsonProjectionBudget,
    *,
    max_chars: int = _STREAM_JSON_MAX_STRING_CHARS,
) -> str:
    available = min(max(0, int(max_chars)), budget.text_chars_remaining)
    if available <= 0:
        return ""
    truncated = len(value) > available
    content_limit = max(0, available - 1) if truncated else available
    fragment = value[:content_limit].encode("utf-8", errors="replace").decode("utf-8")
    result = f"{fragment}…" if truncated else fragment
    budget.text_chars_remaining -= len(result)
    return result


def _safe_type_name(value: object, budget: _JsonProjectionBudget) -> str:
    value_type = type(value)
    module = getattr(value_type, "__module__", "")
    name = getattr(value_type, "__qualname__", getattr(value_type, "__name__", "object"))
    type_name = f"{module}.{name}" if module and module != "builtins" else str(name)
    return _bounded_json_text(type_name, budget, max_chars=256)


def _project_json_key(key: object, budget: _JsonProjectionBudget, index: int) -> str:
    if isinstance(key, str):
        raw_key = key
    elif key is None:
        raw_key = "null"
    elif isinstance(key, bool):
        raw_key = "true" if key else "false"
    elif isinstance(key, int):
        raw_key = str(key) if key.bit_length() <= _STREAM_JSON_MAX_INTEGER_BITS else f"<integer:{key.bit_length()}bits>"
    elif isinstance(key, float):
        if math.isnan(key):
            raw_key = "NaN"
        elif math.isinf(key):
            raw_key = "Infinity" if key > 0 else "-Infinity"
        else:
            raw_key = str(key)
    elif isinstance(key, Enum) and isinstance(key.value, (str, int, float, bool)):
        raw_key = str(key.value)
    else:
        raw_key = f"<{type(key).__name__}:{index}>"
    return _bounded_json_text(raw_key, budget, max_chars=256) or f"item_{index}"


def _project_json_bytes(value: object, budget: _JsonProjectionBudget) -> Dict[str, object]:
    size = len(value)  # type: ignore[arg-type]
    included = min(size, budget.bytes_remaining)
    budget.bytes_remaining -= included
    chunk = bytes(value[:included])  # type: ignore[index]
    encoded = base64.b64encode(chunk).decode("ascii")
    return {
        "$type": "bytes",
        "encoding": "base64",
        "data": _bounded_json_text(encoded, budget),
        "size": size,
        "truncated": included < size,
    }


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
    if frame.topic == "map.frame" and type(values["payload"]) is MapSnapshot:
        snapshot = values["payload"]
        values["payload"] = selected_fields(snapshot, MapSnapshot.model_fields)
        values["payload"]["update_sequence"] = _pointcloud_identity_integer(snapshot.update_sequence, "update_sequence")
    values["metadata"] = frame.metadata
    return values


def _complete_map_occupancy_data(frame: StreamFrame, *, grid=None,
                                max_cells: int = _STREAM_JSON_MAX_OCCUPANCY_CELLS) -> Optional[object]:
    """共享已验证的规范栅格；原始字典输入仍在编码边界验证。"""

    if frame.topic != "map.frame":
        return None
    occupancy_grid = grid if grid is not None else field_value(frame.payload, "occupancy_grid")
    if occupancy_grid is None:
        return None
    from contracts.maps import OccupancyGrid
    if isinstance(occupancy_grid, OccupancyGrid):
        return occupancy_grid.data
    width = (
        occupancy_grid.get("width")
        if isinstance(occupancy_grid, Mapping)
        else getattr(occupancy_grid, "width", None)
    )
    height = (
        occupancy_grid.get("height")
        if isinstance(occupancy_grid, Mapping)
        else getattr(occupancy_grid, "height", None)
    )
    data = (
        occupancy_grid.get("data")
        if isinstance(occupancy_grid, Mapping)
        else getattr(occupancy_grid, "data", None)
    )
    if (
        isinstance(width, bool)
        or isinstance(height, bool)
        or not isinstance(width, int)
        or not isinstance(height, int)
    ):
        return None
    cell_count = width * height
    if (
        cell_count <= 0
        or cell_count > max_cells
        or not isinstance(data, (list, tuple))
        or len(data) != cell_count
    ):
        return None
    if any(
        isinstance(value, bool)
        or not isinstance(value, int)
        or value < -1
        or value > 100
        for value in data
    ):
        return None
    return data


def _project_json_value(
    value: object,
    budget: _JsonProjectionBudget,
    active_ids: set[int],
    *,
    depth: int,
) -> object:
    if not budget.take_node():
        return "<truncated:node_budget>"
    if value is None or isinstance(value, bool):
        return value
    if isinstance(value, int):
        if value.bit_length() <= _STREAM_JSON_MAX_INTEGER_BITS:
            return value
        return f"<integer:{value.bit_length()}bits>"
    if isinstance(value, float):
        if math.isnan(value):
            return "NaN"
        if math.isinf(value):
            return "Infinity" if value > 0 else "-Infinity"
        return value
    if isinstance(value, str):
        return _bounded_json_text(value, budget)
    if isinstance(value, (bytes, bytearray, memoryview)):
        return _project_json_bytes(value, budget)
    if isinstance(value, Enum):
        enum_value = value.value
        if isinstance(enum_value, (str, int, float, bool)) or enum_value is None:
            return _project_json_value(enum_value, budget, active_ids, depth=depth + 1)
        return _safe_type_name(value, budget)
    if isinstance(value, Decimal):
        if not value.is_finite():
            return str(value)
        return _bounded_json_text(str(value), budget)
    if isinstance(value, (datetime, date)):
        try:
            return _bounded_json_text(value.isoformat(), budget, max_chars=160)
        except Exception:
            return _safe_type_name(value, budget)
    if isinstance(value, (Path, UUID)):
        return _bounded_json_text(str(value), budget)
    if depth >= _STREAM_JSON_MAX_DEPTH:
        return "<truncated:max_depth>"

    value_id = id(value)
    if value_id in active_ids:
        return "<truncated:cycle>"
    active_ids.add(value_id)
    try:
        if isinstance(value, StreamFrame):
            frame_values = _stream_frame_values(value)
            if value.topic == "map.frame":
                payload = frame_values["payload"]
                # 仅优化正式快照；子类和通用模型继续保留逐字段异常与预算保护。
                if type(payload) is MapSnapshot:
                    # 保留合同的防御读取，每个图层仅取一份，编码和完整性校验共用。
                    frame_values["payload"] = selected_fields(payload, MapSnapshot.model_fields)
                    frame_values["payload"]["update_sequence"] = _pointcloud_identity_integer(payload.update_sequence, "update_sequence")
            projected_frame = _project_json_value(
                frame_values,
                budget,
                active_ids,
                depth=depth + 1,
            )
            # 平台日志已在写入时脱敏；不要再次把异常栈截成 8KB 的消息片段。
            if value.topic == "logs.entry" and isinstance(value.payload, dict) and isinstance(projected_frame, dict):
                projected_payload = projected_frame.get("payload")
                if isinstance(projected_payload, dict):
                    for key in ("message", "exception"):
                        if isinstance(value.payload.get(key), str):
                            projected_payload[key] = value.payload[key]
            occupancy_data = _complete_map_occupancy_data(
                value,
                grid=field_value(frame_values["payload"], "occupancy_grid") if value.topic == "map.frame" else None,
            )
            if occupancy_data is not None and isinstance(projected_frame, dict):
                projected_payload = projected_frame.get("payload")
                if isinstance(projected_payload, dict):
                    projected_grid = projected_payload.get("occupancy_grid")
                    if isinstance(projected_grid, dict):
                        projected_grid["data"] = occupancy_data.to_list() if type(occupancy_data) is PackedOccupancyCells else occupancy_data
            return projected_frame

        if isinstance(value, Mapping):
            projected: Dict[str, object] = {}
            try:
                for index, (key, item) in enumerate(value.items()):
                    if index >= _STREAM_JSON_MAX_CONTAINER_ITEMS:
                        projected["$truncated"] = "item_limit"
                        break
                    if not budget.take_entry():
                        projected["$truncated"] = "entry_budget"
                        break
                    projected_key = _project_json_key(key, budget, index)
                    while projected_key in projected:
                        projected_key = f"{projected_key}_{index}"
                    projected[projected_key] = _project_json_value(
                        item,
                        budget,
                        active_ids,
                        depth=depth + 1,
                    )
            except Exception as exc:
                projected["$projection_error"] = type(exc).__name__
            return projected

        if isinstance(value, (list, tuple, set, frozenset)):
            projected_items = []
            try:
                for index, item in enumerate(value):
                    if index >= _STREAM_JSON_MAX_CONTAINER_ITEMS:
                        projected_items.append("<truncated:item_limit>")
                        break
                    if not budget.take_entry():
                        projected_items.append("<truncated:entry_budget>")
                        break
                    projected_items.append(
                        _project_json_value(
                            item,
                            budget,
                            active_ids,
                            depth=depth + 1,
                        )
                    )
            except Exception as exc:
                projected_items.append(f"<projection_error:{type(exc).__name__}>")
            return projected_items

        model_fields = getattr(type(value), "model_fields", None)
        if isinstance(model_fields, Mapping):
            field_values: Dict[str, object] = {}
            for index, field_name in enumerate(model_fields):
                if index >= _STREAM_JSON_MAX_CONTAINER_ITEMS or not budget.take_entry():
                    field_values["$truncated"] = "field_budget"
                    break
                try:
                    field_values[str(field_name)] = getattr(value, field_name)
                except Exception as exc:
                    field_values[str(field_name)] = f"<projection_error:{type(exc).__name__}>"
            return _project_json_value(field_values, budget, active_ids, depth=depth + 1)

        if is_dataclass(value) and not isinstance(value, type):
            field_values = {}
            for index, field in enumerate(dataclass_fields(value)):
                if index >= _STREAM_JSON_MAX_CONTAINER_ITEMS or not budget.take_entry():
                    field_values["$truncated"] = "field_budget"
                    break
                try:
                    field_values[field.name] = getattr(value, field.name)
                except Exception as exc:
                    field_values[field.name] = f"<projection_error:{type(exc).__name__}>"
            return _project_json_value(field_values, budget, active_ids, depth=depth + 1)

        try:
            attributes = vars(value)
        except (TypeError, ValueError):
            attributes = None
        projected_object: Dict[str, object] = {"$type": _safe_type_name(value, budget)}
        if isinstance(attributes, Mapping) and attributes:
            projected_object["attributes"] = _project_json_value(
                attributes,
                budget,
                active_ids,
                depth=depth + 1,
            )
        return projected_object
    finally:
        active_ids.discard(value_id)


def _strict_json_dumps(value: object, *, compact: bool = False) -> str:
    projected = _project_json_value(
        value,
        _JsonProjectionBudget(),
        set(),
        depth=0,
    )

    return json.dumps(
        projected,
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":") if compact else None,
    )


def _stream_json_default(value):
    """规范实时载荷直接使用模型序列化，不逐字段重复做预算、反射与字符串编码。"""
    if isinstance(value, StreamFrame):
        return _stream_frame_values(value)
    if hasattr(value, "model_dump"):
        return value.model_dump(mode="json")
    if isinstance(value, (datetime, date)):
        return value.isoformat()
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, (Path, UUID, Decimal)):
        return str(value)
    raise TypeError(type(value).__name__)


def _stream_json_dumps(value, *, ensure_ascii: bool = False) -> str:
    try:
        return json.dumps(value, default=_stream_json_default, ensure_ascii=ensure_ascii,
                          allow_nan=False, separators=(",", ":"))
    except (TypeError, ValueError):
        # 非规范诊断对象仍可展示；正常消息不经过整棵对象树的防御投影。
        return _strict_json_dumps(value, compact=True)


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
    runtime: GatewayRuntime,
    *,
    after_cursor: int,
    topics: Optional[Iterable[str]],
    limit: int = _STREAM_REPLAY_MAX_FRAMES,
    snapshot=None,
) -> Tuple[Dict[str, Any], Tuple[StreamFrame, ...]]:
    topic_filter = tuple(str(topic) for topic in topics) if topics is not None else None
    if snapshot is None:
        snapshot = runtime.stream_bus.replay_snapshot(
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
    if frame.topic == "tasks.state" and event_type in _TERMINAL_TASK_EVENT_TYPES:
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

    def put(self, frame: StreamFrame, runtime: GatewayRuntime) -> None:
        with self._lock:
            if self._closed:
                return
            self._put_locked(frame, runtime)
        self._wake_waiter()

    def put_threadsafe(self, loop: asyncio.AbstractEventLoop, frame: StreamFrame, runtime: GatewayRuntime) -> None:
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

    def _put_locked(self, frame: StreamFrame, runtime: GatewayRuntime) -> None:
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


def _take_queue_gap_notice(queue: _ClientStreamQueue, runtime: GatewayRuntime) -> Optional[Dict[str, Any]]:
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
        "latest_cursor": runtime.stream_bus.latest_cursor(),
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
    runtime: GatewayRuntime,
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
    if frame.topic == "pointcloud_map.live_chunk":
        if len(xyz_bytes) < 8:
            raise ValueError("旧 live compatibility 载荷头被截断。")
        legacy_revision, legacy_point_count = struct.unpack("<II", xyz_bytes[:8])
        del legacy_revision
        xyz_bytes = xyz_bytes[8:]
        if legacy_point_count != metadata.get("point_count"):
            raise ValueError("旧 live compatibility 点数与统一身份不一致。")

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
_MAP_BINARY_VERSION = 3
_MAP_BINARY_MAX_CELLS = 16 * 1024 * 1024
_MAP_ENCODE_CACHE_BYTES = 32 * 1024 * 1024
_MAP_ENCODE_CACHE = collections.OrderedDict()
_MAP_ENCODE_LOCK = threading.Lock()
_MAP_ENCODE_RETAINED_BYTES = 0


def _pack_map_envelope(header: bytes, raw: bytes) -> bytes:
    """相同只读载荷和完整帧头只拼接一次；强引用避免对象编号复用。"""
    global _MAP_ENCODE_RETAINED_BYTES
    key = (header, id(raw))
    size = len(raw) * 2 + len(header) * 2 + 13
    if size > _MAP_ENCODE_CACHE_BYTES:
        return struct.pack("<8sBI", _MAP_BINARY_MAGIC, _MAP_BINARY_VERSION, len(header)) + header + raw
    with _MAP_ENCODE_LOCK:
        cached = _MAP_ENCODE_CACHE.get(key)
        if cached is not None:
            _MAP_ENCODE_CACHE.move_to_end(key)
            return cached[1]
        encoded = struct.pack("<8sBI", _MAP_BINARY_MAGIC, _MAP_BINARY_VERSION, len(header)) + header + raw
        while _MAP_ENCODE_CACHE and (_MAP_ENCODE_RETAINED_BYTES + size > _MAP_ENCODE_CACHE_BYTES or len(_MAP_ENCODE_CACHE) >= 8):
            _, (_, _, retired_size) = _MAP_ENCODE_CACHE.popitem(last=False)
            _MAP_ENCODE_RETAINED_BYTES -= retired_size
        _MAP_ENCODE_CACHE[key] = (raw, encoded, size)
        _MAP_ENCODE_RETAINED_BYTES += size
        return encoded


def _encode_map_binary_frame(frame: StreamFrame) -> bytes:
    """版本3：明确来源代际与修订，不再要求或输出地图内容摘要。"""
    import numpy as np

    # 已发布快照按只读共享；帧头与栅格共享同一份输入。
    occupancy_grid = field_value(frame.payload, "occupancy_grid")
    data = _complete_map_occupancy_data(frame, grid=occupancy_grid, max_cells=_MAP_BINARY_MAX_CELLS)
    if data is None:
        raise ValueError("地图栅格尺寸、数量或取值无效")
    if len(data) > _MAP_BINARY_MAX_CELLS:
        raise ValueError("二进制地图超过16MiB栅格容量")
    payload = selected_fields(frame.payload, (
        "map_name", "source_name", "source_epoch", "update_sequence", "schema_version",
        "frame_id", "timestamp", "metadata",
    ))
    grid = selected_fields(occupancy_grid, (
        "map_id", "frame_id", "width", "height", "resolution_m", "origin", "timestamp",
    ))
    if not payload.get("map_name") or not grid.get("origin") or not grid.get("frame_id"):
        raise ValueError("二进制地图缺少地图身份、坐标系或原点")
    if not payload.get("source_name") or not payload.get("source_epoch"):
        raise ValueError("二进制地图缺少来源代际")
    if payload.get("schema_version") != 2:
        raise ValueError("二进制地图只接受显式修订快照版本2")
    if grid.get("map_id") != payload["map_name"] or grid.get("frame_id") != payload.get("frame_id"):
        raise ValueError("二进制地图图层与来源作用域不一致")
    payload["update_sequence"] = _pointcloud_identity_integer(payload.get("update_sequence"), "update_sequence")
    if payload["update_sequence"] == "0":
        raise ValueError("地图更新修订必须为正数")
    # 栅格先于可变诊断，保持头部字段和显示坐标不被诊断耗尽预算。
    metadata = payload.pop("metadata", {})
    payload["occupancy_grid"] = grid
    payload["metadata"] = metadata
    header_frame = frame.model_copy(update={"payload": payload})
    header = _stream_json_dumps({"type": "stream.frame", "frame": header_frame}).encode("utf-8")
    if len(header) > 64 * 1024:
        raise ValueError("二进制地图头超过64KiB")
    if type(data) is PackedOccupancyCells:
        return _pack_map_envelope(header, data.raw)
    return struct.pack("<8sBI", _MAP_BINARY_MAGIC, _MAP_BINARY_VERSION, len(header)) + header + np.asarray(data, dtype=np.int8).tobytes()


def _encode_ws_frame(frame: StreamFrame, *, map_binary_version=3) -> Union[str, bytes]:
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
    if frame.topic == "map.frame" and frame.payload is not None and map_binary_version == 3:
        try:
            return _encode_map_binary_frame(frame)
        except (TypeError, ValueError, AttributeError):
            pass
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


def _legacy_pointcloud_frame(frame, snapshot):
    """只有明确订阅旧快照通道的连接要求完整兼容载荷。"""
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
    if frame.topic == "pointcloud_map.live_chunk":
        raw = struct.pack("<II", snapshot.revision_ordinal & 0xffffffff, snapshot.point_count) + raw
    elif frame.topic == "pointcloud_map.chunk":
        raw = {"revision": snapshot.revision, "frame_id": snapshot.frame_id, "lod": "preview",
               "chunk_index": 0, "chunk_count": 1, "format": "float32-xyz", "encoding": "base64",
               "data": base64.b64encode(raw).decode("ascii"), "point_count": snapshot.point_count,
               "source_kind": snapshot.provenance.get("source_kind"), "metadata": dict(metadata)}
    return frame.model_copy(update={"payload_kind": StreamPayloadKind.BINARY, "payload": raw,
                                    "metadata": metadata, "frame_id": snapshot.frame_id,
                                    "revision": snapshot.revision})


async def _send_ws_frame(websocket: WebSocket, frame: StreamFrame, *, pointcloud_encoder=None, map_binary_version=None, runtime=None, instance_encoder=None) -> None:
    if ((frame.topic == "pointcloud_map.snapshot" and frame.metadata.get("chunked"))
            or frame.metadata.get("legacy_pointcloud_topic")):
        snapshot = runtime.get_pointcloud_map_snapshot() if runtime is not None else None
        if snapshot is None:
            return
        if ((frame.topic == "pointcloud_map.live_chunk" and snapshot.source != "live")
                or (frame.topic == "pointcloud_map.chunk" and snapshot.source != "saved")):
            return
        frame = await _run_blocking(_legacy_pointcloud_frame, frame, snapshot)
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
    if runtime is not None and ((frame.topic == "pointcloud_map.snapshot" and frame.metadata.get("chunked"))
            or frame.metadata.get("legacy_pointcloud_topic")):
        snapshot = runtime.get_pointcloud_map_snapshot()
        if snapshot is not None:
            frame = await _run_blocking(_legacy_pointcloud_frame, frame, snapshot)
    if frame.topic == "map.frame" or frame.payload_kind == StreamPayloadKind.BINARY:
        return await _run_blocking(_format_sse_payload, frame)
    return _format_sse_payload(frame)


def _map_identity_message(frame: Optional[StreamFrame], *, empty_cursor: int = 0,
                          start_cursor: Optional[int] = None) -> Dict[str, object]:
    """只投影既有地图身份，不读取栅格或为传输创建另一套来源代际。"""
    cleared = frame is None or frame.status == "cleared" or frame.payload is None
    identity = selected_fields(frame.payload, (
        "map_name", "source_name", "source_epoch", "frame_id",
    )) if frame is not None and not cleared else {}
    if frame is not None and not cleared and not identity.get("frame_id"):
        identity["frame_id"] = frame.frame_id
    return {
        "type": "stream.map_identity",
        "cursor": _pointcloud_identity_integer(frame.cursor if frame is not None else empty_cursor, "cursor"),
        "start_cursor": _pointcloud_identity_integer(
            start_cursor if start_cursor is not None else (frame.cursor if frame is not None else empty_cursor), "start_cursor"),
        **{name: identity.get(name) for name in ("map_name", "source_name", "source_epoch", "frame_id")},
        "cleared": cleared,
    }


def build_data_stream_router(runtime: GatewayRuntime, access_manager: GatewayAccessManager) -> APIRouter:
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
            subscription_id, replay_snapshot = runtime.stream_bus.subscribe_with_replay(
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
                        yield _format_sse_keepalive(runtime.stream_bus.latest_cursor())
            finally:
                queue.close()
                runtime.stream_bus.unsubscribe(subscription_id)

        return StreamingResponse(frame_generator(), media_type="text/event-stream")

    @router.get("/streams/video.mjpeg")
    async def video_mjpeg_stream(
        request: Request,
        camera_id: Optional[str] = None,
        fps: Optional[float] = None,
    ):
        source_status = runtime.perception_service.video_frame_source_status()
        source_camera_id = str(source_status.get("camera_id") or "front_camera")
        if camera_id is not None and camera_id != source_camera_id:
            return JSONResponse(status_code=404, content={
                "error": "video_camera_unavailable",
                "message": "当前视频源未绑定请求的相机。",
                "camera_id": camera_id,
                "available_camera_id": source_camera_id,
            })
        video_available = False
        if VideoShmReader is not None:
            _shm_path = os.environ.get("NUWAX_VIDEO_SHM_PATH", "/tmp/nuwax_video.shm")
            _probe = VideoShmReader(_shm_path)
            video_available = _probe.has_new_frame()
            _probe.close()
        if not video_available:
            request.state.error_category = "not_ready"
            return JSONResponse(
                status_code=503,
                content={
                    "error": "video_stream_not_available",
                    "reason": "rust_bridge_no_frames",
                    "message": "Rust 视频中转尚未收到帧数据。",
                },
            )
        # 默认随输入到达交付；显式请求的帧率仅作为发送上限。
        requested_fps = float(fps) if fps is not None else 0.0
        if not math.isfinite(requested_fps) or requested_fps < 0:
            requested_fps = 0.0
        frame_interval_sec = 1.0 / min(max(requested_fps, 1.0), 60.0) if requested_fps else 0.0

        async def frame_generator():
            shm_path = os.environ.get("NUWAX_VIDEO_SHM_PATH", "/tmp/nuwax_video.shm")
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
            map_identity_only = websocket.query_params.get("map_payload") == "identity"
            map_binary_version = 3 if websocket.query_params.get("map_binary_version") == "3" else None
            if map_identity_only and set(topic_filter or ()) != {"localization.pose", "map.frame"}:
                raise ValueError("地图身份模式须仅订阅 localization.pose 与 map.frame。")
        except Exception as exc:
            error = exc if isinstance(exc, GatewayError) else GatewayError(f"数据流订阅初始化失败: {exc}")
            await websocket.close(code=4403, reason=error.message)
            return

        await websocket.accept()
        loop = asyncio.get_running_loop()
        queue = _ClientStreamQueue(
            maxsize=_STREAM_QUEUE_MAXSIZE,
            preserve_topics=("map.frame",) if map_identity_only else (),
        )
        subscription_id: Optional[str] = None
        receive_task = None
        frame_task = None
        pointcloud_detached = websocket.query_params.get("pointcloud_transport") == "pages"

        try:
            subscription_id, replay_snapshot = runtime.stream_bus.subscribe_with_replay(
                lambda frame: _safe_enqueue(loop, queue, frame, runtime),
                # 身份通道只有可替代状态，每次连接从同一原子边界重建地图与位姿。
                after_cursor=0 if map_identity_only else after_cursor,
                topics=topic_filter,
                async_dispatch=True,
                latest_only_topics=(
                    _LATEST_ONLY_STREAM_TOPICS - {"map.frame"}
                    if map_identity_only else _LATEST_ONLY_STREAM_TOPICS
                ),
                on_gap=lambda notice: queue.note_gap_threadsafe(loop, notice),
            )
            replay_status, replay_frames = _build_replay_batch(
                runtime,
                after_cursor=after_cursor,
                topics=topic_filter,
                snapshot=replay_snapshot,
            )
            replay_latest_cursor = replay_status["latest_cursor"]
            if map_identity_only:
                map_frame = next((frame for frame in replay_frames if frame.topic == "map.frame"), None)
                barrier_cursor = int(replay_snapshot.map_identity_start_cursor or map_frame.cursor or 0) if map_frame is not None else replay_latest_cursor
                # 新地图之后不能重放可能属于旧图的缓存位姿，也不再重放旧地图屏障。
                replay_frames = tuple(
                    frame for frame in replay_frames
                    if frame.topic == "localization.pose" and int(frame.cursor or 0) > barrier_cursor
                )
                replay_status["frame_count"] = len(replay_frames)
                await _send_ws_json(websocket, _map_identity_message(map_frame, empty_cursor=barrier_cursor, start_cursor=barrier_cursor))
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
                    if map_identity_only and frame.topic == "map.frame":
                        await _send_ws_json(websocket, _map_identity_message(frame))
                    else:
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
                                runtime.stream_bus.latest_cursor(),
                                "cursor",
                            ),
                        },
                    )
        except WebSocketDisconnect:
            return
        finally:
            queue.close()
            if subscription_id is not None:
                runtime.stream_bus.unsubscribe(subscription_id)
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
    # 实例主题沿用 WS 的公开视图编码；源端 limit 只限制行数，不限制总字节。
    if isinstance(payload, StreamFrame) and payload.topic == "memory.instances":
        try:
            # JSON 转义保证 SSE 文本可编码为 UTF-8，不预扫描或截断公开名称。
            encoded = _stream_json_dumps(payload, ensure_ascii=True)
        except Exception:
            # 沿用旧 SSE 的诊断投影，避免异常对象中断已经开始的响应。
            encoded = _strict_json_dumps(payload)
    else:
        encoded = _strict_json_dumps(payload)
    lines.append(f"data: {encoded}")
    lines.append("")
    return "\n".join(lines) + "\n"
