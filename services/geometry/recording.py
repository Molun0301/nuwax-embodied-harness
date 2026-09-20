"""端侧几何记录：保存规范传感器原文与定位状态，不依赖网页订阅。"""
from __future__ import annotations

from collections import Counter, deque
from datetime import datetime, timezone
import fcntl
import json
import logging
from pathlib import Path
import shutil
import sqlite3
import threading
import time
from typing import Any, Dict, Iterator, Optional
import uuid

from contracts.sensors import CanonicalImuSample, CanonicalLaserScan, CanonicalLidarFrame


LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.recording")
SENSOR_TYPES = {
    "imu": CanonicalImuSample,
    "lidar_3d": CanonicalLidarFrame,
    "lidar_2d": CanonicalLaserScan,
}
_BINARY_FIELDS = (
    "xyz_f32_le", "intensity_f32_le", "ring_u16_le", "time_offset_ns_le",
    "angles_f32_le", "ranges_f32_le", "beam_states_u8", "intensities_f32_le", "time_offsets_ns_le",
)
_QUEUE_BYTES = 64 * 1024 * 1024
_BATCH_BYTES = 8 * 1024 * 1024
_RESERVE_BYTES = 256 * 1024 * 1024


def _field(value, name, default=None):
    return value.get(name, default) if isinstance(value, dict) else getattr(value, name, default)


def _json(value) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"), allow_nan=False)


def _pose_stamp(pose) -> int:
    stamp = _field(pose, "timestamp")
    if isinstance(stamp, str):
        stamp = datetime.fromisoformat(stamp.replace("Z", "+00:00"))
    if not isinstance(stamp, datetime):
        return 0
    delta = stamp.astimezone(timezone.utc) - datetime(1970, 1, 1, tzinfo=timezone.utc)
    return (delta.days * 86400 + delta.seconds) * 1_000_000_000 + delta.microseconds * 1000


class MappingRecorder:
    """网关拥有的有界单写入者；源回调只移交不可变样本引用。

    一个文件覆盖输入桥的预热及该网关内的建图会话，会话边界以地图来源事件记录。
    不删除旧记录；容量不足或写入错误明确报告记录不完整，建图输入继续运行。
    """

    def __init__(self, root: Path, *, enabled: bool = True,
                 max_bytes: int = 16 * 1024 ** 3, queue_bytes: int = _QUEUE_BYTES) -> None:
        self.root = Path(root)
        self.enabled = enabled
        self.max_bytes = max(4096, int(max_bytes))
        self._queue_limit = max(1, int(queue_bytes))
        self._condition = threading.Condition()
        self._lifecycle = threading.Lock()
        self._queue = deque()
        self._queued_bytes = 0
        self._thread = None
        self._ready = threading.Event()
        self._accepting = False
        self._stop_requested = False
        self._closed = False
        self._sequence = 0
        self._written = Counter()
        self._written_bytes = 0
        self._dropped = 0
        self._first_dropped_ns = None
        self._last_dropped_ns = None
        self._error = None
        self._path = None
        self._recording_id = None
        self._metadata = {}
        self._root_lock = None

    def start(self, metadata: Optional[Dict[str, Any]] = None) -> bool:
        """首次输入订阅前启动；重复挂接复用同一记录，覆盖算法预热输入。"""
        with self._lifecycle:
            with self._condition:
                if not self.enabled or self._closed:
                    return False
                if self._thread is None:
                    self._metadata = dict(metadata or {})
                    self._recording_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ-") + uuid.uuid4().hex[:12]
                    self._path = self.root / self._recording_id / "recording.sqlite3"
                    self._thread = threading.Thread(target=self._write_loop, name="geometry-recording", daemon=False)
                    self._thread.start()
            if not self._ready.wait(timeout=5.0):
                with self._condition:
                    self._error = "端侧记录初始化超时"
                    self._stop_requested = True
                    self._condition.notify_all()
            with self._condition:
                return self._accepting

    def _drop_locked(self, stamp: int) -> None:
        self._dropped += 1
        if self._first_dropped_ns is None:
            self._first_dropped_ns = stamp
        self._last_dropped_ns = stamp

    def _enqueue(self, kind: str, source_ns: int, value, size: int) -> bool:
        with self._condition:
            if not self._accepting:
                return False
            self._sequence += 1
            if size > self._queue_limit - self._queued_bytes:
                self._drop_locked(source_ns)
                self._condition.notify_all()
                return False
            self._queue.append((self._sequence, kind, source_ns, time.time_ns(), value, size))
            self._queued_bytes += size
            self._condition.notify_all()
            return True

    def record_transport_loss(self, count: int) -> None:
        """记录跨进程通道未能保留的证据数量，不反向阻塞控制输入。"""
        with self._condition:
            if count > 0:
                self._dropped += count
                now = time.time_ns()
                if self._first_dropped_ns is None:
                    self._first_dropped_ns = now
                self._last_dropped_ns = now
                self._condition.notify_all()

    def record_sensor(self, sample) -> bool:
        """在配组、丢弃和降采样之前保存原规范输入，保留每束时间及惯性值。"""
        if not isinstance(sample, tuple(SENSOR_TYPES.values())):
            return False
        if sample.payload_ownership != "inline":
            with self._condition:
                if self._accepting:
                    self._drop_locked(sample.timestamp_ns)
            self.record_event("recording.missing_payload", {
                "sensor_id": sample.sensor_id, "reason": "外置传感器载荷尚未物化",
            }, source_ns=sample.timestamp_ns)
            return False
        size = 2048 + sum(len(getattr(sample, key, None) or b"") for key in _BINARY_FIELDS)
        return self._enqueue("sensor." + sample.sample_kind.value, sample.timestamp_ns, sample, size)

    def record_event(self, kind: str, metadata: Dict[str, Any], *, source_ns: int = 0) -> bool:
        """小型身份事件在调用处冻结，避免把后续会话变更写进旧记录。"""
        try:
            payload = _json(metadata).encode("utf-8")
        except (TypeError, ValueError):
            with self._condition:
                if self._accepting:
                    self._drop_locked(source_ns)
            return False
        return self._enqueue(kind, source_ns, payload, len(payload) + 512)

    def record_stream_frame(self, frame) -> None:
        """端侧总线监听只记录小型状态，不复制完整地图或整段轨迹。"""
        with self._condition:
            if not self._accepting:
                return
        payload = frame.payload
        if frame.topic == "localization.pose":
            pose = payload.model_dump(mode="json") if hasattr(payload, "model_dump") else payload
            self.record_event("localization.pose", {
                "pose": pose, "status": frame.status, "cursor": str(frame.cursor),
                "metadata": dict(frame.metadata),
            }, source_ns=int(frame.metadata.get("source_timestamp_ns") or _pose_stamp(payload)))
        elif frame.topic == "map.frame":
            self.record_event("mapping.state", {
                "status": frame.status, "cursor": str(frame.cursor),
                **{key: _field(payload, key) for key in (
                    "map_name", "source_name", "source_epoch", "update_sequence", "content_sha256", "frame_id",
                )},
            })
        elif frame.topic == "runtime.governance":
            metadata = _field(payload, "metadata", {}) or {}
            self.record_event("mapping.context", {
                "map_name": metadata.get("map_name"),
                "geometry_runtime": metadata.get("geometry_runtime"),
                "mapping_profile": _field(payload, "mapping_profile"),
            })

    def status(self) -> Dict[str, Any]:
        with self._condition:
            return {
                "enabled": self.enabled, "recording_id": self._recording_id,
                "path": str(self._path) if self._path else None,
                "recording": self._accepting, "closed": self._closed,
                "complete": self._thread is not None and self._closed and self._error is None and self._dropped == 0,
                "incomplete": self._error is not None or self._dropped > 0,
                "written_records": dict(self._written), "written_payload_bytes": self._written_bytes,
                "queued_bytes": self._queued_bytes, "queue_limit_bytes": self._queue_limit,
                "dropped_records": self._dropped, "first_dropped_source_ns": self._first_dropped_ns,
                "last_dropped_source_ns": self._last_dropped_ns, "error": self._error,
            }

    def _open_database(self):
        self.root.mkdir(parents=True, exist_ok=True)
        self._root_lock = (self.root / ".recording.lock").open("a+b")
        try:
            fcntl.flock(self._root_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise OSError("同一记录目录已有写入者，不能并发超额分配容量") from error
        used = sum(p.stat().st_size for p in self.root.glob("*/recording.sqlite3*") if p.is_file())
        available = min(self.max_bytes - used, shutil.disk_usage(self.root).free - _RESERVE_BYTES)
        if available < 1024 * 1024:
            raise OSError("端侧记录容量不足；已有记录保留，请释放空间或调整容量")
        self._path.parent.mkdir(mode=0o700)
        connection = sqlite3.connect(str(self._path), timeout=2.0)
        try:
            connection.execute("PRAGMA journal_mode=WAL")
            connection.execute("PRAGMA synchronous=FULL")
            connection.execute("PRAGMA max_page_count=%d" % (available // 4096))
            connection.execute("CREATE TABLE info (key TEXT PRIMARY KEY, value TEXT NOT NULL)")
            connection.execute("CREATE TABLE records (id INTEGER PRIMARY KEY, kind TEXT NOT NULL, source_ns TEXT NOT NULL, received_ns INTEGER NOT NULL, metadata TEXT NOT NULL, payload BLOB NOT NULL)")
            connection.execute("CREATE INDEX records_kind_id ON records(kind,id)")
            connection.execute("INSERT INTO info VALUES ('schema_version','1')")
            connection.execute("INSERT INTO info VALUES ('configuration',?)", (_json(self._metadata),))
            connection.commit()
            return connection
        except BaseException:
            connection.close()
            raise

    @staticmethod
    def _encode_record(item):
        sequence, kind, source_ns, received_ns, value, _ = item
        if kind.startswith("sensor."):
            binary = [(name, getattr(value, name)) for name in _BINARY_FIELDS
                      if isinstance(getattr(value, name, None), bytes)]
            metadata = {
                "sample": value.model_dump(mode="json", exclude=set(_BINARY_FIELDS)),
                "binary_fields": [(name, len(data)) for name, data in binary],
            }
            payload = b"".join(data for _, data in binary)
        else:
            metadata, payload = {}, value
        return sequence, kind, str(source_ns), received_ns, _json(metadata), payload

    def _write_batch(self, connection, batch) -> None:
        rows = [self._encode_record(item) for item in batch]
        connection.executemany("INSERT INTO records VALUES (?,?,?,?,?,?)", rows)
        connection.commit()
        with self._condition:
            self._written.update(item[1] for item in batch)
            self._written_bytes += sum(len(row[4].encode("utf-8")) + len(row[5]) for row in rows)

    def _write_loop(self) -> None:
        connection = None
        batch = []
        reported_drops = 0
        last_drop_report = 0.0
        try:
            connection = self._open_database()
            with self._condition:
                self._accepting = not self._stop_requested
            self._ready.set()
            while True:
                with self._condition:
                    while not self._queue and self._accepting:
                        self._condition.wait(timeout=0.5)
                        if self._dropped:
                            break
                    if not self._queue and not self._accepting:
                        break
                    # 小批次每250毫秒提交一次，避免每条惯性样本触发磁盘同步。
                    deadline = time.monotonic() + 0.25
                    batch = []
                    batch_bytes = 0
                    while len(batch) < 512 and batch_bytes < _BATCH_BYTES:
                        if self._queue:
                            # 大记录独占一个批次；不能把它叠加到接近上限的普通批次。
                            if batch and batch_bytes + self._queue[0][-1] > _BATCH_BYTES:
                                break
                            item = self._queue.popleft()
                            self._queued_bytes -= item[-1]
                            batch_bytes += item[-1]
                            batch.append(item)
                        elif not self._accepting or time.monotonic() >= deadline:
                            break
                        else:
                            self._condition.wait(timeout=max(0.0, deadline - time.monotonic()))
                if batch:
                    if shutil.disk_usage(self.root).free < _RESERVE_BYTES:
                        raise OSError("端侧记录剩余磁盘空间不足")
                    self._write_batch(connection, batch)
                    batch = []
                state = self.status()
                if state["dropped_records"] > reported_drops and time.monotonic() - last_drop_report >= 5.0:
                    LOGGER.warning("端侧几何记录存在缺口：累计%d条，源时间范围%s至%s；完整性状态已保存",
                                   state["dropped_records"], state["first_dropped_source_ns"], state["last_dropped_source_ns"])
                    reported_drops = state["dropped_records"]
                    last_drop_report = time.monotonic()
                connection.execute("INSERT OR REPLACE INTO info VALUES ('status',?)", (_json(state),))
                connection.commit()
        except Exception as error:
            with self._condition:
                self._error = str(error)
                for item in (*batch, *self._queue):
                    self._drop_locked(item[2])
                self._queue.clear()
                self._queued_bytes = 0
                self._accepting = False
            LOGGER.error("端侧几何记录失败，记录不完整：%s", error)
        finally:
            with self._condition:
                self._accepting = False
            self._ready.set()
            if connection is not None:
                try:
                    connection.rollback()
                    state = self.status()
                    state.update(closed=True, complete=not state["incomplete"])
                    connection.execute("INSERT OR REPLACE INTO info VALUES ('status',?)", (_json(state),))
                    connection.commit()
                    connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")
                except sqlite3.Error:
                    # 磁盘已满时仍保留此前成功提交的记录和旧的未关闭状态。
                    pass
                finally:
                    connection.close()
            if self._root_lock is not None:
                self._root_lock.close()
                self._root_lock = None

    def close(self, timeout: float = 10.0) -> None:
        """停止接纳、排空已入队数据并等待真实写入者退出。"""
        with self._lifecycle:
            with self._condition:
                self._accepting = False
                self._stop_requested = True
                self._condition.notify_all()
                worker = self._thread
            if worker is not None:
                worker.join(timeout)
                if worker.is_alive():
                    raise RuntimeError("端侧记录写入者尚未排空")
            with self._condition:
                self._closed = True


def iter_recorded_sensors(path: Path) -> Iterator[Any]:
    """按端侧接收顺序读回规范样本，供同输入离线验证；不连接实机。"""
    connection = sqlite3.connect(Path(path).resolve().as_uri() + "?mode=ro", uri=True)
    try:
        for kind, metadata_json, payload in connection.execute(
            "SELECT kind,metadata,payload FROM records WHERE kind IN ('sensor.imu','sensor.lidar_3d','sensor.lidar_2d') ORDER BY id"
        ):
            metadata = json.loads(metadata_json)
            sample = metadata["sample"]
            offset = 0
            for name, size in metadata["binary_fields"]:
                if name not in _BINARY_FIELDS or not isinstance(size, int) or size < 0 or offset + size > len(payload):
                    raise ValueError("记录中的传感器载荷布局无效")
                sample[name] = payload[offset:offset + size]
                offset += size
            if offset != len(payload):
                raise ValueError("记录中的传感器载荷长度不一致")
            yield SENSOR_TYPES[kind.split(".", 1)[1]].model_validate(sample)
    finally:
        connection.close()
