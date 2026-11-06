"""网页只订阅拥有方的快照，连接与刷新不改变业务执行。"""
from dataclasses import asdict
import logging
import os
import threading
from datetime import datetime, timedelta, timezone
from io import BytesIO

from core.stream_bus import StreamBus
from contracts.streams import StreamPayloadKind
from services.geometry.pointcloud_map import CloudMapMmapReader, PointCloudMapSnapshot
from services.geometry.state import SessionPhase

LOGGER = logging.getLogger(__name__)


class VideoSource:
    """感知从现有共享内存读取真实 JPEG 帧。"""
    def __init__(self, path, camera_info_provider, frame_id):
        from services.geometry.video_shm_reader import VideoShmReader
        self.reader = VideoShmReader(path)
        self.provider = camera_info_provider
        self.frame_id = frame_id

    def get_latest_image(self, camera_id=None):
        from PIL import Image
        from contracts.image import ImageFrame, ImageEncoding
        value = self.reader.read_latest()
        if value is None:
            return None
        data, sequence, timestamp_ns = value
        seconds, nanoseconds = divmod(timestamp_ns, 1_000_000_000)
        info = self.get_camera_info(camera_id)
        with Image.open(BytesIO(data)) as image:
            width, height = image.size
        return ImageFrame(camera_id=info.camera_id if info is not None else "front_camera",
            frame_id=info.frame_id if info is not None else self.frame_id,
            timestamp=datetime.fromtimestamp(seconds, timezone.utc) + timedelta(microseconds=nanoseconds // 1000),
            width_px=width, height_px=height, encoding=ImageEncoding.JPEG, data=data,
            metadata={"source_kind": "video_shared_memory", "source_timestamp_ns": timestamp_ns,
                      "ingest_sequence": sequence, "camera_info": info})

    def get_camera_info(self, camera_id=None):
        return None if self.provider is None else self.provider.get_camera_info(camera_id)

    def status_snapshot(self):
        return {"connected": self.reader.open(), "source": "video_shared_memory"}

    def close(self):
        self.reader.close()


class Streams:
    def __init__(self, platform, stream_bus: StreamBus):
        self.platform = platform
        self.event_bus = platform.events
        self.stream_bus = stream_bus
        self._unsubscribe = []
        self._task_subscription = None
        self._telemetry_stop = threading.Event()
        self._telemetry_thread = None
        self._running = False
        self._cloud_reader = CloudMapMmapReader(os.environ.get("NUWAX_CLOUDMAP_SHM_PATH"))
        self._cloud_lock = threading.Lock()
        self._cloud_wake = threading.Event()
        self._cloud_thread = None

    def start(self):
        self._running = True
        state = self.platform.geometry_state
        self._unsubscribe = [
            state.subscribe(self._state_updated),
            state.subscribe_pose(self._pose_updated),
            state.subscribe_map(self._map_updated),
        ]
        from providers import StreamingPointCloudProvider
        provider = self.platform.robot.providers
        if isinstance(provider, StreamingPointCloudProvider):
            token = provider.subscribe_point_cloud(self._pointcloud_updated)
            self._unsubscribe.append(lambda: provider.unsubscribe_point_cloud(token))
        self._task_subscription = self.event_bus.subscribe(
            lambda event: self.stream_bus.publish("tasks.state", source="tasks", payload=event),
            event_types=("task.updated",))
        self._cloud_thread = threading.Thread(target=self._cloud_updates, name="cloud-map-stream", daemon=True)
        self._cloud_thread.start()
        self._start_telemetry()

    def stop(self):
        self._running = False
        self._cloud_wake.set()
        if self._cloud_thread is not None:
            self._cloud_thread.join()
            self._cloud_thread = None
        self._stop_telemetry()
        for unsubscribe in self._unsubscribe:
            unsubscribe()
        self._unsubscribe = []
        if self._task_subscription is not None:
            self.event_bus.unsubscribe(self._task_subscription)
            self._task_subscription = None
        self._cloud_reader.close()

    def close(self):
        """发布服务退出后回收总线与视频读取句柄。"""
        video = self.platform.perception._video_frame_source
        if isinstance(video, VideoSource):
            video.close()
        self.stream_bus.shutdown()

    def latest_frames(self, topics=None):
        return self.stream_bus.latest_frames(topics=topics)

    def get_pointcloud_map_snapshot(self):
        info = self.platform.geometry_state.snapshot()
        if info is None or info.phase == SessionPhase.IDLE:
            return None
        with self._cloud_lock:
            value = self._cloud_reader.read_stable_snapshot()
        if value is None:
            return None
        frame_id = info.info.map_frame_id
        return PointCloudMapSnapshot(source="live", source_epoch=value.source_epoch,
            revision=f"cloud:{value.revision}", revision_ordinal=value.revision, frame_id=frame_id,
            source_timestamp_ns=value.timestamp_ns, point_count=value.point_count, format="float32-xyz-le",
            xyz_bytes=value.xyz_bytes, stream_generation=0, provenance={"canonical_owner": "geometry"},
            chunks=value.chunks, layout_generation=value.layout_generation, chunk_size_points=value.chunk_size_points,
            base_revision_ordinal=value.base_revision_ordinal, reset=value.reset)

    def _pose_updated(self, value):
        snapshot = self.platform.geometry_state.snapshot()
        self.stream_bus.publish("localization.pose", source="geometry", payload=value,
            frame_id=value.frame_id, metadata={"source_timestamp_ns": value.timestamp_ns,
                "map_name": snapshot.map_name if snapshot is not None else ""})

    def _pointcloud_updated(self, value):
        snapshot = self.platform.geometry_state.snapshot()
        self.stream_bus.publish("pointcloud.frame", source="robot", payload=value,
            frame_id=value.frame_id, metadata={**value.metadata,
                "map_name": snapshot.map_name if snapshot is not None else ""})

    def _map_updated(self, value):
        self.stream_bus.publish("map.frame", source="geometry", payload=value, frame_id=value.grid.frame_id,
                                revision=str(value.grid.timestamp_ns))
        self._cloud_wake.set()

    def _state_updated(self, value):
        self.stream_bus.publish("geometry.state", source="geometry", payload=asdict(value))
        self.stream_bus.publish("navigation.state", source="geometry",
            payload=None if value.navigation is None else asdict(value.navigation))
        self.stream_bus.publish("exploration.state", source="geometry",
            payload=None if value.exploration is None else asdict(value.exploration))

    def _cloud_updates(self):
        while True:
            self._cloud_wake.wait()
            self._cloud_wake.clear()
            if not self._running:
                return
            try:
                value = self.get_pointcloud_map_snapshot()
                if value is not None:
                    metadata = {"source": value.source, "source_epoch": value.source_epoch,
                        "revision": value.revision, "revision_ordinal": value.revision_ordinal,
                        "frame_id": value.frame_id, "source_timestamp_ns": value.source_timestamp_ns,
                        "point_count": value.point_count, "format": value.format, "stream_generation": 0,
                        "chunked": value.chunked, "layout_generation": value.layout_generation,
                        "chunk_size_points": value.chunk_size_points}
                    self.stream_bus.publish("pointcloud_map.snapshot", source="geometry", payload=value.xyz_bytes,
                        payload_kind=StreamPayloadKind.BINARY, frame_id=value.frame_id,
                        sequence=value.revision_ordinal, revision=value.revision, metadata=metadata)
                    self.stream_bus.publish("pointcloud_map.meta", source="geometry", payload=metadata,
                        frame_id=value.frame_id, sequence=value.revision_ordinal, revision=value.revision)
            except Exception:
                LOGGER.exception("读取 Geometry 点云共享内存失败。")

    def apply_telemetry_settings(self, config):
        self._stop_telemetry()
        self.platform.config.gateway = config
        if self._running:
            self._start_telemetry()

    def _start_telemetry(self):
        config = self.platform.config.gateway
        if not config.telemetry_enabled:
            return
        from services.telemetry import SystemMetricsCollector
        collector = SystemMetricsCollector(keywords=config.telemetry_process_keywords,
            include_process_tree=config.telemetry_include_process_tree, sample_interval_sec=config.telemetry_interval_sec)
        self._telemetry_stop.clear()
        def run():
            collector.warmup()
            while not self._telemetry_stop.wait(config.telemetry_interval_sec):
                self.stream_bus.publish("system.metrics", source="telemetry", payload=collector.sample())
        self._telemetry_thread = threading.Thread(target=run, name="system-metrics", daemon=True)
        self._telemetry_thread.start()

    def _stop_telemetry(self):
        self._telemetry_stop.set()
        if self._telemetry_thread is not None:
            self._telemetry_thread.join()
            self._telemetry_thread = None
