"""由使用原因集合驱动的唯一连续识别循环。"""
from __future__ import annotations

from dataclasses import dataclass, field
import logging
import threading
import time

from contracts.events import RuntimeEventCategory
from contracts.runtime_views import PerceptionRuntimeStatus
from core import StateNamespace
from core.errors import GatewayError
from services.perception.keyframe import PerceptionKeyframeSelector
from services.perception.observation_geometry import contract_pose
from services.perception.runtime_sample import PerceptionSample

LOGGER = logging.getLogger(__name__)


@dataclass
class _Burst:
    reason: str
    count: int
    samples: list = field(default_factory=list)
    frames: set = field(default_factory=set)


class PerceptionVideoRuntime:
    def __init__(self, *, perception_service, state_store, geometry_state, memory_service,
                 camera_id, interval_sec, detector_backend_name="", store_artifact=False,
                 failure_backoff_sec=2.0, keyframe_min_interval_sec=1.0,
                 keyframe_translation_threshold_m=0.75, keyframe_yaw_threshold_deg=20.0,
                 remember_keyframes=True, burst_frame_count=4, burst_interval_sec=0.08,
                 event_bus=None):
        self._perception_service = perception_service
        self._state_store = state_store
        self._geometry_state = geometry_state
        self._memory_service = memory_service
        self._event_bus = event_bus
        self._camera_id = camera_id
        self._interval_sec = interval_sec
        self._detector_backend_name = detector_backend_name
        self._store_artifact = store_artifact
        self._failure_backoff_sec = failure_backoff_sec
        self._remember_keyframes = remember_keyframes
        self._burst_frame_count = burst_frame_count
        self._burst_interval_sec = burst_interval_sec
        self._selector = PerceptionKeyframeSelector(min_interval_sec=keyframe_min_interval_sec,
            translation_threshold_m=keyframe_translation_threshold_m,
            yaw_threshold_deg=keyframe_yaw_threshold_deg)
        self._condition = threading.Condition(threading.RLock())
        self._reasons = {}
        self._samples = {}
        self._bursts = []
        self._seen_frames = {}
        self._thread = None
        self._closed = False
        self._stop = threading.Event()
        self._processed_frames = 0
        self._failure_count = 0
        self._last_error = None
        self._last_success = None
        self._prewarm = {"status": "pending"}
        self._write_status()

    def acquire(self, reason, *, interval_sec=None, camera_id=None,
                detector_backend_name=None, open_vocabulary_classes=()):
        with self._condition:
            if self._closed:
                raise GatewayError("感知运行时已关闭。", error_code="perception_runtime_closed", http_status=409)
            self._reasons[reason] = (self._interval_sec if interval_sec is None else interval_sec,
                camera_id or self._camera_id,
                detector_backend_name or self._detector_backend_name or None,
                tuple(open_vocabulary_classes))
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, name="perception-runtime", daemon=True)
                self._thread.start()
            self._condition.notify_all()
        self._write_status()
        return self.get_status()

    def release(self, reason):
        with self._condition:
            self._reasons.pop(reason, None)
            self._condition.notify_all()
        self._write_status()
        return self.get_status()

    def latest(self, *, camera_id=None, detector_backend_name=None):
        with self._condition:
            candidates = [value for (camera, backend), value in self._samples.items()
                if (camera_id is None or camera == camera_id)
                and (detector_backend_name is None or backend == detector_backend_name)]
            return max(candidates, key=lambda value: value.context.timestamp, default=None)

    def publish_sample(self, context, *, map_name, pose=None, frame_tree=None, image_bytes=None, inputs=None, complete=False):
        sample = PerceptionSample(context, map_name, pose, frame_tree, image_bytes, inputs, complete)
        with self._condition:
            key = (context.camera_id, context.detector_backend)
            previous = self._samples.get(key)
            if previous is None or self._frame_stamp(sample) >= self._frame_stamp(previous):
                self._samples[key] = sample
            self._condition.notify_all()
        return sample

    @staticmethod
    def _frame_stamp(sample):
        return sample.context.metadata.get("source_timestamp_ns", sample.inputs[0].timestamp.timestamp() * 1e9
            if sample.inputs is not None else sample.context.timestamp.timestamp() * 1e9)

    def complete_sample(self, sample):
        if sample.complete:
            return sample
        inputs = sample.inputs
        context, pose, tree = self._perception_service.complete_observation_geometry(
            sample.context, inputs, store_artifact=True, stop_event=self._stop)
        image_bytes = inputs[0].data if inputs is not None else sample.image_bytes
        return self.publish_sample(context, map_name=sample.map_name,
            pose=pose if inputs is not None else sample.pose,
            frame_tree=tree if inputs is not None else sample.frame_tree, image_bytes=image_bytes,
            inputs=inputs, complete=True)

    def run_burst(self, ctx, *, reason, frame_count=None, interval_sec=None):
        burst = _Burst(reason, self._burst_frame_count if frame_count is None else frame_count)
        with self._condition:
            settings = self._reasons[reason]
            self._bursts.append(burst)
        self.acquire(reason, interval_sec=self._burst_interval_sec if interval_sec is None else interval_sec,
            camera_id=settings[1], detector_backend_name=settings[2], open_vocabulary_classes=settings[3])
        try:
            while True:
                ctx.check()
                with self._condition:
                    if self._closed:
                        raise GatewayError("感知运行时已关闭。", error_code="perception_runtime_closed", http_status=409)
                    if len(burst.samples) >= burst.count:
                        return tuple(burst.samples)
                    self._condition.wait(min(0.05, ctx.remaining()) if ctx.remaining() is not None else 0.05)
        finally:
            with self._condition:
                self._bursts.remove(burst)
                # 只恢复仍属于本任务的原因，不复活已经释放的原因。
                if reason in self._reasons:
                    self._reasons[reason] = settings
                self._condition.notify_all()

    def _run(self):
        while True:
            with self._condition:
                while not self._reasons and not self._closed:
                    self._condition.wait()
                if self._closed:
                    return
                settings = dict(self._reasons)
            groups = {}
            interval = min(value[0] for value in settings.values())
            for reason, (_, camera, backend, classes) in settings.items():
                group = groups.setdefault((camera, backend), {"reasons": set(), "classes": set()})
                group["reasons"].add(reason)
                group["classes"].update(classes)
            started = time.monotonic()
            failed = False
            for (camera, backend), group in groups.items():
                try:
                    self._process(camera, backend, group)
                except Exception as error:
                    failed = True
                    with self._condition:
                        self._failure_count += 1
                        self._last_error = str(error)
                    LOGGER.exception("连续感知处理失败")
            self._write_status()
            delay = self._failure_backoff_sec if failed else interval
            with self._condition:
                if settings == self._reasons and not self._closed:
                    self._condition.wait(max(0.0, delay - (time.monotonic() - started)))

    def _process(self, camera, backend, group):
        snapshot = None if self._geometry_state is None else self._geometry_state.snapshot()
        map_name = "default" if self._geometry_state is None else snapshot.map_name if snapshot is not None else ""
        pose = contract_pose(None if self._geometry_state is None else self._geometry_state.pose())
        context = self._perception_service.detect_current_frame_fast(camera_id=camera,
            requested_by="perception_video_runtime", detector_backend_name=backend,
            open_vocabulary_classes=tuple(sorted(group["classes"])) or None)
        inputs = self._perception_service.get_fast_observation_input(context)
        if inputs is None:
            return
        image = inputs[0]
        # 使用原图像采集标识，复用同一帧不能产生新的连拍样本或关键帧。
        frame = (image.metadata.get("source_timestamp_ns", image.timestamp), image.metadata.get("source_sequence",
            image.metadata.get("ingest_sequence")))
        key = (context.camera_id, context.detector_backend)
        if self._seen_frames.get(key) == frame:
            return
        self._seen_frames[key] = frame
        decision = self._selector.decide(now=image.timestamp, current_pose=pose)
        sample = self.publish_sample(context, map_name=map_name, image_bytes=image.data, inputs=inputs)
        if self._event_bus is not None:
            self._event_bus.publish(self._event_bus.build_event("perception.scene_ready",
                category=RuntimeEventCategory.PERCEPTION, source="perception_video_runtime",
                subject_id=context.observation.observation_id, message=context.scene_summary.headline,
                payload={"camera_id": context.camera_id, "detector_backend": context.detector_backend,
                    "detection_count": len(context.observation.detections_2d),
                    "track_count": len(context.observation.tracks),
                    "stage_status": context.metadata.get("stage_status", {})}))
        with self._condition:
            bursts = [burst for burst in self._bursts if burst.reason in group["reasons"]]
        if decision.should_process or bursts:
            sample = self.complete_sample(sample)
            if decision.should_process:
                if self._remember_keyframes:
                    self._memory_service.remember_current_scene(map_name=sample.map_name,
                        perception_context=sample.context, observation_pose=sample.pose,
                        frame_tree=sample.frame_tree, image_bytes=sample.image_bytes,
                        metadata={"source": "perception_video_runtime", "keyframe_reason": decision.reason})
                self._selector.mark_processed(timestamp=image.timestamp, current_pose=pose)
        with self._condition:
            for burst in bursts:
                if burst in self._bursts and frame not in burst.frames and len(burst.samples) < burst.count:
                    burst.frames.add(frame)
                    burst.samples.append(sample)
            self._processed_frames += 1
            self._last_success = context.scene_summary.headline
            self._last_error = None
            self._condition.notify_all()

    def get_status(self):
        with self._condition:
            return PerceptionRuntimeStatus(running=bool(self._reasons) and not self._closed,
                reasons=sorted(self._reasons), camera_id=self._camera_id,
                interval_sec=min((value[0] for value in self._reasons.values()), default=self._interval_sec),
                detector_backend=self._detector_backend_name or self._perception_service.detector_pipeline.default_backend_name,
                processed_frames=self._processed_frames, failure_count=self._failure_count,
                last_success_message=self._last_success, last_error=self._last_error,
                metadata={"prewarm_status": dict(self._prewarm)})

    def apply_tuning(self, config):
        with self._condition:
            old_interval = self._interval_sec
            self._interval_sec = config.interval_sec
            self._failure_backoff_sec = config.failure_backoff_sec
            self._remember_keyframes = config.remember_keyframes
            self._burst_frame_count = config.burst_frame_count
            self._burst_interval_sec = config.burst_interval_sec
            self._reasons = {key: (config.interval_sec if value[0] == old_interval else value[0], *value[1:])
                             for key, value in self._reasons.items()}
            self._selector.update_thresholds(min_interval_sec=config.keyframe_min_interval_sec,
                translation_threshold_m=config.keyframe_translation_threshold_m,
                yaw_threshold_deg=config.keyframe_yaw_threshold_deg)
            self._condition.notify_all()

    def set_prewarm_status(self, status):
        with self._condition:
            self._prewarm = dict(status)

    def close(self):
        with self._condition:
            self._closed = True
            self._reasons.clear()
            self._stop.set()
            self._condition.notify_all()
            thread = self._thread
        if thread is not None:
            thread.join()
        with self._condition:
            self._samples.clear()
            self._seen_frames.clear()
        self._write_status()

    def _write_status(self):
        self._state_store.write(StateNamespace.SYSTEM, "perception_video_runtime", self.get_status(),
            source="perception_video_runtime", metadata={"kind": "perception_runtime_status"})
