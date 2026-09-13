"""二维标准输入桥：回调只排入有界队列，由单一发布线程送入已绑定的建图会话。"""
from __future__ import annotations

from collections import deque
from datetime import timezone
import json
import os
import threading
import time
from typing import Any, Dict, Optional

from contracts.sensors import CanonicalLaserScan
from providers.sensors import ManagedSensorSource


class PlanarRuntimeInputBridge:
    def __init__(self, *, runtime_provider, sensor_sample_provider=None, localization_provider=None,
                 sensor_source_owner: Optional[ManagedSensorSource] = None):
        self.runtime_provider = runtime_provider
        self.sensor_sample_provider = sensor_sample_provider
        self.localization_provider = localization_provider
        self._condition = threading.Condition()
        self._lifecycle = threading.RLock()
        self._started = False
        self._generation = 0
        self._thread = None
        self._scan_subscription = None
        self._odom_subscription = None
        self._source_owner = sensor_source_owner
        self._queue = deque()
        self._runtime_generation = 0
        self._unix_offset_ns = 0
        self._last_error = None
        self._counters = {"published_laser_scan_count": 0, "published_odom_count": 0,
                          "dropped_count": 0, "last_publish_latency_ms": 0.0}
        self._settings = json.loads(os.environ.get("NUWAX_GEOMETRY_SLAM2D_CONFIG", "{}"))

    def start(self) -> bool:
        with self._lifecycle:
            with self._condition:
                if self._started:
                    return self.is_ready()
                if self._thread is not None:
                    raise RuntimeError("二维输入桥旧发布线程尚未回收")
            live = self.runtime_provider.get_geometry_pipeline()
            if not live.get("healthy") or live["geometry_pipeline"] != "slam_2d" or live["runtime_generation"] <= 0:
                raise RuntimeError("实际Geometry进程未健康就绪或未选择二维执行链")
            self._settings = dict(live.get("planar_input_config") or self._settings)
            self._unix_offset_ns = int(live.get("planar_unix_offset_ns") or 0)
            if self._settings.get("odometry_source") not in ("laser", "device"):
                raise ValueError("二维里程计来源配置缺失")
            if self._settings["odometry_source"] == "device" and self.localization_provider is None:
                raise ValueError("设备里程计模式缺少标准定位源")
            with self._condition:
                self._runtime_generation = live["runtime_generation"]
                self._generation += 1
                generation = self._generation
                self._started = True
                self._last_error = None
                self._queue.clear()
                self._thread = threading.Thread(target=self._publish_loop, args=(generation,),
                                                name="geometry-planar-input", daemon=False)
                self._thread.start()
            try:
                if self.sensor_sample_provider is None:
                    raise RuntimeError("二维链缺少已装配的标准扫描提供器")
                self._scan_subscription = self.sensor_sample_provider.subscribe_sensor_sample(
                    lambda scan: self._scan(scan, generation))
                if self._scan_subscription is None:
                    raise RuntimeError("二维规范扫描订阅失败")
                if self._source_owner is not None:
                    self._source_owner.start()
                if self._settings["odometry_source"] == "device":
                    self._odom_subscription = self.localization_provider.subscribe_localization(
                        lambda pose, tree, metadata: self._odom(pose, metadata, generation))
                    if self._odom_subscription is None:
                        raise RuntimeError("二维设备里程计订阅失败")
                return True
            except Exception:
                self.stop()
                raise

    def _capture(self, generation):
        with self._condition:
            if not self._started or generation != self._generation:
                return None
            runtime_generation = self._runtime_generation
        return self.runtime_provider.capture_planar_input_context(runtime_generation)

    def _enqueue(self, kind, sample, context, generation):
        with self._condition:
            if not self._started or generation != self._generation:
                return
            if kind == "scan":
                # 在途之外保留最新扫描；设备里程计仍按真实采样顺序承接。
                retained = deque(item for item in self._queue if item[0] != "scan")
                self._counters["dropped_count"] += len(self._queue) - len(retained)
                self._queue = retained
            if len(self._queue) >= 512:
                self._queue.popleft()
                self._counters["dropped_count"] += 1
            self._queue.append((kind, sample, context))
            self._condition.notify_all()

    def _scan(self, scan, generation):
        if not isinstance(scan, CanonicalLaserScan):
            return True
        context = self._capture(generation)
        if context is not None:
            self._enqueue("scan", scan, context, generation)
        # 建图会话尚未创建时丢弃实时帧，不让驱动误判通信失败。
        return True

    def _odom(self, pose, metadata, generation):
        context = self._capture(generation)
        if context is None:
            return
        metadata = dict(metadata or {})
        domain = metadata.get("clock_domain", "unix_ns")
        clock_id = str(metadata.get("clock_id") or "")
        stamp = metadata.get("timestamp_ns")
        if stamp is None and domain == "unix_ns":
            stamp = round(pose.timestamp.astimezone(timezone.utc).timestamp() * 1e9)
        if not isinstance(stamp, int) or isinstance(stamp, bool) or stamp <= 0:
            with self._condition:
                self._last_error = "二维里程计缺少真实采样时间"
                self._counters["dropped_count"] += 1
            return
        # 使用Geometry启动时捕获的同一时钟映射，统一里程计与扫描的配对时基。
        if domain == "unix_ns" and self._settings.get("clock_domain") == "monotonic_ns":
            stamp -= self._unix_offset_ns
            domain = "monotonic_ns"
            clock_id = self._settings["clock_id"]
        if stamp <= 0 or domain != self._settings["clock_domain"] or clock_id != self._settings["clock_id"]:
            with self._condition:
                self._last_error = "二维里程计时钟无法映射到已绑定扫描来源"
                self._counters["dropped_count"] += 1
            return
        self._enqueue("odom", (pose, stamp, domain, clock_id), context, generation)

    def _publish_loop(self, generation):
        while True:
            with self._condition:
                while self._started and generation == self._generation and not self._queue:
                    self._condition.wait()
                if not self._started or generation != self._generation:
                    return
                kind, sample, context = self._queue.popleft()
            started = time.monotonic()
            try:
                if kind == "scan":
                    accepted = self.runtime_provider.publish_laser_scan(sample, context)
                    counter = "published_laser_scan_count"
                else:
                    accepted = self.runtime_provider.publish_planar_odometry(*sample, context=context)
                    counter = "published_odom_count"
                if not accepted:
                    raise RuntimeError("二维输入发布被运行时拒绝")
                with self._condition:
                    self._counters[counter] += 1
                    self._counters["last_publish_latency_ms"] = (time.monotonic() - started) * 1000
                    self._last_error = None
            except Exception as error:
                with self._condition:
                    self._counters["dropped_count"] += 1
                    self._last_error = str(error)

    def stop(self, **_kwargs) -> None:
        with self._lifecycle:
            with self._condition:
                self._started = False
                self._queue.clear()
                self._condition.notify_all()
                worker = self._thread
            errors = []
            for attr, provider, method in (
                ("_scan_subscription", self.sensor_sample_provider, "unsubscribe_sensor_sample"),
                ("_odom_subscription", self.localization_provider, "unsubscribe_localization"),
            ):
                subscription = getattr(self, attr)
                if subscription is not None:
                    try:
                        getattr(provider, method)(subscription)
                        setattr(self, attr, None)
                    except Exception as error:
                        errors.append(str(error))
            if self._source_owner is not None:
                try:
                    self._source_owner.stop()
                except Exception as error:
                    errors.append(str(error))
            if worker is threading.current_thread():
                errors.append("二维发布线程不能同步等待自身退出")
            elif worker is not None:
                worker.join(timeout=6.0)
                if worker.is_alive():
                    errors.append("二维发布线程尚未退出，保留拥有方供后续清理")
                else:
                    self._thread = None
            if errors:
                raise RuntimeError("；".join(errors))

    def is_ready(self) -> bool:
        with self._condition:
            source_ready = self._source_owner is None or self._source_owner.is_ready()
            available = getattr(self.sensor_sample_provider, "is_available", None)
            provider_ready = not callable(available) or available()
            return bool(self._started and self._thread is not None and self._thread.is_alive()
                        and self._scan_subscription is not None and source_ready and provider_ready)

    def get_status(self) -> Dict[str, Any]:
        with self._condition:
            return {"started": self._started, "ready": self.is_ready(), "geometry_pipeline": "slam_2d",
                    "runtime_generation": self._runtime_generation,
                    "last_error": self._last_error, "counters": dict(self._counters)}
