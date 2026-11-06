"""把通用与采集进程的规范传感器流交给同一个 Geometry 客户端。"""
from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
import json
import logging
import threading
from typing import Any

import numpy as np

from adapters.sensors.lio_measurement import LioMeasurementAssembler, LioMeasurementAssemblerError
from contracts.sensors import (
    CanonicalImuSample, CanonicalLaserScan, CanonicalLidarFrame, CanonicalLioMeasurement,
    SensorStreamFailure,
)
from services.geometry.planar_wire import encode_planar_scan
from services.geometry.proto_gen import geometry_runtime_pb2 as pb
from services.geometry.state import Pose

LOGGER = logging.getLogger(__name__)


def _datetime_ns(timestamp):
    delta = timestamp.astimezone(timezone.utc) - datetime(1970, 1, 1, tzinfo=timezone.utc)
    return (delta.days * 86400 + delta.seconds) * 1_000_000_000 + delta.microseconds * 1000


class InputBridge:
    def __init__(self, *, client, info, sensor_sample_provider=None, localization_provider=None,
                 point_cloud_provider=None, sensor_source_owner=None, recording_sink=None, input_executor=None,
                 lio_backend="device"):
        self._client = client
        self._info = info
        self._sensor_provider = sensor_sample_provider
        self._localization_provider = localization_provider
        self._point_cloud_provider = point_cloud_provider
        self._source_owner = sensor_source_owner
        self._recording = recording_sink
        self._lio_backend = lio_backend
        self._native_lio = False
        self._input_executor = input_executor
        self._executor = None
        self._sender = None
        self._source_started = False
        self._condition = threading.Condition()
        self._lifecycle = threading.RLock()
        self._started = False
        self._running = False
        self._worker = None
        self._subscriptions = []
        self._queue = deque()
        self._assembler = LioMeasurementAssembler()
        self._settings = {}
        self._runtime_info = None
        self._last_error = None
        self._published = 0
        self._dropped = 0

    def start(self):
        with self._lifecycle:
            if self._started:
                return
            try:
                self._runtime_info = self._info() if callable(self._info) else self._info
                self._settings = json.loads(self._runtime_info.planar_input_config_json or "{}")
                self._native_lio = self._runtime_info.pipeline == "lio_3d" and self._lio_backend != "device"
                self._assembler = LioMeasurementAssembler()
                self._last_error = None
                self._running = True
                self._executor = self._input_executor() if callable(self._input_executor) else self._input_executor
                if self._executor is not None:
                    self._executor.start_input_delivery(_ProcessDelivery, {
                        "target": self._client.target, "info": self._runtime_info,
                        "recording": self._recording is not None, "lio_backend": self._lio_backend,
                    }, self._recording)
                else:
                    self._sender = self._client.input_sender()
                    worker = threading.Thread(target=self._publish, args=(self._sender,),
                                              name="geometry-input", daemon=True)
                    worker.start()
                    self._worker = worker
                    if self._recording is not None:
                        self._recording.record_event("input_bridge.started", {"pipeline": self._runtime_info.pipeline})
                    if self._sensor_provider is not None:
                        subscription = self._sensor_provider.subscribe_sensor_sample(self._on_sample)
                        self._subscriptions.append((self._sensor_provider, "unsubscribe_sensor_sample", subscription))
                    if (self._localization_provider is not None
                            and (self._runtime_info.pipeline == "lio_3d" and not self._native_lio
                                 or self._runtime_info.pipeline == "slam_2d"
                                 and self._settings.get("odometry_source") == "device")):
                        subscription = self._localization_provider.subscribe_localization(self._on_odometry)
                        self._subscriptions.append((self._localization_provider, "unsubscribe_localization", subscription))
                    if self._runtime_info.pipeline == "lio_3d" and self._point_cloud_provider is not None:
                        subscription = self._point_cloud_provider.subscribe_point_cloud(self._on_point_cloud)
                        self._subscriptions.append((self._point_cloud_provider, "unsubscribe_point_cloud", subscription))
                    if self._source_owner is not None:
                        self._source_started = True
                        self._source_owner.start()
                self._started = True
            except BaseException:
                try:
                    self.stop()
                except BaseException:
                    LOGGER.exception("清理未启动的 Geometry 输入桥失败")
                raise

    def stop(self):
        with self._lifecycle:
            with self._condition:
                self._running = False
                self._queue.clear()
                self._condition.notify_all()
            errors = []
            if self._sender is not None:
                self._sender.cancel()
            if self._executor is not None:
                try:
                    self._executor.stop_input_delivery()
                    status = self._executor.input_delivery_status()
                    self._published = status.get("published", self._published)
                    self._dropped = status.get("dropped", self._dropped)
                    self._last_error = status.get("last_error")
                except BaseException as error:
                    errors.append(error)
                finally:
                    self._executor = None
            for provider, method, subscription in self._subscriptions:
                try:
                    getattr(provider, method)(subscription)
                except BaseException as error:
                    errors.append(error)
            self._subscriptions.clear()
            if self._source_started:
                try:
                    self._source_owner.stop()
                except BaseException as error:
                    errors.append(error)
                finally:
                    self._source_started = False
            if self._worker is not None and self._worker is not threading.current_thread():
                self._worker.join()
            self._worker = self._sender = None
            self._started = False
            if errors:
                raise errors[0]

    def status(self):
        with self._condition:
            result = {"started": self._started, "ready": self._running,
                      "pipeline": None if self._runtime_info is None else self._runtime_info.pipeline,
                      "published": self._published, "dropped": self._dropped, "last_error": self._last_error}
        executor = self._executor
        if executor is not None:
            result.update(executor.input_delivery_status())
            result["started"] = self._started
            result["ready"] = self._running and result.get("ready", False)
        return result

    def _enqueue(self, kind, value):
        with self._condition:
            if not self._running:
                return
            # 沿用原输入桥的帧数上限；满队列直接丢弃新帧。
            if len(self._queue) >= 32:
                self._dropped += 1
                return
            self._queue.append((kind, value))
            self._condition.notify_all()

    def _on_sample(self, sample):
        with self._condition:
            if not self._running:
                return
            if isinstance(sample, SensorStreamFailure):
                self._last_error = sample.reason
                return
            # 原文入队与停止共用运行状态锁，停止返回后不会再追加记录尾部。
            if self._recording is not None:
                self._recording.record_sensor(sample)
        if isinstance(sample, CanonicalLaserScan):
            self._enqueue("scan", encode_planar_scan(sample))
        elif self._native_lio and isinstance(sample, CanonicalLioMeasurement):
            self._enqueue("cloud", self._measurement_request(sample))
        elif self._native_lio and isinstance(sample, (CanonicalImuSample, CanonicalLidarFrame)):
            try:
                measurement = (self._assembler.add_imu(sample) if isinstance(sample, CanonicalImuSample)
                               else self._assembler.add_lidar(sample))
                while measurement is not None:
                    self._enqueue("cloud", self._measurement_request(measurement))
                    measurement = self._assembler.pop_ready()
            except (LioMeasurementAssemblerError, ValueError) as error:
                with self._condition:
                    self._dropped += 1
                    self._last_error = str(error)
        return True

    def _on_odometry(self, pose, tree, metadata):
        if self._runtime_info.pipeline == "slam_2d":
            domain, clock_id, stamp = metadata["clock_domain"], metadata["clock_id"], metadata["timestamp_ns"]
            if domain == "unix_ns" and self._settings["clock_domain"] == "monotonic_ns":
                stamp -= self._runtime_info.planar_unix_offset_ns
                domain = "monotonic_ns"
                clock_id = self._settings["clock_id"]
        else:
            # 同帧规范惯性元数据保留精确时钟；普通设备位姿沿用显式元数据或自身UTC时间。
            metadata = metadata or {}
            sample = metadata.get("canonical_imu_sample")
            if sample is not None and "timestamp_ns" not in metadata:
                domain, clock_id, stamp = sample.clock_domain.value, sample.clock_id or "", sample.timestamp_ns
            else:
                domain, clock_id = metadata.get("clock_domain", "unix_ns"), metadata.get("clock_id", "")
                stamp = metadata.get("timestamp_ns", _datetime_ns(pose.timestamp))
        value = Pose(pose.position.x, pose.position.y, pose.position.z,
                     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                     pose.frame_id, stamp)
        self._enqueue("odom", (value, domain, clock_id))

    def _on_point_cloud(self, cloud):
        # 只有实际原生LIO配组才跳过显示主云；device仍须交付设备注册的主云。
        if self._native_lio and self._sensor_provider is not None and not cloud.mapping_only:
            return
        metadata = cloud.metadata
        xyz = cloud.points_xyz_array
        if xyz is None:
            xyz = np.asarray([(p.x, p.y, p.z) for p in cloud.points], dtype="<f4")
        values = [("x", "<f4", np.asarray(xyz)[:, 0]),
                  ("y", "<f4", np.asarray(xyz)[:, 1]),
                  ("z", "<f4", np.asarray(xyz)[:, 2])]
        if metadata.get("intensity_array") is not None:
            values.append(("intensity", "<f4", metadata["intensity_array"]))
        if metadata.get("offset_ns_array") is not None:
            values.append(("offset_ns", "<u4", metadata["offset_ns_array"]))
        request = self._pack_cloud(values, cloud.point_count)
        request.frame_id = cloud.frame_id
        if "source_timestamp_ns" in metadata:
            request.timestamp_ns = metadata["source_timestamp_ns"]
        else:
            request.timestamp_ns = _datetime_ns(cloud.timestamp)
        request.is_dense = cloud.is_dense
        request.mapping_only = cloud.mapping_only
        request.sensor_origin_xyz.extend((cloud.sensor_origin.x, cloud.sensor_origin.y, cloud.sensor_origin.z))
        for field in ("sensor_id", "source_sequence", "clock_id", "calibration_ref"):
            if field in metadata:
                setattr(request, field, metadata[field])
        self._enqueue("cloud", request)

    @staticmethod
    def _pack_cloud(values, count):
        dtype = np.dtype([(name, kind) for name, kind, _ in values])
        packed = np.empty(count, dtype=dtype)
        fields = []
        for name, kind, array in values:
            packed[name] = array
            fields.append(pb.PointCloudField(name=name, offset=dtype.fields[name][1],
                                            datatype=7 if kind == "<f4" else 6, count=1))
        return pb.PublishPointCloudRequest(points=packed.tobytes(), point_step=dtype.itemsize,
                                          fields=fields, point_count=count)

    @classmethod
    def _measurement_request(cls, measurement):
        lidar = measurement.lidar
        xyz = np.frombuffer(lidar.xyz_f32_le, dtype="<f4").reshape((-1, 3))
        values = [("x", "<f4", xyz[:, 0]), ("y", "<f4", xyz[:, 1]), ("z", "<f4", xyz[:, 2])]
        if lidar.intensity_f32_le is not None:
            values.append(("intensity", "<f4", np.frombuffer(lidar.intensity_f32_le, dtype="<f4")))
        offsets = np.frombuffer(lidar.time_offset_ns_le, dtype="<u8")
        if np.any(offsets > np.iinfo(np.uint32).max):
            raise ValueError("逐点时间偏移超过原生点云 uint32 编码范围")
        values.append(("offset_ns", "<u4", offsets))
        request = cls._pack_cloud(values, lidar.point_count)
        request.frame_id = lidar.registration_frame_id
        request.timestamp_ns = lidar.timestamp_ns
        request.is_dense = lidar.is_dense
        request.sensor_id = lidar.sensor_id
        request.source_sequence = lidar.source_sequence
        request.clock_id = lidar.clock_id or ""
        request.calibration_ref = lidar.calibration_ref
        for sample in measurement.imu_samples:
            request.imu_samples.add(timestamp_ns=sample.timestamp_ns, frame_id=sample.registration_frame_id,
                accel_mps2_x=sample.accel_mps2[0], accel_mps2_y=sample.accel_mps2[1], accel_mps2_z=sample.accel_mps2[2],
                gyro_rps_x=sample.gyro_rps[0], gyro_rps_y=sample.gyro_rps[1], gyro_rps_z=sample.gyro_rps[2])
        return request

    def _publish(self, sender):
        while True:
            with self._condition:
                while self._running and not self._queue:
                    self._condition.wait()
                if not self._running:
                    return
                kind, value = self._queue.popleft()
            if kind == "cloud":
                accepted = sender.publish_point_cloud(value)
            elif kind == "scan":
                accepted = sender.publish_laser_scan(value)
            else:
                pose, domain, clock_id = value
                accepted = sender.publish_odom(pose, clock_domain=domain, clock_id=clock_id)
            with self._condition:
                if accepted:
                    self._published += 1
                    self._last_error = None
                else:
                    self._dropped += 1
                    self._last_error = "Geometry 输入发送失败"


class _RecordingRelay:
    """采集进程只移交原始记录，SQLite写入者仍由网关唯一拥有。"""

    def __init__(self, emit):
        self._emit = emit

    def record_sensor(self, sample):
        return self._emit("sensor", sample)

    def record_event(self, kind, metadata, *, source_ns=0):
        return self._emit("event", (kind, metadata, source_ns))


class _ProcessDelivery:
    """在已有Go2采集进程里组合相同输入桥，不建立第二套配组或投递算法。"""

    def __init__(self, spec, record, source):
        from services.geometry.client import GeometryClient
        self._client = GeometryClient(spec["target"])
        self._bridge = InputBridge(client=self._client, info=spec["info"],
            sensor_sample_provider=source,
            localization_provider=source if hasattr(source, "subscribe_localization") else None,
            point_cloud_provider=source if hasattr(source, "subscribe_point_cloud") else None,
            recording_sink=_RecordingRelay(record) if spec["recording"] else None,
            lio_backend=spec["lio_backend"])
        try:
            self._bridge.start()
        except BaseException:
            self._client.close()
            raise

    def status(self):
        return self._bridge.status()

    def stop(self):
        try:
            self._bridge.stop()
        finally:
            self._client.close()
