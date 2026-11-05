"""把通用与采集进程的规范传感器流交给同一个 Geometry 客户端。"""
from __future__ import annotations

from collections import deque
from datetime import timezone
import json
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


class InputBridge:
    def __init__(self, *, client, info, sensor_sample_provider=None, localization_provider=None,
                 point_cloud_provider=None, sensor_source_owner=None, recording_sink=None):
        self._client = client
        self._info = info
        self._sensor_provider = sensor_sample_provider
        self._localization_provider = localization_provider
        self._point_cloud_provider = point_cloud_provider
        self._source_owner = sensor_source_owner
        self._recording = recording_sink
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
            self._runtime_info = self._info() if callable(self._info) else self._info
            self._settings = json.loads(self._runtime_info.planar_input_config_json or "{}")
            self._running = True
            self._worker = threading.Thread(target=self._publish, name="geometry-input", daemon=True)
            self._worker.start()
            try:
                if self._sensor_provider is not None:
                    subscription = self._sensor_provider.subscribe_sensor_sample(self._on_sample)
                    self._subscriptions.append((self._sensor_provider, "unsubscribe_sensor_sample", subscription))
                if (self._runtime_info.pipeline == "slam_2d"
                        and self._settings.get("odometry_source") == "device"
                        and self._localization_provider is not None):
                    subscription = self._localization_provider.subscribe_localization(self._on_odometry)
                    self._subscriptions.append((self._localization_provider, "unsubscribe_localization", subscription))
                if self._runtime_info.pipeline == "lio_3d" and self._point_cloud_provider is not None:
                    subscription = self._point_cloud_provider.subscribe_point_cloud(self._on_point_cloud)
                    self._subscriptions.append((self._point_cloud_provider, "unsubscribe_point_cloud", subscription))
                if self._source_owner is not None:
                    self._source_owner.start()
                self._started = True
            except BaseException:
                self.stop()
                raise

    def stop(self):
        with self._lifecycle:
            with self._condition:
                self._running = False
                self._queue.clear()
                self._condition.notify_all()
            self._client._close_inputs()
            try:
                for provider, method, subscription in self._subscriptions:
                    getattr(provider, method)(subscription)
                self._subscriptions.clear()
                if self._source_owner is not None:
                    self._source_owner.stop()
            finally:
                if self._worker is not None and self._worker is not threading.current_thread():
                    self._worker.join()

    def status(self):
        with self._condition:
            return {"started": self._started, "ready": self._running,
                    "pipeline": None if self._runtime_info is None else self._runtime_info.pipeline,
                    "published": self._published, "dropped": self._dropped, "last_error": self._last_error}

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
        if isinstance(sample, SensorStreamFailure):
            with self._condition:
                self._last_error = sample.reason
            return
        if self._recording is not None:
            self._recording.record_sensor(sample)
        if isinstance(sample, CanonicalLaserScan):
            self._enqueue("scan", encode_planar_scan(sample))
        elif isinstance(sample, CanonicalLioMeasurement):
            self._enqueue("cloud", self._measurement_request(sample))
        elif isinstance(sample, (CanonicalImuSample, CanonicalLidarFrame)):
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
        domain = metadata["clock_domain"]
        clock_id = metadata["clock_id"]
        stamp = metadata["timestamp_ns"]
        if domain == "unix_ns" and self._settings["clock_domain"] == "monotonic_ns":
            stamp -= self._runtime_info.planar_unix_offset_ns
            domain = "monotonic_ns"
            clock_id = self._settings["clock_id"]
        value = Pose(pose.position.x, pose.position.y, pose.position.z,
                     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                     pose.frame_id, stamp)
        self._enqueue("odom", (value, domain, clock_id))

    def _on_point_cloud(self, cloud):
        # 规范 LiDAR 已从传感器流配组；点云显示流只补充辅助地图观测。
        if self._sensor_provider is not None and not cloud.mapping_only:
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
        request.timestamp_ns = round(cloud.timestamp.astimezone(timezone.utc).timestamp() * 1e9)
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

    def _publish(self):
        while True:
            with self._condition:
                while self._running and not self._queue:
                    self._condition.wait()
                if not self._running:
                    return
                kind, value = self._queue.popleft()
            if kind == "cloud":
                accepted = self._client.publish_point_cloud(value)
            elif kind == "scan":
                accepted = self._client.publish_laser_scan(value)
            else:
                pose, domain, clock_id = value
                accepted = self._client.publish_odom(pose, clock_domain=domain, clock_id=clock_id)
            with self._condition:
                if accepted:
                    self._published += 1
                    self._last_error = None
                else:
                    self._dropped += 1
                    self._last_error = "Geometry 输入发送失败"
