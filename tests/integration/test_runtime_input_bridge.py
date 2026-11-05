"""统一输入桥经真实Geometry协议交付规范传感器字节；不恢复旧provider。"""
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import queue
import threading
from types import SimpleNamespace

import grpc
import numpy as np
import pytest

from contracts.geometry import Vector3
from contracts.pointcloud import PointCloudFrame
from contracts.sensors import CanonicalImuSample, CanonicalLidarFrame
from services.geometry.client import GeometryClient
from services.geometry.input_bridge import InputBridge
from services.geometry.proto_gen import geometry_runtime_pb2 as pb
from services.geometry.proto_gen import geometry_runtime_pb2_grpc as rpc
from services.geometry.state import RuntimeInfo


class SensorSource:
    def __init__(self):
        self.callback = None
        self.cloud_callback = None
        self.odom_callback = None
    def subscribe_sensor_sample(self, callback):
        self.callback = callback
        return callback
    def emit(self, sample):
        callback = self.callback
        if callback is not None:
            callback(sample)
    def unsubscribe_sensor_sample(self, subscription):
        assert self.callback is subscription
        self.callback = None
    def subscribe_point_cloud(self, callback):
        self.cloud_callback = callback
        return callback
    def unsubscribe_point_cloud(self, subscription):
        assert self.cloud_callback is subscription
        self.cloud_callback = None
    def subscribe_localization(self, callback):
        self.odom_callback = callback
        return callback
    def unsubscribe_localization(self, subscription):
        assert self.odom_callback is subscription
        self.odom_callback = None


def imu_sample(timestamp_ns=1000, sequence=1):
    return CanonicalImuSample(sensor_id='go2_imu', source_sequence=sequence, timestamp_ns=timestamp_ns,
        clock_domain='unix_ns', frame_id='go2_base_link', registration_state='body',
        registration_frame_id='go2_base_link', calibration_ref='test_go2_calibration',
        payload_ownership='inline', accel_mps2=(0., 0., 9.81), gyro_rps=(0., 0., 0.))


def lidar_sample(*, intensity=True, offsets=True, timestamp_ns=1000):
    xyz=np.asarray([[1.,2.,3.],[4.,5.,6.]],dtype='<f4')
    return CanonicalLidarFrame(sensor_id='go2_l2',source_sequence=timestamp_ns,timestamp_ns=timestamp_ns,
        clock_domain='unix_ns',frame_id='go2_base_link',registration_state='body',
        registration_frame_id='go2_base_link',calibration_ref='test_go2_calibration',payload_ownership='inline',
        point_count=2,xyz_f32_le=xyz.tobytes(),
        intensity_f32_le=np.asarray([.25,1.25],dtype='<f4').tobytes() if intensity else None,
        time_offset_ns_le=np.asarray([0,100],dtype='<u8').tobytes() if offsets else None,
        scan_duration_ns=100 if offsets else None,is_dense=True,
        transform_provenance='calibration_applied:test_go2_calibration')


class InputReceiver(rpc.GeometryRuntimeServicer):
    def __init__(self):
        self.clouds=queue.Queue()
        self.scans=queue.Queue()
        self.odometry=queue.Queue()
        self.accept=True
        self.block=False
        self.entered=threading.Event()
        self.release=threading.Event()
    def PublishPointCloud(self,request,context):
        self.entered.set()
        if self.block:
            while not self.release.wait(.01):
                if not context.is_active():
                    return pb.PublishResult(success=False)
        self.clouds.put(pb.PublishPointCloudRequest.FromString(request.SerializeToString()))
        return pb.PublishResult(success=self.accept)
    def PublishLaserScan(self,request,context):
        self.scans.put(request.payload)
        return pb.PublishResult(success=self.accept)
    def PublishOdom(self,request,context):
        self.odometry.put(pb.PublishOdomRequest.FromString(request.SerializeToString()))
        return pb.PublishResult(success=self.accept)


@pytest.fixture
def input_service():
    receiver=InputReceiver()
    server=grpc.server(ThreadPoolExecutor(max_workers=4))
    rpc.add_GeometryRuntimeServicer_to_server(receiver,server)
    port=server.add_insecure_port('127.0.0.1:0')
    server.start()
    client=GeometryClient(f'127.0.0.1:{port}')
    value=SimpleNamespace(client=client,receiver=receiver)
    try:
        yield value
    finally:
        receiver.release.set()
        client.close()
        server.stop(0).wait()


def info(pipeline='lio_3d'):
    return RuntimeInfo(pipeline,'map','odom','base_link','',0)


def wait_status(bridge,predicate,timeout=2):
    import time
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        value=bridge.status()
        if predicate(value):return value
        threading.Event().wait(.005)
    assert predicate(bridge.status()),bridge.status()


@pytest.mark.parametrize('intensity',[False,True])
def test_canonical_pair_without_device_localization_preserves_native_layout_and_time(input_service,intensity):
    source=SensorSource()
    bridge=InputBridge(client=input_service.client,info=info(),sensor_sample_provider=source)
    bridge.start()
    try:
        source.callback(imu_sample(900,1))
        lidar=lidar_sample(intensity=intensity)
        source.callback(lidar)
        source.callback(imu_sample(1100,2))
        request=input_service.receiver.clouds.get(timeout=2)
        assert request.timestamp_ns==lidar.timestamp_ns and request.frame_id=='go2_base_link'
        assert request.point_step==(20 if intensity else 16) and request.point_count==2
        dtype=[('x','<f4'),('y','<f4'),('z','<f4')]+([('intensity','<f4')] if intensity else [])+[('offset_ns','<u4')]
        points=np.frombuffer(request.points,dtype=dtype)
        assert list(points['x'])==[1.,4.] and list(points['y'])==[2.,5.] and list(points['z'])==[3.,6.]
        assert list(points['offset_ns'])==[0,100]
        if intensity:assert list(points['intensity'])==[.25,1.25]
        assert [value.timestamp_ns for value in request.imu_samples]==[900,1100]
        assert source.odom_callback is None
    finally:
        bridge.stop()
    assert source.callback is None


def test_missing_scan_offsets_are_not_replaced_with_invented_per_point_times(input_service):
    source=SensorSource()
    bridge=InputBridge(client=input_service.client,info=info(),sensor_sample_provider=source)
    bridge.start()
    try:
        source.callback(imu_sample(900,1))
        source.callback(lidar_sample(offsets=False))
        source.callback(imu_sample(1100,2))
        state=wait_status(bridge,lambda state:state['dropped']>0)
        assert state['last_error'] and input_service.receiver.clouds.empty()
    finally:bridge.stop()


def test_registered_cloud_preserves_exact_source_timestamp_and_sensor_origin(input_service):
    source=SensorSource()
    bridge=InputBridge(client=input_service.client,info=info(),point_cloud_provider=source)
    stamp=1783908184123456789
    cloud=PointCloudFrame(frame_id='odom',timestamp=datetime.fromtimestamp(stamp//1_000_000_000,timezone.utc),
        point_count=2,points_xyz_array=np.asarray([[1,2,3],[4,5,6]],dtype='<f4'),
        sensor_origin=Vector3(x=.1,y=.2,z=.3),metadata={'source':'go2_direct_dds_pointcloud',
            'source_frame':'utlidar_lidar','point_cloud_xyz_registered_to_odom':True,
            'registration_assertion':'explicit_config','registered_frame':'odom','source_timestamp_ns':stamp})
    bridge.start()
    try:
        source.cloud_callback(cloud)
        request=input_service.receiver.clouds.get(timeout=2)
        assert request.timestamp_ns==stamp
        assert request.frame_id=='odom' and list(request.sensor_origin_xyz)==pytest.approx([.1,.2,.3])
        assert request.points==np.asarray([[1,2,3],[4,5,6]],dtype='<f4').tobytes()
    finally:bridge.stop()


def test_send_failure_is_visible_and_next_real_frame_can_succeed(input_service):
    source=SensorSource()
    bridge=InputBridge(client=input_service.client,info=info(),point_cloud_provider=source)
    frame=PointCloudFrame(frame_id='map',point_count=1,points_xyz_array=np.asarray([[1,2,3]],dtype='<f4'))
    input_service.receiver.accept=False
    bridge.start()
    try:
        source.cloud_callback(frame)
        input_service.receiver.clouds.get(timeout=2)
        state=wait_status(bridge,lambda state:state['dropped']>0)
        assert state['last_error']=='Geometry 输入发送失败'
        input_service.receiver.accept=True
        source.cloud_callback(frame)
        accepted=input_service.receiver.clouds.get(timeout=2)
        assert accepted.points==np.asarray([[1,2,3]],dtype='<f4').tobytes()
        assert wait_status(bridge,lambda state:state['published']>0)['last_error'] is None
    finally:bridge.stop()


def test_stop_cancels_blocked_rpc_reclaims_subscription_and_rejects_late_source_frame(input_service):
    source=SensorSource()
    bridge=InputBridge(client=input_service.client,info=info(),point_cloud_provider=source)
    input_service.receiver.block=True
    bridge.start()
    callback=source.cloud_callback
    callback(PointCloudFrame(frame_id='map',point_count=1,points_xyz_array=np.zeros((1,3),dtype='<f4')))
    assert input_service.receiver.entered.wait(2)
    finished=threading.Event()
    stopper=threading.Thread(target=lambda:(bridge.stop(),finished.set()))
    stopper.start()
    try:
        assert finished.wait(2)
        assert source.cloud_callback is None and bridge.status()['ready'] is False
        callback(PointCloudFrame(frame_id='map',point_count=1,points_xyz_array=np.ones((1,3),dtype='<f4')))
        assert input_service.receiver.clouds.empty()
    finally:
        input_service.receiver.release.set()
        stopper.join(2)


def test_queue_overflow_keeps_bounded_original_frames_and_reports_drops(input_service):
    source=SensorSource()
    bridge=InputBridge(client=input_service.client,info=info(),point_cloud_provider=source)
    input_service.receiver.block=True
    bridge.start()
    try:
        for value in range(40):
            source.cloud_callback(PointCloudFrame(frame_id='map',point_count=1,
                points_xyz_array=np.asarray([[value,0,0]],dtype='<f4')))
            if value==0:assert input_service.receiver.entered.wait(2)
        state=bridge.status()
        assert state['dropped']>0
        input_service.receiver.release.set()
        wait_status(bridge,lambda status:status['published']>0)
        bridge.stop()
        frames=[]
        while not input_service.receiver.clouds.empty():
            frames.append(input_service.receiver.clouds.get_nowait())
        values=[np.frombuffer(frame.points,dtype='<f4')[0] for frame in frames]
        assert values==list(range(len(values))) and len(values)<=33
    finally:
        input_service.receiver.release.set()
        bridge.stop()


@pytest.mark.parametrize('intensity,offsets,step',[(False,False,12),(True,False,16),(False,True,16),(True,True,20)])
def test_registered_device_cloud_preserves_each_actual_field_layout(input_service,intensity,offsets,step):
    source=SensorSource()
    bridge=InputBridge(client=input_service.client,info=info(),point_cloud_provider=source)
    xyz=np.asarray([[1.,2.,3.],[4.,5.,6.]],dtype='<f4')
    metadata={}
    if intensity:metadata['intensity_array']=np.asarray([.25,1.25],dtype='<f4')
    if offsets:metadata['offset_ns_array']=np.asarray([0,100],dtype='<u4')
    cloud=PointCloudFrame(frame_id='odom',timestamp=datetime(2026,1,1,tzinfo=timezone.utc),
        point_count=2,points_xyz_array=xyz,metadata=metadata)
    bridge.start()
    try:
        source.cloud_callback(cloud)
        wire=input_service.receiver.clouds.get(timeout=2)
        names=['x','y','z']+(['intensity'] if intensity else [])+(['offset_ns'] if offsets else [])
        assert wire.point_step==step and wire.point_count==2
        assert [field.name for field in wire.fields]==names
        assert [field.offset for field in wire.fields]==list(range(0,step,4))
        dtype=np.dtype([(name,'<u4' if name=='offset_ns' else '<f4') for name in names])
        decoded=np.frombuffer(wire.points,dtype=dtype)
        assert list(decoded['x'])==[1.,4.] and list(decoded['y'])==[2.,5.] and list(decoded['z'])==[3.,6.]
        if intensity:assert list(decoded['intensity'])==[.25,1.25]
        if offsets:assert list(decoded['offset_ns'])==[0,100]
    finally:bridge.stop()


def test_canonical_source_keeps_device_cloud_only_as_mapping_observation(input_service):
    source=SensorSource()
    bridge=InputBridge(client=input_service.client,info=info(),sensor_sample_provider=source,point_cloud_provider=source)
    bridge.start()
    try:
        xyz=np.asarray([[1.,2.,3.]],dtype='<f4')
        cloud=PointCloudFrame(frame_id='odom',timestamp=datetime(2026,1,1,tzinfo=timezone.utc),point_count=1,points_xyz_array=xyz)
        source.cloud_callback(cloud)
        source.emit(imu_sample(900))
        source.emit(lidar_sample(timestamp_ns=1000))
        source.emit(imu_sample(1100,2))
        canonical=input_service.receiver.clouds.get(timeout=2)
        assert canonical.frame_id=='go2_base_link' and not canonical.mapping_only
        assert canonical.point_count==2 and canonical.imu_samples
        source.cloud_callback(cloud.model_copy(update={'mapping_only':True}))
        auxiliary=input_service.receiver.clouds.get(timeout=2)
        assert auxiliary.mapping_only and auxiliary.frame_id=='odom'
        assert auxiliary.points==xyz.tobytes() and not auxiliary.imu_samples
        assert input_service.receiver.clouds.empty()
    finally:bridge.stop()
