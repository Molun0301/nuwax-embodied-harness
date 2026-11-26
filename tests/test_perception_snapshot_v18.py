"""完整观察保留原图与模型等待前的几何，通知后可直接消费。"""
from contextlib import contextmanager
from dataclasses import replace
from types import SimpleNamespace
import threading

import pytest

from contracts.geometry import Pose, Vector3
from contracts.image import CameraInfo
from contracts.pointcloud import PointCloudFrame
from services.geometry.client import Transform
from services.geometry.state import Pose as GeometryPose
from tests.test_a2c_artifact_lifecycle import perception, frame
from tests.test_a2c_memory_core import Geometry
from tests.test_a2d_memory_inputs import memory_at


def test_capture_is_frozen_before_model_wait_and_published_with_result(tmp_path, monkeypatch):
    with perception(tmp_path) as p:
        geometry=Geometry('house')
        stamp=int(p.current[0].timestamp.timestamp()*1e9)
        geometry.value=replace(geometry.value,pose=GeometryPose(2,0,0,0,0,0,1,'map',stamp))
        p.service._geometry_state=geometry
        p.service._stream_bus.publish('localization.pose',source='geometry',payload=geometry.value.pose,
            metadata={'map_name':'house','source_timestamp_ns':stamp})
        entered,release=threading.Event(),threading.Event()
        detect=p.service._detector_pipeline.detect
        def blocked(*args,**kwargs):
            entered.set();assert release.wait(3)
            return detect(*args,**kwargs)
        monkeypatch.setattr(p.service._detector_pipeline,'detect',blocked)
        results,errors=[],[]
        camera=CameraInfo(camera_id='front',frame_id='camera/front',width_px=64,height_px=48,
            fx=40,fy=40,cx=32,cy=24)
        def run():
            try:results.append(p.service.process_image(p.current[0],camera))
            except BaseException as error:errors.append(error)
        worker=threading.Thread(target=run);worker.start()
        try:
            assert entered.wait(2)
            geometry.value=replace(geometry.value,pose=replace(geometry.value.pose,x=99))
        finally:
            release.set();worker.join(3)
        assert not errors and not worker.is_alive()
        image,info,pose,tree=p.service.get_full_observation_input(results[0])
        assert image.data==p.current[0].data and info==camera
        assert pose.position.x==2 and pose.timestamp==p.current[0].timestamp


def test_latest_full_input_replaces_same_camera_and_close_clears_it(tmp_path):
    with perception(tmp_path) as p:
        first=p.service.perceive_current_scene()
        assert p.service.get_full_observation_input(first)[0].data==p.current[0].data
        p.current[0]=frame(seconds=1)
        second=p.service.perceive_current_scene()
        assert p.service.get_full_observation_input(first) is None
        assert p.service.get_full_observation_input(second)[0].data==p.current[0].data
        p.service.close()
        assert p.service.get_full_observation_input(second) is None
        assert p.service.get_latest_full_perception() is None


def test_subscriber_observes_same_frame_input_outside_both_pipeline_locks(tmp_path):
    with perception(tmp_path) as p:
        received=[]
        def receive(event):
            if event.event_type!='perception.scene_ready':return
            context=p.service.get_latest_full_perception()
            received.append((p.service.get_full_observation_input(context),
                p.service._full_pipeline_lock.locked(),p.service._pipeline_lock.locked()))
        p.events.subscribe(receive)
        context=p.service.perceive_current_scene()
        inputs,full_locked,detector_locked=received[0]
        assert not full_locked and not detector_locked
        assert inputs is not None
        assert inputs[0].data==p.current[0].data
        assert context.observation.observation_id==p.service.get_latest_full_perception().observation.observation_id


def test_missing_captured_pose_is_not_replaced_by_later_geometry(tmp_path):
    with perception(tmp_path/'perception') as p:
        context=p.service.perceive_current_scene()
        with memory_at(tmp_path/'memory',pose=Pose(frame_id='map',position=Vector3(x=99))) as (memory,_):
            entry=memory.remember_current_scene(map_name='house',perception_context=context,
                observation_pose=None,image_bytes=p.current[0].data)
            assert entry.pose is None
            assert entry.observation_id==context.observation.observation_id
            assert memory.read_photo(entry.artifact_ids[0])==p.current[0].data


def test_fast_observation_projects_with_cloud_captured_before_detection(tmp_path, monkeypatch):
    with perception(tmp_path) as p:
        image = p.current[0]
        stamp = int(image.timestamp.timestamp() * 1e9)
        camera = CameraInfo(camera_id="front", frame_id="camera/front", width_px=64, height_px=48,
                            fx=40, fy=40, cx=32, cy=24)
        geometry = Geometry("house")
        measured_pose = GeometryPose(0, 0, 0, 0, 0, 0, 1, "map", stamp)
        geometry.value = replace(geometry.value, pose=measured_pose)
        p.service._geometry_state = geometry
        monkeypatch.setattr(p.service, "_geometry_transforms", lambda: (
            Transform(parent_frame="map", child_frame="base", x=0, y=0, z=0, qx=0, qy=0, qz=0, qw=1, timestamp_ns=stamp),
            Transform(parent_frame="base", child_frame="camera/front", x=0, y=0, z=0, qx=0, qy=0, qz=0, qw=1, timestamp_ns=0),
        ))
        p.streams.publish("localization.pose", source="geometry", payload=measured_pose,
                          metadata={"map_name": "house", "source_timestamp_ns": stamp})
        points = [Vector3(x=-0.29 + step * 0.01, y=-0.05, z=1.0) for step in range(6)]

        def publish_cloud(source_stamp, coordinates):
            cloud = PointCloudFrame(frame_id="map", timestamp=image.timestamp, points=coordinates, point_count=len(coordinates),
                                    metadata={"source_timestamp_ns": source_stamp})
            p.streams.publish("pointcloud.frame", source="robot", payload=cloud, metadata={"map_name": "house"})

        publish_cloud(stamp, points)
        detect = p.service._detector_pipeline.detect

        def detect_after_cloud_updates(*args, **kwargs):
            for index in range(1, 23):
                publish_cloud(stamp + index * 100_000_000, [Vector3(x=5, y=5, z=5)])
            return detect(*args, **kwargs)

        monkeypatch.setattr(p.service._detector_pipeline, "detect", detect_after_cloud_updates)
        context = p.service.process_image_fast(image, camera)
        completed, _, _ = p.service.complete_observation_geometry(context, p.service.get_fast_observation_input(context))
        detection = completed.observation.detections_2d[0]
        assert detection.attributes["geometry_support_point_count"] == len(points)
        assert detection.attributes["support_cluster_summary"]["center"] == pytest.approx([-0.265, -0.05, 1.0])


@pytest.mark.parametrize("fast", [False, True])
def test_capture_keeps_preceding_and_following_scans_until_projection(tmp_path, monkeypatch, fast):
    with perception(tmp_path) as p:
        image = p.current[0]
        stamp = int(image.timestamp.timestamp() * 1e9)
        camera = CameraInfo(camera_id="front", frame_id="camera/front", width_px=64, height_px=48,
                            fx=40, fy=40, cx=32, cy=24)
        geometry = Geometry("house")
        measured_pose = GeometryPose(0, 0, 0, 0, 0, 0, 1, "map", stamp)
        geometry.value = replace(geometry.value, pose=measured_pose)
        p.service._geometry_state = geometry
        monkeypatch.setattr(p.service, "_geometry_transforms", lambda: (
            Transform(parent_frame="map", child_frame="base", x=0, y=0, z=0, qx=0, qy=0, qz=0, qw=1, timestamp_ns=stamp),
            Transform(parent_frame="base", child_frame="camera/front", x=0, y=0, z=0, qx=0, qy=0, qz=0, qw=1, timestamp_ns=0),
        ))
        active = False

        @contextmanager
        def subscribe():
            nonlocal active
            active = True
            try:
                yield
            finally:
                active = False

        def read_image(camera_id=None):
            assert active
            return image

        def publish_pose(delta_ns):
            p.streams.publish("localization.pose", source="geometry",
                payload=replace(measured_pose, timestamp_ns=stamp + delta_ns),
                metadata={"map_name": "house", "source_timestamp_ns": stamp + delta_ns})

        def publish_cloud(delta_ns, points):
            cloud = PointCloudFrame(frame_id="base", timestamp=image.timestamp, points=points, point_count=len(points),
                                    metadata={"source_timestamp_ns": stamp + delta_ns})
            p.streams.publish("pointcloud.frame", source="robot", payload=cloud, metadata={"map_name": "house"})

        monkeypatch.setattr(p.service, "point_cloud_consumer", subscribe)
        p.service._video_frame_source.get_latest_image = read_image
        p.service._video_frame_source.get_camera_info = lambda _: camera
        publish_pose(-100_000_000)
        publish_pose(0)
        first = [Vector3(x=-0.29 + step * 0.005, y=-0.05, z=1.0) for step in range(5)]
        second = [Vector3(x=-0.25 + step * 0.005, y=-0.05, z=1.0) for step in range(5)]
        publish_cloud(-50_000_000, first)
        detect = p.service._detector_pipeline.detect

        def advance_clouds(*args, **kwargs):
            assert active
            publish_pose(100_000_000)
            publish_cloud(80_000_000, second)
            for index in range(19):
                publish_cloud(400_000_000 + index * 100_000_000, [Vector3(x=5, y=5, z=5)])
            return detect(*args, **kwargs)

        monkeypatch.setattr(p.service._detector_pipeline, "detect", advance_clouds)
        if fast:
            result = p.service.detect_current_frame_fast()
            for index in range(21):
                publish_cloud(3_000_000_000 + index * 100_000_000, [Vector3(x=5, y=5, z=5)])
            result, _, _ = p.service.complete_observation_geometry(result, p.service.get_fast_observation_input(result))
        else:
            result = p.service.perceive_current_scene()
        assert not active
        detection = result.observation.detections_2d[0]
        assert detection.attributes["geometry_support_point_count"] == 10
        assert detection.attributes["support_cluster_summary"]["center"] == pytest.approx([-0.26, -0.05, 1.0])
