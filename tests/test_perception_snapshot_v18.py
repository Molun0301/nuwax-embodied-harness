"""完整观察保留原图与模型等待前的几何，通知后可直接消费。"""
from dataclasses import replace
from types import SimpleNamespace
import threading

import pytest

from contracts.geometry import Pose, Vector3
from contracts.image import CameraInfo
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
