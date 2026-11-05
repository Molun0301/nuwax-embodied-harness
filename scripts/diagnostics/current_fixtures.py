"""离线检查共用的当前接口夹具；只在临时目录内保存合成数据。"""
from contextlib import contextmanager
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace

from core.capability import CapabilityTable
from core.event_bus import EventBus
from core.state_store import StateStore
from core.tasks import TaskRunner
from services.memory.service import MemoryService


def finished(platform, task_id, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = platform.tasks.status(task_id)
        if value.finished_at is not None:
            return value
        time.sleep(.002)
    raise AssertionError("任务未完成收尾")


@contextmanager
def task_platform(*, history_limit=200):
    events = EventBus()
    platform = SimpleNamespace(events=events, capabilities=CapabilityTable(),
        ensure=lambda names, timeout: None, motion=SimpleNamespace(begin=lambda: 7),
        robot=SimpleNamespace(manifest=SimpleNamespace(robot_name="go2")))
    platform.tasks = TaskRunner(platform, history_limit=history_limit)
    try:
        yield platform
    finally:
        platform.tasks.shutdown()
        events.shutdown()


@contextmanager
def memory_service(**kwargs):
    with tempfile.TemporaryDirectory(prefix="a2d-memory-check-") as directory:
        root = Path(directory)
        service = MemoryService(geometry_state=None, maps_root=root / "maps", default_root=root / "default",
            observation_service=None, perception_service=None, state_store=StateStore(),
            embedding_model="lexical-only", image_embedding_model="disabled", **kwargs)
        try:
            yield service
        finally:
            service.close()

class StaticGeometryState:
    """显式离线地图输入；只用于诊断，不冒充原生进程或物理结果。"""
    def __init__(self, grid, pose=None):
        from services.geometry.client import _snapshot
        from services.geometry.proto_gen import geometry_runtime_pb2 as pb
        from services.geometry.state import MapUpdate
        value=pb.GeometryState(phase=pb.SESSION_PHASE_MAPPING,map_name=grid.map_name,
            info=pb.RuntimeInfo(pipeline="slam_2d",map_frame_id=grid.frame_id,odom_frame_id="odom",base_frame_id="base_link"))
        if pose is not None:
            from services.geometry.client import _pose_message
            value.pose.CopyFrom(_pose_message(pose))
        self.value=_snapshot(value);self.update=MapUpdate(grid,grid.data)
        self.connected=True;self._callbacks={"state":[],"pose":[],"map":[]}
    def snapshot(self):return self.value
    def latest_map(self):return self.update
    def pose(self):return self.value.pose
    def _subscribe(self,kind,callback):
        self._callbacks[kind].append(callback)
        return lambda:self._callbacks[kind].remove(callback)
    def subscribe(self,callback):return self._subscribe("state",callback)
    def subscribe_pose(self,callback):return self._subscribe("pose",callback)
    def subscribe_map(self,callback):return self._subscribe("map",callback)
    def start(self):
        for kind,value in (("state",self.value),("map",self.update),("pose",self.value.pose)):
            if value is not None:
                for callback in tuple(self._callbacks[kind]):callback(value)
    def stop(self):pass


@contextmanager
def offline_platform(config,grid,pose=None):
    """真实装配根加明确离线Geometry视图；检测与设备调用保持关闭。"""
    from unittest.mock import patch
    from gateways.platform import build_platform
    geometry=StaticGeometryState(grid,pose)
    with patch("gateways.platform.GeometryStateView",return_value=geometry),patch("gateways.platform.components.ensure",return_value=None):
        platform=build_platform(config)
        try:
            platform.start()
            yield platform
        finally:platform.stop()
