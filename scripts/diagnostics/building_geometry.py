"""楼层诊断的同协议假Geometry；目标完成由定时状态变化模拟，不含物理求解。"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import queue
import threading
import time

import grpc

from services.geometry.proto_gen import geometry_runtime_pb2 as pb
from services.geometry.proto_gen import geometry_runtime_pb2_grpc as rpc


def clone(message):
    result = type(message)()
    result.CopyFrom(message)
    return result


class GeometrySimulator(rpc.GeometryRuntimeServicer):
    """持有假地图与状态，通过正式gRPC（远程过程调用）接口提供给真实客户端。"""

    def __init__(self, maps, *, hold_navigation=False, hold_exploration=False):
        self.maps = maps
        self.hold_navigation = hold_navigation
        self.hold_exploration = hold_exploration
        self.state = pb.GeometryState(phase=pb.SESSION_PHASE_IDLE, info=pb.RuntimeInfo(
            pipeline="slam_2d", map_frame_id="map", odom_frame_id="odom", base_frame_id="base_link"))
        self.saved_maps = set()
        self.loaded_maps = []
        self.created_maps = []
        self.map_save_results = []
        self.goals = []
        self.explorations = []
        self.cancelled_goals = []
        self.stopped_explorations = []
        self.lock = threading.RLock()
        self.subscribers = {kind: [] for kind in ("state", "pose", "map")}
        self.timers = []
        self.executor = ThreadPoolExecutor(max_workers=8, thread_name_prefix="楼层假Geometry")
        self.server = grpc.server(self.executor)
        rpc.add_GeometryRuntimeServicer_to_server(self, self.server)
        port = self.server.add_insecure_port("127.0.0.1:0")
        self.target = f"127.0.0.1:{port}"
        self.closed = False

    def __enter__(self):
        self.server.start()
        return self

    def close(self):
        for timer in self.timers:
            timer.cancel()
        for timer in self.timers:
            timer.join()
        self.server.stop(0).wait()
        self.executor.shutdown(wait=True)
        self.closed = True

    def __exit__(self, *_args):
        self.close()

    def prepare_map(self, name):
        """只建假Geometry拥有的目录壳，不能作为真实地图或扫描证据。"""
        self.saved_maps.add(name)
        (self.maps.root / name / "geometry").mkdir(parents=True, exist_ok=True)
        self.maps.write_meta(name, {"display_name": name, "synthetic": True})

    def snapshot(self):
        with self.lock:
            return clone(self.state)

    def _publish(self):
        state = clone(self.state)
        for pending in self.subscribers["state"]:
            self._offer(pending, state)
        if self.state.HasField("pose"):
            pose = clone(self.state.pose)
            for pending in self.subscribers["pose"]:
                self._offer(pending, pose)

    @staticmethod
    def _offer(pending, value):
        try:
            pending.put_nowait(value)
        except queue.Full:
            try:
                pending.get_nowait()
            except queue.Empty:
                pass
            pending.put_nowait(value)

    def _stream(self, kind, context):
        pending = queue.Queue(maxsize=1)
        with self.lock:
            self.subscribers[kind].append(pending)
            if kind == "state":
                pending.put_nowait(clone(self.state))
            elif kind == "pose" and self.state.HasField("pose"):
                pending.put_nowait(clone(self.state.pose))
        try:
            while context.is_active():
                try:
                    yield pending.get(timeout=.05)
                except queue.Empty:
                    pass
        finally:
            with self.lock:
                self.subscribers[kind].remove(pending)

    def _later(self, callback):
        timer = threading.Timer(.05, callback)
        timer.name = "楼层假Geometry状态更新"
        self.timers.append(timer)
        timer.start()

    def SubscribeState(self, request, context):
        yield from self._stream("state", context)

    def SubscribePose(self, request, context):
        yield from self._stream("pose", context)

    def SubscribeMap(self, request, context):
        yield from self._stream("map", context)

    def GetState(self, request, context):
        return self.snapshot()

    def LoadMap(self, request, context):
        with self.lock:
            if request.map_name not in self.saved_maps:
                context.abort(grpc.StatusCode.NOT_FOUND, "假Geometry没有这张已保存地图")
            self.loaded_maps.append(request.map_name)
            self.state.map_name = request.map_name
            self.state.phase = pb.SESSION_PHASE_LOADED
            self.state.ClearField("pose")
            self.state.ClearField("navigation")
            self.state.ClearField("exploration")
            self._publish()
            return clone(self.state)

    def Relocalize(self, request, context):
        with self.lock:
            if self.state.phase != pb.SESSION_PHASE_LOADED:
                context.abort(grpc.StatusCode.FAILED_PRECONDITION, "假Geometry需要先加载地图")
            seed = request.seed
            self.state.pose.CopyFrom(pb.Pose(frame_id="map", timestamp_ns=time.time_ns(),
                position=pb.Vec3(x=seed.x, y=seed.y), orientation=pb.Quat(w=1)))
            self.state.phase = pb.SESSION_PHASE_LOCALIZED
            self._publish()
            return pb.RelocalizationResult(succeeded=True, message="假Geometry定位结果", pose=self.state.pose,
                                           fitness=1.0, frames_used=0, finished_at_ns=time.time_ns())

    def StartMapping(self, request, context):
        with self.lock:
            if (self.state.map_name == request.map_name
                    and self.state.phase in {pb.SESSION_PHASE_MAPPING, pb.SESSION_PHASE_EXPLORING}):
                return clone(self.state)
            if request.continue_saved:
                if self.state.map_name != request.map_name or self.state.phase != pb.SESSION_PHASE_LOCALIZED:
                    context.abort(grpc.StatusCode.FAILED_PRECONDITION, "续建前须加载并定位")
            else:
                if request.map_name in self.saved_maps:
                    context.abort(grpc.StatusCode.ALREADY_EXISTS, "地图名已经存在")
                self.created_maps.append(request.map_name)
                self.state.pose.CopyFrom(pb.Pose(frame_id="map", orientation=pb.Quat(w=1), timestamp_ns=time.time_ns()))
            self.state.phase = pb.SESSION_PHASE_MAPPING
            self.state.map_name = request.map_name
            self.state.ClearField("navigation")
            self.state.ClearField("exploration")
            self._publish()
            return clone(self.state)

    def StopSession(self, request, context):
        with self.lock:
            self.state.phase = pb.SESSION_PHASE_IDLE
            self.state.map_name = ""
            self.state.ClearField("navigation")
            self.state.ClearField("exploration")
            self.state.ClearField("pose")
            self._publish()
            return clone(self.state)

    def SaveMap(self, request, context):
        with self.lock:
            if self.state.phase not in {pb.SESSION_PHASE_MAPPING, pb.SESSION_PHASE_EXPLORING}:
                context.abort(grpc.StatusCode.FAILED_PRECONDITION, "保存前须开始建图")
            name = request.map_name or self.state.map_name
            replaced = name in self.saved_maps
            self.saved_maps.add(name)
            (self.maps.root / name / "geometry").mkdir(parents=True, exist_ok=True)
            self.map_save_results.append({"map_name": name, "replaced": replaced})
            return pb.SaveMapResponse(map_name=name, replaced=replaced)

    def SetGoal(self, request, context):
        with self.lock:
            if self.state.phase not in {pb.SESSION_PHASE_LOCALIZED, pb.SESSION_PHASE_MAPPING, pb.SESSION_PHASE_EXPLORING}:
                context.abort(grpc.StatusCode.FAILED_PRECONDITION, "导航需要定位或建图状态")
            goal_id = f"goal_{len(self.goals) + 1}"
            self.goals.append({"goal_id": goal_id, "map_name": self.state.map_name,
                               "motion_generation": request.motion_generation})
            if self.state.HasField("navigation"):
                self.state.previous_navigation.CopyFrom(self.state.navigation)
            self.state.navigation.CopyFrom(pb.NavigationStatus(goal_id=goal_id, status=pb.GOAL_STATUS_EXECUTING,
                                                                target=request.pose, message="假Geometry正在执行目标"))
            self._publish()
            if not self.hold_navigation:
                def complete():
                    with self.lock:
                        if self.state.navigation.goal_id == goal_id and self.state.navigation.status == pb.GOAL_STATUS_EXECUTING:
                            self.state.navigation.status = pb.GOAL_STATUS_SUCCEEDED
                            self.state.navigation.message = "假Geometry目标已到达"
                            self.state.pose.CopyFrom(request.pose)
                            self._publish()
                self._later(complete)
            return pb.SetGoalResponse(goal_id=goal_id)

    def CancelGoal(self, request, context):
        with self.lock:
            current = self.state.navigation
            if current.goal_id and (not request.goal_id or request.goal_id == current.goal_id):
                current.status = pb.GOAL_STATUS_CANCELLED
                current.message = "假Geometry目标已取消"
                self.cancelled_goals.append(current.goal_id)
                self._publish()
            return pb.Empty()

    def StartExploration(self, request, context):
        with self.lock:
            if self.state.phase != pb.SESSION_PHASE_MAPPING:
                context.abort(grpc.StatusCode.FAILED_PRECONDITION, "探索需要先建图")
            exploration_id = f"exploration_{len(self.explorations) + 1}"
            self.explorations.append({"exploration_id": exploration_id, "map_name": self.state.map_name,
                                      "motion_generation": request.motion_generation})
            self.state.phase = pb.SESSION_PHASE_EXPLORING
            self.state.exploration.CopyFrom(pb.ExplorationStatus(exploration_id=exploration_id,
                phase=pb.EXPLORATION_PHASE_RUNNING, message="假Geometry正在探索"))
            self._publish()
            if not self.hold_exploration:
                def complete():
                    with self.lock:
                        if (self.state.exploration.exploration_id == exploration_id
                                and self.state.exploration.phase == pb.EXPLORATION_PHASE_RUNNING):
                            self.state.exploration.phase = pb.EXPLORATION_PHASE_SUCCEEDED
                            self.state.phase = pb.SESSION_PHASE_MAPPING
                            self.state.exploration.message = "假Geometry探索已完成"
                            self._publish()
                self._later(complete)
            return pb.StartExplorationResponse(exploration_id=exploration_id)

    def StopExploration(self, request, context):
        with self.lock:
            current = self.state.exploration
            if current.exploration_id and (not request.exploration_id or current.exploration_id == request.exploration_id):
                current.phase = pb.EXPLORATION_PHASE_STOPPED
                self.state.phase = pb.SESSION_PHASE_MAPPING
                current.message = "假Geometry探索已停止"
                self.stopped_explorations.append(current.exploration_id)
                self._publish()
            return pb.Empty()
