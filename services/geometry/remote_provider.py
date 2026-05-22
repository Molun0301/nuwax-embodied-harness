"""RosBridgeProvider — 通过 gRPC 调用 Rust nuwax-ros-bridge 子进程。

Rust 桥是主要的 ROS2 交互层，本模块封装其 gRPC 接口，
提供与原始 Python Provider 相同的调用接口。
"""
from __future__ import annotations

import logging
import threading
from typing import Any, Callable, Dict, List, Optional

import grpc

from services.geometry.proto_gen import geometry_runtime_pb2 as pb2
from services.geometry.proto_gen import geometry_runtime_pb2_grpc as pb2_grpc
from services.geometry.ros_bridge_client import RosBridgeSubprocess

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.remote_provider")


class RosBridgeProvider:
    """统一的 RosBridgeProvider，封装所有 gRPC 调用。"""

    def __init__(self, bridge: RosBridgeSubprocess) -> None:
        self._bridge = bridge
        self._channel: Optional[grpc.Channel] = None
        self._stub: Optional[pb2_grpc.GeometryRuntimeStub] = None
        self._connected = False

    def connect(self) -> bool:
        """连接到 Rust 子进程的 gRPC server。"""
        if not self._bridge.is_running:
            LOGGER.error("子进程未启动，无法连接")
            return False
        try:
            self._channel = grpc.insecure_channel(self._bridge.grpc_target)
            self._stub = pb2_grpc.GeometryRuntimeStub(self._channel)
            resp = self._stub.GetStatus(pb2.Empty())
            LOGGER.info("gRPC 连接成功: phase=%s, rss=%dKB", resp.phase, resp.rss_kb)
            self._connected = True
            return True
        except grpc.RpcError as e:
            LOGGER.error("gRPC 连接失败: %s", e)
            return False

    def disconnect(self) -> None:
        if self._channel:
            self._channel.close()
            self._channel = None
            self._stub = None
            self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def is_available(self) -> bool:
        if not self._connected or not self._stub:
            return False
        try:
            resp = self._stub.GetFrameTree(pb2.Empty())
            return len(resp.transforms) > 0
        except grpc.RpcError:
            return False

    def is_localization_available(self) -> bool:
        return self.is_available()

    # === Localization ===
    def get_current_pose(self) -> Optional[Dict[str, Any]]:
        if not self._stub:
            return None
        try:
            resp = self._stub.GetCurrentPose(pb2.PoseRequest())
            pos = resp.position
            ori = resp.orientation
            return {
                "x": pos.x, "y": pos.y, "z": pos.z,
                "qx": ori.x, "qy": ori.y, "qz": ori.z, "qw": ori.w,
                "frame_id": resp.frame_id,
            }
        except grpc.RpcError:
            return None

    def get_frame_tree(self) -> List[Dict[str, Any]]:
        if not self._stub:
            return []
        try:
            resp = self._stub.GetFrameTree(pb2.Empty())
            return [
                {"parent": t.parent_frame, "child": t.child_frame,
                 "tx": t.translation.x, "ty": t.translation.y, "tz": t.translation.z,
                 "rx": t.rotation.x, "ry": t.rotation.y, "rz": t.rotation.z, "rw": t.rotation.w}
                for t in resp.transforms
            ]
        except grpc.RpcError:
            return []

    # === Mapping ===
    def get_occupancy_grid_meta(self) -> Optional[Dict[str, Any]]:
        if not self._stub:
            return None
        try:
            resp = self._stub.GetOccupancyGrid(pb2.Empty())
            return {
                "width": resp.width, "height": resp.height,
                "resolution": resp.resolution,
                "origin_x": resp.origin_x, "origin_y": resp.origin_y,
                "frame_id": resp.frame_id, "revision": resp.revision,
            }
        except grpc.RpcError:
            return None

    def get_occupancy_grid_data(self) -> Optional[Dict[str, Any]]:
        """获取完整 OccupancyGrid 数据（元数据 + 栅格字节）。"""
        if not self._stub:
            return None
        try:
            resp = self._stub.GetOccupancyGridData(pb2.Empty())
            meta = resp.meta
            return {
                "width": meta.width, "height": meta.height,
                "resolution": meta.resolution,
                "origin_x": meta.origin_x, "origin_y": meta.origin_y,
                "frame_id": meta.frame_id, "revision": meta.revision,
                "data": bytes(resp.data),
            }
        except grpc.RpcError:
            return None

    # === Navigation ===
    def set_navigation_goal(self, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> Dict[str, Any]:
        if not self._stub:
            return {"accepted": False, "message": "未连接"}
        try:
            goal = pb2.NavigationGoal(
                target_pose=pb2.Pose(
                    position=pb2.Vec3(x=x, y=y, z=z),
                    orientation=pb2.Quat(x=qx, y=qy, z=qz, w=qw),
                )
            )
            resp = self._stub.SetNavigationGoal(goal)
            return {"accepted": resp.accepted, "goal_id": resp.goal_id, "message": resp.message}
        except grpc.RpcError as e:
            return {"accepted": False, "message": str(e)}

    def cancel_navigation(self) -> bool:
        if not self._stub:
            return False
        try:
            resp = self._stub.CancelNavigation(pb2.Empty())
            return resp.success
        except grpc.RpcError:
            return False

    def get_navigation_readiness(self) -> Dict[str, Any]:
        if not self._stub:
            return {"ready": False, "message": "未连接"}
        try:
            resp = self._stub.GetNavigationReadiness(pb2.Empty())
            return {
                "ready": resp.ready,
                "waiting_for": resp.waiting_for,
                "message": resp.message,
                "action_server_ready": resp.action_server_ready,
                "tf_ready": resp.tf_ready,
            }
        except grpc.RpcError:
            return {"ready": False, "message": "gRPC 错误"}

    # === 输入桥 ===
    def publish_odom(self, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float,
                     lvx: float = 0, lvy: float = 0, lvz: float = 0,
                     avx: float = 0, avy: float = 0, avz: float = 0,
                     frame_id: str = "odom") -> bool:
        if not self._stub:
            return False
        try:
            msg = pb2.OdomInput(
                pose=pb2.Pose(
                    position=pb2.Vec3(x=x, y=y, z=z),
                    orientation=pb2.Quat(x=qx, y=qy, z=qz, w=qw),
                    frame_id=frame_id,
                ),
                linear_velocity=pb2.Vec3(x=lvx, y=lvy, z=lvz),
                angular_velocity=pb2.Vec3(x=avx, y=avy, z=avz),
            )
            self._stub.PublishOdom(msg)
            return True
        except grpc.RpcError:
            return False

    def get_status(self) -> Dict[str, Any]:
        if not self._stub:
            return {"phase": "disconnected", "rss_kb": 0}
        try:
            resp = self._stub.GetStatus(pb2.Empty())
            return {"phase": resp.phase, "message": resp.message, "rss_kb": resp.rss_kb}
        except grpc.RpcError:
            return {"phase": "error", "rss_kb": 0}

    # === 流式订阅 ===
    def subscribe_pose(self, callback: Callable[[Dict[str, Any]], None]) -> Optional[threading.Thread]:
        """订阅位姿更新流，每次更新调用 callback(pose_dict)。返回后台线程。"""
        if not self._stub:
            return None
        def _stream_worker():
            try:
                for update in self._stub.SubscribePose(pb2.PoseRequest()):
                    pose = update.pose
                    pos = pose.position
                    ori = pose.orientation
                    callback({
                        "x": pos.x, "y": pos.y, "z": pos.z,
                        "qx": ori.x, "qy": ori.y, "qz": ori.z, "qw": ori.w,
                        "frame_id": pose.frame_id,
                        "sequence": update.sequence,
                    })
            except grpc.RpcError:
                pass
        t = threading.Thread(target=_stream_worker, name="ros_bridge_pose_stream", daemon=True)
        t.start()
        return t

    def subscribe_map_updates(self, callback: Callable[[Dict[str, Any]], None]) -> Optional[threading.Thread]:
        """订阅地图更新通知流，每次更新调用 callback(map_meta_dict)。"""
        if not self._stub:
            return None
        def _stream_worker():
            try:
                for update in self._stub.SubscribeMapUpdates(pb2.Empty()):
                    meta = update.meta
                    callback({
                        "width": meta.width, "height": meta.height,
                        "resolution": meta.resolution,
                        "origin_x": meta.origin_x, "origin_y": meta.origin_y,
                        "frame_id": meta.frame_id, "revision": meta.revision,
                    })
            except grpc.RpcError:
                pass
        t = threading.Thread(target=_stream_worker, name="ros_bridge_map_stream", daemon=True)
        t.start()
        return t

    def subscribe_navigation_state(self, callback: Callable[[Dict[str, Any]], None]) -> Optional[threading.Thread]:
        """订阅导航状态流，每次更新调用 callback(nav_state_dict)。"""
        if not self._stub:
            return None
        def _stream_worker():
            try:
                for update in self._stub.SubscribeNavigationState(pb2.Empty()):
                    callback({
                        "status": update.status,
                        "message": update.message,
                        "goal_id": update.goal_id,
                        "distance_remaining": update.distance_remaining,
                    })
            except grpc.RpcError:
                pass
        t = threading.Thread(target=_stream_worker, name="ros_bridge_nav_stream", daemon=True)
        t.start()
        return t
