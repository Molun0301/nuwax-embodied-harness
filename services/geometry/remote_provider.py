"""Remote Provider — 通过 gRPC 调用 Rust nuwax-ros-bridge 子进程。

替代 in-process 的 Ros2*Provider，实现相同接口但通过 IPC 通信。
"""
from __future__ import annotations

import logging
import os
import threading
from typing import Any, Callable, Dict, List, Optional

import grpc

from services.geometry.proto_gen import geometry_runtime_pb2 as pb2
from services.geometry.proto_gen import geometry_runtime_pb2_grpc as pb2_grpc
from services.geometry.ros_bridge_client import RosBridgeSubprocess

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.remote_provider")


class RemoteGeometryProvider:
    """统一的 Remote Provider，封装所有 gRPC 调用。"""

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
