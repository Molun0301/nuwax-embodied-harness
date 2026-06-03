"""RosBridgeProvider — 通过 gRPC 调用 Rust nuwax-ros-bridge 子进程。

Rust 桥是主要的 ROS2 交互层，本模块封装其 gRPC 接口，
提供与原始 Python Provider 相同的调用接口。
"""
from __future__ import annotations

from datetime import datetime, timezone
import logging
import os
import threading
from typing import Any, Callable, Dict, List, Optional

import grpc

from contracts.geometry import FrameTree, Pose, Quaternion, Transform, Vector3
from contracts.maps import CostMap, OccupancyGrid
from services.geometry.proto_gen import geometry_runtime_pb2 as pb2
from services.geometry.proto_gen import geometry_runtime_pb2_grpc as pb2_grpc
from services.geometry.ros_bridge_client import RosBridgeSubprocess

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.remote_provider")
_RPC_TIMEOUT_SEC = 1.0


class RosBridgeProvider:
    """统一的 RosBridgeProvider，封装所有 gRPC 调用。"""

    def __init__(self, bridge: RosBridgeSubprocess) -> None:
        self._bridge = bridge
        self._channel: Optional[grpc.Channel] = None
        self._stub: Optional[pb2_grpc.GeometryRuntimeStub] = None
        self._connected = False
        self._map_frame = os.environ.get("NUWAX_GEOMETRY_MAP_FRAME", "map")
        self._odom_frame = os.environ.get("NUWAX_GEOMETRY_ODOM_FRAME", "odom")
        self._base_frame = os.environ.get("NUWAX_GEOMETRY_BASE_FRAME", "base_link")
        self._occupancy_topic = os.environ.get("NUWAX_MAP_TOPIC") or os.environ.get("NUWAX_GEOMETRY_OCCUPANCY_TOPIC", "/map")

    def connect(self) -> bool:
        """连接到 Rust 子进程的 gRPC server。"""
        if not self._bridge.is_running:
            LOGGER.error("子进程未启动，无法连接")
            return False
        try:
            self._channel = grpc.insecure_channel(self._bridge.grpc_target)
            self._stub = pb2_grpc.GeometryRuntimeStub(self._channel)
            resp = self._stub.GetStatus(pb2.Empty(), timeout=_RPC_TIMEOUT_SEC)
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
            resp = self._stub.GetFrameTree(pb2.Empty(), timeout=_RPC_TIMEOUT_SEC)
            return len(resp.transforms) > 0
        except grpc.RpcError:
            return False

    def is_localization_available(self) -> bool:
        return self.is_available()

    # === Localization ===
    def get_current_pose(self) -> Optional[Pose]:
        if not self._stub:
            return None
        try:
            frame_tree = self._stub.GetFrameTree(pb2.Empty(), timeout=_RPC_TIMEOUT_SEC)
            pose_from_tf = self._pose_from_frame_tree(frame_tree)
            if pose_from_tf is not None:
                return pose_from_tf
            resp = self._stub.GetCurrentPose(pb2.PoseRequest(), timeout=_RPC_TIMEOUT_SEC)
            pose = self._pose_from_pb(resp)
            if pose is None:
                return None
            return self._pose_in_map_frame(pose, frame_tree)
        except grpc.RpcError:
            return None

    def get_frame_tree(self) -> Optional[FrameTree]:
        if not self._stub:
            return None
        try:
            resp = self._stub.GetFrameTree(pb2.Empty(), timeout=_RPC_TIMEOUT_SEC)
            return self._frame_tree_from_pb(resp)
        except grpc.RpcError:
            return None

    # === Mapping ===
    def get_occupancy_grid_meta(self) -> Optional[Dict[str, Any]]:
        if not self._stub:
            return None
        try:
            resp = self._stub.GetOccupancyGrid(pb2.Empty(), timeout=_RPC_TIMEOUT_SEC)
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
            resp = self._stub.GetOccupancyGridData(pb2.Empty(), timeout=_RPC_TIMEOUT_SEC)
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

    def get_occupancy_grid(self) -> Optional[OccupancyGrid]:
        payload = self.get_occupancy_grid_data()
        if not payload:
            return None
        width = int(payload.get("width") or 0)
        height = int(payload.get("height") or 0)
        resolution = float(payload.get("resolution") or 0.0)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            return None
        data = self._decode_i8_bytes(payload.get("data") or b"")
        if len(data) != width * height:
            return None
        frame_id = str(payload.get("frame_id") or self._map_frame)
        revision = int(payload.get("revision") or 0)
        return OccupancyGrid(
            map_id="ros_bridge:%s:%d" % (self._occupancy_topic, revision),
            frame_id=frame_id,
            width=width,
            height=height,
            resolution_m=resolution,
            origin=Pose(
                frame_id=frame_id,
                position=Vector3(
                    x=float(payload.get("origin_x") or 0.0),
                    y=float(payload.get("origin_y") or 0.0),
                    z=0.0,
                ),
                orientation=Quaternion(),
            ),
            data=data,
        )

    def get_cost_map(self) -> Optional[CostMap]:
        occupancy_grid = self.get_occupancy_grid()
        if occupancy_grid is None:
            return None
        costs = [100.0 if int(value) < 0 else float(value) for value in occupancy_grid.data]
        return CostMap(
            map_id=occupancy_grid.map_id.replace("ros_bridge:", "ros_bridge_cost:"),
            frame_id=occupancy_grid.frame_id,
            width=occupancy_grid.width,
            height=occupancy_grid.height,
            resolution_m=occupancy_grid.resolution_m,
            origin=occupancy_grid.origin,
            data=costs,
        )

    def get_semantic_map(self):
        return None

    def is_map_available(self) -> bool:
        meta = self.get_occupancy_grid_meta()
        return bool(meta and int(meta.get("width") or 0) > 0 and int(meta.get("height") or 0) > 0 and int(meta.get("revision") or 0) > 0)

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
            resp = self._stub.GetNavigationReadiness(pb2.Empty(), timeout=_RPC_TIMEOUT_SEC)
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
            resp = self._stub.GetStatus(pb2.Empty(), timeout=_RPC_TIMEOUT_SEC)
            status = {"phase": resp.phase, "message": resp.message, "rss_kb": resp.rss_kb}
            status.update(self._build_map_status())
            status.update(self._build_tf_status())
            return status
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
                    current_pose = self._pose_from_pb(pose)
                    frame_tree = self.get_frame_tree()
                    if current_pose is not None and frame_tree is not None:
                        current_pose = self._pose_in_map_frame_pb(current_pose)
                    callback(current_pose, frame_tree, {"source_name": self.provider_name, "sequence": update.sequence})
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
                    occupancy_grid = self.get_occupancy_grid()
                    cost_map = self.get_cost_map()
                    callback({
                        "source_name": self.provider_name,
                        "occupancy_grid": occupancy_grid,
                        "cost_map": cost_map,
                        "semantic_map": None,
                        "width": meta.width, "height": meta.height,
                        "resolution": meta.resolution,
                        "origin_x": meta.origin_x, "origin_y": meta.origin_y,
                        "frame_id": meta.frame_id, "revision": meta.revision,
                        "metadata": {
                            "source_kind": "platform_live_map_provider",
                            "map_snapshot_kind": "live_streaming",
                            "live_streaming_ready": True,
                            "rtabmap_live_streaming_ready": True,
                            "occupancy_topic": self._occupancy_topic,
                            "occupancy_revision": int(meta.revision or 0),
                            "occupancy_sequence": int(meta.revision or 0),
                        },
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

    @property
    def provider_name(self) -> str:
        return "ros_bridge"

    @property
    def provider_version(self) -> str:
        return "0.1.0"

    @staticmethod
    def _decode_i8_bytes(data: bytes) -> List[int]:
        return [value - 256 if value > 127 else value for value in data]

    @staticmethod
    def _quat_mul(a: Quaternion, b: Quaternion) -> Quaternion:
        return Quaternion(
            x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        )

    @classmethod
    def _quat_rotate(cls, q: Quaternion, v: Vector3) -> Vector3:
        vector_quat = Quaternion(x=v.x, y=v.y, z=v.z, w=0.0)
        q_inv = Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)
        rotated = cls._quat_mul(cls._quat_mul(q, vector_quat), q_inv)
        return Vector3(x=rotated.x, y=rotated.y, z=rotated.z)

    @classmethod
    def _compose_transform(cls, first: Transform, second: Transform) -> Transform:
        rotated = cls._quat_rotate(first.rotation, second.translation)
        return Transform(
            parent_frame_id=first.parent_frame_id,
            child_frame_id=second.child_frame_id,
            translation=Vector3(
                x=first.translation.x + rotated.x,
                y=first.translation.y + rotated.y,
                z=first.translation.z + rotated.z,
            ),
            rotation=cls._quat_mul(first.rotation, second.rotation),
            authority="ros_bridge",
        )

    def _pose_from_pb(self, pose_pb) -> Optional[Pose]:
        frame_id = str(getattr(pose_pb, "frame_id", "") or "")
        if not frame_id:
            return None
        pos = pose_pb.position
        ori = pose_pb.orientation
        return Pose(
            frame_id=frame_id,
            position=Vector3(x=pos.x, y=pos.y, z=pos.z),
            orientation=Quaternion(x=ori.x, y=ori.y, z=ori.z, w=ori.w),
        )

    def _transform_from_pb(self, tf_pb) -> Transform:
        trans = tf_pb.translation
        rot = tf_pb.rotation
        return Transform(
            parent_frame_id=str(tf_pb.parent_frame),
            child_frame_id=str(tf_pb.child_frame),
            translation=Vector3(x=trans.x, y=trans.y, z=trans.z),
            rotation=Quaternion(x=rot.x, y=rot.y, z=rot.z, w=rot.w),
            authority="ros_bridge",
        )

    def _frame_tree_from_pb(self, frame_tree_pb) -> FrameTree:
        transforms = [self._transform_from_pb(item) for item in frame_tree_pb.transforms]
        return FrameTree(root_frame_id=self._map_frame, transforms=transforms)

    def _pose_from_frame_tree(self, frame_tree_pb) -> Optional[Pose]:
        transforms = [self._transform_from_pb(item) for item in frame_tree_pb.transforms]
        by_edge = {(item.parent_frame_id, item.child_frame_id): item for item in transforms}
        direct = by_edge.get((self._map_frame, self._base_frame)) or by_edge.get((self._map_frame, "base_footprint"))
        if direct is not None:
            return self._pose_from_transform(direct)
        map_to_odom = by_edge.get((self._map_frame, self._odom_frame))
        odom_to_base = by_edge.get((self._odom_frame, self._base_frame)) or by_edge.get((self._odom_frame, "base_footprint"))
        if map_to_odom is None or odom_to_base is None:
            return None
        return self._pose_from_transform(self._compose_transform(map_to_odom, odom_to_base))

    @staticmethod
    def _pose_from_transform(transform: Transform) -> Pose:
        return Pose(
            frame_id=transform.parent_frame_id,
            position=transform.translation,
            orientation=transform.rotation,
        )

    def _pose_in_map_frame_pb(self, pose: Pose) -> Pose:
        if not self._stub:
            return pose
        try:
            frame_tree = self._stub.GetFrameTree(pb2.Empty(), timeout=_RPC_TIMEOUT_SEC)
        except grpc.RpcError:
            return pose
        return self._pose_in_map_frame(pose, frame_tree)

    def _pose_in_map_frame(self, pose: Pose, frame_tree_pb) -> Pose:
        if pose.frame_id == self._map_frame:
            return pose
        transforms = [self._transform_from_pb(item) for item in frame_tree_pb.transforms]
        by_edge = {(item.parent_frame_id, item.child_frame_id): item for item in transforms}
        parent_to_pose = by_edge.get((self._map_frame, pose.frame_id))
        if parent_to_pose is None:
            return pose
        pose_as_transform = Transform(
            parent_frame_id=pose.frame_id,
            child_frame_id=self._base_frame,
            translation=pose.position,
            rotation=pose.orientation,
            authority="ros_bridge",
        )
        return self._pose_from_transform(self._compose_transform(parent_to_pose, pose_as_transform))

    def _build_map_status(self) -> Dict[str, Any]:
        grid = self.get_occupancy_grid()
        if grid is None:
            return {
                "occupancy_topic": self._occupancy_topic,
                "occupancy_grid_ready": False,
                "occupancy_has_publisher": None,
                "occupancy_message_count": 0,
            }
        known = sum(1 for value in grid.data if int(value) >= 0)
        free = sum(1 for value in grid.data if int(value) == 0)
        occupied = sum(1 for value in grid.data if int(value) >= 50)
        unknown = len(grid.data) - known
        now = datetime.now(timezone.utc).isoformat()
        try:
            revision = int(str(grid.map_id).rsplit(":", 1)[-1])
        except (TypeError, ValueError):
            revision = 1
        metrics = {
            "known_cell_count": known,
            "free_cell_count": free,
            "occupied_cell_count": occupied,
            "unknown_cell_count": unknown,
            "width": grid.width,
            "height": grid.height,
            "resolution_m": grid.resolution_m,
        }
        return {
            "provider_name": self.provider_name,
            "occupancy_topic": self._occupancy_topic,
            "occupancy_grid_ready": True,
            "occupancy_has_publisher": True,
            "occupancy_message_count": max(1, revision),
            "occupancy_sequence": max(1, revision),
            "occupancy_revision": max(1, revision),
            "latest_occupancy_timestamp": now,
            "latest_occupancy_received_at": now,
            "occupancy_metrics": metrics,
            "message_count": max(1, revision),
            "update_sequence": max(1, revision),
        }

    def _build_tf_status(self) -> Dict[str, Any]:
        if not self._stub:
            return {}
        try:
            frame_tree_pb = self._stub.GetFrameTree(pb2.Empty(), timeout=_RPC_TIMEOUT_SEC)
        except grpc.RpcError:
            return {"pose_ready": False, "pose_available": False}
        transforms = [self._transform_from_pb(item) for item in frame_tree_pb.transforms]
        edges = {(item.parent_frame_id, item.child_frame_id) for item in transforms}
        map_to_odom = (self._map_frame, self._odom_frame) in edges
        odom_to_base = (self._odom_frame, self._base_frame) in edges or (self._odom_frame, "base_footprint") in edges
        map_to_base = (self._map_frame, self._base_frame) in edges or (self._map_frame, "base_footprint") in edges or (map_to_odom and odom_to_base)
        pose_ready = self._pose_from_frame_tree(frame_tree_pb) is not None
        return {
            "pose_ready": pose_ready,
            "pose_available": pose_ready,
            "map_to_odom_ready": map_to_odom,
            "odom_to_base_ready": odom_to_base,
            "map_to_base_ready": map_to_base,
            "tf_message_count": len(transforms),
            "frame_count": len({frame for edge in edges for frame in edge}),
        }
