"""RosBridgeProvider — 通过 gRPC 调用 Rust nuwax-ros-bridge 子进程。

Rust 桥是主要的 ROS2 交互层，本模块封装其 gRPC 接口，
提供与原始 Python Provider 相同的调用接口。
"""
from __future__ import annotations

import logging
import os
import threading
from typing import Any, Callable, Dict, List, Optional

import grpc

from contracts.frame_semantics import infer_frame_role
from contracts.geometry import FrameTree, Pose, Quaternion, Transform, Vector3
from services.geometry.proto_gen import geometry_runtime_pb2 as pb2
from services.geometry.proto_gen import geometry_runtime_pb2_grpc as pb2_grpc
from services.geometry.ros_bridge_client import RosBridgeSubprocess

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.remote_provider")

_STATIC_EDGE_SKIP_LOG_INTERVAL_SEC = 30.0
_last_static_edge_skip_log_at = 0.0
_last_static_edge_skip_signature: Optional[str] = None


def _warn_static_edge_skip(message: str, *, signature: str) -> None:
    """对重复的静态外参跳过告警做节流，避免 get_frame_tree 高频刷屏。"""
    global _last_static_edge_skip_log_at, _last_static_edge_skip_signature

    import time

    now = time.monotonic()
    if signature == _last_static_edge_skip_signature and now - _last_static_edge_skip_log_at < _STATIC_EDGE_SKIP_LOG_INTERVAL_SEC:
        return
    _last_static_edge_skip_log_at = now
    _last_static_edge_skip_signature = signature
    LOGGER.warning(message)


def _read_deadline_sec() -> float:
    """unary 轻量读类 gRPC 调用 deadline（秒）。

    下游 Rust 桥/RTAB-Map 慢或未就绪时，无 deadline 的 unary 调用会无界阻塞，
    单点挂死整条 MCP 请求（参见超时根因 D1）。轻量读类（GetStatus / GetOccupancyGrid 元数据 /
    GetNavigationReadiness / CancelNavigation / SetNavigationGoal / PublishOdom）统一施加该 deadline。

    经 NUWAX_GEOMETRY_GRPC_READ_DEADLINE_SEC 可配；默认 2.0s，下限 0.1s 避免误配为 0。
    """
    try:
        return max(0.1, float(os.environ.get("NUWAX_GEOMETRY_GRPC_READ_DEADLINE_SEC", "2.0")))
    except (TypeError, ValueError):
        return 2.0


def _pose_deadline_sec() -> float:
    """unary 位姿/坐标树读类 gRPC 调用 deadline（秒）。

    GetCurrentPose / GetFrameTree 在首帧 map→odom→base_link tf2 链合成偏慢时可能 >2s
    （见 rust-bridge-pose-localizing-bug），用轻量读 deadline 会把"合法但偏慢"的首帧误降级成
    None（伪"无定位"）。故单独给位姿类一档更宽的 deadline，仍有界（不退回无界阻塞）。

    经 NUWAX_GEOMETRY_GRPC_POSE_DEADLINE_SEC 可配；默认 5.0s，下限 0.1s。
    默认值需按实机 p95 复核。
    """
    try:
        return max(0.1, float(os.environ.get("NUWAX_GEOMETRY_GRPC_POSE_DEADLINE_SEC", "5.0")))
    except (TypeError, ValueError):
        return 5.0


def _heavy_deadline_sec() -> float:
    """unary 重操作 gRPC 调用 deadline（秒）。

    用于可能传输较大载荷的 unary 调用（如 GetOccupancyGridData 全量栅格字节）。
    经 NUWAX_GEOMETRY_GRPC_HEAVY_DEADLINE_SEC 可配；默认 10.0s，下限 0.5s。
    """
    try:
        return max(0.5, float(os.environ.get("NUWAX_GEOMETRY_GRPC_HEAVY_DEADLINE_SEC", "10.0")))
    except (TypeError, ValueError):
        return 10.0


def _log_rpc_error(rpc_name: str, exc: grpc.RpcError) -> None:
    """统一记录 unary gRPC 失败；区分 DEADLINE_EXCEEDED 以便排障检索。

    DEADLINE_EXCEEDED 表示下游未在 deadline 内回包（本应触发降级而非挂死），
    单列 warning 便于在运行时日志中检索"被 deadline 截断"的调用。
    """
    code = exc.code() if callable(getattr(exc, "code", None)) else None
    if code == grpc.StatusCode.DEADLINE_EXCEEDED:
        LOGGER.warning("gRPC %s 超过 deadline 被截断（DEADLINE_EXCEEDED），按降级返回", rpc_name)
    else:
        LOGGER.debug("gRPC %s 调用失败: %s", rpc_name, exc)


def _normalize_frame(frame_id: str) -> str:
    """去掉 ROS 风格前导斜杠并去空白，便于契约校验。"""
    return str(frame_id or "").strip().lstrip("/")


def _build_pose(proto_pose: Any) -> Optional[Pose]:
    """把 gRPC Pose 转成契约 Pose；frame_id 为空（无定位）时返回 None。"""
    frame_id = _normalize_frame(getattr(proto_pose, "frame_id", ""))
    if not frame_id:
        return None
    pos = proto_pose.position
    ori = proto_pose.orientation
    try:
        return Pose(
            frame_id=frame_id,
            position=Vector3(x=pos.x, y=pos.y, z=pos.z),
            orientation=Quaternion(x=ori.x, y=ori.y, z=ori.z, w=ori.w),
        )
    except Exception:  # pragma: no cover - 非法 frame_id 等
        return None


def _build_frame_tree(transforms: Any) -> Optional[FrameTree]:
    """把 gRPC Transform 列表转成契约 FrameTree；空树返回 None。"""
    contract_transforms: List[Transform] = []
    children: set = set()
    parents: List[str] = []
    for t in transforms:
        parent = _normalize_frame(t.parent_frame)
        child = _normalize_frame(t.child_frame)
        if not parent or not child:
            continue
        try:
            contract_transforms.append(
                Transform(
                    parent_frame_id=parent,
                    child_frame_id=child,
                    translation=Vector3(x=t.translation.x, y=t.translation.y, z=t.translation.z),
                    rotation=Quaternion(x=t.rotation.x, y=t.rotation.y, z=t.rotation.z, w=t.rotation.w),
                )
            )
        except Exception:  # pragma: no cover - 非法 frame_id
            continue
        children.add(child)
        if parent not in parents:
            parents.append(parent)
    if not contract_transforms:
        return None
    # 根坐标系：从不作为子坐标系出现的父坐标系；缺省回退 map / 第一个父系。
    root = next((p for p in parents if p not in children), None) or (
        "map" if any(p == "map" for p in parents) else parents[0]
    )
    try:
        return FrameTree(root_frame_id=root, transforms=contract_transforms)
    except Exception:  # pragma: no cover
        return None


def _merge_static_edges(frame_tree: Optional[FrameTree], static_edges: List[Transform]) -> Optional[FrameTree]:
    """把装配期缓存的静态外参边合并进每帧重建的 frame_tree。

    设计约束（见评测报告 §4.5 不变量）：
    1. frame_tree 每帧由 gRPC 全量重建，本函数在重建后合并，不在 _build_frame_tree
       纯函数里做一次性注入（那样会被下一帧覆盖）。
    2. 静态边只新增标定自有的 child（如相机光学系），**禁止覆盖** map/odom/base
       等动态定位链；child 已存在或角色属动态链时跳过。
    3. 静态边声明的 parent 是端侧 driver 的 base 命名（如 world/go2/base），可能与
       实机 tf 实际发布的 base 串（base_footprint/base_link）不同。这里按语义角色把
       parent 重锚定到“当前帧真实存在的 base 节点串”，否则静态边会成为孤岛、
       camera→map 路径不可达。找不到 base 节点则跳过该边（不兜底、不投错链）。
    """
    if not static_edges or frame_tree is None:
        return frame_tree
    existing = list(frame_tree.transforms)
    present_children = {t.child_frame_id for t in existing}
    present_frames = set()
    for t in existing:
        present_frames.add(t.parent_frame_id)
        present_frames.add(t.child_frame_id)
    # 当前帧真实存在的 base 节点串（按语义角色匹配，取第一个角色为 base 的 frame）。
    live_base_frame = next((f for f in present_frames if infer_frame_role(f) == "base"), None)
    if live_base_frame is None:
        configured_base = _normalize_frame(os.environ.get("NUWAX_GEOMETRY_BASE_FRAME", ""))
        if configured_base and configured_base in present_frames:
            live_base_frame = configured_base

    merged = list(existing)
    appended = 0
    for edge in static_edges:
        # 守护：child 已存在 → 跳过，绝不覆盖既有边（含动态链）。
        if edge.child_frame_id in present_children:
            continue
        # 守护：静态边的 child 不得是 map/odom/base 等动态语义角色。
        if infer_frame_role(edge.child_frame_id) in {"map", "odom", "world", "base"}:
            LOGGER.warning("拒绝合并静态外参边：child=%s 属动态定位链角色", edge.child_frame_id)
            continue
        # 重锚定 parent 到当前帧真实 base 节点串。
        parent_frame = edge.parent_frame_id
        if parent_frame not in present_frames:
            if live_base_frame is None:
                _warn_static_edge_skip(
                    "静态外参边 parent=%s 在当前 frame_tree 不存在且无 base 节点可锚定，跳过 child=%s"
                    % (parent_frame, edge.child_frame_id),
                    signature="missing_base:%s:%s" % (parent_frame, edge.child_frame_id),
                )
                continue
            if infer_frame_role(parent_frame) == "base":
                parent_frame = live_base_frame
            else:
                LOGGER.warning(
                    "静态外参边 parent=%s 既不在当前 frame_tree 也非 base 角色，跳过 child=%s",
                    parent_frame, edge.child_frame_id,
                )
                continue
        try:
            merged.append(
                Transform(
                    parent_frame_id=parent_frame,
                    child_frame_id=edge.child_frame_id,
                    translation=edge.translation,
                    rotation=edge.rotation,
                )
            )
            appended += 1
        except Exception:  # pragma: no cover - 非法 frame_id
            continue
    if appended == 0:
        return frame_tree
    try:
        return FrameTree(root_frame_id=frame_tree.root_frame_id, transforms=merged)
    except Exception:  # pragma: no cover
        return frame_tree



class RosBridgeProvider:
    """统一的 RosBridgeProvider，封装所有 gRPC 调用。"""

    def __init__(self, bridge: RosBridgeSubprocess) -> None:
        self._bridge = bridge
        self._channel: Optional[grpc.Channel] = None
        self._stub: Optional[pb2_grpc.GeometryRuntimeStub] = None
        self._connected = False
        # 装配期一次性缓存的静态外参边（相机光学系等），每帧 get_frame_tree 合并。
        self._static_extrinsic_edges: List[Transform] = []

    def set_static_extrinsics(self, edges: List[Transform]) -> None:
        """装配期由平台几何运行时注入静态外参边（来自 CalibrationProvider）。

        外参为静态值，此处缓存一次；get_frame_tree 每帧重建后零 I/O 合并这些边，
        既不被全量重建覆盖，又不引入运行时读盘/高频写。重复调用以最新一次为准。
        """
        self._static_extrinsic_edges = list(edges or [])
        LOGGER.info("已缓存 %d 条静态外参边用于 frame_tree 合并", len(self._static_extrinsic_edges))

    def connect(self) -> bool:
        """连接到 Rust 子进程的 gRPC server。"""
        if not self._bridge.is_running:
            LOGGER.error("子进程未启动，无法连接")
            return False
        try:
            self._channel = grpc.insecure_channel(self._bridge.grpc_target)
            self._stub = pb2_grpc.GeometryRuntimeStub(self._channel)
            resp = self._stub.GetStatus(pb2.Empty(), timeout=_read_deadline_sec())
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
            resp = self._stub.GetFrameTree(pb2.Empty(), timeout=_pose_deadline_sec())
            return len(resp.transforms) > 0
        except grpc.RpcError as e:
            _log_rpc_error("GetFrameTree(is_available)", e)
            return False

    def is_localization_available(self) -> bool:
        return self.is_available()

    # === Localization ===
    def get_current_pose(self) -> Optional[Pose]:
        if not self._stub:
            return None
        try:
            resp = self._stub.GetCurrentPose(pb2.PoseRequest(), timeout=_pose_deadline_sec())
            return _build_pose(resp)
        except grpc.RpcError as e:
            _log_rpc_error("GetCurrentPose", e)
            return None

    def get_frame_tree(self) -> Optional[FrameTree]:
        if not self._stub:
            return None
        try:
            resp = self._stub.GetFrameTree(pb2.Empty(), timeout=_pose_deadline_sec())
            frame_tree = _build_frame_tree(resp.transforms)
            return _merge_static_edges(frame_tree, self._static_extrinsic_edges)
        except grpc.RpcError as e:
            _log_rpc_error("GetFrameTree", e)
            return None

    # === Mapping ===
    def get_occupancy_grid_meta(self) -> Optional[Dict[str, Any]]:
        if not self._stub:
            return None
        try:
            resp = self._stub.GetOccupancyGrid(pb2.Empty(), timeout=_read_deadline_sec())
            return {
                "width": resp.width, "height": resp.height,
                "resolution": resp.resolution,
                "origin_x": resp.origin_x, "origin_y": resp.origin_y,
                "frame_id": resp.frame_id, "revision": resp.revision,
            }
        except grpc.RpcError as e:
            _log_rpc_error("GetOccupancyGrid", e)
            return None

    def get_occupancy_grid_data(self) -> Optional[Dict[str, Any]]:
        """获取完整 OccupancyGrid 数据（元数据 + 栅格字节）。"""
        if not self._stub:
            return None
        try:
            # 全量栅格字节可能较大，用重操作 deadline 而非读类 deadline。
            resp = self._stub.GetOccupancyGridData(pb2.Empty(), timeout=_heavy_deadline_sec())
            meta = resp.meta
            return {
                "width": meta.width, "height": meta.height,
                "resolution": meta.resolution,
                "origin_x": meta.origin_x, "origin_y": meta.origin_y,
                "frame_id": meta.frame_id, "revision": meta.revision,
                "data": bytes(resp.data),
            }
        except grpc.RpcError as e:
            _log_rpc_error("GetOccupancyGridData", e)
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
            resp = self._stub.SetNavigationGoal(goal, timeout=_read_deadline_sec())
            return {"accepted": resp.accepted, "goal_id": resp.goal_id, "message": resp.message}
        except grpc.RpcError as e:
            _log_rpc_error("SetNavigationGoal", e)
            return {"accepted": False, "message": str(e)}

    def cancel_navigation(self) -> bool:
        if not self._stub:
            return False
        try:
            resp = self._stub.CancelNavigation(pb2.Empty(), timeout=_read_deadline_sec())
            return resp.success
        except grpc.RpcError as e:
            _log_rpc_error("CancelNavigation", e)
            return False

    def get_navigation_readiness(self) -> Dict[str, Any]:
        if not self._stub:
            return {"ready": False, "message": "未连接"}
        try:
            resp = self._stub.GetNavigationReadiness(pb2.Empty(), timeout=_read_deadline_sec())
            return {
                "ready": resp.ready,
                "waiting_for": resp.waiting_for,
                "message": resp.message,
                "action_server_ready": resp.action_server_ready,
                "tf_ready": resp.tf_ready,
            }
        except grpc.RpcError as e:
            _log_rpc_error("GetNavigationReadiness", e)
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
            self._stub.PublishOdom(msg, timeout=_read_deadline_sec())
            return True
        except grpc.RpcError as e:
            _log_rpc_error("PublishOdom", e)
            return False

    def get_status(self) -> Dict[str, Any]:
        if not self._stub:
            return {"phase": "disconnected", "rss_kb": 0}
        try:
            resp = self._stub.GetStatus(pb2.Empty(), timeout=_read_deadline_sec())
            return {"phase": resp.phase, "message": resp.message, "rss_kb": resp.rss_kb}
        except grpc.RpcError as e:
            _log_rpc_error("GetStatus", e)
            return {"phase": "error", "rss_kb": 0}

    # === 流式订阅 ===
    def subscribe_pose(self, callback: Callable[[Pose, Optional[FrameTree], Dict[str, object]], None]) -> Optional[threading.Thread]:
        """订阅位姿更新流。按 LocalizationProvider 协议回调 (Pose, Optional[FrameTree], metadata)。

        frame_id 为空（尚未定位）的更新会被跳过，避免下游收到非法 Pose。
        """
        if not self._stub:
            return None
        import time
        # frame_tree 由 GetFrameTree 全量重建（_build_frame_tree + 静态边合并），代价较高。
        # pose 流频率可能远高于 tf 实际变化频率，逐帧重建是冗余。此处按时间节流缓存：
        # 同一帧位姿到来时复用最近 _FRAME_TREE_TTL_S 内的 frame_tree，超时才重新拉取。
        # tf 边自带 stamp/TTL，下游容忍亚秒级陈旧，正确性不受影响。
        _FRAME_TREE_TTL_S = 0.2
        def _stream_worker():
            cached_tree: Optional[FrameTree] = None
            cached_at = 0.0
            while self._stub:
                try:
                    for update in self._stub.SubscribePose(pb2.PoseRequest()):
                        pose = _build_pose(update.pose)
                        if pose is None:
                            continue
                        now = time.monotonic()
                        if cached_tree is None or (now - cached_at) >= _FRAME_TREE_TTL_S:
                            cached_tree = self.get_frame_tree()
                            cached_at = now
                        callback(pose, cached_tree, {"sequence": update.sequence})
                except grpc.RpcError:
                    pass
                if not self._stub:
                    break
                time.sleep(1.0)
        t = threading.Thread(target=_stream_worker, name="ros_bridge_pose_stream", daemon=True)
        t.start()
        return t

    def subscribe_map_updates(self, callback: Callable[[Dict[str, Any]], None]) -> Optional[threading.Thread]:
        """订阅地图更新通知流，每次更新调用 callback(map_meta_dict)。"""
        if not self._stub:
            return None
        import time
        def _stream_worker():
            while self._stub:
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
                if not self._stub:
                    break
                time.sleep(1.0)
        t = threading.Thread(target=_stream_worker, name="ros_bridge_map_stream", daemon=True)
        t.start()
        return t

    def subscribe_navigation_state(self, callback: Callable[[Dict[str, Any]], None]) -> Optional[threading.Thread]:
        """订阅导航状态流，每次更新调用 callback(nav_state_dict)。"""
        if not self._stub:
            return None
        import time
        def _stream_worker():
            while self._stub:
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
                if not self._stub:
                    break
                time.sleep(1.0)
        t = threading.Thread(target=_stream_worker, name="ros_bridge_nav_stream", daemon=True)
        t.start()
        return t
