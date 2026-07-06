"""GeometryRuntimeProvider — 通过 gRPC 调用 Rust nuwax-geometry-runtime 子进程。

Rust 运行时是主要的平台几何交互层（替代旧 nuwax-ros-bridge），本模块封装其 gRPC 接口，
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
from contracts.navigation import NavigationState, NavigationStatus
from services.geometry.proto_gen import geometry_runtime_pb2 as pb2
from services.geometry.proto_gen import geometry_runtime_pb2_grpc as pb2_grpc
from services.geometry.geometry_runtime_client import GeometryRuntimeSubprocess

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.geometry_runtime_provider")

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

    下游 Rust runtime / RTAB-Map 慢或未就绪时，无 deadline 的 unary 调用会无界阻塞，
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


def _is_shutdown_guard_enabled() -> bool:
    """NUWAX_GEOMETRY_SHUTDOWN_GUARD=0 时关闭（紧急回滚）。"""
    value = os.environ.get("NUWAX_GEOMETRY_SHUTDOWN_GUARD", "1").strip().lower()
    return value not in {"0", "false", "no", "off"}


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
       parent 重锚定到"当前帧真实存在的 base 节点串"，否则静态边会成为孤岛、
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



class GeometryRuntimeProvider:
    """统一的 GeometryRuntimeProvider，封装所有 gRPC 调用。"""

    def __init__(self, bridge: GeometryRuntimeSubprocess) -> None:
        self._bridge = bridge
        self._channel: Optional[grpc.Channel] = None
        self._stub: Optional[pb2_grpc.GeometryRuntimeStub] = None
        self._connected = False
        # shutdown 期守卫标志：disconnect()/清状态时置 True，所有 RPC 入口 guard。
        # 防止 in-flight task 持陈旧 _stub 引用继续打 RPC 全 UNAVAILABLE。
        self._stopping = False
        # 装配期一次性缓存的静态外参边（相机光学系等），每帧 get_frame_tree 合并。
        self._static_extrinsic_edges: List[Transform] = []
        # NavigationProvider 协议 shim 的本地状态缓存：SubscribeNavigationState 流最近一帧
        # + 上一次 set_goal/cancel_goal 触发的目标 id；get_navigation_state() 据此合成
        # contracts.NavigationState。lock 防后台订阅线程与 caller 并发改写。
        self._nav_state_lock = threading.Lock()
        self._last_nav_goal_id: Optional[str] = None
        self._last_nav_status: str = ""  # 空串表示尚未收到流更新；视为 IDLE
        self._last_nav_message: Optional[str] = None
        self._last_nav_distance_remaining: Optional[float] = None
        self._last_nav_current_pose: Optional[Pose] = None
        self._last_nav_timestamp_ns: int = 0
        self._nav_stream_started = False

    def set_static_extrinsics(self, edges: List[Transform]) -> None:
        """装配期由平台几何运行时注入静态外参边（来自 CalibrationProvider）。

        外参为静态值，此处缓存一次；get_frame_tree 每帧重建后零 I/O 合并这些边，
        既不被全量重建覆盖，又不引入运行时读盘/高频写。重复调用以最新一次为准。
        """
        self._static_extrinsic_edges = list(edges or [])
        LOGGER.info("已缓存 %d 条静态外参边用于 frame_tree 合并", len(self._static_extrinsic_edges))

    def connect(self) -> bool:
        """连接到 Rust 子进程的 gRPC server。

        失败时必须清空 channel/stub 并调 `bridge.disconnect()`，让下一次
        GeometryRuntimeSubprocess.start() 不再 short-circuit。attach 到僵尸进程时
        `bridge.is_running` 在 attach 状态短暂为 True（端口刚关但状态未清），
        RPC 失败后必须 reset bridge 状态，否则会留下永久 `_started=True`
        假活锁（参见 grpc-attach-zombie-bridge-2026-06-27）。
        """
        # shutdown 后禁止重连——防止留下半连接 ghost channel。
        if self._stopping and _is_shutdown_guard_enabled():
            LOGGER.debug("shutdown 中，拒绝重新连接 gRPC")
            return False
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
            self._cleanup_after_failed_connect()
            return False

    def _cleanup_after_failed_connect(self) -> None:
        """connect 失败时释放已建 channel/stub 并强制 bridge 进入未启动状态。"""
        # 第一行置位 _stopping：竞态防护，确保后续清理时 guard 已生效。
        self._stopping = True
        # 先关 channel 让底层连接断开，再清 stub 防止后续误用。
        if self._channel is not None:
            try:
                self._channel.close()
            except Exception:  # pragma: no cover - 关 channel 失败不影响 reset 目标
                pass
        self._channel = None
        self._stub = None
        self._connected = False
        # 关键：让 bridge 子进程对象也回到「未启动」状态，使下一次 start() 走完整 spawn。
        # 注意只清状态不杀进程：调用方后续会走 _start_geometry_runtime_subprocess 的 zombie 自愈路径，
        # 视情况 stop() 旧 bridge 或直接 spawn 新 bridge 取代。
        try:
            self._bridge.disconnect()
        except Exception as exc:  # pragma: no cover - disconnect 失败不影响主流程
            LOGGER.warning("bridge.disconnect 失败（不影响主流程）: %s", exc)

    def disconnect(self) -> None:
        # 第一行置位 _stopping：竞态防护，确保 guard 在 channel 关闭前生效。
        self._stopping = True
        if self._channel:
            self._channel.close()
            self._channel = None
            self._stub = None
            self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def _raise_if_stopping(self, rpc_name: str) -> None:
        """shutdown 守卫：_stopping=True 且 guard 启用时，所有 RPC 入口抛 shutting_down。

        防止 shutdown 期间 in-flight task 持陈旧 _stub 引用继续打 RPC 全 UNAVAILABLE。
        调用方（task runner 等）应 catch RuntimeError 优雅退出（AC4 期望 DEBUG 日志）。
        通过 NUWAX_GEOMETRY_SHUTDOWN_GUARD=0 可紧急回滚禁用此守卫。
        """
        if self._stopping and _is_shutdown_guard_enabled():
            LOGGER.debug("GeometryRuntimeProvider.%s 在 shutdown 期被阻断", rpc_name)
            raise RuntimeError("shutting_down")

    def is_available(self) -> bool:
        self._raise_if_stopping("is_available")
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
        self._raise_if_stopping("get_current_pose")
        if not self._stub:
            return None
        try:
            resp = self._stub.GetCurrentPose(pb2.PoseRequest(), timeout=_pose_deadline_sec())
            return _build_pose(resp)
        except grpc.RpcError as e:
            _log_rpc_error("GetCurrentPose", e)
            return None

    def get_frame_tree(self) -> Optional[FrameTree]:
        self._raise_if_stopping("get_frame_tree")
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
        self._raise_if_stopping("get_occupancy_grid_meta")
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
        self._raise_if_stopping("get_occupancy_grid_data")
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
        self._raise_if_stopping("set_navigation_goal")
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
        self._raise_if_stopping("cancel_navigation")
        if not self._stub:
            return False
        try:
            resp = self._stub.CancelNavigation(pb2.Empty(), timeout=_read_deadline_sec())
            return resp.success
        except grpc.RpcError as e:
            _log_rpc_error("CancelNavigation", e)
            return False

    def get_navigation_readiness(self) -> Dict[str, Any]:
        self._raise_if_stopping("get_navigation_readiness")
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

    # === NavigationProvider 协议 shim ===
    # 去 ROS2 目标态（round-4 HIGH 3 修复）：
    # - provider bundle / runtime.py 直调 backend.set_goal/cancel_goal/get_navigation_state，
    #   而 GeometryRuntimeProvider gRPC facade 的真实方法名是 set_navigation_goal / cancel_navigation
    #   / (subscribe-driven 状态缓存)。
    # - 这里在 provider 侧补齐旧 NavigationProvider 协议方法名，全部委托到 gRPC facade；
    #   不假装成功——gRPC 失败或 backend 未就绪时，返回值如实反映（accepted=False / success=False / IDLE）。
    # - 等 nuwax-nav-stack crate 上线、Nav2 动作语义彻底废弃后，可删除整个 shim 段。
    def _ensure_nav_state_stream(self) -> None:
        """懒启动 SubscribeNavigationState 后台线程；只起一次。"""
        if self._nav_stream_started:
            return
        if not self._stub or self._stopping:
            return
        self._nav_stream_started = True
        thread = threading.Thread(
            target=self._nav_state_stream_worker,
            name="geometry_runtime_nav_state_cache",
            daemon=True,
        )
        thread.start()

    def _nav_state_stream_worker(self) -> None:
        """后台线程：拉 SubscribeNavigationState 流，把最近一帧写进本地缓存。

        流断开时不重连（避免 hot loop），下次 set_goal/get_navigation_state 触发的 ensure
        仍会 no-op 已启动标志；如确需重连，只能 disconnect+connect 重建 provider。
        """
        while self._stub and not self._stopping:
            try:
                for update in self._stub.SubscribeNavigationState(pb2.Empty()):
                    goal_id = getattr(update, "goal_id", "") or None
                    status = getattr(update, "status", "") or ""
                    message = getattr(update, "message", "") or None
                    distance = getattr(update, "distance_remaining", None)
                    pose = _build_pose(getattr(update, "current_pose", None))
                    ts_ns = int(getattr(update, "timestamp_ns", 0) or 0)
                    with self._nav_state_lock:
                        self._last_nav_goal_id = goal_id
                        self._last_nav_status = status
                        self._last_nav_message = message
                        self._last_nav_distance_remaining = (
                            float(distance) if distance is not None else None
                        )
                        self._last_nav_current_pose = pose
                        self._last_nav_timestamp_ns = ts_ns
                    if not self._stub or self._stopping:
                        return
            except grpc.RpcError as exc:
                LOGGER.warning(
                    "SubscribeNavigationState 流断开（不影响主流程）: %s", exc
                )
                return  # 不重连，下次 ensure 也不再启动（按设计）
            except Exception:  # pragma: no cover - 防御性兜底
                LOGGER.exception("SubscribeNavigationState worker 异常退出。")
                return

    @staticmethod
    def _nav_status_to_enum(raw_status: str) -> NavigationStatus:
        """把 gRPC 流推送的 status 字符串映射成 NavigationStatus 枚举。

        未识别字符串映射为 IDLE（保守），绝不向上伪造 SUCCEEDED。
        """
        normalized = (raw_status or "").strip().lower()
        mapping = {
            "idle": NavigationStatus.IDLE,
            "accepted": NavigationStatus.ACCEPTED,
            "planning": NavigationStatus.PLANNING,
            "running": NavigationStatus.RUNNING,
            "succeeded": NavigationStatus.SUCCEEDED,
            "success": NavigationStatus.SUCCEEDED,
            "completed": NavigationStatus.SUCCEEDED,
            "failed": NavigationStatus.FAILED,
            "failure": NavigationStatus.FAILED,
            "cancelled": NavigationStatus.CANCELLED,
            "canceled": NavigationStatus.CANCELLED,
        }
        return mapping.get(normalized, NavigationStatus.IDLE)

    def _build_navigation_state_snapshot(self) -> NavigationState:
        """合成 NavigationState 契约（按本地缓存 + readiness，不假装 ready）。

        - 未收到任何 SubscribeNavigationState 帧：返回 IDLE；
        - readiness 不就绪：state.message 携带 readiness 提示；
        - 缓存的 current_goal_id 与最近一次 set_goal 的 goal_id 不一致：以缓存为准
          （流是 Rust 侧权威来源）。
        """
        with self._nav_state_lock:
            raw_status = self._last_nav_status
            goal_id = self._last_nav_goal_id
            message = self._last_nav_message
            distance = self._last_nav_distance_remaining
            pose = self._last_nav_current_pose

        status_enum = self._nav_status_to_enum(raw_status)
        # 没有任何流帧 → IDLE，goal_id 留空，绝不假装有 current goal
        if not raw_status:
            return NavigationState(
                status=NavigationStatus.IDLE,
                current_goal_id=None,
                goal_reached=False,
                message="GeometryRuntimeProvider 尚未收到 SubscribeNavigationState 流更新。",
                metadata={"provider": "geometry_runtime", "shim": True},
            )

        # readiness 兜底：未就绪时在 message 上补 waiting_for 提示（不伪造状态）
        metadata: Dict[str, Any] = {"provider": "geometry_runtime", "shim": True}
        try:
            readiness = self.get_navigation_readiness()
            if readiness:
                metadata["nav2_readiness"] = dict(readiness)
                if not bool(readiness.get("ready")):
                    hint = readiness.get("message") or readiness.get("waiting_for") or "导航后端未就绪"
                    if message:
                        message = "%s | %s" % (message, hint)
                    else:
                        message = str(hint)
        except Exception:  # pragma: no cover - readiness 失败不影响主流程
            pass

        goal_reached = status_enum == NavigationStatus.SUCCEEDED

        return NavigationState(
            status=status_enum,
            current_goal_id=goal_id,
            current_pose=pose,
            remaining_distance_m=distance,
            remaining_yaw_rad=None,
            goal_reached=goal_reached,
            message=message,
            metadata=metadata,
        )

    def set_goal(self, goal) -> bool:
        """NavigationProvider 协议 shim：下发导航目标到 nuwax-nav-stack。

        参数：
            goal: contracts.navigation.NavigationGoal 契约实例（runtime.py 的
                  _dispatch_navigate / platform_exploration_runtime 实际传入的对象）。

        返回：
            bool: gRPC NavigationAck.accepted 透传；调用失败 / stub 不可用返回 False，
            绝不假装成功。
        """
        self._raise_if_stopping("set_goal")
        if not self._stub:
            LOGGER.warning("GeometryRuntimeProvider.set_goal stub 不可用，返回 False。")
            return False
        target_pose = getattr(goal, "target_pose", None)
        if target_pose is None:
            LOGGER.warning(
                "GeometryRuntimeProvider.set_goal goal=%s 缺少 target_pose，返回 False。",
                getattr(goal, "goal_id", None),
            )
            return False
        try:
            proto_pose = pb2.Pose(
                position=pb2.Vec3(
                    x=float(target_pose.position.x),
                    y=float(target_pose.position.y),
                    z=float(target_pose.position.z),
                ),
                orientation=pb2.Quat(
                    x=float(target_pose.orientation.x),
                    y=float(target_pose.orientation.y),
                    z=float(target_pose.orientation.z),
                    w=float(target_pose.orientation.w),
                ),
                frame_id=str(getattr(target_pose, "frame_id", "") or ""),
            )
            request_goal_id = str(getattr(goal, "goal_id", "") or "")
            proto_goal = pb2.NavigationGoal(
                target_pose=proto_pose,
                behavior_tree=request_goal_id or "",
            )
            resp = self._stub.SetNavigationGoal(proto_goal, timeout=_read_deadline_sec())
            accepted = bool(getattr(resp, "accepted", False))
            ack_goal_id = getattr(resp, "goal_id", "") or request_goal_id or None
            ack_message = getattr(resp, "message", "") or None
            # 同步本地缓存：accepted 时把状态置为 ACCEPTED，等待 SubscribeNavigationState 流覆盖
            with self._nav_state_lock:
                if accepted:
                    self._last_nav_goal_id = ack_goal_id
                    self._last_nav_status = "accepted"
                    self._last_nav_message = ack_message
                    self._last_nav_distance_remaining = None
                    self._last_nav_current_pose = None
                    self._last_nav_timestamp_ns = 0
            # accepted 后再异步起流，避免 set_goal 同步路径被阻塞
            self._ensure_nav_state_stream()
            return accepted
        except grpc.RpcError as exc:
            _log_rpc_error("SetNavigationGoal", exc)
            return False
        except Exception:  # pragma: no cover - 防御性兜底
            LOGGER.exception("GeometryRuntimeProvider.set_goal 异常。")
            return False

    def cancel_goal(self, request_id: Optional[str] = None) -> bool:
        """NavigationProvider 协议 shim：取消当前导航目标。

        request_id 参数在 Rust facade 当前实现下未消费（CancelNavigation 不接受参数），
        保留以匹配 NavigationProvider 协议签名。
        """
        self._raise_if_stopping("cancel_goal")
        if not self._stub:
            LOGGER.warning("GeometryRuntimeProvider.cancel_goal stub 不可用，返回 False。")
            return False
        try:
            resp = self._stub.CancelNavigation(pb2.Empty(), timeout=_read_deadline_sec())
            success = bool(getattr(resp, "success", False))
            cancel_message = getattr(resp, "message", "") or None
            with self._nav_state_lock:
                if success:
                    self._last_nav_status = "cancelled"
                    if cancel_message:
                        self._last_nav_message = cancel_message
            return success
        except grpc.RpcError as exc:
            _log_rpc_error("CancelNavigation", exc)
            return False
        except Exception:  # pragma: no cover
            LOGGER.exception("GeometryRuntimeProvider.cancel_goal 异常。")
            return False

    def is_goal_reached(self) -> bool:
        """NavigationProvider 协议 shim：当前目标是否已到达。

        基于本地缓存（SubscribeNavigationState 最近一帧的 status）。
        """
        with self._nav_state_lock:
            return self._last_nav_status == "succeeded"

    def get_navigation_state(self) -> NavigationState:
        """NavigationProvider 协议 shim：返回 NavigationState 契约实例。

        第一次调用前若无 set_goal/get_navigation_state 触发，状态为 IDLE；订阅流启动后
        Rust 端推送会持续覆盖本地缓存。
        """
        self._ensure_nav_state_stream()
        return self._build_navigation_state_snapshot()

    # === 输入桥 ===
    def publish_odom(self, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float,
                     lvx: float = 0, lvy: float = 0, lvz: float = 0,
                     avx: float = 0, avy: float = 0, avz: float = 0,
                     frame_id: str = "odom") -> bool:
        self._raise_if_stopping("publish_odom")
        if not self._stub:
            return False
        try:
            msg = pb2.OdomInput(
                pose=pb2.Pose(
                    position=pb2.Vec3(x=x, y=y, z=z),
                    orientation=pb2.Quat(x=qx, y=qy, z=qz, w=qw),
                    frame_id=frame_id,
                ),
                linear_velocity=pb2.Vec3(x=lvx, lvy=lvy, z=lvz),
                angular_velocity=pb2.Vec3(x=avx, y=avy, z=avz),
            )
            self._stub.PublishOdom(msg, timeout=_read_deadline_sec())
            return True
        except grpc.RpcError as e:
            _log_rpc_error("PublishOdom", e)
            return False

    def get_status(self) -> Dict[str, Any]:
        self._raise_if_stopping("get_status")
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
        t = threading.Thread(target=_stream_worker, name="geometry_runtime_pose_stream", daemon=True)
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
        t = threading.Thread(target=_stream_worker, name="geometry_runtime_map_stream", daemon=True)
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
        t = threading.Thread(target=_stream_worker, name="geometry_runtime_nav_stream", daemon=True)
        t.start()
        return t
