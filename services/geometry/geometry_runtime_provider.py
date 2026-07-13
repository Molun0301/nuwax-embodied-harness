"""GeometryRuntimeProvider — 通过 gRPC 调用 Rust nuwax-geometry-runtime 子进程。

Rust 运行时是主要的平台几何交互层（替代旧 nuwax-ros-bridge），本模块封装其 gRPC 接口，
提供与原始 Python Provider 相同的调用接口。
"""
from __future__ import annotations

import json
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


# --- NavigationState.status 字符串语义映射（round-4 HIGH 3 同步） ---
# Rust proto SetNavigationGoal / CancelNavigation / SubscribeNavigationState
# 全部返回/推送 NavigationState，没有 Rust 端 NavigationAck.accepted / CancelResult.success
# 之类的布尔字段；accepted / success 语义由 status 字符串反推。
_NAV_ACCEPTED_STATUS = frozenset({
    "",                # 空 status 也视作"服务端接受但尚未分类"，避免误判为失败
    "accepted",
    "queued",
    "pending",
    "planning",
    "running",
    "active",
    "executing",
    "in_progress",
})
_NAV_SUCCEEDED_STATUS = frozenset({
    "succeeded",
    "success",
    "completed",
    "reached",
})
_NAV_CANCELLED_STATUS = frozenset({
    "cancelled",
    "canceled",
    "aborted",
    "stopped",
})


def _status_means_accepted(status: str) -> bool:
    """SetNavigationGoal 返回时：服务端是否接受请求（非失败类）。"""
    return str(status or "").strip().lower() in _NAV_ACCEPTED_STATUS


def _status_means_succeeded(status: str) -> bool:
    """navigation 是否完成。"""
    return str(status or "").strip().lower() in _NAV_SUCCEEDED_STATUS


def _status_means_cancelled(status: str) -> bool:
    """CancelNavigation 返回时：是否被服务端视为已取消。"""
    return str(status or "").strip().lower() in _NAV_CANCELLED_STATUS


def _is_shutdown_guard_enabled() -> bool:
    """NUWAX_GEOMETRY_SHUTDOWN_GUARD=0 时关闭（紧急回滚）。"""
    value = os.environ.get("NUWAX_GEOMETRY_SHUTDOWN_GUARD", "1").strip().lower()
    return value not in {"0", "false", "no", "off"}


# Phase 4 nav_runtime:`get_navigation_readiness` 把细分 readiness 子项
# JSON 序列化塞到 `reason` 字段尾部。本模块把该 dict 在 Python 侧解析出来。
# Phase 7 在此基础上新增 `backend` / `tf_count` / `costmap_revision` /
# `cmd_vel_sink_ready` / `active_profile` 五个字段,下游 diagnostics
# 调用方可逐一读取,不必 break 旧字段。
_NAV_READINESS_FIELDS = (
    "lcm_backend",
    "tf",
    "costmap",
    "planner",
    "cmd_vel_sink",
    "profile",
    "module_node_count",
    "p1_native_profiles",
    "backend",
    "tf_count",
    "costmap_revision",
    "cmd_vel_sink_ready",
    "active_profile",
)


def _parse_navigation_readiness_reason(ready: bool, reason: str) -> Dict[str, Any]:
    """从 gRPC `NavigationReadiness.reason` 提取细分字段。

    Rust 端格式:``"{top_reason}; details={...json...}"``。
    解析失败或字段缺失时,top-level reason 仍保留,子项用 `None` 占位,
    避免上层 `readiness["planner"]` 抛 `KeyError`。
    """
    result: Dict[str, Any] = {
        "ready": ready,
        "message": reason,
        "lcm_backend": None,
        "tf": None,
        "costmap": None,
        "planner": None,
        "cmd_vel_sink": None,
        "profile": None,
        "module_node_count": None,
        "p1_native_profiles": None,
    }
    marker = "; details="
    idx = reason.find(marker)
    if idx < 0:
        return result
    top_reason = reason[:idx].strip()
    json_str = reason[idx + len(marker) :].strip()
    result["message"] = top_reason
    try:
        details = json.loads(json_str)
    except (ValueError, TypeError):
        return result
    if not isinstance(details, dict):
        return result
    for field in _NAV_READINESS_FIELDS:
        if field in details:
            result[field] = details[field]
    return result


def _nav_readiness_disconnected_dict(message: str = "未连接") -> Dict[str, Any]:
    """``_stub`` 未就绪 / gRPC 失败时返回的最小 dict。"""
    return {
        "ready": False,
        "message": message,
        "lcm_backend": None,
        "tf": None,
        "costmap": None,
        "planner": None,
        "cmd_vel_sink": None,
        "profile": None,
        "module_node_count": None,
        "p1_native_profiles": None,
        # Phase 7 增量字段
        "backend": None,
        "tf_count": None,
        "costmap_revision": None,
        "cmd_vel_sink_ready": None,
        "active_profile": None,
    }


def _parse_readiness_message(message: str) -> Dict[str, Any]:
    """把 Phase 7 gRPC `MappingReadiness.message` / `ExplorationReadiness.message`
    中的 JSON 字符串解析为 dict。失败时返回空 dict(调用方负责补默认值)。

    该 helper 与 `_parse_navigation_readiness_reason` 形状相似但 payload 不同:
    - mapping: {lcm_backend, adapter_present, odom_count, pointcloud_count,
                last_occupancy_revision, accumulated_point_count, last_error}
    - exploration: {backend_available, controller_active, source_count,
                    frontier_source_ready, costmap_input_ready,
                    costmap_input_revision, active_source, arbitration}
    """
    try:
        data = json.loads(message or "{}")
    except (ValueError, TypeError):
        return {}
    if not isinstance(data, dict):
        return {}
    return data


def _normalize_frame(frame_id: str) -> str:
    """去掉 ROS 风格前导斜杠并去空白，便于契约校验。"""
    return str(frame_id or "").strip().lstrip("/")


def _exploration_state_to_dict(resp: Any) -> Dict[str, Any]:
    """把 proto `ExplorationState` 转成 dict(给 `GeometryRuntimeProvider` 对外契约用)。

    Phase 5:与 dimos `nuwax_exploration::ExplorationController` 的 status 字符串
    一一对应:ACCEPTED / REJECTED / PENDING / STOPPED / ACTIVE / UNIMPLEMENTED。
    失败时上游 RPC 层已经处理,这里只做字段拷贝,不二次校验。
    """
    return {
        "status": resp.status,
        "message": resp.message,
        "request_id": resp.request_id,
        "selected_source": resp.selected_source,
        "last_stop_reason": resp.last_stop_reason,
        "frontier_count": int(resp.frontier_count),
        "revision": int(resp.revision),
        "timestamp_ns": int(resp.timestamp_ns),
    }


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

    # === Phase 3: mapping / lcm backend diagnostics ===
    def is_mapping_ready(self) -> bool:
        """Phase 3: mapping 板块 readiness。

        走 gRPC `GetMappingReadiness`，不把进程级 `HealthCheck.healthy`
        误当作 mapping capability ready。该方法与
        `runtime_diagnostics.rs::mapping_readiness` 对账。

        返回 True —— runtime 健康 + mapping adapter 已就绪(有点云数据)。
        返回 False —— 未连接 / backend 不可用 / mapping adapter 未实装 /
                      无点云数据。
        """
        diagnostics = self.get_mapping_diagnostics()
        return bool(diagnostics and diagnostics.get("ready", False))

    def get_lcm_backend_name(self) -> str:
        """Phase 3: 取 canonical LCM backend 名。

2026-07-08 fake-lcm 物理删除后:返回值固定为 `RealLcm` 或
`real-lcm-unavailable`(liblcm 不可用时);fake-lcm 已物理删除(v3 F2 决策)。"""
        if not self._connected or not self._stub:
            return "<disconnected>"
        try:
            resp = self._stub.HealthCheck(
                pb2.HealthCheckRequest(), timeout=_pose_deadline_sec()
            )
            return str(getattr(resp, "lcm_backend", "") or "<unknown>")
        except grpc.RpcError as e:
            _log_rpc_error("HealthCheck(get_lcm_backend_name)", e)
            return "<unknown>"

    # === Phase 6: mapping / nav / explore diagnostics facade ===
    # 阶段目标:让 Python gateway/service 只感知 provider 协议,不再依赖历史
    # 外部脚本或 nav2 命名。provider 对外暴露三套统一 diagnostics dict:
    #   - get_mapping_diagnostics()    : 建图板块
    #   - get_navigation_diagnostics() : 导航板块
    #   - get_exploration_diagnostics(): 探索板块(已在 Phase 5 实装,本阶段保留入口)
    # 任一方法失败都返回 None,绝不假成功。
    def get_mapping_diagnostics(self) -> Optional[Dict[str, Any]]:
        """返回 mapping 板块 diagnostics dict(Phase 7:走 `GetMappingReadiness`)。

        Phase 7:不再只用 HealthCheck + GetOccupancyGrid 拼凑,直接走新增的
        `GetMappingReadiness` RPC,`message` 字段为 JSON dict,涵盖
        odom_count / pointcloud_count / last_occupancy_revision /
        accumulated_point_count / last_error / adapter_present。

        字段:
          - ``ready`` (bool): 综合 ready(bool)
          - ``lcm_backend`` (str): canonical LCM backend 名(从 message JSON 解析)
          - ``adapter_present`` (bool): mapping adapter 是否已构造
          - ``odom_count`` (int): 累计 odom 帧数
          - ``pointcloud_count`` (int): 累计 pointcloud 帧数
          - ``last_occupancy_revision`` (int): 最近 occupancy revision
          - ``accumulated_point_count`` (int): accumulator 累计点数
          - ``last_error`` (str|None): 失败原因
          - ``error`` (str|None): RPC 层失败原因

        返回 None —— 子进程未连接。
        """
        if not self._connected or not self._stub:
            return None
        # Phase 7:主路径走新增 GetMappingReadiness。
        try:
            resp = self._stub.GetMappingReadiness(
                pb2.Empty(), timeout=_read_deadline_sec()
            )
            payload = _parse_readiness_message(getattr(resp, "message", "") or "{}")
            payload["ready"] = bool(getattr(resp, "ready", False))
            payload["error"] = None
            # fallback 字段名兼容旧调用方。
            payload.setdefault("has_occupancy_grid", payload.get("last_occupancy_revision", 0) > 0)
            payload.setdefault("occupancy_revision", payload.get("last_occupancy_revision"))
            return payload
        except grpc.RpcError as e:
            _log_rpc_error("GetMappingReadiness", e)
            # 兜底:旧 HealthCheck + GetOccupancyGrid 路径,保持向后兼容。
            try:
                health = self._stub.HealthCheck(
                    pb2.HealthCheckRequest(), timeout=_pose_deadline_sec()
                )
                lcm_backend = str(getattr(health, "lcm_backend", "") or "<unknown>")
            except grpc.RpcError as e2:
                _log_rpc_error("HealthCheck(get_mapping_diagnostics fallback)", e2)
                return {
                    "ready": False,
                    "lcm_backend": "<unknown>",
                    "odom_count": None,
                    "pointcloud_count": None,
                    "last_occupancy_revision": None,
                    "accumulated_point_count": None,
                    "adapter_present": None,
                    "last_error": None,
                    "has_occupancy_grid": False,
                    "occupancy_revision": None,
                    "error": "GetMappingReadiness+HealthCheck 失败: %s / %s" % (e, e2),
                }
            return {
                "ready": False,
                "lcm_backend": lcm_backend,
                "odom_count": None,
                "pointcloud_count": None,
                "last_occupancy_revision": None,
                "accumulated_point_count": None,
                "adapter_present": None,
                "last_error": "GetMappingReadiness 不可用: %s" % e,
                "has_occupancy_grid": False,
                "occupancy_revision": None,
                "error": "GetMappingReadiness 失败: %s" % e,
            }

    def get_navigation_diagnostics(self) -> Optional[Dict[str, Any]]:
        """返回 navigation 板块 diagnostics dict(走 `GetNavigationReadiness`)。

        字段:
          - ``ready`` (bool): nuwax nav_runtime + lcm backend 都健康
          - ``lcm_backend`` (str): canonical LCM backend 名
          - ``readiness`` (dict): `GetNavigationReadiness` 解析后字段
            (ready/message/lcm_backend/tf/costmap/planner/cmd_vel_sink/profile/
             module_node_count/p1_native_profiles)
          - ``profile`` / ``active_profile`` (str|None): 当前 nav profile
            (Phase 7 兼容旧 `profile` + 新 `active_profile`)
          - ``backend`` (str|None): 2026-07-08 fake-lcm 物理删除后字段值固定为
            "RealLcm" 或 "Unknown"(未启动时);fake-lcm 已物理删除(v3 F2 决策),无分支
          - ``tf_count`` (int|None): 累计 tf 帧数
          - ``costmap_revision`` (int|None): 当前 occupancy revision
          - ``cmd_vel_sink_ready`` (bool|None): cmd_vel consumer + watchdog 状态
          - ``module_node_count`` / ``p1_native_profiles``: 兼容旧字段
          - ``error`` (str|None): 失败原因

        返回 None —— 子进程未连接。
        """
        if not self._connected or not self._stub:
            return None
        lcm_backend = "<unknown>"
        try:
            health = self._stub.HealthCheck(
                pb2.HealthCheckRequest(), timeout=_pose_deadline_sec()
            )
            lcm_backend = str(getattr(health, "lcm_backend", "") or "<unknown>")
        except grpc.RpcError as e:
            _log_rpc_error("HealthCheck(get_navigation_diagnostics)", e)
            return {
                "ready": False,
                "lcm_backend": "<unknown>",
                "readiness": None,
                "profile": None,
                "active_profile": None,
                "backend": None,
                "tf_count": None,
                "costmap_revision": None,
                "cmd_vel_sink_ready": None,
                "module_node_count": None,
                "p1_native_profiles": None,
                "error": "HealthCheck 失败: %s" % e,
            }
        readiness = self.get_navigation_readiness()
        profile = readiness.get("profile") if isinstance(readiness, dict) else None
        active_profile = (
            readiness.get("active_profile") if isinstance(readiness, dict) else None
        ) or profile
        module_node_count = (
            readiness.get("module_node_count") if isinstance(readiness, dict) else None
        )
        p1_native_profiles = (
            readiness.get("p1_native_profiles") if isinstance(readiness, dict) else None
        )
        backend = readiness.get("backend") if isinstance(readiness, dict) else None
        tf_count = readiness.get("tf_count") if isinstance(readiness, dict) else None
        costmap_revision = (
            readiness.get("costmap_revision") if isinstance(readiness, dict) else None
        )
        cmd_vel_sink_ready = (
            readiness.get("cmd_vel_sink_ready") if isinstance(readiness, dict) else None
        )
        return {
            "ready": bool(getattr(health, "healthy", False)) and bool(
                isinstance(readiness, dict) and readiness.get("ready")
            ),
            "lcm_backend": lcm_backend,
            "readiness": readiness,
            "profile": profile,
            "active_profile": active_profile,
            "backend": backend,
            "tf_count": tf_count,
            "costmap_revision": costmap_revision,
            "cmd_vel_sink_ready": cmd_vel_sink_ready,
            "module_node_count": module_node_count,
            "p1_native_profiles": p1_native_profiles,
            "error": None,
        }

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
    # Rust proto 字段映射（round-4 HIGH 3 同步）：
    # - NavigationGoal.target_pose/behavior_tree → pose/frame_id/goal_id
    # - SetNavigationGoal 返回 NavigationState（非 NavigationAck）：
    #   accepted/goal_id/message → status/goal_id/message
    # - CancelNavigation 接受 CancelNavigationRequest（带 goal_id），返回 NavigationState
    # - NavigationReadiness 仅 ready + reason（去 Nav2 残留字段）
    def set_navigation_goal(self, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float,
                            frame_id: str = "map", goal_id: str = "") -> Dict[str, Any]:
        self._raise_if_stopping("set_navigation_goal")
        if not self._stub:
            return {"accepted": False, "message": "未连接"}
        try:
            goal = pb2.NavigationGoal(
                pose=pb2.Pose(
                    position=pb2.Vec3(x=x, y=y, z=z),
                    orientation=pb2.Quat(x=qx, y=qy, z=qz, w=qw),
                    frame_id=frame_id,
                ),
                frame_id=frame_id,
                goal_id=goal_id,
            )
            resp = self._stub.SetNavigationGoal(goal, timeout=_read_deadline_sec())
            return {
                "accepted": _status_means_accepted(resp.status),
                "goal_id": resp.goal_id,
                "message": resp.message,
            }
        except grpc.RpcError as e:
            _log_rpc_error("SetNavigationGoal", e)
            return {"accepted": False, "message": str(e)}

    def cancel_navigation(self, goal_id: str = "") -> bool:
        self._raise_if_stopping("cancel_navigation")
        if not self._stub:
            return False
        try:
            req = pb2.CancelNavigationRequest(goal_id=goal_id)
            resp = self._stub.CancelNavigation(req, timeout=_read_deadline_sec())
            return (
                _status_means_cancelled(resp.status)
                or _status_means_accepted(resp.status)
                or _status_means_succeeded(resp.status)
            )
        except grpc.RpcError as e:
            _log_rpc_error("CancelNavigation", e)
            return False

    def get_navigation_readiness(self) -> Dict[str, Any]:
        """Phase 4:从 gRPC `GetNavigationReadiness` 返回 dict。

        Rust 端把细分 readiness 子项(LCM backend / tf / costmap /
        planner / cmd_vel sink)JSON 序列化塞到 `reason` 字段尾部,
        形如 `"{top_reason}; details={...json...}"`,本函数解析
        返回 dict。

        返回 dict 字段:
        - ``ready`` (bool): 整体 ready
        - ``message`` (str): 顶层 reason
        - ``lcm_backend`` / ``tf`` / ``costmap`` / ``planner`` /
          ``cmd_vel_sink`` (bool): 子项
        - ``profile`` (str): 当前 NUWAX_NAV_PROFILE
        - ``module_node_count`` (int): ModuleGraph 节点数
        - ``p1_native_profiles`` (list[str]): 本期未启用的 native profile
        """
        self._raise_if_stopping("get_navigation_readiness")
        if not self._stub:
            return _nav_readiness_disconnected_dict()
        try:
            resp = self._stub.GetNavigationReadiness(
                pb2.Empty(), timeout=_read_deadline_sec()
            )
            return _parse_navigation_readiness_reason(
                bool(resp.ready), str(resp.reason)
            )
        except grpc.RpcError as e:
            _log_rpc_error("GetNavigationReadiness", e)
            return _nav_readiness_disconnected_dict(message="gRPC 错误")

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
                    # Rust SubscribeNavigationState 流推送 NavigationState
                    # （非 NavigationStateUpdate，无 current_pose 字段；位姿由 SubscribePose 流补齐）
                    goal_id = getattr(update, "goal_id", "") or None
                    status = getattr(update, "status", "") or ""
                    message = getattr(update, "message", "") or None
                    distance = getattr(update, "distance_remaining", None)
                    ts_ns = int(getattr(update, "timestamp_ns", 0) or 0)
                    with self._nav_state_lock:
                        self._last_nav_goal_id = goal_id
                        self._last_nav_status = status
                        self._last_nav_message = message
                        self._last_nav_distance_remaining = (
                            float(distance) if distance is not None else None
                        )
                        self._last_nav_current_pose = None  # 由 SubscribePose 流维护
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
            "pending": NavigationStatus.ACCEPTED,
            "queued": NavigationStatus.ACCEPTED,
            "planning": NavigationStatus.PLANNING,
            "running": NavigationStatus.RUNNING,
            "moving": NavigationStatus.RUNNING,
            "succeeded": NavigationStatus.SUCCEEDED,
            "success": NavigationStatus.SUCCEEDED,
            "completed": NavigationStatus.SUCCEEDED,
            "arrived": NavigationStatus.SUCCEEDED,
            "reached": NavigationStatus.SUCCEEDED,
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
            bool: 由 NavigationState.status 经 _status_means_accepted 反推（accepted/
            planning/running → True；failed/cancelled → False）；调用失败 / stub 不可用
            返回 False，绝不假装成功。
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
            target_frame_id = str(getattr(target_pose, "frame_id", "") or "") or "map"
            proto_goal = pb2.NavigationGoal(
                pose=proto_pose,
                frame_id=target_frame_id,
                goal_id=request_goal_id or "",
            )
            resp = self._stub.SetNavigationGoal(proto_goal, timeout=_read_deadline_sec())
            # Rust SetNavigationGoal 返回 NavigationState（非 NavigationAck）；
            # accepted 语义由 status 字符串反推（accepted/planning/running → True）
            accepted = _status_means_accepted(getattr(resp, "status", ""))
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
            req = pb2.CancelNavigationRequest(goal_id=request_id or "")
            resp = self._stub.CancelNavigation(req, timeout=_read_deadline_sec())
            raw_status = getattr(resp, "status", "") or ""
            terminal = _status_means_cancelled(raw_status)
            success = terminal or _status_means_accepted(raw_status)
            cancel_message = getattr(resp, "message", "") or None
            with self._nav_state_lock:
                if terminal:
                    self._last_nav_status = "cancelled"
                    if cancel_message:
                        self._last_nav_message = cancel_message
                elif success:
                    self._last_nav_status = raw_status.strip().lower() or "pending"
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

    # === 探索 RPC（Phase 5 additive, 走 Rust explore_runtime） ===
    def start_exploration(
        self,
        *,
        request_id: str = "",
        strategy: str = "wavefront",
        arbitration: str = "prefer_wavefront",
        map_version_id: str = "",
    ) -> Optional[Dict[str, Any]]:
        """启动探索(走 Rust `nuwax_exploration::ExplorationController`)。

        返回 dict 表示 ExplorationState 字段(status/message/request_id/
        selected_source/last_stop_reason/frontier_count/revision/timestamp_ns);
        stub 未就绪或 RPC 异常时返回 None,绝不假装成功。
        """
        self._raise_if_stopping("start_exploration")
        if not self._stub:
            return None
        try:
            req = pb2.ExploreRequest(
                strategy=strategy,
                arbitration=arbitration,
                map_version_id=map_version_id,
                request_id=request_id,
            )
            resp = self._stub.StartExploration(req, timeout=_read_deadline_sec())
            return _exploration_state_to_dict(resp)
        except grpc.RpcError as e:
            _log_rpc_error("StartExploration", e)
            return None

    def stop_exploration(self, *, request_id: str = "") -> Optional[Dict[str, Any]]:
        """停止探索。

        返回 ExplorationState 字典(同 `start_exploration`),失败时 None。
        """
        self._raise_if_stopping("stop_exploration")
        if not self._stub:
            return None
        try:
            req = pb2.StopExplorationRequest(request_id=request_id)
            resp = self._stub.StopExploration(req, timeout=_read_deadline_sec())
            return _exploration_state_to_dict(resp)
        except grpc.RpcError as e:
            _log_rpc_error("StopExploration", e)
            return None

    def get_exploration_state(self) -> Optional[Dict[str, Any]]:
        """读取当前 exploration controller 状态(单次 tick)。

        返回 ExplorationState 字典;失败时 None。
        """
        self._raise_if_stopping("get_exploration_state")
        if not self._stub:
            return None
        try:
            resp = self._stub.GetExplorationState(
                pb2.Empty(), timeout=_read_deadline_sec()
            )
            return _exploration_state_to_dict(resp)
        except grpc.RpcError as e:
            _log_rpc_error("GetExplorationState", e)
            return None

    def get_exploration_diagnostics(self) -> Optional[Dict[str, Any]]:
        """读取探索 diagnostics(Phase 7:主路径走新增的 `GetExplorationReadiness`)。

        返回 dict 字段:
          - 旧字段(active / arbitration / selected_source / last_stop_reason /
            sources(list) / frontier_count / revision / timestamp_ns)
          - Phase 7 readiness 字段(ready / backend_available / controller_active /
            source_count / frontier_source_ready / costmap_input_ready /
            costmap_input_revision / active_source / message)

        失败时返回 None。
        """
        self._raise_if_stopping("get_exploration_diagnostics")
        if not self._stub:
            return None
        # Phase 7:主路径走 GetExplorationReadiness(JSON message)。
        try:
            resp = self._stub.GetExplorationReadiness(
                pb2.Empty(), timeout=_read_deadline_sec()
            )
            payload = _parse_readiness_message(getattr(resp, "message", "") or "{}")
            payload["ready"] = bool(getattr(resp, "ready", False))
            # 兼容旧字段(snake_case → 已有 Python 调用方)
            payload.setdefault("backend_available", payload.get("backend_available"))
            payload.setdefault("controller_active", payload.get("controller_active"))
            payload.setdefault("active_source", payload.get("active_source"))
            payload.setdefault("sources", [])
            payload.setdefault("frontier_count", 0)
            payload.setdefault("selected_source", payload.get("active_source", ""))
            payload.setdefault("last_stop_reason", "")
            payload.setdefault("revision", payload.get("costmap_input_revision"))
            payload.setdefault("timestamp_ns", 0)
            payload.setdefault("arbitration", payload.get("arbitration"))
            payload["error"] = None
            return payload
        except grpc.RpcError as e:
            _log_rpc_error("GetExplorationReadiness", e)
            # 兜底:旧 GetExplorationDiagnostics。
            try:
                resp = self._stub.GetExplorationDiagnostics(
                    pb2.Empty(), timeout=_read_deadline_sec()
                )
                sources = [
                    {
                        "source_name": s.source_name,
                        "is_active": bool(s.is_active),
                        "last_goal_id": s.last_goal_id,
                        "last_error": s.last_error,
                    }
                    for s in resp.sources
                ]
                return {
                    "ready": bool(resp.active),
                    "active": bool(resp.active),
                    "backend_available": None,
                    "controller_active": bool(resp.active),
                    "source_count": len(sources),
                    "frontier_source_ready": bool(resp.active) and len(sources) > 0,
                    "costmap_input_ready": False,
                    "costmap_input_revision": 0,
                    "active_source": resp.selected_source,
                    "arbitration": resp.arbitration,
                    "selected_source": resp.selected_source,
                    "last_stop_reason": resp.last_stop_reason,
                    "sources": sources,
                    "frontier_count": int(resp.frontier_count),
                    "revision": int(resp.revision),
                    "timestamp_ns": int(resp.timestamp_ns),
                    "message": "Phase 6 fallback: GetExplorationReadiness 不可用",
                    "error": "GetExplorationReadiness 失败: %s" % e,
                }
            except grpc.RpcError as e2:
                _log_rpc_error("GetExplorationDiagnostics", e2)
                return None

    # === 输入桥 ===
    # Rust proto PublishOdom 接受 PublishOdomRequest（仅 pose 字段；速度由 nuwax-nav-stack
    # 内部从 pose 增量计算，不再走 gRPC facade）。速度参数保留以兼容旧调用方签名，丢弃。
    def publish_odom(self, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float,
                     lvx: float = 0, lvy: float = 0, lvz: float = 0,
                     avx: float = 0, avy: float = 0, avz: float = 0,
                     frame_id: str = "odom", timestamp_ns: int = 0) -> bool:
        del lvx, lvy, lvz, avx, avy, avz  # round-4 HIGH 3：Rust 端不再接收 velocity 字段
        self._raise_if_stopping("publish_odom")
        if not self._stub:
            return False
        try:
            msg = pb2.PublishOdomRequest(
                pose=pb2.Pose(
                    position=pb2.Vec3(x=x, y=y, z=z),
                    orientation=pb2.Quat(x=qx, y=qy, z=qz, w=qw),
                    frame_id=frame_id,
                    timestamp_ns=max(0, int(timestamp_ns)),
                ),
            )
            resp = self._stub.PublishOdom(msg, timeout=_read_deadline_sec())
            return bool(getattr(resp, "success", False))
        except grpc.RpcError as e:
            _log_rpc_error("PublishOdom", e)
            return False

    def publish_point_cloud(
        self,
        payload: bytes,
        point_step: int,
        *,
        frame_id: str = "map",
        timestamp_ns: int = 0,
        fields: Optional[List[Dict[str, Any]]] = None,
        point_count: int = 0,
        is_dense: bool = True,
    ) -> bool:
        """Publish normalized PointCloud2 data through the Rust gRPC facade.

        ``payload`` and ``point_step`` remain positional for compatibility with the
        original bridge call shape. Metadata is keyword-only so older direct callers
        can continue to publish while the runtime applies its legacy defaults.
        """
        self._raise_if_stopping("publish_point_cloud")
        if not self._stub:
            return False

        proto_fields = []
        for point_field in fields or []:
            proto_fields.append(
                pb2.PointCloudField(
                    name=str(point_field.get("name", "")),
                    offset=max(0, int(point_field.get("offset", 0))),
                    datatype=max(0, int(point_field.get("datatype", 0))),
                    count=max(0, int(point_field.get("count", 0))),
                )
            )

        try:
            request = pb2.PublishPointCloudRequest(
                points=bytes(payload),
                point_step=max(0, int(point_step)),
                frame_id=str(frame_id or "map"),
                timestamp_ns=max(0, int(timestamp_ns)),
                fields=proto_fields,
                point_count=max(0, int(point_count)),
                is_dense=bool(is_dense),
            )
            response = self._stub.PublishPointCloud(
                request,
                timeout=_heavy_deadline_sec(),
            )
            return bool(getattr(response, "success", False))
        except grpc.RpcError as e:
            _log_rpc_error("PublishPointCloud", e)
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

    def notify_cmd_vel_sink_ready(self, ready: bool) -> bool:
        """把 cmd_vel sink ready 状态跨进程写回到 Rust handle（rework17）。

        Returns:
            True 当 RPC 调用成功；False 当 stub 未连 / RPC 失败。
        """
        self._raise_if_stopping("notify_cmd_vel_sink_ready")
        if not self._stub:
            return False
        try:
            req = pb2.CmdVelSinkReadyRequest(ready=bool(ready))
            resp = self._stub.SetCmdVelSinkReady(req, timeout=_read_deadline_sec())
            return bool(resp.success)
        except grpc.RpcError as e:
            _log_rpc_error("SetCmdVelSinkReady", e)
            return False

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
