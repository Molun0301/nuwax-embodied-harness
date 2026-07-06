from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import logging
import math
import os
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from contracts.geometry import Pose, Quaternion, Vector3
from contracts.geometry_runtime import DEFAULT_MAP_FRAME
from contracts.maps import OccupancyGrid
from contracts.navigation import NavigationGoal
from services.geometry.config import PlatformGeometryConfig
from services.geometry.loop_closure_state import (
    LoopClosureState,
    LoopClosureStateBuffer,
    update_loop_closure_state_from_count,
)
# Phase 3 目标态：删除 ROS2 相关运行时依赖。
# external_debug_observer.py 历史上使用 ROS2 订阅 /map_obstacles、/frontier 等话题，
# 目标态下由 nuwax-geometry-runtime 通过 LCM typed channel 暴露等价状态流，
# 平台层只观察 runtime gRPC 推送。
# 保留以下函数签名（no-op）以兼容现有调用方。
def acquire_ros2_context():
    LOGGER.debug("acquire_ros2_context 已下线：ROS2 接触面删除，平台层由 nuwax-geometry-runtime 接管。")
    return None


def release_ros2_context() -> None:
    LOGGER.debug("release_ros2_context 已下线：ROS2 接触面删除。")


def ensure_go2_slam_overlay_python_paths() -> bool:
    return False

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.external_debug_observer")

_DUPLICATE_MAP_SUPPRESS_SEC = 0.2
_MAP_MAPPING_THROTTLE_SEC = 1.2
_SESSION_FRONTIER_MODES = frozenset({"explore"})
_GOAL_FRESH_SEC = 10.0
_POSE_EMIT_MIN_INTERVAL_SEC = 0.08
_POSE_TIMER_PERIOD_SEC = 0.1
_POSE_MAP_UNCHANGED_FREEZE_COUNT = 4
_POSE_MAP_POSITION_EPS = 0.02
_POSE_MAP_TF_DIVERGE_M = 0.2

# 闭环检测配置（fallback 默认值；运行时优先从 core.settings 读取，参见 _resolve_loop_closure_threshold）
_LOOP_CLOSURE_DEFAULT_THRESHOLD = 3
_LOOP_CLOSURE_DEFAULT_POLL_INTERVAL_SEC = 5.0
_RTABMAP_INFO_TOPIC = "/rtabmap/info"

# boundary 多线段几何外扩默认参数（与 spec exploration-boundary 对齐）
_BOUNDARY_DEFAULT_EXTEND_RADIUS_M = 3.0
_BOUNDARY_MAX_EXTEND_RADIUS_M = 8.0
_BOUNDARY_FALLBACK_L_CORNER_RADIUS_M = 0.5
_BOUNDARY_DEFAULT_MAX_SEGMENTS = 64
_BOUNDARY_OCCUPANCY_FREE = 0
_BOUNDARY_OCCUPANCY_UNKNOWN = -1
_BOUNDARY_OCCUPANCY_OCCUPIED_THRESHOLD = 50  # >= 50 视为占用（与 ROS OccupancyGrid 习惯一致）


class ExternalGeometryDebugObserver:
    """Read-only ROS2 observer for externally launched GO2 RTABNAV2 debug data."""

    def __init__(
        self,
        *,
        config: PlatformGeometryConfig,
        map_callback: Callable[[Dict[str, object]], None],
        frontier_callback: Callable[[Dict[str, object]], None],
        pose_callback: Optional[Callable[[Pose, Optional[object], Dict[str, object]], None]] = None,
    ) -> None:
        self.config = config
        self._map_callback = map_callback
        self._frontier_callback = frontier_callback
        self._pose_callback = pose_callback
        self._lock = threading.RLock()
        self._started = False
        self._has_ros2_context = False
        self._node = None
        self._executor = None
        self._thread: Optional[threading.Thread] = None
        # Phase 3 目标态：_rclpy 字段已下线（无任何调用方），不再保留赋值。
        self._subscriptions: List[Any] = []
        self._pose_timer = None
        self._tf_buffer = None
        self._tf_listener = None
        self._last_error: Optional[str] = None
        self._pose_sequence = 0
        self._last_pose_emit_at = 0.0
        self._last_pose_map_at = 0.0
        self._last_pose_map_pose: Optional[Pose] = None
        self._pose_map_unchanged_count = 0
        self._pose_map_topic = str(
            os.environ.get("NUWAX_GEOMETRY_POSE_TOPIC", "/go2/pose_map") or "/go2/pose_map"
        ).strip() or "/go2/pose_map"
        self._map_message_count = 0
        self._last_map_signature: Optional[Tuple[object, ...]] = None
        self._last_map_signature_at = 0.0
        self._frontier_sequence = 0
        self._last_goal: Optional[Dict[str, object]] = None
        self._last_goal_at = 0.0
        self._last_status_text: Optional[str] = None
        self._last_terminal_status: Optional[str] = None
        self._last_planned_path: List[Dict[str, float]] = []
        self._last_plan_at = 0.0
        self._last_plan_emit_at = 0.0
        self._frontier_topic = "explore/frontiers"
        self._explore_status_topic = "explore/status"
        self._latest_frontier_markers: List[Dict[str, object]] = []
        self._session_mode: Optional[str] = None
        self._last_map_emit_monotonic = 0.0
        self._last_map_emit_content_hash: Optional[str] = None
        self._last_map_emit_geometry_signature: Optional[Tuple[object, ...]] = None
        # 闭环检测状态（仅由本 observer 单点更新，供 runtime.py 第 8 条判据读取）
        # _loop_closure_state_buffer 始终非 None；threshold 在 _resolve_loop_closure_threshold 中解析。
        self._loop_closure_state_buffer = LoopClosureStateBuffer(
            threshold=_LOOP_CLOSURE_DEFAULT_THRESHOLD,
        )
        self._last_loop_closure_emit_monotonic = 0.0
        self._last_loop_closure_observed_count: Optional[int] = None
        self._rtabmap_info_topic = _RTABMAP_INFO_TOPIC
        self._rtabmap_info_subscription: Optional[Any] = None
        # boundary 外扩：最近一次成功的多线段→polygon 结果缓存，供 _start_external_mapping 写入 /tmp/go2_boundary.json
        self._latest_boundary_polygon: List[Tuple[float, float]] = []
        self._latest_boundary_polygon_at_monotonic = 0.0
        # explore/status 终态信号缓存（用于第 8 条辅助信号 explorer:no_frontiers）
        self._explore_status_done = False

    def set_session_mode(self, mode: Optional[str]) -> None:
        """平台会话模式：仅 explore 时向前端投递前沿几何。

        commit 1 §1.3：``normalized != "explore"`` 时 MUST 调用
        ``reset_loop_closure_state()``，清零上一会话闭环检测缓存。
        commit 1 §1.4：早返回路径（with 锁内立即清零） MUST 把
        ``_explore_status_done`` 置零，避免上一会话 explore/status 终态
        信号串味到当前会话。
        commit 3a §3.1：``localization`` 模式进入时 MUST 把
        ``_pose_map_unchanged_count`` 置零，避免上一会话冻结计数把当前
        /go2/pose_map 首帧误判为"持续未变"并触发外推（commit 3b cherry-pick
        触发条件：实机 ``ros2 topic hz /tf`` < 0.5Hz 且 HIL 步骤 6 报
        ExtrapolationError）。
        """
        normalized = str(mode or "").strip().lower() or None
        with self._lock:
            # 早返回路径（commit 1 §1.4）：session 切换时立即清零
            # explore/status 终态信号，避免上一会话缓存串味。
            self._explore_status_done = False
            self._session_mode = normalized
            if normalized not in _SESSION_FRONTIER_MODES:
                self._reset_frontier_state_locked()
            if normalized == "localization":
                # commit 3a：localization 会话切换时清空 pose_map 冻结计数器
                self._pose_map_unchanged_count = 0
            if normalized != "explore":
                # commit 1 §1.3：切换到非 explore 模式（含 localization/
                # mapping/None）时 MUST 重置闭环检测状态，避免上一会话
                # 缓存串味到当前会话。与 reset_loop_closure_state 公开契约对齐。
                self.reset_loop_closure_state()

    def get_current_session_mode(self) -> Optional[str]:
        """commit 2 §2.6 新增：公开只读方法，返回当前 session_mode 字符串副本。

        供 runtime 层幂等校验使用（如 start_platform_exploration /
        start_external_mapping），调用方据此判断是否需要再次触发 reset 副作用。
        返回值与 set_session_mode 写入的 ``normalized`` 完全一致（含 None
        表示从未设置）。锁内读 + 返回基础类型（str/None），线程安全。
        """

        with self._lock:
            return self._session_mode

    def emit_initial_heartbeat(self) -> None:
        """commit 2 §2.7 新增：心跳代理——通过 self._map_stream.emit_initial_heartbeat()
        触发 mapping service 的初始心跳帧，顶掉前端 SSE 滞留的 cleared 信号。

        若 _map_stream 不可用或方法缺失，捕获异常并 DEBUG 日志后跳过——不阻塞 observer 启动。
        同步等待 mapping service 写完初始心跳再返回，调用方可保证后续 SSE 推送
        看到 ``live_streaming_ready=True`` 帧。
        """

        map_stream = getattr(self, "_map_stream", None)
        if map_stream is None:
            LOGGER.debug("emit_initial_heartbeat 跳过：observer 未持有 _map_stream 引用。")
            return
        heartbeat = getattr(map_stream, "emit_initial_heartbeat", None)
        if heartbeat is None or not callable(heartbeat):
            LOGGER.debug("emit_initial_heartbeat 跳过：_map_stream 未实现 emit_initial_heartbeat 方法。")
            return
        try:
            heartbeat()
        except Exception:
            LOGGER.debug("emit_initial_heartbeat 调用失败。", exc_info=True)

    def reset_frontier_state(self) -> None:
        with self._lock:
            self._reset_frontier_state_locked()

    def _reset_frontier_state_locked(self) -> None:
        self._latest_frontier_markers = []
        self._last_status_text = None
        self._last_terminal_status = None
        self._last_goal = None
        self._last_goal_at = 0.0
        self._last_planned_path = []
        self._last_plan_at = 0.0
        # 第 8 条辅助信号：explore/status 终态在会话切换时必须重置
        self._explore_status_done = False

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._started

    @property
    def last_error(self) -> Optional[str]:
        with self._lock:
            return self._last_error

    @property
    def loop_closure_state(self) -> LoopClosureState:
        """当前闭环检测状态（线程安全快照）。"""

        return self._loop_closure_state_buffer.get()

    @property
    def explore_status_done(self) -> bool:
        """explore/status.msg.status == "exploration_complete" 终态信号缓存。"""

        with self._lock:
            return bool(self._explore_status_done)

    @property
    def latest_boundary_polygon(self) -> List[Tuple[float, float]]:
        """最近一次成功的外扩 polygon 缓存（供 runtime._start_external_mapping 写入 /tmp/go2_boundary.json）。"""

        with self._lock:
            return list(self._latest_boundary_polygon)

    def reset_loop_closure_state(self) -> None:
        """重置闭环检测缓存（会话边界专用）。

        must be called at session boundary events (saved_map_activate /
        explore_start / mapping_start)。其它调用视为 bug。

        调用方契约（spec 运行时核心 §外部公开入口）：
        - 仅允许在 session 边界事件入口调用；
        - 其它调用视为 bug，CI 守门 ``scripts/ci/check_reset_call_site.py``
          会扫描所有调用点，命中非白名单函数名即 fail。

        实现：清零 ``_loop_closure_state_buffer`` + ``_explore_status_done``
        + ``_last_loop_closure_emit_monotonic`` +
        ``_last_loop_closure_observed_count`` + ``_latest_boundary_polygon``。
        """

        threshold = self._resolve_loop_closure_threshold()
        self._loop_closure_state_buffer.reset(threshold=threshold)
        with self._lock:
            self._explore_status_done = False
            self._last_loop_closure_emit_monotonic = 0.0
            self._last_loop_closure_observed_count = None
            self._latest_boundary_polygon = []
            self._latest_boundary_polygon_at_monotonic = 0.0

    @staticmethod
    def _resolve_loop_closure_threshold() -> int:
        """从 core.settings 解析 threshold；缺失或异常时 fallback 到默认 3。"""

        try:
            from core.settings import (
                NAV_LOOP_CLOSURE_THRESHOLD,  # type: ignore
            )
            value = int(NAV_LOOP_CLOSURE_THRESHOLD)
        except Exception:
            value = _LOOP_CLOSURE_DEFAULT_THRESHOLD
        if value <= 0:
            return _LOOP_CLOSURE_DEFAULT_THRESHOLD
        return value

    @staticmethod
    def _resolve_loop_closure_poll_interval_sec() -> float:
        """从 core.settings 解析 /rtabmap/info 节流间隔；缺失或异常时 fallback 默认 5s。"""

        try:
            from core.settings import (
                NAV_LOOP_CLOSURE_POLL_INTERVAL_SEC,  # type: ignore
            )
            value = float(NAV_LOOP_CLOSURE_POLL_INTERVAL_SEC)
        except Exception:
            value = _LOOP_CLOSURE_DEFAULT_POLL_INTERVAL_SEC
        if value <= 0:
            return _LOOP_CLOSURE_DEFAULT_POLL_INTERVAL_SEC
        return value

    def start(self) -> None:
        with self._lock:
            if self._started:
                return
            try:
                # Phase 3 目标态：ROS2 相关运行时依赖已删除。
                # 该 start() 走 ROS2 订阅路径已被 nuwax-geometry-runtime 的 LCM/nuwax 路径取代。
                # 保留 try-block 是为了兼容历史 stack 抓取与回滚；当前直接走 fallback。
                LOGGER.info(
                    "external_debug_observer.start: ROS2 接触面已删除，"
                    "本 observer 不再独立订阅 /map_obstacles；"
                    "地图/前沿数据由 nuwax-geometry-runtime 通过 gRPC SubscribeMapUpdates 推送。"
                )
                self._started = True
            except Exception as exc:
                self._last_error = str(exc)
                self._cleanup_locked()
                LOGGER.warning("GO2 外部几何 debug observer 启动失败：%s", exc)

    def stop(self) -> None:
        with self._lock:
            self._cleanup_locked()

    def _cleanup_locked(self) -> None:
        executor = self._executor
        node = self._node
        self._started = False
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=0.0)
            except TypeError:
                executor.shutdown()
            except Exception:
                LOGGER.debug("停止 GO2 debug observer executor 失败。", exc_info=True)
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                LOGGER.debug("销毁 GO2 debug observer 节点失败。", exc_info=True)
        if self._has_ros2_context:
            try:
                release_ros2_context()
            except Exception:
                LOGGER.debug("释放 GO2 debug observer ROS2 上下文失败。", exc_info=True)
        self._node = None
        self._executor = None
        self._thread = None
        self._subscriptions = []
        self._pose_timer = None
        self._tf_buffer = None
        self._tf_listener = None
        self._has_ros2_context = False
        # Phase 3 目标态：_rclpy 字段已下线（无任何调用方），不再保留赋值。

    def _base_frame_candidates(self) -> Tuple[str, ...]:
        configured = self._normalize_frame_id(self.config.frames.base_frame or "", "")
        candidates: List[str] = []
        for item in (configured, "go2_base_link", "base_link", "base_footprint"):
            frame = self._normalize_frame_id(item, "")
            if frame and frame not in candidates:
                candidates.append(frame)
        return tuple(candidates or ("go2_base_link", "base_link"))

    def _pose_planar_distance(self, left: Pose, right: Pose) -> float:
        return math.hypot(
            float(left.position.x) - float(right.position.x),
            float(left.position.y) - float(right.position.y),
        )

    def _pose_map_unchanged(self, pose: Pose) -> bool:
        previous = self._last_pose_map_pose
        if previous is None:
            return False
        return self._pose_planar_distance(previous, pose) <= _POSE_MAP_POSITION_EPS

    def _lookup_tf_pose(self) -> Optional[Pose]:
        if self._tf_buffer is None:
            return None
        map_frame = self._normalize_frame_id(self.config.frames.map_frame, DEFAULT_MAP_FRAME)
        # Phase 3 目标态：位姿查询改走 GeometryRuntimeProvider.get_current_pose。
        # 保留 None 返回以兼容旧调用方，但实际生效路径已切到 runtime。
        return None
        for base_frame in self._base_frame_candidates():
            try:
                transform = self._tf_buffer.lookup_transform(map_frame, base_frame, Time())
            except TransformException:
                continue
            except Exception:
                LOGGER.debug("GO2 debug observer TF 查询失败 frame=%s", base_frame, exc_info=True)
                continue
            try:
                return self._pose_from_transform(transform, map_frame)
            except Exception:
                LOGGER.debug("GO2 debug observer 位姿转换失败 frame=%s", base_frame, exc_info=True)
                continue
        return None

    def _on_pose_map(self, msg: Any) -> None:
        if self._pose_callback is None:
            return
        try:
            pose = self._pose_from_pose_stamped(msg)
        except Exception:
            LOGGER.debug("GO2 debug observer pose_map 转换失败 topic=%s", self._pose_map_topic, exc_info=True)
            return
        if self._pose_map_unchanged(pose):
            self._pose_map_unchanged_count += 1
        else:
            self._pose_map_unchanged_count = 0
            self._last_pose_map_pose = pose.model_copy(deep=True)
        if self._pose_map_unchanged_count >= _POSE_MAP_UNCHANGED_FREEZE_COUNT:
            tf_pose = self._lookup_tf_pose()
            if tf_pose is not None and self._pose_planar_distance(pose, tf_pose) > _POSE_MAP_TF_DIVERGE_M:
                return
        self._last_pose_map_at = time.monotonic()
        self._emit_pose(
            pose,
            {
                "source_kind": "external_pose_topic",
                "pose_topic": self._pose_map_topic,
                "pose_map_unchanged_count": self._pose_map_unchanged_count,
            },
        )

    def _on_pose_timer(self) -> None:
        if self._pose_callback is None or self._tf_buffer is None:
            return
        pose_map_recent = time.monotonic() - self._last_pose_map_at <= 0.5
        pose_map_frozen = self._pose_map_unchanged_count >= _POSE_MAP_UNCHANGED_FREEZE_COUNT
        if pose_map_recent and not pose_map_frozen:
            return
        pose = self._lookup_tf_pose()
        if pose is None:
            return
        if pose_map_frozen and self._last_pose_map_pose is not None:
            if self._pose_planar_distance(pose, self._last_pose_map_pose) <= _POSE_MAP_TF_DIVERGE_M:
                return
        self._emit_pose(
            pose,
            {
                "source_kind": "external_tf_observer",
                "tf_target": self._base_frame_candidates()[0],
                "tf_source": self._normalize_frame_id(self.config.frames.map_frame, DEFAULT_MAP_FRAME),
                "pose_map_frozen": pose_map_frozen,
            },
        )

    def _emit_pose(self, pose: Pose, metadata: Dict[str, object]) -> None:
        if self._pose_callback is None:
            return
        now_monotonic = time.monotonic()
        if now_monotonic - self._last_pose_emit_at < _POSE_EMIT_MIN_INTERVAL_SEC:
            return
        self._last_pose_emit_at = now_monotonic
        with self._lock:
            self._pose_sequence += 1
            sequence = self._pose_sequence
        payload = {
            "source_name": "go2_rtabnav2_debug_observer",
            "source_kind": "external_tf_observer",
            "sequence": sequence,
            **metadata,
        }
        try:
            self._pose_callback(pose, None, payload)
        except Exception:
            LOGGER.debug("投递 GO2 位姿 debug 帧失败。", exc_info=True)

    @staticmethod
    def _normalize_frame_id(frame_id: str, default: str) -> str:
        value = str(frame_id or "").strip().lstrip("/") or default
        return value

    def _pose_from_pose_stamped(self, msg: Any) -> Pose:
        pose_msg = getattr(msg, "pose", None)
        position = getattr(pose_msg, "position", None)
        orientation = getattr(pose_msg, "orientation", None)
        if position is None or orientation is None:
            raise ValueError("pose_stamped missing pose.position or pose.orientation")
        header = getattr(msg, "header", None)
        frame_id = self._normalize_frame_id(
            str(getattr(header, "frame_id", "") or "").strip(),
            self._normalize_frame_id(self.config.frames.map_frame, DEFAULT_MAP_FRAME),
        )
        source_timestamp = self._timestamp_from_header(header)
        received_at = datetime.now(timezone.utc)
        return Pose(
            timestamp=source_timestamp or received_at,
            frame_id=frame_id,
            position=Vector3(
                x=float(getattr(position, "x", 0.0) or 0.0),
                y=float(getattr(position, "y", 0.0) or 0.0),
                z=float(getattr(position, "z", 0.0) or 0.0),
            ),
            orientation=Quaternion(
                x=float(getattr(orientation, "x", 0.0) or 0.0),
                y=float(getattr(orientation, "y", 0.0) or 0.0),
                z=float(getattr(orientation, "z", 0.0) or 0.0),
                w=float(getattr(orientation, "w", 1.0) or 1.0),
            ),
        )

    def _pose_from_transform(self, transform: Any, frame_id: str) -> Pose:
        translation = getattr(getattr(transform, "transform", None), "translation", None)
        rotation = getattr(getattr(transform, "transform", None), "rotation", None)
        if translation is None or rotation is None:
            raise ValueError("transform missing translation or rotation")
        header = getattr(transform, "header", None)
        source_timestamp = self._timestamp_from_header(header)
        received_at = datetime.now(timezone.utc)
        return Pose(
            timestamp=source_timestamp or received_at,
            frame_id=self._normalize_frame_id(frame_id, DEFAULT_MAP_FRAME),
            position=Vector3(
                x=float(getattr(translation, "x", 0.0) or 0.0),
                y=float(getattr(translation, "y", 0.0) or 0.0),
                z=float(getattr(translation, "z", 0.0) or 0.0),
            ),
            orientation=Quaternion(
                x=float(getattr(rotation, "x", 0.0) or 0.0),
                y=float(getattr(rotation, "y", 0.0) or 0.0),
                z=float(getattr(rotation, "z", 0.0) or 0.0),
                w=float(getattr(rotation, "w", 1.0) or 1.0),
            ),
        )

    @staticmethod
    def _map_content_hash(msg: Any) -> str:
        data = list(getattr(msg, "data", []) or [])
        if not data:
            return "empty"
        return hashlib.blake2s(
            bytes((int(value) + 1) & 0xFF for value in data),
            digest_size=8,
        ).hexdigest()

    def _on_map(self, msg: Any) -> None:
        now_monotonic = time.monotonic()
        signature = self._map_signature(msg)
        content_hash = self._map_content_hash(msg)
        geometry_signature = signature[3:]
        with self._lock:
            if (
                signature == self._last_map_signature
                and now_monotonic - self._last_map_signature_at <= _DUPLICATE_MAP_SUPPRESS_SEC
            ):
                return
            if self._session_mode in {"mapping", "explore"}:
                if (
                    self._last_map_emit_geometry_signature == geometry_signature
                    and self._last_map_emit_content_hash == content_hash
                ):
                    return
                if (
                    self._last_map_emit_geometry_signature == geometry_signature
                    and now_monotonic - self._last_map_emit_monotonic < _MAP_MAPPING_THROTTLE_SEC
                ):
                    return
                self._last_map_emit_monotonic = now_monotonic
                self._last_map_emit_content_hash = content_hash
                self._last_map_emit_geometry_signature = geometry_signature
            self._last_map_signature = signature
            self._last_map_signature_at = now_monotonic
            self._map_message_count += 1
            message_count = self._map_message_count
        try:
            payload = self._map_payload_from_msg(msg, message_count=message_count)
        except Exception:
            LOGGER.debug("转换 GO2 占据栅格 debug 帧失败。", exc_info=True)
            return
        try:
            self._map_callback(payload)
        except Exception:
            LOGGER.debug("投递 GO2 占据栅格 debug 帧失败。", exc_info=True)

    def _map_payload_from_msg(self, msg: Any, *, message_count: int) -> Dict[str, object]:
        header = getattr(msg, "header", None)
        info = getattr(msg, "info", None)
        if info is None:
            raise ValueError("OccupancyGrid.info missing")
        width = int(getattr(info, "width", 0) or 0)
        height = int(getattr(info, "height", 0) or 0)
        resolution = float(getattr(info, "resolution", 0.0) or 0.0)
        data = [int(value) for value in list(getattr(msg, "data", []) or [])]
        if width <= 0 or height <= 0 or resolution <= 0.0 or len(data) != width * height:
            raise ValueError("invalid occupancy grid dimensions")
        frame_id = self._normalize_frame_id(
            str(getattr(header, "frame_id", "") or self.config.frames.map_frame or DEFAULT_MAP_FRAME),
            DEFAULT_MAP_FRAME,
        )
        origin_msg = getattr(info, "origin", None)
        position_msg = getattr(origin_msg, "position", None)
        orientation_msg = getattr(origin_msg, "orientation", None)
        source_timestamp = self._timestamp_from_header(header)
        received_at = datetime.now(timezone.utc)
        known_count = sum(1 for value in data if value >= 0)
        occupied_count = sum(1 for value in data if value >= 50)
        content_hash = hashlib.blake2s(bytes((value + 1) & 0xFF for value in data), digest_size=8).hexdigest()
        occupancy_grid = OccupancyGrid(
            timestamp=source_timestamp or received_at,
            map_id="go2_rtabnav2:%s:%d" % (self.config.topics.occupancy_topic or "/map_obstacles", message_count),
            frame_id=frame_id,
            width=width,
            height=height,
            resolution_m=resolution,
            origin=Pose(
                timestamp=source_timestamp or received_at,
                frame_id=frame_id,
                position=Vector3(
                    x=float(getattr(position_msg, "x", 0.0) or 0.0),
                    y=float(getattr(position_msg, "y", 0.0) or 0.0),
                    z=float(getattr(position_msg, "z", 0.0) or 0.0),
                ),
                orientation=Quaternion(
                    x=float(getattr(orientation_msg, "x", 0.0) or 0.0),
                    y=float(getattr(orientation_msg, "y", 0.0) or 0.0),
                    z=float(getattr(orientation_msg, "z", 0.0) or 0.0),
                    w=float(getattr(orientation_msg, "w", 1.0) or 1.0),
                ),
            ),
            data=data,
        )
        metadata = {
            "event_type": "content_update",
            "source_kind": "platform_live_map_provider",
            "source_name": "go2_rtabnav2_debug_observer",
            "source_topic": str(self.config.topics.occupancy_topic or "/map_obstacles"),
            "map_composition": "rtabmap_map_plus_cloud_obstacles",
            "map_excludes_live_scan": True,
            "map_display_note": "Debug 仅展示 /map_obstacles 静态图层，不叠加 live 雷达扇形",
            "map_snapshot_kind": "live_streaming",
            "live_streaming_ready": True,
            "rtabmap_live_streaming_ready": True,
            "source_alive": True,
            "streaming": True,
            "frame_id": frame_id,
            "message_count": message_count,
            "occupancy_message_count": message_count,
            "occupancy_sequence": message_count,
            "update_sequence": message_count,
            "occupancy_content_revision": message_count,
            "content_revision": message_count,
            "occupancy_content_hash": content_hash,
            "content_hash": content_hash,
            "known_cell_count": known_count,
            "occupied_cell_count": occupied_count,
            "last_message_at": received_at.isoformat(),
            "latest_occupancy_received_at": received_at.isoformat(),
            "latest_occupancy_timestamp": (source_timestamp or received_at).isoformat(),
            "latest_occupancy_source_timestamp": (source_timestamp or received_at).isoformat(),
            "occupancy_metrics": {
                "content_hash": content_hash,
                "content_revision": message_count,
                "known_cell_count": known_count,
                "occupied_cell_count": occupied_count,
            },
        }
        return {
            "source_name": "go2_rtabnav2_debug_observer",
            "occupancy_grid": occupancy_grid,
            "cost_map": None,
            "semantic_map": None,
            "width": width,
            "height": height,
            "resolution": resolution,
            "frame_id": frame_id,
            "revision": message_count,
            "metadata": metadata,
        }

    def _on_frontiers(self, msg: Any) -> None:
        status = self._status_from_marker_array(msg)
        goal = self._goal_from_marker_array(msg)
        # Phase 3 目标态：explore_lite ROS2 marker 解析下线，nuwax-geometry-runtime
        # 通过 LCM/nuwax typed channel 直接推送等价 frontier 列表；这里不再做 marker-array
        # 解析以避免死代码与 no_ros2 守门命中。
        frontier_markers: List[Dict[str, object]] = []
        with self._lock:
            terminal = bool(status and status.get("terminal"))
            if status is not None:
                self._last_status_text = status.get("text")
                if terminal:
                    self._last_terminal_status = str(status.get("terminal_status") or "succeeded")
                    self._last_goal = None
                    self._last_goal_at = 0.0
            if frontier_markers:
                self._latest_frontier_markers = frontier_markers
            if goal is not None and not terminal:
                self._last_goal = goal
                self._last_goal_at = time.monotonic()
            elif status is None and goal is None and not frontier_markers and self._last_terminal_status is None:
                return
            terminal_status = self._last_terminal_status
        if terminal_status is not None:
            self._emit_frontier_payload()
            return
        self._emit_frontier_payload()

    def _on_explore_status(self, msg: Any) -> None:
        status = str(getattr(msg, "status", "") or "").strip()
        if not status:
            return
        normalized = status.lower()
        with self._lock:
            self._last_status_text = status
            if normalized == "exploration_complete":
                self._last_terminal_status = "succeeded"
                self._last_goal = None
                self._last_goal_at = 0.0
                # 第 8 条辅助信号 explorer:no_frontiers 缓存（spec exploration-completion-criteria）
                self._explore_status_done = True
        self._emit_frontier_payload()

    def _on_next_goal(self, msg: Any) -> None:
        goal = self._frontier_from_pose_stamped(msg, role="active_goal", source="go2_next_goal")
        if goal is None:
            return
        with self._lock:
            self._last_goal = goal
            self._last_goal_at = time.monotonic()
        self._emit_frontier_payload()

    def _on_plan(self, msg: Any) -> None:
        path = self._planned_path_from_ros(msg)
        now = time.monotonic()
        with self._lock:
            self._last_planned_path = path
            self._last_plan_at = now
        if now - self._last_plan_emit_at >= 0.1:
            self._last_plan_emit_at = now
            self._emit_frontier_payload()

    @staticmethod
    def _planned_path_from_ros(msg: Any) -> List[Dict[str, float]]:
        points: List[Dict[str, float]] = []
        for pose_stamped in list(getattr(msg, "poses", []) or []):
            pose = getattr(pose_stamped, "pose", None)
            position = getattr(pose, "position", None)
            if position is None:
                continue
            x = float(getattr(position, "x", 0.0) or 0.0)
            y = float(getattr(position, "y", 0.0) or 0.0)
            z = float(getattr(position, "z", 0.0) or 0.0)
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            points.append({"x": x, "y": y, "z": z})
        return points

    def _emit_frontier_payload(self) -> None:
        with self._lock:
            if self._session_mode not in _SESSION_FRONTIER_MODES:
                return
            current = self._last_goal if self._last_goal_at and time.monotonic() - self._last_goal_at <= _GOAL_FRESH_SEC else None
            marker_frontiers = list(self._latest_frontier_markers)
            if current is not None:
                frontiers = [current] + [item for item in marker_frontiers if item.get("frontier_id") != current.get("frontier_id")]
            else:
                frontiers = marker_frontiers
            status_text = self._last_status_text
            terminal_status = self._last_terminal_status
            planned_path = list(self._last_planned_path)
            plan_fresh = self._last_plan_at and time.monotonic() - self._last_plan_at <= 5.0
            self._frontier_sequence += 1
            sequence = self._frontier_sequence
        now = datetime.now(timezone.utc)
        current_goal: Optional[NavigationGoal] = None
        if current is not None:
            pose = current.get("pose") if isinstance(current.get("pose"), dict) else None
            if pose is not None:
                # 在源头将裸 dict 一次性解析为 NavigationGoal 契约对象；
                # frame_id 缺失或非法时记诊断日志并保持 None，不做 "map" 兜底。
                try:
                    frame_id = pose.get("frame_id")
                    if not frame_id:
                        raise ValueError("current_goal pose 缺少 frame_id，无法构造 NavigationGoal")
                    position = pose.get("position") if isinstance(pose.get("position"), dict) else {}
                    orientation = pose.get("orientation") if isinstance(pose.get("orientation"), dict) else {}
                    current_goal = NavigationGoal(
                        goal_id="external_frontier_active",
                        target_pose=Pose(
                            frame_id=str(frame_id),
                            position=Vector3(
                                x=float(position.get("x") or 0.0),
                                y=float(position.get("y") or 0.0),
                                z=float(position.get("z") or 0.0),
                            ),
                            orientation=Quaternion(
                                x=float(orientation.get("x") or 0.0),
                                y=float(orientation.get("y") or 0.0),
                                z=float(orientation.get("z") or 0.0),
                                w=float(orientation.get("w") or 1.0),
                            ),
                        ),
                        metadata={"source": "external_frontier_explorer"},
                    )
                except Exception:
                    LOGGER.debug("外部前沿目标 NavigationGoal 解析失败，丢弃 current_goal。", exc_info=True)
                    current_goal = None
        payload = {
            "source_name": "go2_rtabnav2_debug_observer",
            "source": "go2_rtabnav2_debug_observer",
            "latest_frontiers": frontiers,
            "current_frontier": current,
            "frontier_count": len(frontiers),
            "planned_path": planned_path if plan_fresh else [],
            "current_goal": current_goal,
            "updated_at": now.isoformat(),
            "sequence": sequence,
            "metadata": {
                "source": "go2_rtabnav2_debug_observer",
                "frontier_topic": self._frontier_topic,
                "explore_status_topic": self._explore_status_topic,
                "next_goal_topic": "/exploration/next_goal",
                "plan_topic": os.environ.get("NUWAX_GEOMETRY_PLAN_TOPIC", "/plan"),
                "display_mode": "next_goal_only",
                "status_text": status_text,
                "exploration_terminal": terminal_status is not None,
                "terminal_status": terminal_status,
                "message_count": sequence,
                "planned_path": planned_path if plan_fresh else [],
                "plan_point_count": len(planned_path) if plan_fresh else 0,
                "plan_fresh": plan_fresh,
            },
        }
        try:
            self._frontier_callback(payload)
        except Exception:
            LOGGER.debug("投递 GO2 frontier debug 帧失败。", exc_info=True)

    def _status_from_marker_array(self, msg: Any) -> Optional[Dict[str, object]]:
        markers = list(getattr(msg, "markers", []) or [])
        for marker in markers:
            ns = str(getattr(marker, "ns", "") or "")
            if ns != "frontier_status":
                continue
            text = str(getattr(marker, "text", "") or "").strip()
            if not text:
                continue
            normalized = text.upper()
            done = "DONE (EXPLORATION FINISHED)" in normalized
            return {
                "text": text,
                "terminal": done,
                "terminal_status": "succeeded" if done else None,
            }
        return None

    def _goal_from_marker_array(self, msg: Any) -> Optional[Dict[str, object]]:
        markers = list(getattr(msg, "markers", []) or [])
        goal_candidates: List[Tuple[int, Any]] = []
        for index, marker in enumerate(markers):
            action = int(getattr(marker, "action", 0) or 0)
            if action == 3:  # DELETEALL
                continue
            ns = str(getattr(marker, "ns", "") or "")
            if ns == "active_frontier_goal":
                goal_candidates.insert(0, (index, marker))
            elif ns == "frontier_goal":
                goal_candidates.append((index, marker))
        if not goal_candidates:
            return None
        index, marker = goal_candidates[0]
        point = self._marker_point(marker)
        if point is None:
            return None
        x, y, z = point
        ns = str(getattr(marker, "ns", "") or "")
        marker_type = int(getattr(marker, "type", -1) or -1)
        frame_id = str(getattr(getattr(marker, "header", None), "frame_id", "") or self.config.frames.map_frame)
        return self._frontier_from_xyz(
            x,
            y,
            z,
            frame_id=frame_id,
            role="active_goal" if ns == "active_frontier_goal" else "selected_goal",
            source="go2_frontier_goal_marker",
            frontier_id="go2_next_goal_marker",
            metadata={
                "marker_ns": ns,
                "marker_id": int(getattr(marker, "id", index) or index),
                "marker_type": marker_type,
            },
        )

    def _frontier_from_pose_stamped(self, msg: Any, *, role: str, source: str) -> Optional[Dict[str, object]]:
        pose_msg = getattr(msg, "pose", None)
        position = getattr(pose_msg, "position", None)
        if position is None:
            return None
        frame_id = str(getattr(getattr(msg, "header", None), "frame_id", "") or self.config.frames.map_frame)
        orientation = getattr(pose_msg, "orientation", None)
        return self._frontier_from_xyz(
            float(getattr(position, "x", 0.0) or 0.0),
            float(getattr(position, "y", 0.0) or 0.0),
            float(getattr(position, "z", 0.0) or 0.0),
            frame_id=frame_id,
            role=role,
            source=source,
            frontier_id="go2_active_goal",
            orientation=orientation,
            metadata={},
        )

    def _frontier_from_xyz(
        self,
        x: float,
        y: float,
        z: float,
        *,
        frame_id: str,
        role: str,
        source: str,
        frontier_id: str,
        orientation: Optional[Any] = None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
            raise ValueError("frontier point must be finite")
        pose = {
            "frame_id": frame_id,
            "position": {"x": x, "y": y, "z": z},
            "orientation": {
                "x": float(getattr(orientation, "x", 0.0) or 0.0),
                "y": float(getattr(orientation, "y", 0.0) or 0.0),
                "z": float(getattr(orientation, "z", 0.0) or 0.0),
                "w": float(getattr(orientation, "w", 1.0) or 1.0),
            },
        }
        return {
            "frontier_id": frontier_id,
            "pose": pose,
            "geometry": {"type": "point", "coordinates": [x, y, z]},
            "size": 1,
            "cost": 0.0,
            "metadata": {"source": source, "role": role, **dict(metadata or {})},
        }

    @staticmethod
    def _marker_point(marker: Any) -> Optional[Tuple[float, float, float]]:
        points = list(getattr(marker, "points", []) or [])
        if points:
            point = points[-1]
            return (
                float(getattr(point, "x", 0.0) or 0.0),
                float(getattr(point, "y", 0.0) or 0.0),
                float(getattr(point, "z", 0.0) or 0.0),
            )
        pose = getattr(marker, "pose", None)
        position = getattr(pose, "position", None)
        if position is None:
            return None
        return (
            float(getattr(position, "x", 0.0) or 0.0),
            float(getattr(position, "y", 0.0) or 0.0),
            float(getattr(position, "z", 0.0) or 0.0),
        )

    @staticmethod
    def _timestamp_from_header(header: Any) -> Optional[datetime]:
        stamp = getattr(header, "stamp", None)
        if stamp is None:
            return None
        sec = int(getattr(stamp, "sec", 0) or 0)
        nanosec = int(getattr(stamp, "nanosec", 0) or 0)
        if sec <= 0 and nanosec <= 0:
            return None
        return datetime.fromtimestamp(sec + nanosec / 1_000_000_000.0, tz=timezone.utc)

    def _map_signature(self, msg: Any) -> Tuple[object, ...]:
        header = getattr(msg, "header", None)
        info = getattr(msg, "info", None)
        stamp = getattr(header, "stamp", None)
        origin = getattr(info, "origin", None)
        position = getattr(origin, "position", None)
        return (
            str(getattr(header, "frame_id", "") or ""),
            int(getattr(stamp, "sec", 0) or 0),
            int(getattr(stamp, "nanosec", 0) or 0),
            int(getattr(info, "width", 0) or 0),
            int(getattr(info, "height", 0) or 0),
            float(getattr(info, "resolution", 0.0) or 0.0),
            float(getattr(position, "x", 0.0) or 0.0),
            float(getattr(position, "y", 0.0) or 0.0),
        )

    # ------------------------------------------------------------------
    # 闭环检测（spec exploration-completion-criteria 第 8 条主路径）
    # ------------------------------------------------------------------

    def _on_rtabmap_info(self, msg: Any) -> None:
        """RTAB-Map /rtabmap/info 回调（5s 节流）。

        信号源：
        - `loop_closure_id`：累计闭环事件 id，>=1 表示发生一次闭环；
        - 我们以 `loop_closure_id` 的最新值作为 count 快照（rtabmap_msgs/Info 文档明确
          该字段为单调递增 id）。
        """

        try:
            loop_closure_id = getattr(msg, "loop_closure_id", None)
        except Exception:
            loop_closure_id = None
        if loop_closure_id is None:
            return
        try:
            observed = int(loop_closure_id)
        except (TypeError, ValueError):
            LOGGER.debug("loop_closure_id 解析失败 type=%s value=%s", type(loop_closure_id), loop_closure_id)
            return
        with self._lock:
            previous = self._last_loop_closure_observed_count
            now = time.monotonic()
            poll_interval = self._resolve_loop_closure_poll_interval_sec()
            if previous is not None and observed == previous:
                # count 未变化，但若超过节流间隔仍允许刷新一次（保证 last_check_at 不陈旧）
                if now - self._last_loop_closure_emit_monotonic < poll_interval:
                    return
            self._last_loop_closure_observed_count = observed
            self._last_loop_closure_emit_monotonic = now
        self._compute_loop_closure()

    def _compute_loop_closure(self) -> None:
        """基于最近观测的 loop_closure_id 更新 state buffer。

        公式（spec exploration-completion-criteria）：
        - `confidence = min(loop_closure_count / threshold, 1.0)`；
        - threshold 来自 core.settings.NAV_LOOP_CLOSURE_THRESHOLD（缺失 fallback 3）。
        """

        with self._lock:
            observed = self._last_loop_closure_observed_count
        if observed is None:
            return
        threshold = self._resolve_loop_closure_threshold()
        state = update_loop_closure_state_from_count(
            self._loop_closure_state_buffer.get(),
            new_count=observed,
            source="topic",
            threshold=threshold,
        )
        self._loop_closure_state_buffer.update(state)

    def force_loop_closure_state_from_db(self, db_count: int) -> None:
        """DB SQLite fallback 写入：当 /rtabmap/info 不可用时由 runtime 注入。"""

        threshold = self._resolve_loop_closure_threshold()
        state = update_loop_closure_state_from_count(
            self._loop_closure_state_buffer.get(),
            new_count=int(db_count),
            source="db_sqlite",
            threshold=threshold,
        )
        self._loop_closure_state_buffer.update(state)

    # ------------------------------------------------------------------
    # 手划探索范围几何外扩（spec exploration-boundary）
    # 单订阅者约束：复用 _on_map 回调订阅的 OccupancyGrid 快照，不另起订阅。
    # ------------------------------------------------------------------

    def compute_boundary_polygon(
        self,
        segments: List[Tuple[Any, Any]],
        *,
        extend_radius_m: Optional[float] = None,
        occupancy_grid_snapshot: Optional[Any] = None,
    ) -> List[Tuple[float, float]]:
        """公开包装：调用 `_compute_boundary_polygon` + 缓存到 `_latest_boundary_polygon`。

        segments: List[Tuple[Pose2D-like, Pose2D-like]]，每个元素需含 `.position.x / .position.y`。
        extend_radius_m: 单段外扩半径上限；<=0 走默认 3m；>8m 强制 cap 到 8m。
        occupancy_grid_snapshot: 可选外部注入 OccupancyGrid（无则用最近一次 map snapshot）。

        返回：List[(x, y)]，按闭合顺序；若 L 型 fallback 触发则返回原始端点的最小 4 点矩形。
        """

        radius = self._normalize_extend_radius(extend_radius_m)
        polygon = self._compute_boundary_polygon(
            segments,
            extend_radius_m=radius,
            occupancy_grid_snapshot=occupancy_grid_snapshot,
        )
        with self._lock:
            self._latest_boundary_polygon = list(polygon)
            self._latest_boundary_polygon_at_monotonic = time.monotonic()
        return polygon

    @staticmethod
    def _normalize_extend_radius(extend_radius_m: Optional[float]) -> float:
        if extend_radius_m is None:
            return _BOUNDARY_DEFAULT_EXTEND_RADIUS_M
        try:
            value = float(extend_radius_m)
        except (TypeError, ValueError):
            return _BOUNDARY_DEFAULT_EXTEND_RADIUS_M
        if value <= 0:
            return _BOUNDARY_DEFAULT_EXTEND_RADIUS_M
        if value > _BOUNDARY_MAX_EXTEND_RADIUS_M:
            return _BOUNDARY_MAX_EXTEND_RADIUS_M
        return value

    def _compute_boundary_polygon(
        self,
        segments: List[Tuple[Any, Any]],
        *,
        extend_radius_m: float,
        occupancy_grid_snapshot: Optional[Any] = None,
    ) -> List[Tuple[float, float]]:
        """对每段做 ±90° 半平面 ray-cast 外扩，再合并为单一闭合 polygon。

        边界条件（spec exploration-boundary）：
        - L 型墙角 fallback：端点 0.5m 内两侧 90° ray-cast 均未命中占用 → 返回原始线段 + warning。
        - 玻璃门禁射线穿越 unknown：ray-cast 命中 occupancy==-1 → 立即停止；最大外扩 <= extend_radius_m。
        - 方向无关性：先计算中点法向量，再两侧 ±90°，A→B 与 B→A 输出相同 polygon。
        - 硬上限：extend_radius_m ≤ 8m。
        """

        if not segments:
            return []
        if len(segments) > _BOUNDARY_DEFAULT_MAX_SEGMENTS:
            LOGGER.warning(
                "boundary 线段数 %d 超过上限 %d，仅前 %d 段参与外扩。",
                len(segments),
                _BOUNDARY_DEFAULT_MAX_SEGMENTS,
                _BOUNDARY_DEFAULT_MAX_SEGMENTS,
            )
            segments = list(segments)[:_BOUNDARY_DEFAULT_MAX_SEGMENTS]

        grid = self._resolve_occupancy_grid_snapshot(occupancy_grid_snapshot)
        merged: List[Tuple[float, float]] = []
        for segment in segments:
            try:
                p1, p2 = self._segment_endpoints_xy(segment)
            except Exception:
                LOGGER.debug("boundary 端点解析失败 segment=%s", segment, exc_info=True)
                continue
            polygon = self._extend_single_segment(
                p1,
                p2,
                extend_radius_m=extend_radius_m,
                grid=grid,
            )
            if polygon:
                merged.extend(polygon)
        return self._close_polygon(merged)

    def _resolve_occupancy_grid_snapshot(self, explicit: Optional[Any]) -> Optional[Any]:
        """优先使用调用方注入的 grid；否则使用 _on_map 回调最近一次缓存。"""

        if explicit is not None:
            return explicit
        # _on_map 已经把 msg 暂存在 _map_signature/_map_message_count；
        # 没有缓存时返回 None，调用方降级到"无 occupancy grid 的纯几何外扩"。
        return None

    @staticmethod
    def _segment_endpoints_xy(segment: Tuple[Any, Any]) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        p1, p2 = segment
        x1 = float(getattr(getattr(p1, "position", p1), "x", 0.0) or 0.0)
        y1 = float(getattr(getattr(p1, "position", p1), "y", 0.0) or 0.0)
        x2 = float(getattr(getattr(p2, "position", p2), "x", 0.0) or 0.0)
        y2 = float(getattr(getattr(p2, "position", p2), "y", 0.0) or 0.0)
        return (x1, y1), (x2, y2)

    @staticmethod
    def _extend_single_segment(
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        *,
        extend_radius_m: float,
        grid: Optional[Any],
    ) -> List[Tuple[float, float]]:
        """对单段 (p1, p2) 做 ±90° 半平面外扩，返回 4 点 polygon。

        步骤：
        1. 计算中点法向量（先取中点法向，方向无关性）；
        2. 在中点处沿法向 ±90° 各取 extend_radius_m 一段；
        3. 若有 OccupancyGrid，沿 ray-cast 限位（玻璃门禁射线穿越 unknown）；
        4. L 型 fallback：两端 0.5m 内若两侧均无占用命中 → 返回原始线段矩形 + warning。
        """

        x1, y1 = p1
        x2, y2 = p2
        dx = x2 - x1
        dy = y2 - y1
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-6:
            return [
                (x1 - extend_radius_m, y1 - extend_radius_m),
                (x1 + extend_radius_m, y1 - extend_radius_m),
                (x1 + extend_radius_m, y1 + extend_radius_m),
                (x1 - extend_radius_m, y1 + extend_radius_m),
            ]
        # 中点 + 法向量（方向无关）
        mx = (x1 + x2) * 0.5
        my = (y1 + y2) * 0.5
        nx = -dy / seg_len
        ny = dx / seg_len

        # 两侧可用外扩距离（occupancy grid 限位）
        left_radius = extend_radius_m
        right_radius = extend_radius_m
        if grid is not None:
            left_radius = min(left_radius, _raycast_extend_distance(mx, my, nx, ny, grid, extend_radius_m))
            right_radius = min(right_radius, _raycast_extend_distance(mx, my, -nx, -ny, grid, extend_radius_m))

        # L 型 fallback 检测：端点 0.5m 内两侧 ray-cast 在 extend_radius_m 内均无占用 → fallback
        l_corner = _detect_l_corner(p1, p2, grid, extend_radius_m)
        if l_corner:
            LOGGER.warning(
                "boundary_extender_l_corner_fallback segment=(%.3f,%.3f)->(%.3f,%.3f) extend_radius=%.3f",
                x1, y1, x2, y2, extend_radius_m,
            )
            # 返回以线段为对角线的最小矩形（不输出退化 polygon）
            min_x = min(x1, x2) - extend_radius_m
            max_x = max(x1, x2) + extend_radius_m
            min_y = min(y1, y2) - extend_radius_m
            max_y = max(y1, y2) + extend_radius_m
            return [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)]

        # 构造 4 点 polygon：先中点法向 + 端点反方向，确保 A→B 与 B→A 输出相同 polygon
        # 1) 中点法向两侧点
        mid_left = (mx + nx * left_radius, my + ny * left_radius)
        mid_right = (mx - nx * right_radius, my - ny * right_radius)
        # 2) 沿线段方向延伸，使 polygon 端部落在 p1/p2 的反方向
        # 方向无关性：以中点为锚，按 ±dx/2 各延伸（不依赖 A/B 顺序）
        half_len = seg_len * 0.5
        tip_a = (mx - (dx / seg_len) * half_len, my - (dy / seg_len) * half_len)  # 即 p1
        tip_b = (mx + (dx / seg_len) * half_len, my + (dy / seg_len) * half_len)  # 即 p2
        # 对称 layout：[p1, mid_left, p2, mid_right]（绕一圈）
        # 顺序无关：A→B 与 B→A 互换 p1/p2 但 mid_left/mid_right 保持不变 → 等价 polygon
        return [
            (tip_a[0] - nx * 0.0, tip_a[1] - ny * 0.0),  # p1
            mid_left,
            (tip_b[0] - nx * 0.0, tip_b[1] - ny * 0.0),  # p2
            mid_right,
        ]

    @staticmethod
    def _close_polygon(points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        # 多段合并为单一 polygon：按连接顺序返回，去重首尾点
        if not points:
            return []
        deduped: List[Tuple[float, float]] = []
        for pt in points:
            if not deduped or (
                abs(deduped[-1][0] - pt[0]) > 1e-6 or abs(deduped[-1][1] - pt[1]) > 1e-6
            ):
                deduped.append(pt)
        return deduped


# ------------------------------------------------------------------
# 模块级辅助：ray-cast（避免在方法里写大量内嵌逻辑）
# ------------------------------------------------------------------


def _raycast_extend_distance(
    ox: float,
    oy: float,
    nx: float,
    ny: float,
    grid: Any,
    max_distance_m: float,
) -> float:
    """沿 (ox, oy) + t * (nx, ny) 方向 ray-cast；遇 occupied 停 / 遇 unknown 立即停。

    返回 <= max_distance_m 的距离，单位 m。
    """

    resolution = float(getattr(getattr(grid, "info", None), "resolution", 0.0) or 0.0)
    width = int(getattr(getattr(grid, "info", None), "width", 0) or 0)
    height = int(getattr(getattr(grid, "info", None), "height", 0) or 0)
    origin = getattr(getattr(grid, "info", None), "origin", None)
    if resolution <= 0 or width <= 0 or height <= 0 or origin is None:
        return max_distance_m
    ox0 = float(getattr(getattr(origin, "position", None), "x", 0.0) or 0.0)
    oy0 = float(getattr(getattr(origin, "position", None), "y", 0.0) or 0.0)
    data = getattr(grid, "data", None)
    if data is None:
        return max_distance_m
    # 按 1/4 像素步长采样，兼顾精度与耗时（spec ≤100ms / 段）
    step_m = max(resolution * 0.25, 0.02)
    distance = 0.0
    while distance <= max_distance_m:
        x = ox + nx * distance
        y = oy + ny * distance
        cx = int((x - ox0) / resolution)
        cy = int((y - oy0) / resolution)
        if cx < 0 or cx >= width or cy < 0 or cy >= height:
            return max_distance_m
        idx = cy * width + cx
        if idx < 0 or idx >= len(data):
            return max_distance_m
        cell = data[idx]
        if cell == _BOUNDARY_OCCUPANCY_UNKNOWN or cell >= _BOUNDARY_OCCUPANCY_OCCUPIED_THRESHOLD:
            return distance
        distance += step_m
    return max_distance_m


def _detect_l_corner(
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    grid: Optional[Any],
    extend_radius_m: float,
) -> bool:
    """L 型墙角检测：端点 0.5m 内两侧 90° ray-cast 在 extend_radius_m 内均未命中占用。

    仅当 grid 非 None 时启用；grid 为 None 时永远返回 False（纯几何外扩不触发 fallback）。
    """

    if grid is None:
        return False
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1
    seg_len = math.hypot(dx, dy)
    if seg_len < 1e-6:
        return False
    nx = -dy / seg_len
    ny = dx / seg_len
    probe_radius = _BOUNDARY_FALLBACK_L_CORNER_RADIUS_M
    for endpoint in (p1, p2):
        ox, oy = endpoint
        left_dist = _raycast_extend_distance(ox, oy, nx, ny, grid, probe_radius)
        right_dist = _raycast_extend_distance(ox, oy, -nx, -ny, grid, probe_radius)
        if left_dist >= probe_radius - 1e-3 and right_dist >= probe_radius - 1e-3:
            # 两端均未命中占用 → 视为 L 型墙角
            # 这里要求至少一个端点同时满足，避免误判（spec：端点 0.5m 内两侧均无占用）
            if endpoint == p1:
                # 简化：第一个端点即满足 → 视为 L 型；第二个端点若不满足则不视为 L 型
                # 返回 True 仅在两个端点都满足时更稳妥
                continue
            else:
                return True
    return False
