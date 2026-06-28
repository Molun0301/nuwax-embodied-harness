from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import logging
import threading
import time
from typing import Any, Dict, Optional, Tuple

from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.geometry import Pose
from contracts.navigation import NavigationGoal, NavigationState, NavigationStatus
from services.geometry.ros2_context import acquire_ros2_context, release_ros2_context


_NAV2_LIFECYCLE_NODES = (
    "planner_server",
    "controller_server",
    "bt_navigator",
    "global_costmap/global_costmap",
    "local_costmap/local_costmap",
)
_COSTMAP_FRESH_SEC = 5.0
_LIFECYCLE_CACHE_SEC = 1.0
_LIFECYCLE_GET_STATE_TIMEOUT_SEC = 0.75

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.nav2_action_provider")


@dataclass
class _CostmapTopicState:
    topic: str
    message_count: int = 0
    last_received_at: Optional[datetime] = None
    frame_id: Optional[str] = None
    width: Optional[int] = None
    height: Optional[int] = None
    resolution_m: Optional[float] = None
    origin_x: Optional[float] = None
    origin_y: Optional[float] = None

    def update(self, message: Any) -> None:
        self.message_count += 1
        self.last_received_at = datetime.now(timezone.utc)
        self.frame_id = str(getattr(getattr(message, "header", None), "frame_id", "") or "") or None
        info = getattr(message, "info", None)
        if info is not None:
            self.width = int(getattr(info, "width", 0) or 0)
            self.height = int(getattr(info, "height", 0) or 0)
            self.resolution_m = float(getattr(info, "resolution", 0.0) or 0.0)
            origin = getattr(info, "origin", None)
            position = getattr(origin, "position", None)
            if position is not None:
                self.origin_x = float(getattr(position, "x", 0.0) or 0.0)
                self.origin_y = float(getattr(position, "y", 0.0) or 0.0)

    def snapshot(self) -> Dict[str, object]:
        now = datetime.now(timezone.utc)
        age_sec = None
        if self.last_received_at is not None:
            age_sec = max(0.0, (now - self.last_received_at).total_seconds())
        return {
            "topic": self.topic,
            "message_count": self.message_count,
            "last_received_at": self.last_received_at.isoformat() if self.last_received_at else None,
            "age_sec": age_sec,
            "frame_id": self.frame_id,
            "width": self.width,
            "height": self.height,
            "resolution_m": self.resolution_m,
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "first_frame_ready": self.message_count >= 1,
            "streaming": self.message_count >= 2 and age_sec is not None and age_sec <= _COSTMAP_FRESH_SEC,
        }


@dataclass
class _Nav2Readiness:
    ready: bool
    waiting_for: Optional[str]
    message: str
    lifecycle: Dict[str, object] = field(default_factory=dict)
    action_server_ready: bool = False
    costmaps: Dict[str, object] = field(default_factory=dict)
    tf_ready: bool = False
    errors: Dict[str, str] = field(default_factory=dict)


class Nav2ActionNavigationProvider:
    provider_name = "nav2_action_navigation"
    provider_version = "0.1.0"

    def __init__(self, runtime: Any) -> None:
        self.runtime = runtime
        self._lock = threading.RLock()
        self._started = False
        self._has_ros2_context = False
        self._last_error: Optional[str] = None
        self._node = None
        self._executor = None
        self._thread: Optional[threading.Thread] = None
        self._action_client = None
        self._goal_handle = None
        self._result_future = None
        self._send_future = None
        self._send_goal_sequence = 0
        self._goal_pose_publisher = None
        self._clicked_point_publisher = None
        self._initial_pose_publisher = None
        self._goal_sequence = 0
        self._rclpy = None
        self._get_state_type = None
        self._lifecycle_clients: Dict[str, Any] = {}
        self._last_lifecycle_check_at = 0.0
        self._last_lifecycle_status: Dict[str, object] = {}
        self._global_costmap = _CostmapTopicState(str(runtime.config.nav2.global_costmap_topic or "/global_costmap/costmap"))
        self._local_costmap = _CostmapTopicState(str(runtime.config.nav2.local_costmap_topic or "/local_costmap/costmap"))
        self._state = NavigationState(
            status=NavigationStatus.IDLE,
            message="Nav2 action 导航空闲。",
            metadata={"backend": "nav2", "action_name": runtime.config.nav2.action_name},
        )

    def is_available(self) -> bool:
        return self.is_navigation_available()

    def is_navigation_available(self) -> bool:
        return self._readiness().ready

    def set_goal(self, goal: NavigationGoal) -> bool:
        if goal.target_pose is not None:
            LOGGER.info(
                "Nav2 provider 接收导航目标 goal_id=%s target=(%.3f, %.3f, %.3f) frame=%s map_name=%s target_name=%s metadata=%s",
                goal.goal_id,
                float(goal.target_pose.position.x),
                float(goal.target_pose.position.y),
                float(goal.target_pose.position.z),
                goal.target_pose.frame_id,
                goal.map_name,
                goal.target_name,
                dict(goal.metadata),
            )
        else:
            LOGGER.info(
                "Nav2 provider 接收导航目标 goal_id=%s target_pose=None map_name=%s target_name=%s metadata=%s",
                goal.goal_id,
                goal.map_name,
                goal.target_name,
                dict(goal.metadata),
            )
        target_pose_metadata = goal.target_pose.model_dump(mode="json") if goal.target_pose is not None else None
        force_nav2_goal = bool((goal.metadata or {}).get("force_nav2_goal"))
        if force_nav2_goal:
            self._cancel_active_nav2_goals_before_send()
        readiness = self._readiness_force() if force_nav2_goal else self._readiness()
        if not readiness.ready:
            LOGGER.info(
                "Nav2 provider 拒绝目标：not_ready goal_id=%s waiting_for=%s message=%s force=%s target=%s",
                goal.goal_id,
                readiness.waiting_for,
                readiness.message,
                force_nav2_goal,
                target_pose_metadata,
            )
            self._set_blocked_state(goal.goal_id, readiness)
            return False
        if goal.target_pose is None:
            LOGGER.info("Nav2 provider 拒绝目标：target_pose_missing goal_id=%s", goal.goal_id)
            self._state = NavigationState(
                current_goal_id=goal.goal_id,
                status=NavigationStatus.FAILED,
                message="Nav2 /navigate_to_pose 需要显式 target_pose，命名目标必须先解析。",
                metadata={"backend": "nav2", "failure_reason": "target_pose_missing", "readiness": self._status_dict(readiness), "target_pose": target_pose_metadata},
            )
            return False
        goal_sequence = None
        try:
            goal_msg = self._build_nav2_goal(goal.target_pose)
            with self._lock:
                self._goal_sequence += 1
                goal_sequence = self._goal_sequence
                self._goal_handle = None
                self._result_future = None
                self._state = NavigationState(
                    current_goal_id=goal.goal_id,
                    status=NavigationStatus.PLANNING,
                    current_pose=self._read_current_pose(),
                    goal_reached=False,
                    message="Nav2 /navigate_to_pose 目标已发送，等待 action server 接受。",
                    metadata={
                        "backend": "nav2",
                        "action_name": self.runtime.config.nav2.action_name,
                        "readiness": self._status_dict(readiness),
                        "target_pose": goal.target_pose.model_dump(mode="json"),
                        "goal_sequence": goal_sequence,
                    },
                )
            LOGGER.info(
                "Nav2 action 发送 NavigateToPose goal_id=%s goal_sequence=%s target=(%.3f, %.3f, %.3f) frame=%s action=%s",
                goal.goal_id,
                goal_sequence,
                float(goal_msg.pose.pose.position.x),
                float(goal_msg.pose.pose.position.y),
                float(goal_msg.pose.pose.position.z),
                goal_msg.pose.header.frame_id,
                self.runtime.config.nav2.action_name,
            )
            send_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda feedback: self._on_feedback(feedback, goal.goal_id, goal_sequence),
            )
            with self._lock:
                self._send_future = send_future
                self._send_goal_sequence = goal_sequence
            send_future.add_done_callback(
                lambda completed: self._on_goal_response(completed, goal.goal_id, goal_sequence)
            )
            if force_nav2_goal:
                self._wait_action_dispatch(send_future, timeout_sec=1.5)
        except Exception as exc:
            LOGGER.info("Nav2 action 发送目标异常 goal_id=%s target=%s error=%s", goal.goal_id, target_pose_metadata, exc)
            metadata = {"backend": "nav2", "failure_reason": "action_send_failed", "error": str(exc), "target_pose": target_pose_metadata}
            if goal_sequence is not None:
                metadata["goal_sequence"] = goal_sequence
            with self._lock:
                self._state = NavigationState(
                    current_goal_id=goal.goal_id,
                    status=NavigationStatus.FAILED,
                    current_pose=self._read_current_pose(),
                    message="Nav2 action 发送目标失败：%s" % exc,
                    metadata=metadata,
                )
            return False
        return True

    def _cancel_inflight_goal_for_resend(self) -> None:
        self._cancel_active_nav2_goals_before_send()

    def _cancel_active_nav2_goals_before_send(self, *, wait_timeout_sec: float = 2.0) -> None:
        """取消在途 Nav2 action 目标并等待完成，避免新目标发出后仍短暂跟随旧目标。"""

        with self._lock:
            prev_handle = self._goal_handle
            self._goal_sequence += 1
            self._goal_handle = None
            self._result_future = None
            self._send_future = None
        cancel_future = None
        if prev_handle is not None:
            try:
                cancel_future = prev_handle.cancel_goal_async()
            except Exception:
                LOGGER.debug("取消在途 Nav2 目标失败。", exc_info=True)
        if cancel_future is not None:
            deadline = time.monotonic() + max(0.2, float(wait_timeout_sec))
            while time.monotonic() < deadline and not cancel_future.done():
                time.sleep(0.05)
            if cancel_future.done():
                try:
                    cancel_future.result()
                except Exception:
                    pass
        time.sleep(0.05)

    def _wait_action_dispatch(self, send_future: Any, *, timeout_sec: float = 1.5) -> None:
        """等待 action client 把 NavigateToPose 请求真正发到 server，缩短旧目标残留窗口。"""

        if send_future is None:
            return
        deadline = time.monotonic() + max(0.1, float(timeout_sec))
        while time.monotonic() < deadline and not send_future.done():
            time.sleep(0.05)

    def publish_initial_pose(self, target_pose: Pose) -> Dict[str, object]:
        """RViz 2D Pose Estimate 等价：发布 /initialpose 触发重定位。"""

        return self.publish_rviz_initial_pose(target_pose)

    def publish_rviz_initial_pose(self, target_pose: Pose) -> Dict[str, object]:
        """RViz 2D Pose Estimate 等价：发布 geometry_msgs/PoseWithCovarianceStamped 到 /initialpose。"""

        if not self._ensure_started() or self._node is None:
            return {
                "published": False,
                "topic": "/initialpose",
                "message": "Nav2 action ROS 节点未启动，无法发布 /initialpose。",
            }
        try:
            from geometry_msgs.msg import PoseWithCovarianceStamped

            if self._initial_pose_publisher is None:
                self._initial_pose_publisher = self._node.create_publisher(
                    PoseWithCovarianceStamped,
                    "/initialpose",
                    10,
                )
            frame_id = str(target_pose.frame_id or "map").lstrip("/") or "map"
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = frame_id
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.pose.pose.position.x = float(target_pose.position.x)
            msg.pose.pose.position.y = float(target_pose.position.y)
            msg.pose.pose.position.z = float(target_pose.position.z)
            msg.pose.pose.orientation.x = float(target_pose.orientation.x)
            msg.pose.pose.orientation.y = float(target_pose.orientation.y)
            msg.pose.pose.orientation.z = float(target_pose.orientation.z)
            msg.pose.pose.orientation.w = float(target_pose.orientation.w)
            msg.pose.covariance = [0.25 if index in (0, 7, 35) else 0.0 for index in range(36)]
            self._initial_pose_publisher.publish(msg)
            LOGGER.info(
                "debug 发布 /initialpose（RViz 2D Pose Estimate 等价）pose=(%.3f, %.3f, %.3f) frame=%s",
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                frame_id,
            )
            return {
                "published": True,
                "topic": "/initialpose",
                "frame_id": frame_id,
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z,
                "dispatch_mode": "rviz_initial_pose_topic",
                "message": "已发布 /initialpose（RViz 2D Pose Estimate 直通）。",
            }
        except Exception as exc:
            LOGGER.info("debug 发布 /initialpose 失败：%s", exc)
            return {
                "published": False,
                "topic": "/initialpose",
                "message": "发布 /initialpose 失败：%s" % exc,
            }

    def publish_clicked_point(self, target_pose: Pose) -> Dict[str, object]:
        """RViz Publish Point 等价：发布 geometry_msgs/PointStamped 到 /clicked_point。"""

        if not self._ensure_started() or self._node is None:
            return {
                "published": False,
                "topic": "/clicked_point",
                "message": "Nav2 action ROS 节点未启动，无法发布 /clicked_point。",
            }
        try:
            from geometry_msgs.msg import PointStamped

            if self._clicked_point_publisher is None:
                self._clicked_point_publisher = self._node.create_publisher(PointStamped, "/clicked_point", 10)
            frame_id = str(target_pose.frame_id or "map").lstrip("/") or "map"
            msg = PointStamped()
            msg.header.frame_id = frame_id
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.point.x = float(target_pose.position.x)
            msg.point.y = float(target_pose.position.y)
            msg.point.z = float(target_pose.position.z)
            self._clicked_point_publisher.publish(msg)
            LOGGER.info(
                "debug 发布 /clicked_point point=(%.3f, %.3f, %.3f) frame=%s",
                msg.point.x,
                msg.point.y,
                msg.point.z,
                frame_id,
            )
            return {
                "published": True,
                "topic": "/clicked_point",
                "frame_id": frame_id,
                "x": msg.point.x,
                "y": msg.point.y,
                "z": msg.point.z,
                "message": "已发布 /clicked_point（RViz Publish Point 等价）。",
            }
        except Exception as exc:
            LOGGER.info("debug 发布 /clicked_point 失败：%s", exc)
            return {
                "published": False,
                "topic": "/clicked_point",
                "message": "发布 /clicked_point 失败：%s" % exc,
            }

    def publish_rviz_goal_pose(self, target_pose: Pose) -> Dict[str, object]:
        """RViz SetGoal 等价：发布 geometry_msgs/PoseStamped 到 /goal_pose。"""

        if not self._ensure_started() or self._node is None:
            return {
                "published": False,
                "topic": "/goal_pose",
                "message": "Nav2 action ROS 节点未启动，无法发布 /goal_pose。",
            }
        try:
            from geometry_msgs.msg import PoseStamped

            if self._goal_pose_publisher is None:
                self._goal_pose_publisher = self._node.create_publisher(PoseStamped, "/goal_pose", 10)
            frame_id = str(target_pose.frame_id or "map").lstrip("/") or "map"
            msg = PoseStamped()
            msg.header.frame_id = frame_id
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.pose.position.x = float(target_pose.position.x)
            msg.pose.position.y = float(target_pose.position.y)
            msg.pose.position.z = float(target_pose.position.z)
            msg.pose.orientation.x = float(target_pose.orientation.x)
            msg.pose.orientation.y = float(target_pose.orientation.y)
            msg.pose.orientation.z = float(target_pose.orientation.z)
            msg.pose.orientation.w = float(target_pose.orientation.w)
            self._goal_pose_publisher.publish(msg)
            LOGGER.info(
                "debug 发布 /goal_pose（RViz SetGoal 等价）target=(%.3f, %.3f, %.3f) frame=%s",
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                frame_id,
            )
            return {
                "published": True,
                "topic": "/goal_pose",
                "frame_id": frame_id,
                "x": msg.pose.position.x,
                "y": msg.pose.position.y,
                "z": msg.pose.position.z,
                "dispatch_mode": "rviz_goal_pose_topic",
                "message": "已发布 /goal_pose（RViz 2D Goal 直通）。",
            }
        except Exception as exc:
            LOGGER.info("debug 发布 /goal_pose 失败：%s", exc)
            return {
                "published": False,
                "topic": "/goal_pose",
                "message": "发布 /goal_pose 失败：%s" % exc,
            }

    def cancel_goal(self) -> bool:
        with self._lock:
            current_goal_id = self._state.current_goal_id
            goal_handle = self._goal_handle
            self._goal_sequence += 1
            self._goal_handle = None
            self._result_future = None
        try:
            if goal_handle is not None:
                goal_handle.cancel_goal_async()
        except Exception:
            pass
        with self._lock:
            self._state = NavigationState(
                current_goal_id=current_goal_id,
                status=NavigationStatus.CANCELLED,
                current_pose=self._read_current_pose(),
                goal_reached=False,
                message="Nav2 导航目标已取消。",
                metadata={"backend": "nav2", "action_name": self.runtime.config.nav2.action_name},
            )
        return True

    def get_navigation_state(self) -> NavigationState:
        readiness = self._readiness()
        with self._lock:
            state = self._state.model_copy(deep=True)
        metadata = {
            **dict(state.metadata),
            **self._navigation_backend_policy_metadata(),
            "provider_name": self.provider_name,
            "nav2_readiness": self._status_dict(readiness),
        }
        if state.status == NavigationStatus.IDLE and not readiness.ready:
            return NavigationState(
                status=NavigationStatus.IDLE,
                message=readiness.message,
                metadata={
                    "backend": "nav2",
                    "waiting_for": readiness.waiting_for,
                    **self._navigation_backend_policy_metadata(),
                    "provider_name": self.provider_name,
                    "nav2_readiness": self._status_dict(readiness),
                },
            )
        return state.model_copy(update={"metadata": metadata}, deep=True)

    def is_goal_reached(self) -> bool:
        with self._lock:
            return bool(self._state.goal_reached)

    def get_nav2_status(self) -> Dict[str, object]:
        readiness = self._readiness()
        return {
            "phase": "ready" if readiness.ready else "blocked",
            "message": readiness.message,
            "action_name": self.runtime.config.nav2.action_name,
            "launch_package": self.runtime.config.nav2.launch_package,
            "launch_file": self.runtime.config.nav2.launch_file,
            "global_costmap_topic": self.runtime.config.nav2.global_costmap_topic,
            "local_costmap_topic": self.runtime.config.nav2.local_costmap_topic,
            "costmap_source": self.runtime.config.nav2.costmap_source,
            "action_client_connected": readiness.action_server_ready,
            "lifecycle_ready": self._lifecycle_ready(readiness.lifecycle),
            "ready": readiness.ready,
            "waiting_for": readiness.waiting_for,
            "lifecycle": readiness.lifecycle,
            "costmaps": readiness.costmaps,
            "tf_ready": readiness.tf_ready,
            "errors": readiness.errors,
        }

    def _navigation_backend_policy_metadata(self) -> Dict[str, object]:
        policy_builder = getattr(self.runtime, "_navigation_backend_policy", None)
        if not callable(policy_builder):
            return {}
        policy = policy_builder()
        return {
            "navigation_backend_policy": self.runtime._clone_metadata(policy),
            "active_navigation_backend": policy.get("active_backend_provider"),
            "nav2_action_active": bool(policy.get("nav2_action_active")),
        }

    def _readiness(self) -> _Nav2Readiness:
        if not self.runtime._is_geometry_profile_enabled():
            return _Nav2Readiness(False, "geometry_profile", "平台几何运行时或 RTAB-Map profile 已禁用。")
        if not self._ensure_started():
            return _Nav2Readiness(False, "ros2_action", "Nav2 action client 未启动：%s" % (self._last_error or "ROS2 环境不可用"), errors={"action": self._last_error or "ros2_unavailable"})
        lifecycle = self._read_lifecycle_status()
        action_ready = self._is_action_server_ready()
        costmaps = self._costmap_status()
        current_pose = self._read_current_pose()
        costmaps_ready = self._costmaps_ready(costmaps, current_pose=current_pose)
        tf_ready, tf_error = self._tf_ready(current_pose=current_pose)
        lifecycle_ready = self._lifecycle_ready(lifecycle)
        lifecycle_observation_uncertain = self._lifecycle_observation_uncertain(lifecycle)
        errors: Dict[str, str] = {}
        if not lifecycle_ready and not (lifecycle_observation_uncertain and action_ready and costmaps_ready and tf_ready):
            errors["lifecycle"] = "Nav2 lifecycle 未全部 active。"
        if not action_ready:
            errors["action"] = "Nav2 /navigate_to_pose action server 不可用。"
        if not costmaps_ready:
            global_status = costmaps.get("global") if isinstance(costmaps.get("global"), dict) else {}
            if global_status.get("bounds_error") == "current_pose_out_of_bounds":
                errors["costmap"] = "当前位姿不在 Nav2 global costmap 范围内。"
            elif global_status.get("bounds_error") == "current_pose_unavailable":
                errors["costmap"] = "当前位姿不可用，无法验证 Nav2 global costmap 范围。"
            else:
                errors["costmap"] = "Nav2 global/local costmap 尚未首帧并连续更新。"
        if not tf_ready:
            errors["tf"] = tf_error or "TF/localization 不可用。"
        if errors:
            waiting_for = next(iter(errors.keys()))
            return _Nav2Readiness(False, waiting_for, "Nav2 未 ready：" + "；".join(errors.values()), lifecycle, action_ready, costmaps, tf_ready, errors)
        return _Nav2Readiness(True, None, "Nav2 lifecycle、action、costmap 与 TF 均已 ready。", lifecycle, action_ready, costmaps, tf_ready, {})

    def _readiness_force(self) -> _Nav2Readiness:
        """RViz 风格：只要 action server 与实时位姿可用就允许下发目标。"""

        if not self.runtime._is_geometry_profile_enabled():
            return _Nav2Readiness(False, "geometry_profile", "平台几何运行时或 RTAB-Map profile 已禁用。")
        if not self._ensure_started():
            return _Nav2Readiness(
                False,
                "ros2_action",
                "Nav2 action client 未启动：%s" % (self._last_error or "ROS2 环境不可用"),
                errors={"action": self._last_error or "ros2_unavailable"},
            )
        if not self._wait_action_server(timeout_sec=2.0):
            return _Nav2Readiness(
                False,
                "action",
                "Nav2 /navigate_to_pose action server 不可用。",
                action_server_ready=False,
                errors={"action": "action_server_unavailable"},
            )
        current_pose = self._read_current_pose()
        if current_pose is None:
            return _Nav2Readiness(
                False,
                "tf",
                "无实时位姿，无法下发 Nav2 目标。",
                action_server_ready=True,
                errors={"tf": "current_pose_unavailable"},
            )
        return _Nav2Readiness(
            True,
            None,
            "Nav2 action server 可用（force 模式，已跳过 lifecycle/costmap 平台检查）。",
            action_server_ready=True,
            tf_ready=True,
        )

    def _ensure_started(self) -> bool:
        with self._lock:
            if self._started:
                return True
            try:
                from nav2_msgs.action import NavigateToPose
                from nav_msgs.msg import OccupancyGrid
                from lifecycle_msgs.srv import GetState
                from rclpy.action import ActionClient
                from rclpy.executors import MultiThreadedExecutor

                rclpy = acquire_ros2_context()
                self._has_ros2_context = True
                self._rclpy = rclpy
                self._get_state_type = GetState
                self._node = rclpy.create_node("nuwax_nav2_action_provider")
                self._executor = MultiThreadedExecutor(num_threads=2)
                self._executor.add_node(self._node)
                self._action_client = ActionClient(self._node, NavigateToPose, str(self.runtime.config.nav2.action_name or "/navigate_to_pose"))
                self._node.create_subscription(OccupancyGrid, self._global_costmap.topic, self._on_global_costmap, 10)
                self._node.create_subscription(OccupancyGrid, self._local_costmap.topic, self._on_local_costmap, 10)
                for node_name in _NAV2_LIFECYCLE_NODES:
                    self._lifecycle_clients[node_name] = self._node.create_client(GetState, "/%s/get_state" % node_name)
                self._thread = threading.Thread(target=self._executor.spin, name="nuwax_nav2_action_provider", daemon=True)
                self._thread.start()
                self._started = True
                self._last_error = None
                return True
            except Exception as exc:
                self._last_error = str(exc)
                self._cleanup_failed_start_locked()
                return False

    def _cleanup_failed_start_locked(self) -> None:
        executor = self._executor
        node = self._node
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=0.0)
            except TypeError:
                executor.shutdown()
            except Exception:
                pass
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if self._has_ros2_context:
            try:
                release_ros2_context()
            except Exception:
                pass
        self._started = False
        self._has_ros2_context = False
        self._node = None
        self._executor = None
        self._thread = None
        self._action_client = None
        self._lifecycle_clients = {}

    def _read_lifecycle_status(self) -> Dict[str, object]:
        now = time.monotonic()
        if now - self._last_lifecycle_check_at < _LIFECYCLE_CACHE_SEC:
            return dict(self._last_lifecycle_status)
        status: Dict[str, object] = {}
        request_type = self._get_state_type
        if request_type is None:
            return status
        for node_name, client in self._lifecycle_clients.items():
            item: Dict[str, object] = {"service": "/%s/get_state" % node_name, "available": False, "state": None, "active": False}
            try:
                if not client.wait_for_service(timeout_sec=0.0):
                    status[node_name] = item
                    continue
                item["available"] = True
                future = client.call_async(request_type.Request())
                deadline = time.monotonic() + _LIFECYCLE_GET_STATE_TIMEOUT_SEC
                while not future.done() and time.monotonic() < deadline:
                    time.sleep(0.01)
                if future.done() and future.result() is not None:
                    current_state = future.result().current_state
                    label = str(getattr(current_state, "label", "") or "")
                    state_id = int(getattr(current_state, "id", 0) or 0)
                    item.update({"state": label or state_id, "active": label.lower() == "active" or state_id == 3})
            except Exception as exc:
                item["error"] = str(exc)
            status[node_name] = item
        self._last_lifecycle_check_at = now
        self._last_lifecycle_status = status
        return dict(status)

    def _is_action_server_ready(self) -> bool:
        return self._wait_action_server(timeout_sec=0.0)

    def _wait_action_server(self, *, timeout_sec: float) -> bool:
        if not self._ensure_started():
            return False
        try:
            return bool(self._action_client is not None and self._action_client.wait_for_server(timeout_sec=timeout_sec))
        except Exception:
            return False

    def _costmap_status(self) -> Dict[str, object]:
        return {"global": self._global_costmap.snapshot(), "local": self._local_costmap.snapshot()}

    def _tf_ready(self, *, current_pose: Optional[Pose] = None) -> Tuple[bool, Optional[str]]:
        pose = current_pose or self._read_current_pose()
        if pose is None:
            return False, "TF/localization 缺失：当前位姿不可用。"
        backend = self.runtime._resolve_localization_backend()
        frame_tree_getter = getattr(backend, "get_frame_tree", None)
        if callable(frame_tree_getter):
            try:
                frame_tree = frame_tree_getter()
            except Exception as exc:
                return False, "TF 检查失败：%s" % exc
            if frame_tree is None:
                return False, "TF 缺失：frame tree 不可用。"
        # 额外确认 map→base_footprint TF 链健康：get_current_pose() 必须返回非 None
        live_pose = self._read_current_pose()
        if live_pose is None:
            return False, "TF 定位不可用：get_current_pose() 返回 None，map→base_footprint TF 链可能断裂。"
        return True, None

    def _read_current_pose(self) -> Optional[Pose]:
        live_pose_getter = getattr(self.runtime, "get_external_live_pose", None)
        if callable(live_pose_getter):
            try:
                cached_pose = live_pose_getter()
            except Exception:
                cached_pose = None
            if cached_pose is not None:
                return cached_pose.model_copy(deep=True)
        backend = self.runtime._resolve_localization_backend()
        if backend is None:
            return None
        try:
            pose = backend.get_current_pose()
        except Exception:
            return None
        return None if pose is None else pose.model_copy(deep=True)

    def _build_nav2_goal(self, target_pose: Pose) -> Any:
        from builtin_interfaces.msg import Time
        from nav2_msgs.action import NavigateToPose

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = str(target_pose.frame_id or self.runtime.config.frames.map_frame)
        # stamp=0：Nav2 使用最新 TF，避免时间戳超前导致目标被错误变换到 map 原点。
        goal_msg.pose.header.stamp = Time(sec=0, nanosec=0)
        goal_msg.pose.pose.position.x = float(target_pose.position.x)
        goal_msg.pose.pose.position.y = float(target_pose.position.y)
        goal_msg.pose.pose.position.z = float(target_pose.position.z)
        goal_msg.pose.pose.orientation.x = float(target_pose.orientation.x)
        goal_msg.pose.pose.orientation.y = float(target_pose.orientation.y)
        goal_msg.pose.pose.orientation.z = float(target_pose.orientation.z)
        goal_msg.pose.pose.orientation.w = float(target_pose.orientation.w)
        LOGGER.debug(
            "Nav2 NavigateToPose goal 构造 target=(%.3f, %.3f, %.3f) frame=%s orientation=(%.4f, %.4f, %.4f, %.4f)",
            float(goal_msg.pose.pose.position.x),
            float(goal_msg.pose.pose.position.y),
            float(goal_msg.pose.pose.position.z),
            goal_msg.pose.header.frame_id,
            float(goal_msg.pose.pose.orientation.x),
            float(goal_msg.pose.pose.orientation.y),
            float(goal_msg.pose.pose.orientation.z),
            float(goal_msg.pose.pose.orientation.w),
        )
        return goal_msg

    def _on_goal_response(self, future: Any, goal_id: str, goal_sequence: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            with self._lock:
                if goal_sequence != self._goal_sequence or self._state.current_goal_id != goal_id:
                    return
                self._state = NavigationState(current_goal_id=goal_id, status=NavigationStatus.FAILED, current_pose=self._read_current_pose(), message="Nav2 action 接受目标失败：%s" % exc, metadata={"backend": "nav2", "failure_reason": "goal_response_error", "error": str(exc), "goal_sequence": goal_sequence})
            return
        with self._lock:
            stale_response = goal_sequence != self._goal_sequence or self._state.current_goal_id != goal_id
        if stale_response:
            if getattr(goal_handle, "accepted", False):
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
            return
        with self._lock:
            state_metadata = dict(self._state.metadata)
            target_pose = state_metadata.get("target_pose") if isinstance(state_metadata.get("target_pose"), dict) else {}
            if not getattr(goal_handle, "accepted", False):
                LOGGER.info(
                    "Nav2 action server 拒绝目标 goal_id=%s goal_sequence=%s target=%s",
                    goal_id,
                    goal_sequence,
                    target_pose,
                )
                self._state = NavigationState(current_goal_id=goal_id, status=NavigationStatus.FAILED, current_pose=self._read_current_pose(), message="Nav2 action server 拒绝导航目标。", metadata={"backend": "nav2", "failure_reason": "goal_rejected", "goal_sequence": goal_sequence, "target_pose": target_pose})
                return
            LOGGER.info(
                "Nav2 action server 接受目标 goal_id=%s goal_sequence=%s target=%s",
                goal_id,
                goal_sequence,
                target_pose,
            )
            self._goal_handle = goal_handle
            self._state = self._state.model_copy(update={"status": NavigationStatus.RUNNING, "message": "Nav2 action 已接受目标并开始执行。", "metadata": {**state_metadata, "goal_sequence": goal_sequence}}, deep=True)
            self._result_future = goal_handle.get_result_async()
            self._result_future.add_done_callback(lambda completed: self._on_result(completed, goal_id, goal_sequence))

    def _on_feedback(self, feedback_msg: Any, goal_id: str, goal_sequence: int) -> None:
        feedback = getattr(feedback_msg, "feedback", None)
        if feedback is None:
            return
        distance_remaining = getattr(feedback, "distance_remaining", None)
        with self._lock:
            if goal_sequence != self._goal_sequence or self._state.current_goal_id != goal_id:
                return
            self._state = self._state.model_copy(
                update={
                    "status": NavigationStatus.RUNNING,
                    "current_pose": self._read_current_pose(),
                    "remaining_distance_m": None if distance_remaining is None else max(0.0, float(distance_remaining)),
                    "message": "Nav2 正在执行导航。",
                    "metadata": {**dict(self._state.metadata), "feedback": self._feedback_metadata(feedback)},
                },
                deep=True,
            )

    def _on_result(self, future: Any, goal_id: str, goal_sequence: int) -> None:
        try:
            result = future.result()
            status_code = int(getattr(result, "status", 0) or 0)
        except Exception as exc:
            with self._lock:
                if goal_sequence != self._goal_sequence or self._state.current_goal_id != goal_id:
                    return
                self._state = NavigationState(current_goal_id=goal_id, status=NavigationStatus.FAILED, current_pose=self._read_current_pose(), message="Nav2 action result 读取失败：%s" % exc, metadata={"backend": "nav2", "failure_reason": "result_error", "error": str(exc), "goal_sequence": goal_sequence})
            return
        if status_code == 4:
            status = NavigationStatus.SUCCEEDED
            message = "Nav2 导航目标已到达。"
            goal_reached = True
        elif status_code in {5, 6}:
            status = NavigationStatus.CANCELLED if status_code == 5 else NavigationStatus.FAILED
            message = "Nav2 导航已取消。" if status_code == 5 else "Nav2 导航失败或被中止。"
            goal_reached = False
        else:
            status = NavigationStatus.FAILED
            message = "Nav2 返回未知终态 status=%s。" % status_code
            goal_reached = False
        with self._lock:
            if goal_sequence != self._goal_sequence or self._state.current_goal_id != goal_id:
                return
            state_metadata = dict(self._state.metadata)
            target_pose = state_metadata.get("target_pose") if isinstance(state_metadata.get("target_pose"), dict) else {}
            LOGGER.info(
                "Nav2 action 结果 goal_id=%s goal_sequence=%s status=%s goal_reached=%s target=%s",
                goal_id,
                goal_sequence,
                status_code,
                goal_reached,
                target_pose,
            )
            self._goal_handle = None
            self._result_future = None
            self._state = NavigationState(
                current_goal_id=goal_id,
                status=status,
                current_pose=self._read_current_pose(),
                goal_reached=goal_reached,
                message=message,
                metadata={"backend": "nav2", "action_status_code": status_code, "action_name": self.runtime.config.nav2.action_name, "goal_sequence": goal_sequence, "target_pose": target_pose},
            )

    def _on_bridge_result(self, msg: Any) -> None:
        """处理桥接节点返回的导航结果。"""
        import json
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        status_str = data.get("status", "")
        detail = data.get("detail", "")
        with self._lock:
            goal_id = self._state.current_goal_id
            if not goal_id:
                return
            if status_str == "succeeded":
                self._state = NavigationState(
                    current_goal_id=goal_id,
                    status=NavigationStatus.SUCCEEDED,
                    current_pose=self._read_current_pose(),
                    goal_reached=True,
                    message="Nav2 导航成功。",
                    metadata={"backend": "nav2", "bridge_detail": detail},
                )
            elif status_str == "cancelled":
                self._state = NavigationState(
                    current_goal_id=goal_id,
                    status=NavigationStatus.CANCELLED,
                    current_pose=self._read_current_pose(),
                    message="Nav2 导航已取消。",
                    metadata={"backend": "nav2", "bridge_detail": detail},
                )
            else:
                self._state = NavigationState(
                    current_goal_id=goal_id,
                    status=NavigationStatus.FAILED,
                    current_pose=self._read_current_pose(),
                    message="Nav2 导航失败：%s" % detail,
                    metadata={"backend": "nav2", "bridge_detail": detail},
                )

    def _on_bridge_feedback(self, msg: Any) -> None:
        """处理桥接节点返回的导航反馈。"""
        import json
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        distance = data.get("distance_remaining")
        with self._lock:
            if self._state.current_goal_id and self._state.status == NavigationStatus.RUNNING:
                metadata = dict(self._state.metadata)
                metadata["distance_remaining"] = distance
                self._state = self._state.model_copy(update={"metadata": metadata}, deep=True)

    def _on_global_costmap(self, message: Any) -> None:
        with self._lock:
            self._global_costmap.update(message)

    def _on_local_costmap(self, message: Any) -> None:
        with self._lock:
            self._local_costmap.update(message)

    def _set_blocked_state(self, goal_id: str, readiness: _Nav2Readiness) -> None:
        with self._lock:
            self._state = NavigationState(
                current_goal_id=goal_id,
                status=NavigationStatus.FAILED,
                current_pose=self._read_current_pose(),
                goal_reached=False,
                message=readiness.message,
                metadata={"backend": "nav2", "waiting_for": readiness.waiting_for, "failure_reason": "nav2_not_ready", "readiness": self._status_dict(readiness)},
            )

    @staticmethod
    def _lifecycle_ready(lifecycle: Dict[str, object]) -> bool:
        return bool(lifecycle) and all(bool(item.get("active")) for item in lifecycle.values() if isinstance(item, dict))

    @staticmethod
    def _lifecycle_observation_uncertain(lifecycle: Dict[str, object]) -> bool:
        if not lifecycle:
            return False
        for item in lifecycle.values():
            if not isinstance(item, dict) or not item.get("available") or item.get("state") is not None or item.get("error"):
                return False
        return True

    @staticmethod
    def _costmaps_ready(costmaps: Dict[str, object], *, current_pose: Optional[Pose] = None) -> bool:
        global_status = costmaps.get("global") if isinstance(costmaps.get("global"), dict) else {}
        local_status = costmaps.get("local") if isinstance(costmaps.get("local"), dict) else {}
        if not bool(global_status.get("streaming") and local_status.get("streaming")):
            return False
        if current_pose is None:
            global_status["current_pose_in_bounds"] = False
            global_status["bounds_error"] = "current_pose_unavailable"
            return False
        return _annotate_pose_in_costmap(current_pose, global_status)

    @staticmethod
    def _status_dict(readiness: _Nav2Readiness) -> Dict[str, object]:
        return {
            "ready": readiness.ready,
            "waiting_for": readiness.waiting_for,
            "message": readiness.message,
            "lifecycle": readiness.lifecycle,
            "action_server_ready": readiness.action_server_ready,
            "costmaps": readiness.costmaps,
            "tf_ready": readiness.tf_ready,
            "errors": readiness.errors,
        }

    @staticmethod
    def _feedback_metadata(feedback: Any) -> Dict[str, object]:
        metadata: Dict[str, object] = {}
        for name in ("distance_remaining", "number_of_recoveries"):
            value = getattr(feedback, name, None)
            if value is not None:
                metadata[name] = float(value) if name == "distance_remaining" else int(value)
        navigation_time = getattr(feedback, "navigation_time", None)
        if navigation_time is not None:
            metadata["navigation_time"] = str(navigation_time)
        return metadata


def _annotate_pose_in_costmap(pose: Pose, costmap_status: Dict[str, object]) -> bool:
    frame_id = str(costmap_status.get("frame_id") or "")
    pose_frame_id = str(pose.frame_id or "")
    if frame_id and pose_frame_id and not frame_ids_semantically_equal(frame_id, pose_frame_id):
        costmap_status.update({"current_pose_in_bounds": False, "bounds_error": "frame_mismatch"})
        return False
    try:
        width = int(costmap_status.get("width") or 0)
        height = int(costmap_status.get("height") or 0)
        resolution_m = float(costmap_status.get("resolution_m") or 0.0)
        origin_x = float(costmap_status.get("origin_x") or 0.0)
        origin_y = float(costmap_status.get("origin_y") or 0.0)
    except (TypeError, ValueError):
        costmap_status.update({"current_pose_in_bounds": False, "bounds_error": "invalid_bounds"})
        return False
    if width <= 0 or height <= 0 or resolution_m <= 0.0:
        costmap_status.update({"current_pose_in_bounds": False, "bounds_error": "invalid_bounds"})
        return False
    max_x = origin_x + float(width) * resolution_m
    max_y = origin_y + float(height) * resolution_m
    x = float(pose.position.x)
    y = float(pose.position.y)
    in_bounds = origin_x <= x < max_x and origin_y <= y < max_y
    costmap_status.update(
        {
            "current_pose": {"frame_id": pose_frame_id, "x": x, "y": y},
            "bounds": {"min_x": origin_x, "min_y": origin_y, "max_x": max_x, "max_y": max_y},
            "current_pose_in_bounds": in_bounds,
            "bounds_error": None if in_bounds else "current_pose_out_of_bounds",
        }
    )
    return in_bounds
