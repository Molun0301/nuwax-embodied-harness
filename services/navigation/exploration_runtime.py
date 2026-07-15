"""平台探索编排运行时。"""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any, Dict, List, Optional

from contracts.base import utc_now
from contracts.geometry import Pose
from contracts.navigation import ExplorationState as ContractExplorationState, ExplorationStatus, ExploreAreaRequest, NavigationGoal
from contracts.runtime_views import ExplorationContext
from services.navigation.config import ExplorationConfig, NavigationBackendConfig
from services.navigation.frontier import FrontierCandidate, FrontierExplorer
from services.navigation.runtime import GridNavigationPlanner

LOGGER = logging.getLogger("nuwax_robot_bridge.navigation.exploration_runtime")


class ExplorationRuntimePhase(Enum):
    """平台探索运行时内部阶段。"""

    IDLE = "idle"
    PLANNING = "planning"
    MOVING_TO_FRONTIER = "moving_to_frontier"
    SCANNING = "scanning"
    HIT_STOP = "hit_stop"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass
class FrontierTarget:
    """当前探索目标。"""

    frontier: FrontierCandidate
    navigation_goal: NavigationGoal
    arrived: bool = False
    scanned: bool = False
    map_revision: Optional[str] = None
    costmap_revision: Optional[str] = None
    pose_revision: Optional[str] = None
    created_at: datetime = field(default_factory=utc_now)
    stale: bool = False
    stale_reason: Optional[str] = None


@dataclass
class ExplorationRuntimeState:
    """探索运行时状态。"""

    request_id: str
    request: ExploreAreaRequest
    phase: ExplorationRuntimePhase
    current_target: Optional[FrontierTarget] = None
    visited_frontiers: List[Pose] = field(default_factory=list)
    rejected_frontiers: List[Pose] = field(default_factory=list)
    frontier_history: List[FrontierCandidate] = field(default_factory=list)
    latest_frontiers: List[FrontierCandidate] = field(default_factory=list)
    last_frontier_count: Optional[int] = None
    completed_frontiers: int = 0
    no_gain_rounds: int = 0
    known_cells: int = 0
    waiting_for: Optional[str] = None
    failure_code: Optional[str] = None
    frontier_freshness: Dict[str, Any] = field(default_factory=dict)
    map_revision: Optional[str] = None
    costmap_revision: Optional[str] = None
    pose_revision: Optional[str] = None
    start_time: datetime = field(default_factory=utc_now)
    end_time: Optional[datetime] = None
    last_error: Optional[str] = None
    hit_stop_trigger: Optional[str] = None
    stop_reason: Optional[str] = None
    stop_condition: Optional[str] = None
    stop_condition_detail: Dict[str, Any] = field(default_factory=dict)
    metadata: Dict[str, Any] = field(default_factory=dict)


class PlatformExplorationRuntime:
    """平台探索编排运行时。"""

    provider_name = "platform_exploration_runtime"
    provider_version = "0.1.0"

    def __init__(
        self,
        *,
        frontier_explorer: FrontierExplorer,
        navigation_planner: GridNavigationPlanner,
        config: ExplorationConfig,
    ) -> None:
        self._frontier_explorer = frontier_explorer
        self._navigation_planner = navigation_planner
        self._config = config
        self._current_state: Optional[ExplorationRuntimeState] = None
        self._running = False

    @property
    def is_running(self) -> bool:
        """返回探索是否正在运行。"""

        return self._running and self._current_state is not None

    def is_available(self) -> bool:
        """运行时对象本身是否可用。"""

        return True

    def is_exploration_available(self) -> bool:
        """探索运行时是否具备执行能力。"""

        return bool(self._config.enabled)

    def start(self, request: ExploreAreaRequest) -> bool:
        """启动探索任务。"""

        if self._running or not self.is_exploration_available():
            return False
        start_hook = getattr(self._frontier_explorer, "start_exploration", None)
        if callable(start_hook):
            start_hook()
        self._current_state = ExplorationRuntimeState(
            request_id=request.request_id,
            request=request.model_copy(deep=True),
            phase=ExplorationRuntimePhase.PLANNING,
            metadata={
                "map_name": request.map_name,
                "center_pose": request.center_pose.model_dump(mode="json") if request.center_pose is not None else None,
                "radius_m": request.radius_m,
                "target_name": request.target_name,
                "max_duration_sec": request.max_duration_sec,
                "stop_conditions": dict(request.stop_conditions),
                **dict(request.metadata),
            },
        )
        self._running = True
        return True

    def start_exploration(self, request: ExploreAreaRequest) -> bool:
        """兼容 `ExplorationProvider` 的启动接口。"""

        return self.start(request)

    def stop(self) -> bool:
        """停止探索任务。"""

        if self._current_state is None:
            return False
        if not self._is_terminal_phase(self._current_state.phase):
            self._record_stop(
                self._current_state,
                reason="cancelled",
                condition="manual_cancel",
                detail={"cancelled_at": utc_now().isoformat()},
            )
            self._finish(self._current_state, phase=ExplorationRuntimePhase.CANCELLED)
        else:
            self._stop_frontier_runtime()
            self._running = False
        return True

    def stop_exploration(self) -> bool:
        """兼容 `ExplorationProvider` 的停止接口。"""

        return self.stop()

    def tick(
        self,
        *,
        current_pose: Optional[Pose],
        occupancy_grid=None,
        cost_map=None,
        runtime_metadata: Optional[Dict[str, Any]] = None,
    ) -> ExplorationRuntimeState:
        """推进探索状态机。"""

        if self._current_state is None:
            raise RuntimeError("探索任务未启动。")
        if not self._running:
            return self._current_state

        state = self._current_state
        self._apply_runtime_metadata(state, runtime_metadata)
        state.waiting_for = None
        if self._check_stop_conditions(state):
            return state
        if state.phase == ExplorationRuntimePhase.PLANNING:
            self._handle_planning_state(state, current_pose, occupancy_grid, cost_map)
        elif state.phase == ExplorationRuntimePhase.MOVING_TO_FRONTIER:
            self._handle_moving_state(state, current_pose)
        elif state.phase == ExplorationRuntimePhase.SCANNING:
            self._handle_scanning_state(state, occupancy_grid)
        elif state.phase in {
            ExplorationRuntimePhase.HIT_STOP,
            ExplorationRuntimePhase.SUCCEEDED,
            ExplorationRuntimePhase.FAILED,
            ExplorationRuntimePhase.CANCELLED,
        }:
            self._running = False
            self._stop_frontier_runtime()
        if not self._is_terminal_phase(state.phase):
            self._check_stop_conditions(state)
        return state

    def get_exploration_state(self) -> ContractExplorationState:
        """返回当前探索状态。"""

        if self._current_state is None:
            return ContractExplorationState(status=ExplorationStatus.IDLE)

        state = self._current_state
        covered_ratio = None
        total_attempts = state.completed_frontiers + len(state.rejected_frontiers)
        if total_attempts > 0:
            covered_ratio = float(state.completed_frontiers) / float(total_attempts)
        metadata = {
            **dict(state.metadata),
            "runtime_phase": state.phase.value,
            "frontiers_visited": len(state.visited_frontiers),
            "frontiers_rejected": len(state.rejected_frontiers),
            "completed_frontiers": state.completed_frontiers,
            "latest_frontiers": [self._frontier_candidate_to_metadata(item) for item in state.latest_frontiers],
            "current_frontier": (
                self._frontier_candidate_to_metadata(state.current_target.frontier, target=state.current_target)
                if state.current_target is not None
                else None
            ),
            "known_cells": state.known_cells,
            "no_gain_rounds": state.no_gain_rounds,
            "waiting_for": state.waiting_for,
            "failure_code": state.failure_code,
            "frontier_freshness": dict(state.frontier_freshness),
            "map_revision": state.map_revision,
            "costmap_revision": state.costmap_revision,
            "pose_revision": state.pose_revision,
            "hit_stop_trigger": state.hit_stop_trigger,
            "stop_reason": state.stop_reason,
            "stop_condition": state.stop_condition,
            "stop_condition_detail": dict(state.stop_condition_detail),
            "elapsed_time_sec": self._elapsed_time_sec(state),
            "current_target_goal_id": (
                state.current_target.navigation_goal.goal_id if state.current_target is not None else None
            ),
            "start_time": state.start_time.isoformat(),
            "end_time": state.end_time.isoformat() if state.end_time is not None else None,
        }
        return ContractExplorationState(
            current_request_id=state.request_id,
            status=self._map_runtime_phase_to_status(state.phase),
            strategy=state.request.strategy,
            covered_ratio=covered_ratio,
            frontier_count=state.last_frontier_count,
            message=self._build_status_message(state),
            metadata=metadata,
        )

    def get_exploration_context(self) -> ExplorationContext:
        """返回包含请求和状态的探索上下文。"""

        return ExplorationContext(
            current_request=self._build_explore_request(self._current_state),
            exploration_state=self.get_exploration_state(),
            backend_name=self.provider_name,
            metadata={
                "running": self.is_running,
                "elapsed_time_sec": self._elapsed_time_sec(self._current_state),
            },
        )

    def report_hit_stop(self, reason: str) -> None:
        """由外部能力报告命中即停。"""

        if self._current_state is None:
            return
        self._current_state.hit_stop_trigger = str(reason or "").strip() or "external_report"
        self._record_stop(
            self._current_state,
            reason="hit_stop",
            condition="hit_stop",
            detail={"trigger": self._current_state.hit_stop_trigger},
        )
        self._finish(self._current_state, phase=ExplorationRuntimePhase.HIT_STOP)

    def block_on_prerequisite(
        self,
        *,
        waiting_for: str,
        message: str,
        failure_code: str = "exploration_prerequisite_unavailable",
        metadata: Optional[Dict[str, Any]] = None,
    ) -> None:
        """记录探索前置条件阻塞，保持任务在可恢复规划态。"""

        if self._current_state is None:
            return
        state = self._current_state
        if state.current_target is not None:
            self.mark_current_frontier_stale(
                reason="prerequisite_changed:%s" % waiting_for,
                replan=False,
            )
        state.phase = ExplorationRuntimePhase.PLANNING
        state.waiting_for = str(waiting_for or "prerequisite")
        state.failure_code = str(failure_code or "exploration_prerequisite_unavailable")
        state.last_error = str(message or "探索前置条件未就绪。")
        self._apply_runtime_metadata(state, metadata)
        state.metadata["readiness_phase"] = "blocked"

    def fail_current(
        self,
        *,
        reason: str,
        failure_code: str,
        waiting_for: Optional[str] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> None:
        """将当前探索任务置为失败。"""

        if self._current_state is None:
            return
        state = self._current_state
        state.waiting_for = waiting_for
        state.failure_code = str(failure_code or "exploration_failed")
        state.last_error = str(reason or "探索失败。")
        self._apply_runtime_metadata(state, metadata)
        self._finish(state, phase=ExplorationRuntimePhase.FAILED)

    def mark_current_frontier_stale(self, *, reason: str, replan: bool = True) -> None:
        """标记当前 frontier 过期，不再复用旧目标继续导航。"""

        if self._current_state is None or self._current_state.current_target is None:
            return
        target = self._current_state.current_target
        target.stale = True
        target.stale_reason = str(reason or "frontier_stale")
        self._current_state.rejected_frontiers.append(target.frontier.pose)
        self._current_state.metadata["last_stale_frontier"] = self._frontier_candidate_to_metadata(target.frontier, target=target)
        self._current_state.current_target = None
        self._current_state.waiting_for = "frontier"
        self._current_state.failure_code = "frontier_stale"
        self._current_state.last_error = "frontier 已过期，等待基于最新地图重新规划。"
        if replan:
            self._current_state.phase = ExplorationRuntimePhase.PLANNING

    def reject_current_target(self, reason: str) -> None:
        """由外部能力拒绝当前前沿目标，要求重新规划。"""

        if self._current_state is None or self._current_state.current_target is None:
            return
        target = self._current_state.current_target
        self._current_state.rejected_frontiers.append(target.frontier.pose)
        self._current_state.current_target = None
        self._current_state.last_error = str(reason or "").strip() or "frontier_rejected"
        self._current_state.phase = ExplorationRuntimePhase.PLANNING

    def get_current_navigation_goal(self) -> Optional[NavigationGoal]:
        """返回当前正在执行的前沿导航目标。"""

        if self._current_state is None or self._current_state.current_target is None:
            return None
        if self._current_state.phase != ExplorationRuntimePhase.MOVING_TO_FRONTIER:
            return None
        goal = self._current_state.current_target.navigation_goal.model_copy(deep=True)
        if goal.target_pose is not None:
            LOGGER.debug(
                "frontier 导航目标读取 goal_id=%s target=(%.3f, %.3f, %.3f) frame=%s",
                goal.goal_id,
                float(goal.target_pose.position.x),
                float(goal.target_pose.position.y),
                float(goal.target_pose.position.z),
                goal.target_pose.frame_id,
            )
        return goal

    def mark_navigation_arrived(self) -> None:
        """由平台导航链路标记当前前沿已到达。"""

        if self._current_state is None or self._current_state.current_target is None:
            return
        self._current_state.current_target.arrived = True
        self._current_state.phase = ExplorationRuntimePhase.SCANNING

    def _handle_planning_state(
        self,
        state: ExplorationRuntimeState,
        current_pose: Optional[Pose],
        occupancy_grid,
        cost_map,
    ) -> None:
        if current_pose is None:
            state.waiting_for = "current_pose"
            state.last_error = "当前位姿不可用，无法选择 frontier。"
            return
        if occupancy_grid is None:
            state.waiting_for = "map"
            state.failure_code = "map_unavailable"
            state.last_error = "当前 /map 占据栅格不可用，无法选择 frontier。"
            return
        if cost_map is None:
            state.waiting_for = "costmap"
            state.failure_code = "costmap_unavailable"
            state.last_error = "当前 costmap 不可用，无法安全选择 frontier。"
            return

        attempted_poses = [*state.visited_frontiers, *state.rejected_frontiers]
        center_pose = state.request.center_pose or current_pose

        if len(attempted_poses) == 0:
            if self._costmap_not_ready_guard(state):
                # 非阻塞守卫：costmap 尚未稳定，本 tick 直接返回，等待下一 tick 自然推进。
                return

        LOGGER.info(
            "frontier 规划请求 request_id=%s current=(%.3f, %.3f, %.3f) frame=%s center=(%.3f, %.3f, %.3f) radius_m=%s map_revision=%s costmap_revision=%s pose_revision=%s attempted=%d",
            state.request_id,
            float(current_pose.position.x),
            float(current_pose.position.y),
            float(current_pose.position.z),
            current_pose.frame_id,
            float(center_pose.position.x),
            float(center_pose.position.y),
            float(center_pose.position.z),
            None if state.request.radius_m is None else float(state.request.radius_m),
            state.map_revision,
            state.costmap_revision,
            state.pose_revision,
            len(attempted_poses),
        )
        frontiers = self._frontier_explorer.select_candidates(
            current_pose=current_pose,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
            planner=self._navigation_planner,
            center_pose=center_pose,
            radius_m=state.request.radius_m,
            attempted_poses=attempted_poses,
        )
        if not self.is_running:
            LOGGER.info("frontier 计算完成但探索已停止，丢弃结果 request_id=%s", state.request_id)
            return
        state.latest_frontiers = list(frontiers)
        state.last_frontier_count = len(frontiers)
        LOGGER.info("frontier 规划接收候选 request_id=%s count=%d", state.request_id, len(frontiers))
        state.last_error = None
        state.failure_code = None
        state.known_cells = self._count_known_cells(occupancy_grid)
        if self._check_stop_conditions(state):
            return

        if not frontiers:
            state.waiting_for = "frontier"
            state.failure_code = "frontier_unavailable"
            state.last_error = "最新 live map/costmap/pose 未产生可达 frontier。"
            self._finish(state, phase=ExplorationRuntimePhase.FAILED)
            return

        best_frontier = self._select_best_frontier(frontiers, state)
        if best_frontier is None:
            state.waiting_for = "frontier"
            state.failure_code = "frontier_exhausted"
            state.last_error = "最新 frontier 均已访问或被拒绝，无法继续探索。"
            self._finish(state, phase=ExplorationRuntimePhase.FAILED)
            return

        LOGGER.info(
            "frontier 目标选中 request_id=%s pose=(%.3f, %.3f, %.3f) frame=%s score=%.3f distance_m=%.3f cluster_size=%d info_gain=%d",
            state.request_id,
            float(best_frontier.pose.position.x),
            float(best_frontier.pose.position.y),
            float(best_frontier.pose.position.z),
            best_frontier.pose.frame_id,
            float(best_frontier.score),
            float(best_frontier.path_distance_m),
            int(best_frontier.cluster_size),
            int(best_frontier.info_gain_cells),
        )
        frontier_revision = self._frontier_revision(state)
        state.frontier_freshness = {
            "fresh": True,
            "frontier_revision": frontier_revision,
            "computed_at": utc_now().isoformat(),
            "map_revision": state.map_revision,
            "costmap_revision": state.costmap_revision,
            "pose_revision": state.pose_revision,
        }
        state.current_target = FrontierTarget(
            frontier=best_frontier,
            map_revision=state.map_revision,
            costmap_revision=state.costmap_revision,
            pose_revision=state.pose_revision,
            navigation_goal=NavigationGoal(
                goal_id=(
                    "explore_frontier_"
                    f"{best_frontier.pose.position.x:.2f}_{best_frontier.pose.position.y:.2f}"
                ),
                target_pose=best_frontier.pose,
                map_name=state.request.map_name,
                target_name=state.request.target_name,
                metadata={
                    "exploration_request_id": state.request_id,
                    "frontier_score": best_frontier.score,
                    "frontier_cluster_size": best_frontier.cluster_size,
                    "frontier_revision": frontier_revision,
                    "map_revision": state.map_revision,
                    "costmap_revision": state.costmap_revision,
                    "pose_revision": state.pose_revision,
                    "frontier_computed_at": state.frontier_freshness.get("computed_at"),
                },
            ),
        )
        target_pose = state.current_target.navigation_goal.target_pose
        LOGGER.info(
            "frontier 导航目标组装 goal_id=%s request_id=%s target=(%.3f, %.3f, %.3f) frame=%s map_name=%s target_name=%s revision=%s",
            state.current_target.navigation_goal.goal_id,
            state.request_id,
            float(target_pose.position.x),
            float(target_pose.position.y),
            float(target_pose.position.z),
            target_pose.frame_id,
            state.request.map_name,
            state.request.target_name,
            frontier_revision,
        )
        state.phase = ExplorationRuntimePhase.MOVING_TO_FRONTIER

    def _costmap_not_ready_guard(self, state: ExplorationRuntimeState) -> bool:
        """首轮 frontier 计算前的非阻塞 costmap 稳定守卫。

        单线程 tick 模型下不能在函数内自旋等待（costmap_revision 只能由后续 tick 推进），
        因此改为非阻塞守卫：未达阈值且未超时则设置 waiting_for 并要求本 tick 提前返回，
        让 geometry 探索循环线程在下一个 tick 继续推进；超时后放行并告警。

        Returns:
            True 表示尚未就绪、本 tick 应提前返回；False 表示已就绪或已超时，可继续 frontier 计算。
        """

        min_revision = int(getattr(self._config, "costmap_min_revision", 5))
        timeout_sec = float(getattr(self._config, "costmap_wait_timeout_sec", 15.0))

        current_rev = self._parse_costmap_revision(state.costmap_revision)
        if current_rev >= min_revision:
            return False

        wait_started = state.metadata.get("costmap_wait_started_monotonic")
        now = time.monotonic()
        if not isinstance(wait_started, (int, float)):
            wait_started = now
            state.metadata["costmap_wait_started_monotonic"] = wait_started
            LOGGER.info(
                "首轮 frontier 计算前等待 costmap 稳定 request_id=%s current_revision=%s min_revision=%d timeout_sec=%.1f",
                state.request_id,
                state.costmap_revision,
                min_revision,
                timeout_sec,
            )

        if (now - float(wait_started)) >= timeout_sec:
            LOGGER.warning(
                "等待 costmap 稳定超时，继续执行首轮 frontier 计算 request_id=%s current_revision=%s min_revision=%d",
                state.request_id,
                state.costmap_revision,
                min_revision,
            )
            return False

        state.waiting_for = "costmap_stabilizing"
        return True

    @staticmethod
    def _parse_costmap_revision(revision: Optional[str]) -> int:
        if not revision:
            return 0
        try:
            return int(str(revision).split(":")[-1])
        except (ValueError, IndexError):
            return 0

    def _handle_moving_state(
        self,
        state: ExplorationRuntimeState,
        current_pose: Optional[Pose],
    ) -> None:
        if state.current_target is None:
            state.phase = ExplorationRuntimePhase.PLANNING
            return
        if current_pose is None:
            state.waiting_for = "current_pose"
            state.last_error = "移动过程中丢失定位。"
            return

        state.last_error = None
        if state.current_target.stale:
            self.mark_current_frontier_stale(reason=state.current_target.stale_reason or "frontier_stale")

    def _handle_scanning_state(self, state: ExplorationRuntimeState, occupancy_grid) -> None:
        if state.current_target is None:
            state.phase = ExplorationRuntimePhase.PLANNING
            return

        previous_known_cells = state.known_cells
        current_known_cells = self._count_known_cells(occupancy_grid)
        info_gain_cells = max(0, current_known_cells - previous_known_cells)
        state.known_cells = max(previous_known_cells, current_known_cells)
        if info_gain_cells <= int(self._config.frontier_min_information_gain_cells):
            state.no_gain_rounds += 1
        else:
            state.no_gain_rounds = 0

        state.current_target.scanned = True
        state.visited_frontiers.append(state.current_target.frontier.pose)
        state.frontier_history.append(state.current_target.frontier)
        state.completed_frontiers += 1

        if self._check_hit_stop(state):
            return
        if self._check_stop_conditions(state):
            return
        if state.no_gain_rounds >= int(self._config.frontier_max_no_gain_rounds):
            self._record_stop(
                state,
                reason="stop_condition",
                condition="frontier_max_no_gain_rounds",
                detail={
                    "no_gain_rounds": state.no_gain_rounds,
                    "threshold": int(self._config.frontier_max_no_gain_rounds),
                },
            )
            self._finish(state, phase=ExplorationRuntimePhase.SUCCEEDED)
            return

        state.current_target = None
        state.phase = ExplorationRuntimePhase.PLANNING

    def _check_hit_stop(self, state: ExplorationRuntimeState) -> bool:
        """检查是否命中外部停止条件。"""

        return state.phase == ExplorationRuntimePhase.HIT_STOP

    def _check_stop_conditions(self, state: ExplorationRuntimeState) -> bool:
        """检查请求级停止条件，达成后以成功终态结束探索。"""

        if self._is_terminal_phase(state.phase):
            return True
        duration_limit = self._max_duration_limit_sec(state)
        if duration_limit is not None:
            elapsed_time_sec = self._elapsed_time_sec(state)
            if elapsed_time_sec >= duration_limit:
                return self._finish_stop_condition(
                    state,
                    condition="max_duration_sec",
                    detail={
                        "elapsed_time_sec": elapsed_time_sec,
                        "limit_sec": duration_limit,
                    },
                )

        stop_conditions = dict(state.request.stop_conditions or {})
        max_completed_frontiers = self._numeric_stop_condition(stop_conditions, "max_completed_frontiers")
        if max_completed_frontiers is not None and max_completed_frontiers > 0:
            if float(state.completed_frontiers) >= max_completed_frontiers:
                return self._finish_stop_condition(
                    state,
                    condition="max_completed_frontiers",
                    detail={
                        "completed_frontiers": state.completed_frontiers,
                        "threshold": max_completed_frontiers,
                    },
                )

        min_known_cells = self._numeric_stop_condition(stop_conditions, "min_known_cells")
        if min_known_cells is not None and min_known_cells > 0:
            if float(state.known_cells) >= min_known_cells:
                return self._finish_stop_condition(
                    state,
                    condition="min_known_cells",
                    detail={
                        "known_cells": state.known_cells,
                        "threshold": min_known_cells,
                    },
                )

        max_no_gain_rounds = self._numeric_stop_condition(stop_conditions, "max_no_gain_rounds")
        if max_no_gain_rounds is not None and max_no_gain_rounds > 0:
            if float(state.no_gain_rounds) >= max_no_gain_rounds:
                return self._finish_stop_condition(
                    state,
                    condition="max_no_gain_rounds",
                    detail={
                        "no_gain_rounds": state.no_gain_rounds,
                        "threshold": max_no_gain_rounds,
                    },
                )

        min_covered_ratio = self._numeric_stop_condition(stop_conditions, "min_covered_ratio")
        covered_ratio = self._covered_ratio(state)
        if min_covered_ratio is not None and 0.0 < min_covered_ratio <= 1.0 and covered_ratio is not None:
            if covered_ratio >= min_covered_ratio:
                return self._finish_stop_condition(
                    state,
                    condition="min_covered_ratio",
                    detail={
                        "covered_ratio": covered_ratio,
                        "threshold": min_covered_ratio,
                    },
                )
        return False

    def _finish_stop_condition(
        self,
        state: ExplorationRuntimeState,
        *,
        condition: str,
        detail: Dict[str, Any],
    ) -> bool:
        self._record_stop(
            state,
            reason="stop_condition",
            condition=condition,
            detail=detail,
        )
        self._finish(state, phase=ExplorationRuntimePhase.SUCCEEDED)
        return True

    def _record_stop(
        self,
        state: ExplorationRuntimeState,
        *,
        reason: str,
        condition: str,
        detail: Dict[str, Any],
    ) -> None:
        state.stop_reason = str(reason or "").strip() or None
        state.stop_condition = str(condition or "").strip() or None
        state.stop_condition_detail = dict(detail or {})
        state.metadata.update(
            {
                "stop_reason": state.stop_reason,
                "stop_condition": state.stop_condition,
                "stop_condition_detail": dict(state.stop_condition_detail),
            }
        )

    def _finish(self, state: ExplorationRuntimeState, *, phase: ExplorationRuntimePhase) -> None:
        state.phase = phase
        state.end_time = utc_now()
        if phase == ExplorationRuntimePhase.FAILED and state.stop_reason is None:
            self._record_stop(
                state,
                reason="failed",
                condition=state.failure_code or "exploration_failed",
                detail={"message": state.last_error},
            )
        self._running = False
        self._stop_frontier_runtime()

    def _stop_frontier_runtime(self) -> None:
        stop_hook = getattr(self._frontier_explorer, "stop_exploration", None)
        if callable(stop_hook):
            stop_hook()

    def _select_best_frontier(
        self,
        frontiers: List[FrontierCandidate],
        state: ExplorationRuntimeState,
    ) -> Optional[FrontierCandidate]:
        visited_positions = {(pose.position.x, pose.position.y) for pose in state.visited_frontiers}
        rejected_positions = {(pose.position.x, pose.position.y) for pose in state.rejected_frontiers}
        LOGGER.debug(
            "frontier 选择器接收候选 request_id=%s count=%d visited=%d rejected=%d",
            state.request_id,
            len(frontiers),
            len(visited_positions),
            len(rejected_positions),
        )
        for frontier in frontiers:
            position = (frontier.pose.position.x, frontier.pose.position.y)
            if position in visited_positions or position in rejected_positions:
                LOGGER.debug(
                    "frontier 选择器跳过已尝试坐标 request_id=%s pose=(%.3f, %.3f, %.3f) frame=%s score=%.3f",
                    state.request_id,
                    float(frontier.pose.position.x),
                    float(frontier.pose.position.y),
                    float(frontier.pose.position.z),
                    frontier.pose.frame_id,
                    float(frontier.score),
                )
                continue
            LOGGER.debug(
                "frontier 选择器返回目标 request_id=%s pose=(%.3f, %.3f, %.3f) frame=%s score=%.3f distance_m=%.3f",
                state.request_id,
                float(frontier.pose.position.x),
                float(frontier.pose.position.y),
                float(frontier.pose.position.z),
                frontier.pose.frame_id,
                float(frontier.score),
                float(frontier.path_distance_m),
            )
            return frontier
        LOGGER.info("frontier 选择器没有可返回目标 request_id=%s", state.request_id)
        return None

    def _count_known_cells(self, occupancy_grid) -> int:
        if occupancy_grid is None:
            return 0
        counter = getattr(self._frontier_explorer, "count_known_cells", None)
        if callable(counter):
            try:
                return int(counter(occupancy_grid))
            except Exception:
                pass
        return sum(1 for cell in list(getattr(occupancy_grid, "data", []) or []) if int(cell) >= 0)

    @staticmethod
    def _frontier_candidate_to_metadata(frontier: FrontierCandidate, *, target: Optional[FrontierTarget] = None) -> Dict[str, Any]:
        metadata = {
            "id": (
                "frontier_"
                f"{frontier.pose.position.x:.3f}_{frontier.pose.position.y:.3f}"
            ),
            "pose": frontier.pose.model_dump(mode="json"),
            "geometry": {
                "type": "point",
                "coordinates": [
                    frontier.pose.position.x,
                    frontier.pose.position.y,
                    frontier.pose.position.z,
                ],
            },
            "score": frontier.score,
            "cluster_size": frontier.cluster_size,
            "info_gain_cells": frontier.info_gain_cells,
            "path_distance_m": frontier.path_distance_m,
            "average_cost": frontier.average_cost,
        }
        if target is not None:
            metadata.update({
                "fresh": not target.stale,
                "stale": target.stale,
                "stale_reason": target.stale_reason,
                "map_revision": target.map_revision,
                "costmap_revision": target.costmap_revision,
                "pose_revision": target.pose_revision,
                "computed_at": target.created_at.isoformat(),
            })
        return metadata

    def _apply_runtime_metadata(self, state: ExplorationRuntimeState, metadata: Optional[Dict[str, Any]]) -> None:
        if not isinstance(metadata, dict):
            return
        state.metadata.update(dict(metadata))
        state.map_revision = _string_or_none(metadata.get("map_revision")) or state.map_revision
        state.costmap_revision = _string_or_none(metadata.get("costmap_revision")) or state.costmap_revision
        state.pose_revision = _string_or_none(metadata.get("pose_revision")) or state.pose_revision
        frontier_freshness = metadata.get("frontier_freshness")
        if isinstance(frontier_freshness, dict):
            state.frontier_freshness.update(frontier_freshness)

    def _frontier_revision(self, state: ExplorationRuntimeState) -> str:
        return "map=%s|costmap=%s|pose=%s|round=%d" % (
            state.map_revision or "unknown",
            state.costmap_revision or "unknown",
            state.pose_revision or "unknown",
            state.completed_frontiers + len(state.rejected_frontiers),
        )

    def _build_explore_request(self, state: Optional[ExplorationRuntimeState]) -> Optional[ExploreAreaRequest]:
        if state is None:
            return None
        return state.request.model_copy(deep=True)

    def _build_status_message(self, state: ExplorationRuntimeState) -> str:
        if state.phase == ExplorationRuntimePhase.PLANNING:
            if state.waiting_for == "current_pose":
                return "当前定位尚未就绪，等待探索起步条件。"
            if state.waiting_for in ("occupancy_grid", "map"):
                return "当前 /map 占据栅格尚未就绪，等待 SLAM 首帧地图。"
            if state.waiting_for == "costmap":
                return "全局代价地图尚未就绪，请检查定位与坐标变换是否持续更新。"
            if state.waiting_for == "frontier":
                return "live map/costmap/pose 暂未产生可达 frontier；若持续如此，请检查 amcl 是否健康（map→odom TF 是否在更新）。"
            return "正在规划下一个 frontier。"
        if state.phase == ExplorationRuntimePhase.MOVING_TO_FRONTIER:
            if state.current_target is None:
                return "正在前往探索目标。"
            return "正在前往 frontier，得分 %.2f。" % state.current_target.frontier.score
        if state.phase == ExplorationRuntimePhase.SCANNING:
            return "正在扫描当前 frontier 周边环境。"
        if state.phase == ExplorationRuntimePhase.HIT_STOP:
            return "探索命中停机：%s。" % (state.hit_stop_trigger or "已满足停止条件")
        if state.phase == ExplorationRuntimePhase.SUCCEEDED:
            if state.stop_reason == "stop_condition" and state.stop_condition:
                return "探索停止条件已达成：%s。" % self._stop_condition_label(state.stop_condition)
            return "探索完成，共访问 %d 个 frontier。" % len(state.visited_frontiers)
        if state.phase == ExplorationRuntimePhase.FAILED:
            return "探索失败：%s。" % (state.last_error or "未知错误")
        if state.phase == ExplorationRuntimePhase.CANCELLED:
            return "探索已取消。"
        return "探索未启动。"

    def _map_runtime_phase_to_status(self, phase: ExplorationRuntimePhase) -> ExplorationStatus:
        status_map = {
            ExplorationRuntimePhase.IDLE: ExplorationStatus.IDLE,
            ExplorationRuntimePhase.PLANNING: ExplorationStatus.ACCEPTED,
            ExplorationRuntimePhase.MOVING_TO_FRONTIER: ExplorationStatus.RUNNING,
            ExplorationRuntimePhase.SCANNING: ExplorationStatus.RUNNING,
            ExplorationRuntimePhase.HIT_STOP: ExplorationStatus.SUCCEEDED,
            ExplorationRuntimePhase.SUCCEEDED: ExplorationStatus.SUCCEEDED,
            ExplorationRuntimePhase.FAILED: ExplorationStatus.FAILED,
            ExplorationRuntimePhase.CANCELLED: ExplorationStatus.CANCELLED,
        }
        return status_map.get(phase, ExplorationStatus.IDLE)

    @staticmethod
    def _is_terminal_phase(phase: ExplorationRuntimePhase) -> bool:
        return phase in {
            ExplorationRuntimePhase.HIT_STOP,
            ExplorationRuntimePhase.SUCCEEDED,
            ExplorationRuntimePhase.FAILED,
            ExplorationRuntimePhase.CANCELLED,
        }

    def _max_duration_limit_sec(self, state: ExplorationRuntimeState) -> Optional[float]:
        limits: List[float] = []
        if state.request.max_duration_sec is not None:
            limits.append(float(state.request.max_duration_sec))
        condition_limit = self._numeric_stop_condition(dict(state.request.stop_conditions or {}), "max_duration_sec")
        if condition_limit is not None and condition_limit > 0.0:
            limits.append(condition_limit)
        if not limits:
            return None
        return min(limits)

    @staticmethod
    def _numeric_stop_condition(stop_conditions: Dict[str, Any], key: str) -> Optional[float]:
        if key not in stop_conditions:
            return None
        value = stop_conditions.get(key)
        if isinstance(value, bool):
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(number):
            return None
        return number

    @staticmethod
    def _covered_ratio(state: ExplorationRuntimeState) -> Optional[float]:
        total_attempts = state.completed_frontiers + len(state.rejected_frontiers)
        if total_attempts <= 0:
            return None
        return float(state.completed_frontiers) / float(total_attempts)

    @staticmethod
    def _stop_condition_label(condition: str) -> str:
        labels = {
            "max_duration_sec": "最大探索时长",
            "max_completed_frontiers": "最大 frontier 完成数",
            "min_covered_ratio": "最小覆盖比例",
            "max_no_gain_rounds": "最大无增益轮数",
            "min_known_cells": "最小已知栅格数",
            "frontier_max_no_gain_rounds": "frontier 无增益上限",
        }
        return labels.get(condition, condition)

    @staticmethod
    def _elapsed_time_sec(state: Optional[ExplorationRuntimeState]) -> float:
        if state is None:
            return 0.0
        end_time = state.end_time or utc_now()
        return max(0.0, (end_time - state.start_time).total_seconds())


def create_platform_exploration_runtime(
    *,
    config: Optional[ExplorationConfig] = None,
    frontier_explorer: Optional[FrontierExplorer] = None,
    navigation_planner: Optional[GridNavigationPlanner] = None,
) -> PlatformExplorationRuntime:
    """构造默认的平台探索运行时。"""

    resolved_config = config or ExplorationConfig(enabled=True)
    return PlatformExplorationRuntime(
        frontier_explorer=frontier_explorer or FrontierExplorer(resolved_config),
        navigation_planner=navigation_planner or GridNavigationPlanner(NavigationBackendConfig()),
        config=resolved_config,
    )


def _string_or_none(value: Any) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip()
    return text or None
