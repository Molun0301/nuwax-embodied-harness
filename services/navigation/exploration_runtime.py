"""平台探索编排运行时。"""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any, Dict, List, Optional

from contracts.base import utc_now
from contracts.geometry import Pose
from contracts.navigation import ExplorationState as ContractExplorationState, ExplorationStatus, ExploreAreaRequest, NavigationGoal
from contracts.runtime_views import ExplorationContext
from services.navigation.config import ExplorationConfig
from services.navigation.frontier import FrontierCandidate, FrontierExplorer
from services.navigation.runtime import GridNavigationPlanner


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
    last_frontier_count: Optional[int] = None
    completed_frontiers: int = 0
    no_gain_rounds: int = 0
    known_cells: int = 0
    waiting_for: Optional[str] = None
    start_time: datetime = field(default_factory=utc_now)
    end_time: Optional[datetime] = None
    last_error: Optional[str] = None
    hit_stop_trigger: Optional[str] = None
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
        if self._current_state.phase not in {
            ExplorationRuntimePhase.SUCCEEDED,
            ExplorationRuntimePhase.FAILED,
            ExplorationRuntimePhase.CANCELLED,
            ExplorationRuntimePhase.HIT_STOP,
        }:
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
    ) -> ExplorationRuntimeState:
        """推进探索状态机。"""

        if self._current_state is None:
            raise RuntimeError("探索任务未启动。")
        if not self._running:
            return self._current_state

        state = self._current_state
        state.waiting_for = None
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
        return ContractExplorationState(
            current_request_id=state.request_id,
            status=self._map_runtime_phase_to_status(state.phase),
            strategy=state.request.strategy,
            covered_ratio=covered_ratio,
            frontier_count=state.last_frontier_count,
            message=self._build_status_message(state),
            metadata={
                "runtime_phase": state.phase.value,
                "frontiers_visited": len(state.visited_frontiers),
                "frontiers_rejected": len(state.rejected_frontiers),
                "completed_frontiers": state.completed_frontiers,
                "known_cells": state.known_cells,
                "no_gain_rounds": state.no_gain_rounds,
                "waiting_for": state.waiting_for,
                "hit_stop_trigger": state.hit_stop_trigger,
                "current_target_goal_id": (
                    state.current_target.navigation_goal.goal_id if state.current_target is not None else None
                ),
                "start_time": state.start_time.isoformat(),
                "end_time": state.end_time.isoformat() if state.end_time is not None else None,
                **dict(state.metadata),
            },
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
        self._finish(self._current_state, phase=ExplorationRuntimePhase.HIT_STOP)

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
        return self._current_state.current_target.navigation_goal.model_copy(deep=True)

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
            state.waiting_for = "occupancy_grid"
            state.last_error = "当前占据栅格不可用，无法选择 frontier。"
            return

        attempted_poses = [*state.visited_frontiers, *state.rejected_frontiers]
        center_pose = state.request.center_pose or current_pose
        frontiers = self._frontier_explorer.select_candidates(
            current_pose=current_pose,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
            planner=self._navigation_planner,
            center_pose=center_pose,
            radius_m=state.request.radius_m,
            attempted_poses=attempted_poses,
        )
        state.last_frontier_count = len(frontiers)
        state.last_error = None
        state.known_cells = self._count_known_cells(occupancy_grid)

        if not frontiers:
            self._finish(state, phase=ExplorationRuntimePhase.SUCCEEDED)
            return

        best_frontier = self._select_best_frontier(frontiers, state)
        if best_frontier is None:
            self._finish(state, phase=ExplorationRuntimePhase.SUCCEEDED)
            return

        state.current_target = FrontierTarget(
            frontier=best_frontier,
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
                },
            ),
        )
        state.phase = ExplorationRuntimePhase.MOVING_TO_FRONTIER

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

        target_pose = state.current_target.navigation_goal.target_pose
        dx = current_pose.position.x - target_pose.position.x
        dy = current_pose.position.y - target_pose.position.y
        distance = (dx * dx + dy * dy) ** 0.5
        if distance <= max(0.0, float(self._config.reach_distance_m)):
            state.current_target.arrived = True
            state.phase = ExplorationRuntimePhase.SCANNING

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
        if state.no_gain_rounds >= int(self._config.frontier_max_no_gain_rounds):
            self._finish(state, phase=ExplorationRuntimePhase.SUCCEEDED)
            return

        state.current_target = None
        state.phase = ExplorationRuntimePhase.PLANNING

    def _check_hit_stop(self, state: ExplorationRuntimeState) -> bool:
        """检查是否命中外部停止条件。"""

        return state.phase == ExplorationRuntimePhase.HIT_STOP

    def _finish(self, state: ExplorationRuntimeState, *, phase: ExplorationRuntimePhase) -> None:
        state.phase = phase
        state.end_time = utc_now()
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
        for frontier in frontiers:
            position = (frontier.pose.position.x, frontier.pose.position.y)
            if position in visited_positions or position in rejected_positions:
                continue
            return frontier
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

    def _build_explore_request(self, state: Optional[ExplorationRuntimeState]) -> Optional[ExploreAreaRequest]:
        if state is None:
            return None
        return state.request.model_copy(deep=True)

    def _build_status_message(self, state: ExplorationRuntimeState) -> str:
        if state.phase == ExplorationRuntimePhase.PLANNING:
            if state.waiting_for == "current_pose":
                return "当前定位尚未就绪，等待探索起步条件。"
            if state.waiting_for == "occupancy_grid":
                return "当前探索地图尚未就绪，等待首帧地图。"
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
        navigation_planner=navigation_planner or GridNavigationPlanner(),
        config=resolved_config,
    )
