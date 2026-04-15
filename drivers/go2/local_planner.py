from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import List, Optional, Tuple

import numpy as np

from contracts.geometry import Pose, Quaternion, Vector3


class LocalPlannerState(IntEnum):
    IDLE = 0
    INITIAL_ROTATION = 1
    PATH_FOLLOWING = 2
    FINAL_ROTATION = 3
    ARRIVED = 4


@dataclass
class Go2LocalPlannerConfig:
    enabled: bool = True
    max_linear_velocity_mps: float = 0.35
    max_angular_velocity_rps: float = 0.8
    min_linear_velocity_mps: float = 0.08
    max_samples_vx: int = 8
    max_samples_vy: int = 5
    max_samples_omega: int = 11
    trajectory_horizon_sec: float = 1.0
    control_frequency_hz: float = 10.0
    orientation_tolerance_rad: float = 0.35
    goal_tolerance_m: float = 0.45
    path_deviation_tolerance_m: float = 0.75
    obstacle_weight: float = 1.5
    path_weight: float = 0.8
    velocity_weight: float = 0.3


@dataclass
class TrajectorySample:
    vx: float
    vy: float
    omega: float
    score: float
    clearance: float


@dataclass
class Go2VelocityCommand:
    linear_x_mps: float
    linear_y_mps: float
    angular_z_rps: float
    state: LocalPlannerState


class Go2DWALocalPlanner:
    """动态窗口法(DWA)局部规划器。

    在A*全局路径基础上，用DWA进行实时动态避障：
    1. 在速度空间采样
    2. 模拟轨迹，评估离障碍物距离、离路径距离、速度大小
    3. 选择最优速度
    """

    def __init__(self, config: Go2LocalPlannerConfig) -> None:
        self.config = config
        self._current_state = LocalPlannerState.IDLE
        self._current_path_index: int = 0
        self._last_odom: Optional[Pose] = None

    def compute_velocity(
        self,
        current_pose: Pose,
        global_plan: List[Tuple[float, float]],
        cost_map: Optional[np.ndarray],
        scan_ranges: Optional[np.ndarray] = None,
        obstacle_distances: Optional[np.ndarray] = None,
    ) -> Go2VelocityCommand:
        if len(global_plan) < 2:
            if self._current_state != LocalPlannerState.IDLE:
                self._current_state = LocalPlannerState.IDLE
            return Go2VelocityCommand(
                linear_x_mps=0.0,
                linear_y_mps=0.0,
                angular_z_rps=0.0,
                state=self._current_state,
            )

        current_yaw = self._yaw_from_pose(current_pose)
        target_point = global_plan[-1]

        dist_to_target = self._point_distance(
            (current_pose.position.x, current_pose.position.y),
            target_point,
        )

        if dist_to_target <= self.config.goal_tolerance_m:
            self._current_state = LocalPlannerState.ARRIVED
            return Go2VelocityCommand(
                linear_x_mps=0.0,
                linear_y_mps=0.0,
                angular_z_rps=0.0,
                state=self._current_state,
            )

        if self._current_state == LocalPlannerState.IDLE:
            self._current_state = LocalPlannerState.INITIAL_ROTATION

        if self._current_state == LocalPlannerState.INITIAL_ROTATION:
            return self._compute_initial_rotation(current_pose, global_plan)

        if self._current_state == LocalPlannerState.PATH_FOLLOWING:
            return self._compute_path_following(
                current_pose,
                global_plan,
                cost_map,
                scan_ranges,
                obstacle_distances,
            )

        return Go2VelocityCommand(
            linear_x_mps=0.0,
            linear_y_mps=0.0,
            angular_z_rps=0.0,
            state=self._current_state,
        )

    def _compute_initial_rotation(
        self,
        current_pose: Pose,
        global_plan: List[Tuple[float, float]],
    ) -> Go2VelocityCommand:
        current_yaw = self._yaw_from_pose(current_pose)
        target_yaw = np.arctan2(
            global_plan[-1][1] - current_pose.position.y,
            global_plan[-1][0] - current_pose.position.x,
        )
        yaw_error = self._normalize_angle(target_yaw - current_yaw)

        if abs(yaw_error) < self.config.orientation_tolerance_rad:
            self._current_state = LocalPlannerState.PATH_FOLLOWING
            return self._compute_path_following(
                current_pose, global_plan, None, None, None
            )

        omega = self._clamp(
            yaw_error * 2.0,
            -self.config.max_angular_velocity_rps,
            self.config.max_angular_velocity_rps,
        )

        return Go2VelocityCommand(
            linear_x_mps=0.0,
            linear_y_mps=0.0,
            angular_z_rps=omega,
            state=self._current_state,
        )

    def _compute_path_following(
        self,
        current_pose: Pose,
        global_plan: List[Tuple[float, float]],
        cost_map: Optional[np.ndarray],
        scan_ranges: Optional[np.ndarray],
        obstacle_distances: Optional[np.ndarray],
    ) -> Go2VelocityCommand:
        current_yaw = self._yaw_from_pose(current_pose)
        current_pos = (current_pose.position.x, current_pose.position.y)

        closest_idx = self._find_closest_point_on_path(global_plan, current_pos)
        self._current_path_index = closest_idx

        lookahead_idx = min(
            closest_idx + 3,
            len(global_plan) - 1,
        )
        lookahead_point = global_plan[lookahead_idx]

        heading_error = self._normalize_angle(
            np.arctan2(
                lookahead_point[1] - current_pos[1],
                lookahead_point[0] - current_pos[0],
            ) - current_yaw
        )

        obstacle_clearance = self._compute_obstacle_clearance(
            current_pose, scan_ranges, obstacle_distances
        )

        if obstacle_clearance < 0.3:
            return Go2VelocityCommand(
                linear_x_mps=0.0,
                linear_y_mps=0.0,
                angular_z_rps=0.0,
                state=self._current_state,
            )
        elif obstacle_clearance < 0.6:
            speed_scale = obstacle_clearance / 0.6
        else:
            speed_scale = 1.0

        if abs(heading_error) > self.config.orientation_tolerance_rad * 2:
            omega = self._clamp(
                heading_error * 1.5,
                -self.config.max_angular_velocity_rps,
                self.config.max_angular_velocity_rps,
            )
            return Go2VelocityCommand(
                linear_x_mps=self.config.min_linear_velocity_mps * speed_scale,
                linear_y_mps=0.0,
                angular_z_rps=omega,
                state=self._current_state,
            )

        best_vx, best_omega = self._dwa_optimize(
            current_pose,
            global_plan,
            closest_idx,
            lookahead_point,
            cost_map,
            scan_ranges,
            obstacle_distances,
            speed_scale,
        )

        return Go2VelocityCommand(
            linear_x_mps=best_vx,
            linear_y_mps=0.0,
            angular_z_rps=best_omega,
            state=self._current_state,
        )

    def _dwa_optimize(
        self,
        current_pose: Pose,
        global_plan: List[Tuple[float, float]],
        closest_idx: int,
        lookahead_point: Tuple[float, float],
        cost_map: Optional[np.ndarray],
        scan_ranges: Optional[np.ndarray],
        obstacle_distances: Optional[np.ndarray],
        speed_scale: float,
    ) -> Tuple[float, float]:
        vx_samples = np.linspace(
            self.config.min_linear_velocity_mps,
            self.config.max_linear_velocity_mps * speed_scale,
            self.config.max_samples_vx,
        )
        omega_samples = np.linspace(
            -self.config.max_angular_velocity_rps,
            self.config.max_angular_velocity_rps,
            self.config.max_samples_omega,
        )

        best_score = -np.inf
        best_vx = 0.0
        best_omega = 0.0

        dt = 1.0 / self.config.control_frequency_hz
        sim_time = self.config.trajectory_horizon_sec

        for vx in vx_samples:
            for omega in omega_samples:
                trajectory = self._simulate_trajectory(
                    current_pose, vx, omega, dt, sim_time
                )

                clearance = self._evaluate_clearance(
                    trajectory, scan_ranges, obstacle_distances
                )
                if clearance <= 0:
                    continue

                path_score = self._evaluate_path_following(
                    trajectory, global_plan, closest_idx, lookahead_point
                )

                velocity_score = vx / self.config.max_linear_velocity_mps

                score = (
                    self.config.obstacle_weight * clearance +
                    self.config.path_weight * path_score +
                    self.config.velocity_weight * velocity_score
                )

                if score > best_score:
                    best_score = score
                    best_vx = vx
                    best_omega = omega

        if best_score < 0:
            return (self.config.min_linear_velocity_mps, 0.0)

        return (best_vx, best_omega)

    def _simulate_trajectory(
        self,
        current_pose: Pose,
        vx: float,
        omega: float,
        dt: float,
        sim_time: float,
    ) -> List[Tuple[float, float]]:
        trajectory = [(current_pose.position.x, current_pose.position.y)]
        x, y, theta = (
            current_pose.position.x,
            current_pose.position.y,
            self._yaw_from_pose(current_pose),
        )

        num_steps = int(sim_time / dt)
        for _ in range(num_steps):
            x += vx * np.cos(theta) * dt
            y += vx * np.sin(theta) * dt
            theta += omega * dt
            trajectory.append((x, y))

        return trajectory

    def _evaluate_clearance(
        self,
        trajectory: List[Tuple[float, float]],
        scan_ranges: Optional[np.ndarray],
        obstacle_distances: Optional[np.ndarray],
    ) -> float:
        if scan_ranges is None or len(scan_ranges) == 0:
            return 1.0

        min_clearance = 1.0
        for x, y in trajectory:
            for i, r in enumerate(scan_ranges):
                if r <= 0 or r > 25.0:
                    continue
                angle = -np.pi / 2 + (i / len(scan_ranges)) * np.pi
                ox = x + r * np.cos(angle)
                oy = y + r * np.sin(angle)

                for tx, ty in trajectory:
                    dist = np.sqrt((tx - ox) ** 2 + (ty - oy) ** 2)
                    if dist < 0.15:
                        clearance = dist / 0.15
                        if clearance < min_clearance:
                            min_clearance = clearance

        return min_clearance

    def _evaluate_path_following(
        self,
        trajectory: List[Tuple[float, float]],
        global_plan: List[Tuple[float, float]],
        closest_idx: int,
        lookahead_point: Tuple[float, float],
    ) -> float:
        if len(trajectory) < 2:
            return 0.0

        final_point = trajectory[-1]
        dist_to_lookahead = self._point_distance(final_point, lookahead_point)
        dist_to_target = self._point_distance(final_point, global_plan[-1])

        path_score = 1.0 - min(1.0, dist_to_lookahead / 2.0)

        return max(0.0, path_score)

    def _compute_obstacle_clearance(
        self,
        current_pose: Pose,
        scan_ranges: Optional[np.ndarray],
        obstacle_distances: Optional[np.ndarray],
    ) -> float:
        if scan_ranges is None or len(scan_ranges) == 0:
            return 1.0

        min_range = float(np.min(scan_ranges))

        front_mask = np.zeros(len(scan_ranges), dtype=bool)
        half_front = len(scan_ranges) // 3
        front_mask[half_front:2*half_front] = True

        front_ranges = scan_ranges[front_mask]
        if len(front_ranges) > 0:
            min_front = float(np.min(front_ranges))
            if min_front < 0.5:
                return min_front

        return min_range

    def _find_closest_point_on_path(
        self,
        path: List[Tuple[float, float]],
        point: Tuple[float, float],
    ) -> int:
        min_dist = np.inf
        closest_idx = 0

        for i, (px, py) in enumerate(path):
            dist = self._point_distance(point, (px, py))
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        return closest_idx

    def _point_distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def _normalize_angle(self, angle: float) -> float:
        return np.arctan2(np.sin(angle), np.cos(angle))

    def _yaw_from_pose(self, pose: Pose) -> float:
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def _clamp(
        self,
        value: float,
        min_val: float,
        max_val: float,
    ) -> float:
        return max(min_val, min(max_val, value))

    def reset(self) -> None:
        self._current_state = LocalPlannerState.IDLE
        self._current_path_index = 0
        self._last_odom = None

    @property
    def state(self) -> LocalPlannerState:
        return self._current_state

    def is_idle(self) -> bool:
        return self._current_state == LocalPlannerState.IDLE

    def is_arrived(self) -> bool:
        return self._current_state == LocalPlannerState.ARRIVED
