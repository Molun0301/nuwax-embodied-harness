//! 局部路径跟随状态机
//!
//! 对应 Python 源 `dimos/navigation/replanning_a_star/local_planner.py`。
//!
//! 状态机：
//! - `Idle` → `InitialRotation` → `PathFollowing` → `FinalRotation` → `Arrived`
//! - 或 `Idle` → `PathFollowing`（如果 yaw error 在 tolerance 内且位置未到）
//! - 或 `Idle` → `FinalRotation`（如果已经在 pose_index 0 位置）
//!
//! Phase 1：本模块以**确定性 tick**形式暴露状态机（不内置 thread / reactive stream）。
//! 由 [`crate::global_planner`] 在每次 tick 中显式调用 `step()` 并消费输出 twist 与 stop message。
//! Foundation 完成 `nuwax-stream` 后，可选接入 thread + reactive subject。

use crate::controllers::{angle_diff, Controller, PController};
use crate::helpers::{
    make_path2d, make_twist2_zero, pose_to_point2, pose_xy_yaw, Path2D, PoseStamped, Twist2,
};
use crate::navigation_map::CostMap2D;
use crate::path_clearance::PathClearance;
use crate::path_distancer::PathDistancer;

/// 局部规划状态
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LocalPlannerState {
    /// 空闲
    Idle,
    /// 初始对齐到第一段方向
    InitialRotation,
    /// 沿路径前进
    PathFollowing,
    /// 终点对齐
    FinalRotation,
    /// 已到达
    Arrived,
}

/// 停止原因（与 Python `StopMessage` 对应）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopMessage {
    /// 到达 goal
    Arrived,
    /// 前方检测到 obstacle
    ObstacleFound,
    /// 错误
    Error,
}

/// 局部规划器（deterministic core）
pub struct LocalPlanner {
    controller: PController,
    state: LocalPlannerState,
    state_unique_id: u32,
    pose_index: usize,
    current_odom: Option<PoseStamped>,
    path: Option<Path2D>,
    path_distancer: Option<PathDistancer>,
    path_clearance: Option<PathClearance>,
    costmap: Option<CostMap2D>,
    orientation_tolerance: f64,
    goal_tolerance: f64,
    #[allow(dead_code)] // 字段由 ctor 持有,供后续 monitor/log API 暴露
    speed: f64,
}

impl LocalPlanner {
    /// 构造
    #[must_use]
    pub fn new(speed: f64, control_frequency: f64, goal_tolerance: f64) -> Self {
        Self {
            controller: PController::new(speed, control_frequency),
            state: LocalPlannerState::Idle,
            state_unique_id: 0,
            pose_index: 0,
            current_odom: None,
            path: None,
            path_distancer: None,
            path_clearance: None,
            costmap: None,
            orientation_tolerance: 0.35,
            goal_tolerance,
            speed,
        }
    }

    /// 启动跟随一条新路径
    pub fn start_planning(&mut self, path: Path2D) {
        self.stop_planning();
        self.path = Some(make_path2d(path.points.clone()));
        self.path_distancer = Some(PathDistancer::new(self.path.as_ref().expect("set"), 0.5));
        self.path_clearance = None;
        self.pose_index = 0;
        self.state = LocalPlannerState::Idle;
        self.state_unique_id = 0;
    }

    /// 停止规划
    pub fn stop_planning(&mut self) {
        self.path = None;
        self.path_distancer = None;
        self.path_clearance = None;
        self.state = LocalPlannerState::Idle;
        self.state_unique_id = 0;
        self.pose_index = 0;
        self.controller.reset_errors();
    }

    /// 更新 odom
    pub fn handle_odom(&mut self, odom: PoseStamped) {
        self.current_odom = Some(odom);
    }

    /// 更新 costmap（每次 tick 调用）
    pub fn update_costmap(&mut self, costmap: CostMap2D) {
        self.costmap = Some(costmap);
    }

    /// 当前状态
    #[must_use]
    pub fn state(&self) -> LocalPlannerState {
        self.state
    }

    /// 当前 (state, unique_id)
    #[must_use]
    pub fn unique_state(&self) -> (LocalPlannerState, u32) {
        (self.state, self.state_unique_id)
    }

    /// 当前路径点到路径距离
    #[must_use]
    pub fn get_distance_to_path(&self) -> Option<f64> {
        let pd = self.path_distancer.as_ref()?;
        let odom = self.current_odom.as_ref()?;
        Some(pd.get_distance_to_path(pose_to_point2(odom)))
    }

    /// 单次确定性 tick（返回 `(Twist2, Option<StopMessage>)`）
    ///
    /// - 若返回 `Some(StopMessage)`，全局规划器应据此触发 arrived / replan / cancel。
    /// - 若返回 `None`，表示继续按当前 twist 运动。
    pub fn step(&mut self, robot_width: f64) -> (Twist2, Option<StopMessage>) {
        // 初始化：若 path 已设置且状态仍 Idle，进入 InitialRotation（或 PathFollowing / FinalRotation）
        if self.state == LocalPlannerState::Idle {
            if let Some(path) = self.path.as_ref() {
                if let Some(odom) = self.current_odom.as_ref() {
                    let (ox, oy, oyaw) = pose_xy_yaw(odom);
                    let first_yaw = if let Some(_p) = path.points.first() {
                        // 第一段方向
                        if path.points.len() >= 2 {
                            let a = path.points[0];
                            let b = path.points[1];
                            (b.y - a.y).atan2(b.x - a.x)
                        } else {
                            oyaw
                        }
                    } else {
                        oyaw
                    };
                    let initial_yaw_error = angle_diff(first_yaw, oyaw);
                    self.controller.reset_yaw_error(initial_yaw_error);
                    let angle_in_tol = initial_yaw_error.abs() < self.orientation_tolerance;
                    let new_state = if angle_in_tol {
                        // 检查位置
                        let position_in_tol = if let Some(p0) = path.points.first() {
                            let dx = p0.x - ox;
                            let dy = p0.y - oy;
                            (dx * dx + dy * dy).sqrt() < 0.01
                        } else {
                            true
                        };
                        if position_in_tol {
                            LocalPlannerState::FinalRotation
                        } else {
                            LocalPlannerState::PathFollowing
                        }
                    } else {
                        LocalPlannerState::InitialRotation
                    };
                    self.change_state(new_state);
                }
            }
        }

        // 检查 costmap 与 obstacle
        let mut obstacle_ahead = false;
        if let Some(pc) = self.path_clearance.as_mut() {
            if let Some(cm) = self.costmap.as_ref() {
                pc.update_costmap(cm.clone());
                pc.update_pose_index(self.pose_index);
            }
            obstacle_ahead = pc.is_obstacle_ahead();
        } else if let (Some(cm), Some(path)) = (self.costmap.as_ref(), self.path.as_ref()) {
            // 第一次：构造 path_clearance
            let mut pc = PathClearance::new(path.clone(), robot_width, 3.0);
            pc.update_costmap(cm.clone());
            pc.update_pose_index(self.pose_index);
            obstacle_ahead = pc.is_obstacle_ahead();
            self.path_clearance = Some(pc);
        }

        if obstacle_ahead {
            return (make_twist2_zero(), Some(StopMessage::ObstacleFound));
        }

        match self.state {
            LocalPlannerState::Idle => (make_twist2_zero(), None),
            LocalPlannerState::InitialRotation => self.compute_initial_rotation(),
            LocalPlannerState::PathFollowing => self.compute_path_following(),
            LocalPlannerState::FinalRotation => self.compute_final_rotation(),
            LocalPlannerState::Arrived => (make_twist2_zero(), Some(StopMessage::Arrived)),
        }
    }

    fn compute_initial_rotation(&mut self) -> (Twist2, Option<StopMessage>) {
        let (path, odom) = match (self.path.as_ref(), self.current_odom.as_ref()) {
            (Some(p), Some(o)) => (p, o),
            _ => return (make_twist2_zero(), None),
        };
        let (_, _, oyaw) = pose_xy_yaw(odom);
        let first_yaw = if path.points.len() >= 2 {
            let a = path.points[0];
            let b = path.points[1];
            (b.y - a.y).atan2(b.x - a.x)
        } else {
            oyaw
        };
        let yaw_error = angle_diff(first_yaw, oyaw);
        if yaw_error.abs() < self.orientation_tolerance {
            self.change_state(LocalPlannerState::PathFollowing);
            return self.compute_path_following();
        }
        // 接近 goal：跳过 path-following，直接 FinalRotation
        if let Some(pd) = self.path_distancer.as_ref() {
            let pos = pose_to_point2(odom);
            if pd.distance_to_goal(pos) < self.goal_tolerance {
                self.change_state(LocalPlannerState::FinalRotation);
                return self.compute_final_rotation();
            }
        }
        (self.controller.rotate(yaw_error), None)
    }

    fn compute_path_following(&mut self) -> (Twist2, Option<StopMessage>) {
        let (pd, odom) = match (self.path_distancer.as_ref(), self.current_odom.as_ref()) {
            (Some(p), Some(o)) => (p, o),
            _ => return (make_twist2_zero(), None),
        };
        let pos = pose_to_point2(odom);
        if pd.distance_to_goal(pos) < self.goal_tolerance {
            self.change_state(LocalPlannerState::FinalRotation);
            return self.compute_final_rotation();
        }
        let closest = pd.find_closest_point_index(pos);
        self.pose_index = closest;
        let lookahead = pd.find_lookahead_point(closest);
        let t = self
            .controller
            .advance((lookahead.x, lookahead.y), odom.clone());
        (t, None)
    }

    fn compute_final_rotation(&mut self) -> (Twist2, Option<StopMessage>) {
        let (path, odom) = match (self.path.as_ref(), self.current_odom.as_ref()) {
            (Some(p), Some(o)) => (p, o),
            _ => return (make_twist2_zero(), None),
        };
        let (_, _, oyaw) = pose_xy_yaw(odom);
        if let Some(last) = path.points.last() {
            if path.points.len() >= 2 {
                let prev = path.points[path.points.len() - 2];
                let goal_yaw = (last.y - prev.y).atan2(last.x - prev.x);
                let yaw_error = angle_diff(goal_yaw, oyaw);
                if yaw_error.abs() < self.orientation_tolerance {
                    self.change_state(LocalPlannerState::Arrived);
                    return (make_twist2_zero(), Some(StopMessage::Arrived));
                }
                return (self.controller.rotate(yaw_error), None);
            }
        }
        self.change_state(LocalPlannerState::Arrived);
        (make_twist2_zero(), Some(StopMessage::Arrived))
    }

    fn change_state(&mut self, new_state: LocalPlannerState) {
        if new_state == self.state {
            return;
        }
        self.state = new_state;
        self.state_unique_id = self.state_unique_id.wrapping_add(1);
        tracing::info!("local planner state changed: {:?}", new_state);
    }

    /// 当前控制器（mutable，用于测试）
    pub fn controller_mut(&mut self) -> &mut PController {
        &mut self.controller
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::helpers::{make_path2d, make_pose_stamped, Point2};
    use crate::navigation_map::CostMap2D;

    #[test]
    fn fresh_planner_is_idle() {
        let lp = LocalPlanner::new(0.5, 10.0, 0.2);
        assert_eq!(lp.state(), LocalPlannerState::Idle);
    }

    #[test]
    fn start_then_idle_step_returns_zero_when_no_odom() {
        let mut lp = LocalPlanner::new(0.5, 10.0, 0.2);
        lp.start_planning(make_path2d(vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
        ]));
        // 无 odom, step 后返回 zero
        let (t, stop) = lp.step(0.3);
        assert_eq!(t.linear.x, 0.0);
        assert!(stop.is_none());
    }

    #[test]
    fn aligned_odom_skips_initial_rotation() {
        let mut lp = LocalPlanner::new(0.5, 10.0, 0.2);
        lp.start_planning(make_path2d(vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
        ]));
        let cm = CostMap2D::new_all_free("map", 20, 20, 0.1);
        lp.update_costmap(cm);
        lp.handle_odom(make_pose_stamped("map", 0.05, 0.0, 0.0));
        let _ = lp.step(0.3);
        // 朝向与第一段 (1,0) 一致，应该直接进入 PathFollowing
        assert_eq!(lp.state(), LocalPlannerState::PathFollowing);
    }

    #[test]
    fn obstacle_ahead_triggers_stop() {
        let mut lp = LocalPlanner::new(0.5, 10.0, 0.2);
        lp.start_planning(make_path2d(vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(2.0, 0.0),
            Point2::new(3.0, 0.0),
        ]));
        let mut cm = CostMap2D::new_all_free("map", 40, 10, 0.1);
        // 在 path 上 (2.0, 0.0) 附近放 obstacle（cell 20,5 -> world (2.05, 0.55)）
        cm.set(20, 5, 100);
        lp.update_costmap(cm);
        lp.handle_odom(make_pose_stamped("map", 0.05, 0.0, 0.0));
        let (_t, stop) = lp.step(1.2); // robot_width=1.2 让 disk 半径覆盖 y=5
        assert!(matches!(stop, Some(StopMessage::ObstacleFound)));
    }

    #[test]
    fn stop_planning_resets_state() {
        let mut lp = LocalPlanner::new(0.5, 10.0, 0.2);
        lp.start_planning(make_path2d(vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
        ]));
        lp.stop_planning();
        assert_eq!(lp.state(), LocalPlannerState::Idle);
    }

    #[test]
    fn close_to_goal_triggers_final_rotation() {
        // goal_tolerance=0.5 让 path 上靠 goal 但还没到 tol
        let mut lp = LocalPlanner::new(0.5, 10.0, 0.5);
        lp.start_planning(make_path2d(vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
        ]));
        let cm = CostMap2D::new_all_free("map", 20, 20, 0.1);
        lp.update_costmap(cm);
        // 接近 goal（distance_to_goal = 0.05 < 0.5），yaw 偏差大 → FinalRotation
        lp.handle_odom(make_pose_stamped("map", 0.95, 0.0, 1.0));
        let _ = lp.step(0.3);
        assert_eq!(lp.state(), LocalPlannerState::FinalRotation);
    }

    #[test]
    fn distance_to_path_returns_value() {
        let mut lp = LocalPlanner::new(0.5, 10.0, 0.2);
        lp.start_planning(make_path2d(vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
        ]));
        lp.handle_odom(make_pose_stamped("map", 0.5, 0.5, 0.0));
        let d = lp.get_distance_to_path();
        assert!(d.is_some());
        let d = d.unwrap();
        assert!((d - 0.5).abs() < 1e-9);
    }
}
