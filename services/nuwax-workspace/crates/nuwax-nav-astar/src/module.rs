//! Stream / RPC adapter
//!
//! Phase 1 仅占位。Foundation `nuwax-stream` 与 `nuwax-nav-types-messages` 完成后，
//! 在此模块内实现：
//! - `ReplanningAStarPlannerModule`：暴露输入端口 `odom / goal / costmap / stop_movement`、
//!   输出端口 `path / cmd_vel / goal_reached / navigation_state`。
//! - RPC：`set_goal / cancel_goal / get_state / is_goal_reached / set_replanning_enabled / set_safe_goal_clearance`。
//!
//! 与 Python `dimos/navigation/replanning_a_star/module.py` 行为对齐，但不引入 LCM/DDS。

// 当前 Foundation 未交付，本模块保留 trait 占位，避免编译错误
// Foundation 完成后切换为：
//   use nuwax_nav_types_messages::*;
//   use nuwax_nav_base::{NavigationInterface, NavError};

use crate::global_planner::{GlobalPlanner, GlobalPlannerConfig};
use crate::helpers::{NavError, PoseStamped, Twist2};

/// 模块 trait 占位（Foundation 完成后切换到 `nuwax-nav-base::NavigationInterface`）
pub trait NavigationInterface {
    /// 启动模块
    fn start(&mut self) -> Result<(), NavError>;
    /// 停止模块
    fn stop(&mut self) -> Result<(), NavError>;
    /// 设置 goal
    fn set_goal(&mut self, goal: PoseStamped) -> Result<bool, NavError>;
    /// 取消 goal
    fn cancel_goal(&mut self) -> Result<bool, NavError>;
    /// 当前状态（string）
    fn get_state(&self) -> &'static str;
    /// 是否到达
    fn is_goal_reached(&self) -> bool;
}

/// ReplanningAStar 模块包装
pub struct ReplanningAStarPlannerModule {
    planner: GlobalPlanner,
    running: bool,
}

impl ReplanningAStarPlannerModule {
    /// 构造
    #[must_use]
    pub fn new(config: GlobalPlannerConfig) -> Self {
        Self {
            planner: GlobalPlanner::new(config),
            running: false,
        }
    }

    /// 访问内部 GlobalPlanner（供测试 / Phase 1 调用）
    pub fn planner_mut(&mut self) -> &mut GlobalPlanner {
        &mut self.planner
    }
}

impl NavigationInterface for ReplanningAStarPlannerModule {
    fn start(&mut self) -> Result<(), NavError> {
        if self.running {
            return Ok(());
        }
        self.planner.start();
        self.running = true;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), NavError> {
        if !self.running {
            return Ok(());
        }
        self.planner.stop();
        self.running = false;
        Ok(())
    }

    fn set_goal(&mut self, goal: PoseStamped) -> Result<bool, NavError> {
        self.planner.handle_goal_request(goal);
        Ok(true)
    }

    fn cancel_goal(&mut self) -> Result<bool, NavError> {
        self.planner.cancel_goal(false);
        Ok(true)
    }

    fn get_state(&self) -> &'static str {
        match self.planner.status() {
            crate::helpers::NavigationStatus::Idle => "idle",
            crate::helpers::NavigationStatus::Planning => "planning",
            crate::helpers::NavigationStatus::Moving => "moving",
            crate::helpers::NavigationStatus::Reached => "reached",
            crate::helpers::NavigationStatus::Blocked => "blocked",
            crate::helpers::NavigationStatus::Cancelled => "cancelled",
            crate::helpers::NavigationStatus::Error => "error",
        }
    }

    fn is_goal_reached(&self) -> bool {
        self.planner.is_goal_reached()
    }
}

/// 一次性 cmd_vel 推算 helper（供 Phase 1 测试或上层 stream 适配层使用）
pub fn twist_to_proto(t: Twist2) -> (f64, f64) {
    (t.linear.x, t.angular_z)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::helpers::{make_pose_stamped, make_twist2};

    #[test]
    fn module_lifecycle() {
        let mut m = ReplanningAStarPlannerModule::new(GlobalPlannerConfig::default());
        assert!(m.start().is_ok());
        assert!(m.start().is_ok()); // 重复 start 不报错
        assert_eq!(m.get_state(), "idle");
        let _ = m.stop();
    }

    #[test]
    fn module_set_and_cancel_goal() {
        let mut m = ReplanningAStarPlannerModule::new(GlobalPlannerConfig::default());
        let _ = m.start();
        let goal = make_pose_stamped("map", 1.0, 1.0, 0.0);
        assert!(m.set_goal(goal).unwrap());
        let _ = m.cancel_goal().unwrap();
        assert!(!m.is_goal_reached());
    }

    #[test]
    fn twist_to_proto_roundtrip() {
        let t = make_twist2(0.5, 0.1);
        let (l, a) = twist_to_proto(t);
        assert!((l - 0.5).abs() < 1e-9);
        assert!((a - 0.1).abs() < 1e-9);
    }
}
