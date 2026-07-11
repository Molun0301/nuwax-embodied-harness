//! Stream / RPC adapter
//!
//! Phase 1 仅占位。Foundation `nuwax-stream` 与 `nuwax-nav-types-messages` 完成后，
//! 在此模块内实现：
//! - `ReplanningAStarPlannerModule`：暴露输入端口 `odom / goal / costmap / stop_movement`、
//!   输出端口 `path / cmd_vel / goal_reached / navigation_state`。
//! - RPC：`set_goal / cancel_goal / get_state / is_goal_reached / set_replanning_enabled / set_safe_goal_clearance`。
//!
//! 与 Python `dimos/navigation/replanning_a_star/module.py` 行为对齐，但不引入 LCM/DDS。
//!
//! ## rework22 P0-11
//!
//! 新增 `Module` trait(对外契约的 10Hz lifecycle + 三个 `on_*` 输入),
//! 与既有 `NavigationInterface` 共存(后者为 RPC 旧名,前者为新版契约)。
//! `ModuleOutput` 表示本 tick 产生的主要输出;端口用 `nuwax-core::channel`
//! mpsc 实现 in-memory mock,不做 LCM/DDS 接线。

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

// ===== rework22 P0-11:Module trait + ModuleOutput + 端口工厂 =====

/// rework22 P0-11 `Module` trait。
///
/// 三个输入(`on_odom / on_costmap / on_goal`)和 lifecycle (`start / stop`)。
/// 单一 trait 避免和上面 `NavigationInterface` 重复;`NavigationInterface`
/// 继续保留作 RPC 旧名。
pub trait Module: Send + Sync + 'static {
    /// 10Hz lifecycle tick(等价于"启动并 tick")。`NavigationInterface::start/stop`
    /// 仍可独立调用,RPC 层用 NavigationInterface;Module 层的 lifecycle 由本 trait 拥有。
    fn tick(&mut self) -> ModuleOutput;
    /// 收到 odom 输入
    fn on_odom(&mut self, odom: PoseStamped);
    /// 收到 costmap 输入
    fn on_costmap(&mut self, grid: crate::navigation_map::CostMap2D);
    /// 主动 goal(RPC `set_goal` 等价入口)
    fn on_goal(&mut self, goal: PoseStamped) -> Result<ModuleOutput, NavError>;
}

/// 模块单次输出(rework22 P0-11)
#[derive(Debug, Clone)]
pub enum ModuleOutput {
    /// 产生一条 Path2D
    Path(crate::helpers::Path2D),
    /// 产生一条 Twist2 cmd_vel
    Twist(Twist2),
    /// 报告 goal_reached(true / false)
    GoalReached(bool),
    /// 无输出(idle / 等待输入)
    None,
}

impl Module for ReplanningAStarPlannerModule {
    fn tick(&mut self) -> ModuleOutput {
        // Phase 1 占位:从 self.planner 收集本 tick 输出
        if self.planner.is_goal_reached() {
            ModuleOutput::GoalReached(true)
        } else {
            ModuleOutput::None
        }
    }

    fn on_odom(&mut self, odom: PoseStamped) {
        self.planner.handle_odom(odom);
    }

    fn on_costmap(&mut self, _grid: crate::navigation_map::CostMap2D) {
        // Phase 1 占位:costmap 输入留 hook 后续 PR 接到 NavigationMap。
    }

    fn on_goal(&mut self, goal: PoseStamped) -> Result<ModuleOutput, NavError> {
        self.planner.handle_goal_request(goal);
        // Phase 1 立即返回 None;Phase 2 包成 Path2D 输出
        Ok(ModuleOutput::None)
    }
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
