//! Stream / RPC adapter (rework22 P0-11)
//!
//! `Module` trait + `SimplePlannerModule` 实现:
//! - 4 输入 (goal / stop_movement / terrain_map / terrain_map_ext)
//! - 2 输出 (path / safety_hold)
//! - 10Hz lifecycle;不接 LCM transport,用 in-memory mpsc。

use crate::{NavError, PointStamped, Pose3};

/// Tick 周期(rework22 P0-11:10Hz)
pub const TICK_PERIOD_MS: u64 = 100;

/// 模块输出:plan 完成后应输出 Path + safety hold。
#[derive(Debug, Clone)]
pub enum ModuleOutput {
    /// 产生一条路径(waypoint 列表)
    Path(Vec<crate::Point2>),
    /// 安全 hold 信号(主动让 navigation 暂停,例如探测到 stop / 校验失败)
    SafetyHold,
    /// 无输出(idle / 等待输入)
    None,
}

/// 输入端口 payload
#[derive(Debug, Clone)]
pub enum ModuleInput {
    /// Goal 输入
    Goal(PointStamped),
    /// Stop movement
    StopMovement(bool),
    /// Terrain map(原始)
    TerrainMap(Vec<crate::Point3>),
    /// Terrain map ext(扩展)
    TerrainMapExt {
        /// points
        points: Vec<crate::Point3>,
        /// robot pose
        pose: Pose3,
    },
}

/// rework22 P0-11 `Module` trait:统一接口
pub trait Module: Send + Sync + 'static {
    /// 启动
    fn start(&mut self) -> Result<(), NavError>;
    /// 停止
    fn stop(&mut self) -> Result<(), NavError>;
    /// 处理 goal
    fn on_goal(&mut self, goal: PointStamped) -> Result<(), NavError>;
    /// 处理 stop movement
    fn on_stop_movement(&mut self, stop: bool) -> Result<(), NavError>;
    /// 处理 terrain map
    fn on_terrain_map(&mut self, points: Vec<crate::Point3>) -> Result<(), NavError>;
    /// 处理 terrain map ext
    fn on_terrain_map_ext(&mut self, points: Vec<crate::Point3>, pose: Pose3) -> Result<(), NavError>;
    /// 当前 tick 输出
    fn tick(&mut self) -> ModuleOutput;
}

/// `SimplePlannerModule`:plan 完成后输出 Path + safety hold。
pub struct SimplePlannerModule {
    running: bool,
    /// 最近一次 goal(尚未 plan)
    pending_goal: Option<PointStamped>,
    /// 最近一次 stop 信号
    pending_stop: Option<bool>,
    /// 安全 hold 持续:收到 stop=true 后,下一次 tick 强制返回 SafetyHold
    safety_hold_emitted: bool,
}

impl SimplePlannerModule {
    /// 构造
    #[must_use]
    pub fn new() -> Self {
        Self {
            running: false,
            pending_goal: None,
            pending_stop: None,
            safety_hold_emitted: false,
        }
    }

    /// 是否运行
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.running
    }
}

impl Default for SimplePlannerModule {
    fn default() -> Self {
        Self::new()
    }
}

impl Module for SimplePlannerModule {
    fn start(&mut self) -> Result<(), NavError> {
        self.running = true;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), NavError> {
        self.running = false;
        Ok(())
    }

    fn on_goal(&mut self, goal: PointStamped) -> Result<(), NavError> {
        if !self.running {
            return Err(NavError::Internal("module not running".into()));
        }
        self.pending_goal = Some(goal);
        // 新 goal 清除之前的 safety hold
        self.safety_hold_emitted = false;
        Ok(())
    }

    fn on_stop_movement(&mut self, stop: bool) -> Result<(), NavError> {
        if !self.running {
            return Err(NavError::Internal("module not running".into()));
        }
        self.pending_stop = Some(stop);
        if stop {
            // stop 到来后下次 tick 必发 SafetyHold
            self.safety_hold_emitted = false;
        }
        Ok(())
    }

    fn on_terrain_map(&mut self, _points: Vec<crate::Point3>) -> Result<(), NavError> {
        if !self.running {
            return Err(NavError::Internal("module not running".into()));
        }
        // Phase 1:不接 costmap 缓存,留 hook 给后续 PR
        Ok(())
    }

    fn on_terrain_map_ext(
        &mut self,
        _points: Vec<crate::Point3>,
        _pose: Pose3,
    ) -> Result<(), NavError> {
        if !self.running {
            return Err(NavError::Internal("module not running".into()));
        }
        // Phase 1:同上
        Ok(())
    }

    fn tick(&mut self) -> ModuleOutput {
        if !self.running {
            return ModuleOutput::None;
        }
        // stop 优先:只要最近收到 stop=true,先发 SafetyHold 再清 pending
        if let Some(true) = self.pending_stop.take() {
            self.safety_hold_emitted = true;
            return ModuleOutput::SafetyHold;
        }
        if self.safety_hold_emitted {
            // 上一次发了 SafetyHold,本次保持 hold 直到收到新 goal
            self.safety_hold_emitted = false;
            return ModuleOutput::SafetyHold;
        }
        if let Some(goal) = self.pending_goal.take() {
            // Phase 1:直接产出一条单点 path(goal -> goal)
            // Phase 2 接 costmap 后真正 plan
            ModuleOutput::Path(vec![crate::Point2 {
                x: goal.point.x,
                y: goal.point.y,
            }])
        } else {
            ModuleOutput::None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_types_messages::NavFrameId;

    fn make_goal(x: f64, y: f64) -> PointStamped {
        PointStamped::from_xyz(NavFrameId::from_static("map"), x, y, 0.0)
    }

    #[test]
    fn module_lifecycle_idempotent() {
        let mut m = SimplePlannerModule::new();
        assert!(m.start().is_ok());
        assert!(m.start().is_ok());
        assert!(m.is_running());
        assert!(m.stop().is_ok());
        assert!(!m.is_running());
    }

    #[test]
    fn on_inputs_require_running() {
        let mut m = SimplePlannerModule::new();
        assert!(m.on_goal(make_goal(1.0, 2.0)).is_err());
        assert!(m.on_stop_movement(true).is_err());
    }

    #[test]
    fn goal_emits_path_after_tick() {
        let mut m = SimplePlannerModule::new();
        m.start().unwrap();
        m.on_goal(make_goal(3.0, 4.0)).unwrap();
        match m.tick() {
            ModuleOutput::Path(p) => {
                assert_eq!(p.len(), 1);
                assert!((p[0].x - 3.0).abs() < 1e-9);
                assert!((p[0].y - 4.0).abs() < 1e-9);
            }
            other => panic!("expected Path, got {other:?}"),
        }
    }

    #[test]
    fn stop_emits_safety_hold_then_idle() {
        let mut m = SimplePlannerModule::new();
        m.start().unwrap();
        m.on_stop_movement(true).unwrap();
        // 第一次 tick:SafetyHold
        assert!(matches!(m.tick(), ModuleOutput::SafetyHold));
        // 第二次 tick:进入 hold-tail(仍 SafetyHold,直到新 goal)
        assert!(matches!(m.tick(), ModuleOutput::SafetyHold));
        // 第三次 tick:hold 释放 → None
        assert!(matches!(m.tick(), ModuleOutput::None));
    }

    #[test]
    fn new_goal_after_stop_clears_hold() {
        let mut m = SimplePlannerModule::new();
        m.start().unwrap();
        m.on_stop_movement(true).unwrap();
        let _ = m.tick(); // SafetyHold
        m.on_goal(make_goal(1.0, 1.0)).unwrap();
        // 新 goal 清 hold → 直接 Path
        match m.tick() {
            ModuleOutput::Path(_) => {}
            other => panic!("expected Path after new goal, got {other:?}"),
        }
    }
}

/// 兼容历史命名
pub fn subscribe_goal<F>(callback: F) -> Result<(), NavError>
where
    F: Fn(PointStamped) + Send + Sync + 'static,
{
    let _ = callback;
    Err(NavError::Internal(
        "subscribe_goal:待 Foundation 接入 transport,改用 SimplePlannerModule.on_goal".into(),
    ))
}

/// 兼容历史命名
pub fn subscribe_stop_movement<F>(callback: F) -> Result<(), NavError>
where
    F: Fn(bool) + Send + Sync + 'static,
{
    let _ = callback;
    Err(NavError::Internal(
        "subscribe_stop_movement:待 Foundation 接入 transport,改用 SimplePlannerModule.on_stop_movement".into(),
    ))
}

/// 兼容历史命名
pub fn subscribe_terrain_map_ext<F>(callback: F) -> Result<(), NavError>
where
    F: Fn(Vec<crate::Point3>, Pose3) + Send + Sync + 'static,
{
    let _ = callback;
    Err(NavError::Internal(
        "subscribe_terrain_map_ext:待 Foundation 接入 transport,改用 SimplePlannerModule.on_terrain_map_ext".into(),
    ))
}