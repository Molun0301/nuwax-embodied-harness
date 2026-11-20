//! 三维执行入口。启动时选择实现，两者调用同一调度器与地图工作线程代码。
use crate::sampling_follower::SamplingFollower;
use crate::{
    ActualExecutionContext, ExecutionMap, ExecutionPath, MapInitializer, MeasuredMotion,
    NavPlanner, PlannerStep, RegisteredFrame, RuntimeError, SurfaceFollower, SurfaceFollowerConfig,
    TerminalPhase,
};
use nuwax_config::sections::SurfaceLocalPlanner;
use nuwax_nav_astar::navigation_map::CostMap2D;
use nuwax_nav_types_messages::{GoalOptions, Path2D, Path3D, PoseStamped, Twist2};
use std::{sync::Arc, time::Duration};

enum LocalExecution {
    Scan(SurfaceFollower),
    Sampling(SamplingFollower),
}

/// 由三维导航配置选定的局部执行器。
pub struct SurfaceExecutor(LocalExecution);

impl SurfaceExecutor {
    pub fn new(config: SurfaceFollowerConfig) -> Result<Self, RuntimeError> {
        Self::with_map_initializer(config, None)
    }

    pub fn with_map_initializer(
        config: SurfaceFollowerConfig,
        initializer: Option<MapInitializer>,
    ) -> Result<Self, RuntimeError> {
        match config.local_planner {
            SurfaceLocalPlanner::Scan => SurfaceFollower::with_map_initializer(config, initializer)
                .map(|follower| Self(LocalExecution::Scan(follower))),
            SurfaceLocalPlanner::Sampling => {
                SamplingFollower::with_map_initializer(config, initializer)
                    .map(|follower| Self(LocalExecution::Sampling(follower)))
            }
        }
    }

    /// 当前样条或本拍采样预览。
    pub fn local_path(&self) -> Option<&Path3D> {
        match &self.0 {
            LocalExecution::Scan(scan) => scan.local_path(),
            LocalExecution::Sampling(sampling) => sampling.local_path(),
        }
    }

    /// 已安装的 SCAN 轨迹数量。
    pub fn local_plans(&self) -> u64 {
        match &self.0 {
            LocalExecution::Scan(scan) => scan.local_plans(),
            LocalExecution::Sampling(_) => 0,
        }
    }

    /// 仿真里 SCAN 保留物理响应回执，采样使用上一拍的下发值。
    pub fn observe_simulated_command(&mut self, requested: Twist2, physical: Twist2) {
        match &mut self.0 {
            LocalExecution::Scan(scan) => scan.observe_applied_command(physical),
            LocalExecution::Sampling(sampling) => sampling.observe_applied_command(requested),
        }
    }

    /// 前视方向诊断，采样控制拍才有值。
    pub fn lookahead_yaw_error(&self) -> Option<f64> {
        match &self.0 {
            LocalExecution::Scan(_) => None,
            LocalExecution::Sampling(sampling) => sampling.lookahead_yaw_error(),
        }
    }
}

macro_rules! delegate {
    ($self:ident.$method:ident($($arg:expr),*)) => {
        match &mut $self.0 {
            LocalExecution::Scan(follower) => NavPlanner::$method(follower, $($arg),*),
            LocalExecution::Sampling(follower) => NavPlanner::$method(follower, $($arg),*),
        }
    };
    (read $self:ident.$method:ident()) => {
        match &$self.0 {
            LocalExecution::Scan(follower) => NavPlanner::$method(follower),
            LocalExecution::Sampling(follower) => NavPlanner::$method(follower),
        }
    };
}

impl NavPlanner for SurfaceExecutor {
    fn set_control_period(&mut self, period: Duration) {
        delegate!(self.set_control_period(period))
    }

    fn refresh_execution_context(
        &mut self,
        applied: Twist2,
    ) -> Option<Arc<ActualExecutionContext>> {
        delegate!(self.refresh_execution_context(applied))
    }

    fn configure_boundary(&mut self, boundary: &[[f64; 2]]) {
        delegate!(self.configure_boundary(boundary))
    }

    fn configure_goal(
        &mut self,
        goal: &PoseStamped,
        options: GoalOptions,
    ) -> Result<(), RuntimeError> {
        delegate!(self.configure_goal(goal, options))
    }

    fn replace_path(&mut self, path: Path2D) {
        delegate!(self.replace_path(path))
    }

    fn replace_execution_path(
        &mut self,
        path: ExecutionPath,
        yaws: Option<Vec<f64>>,
        goal: PoseStamped,
    ) -> Result<(), RuntimeError> {
        delegate!(self.replace_execution_path(path, yaws, goal))
    }

    fn update_odometry(&mut self, odometry: PoseStamped) {
        delegate!(self.update_odometry(odometry))
    }

    fn update_costmap(&mut self, costmap: CostMap2D) {
        delegate!(self.update_costmap(costmap))
    }

    fn update_execution_map(&mut self, map: ExecutionMap) -> Result<(), RuntimeError> {
        delegate!(self.update_execution_map(map))
    }

    fn invalidate_execution_map(&mut self) {
        delegate!(self.invalidate_execution_map())
    }

    fn observe_applied_command(&mut self, command: Twist2) {
        delegate!(self.observe_applied_command(command))
    }

    fn observe_measured_motion(&mut self, motion: Option<MeasuredMotion>) {
        delegate!(self.observe_measured_motion(motion))
    }

    fn observe_registered_frame(&mut self, frame: Option<Arc<RegisteredFrame>>) {
        delegate!(self.observe_registered_frame(frame))
    }

    fn uses_route_progress(&self) -> bool {
        delegate!(read self.uses_route_progress())
    }

    fn suspended_route_failure(&mut self) -> Option<String> {
        delegate!(self.suspended_route_failure())
    }

    fn has_pending_replan(&self) -> bool {
        delegate!(read self.has_pending_replan())
    }

    fn execution_cycle_complete(&self) -> bool {
        delegate!(read self.execution_cycle_complete())
    }

    fn execution_wait_reason(&self) -> Option<String> {
        delegate!(read self.execution_wait_reason())
    }

    fn terminal_phase(&self) -> TerminalPhase {
        delegate!(read self.terminal_phase())
    }

    fn local_path_for_display(&self) -> Vec<[f64; 3]> {
        delegate!(read self.local_path_for_display())
    }

    fn step(&mut self) -> PlannerStep {
        delegate!(self.step())
    }

    fn step_at_goal(&mut self) -> Option<PlannerStep> {
        delegate!(self.step_at_goal())
    }

    fn stop(&mut self) {
        delegate!(self.stop())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn surface_config_selects_one_executor_with_sampling_default() {
        let config = SurfaceFollowerConfig::new(0.4, 0.15);
        assert!(matches!(
            SurfaceExecutor::new(config.clone()).unwrap().0,
            LocalExecution::Sampling(_)
        ));
        let mut config = config;
        config.local_planner = SurfaceLocalPlanner::Scan;
        assert!(matches!(
            SurfaceExecutor::new(config).unwrap().0,
            LocalExecution::Scan(_)
        ));
    }
}
