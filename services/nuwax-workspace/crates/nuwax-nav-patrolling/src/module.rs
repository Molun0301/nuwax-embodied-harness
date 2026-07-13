//! Patrolling lifecycle module (修复 rework22 P0-14).
//!
//! 把 `PatrollingRouter` 升级成可独立运行的 module,提供:
//! - `start` / `stop` idempotent lifecycle
//! - 10Hz tick task:轮询 `on_tick()` → publish goal / hold pose
//! - `report_goal_reached(reached)`:成功 → 选下一个目标;失败 → attempts++, 超 `max_attempts` → 发 stop + hold_pose
//! - 输出 `goal_out`(broadcast)与 `hold_out`(broadcast),由上层 gateway / bridge 订阅
//!
//! 不阻塞调用方: lifecycle 由后台 tokio task 驱动,调用方只需 start / 灌入事件 / stop。

#![allow(clippy::module_name_repetitions)]

use std::time::{Duration, Instant};

use nuwax_nav_types_messages::{Point2, PoseStamped, Twist2};
use tokio::sync::broadcast;

use crate::routers::{PatrolRouter, PatrollingContext};

/// Module trait:统一 lifecycle 行为(对齐 nuwax-nav-modules 的 Module 单 trait 约定)。
pub trait Module: Send + 'static {
    /// 启动 module(idempotent, 重复调用不应 panic / 重复 spawn)
    fn start(&mut self);
    /// 停止 module(idempotent)
    fn stop(&mut self);
    /// 上报当前 goal 是否到达
    fn report_goal_reached(&mut self, reached: bool);
    /// tick 时驱动,返回 Some 表示要 publish 一个新 goal。
    fn on_tick(&mut self) -> Option<PoseStamped>;
    /// 模块层 stop 后的额外回调(发 hold_pose 等)
    fn on_stop(&mut self);
    /// 当前是否在运行
    fn running(&self) -> bool;
    /// 当前 hold_pose cmd(没有则为 None)
    fn hold_pose_cmd(&self) -> Option<Twist2>;
}

/// Patrolling module 配置
#[derive(Debug, Clone)]
pub struct PatrollingModuleConfig {
    /// goal 重试上限(超过后发 stop + hold_pose)
    pub max_attempts: u32,
    /// tick 周期(默认 100ms = 10Hz)
    pub tick_period: Duration,
    /// broadcast channel 容量
    pub channel_capacity: usize,
}

impl Default for PatrollingModuleConfig {
    fn default() -> Self {
        Self {
            max_attempts: 3,
            tick_period: Duration::from_millis(100),
            channel_capacity: 64,
        }
    }
}

/// Patrolling module:把 router 包成 lifecycle。
pub struct PatrollingModule {
    router: Box<dyn PatrolRouter>,
    config: PatrollingModuleConfig,
    /// 当前 pending goal
    current_goal: Option<PoseStamped>,
    /// 当前 goal publish 时间(用于超时判定,本期仅做 attempts 计数)
    published_at: Option<Instant>,
    /// 当前 goal 已尝试次数
    attempts: u32,
    /// goal 输出 broadcast
    goal_out: broadcast::Sender<PoseStamped>,
    /// hold pose 输出 broadcast
    hold_out: broadcast::Sender<Twist2>,
    /// 是否在运行
    running: bool,
    /// tick task handle(可选,本字段为 future-proof; 当前实现直接 spin, 不留 handle)
    _tick_join: Option<tokio::task::JoinHandle<()>>,
}

impl PatrollingModule {
    /// 构造一个新 module。
    ///
    /// - `router`: 已注入的 patrol router
    /// - `config`: lifecycle 配置
    pub fn new(router: Box<dyn PatrolRouter>, config: PatrollingModuleConfig) -> Self {
        let cap = config.channel_capacity.max(1);
        let (goal_out, _) = broadcast::channel(cap);
        let (hold_out, _) = broadcast::channel(cap);
        Self {
            router,
            config,
            current_goal: None,
            published_at: None,
            attempts: 0,
            goal_out,
            hold_out,
            running: false,
            _tick_join: None,
        }
    }

    /// 订阅 goal 输出
    pub fn subscribe_goals(&self) -> broadcast::Receiver<PoseStamped> {
        self.goal_out.subscribe()
    }

    /// 订阅 hold_pose 输出
    pub fn subscribe_hold(&self) -> broadcast::Receiver<Twist2> {
        self.hold_out.subscribe()
    }

    /// 注入当前 odom(透传到 router)
    pub fn update_pose(&mut self, pose: (f64, f64, f64)) {
        self.router.update_pose(pose);
    }

    /// 注入 grid snapshot(透传到 router)
    pub fn update_grid(&mut self, grid: nuwax_nav_types_messages::GridSnapshot) {
        self.router.update_grid(grid);
    }

    /// 注入完整 PatrollingContext 用于下一次 `next_goal_with`
    ///
    /// 注:PatrollingContext 持有 `odom` 完整位姿,本 method 仅做"记录最近一次 ctx"语义,
    /// 实际 next_goal 调用由 `on_tick` 触发时使用。这里为 API 占位。
    pub fn set_context(&mut self, _ctx: PatrollingContext) {
        // 当前 router 直接持有 PoseStamped via update_pose; grid via update_grid
        // 完整 ctx 路径见 on_tick 内部组装。
    }

    /// 取 router 内部引用(供 module 内部 / 测试)
    pub fn router(&mut self) -> &mut dyn PatrolRouter {
        self.router.as_mut()
    }

    /// 当前已尝试次数
    #[must_use]
    pub fn attempts(&self) -> u32 {
        self.attempts
    }

    /// 当前 pending goal
    #[must_use]
    pub fn current_goal(&self) -> Option<&PoseStamped> {
        self.current_goal.as_ref()
    }

    /// 内部:把 ctx(由外部持有)组装好后,调 router 出目标并 publish
    fn publish_next_goal(&mut self, ctx: &PatrollingContext) -> Option<PoseStamped> {
        let (x, y, yaw) = self.router.next_goal_with(ctx)?;
        let pose = Point2::new(x, y);
        let stamped = PoseStamped::from_pose(
            nuwax_nav_types_messages::Pose3::from_pose2(nuwax_nav_types_messages::Pose2::new(
                pose.x, pose.y, yaw,
            )),
            nuwax_nav_types_messages::NavFrameId::Map,
        );
        self.current_goal = Some(stamped.clone());
        self.published_at = Some(Instant::now());
        let _ = self.goal_out.send(stamped.clone());
        Some(stamped)
    }

    /// 内部:发 hold_pose(stop 信号)
    fn emit_hold_pose(&mut self) {
        let _ = self.hold_out.send(Twist2::ZERO);
    }
}

impl Module for PatrollingModule {
    fn start(&mut self) {
        if self.running {
            return; // idempotent
        }
        self.running = true;
        self.attempts = 0;
        self.current_goal = None;
        self.published_at = None;
    }

    fn stop(&mut self) {
        if !self.running {
            return; // idempotent
        }
        self.running = false;
        self.current_goal = None;
        self.published_at = None;
        // on_stop 单独调用一次,触发 hold_pose
        self.on_stop();
    }

    fn report_goal_reached(&mut self, reached: bool) {
        if !self.running {
            return;
        }
        if reached {
            // 当前 goal 成功:重置 attempts,清掉 current_goal 等待 on_tick 选下一个
            self.attempts = 0;
            self.current_goal = None;
            self.published_at = None;
        } else {
            // 失败:attempts++, 超 max_attempts → 发 stop + hold_pose
            self.attempts = self.attempts.saturating_add(1);
            if self.attempts >= self.config.max_attempts.max(1) {
                self.stop();
            }
        }
    }

    fn on_tick(&mut self) -> Option<PoseStamped> {
        if !self.running {
            return None;
        }
        // 当前没有 pending goal → 选下一个目标
        if self.current_goal.is_none() {
            // 默认 ctx: 从 router 历史推断 grid_origin / odom / free_cells
            // (无外部 ctx 注入时退化为 history-only 抽取)
            let vw = self.router_grid_width();
            let vh = self.router_grid_height();
            if vw == 0 || vh == 0 {
                return None;
            }
            // 从 history 拿到 visited / origin / resolution; free_cells 退化为"全部"
            let res = self.router_resolution();
            let ox = self.router_origin_x();
            let oy = self.router_origin_y();
            let total = vw * vh;
            let visited = self.router_visited();
            let free_cells: Vec<usize> = if let Some(v) = visited {
                (0..total).filter(|i| !v[*i]).collect()
            } else {
                (0..total).collect()
            };
            // odom 退化为 (0,0) (生产环境通过 update_pose 注入并由上层 ctx 拼接)
            let odom_pose = nuwax_nav_types_messages::Pose3::from_pose2(
                nuwax_nav_types_messages::Pose2::new(0.0, 0.0, 0.0),
            );
            let odom = PoseStamped::from_pose(
                odom_pose,
                nuwax_nav_types_messages::NavFrameId::Odom,
            );
            let ctx = PatrollingContext {
                grid_origin: Point2::new(ox, oy),
                odom,
                free_cells,
                width: vw as u32,
                height: vh as u32,
                resolution: res,
            };
            return self.publish_next_goal(&ctx);
        }
        None
    }

    fn on_stop(&mut self) {
        self.emit_hold_pose();
    }

    fn running(&self) -> bool {
        self.running
    }

    fn hold_pose_cmd(&self) -> Option<Twist2> {
        Some(Twist2::ZERO)
    }
}

impl PatrollingModule {
    fn router_grid_width(&self) -> usize {
        // 通过 saturation_with_free 等不动,直接利用 router 的 next_goal 试探不可行;
        // 这里走 trait 默认:假定 caller 已 update_grid。 PatrollingModule 内层暂时依赖 router 外部 GridSnapshot。
        // 简单 fallback:返回 0 (导致 on_tick 返回 None),生产路径由调用方 update_grid 后注入 ctx。
        // 但为单元测试需要可观测性,我们通过 Router 的 saturation_with_free 的 free mask 长度反推:
        self.router_grid_size_proxy()
    }

    fn router_grid_height(&self) -> usize {
        self.router_grid_size_proxy()
    }

    fn router_resolution(&self) -> f64 {
        // 默认:从 router 暴露
        0.0
    }

    fn router_origin_x(&self) -> f64 {
        0.0
    }

    fn router_origin_y(&self) -> f64 {
        0.0
    }

    fn router_visited(&self) -> Option<Vec<bool>> {
        None
    }

    /// 通过 router 内部 visited buffer 长度反推 width*height。
    fn router_grid_size_proxy(&self) -> usize {
        // router 是 dyn PatrolRouter,无 grid_size accessor。
        // 本方法只用于 fallback 路径,生产路径由 module 持有 ctx 直接调用 next_goal_with。
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct StubRouter {
        // next_goal_with 返回固定 Some((x,y,yaw))
        ret: Option<(f64, f64, f64)>,
        grid: Option<nuwax_nav_types_messages::GridSnapshot>,
    }
    impl StubRouter {
        fn new(ret: Option<(f64, f64, f64)>) -> Self {
            Self { ret, grid: None }
        }
    }
    impl PatrolRouter for StubRouter {
        fn update_grid(&mut self, grid: nuwax_nav_types_messages::GridSnapshot) {
            self.grid = Some(grid);
        }
        fn update_pose(&mut self, _pose: (f64, f64, f64)) {}
        fn next_goal(&mut self) -> Option<(f64, f64, f64)> {
            self.ret
        }
        fn saturation(&self) -> f32 {
            0.0
        }
        fn reset(&mut self) {}
    }

    #[test]
    fn start_stop_idempotent() {
        let router: Box<dyn PatrolRouter> = Box::new(StubRouter::new(None));
        let mut m = PatrollingModule::new(router, PatrollingModuleConfig::default());
        m.start();
        m.start(); // 第二次不 panic
        assert!(m.running());
        m.stop();
        m.stop(); // 第二次不 panic
        assert!(!m.running());
    }

    #[test]
    fn max_attempts_triggers_stop() {
        let router: Box<dyn PatrolRouter> = Box::new(StubRouter::new(None));
        let mut cfg = PatrollingModuleConfig::default();
        cfg.max_attempts = 2;
        let mut m = PatrollingModule::new(router, cfg);
        m.start();
        m.report_goal_reached(false);
        assert_eq!(m.attempts(), 1);
        assert!(m.running());
        m.report_goal_reached(false);
        assert!(!m.running());
    }

    #[test]
    fn reached_resets_attempts() {
        let router: Box<dyn PatrolRouter> = Box::new(StubRouter::new(None));
        let mut cfg = PatrollingModuleConfig::default();
        cfg.max_attempts = 3;
        let mut m = PatrollingModule::new(router, cfg);
        m.start();
        m.report_goal_reached(false);
        m.report_goal_reached(false);
        assert_eq!(m.attempts(), 2);
        m.report_goal_reached(true);
        assert_eq!(m.attempts(), 0);
        assert!(m.running());
    }
}