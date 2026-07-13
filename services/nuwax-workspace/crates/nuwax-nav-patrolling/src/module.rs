//! Lifecycle wrapper for patrol routers.
//!
//! The module retains the latest grid/odometry context, runs a periodic patrol
//! ticker, validates router goals against the free-cell set, and publishes hold
//! commands when patrol is stopped.

#![allow(clippy::module_name_repetitions)]

use std::ops::{Deref, DerefMut};
use std::sync::{mpsc as std_mpsc, Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use nuwax_nav_types_messages::{
    GridSnapshot, NavFrameId, Point2, Pose2, Pose3, PoseStamped, Twist2,
};
use tokio::sync::broadcast;

use crate::routers::{PatrolRouter, PatrollingContext};

/// Patrol lifecycle interface.
pub trait Module: Send + 'static {
    /// Start the patrol ticker. Repeated calls are idempotent.
    fn start(&mut self);
    /// Stop patrolling and publish hold outputs. Repeated calls are idempotent.
    fn stop(&mut self);
    /// Report whether the active goal was reached.
    fn report_goal_reached(&mut self, reached: bool);
    /// Report a terminal planning failure for the active patrol goal.
    fn report_goal_failed(&mut self);
    /// Drive one patrol iteration synchronously.
    fn on_tick(&mut self) -> Option<PoseStamped>;
    /// Publish the stop/hold outputs.
    fn on_stop(&mut self);
    /// Return whether patrol is running.
    fn running(&self) -> bool;
    /// Return the hold command used by the module.
    fn hold_pose_cmd(&self) -> Option<Twist2>;
}

/// Patrol lifecycle configuration.
#[derive(Debug, Clone)]
pub struct PatrollingModuleConfig {
    /// Maximum consecutive failed goals before stopping.
    pub max_attempts: u32,
    /// Period between patrol attempts.
    pub tick_period: Duration,
    /// Capacity of goal and hold output ports.
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

struct PatrollingState {
    router: Box<dyn PatrolRouter>,
    context: Option<PatrollingContext>,
    latest_grid: Option<GridSnapshot>,
    latest_odom: Option<PoseStamped>,
    current_goal: Option<PoseStamped>,
    published_at: Option<Instant>,
    attempts: u32,
    running: bool,
}

/// Mutable guard for the module's injected patrol router.
pub struct PatrolRouterGuard<'a> {
    guard: MutexGuard<'a, PatrollingState>,
}

impl Deref for PatrolRouterGuard<'_> {
    type Target = dyn PatrolRouter;

    fn deref(&self) -> &Self::Target {
        self.guard.router.as_ref()
    }
}

impl DerefMut for PatrolRouterGuard<'_> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.guard.router.as_mut()
    }
}

/// Patrol module with retained context and an active lifecycle ticker.
pub struct PatrollingModule {
    state: Arc<Mutex<PatrollingState>>,
    config: PatrollingModuleConfig,
    goal_out: broadcast::Sender<PoseStamped>,
    hold_out: broadcast::Sender<Twist2>,
    tick_shutdown: Option<std_mpsc::Sender<()>>,
    tick_thread: Option<std::thread::JoinHandle<()>>,
}

impl PatrollingModule {
    /// Construct a module around an injected router.
    #[must_use]
    pub fn new(router: Box<dyn PatrolRouter>, config: PatrollingModuleConfig) -> Self {
        let capacity = config.channel_capacity.max(1);
        let (goal_out, _) = broadcast::channel(capacity);
        let (hold_out, _) = broadcast::channel(capacity);
        Self {
            state: Arc::new(Mutex::new(PatrollingState {
                router,
                context: None,
                latest_grid: None,
                latest_odom: None,
                current_goal: None,
                published_at: None,
                attempts: 0,
                running: false,
            })),
            config,
            goal_out,
            hold_out,
            tick_shutdown: None,
            tick_thread: None,
        }
    }

    /// Subscribe to navigation-goal publications.
    #[must_use]
    pub fn subscribe_goals(&self) -> broadcast::Receiver<PoseStamped> {
        self.goal_out.subscribe()
    }

    /// Subscribe to zero-velocity hold publications.
    #[must_use]
    pub fn subscribe_hold(&self) -> broadcast::Receiver<Twist2> {
        self.hold_out.subscribe()
    }

    /// Update the current pose from planar coordinates.
    pub fn update_pose(&mut self, pose: (f64, f64, f64)) {
        self.update_odometry(PoseStamped::from_pose(
            Pose3::from_pose2(Pose2::new(pose.0, pose.1, pose.2)),
            NavFrameId::Odom,
        ));
    }

    /// Update the current stamped odometry pose.
    pub fn update_odometry(&mut self, odom: PoseStamped) {
        if !valid_pose(&odom) {
            if let Ok(mut state) = self.state.lock() {
                state.latest_odom = None;
                state.context = None;
            }
            return;
        }
        let planar = odom.pose.to_pose2();
        if let Ok(mut state) = self.state.lock() {
            state
                .router
                .update_pose((planar.position.x, planar.position.y, planar.yaw));
            state.latest_odom = Some(odom);
            refresh_context(&mut state);
        }
    }

    /// Update the occupancy grid and rebuild the retained context when odometry exists.
    pub fn update_grid(&mut self, grid: GridSnapshot) {
        if let Ok(mut state) = self.state.lock() {
            state.router.update_grid(grid.clone());
            state.latest_grid = Some(grid);
            refresh_context(&mut state);
        }
    }

    /// Retain a complete patrol context for subsequent ticks.
    pub fn set_context(&mut self, context: PatrollingContext) {
        if !valid_context(&context) {
            if let Ok(mut state) = self.state.lock() {
                state.context = None;
            }
            return;
        }
        let planar = context.odom.pose.to_pose2();
        if let Ok(mut state) = self.state.lock() {
            state
                .router
                .update_pose((planar.position.x, planar.position.y, planar.yaw));
            state.latest_odom = Some(context.odom.clone());
            state.context = Some(context);
        }
    }

    /// Borrow the injected router while holding the module state lock.
    pub fn router(&mut self) -> PatrolRouterGuard<'_> {
        PatrolRouterGuard {
            guard: self.state.lock().unwrap_or_else(|error| error.into_inner()),
        }
    }

    /// Return the number of consecutive failed goals.
    #[must_use]
    pub fn attempts(&self) -> u32 {
        self.state.lock().map(|state| state.attempts).unwrap_or(0)
    }

    /// Return a snapshot of the active goal.
    #[must_use]
    pub fn current_goal(&self) -> Option<PoseStamped> {
        self.state
            .lock()
            .ok()
            .and_then(|state| state.current_goal.clone())
    }

    fn publish_next_goal(&mut self, context: &PatrollingContext) -> Option<PoseStamped> {
        publish_next_goal(&self.state, &self.goal_out, context)
    }

    fn emit_hold_pose(&mut self) {
        let _ = self.hold_out.send(Twist2::ZERO);
    }
}

impl Module for PatrollingModule {
    fn start(&mut self) {
        {
            let mut state = self.state.lock().unwrap_or_else(|error| error.into_inner());
            if state.running {
                return;
            }
            let latest_grid = state.latest_grid.clone();
            let latest_odom = state.latest_odom.clone();
            state.router.reset();
            if let Some(grid) = latest_grid {
                state.router.update_grid(grid);
            }
            if let Some(odom) = latest_odom {
                let planar = odom.pose.to_pose2();
                state
                    .router
                    .update_pose((planar.position.x, planar.position.y, planar.yaw));
            }
            state.running = true;
            state.attempts = 0;
            state.current_goal = None;
            state.published_at = None;
        }

        let state = Arc::clone(&self.state);
        let goal_out = self.goal_out.clone();
        let period = self.config.tick_period.max(Duration::from_millis(1));
        let (shutdown_tx, shutdown_rx) = std_mpsc::channel();
        let thread = std::thread::Builder::new()
            .name("patrolling-module".to_string())
            .spawn(move || loop {
                tick_and_publish(&state, &goal_out);
                match shutdown_rx.recv_timeout(period) {
                    Ok(()) | Err(std_mpsc::RecvTimeoutError::Disconnected) => break,
                    Err(std_mpsc::RecvTimeoutError::Timeout) => {}
                }
            });

        match thread {
            Ok(thread) => {
                self.tick_shutdown = Some(shutdown_tx);
                self.tick_thread = Some(thread);
            }
            Err(_) => {
                if let Ok(mut state) = self.state.lock() {
                    state.running = false;
                }
            }
        }
    }

    fn stop(&mut self) {
        let was_running = {
            let mut state = self.state.lock().unwrap_or_else(|error| error.into_inner());
            let was_running = state.running;
            state.running = false;
            state.current_goal = None;
            state.published_at = None;
            was_running
        };
        if !was_running {
            return;
        }

        if let Some(shutdown) = self.tick_shutdown.take() {
            let _ = shutdown.send(());
        }
        if let Some(thread) = self.tick_thread.take() {
            let _ = thread.join();
        }
        self.on_stop();
    }

    fn report_goal_reached(&mut self, reached: bool) {
        if !reached {
            return;
        }
        let mut state = self.state.lock().unwrap_or_else(|error| error.into_inner());
        if !state.running {
            return;
        }
        state.current_goal = None;
        state.published_at = None;
        state.attempts = 0;
    }

    fn report_goal_failed(&mut self) {
        let should_stop = {
            let mut state = self.state.lock().unwrap_or_else(|error| error.into_inner());
            if !state.running || state.current_goal.is_none() {
                return;
            }
            state.current_goal = None;
            state.published_at = None;
            state.attempts = state.attempts.saturating_add(1);
            state.attempts >= self.config.max_attempts.max(1)
        };
        if should_stop {
            self.stop();
        }
    }

    fn on_tick(&mut self) -> Option<PoseStamped> {
        let context = {
            let state = self.state.lock().ok()?;
            if !state.running || state.current_goal.is_some() {
                return None;
            }
            state.context.clone()?
        };
        self.publish_next_goal(&context)
    }

    fn on_stop(&mut self) {
        self.emit_hold_pose();
        let latest_odom = self
            .state
            .lock()
            .ok()
            .and_then(|state| state.latest_odom.clone());
        if let Some(hold_goal) = latest_odom {
            let _ = self.goal_out.send(hold_goal);
        }
    }

    fn running(&self) -> bool {
        self.state
            .lock()
            .map(|state| state.running)
            .unwrap_or(false)
    }

    fn hold_pose_cmd(&self) -> Option<Twist2> {
        Some(Twist2::ZERO)
    }
}

impl Drop for PatrollingModule {
    fn drop(&mut self) {
        if let Ok(mut state) = self.state.lock() {
            state.running = false;
        }
        if let Some(shutdown) = self.tick_shutdown.take() {
            let _ = shutdown.send(());
        }
        if let Some(thread) = self.tick_thread.take() {
            let _ = thread.join();
        }
    }
}

fn refresh_context(state: &mut PatrollingState) {
    let (Some(grid), Some(odom)) = (&state.latest_grid, &state.latest_odom) else {
        return;
    };
    let total = (grid.width as usize).saturating_mul(grid.height as usize);
    if total == 0
        || grid.data.len() < total
        || !(grid.resolution.is_finite() && grid.resolution > 0.0)
    {
        state.context = None;
        return;
    }
    let free_cells = grid
        .data
        .iter()
        .take(total)
        .enumerate()
        .filter_map(|(index, occupancy)| (*occupancy == 0).then_some(index))
        .collect();
    let context = PatrollingContext {
        grid_origin: Point2::new(grid.origin_x, grid.origin_y),
        odom: odom.clone(),
        free_cells,
        width: grid.width,
        height: grid.height,
        resolution: grid.resolution,
    };
    state.context = valid_context(&context).then_some(context);
}

fn valid_context(context: &PatrollingContext) -> bool {
    let total = (context.width as usize).saturating_mul(context.height as usize);
    total > 0
        && context.resolution.is_finite()
        && context.resolution > 0.0
        && context.grid_origin.x.is_finite()
        && context.grid_origin.y.is_finite()
        && valid_pose(&context.odom)
        && context.free_cells.iter().all(|index| *index < total)
}

fn valid_pose(pose: &PoseStamped) -> bool {
    pose.pose.position.x.is_finite()
        && pose.pose.position.y.is_finite()
        && pose.pose.position.z.is_finite()
        && pose.pose.orientation.x.is_finite()
        && pose.pose.orientation.y.is_finite()
        && pose.pose.orientation.z.is_finite()
        && pose.pose.orientation.w.is_finite()
}

fn tick_and_publish(
    state: &Arc<Mutex<PatrollingState>>,
    goal_out: &broadcast::Sender<PoseStamped>,
) -> Option<PoseStamped> {
    let context = {
        let state = state.lock().ok()?;
        if !state.running || state.current_goal.is_some() {
            return None;
        }
        state.context.clone()?
    };
    publish_next_goal(state, goal_out, &context)
}

fn publish_next_goal(
    state: &Arc<Mutex<PatrollingState>>,
    goal_out: &broadcast::Sender<PoseStamped>,
    context: &PatrollingContext,
) -> Option<PoseStamped> {
    let mut state = state.lock().ok()?;
    if !state.running || state.current_goal.is_some() {
        return None;
    }
    let (x, y, yaw) = state.router.next_goal_with(context)?;
    if !goal_is_reachable(context, x, y, yaw) {
        return None;
    }
    let goal = PoseStamped {
        frame_id: NavFrameId::Map,
        stamp: context.odom.stamp,
        pose: Pose3::from_pose2(Pose2::new(x, y, yaw)),
    };
    state.current_goal = Some(goal.clone());
    state.published_at = Some(Instant::now());
    let _ = goal_out.send(goal.clone());
    Some(goal)
}

fn goal_is_reachable(context: &PatrollingContext, x: f64, y: f64, yaw: f64) -> bool {
    if !(x.is_finite() && y.is_finite() && yaw.is_finite()) {
        return false;
    }
    let column = ((x - context.grid_origin.x) / context.resolution).floor();
    let row = ((y - context.grid_origin.y) / context.resolution).floor();
    if column < 0.0
        || row < 0.0
        || column >= f64::from(context.width)
        || row >= f64::from(context.height)
    {
        return false;
    }
    let index = row as usize * context.width as usize + column as usize;
    context.free_cells.contains(&index)
}
