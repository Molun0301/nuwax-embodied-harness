//! Typed lifecycle and publication layer for [`crate::MovementManager`].
//!
//! The module consumes click, navigation velocity, teleoperation velocity, and
//! cancellation inputs on typed ports. Core events are published to typed goal,
//! waypoint, command velocity, and stop outputs. Transport binding remains the
//! responsibility of the process composition layer.

#![allow(clippy::module_name_repetitions)]

use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};

use serde::{Deserialize, Serialize};
use tokio::sync::{broadcast, oneshot};

use nuwax_nav_base::NavError;
use nuwax_nav_types_messages::{
    NavFrameId, Point3, PointStamped, Pose3, PoseStamped, Quaternion, Timestamp,
};

use crate::{CancelGoalPolicy, MovementEvent, MovementManager, MovementManagerConfig, Twist};

/// Default capacity for every typed port.
pub const DEFAULT_BROADCAST_CAPACITY: usize = 16;

/// Module configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MovementManagerModuleConfig {
    /// Core movement-manager configuration.
    pub manager_config: MovementManagerConfig,
    /// Capacity of each broadcast port.
    pub broadcast_capacity: usize,
    /// Robot identifier used by process composition.
    pub robot_id: String,
}

impl Default for MovementManagerModuleConfig {
    fn default() -> Self {
        Self {
            manager_config: MovementManagerConfig::default(),
            broadcast_capacity: DEFAULT_BROADCAST_CAPACITY,
            robot_id: "default_robot".to_string(),
        }
    }
}

/// High-level movement commands retained for typed-stop consumers.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum MovementCommand {
    /// Stop immediately.
    Stop,
    /// Stop while retaining a resumable goal.
    Pause,
    /// Resume a paused command.
    Resume,
}

impl MovementCommand {
    /// Return whether this command prevents motion.
    #[must_use]
    pub fn is_stop_like(&self) -> bool {
        matches!(self, Self::Stop | Self::Pause)
    }
}

#[derive(Clone)]
struct OutputPorts {
    stop_command: broadcast::Sender<MovementCommand>,
    stop_movement: broadcast::Sender<bool>,
    legacy_nan_pose: broadcast::Sender<PoseStamped>,
    nan_goal_point: broadcast::Sender<PointStamped>,
    goal: broadcast::Sender<PointStamped>,
    way_point: broadcast::Sender<PointStamped>,
    cmd_vel: broadcast::Sender<Twist>,
}

/// Movement manager with typed input and output ports.
pub struct MovementManagerModule {
    manager: Arc<Mutex<MovementManager>>,
    config: MovementManagerModuleConfig,
    running: bool,
    clicked_point_in: broadcast::Sender<PointStamped>,
    nav_cmd_vel_in: broadcast::Sender<Twist>,
    tele_cmd_vel_in: broadcast::Sender<Twist>,
    cancel_in: broadcast::Sender<bool>,
    outputs: OutputPorts,
    lifecycle_shutdown: Option<oneshot::Sender<()>>,
    lifecycle_thread: Option<std::thread::JoinHandle<()>>,
}

impl MovementManagerModule {
    /// Construct and validate a module.
    pub fn new(config: MovementManagerModuleConfig) -> Result<Self, NavError> {
        let manager = MovementManager::new(config.manager_config.clone()).map_err(|error| {
            NavError::InvalidInput {
                field: "manager_config",
                reason: error.to_string(),
            }
        })?;
        let capacity = config.broadcast_capacity.max(1);
        let (clicked_point_in, _) = broadcast::channel(capacity);
        let (nav_cmd_vel_in, _) = broadcast::channel(capacity);
        let (tele_cmd_vel_in, _) = broadcast::channel(capacity);
        let (cancel_in, _) = broadcast::channel(capacity);
        let (stop_command, _) = broadcast::channel(capacity);
        let (stop_movement, _) = broadcast::channel(capacity);
        let (legacy_nan_pose, _) = broadcast::channel(capacity);
        let (nan_goal_point, _) = broadcast::channel(capacity);
        let (goal, _) = broadcast::channel(capacity);
        let (way_point, _) = broadcast::channel(capacity);
        let (cmd_vel, _) = broadcast::channel(capacity);

        Ok(Self {
            manager: Arc::new(Mutex::new(manager)),
            config,
            running: false,
            clicked_point_in,
            nav_cmd_vel_in,
            tele_cmd_vel_in,
            cancel_in,
            outputs: OutputPorts {
                stop_command,
                stop_movement,
                legacy_nan_pose,
                nan_goal_point,
                goal,
                way_point,
                cmd_vel,
            },
            lifecycle_shutdown: None,
            lifecycle_thread: None,
        })
    }

    /// Robot identifier.
    #[must_use]
    pub fn robot_id(&self) -> &str {
        &self.config.robot_id
    }

    /// Clone the typed clicked-point input port.
    #[must_use]
    pub fn clicked_point_input(&self) -> broadcast::Sender<PointStamped> {
        self.clicked_point_in.clone()
    }

    /// Clone the typed navigation-velocity input port.
    #[must_use]
    pub fn nav_cmd_vel_input(&self) -> broadcast::Sender<Twist> {
        self.nav_cmd_vel_in.clone()
    }

    /// Clone the typed teleoperation-velocity input port.
    #[must_use]
    pub fn tele_cmd_vel_input(&self) -> broadcast::Sender<Twist> {
        self.tele_cmd_vel_in.clone()
    }

    /// Clone the cancellation input port. Only `true` requests cancellation.
    #[must_use]
    pub fn cancel_input(&self) -> broadcast::Sender<bool> {
        self.cancel_in.clone()
    }

    /// Clone the legacy typed-stop sender.
    #[must_use]
    pub fn stop_out(&self) -> broadcast::Sender<MovementCommand> {
        self.outputs.stop_command.clone()
    }

    /// Clone the legacy NaN pose sender.
    #[must_use]
    pub fn nan_goal_out(&self) -> broadcast::Sender<PoseStamped> {
        self.outputs.legacy_nan_pose.clone()
    }

    /// Subscribe to typed-stop commands.
    #[must_use]
    pub fn subscribe_stop(&self) -> broadcast::Receiver<MovementCommand> {
        self.outputs.stop_command.subscribe()
    }

    /// Subscribe to legacy NaN pose cancellation messages.
    #[must_use]
    pub fn subscribe_nan_goal(&self) -> broadcast::Receiver<PoseStamped> {
        self.outputs.legacy_nan_pose.subscribe()
    }

    /// Subscribe to boolean stop-movement messages.
    #[must_use]
    pub fn subscribe_stop_movement(&self) -> broadcast::Receiver<bool> {
        self.outputs.stop_movement.subscribe()
    }

    /// Subscribe to PointStamped cancellation messages.
    #[must_use]
    pub fn subscribe_nan_goal_point(&self) -> broadcast::Receiver<PointStamped> {
        self.outputs.nan_goal_point.subscribe()
    }

    /// Subscribe to goal publications.
    #[must_use]
    pub fn subscribe_goal(&self) -> broadcast::Receiver<PointStamped> {
        self.outputs.goal.subscribe()
    }

    /// Subscribe to waypoint publications.
    #[must_use]
    pub fn subscribe_way_point(&self) -> broadcast::Receiver<PointStamped> {
        self.outputs.way_point.subscribe()
    }

    /// Subscribe to command-velocity publications.
    #[must_use]
    pub fn subscribe_cmd_vel(&self) -> broadcast::Receiver<Twist> {
        self.outputs.cmd_vel.subscribe()
    }

    /// Return the configured cancellation policy.
    #[must_use]
    pub fn cancel_policy(&self) -> CancelGoalPolicy {
        self.manager
            .lock()
            .map(|manager| manager.config().cancel_goal_policy)
            .unwrap_or(self.config.manager_config.cancel_goal_policy)
    }

    fn build_nan_goal_stamped() -> PoseStamped {
        let nan = f64::NAN;
        PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: unix_timestamp_now(),
            pose: Pose3 {
                position: Point3 {
                    x: nan,
                    y: nan,
                    z: nan,
                },
                orientation: Quaternion {
                    x: nan,
                    y: nan,
                    z: nan,
                    w: nan,
                },
            },
        }
    }

    /// Publish an explicit cancellation request.
    pub fn trigger_cancel(
        &mut self,
        publish_stop: bool,
        publish_nan_goal: bool,
    ) -> Result<(), NavError> {
        if let Ok(mut manager) = self.manager.lock() {
            let _ = manager.cancel_navigation();
        }
        publish_cancel(&self.outputs, publish_stop, publish_nan_goal);
        Ok(())
    }

    /// Bind input receivers and start the event lifecycle.
    pub fn start(&mut self) -> Result<(), NavError> {
        if self.running {
            return Ok(());
        }

        let click_rx = self.clicked_point_in.subscribe();
        let nav_rx = self.nav_cmd_vel_in.subscribe();
        let teleop_rx = self.tele_cmd_vel_in.subscribe();
        let cancel_rx = self.cancel_in.subscribe();
        let manager = Arc::clone(&self.manager);
        let outputs = self.outputs.clone();
        let (shutdown_tx, shutdown_rx) = oneshot::channel();
        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .map_err(|error| {
                NavError::Internal(format!(
                    "failed to build MovementManager lifecycle runtime: {error}"
                ))
            })?;
        let thread = std::thread::Builder::new()
            .name("movement-manager".to_string())
            .spawn(move || {
                runtime.block_on(run_input_loop(
                    manager,
                    outputs,
                    click_rx,
                    nav_rx,
                    teleop_rx,
                    cancel_rx,
                    shutdown_rx,
                ));
            })
            .map_err(|error| {
                NavError::Internal(format!(
                    "failed to spawn MovementManager lifecycle: {error}"
                ))
            })?;

        self.lifecycle_shutdown = Some(shutdown_tx);
        self.lifecycle_thread = Some(thread);
        self.running = true;
        Ok(())
    }

    /// Stop the lifecycle and clear teleoperation takeover state.
    pub fn stop(&mut self) -> Result<(), NavError> {
        if let Some(shutdown) = self.lifecycle_shutdown.take() {
            let _ = shutdown.send(());
        }
        let lifecycle_panicked = self
            .lifecycle_thread
            .take()
            .is_some_and(|thread| thread.join().is_err());
        if self.running {
            let reset =
                MovementManager::new(self.config.manager_config.clone()).map_err(|error| {
                    NavError::Internal(format!("failed to reset MovementManager: {error}"))
                })?;
            if let Ok(mut manager) = self.manager.lock() {
                *manager = reset;
            }
        }
        self.running = false;
        if lifecycle_panicked {
            Err(NavError::Internal(
                "MovementManager lifecycle panicked".into(),
            ))
        } else {
            Ok(())
        }
    }

    /// Return whether the lifecycle is running.
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.running
    }
}

impl Drop for MovementManagerModule {
    fn drop(&mut self) {
        if let Some(shutdown) = self.lifecycle_shutdown.take() {
            let _ = shutdown.send(());
        }
        if let Some(thread) = self.lifecycle_thread.take() {
            let _ = thread.join();
        }
    }
}

#[allow(clippy::too_many_arguments)]
async fn run_input_loop(
    manager: Arc<Mutex<MovementManager>>,
    outputs: OutputPorts,
    mut click_rx: broadcast::Receiver<PointStamped>,
    mut nav_rx: broadcast::Receiver<Twist>,
    mut teleop_rx: broadcast::Receiver<Twist>,
    mut cancel_rx: broadcast::Receiver<bool>,
    mut shutdown_rx: oneshot::Receiver<()>,
) {
    loop {
        tokio::select! {
            _ = &mut shutdown_rx => break,
            received = click_rx.recv() => match received {
                Ok(point) => {
                    let events = manager.lock()
                        .map(|mut manager| manager.handle_click(&point))
                        .unwrap_or_default();
                    publish_events(&outputs, events);
                }
                Err(broadcast::error::RecvError::Lagged(_)) => {},
                Err(broadcast::error::RecvError::Closed) => break,
            },
            received = nav_rx.recv() => match received {
                Ok(command) => {
                    let events = manager.lock()
                        .map(|mut manager| manager.handle_nav_cmd_vel(command))
                        .unwrap_or_default();
                    publish_events(&outputs, events);
                }
                Err(broadcast::error::RecvError::Lagged(_)) => {},
                Err(broadcast::error::RecvError::Closed) => break,
            },
            received = teleop_rx.recv() => match received {
                Ok(command) => {
                    let events = manager.lock()
                        .map(|mut manager| manager.handle_tele_cmd_vel(command))
                        .unwrap_or_default();
                    publish_events(&outputs, events);
                }
                Err(broadcast::error::RecvError::Lagged(_)) => {},
                Err(broadcast::error::RecvError::Closed) => break,
            },
            received = cancel_rx.recv() => match received {
                Ok(true) => {
                    let events = manager.lock()
                        .map(|mut manager| manager.cancel_navigation())
                        .unwrap_or_default();
                    publish_events(&outputs, events);
                }
                Ok(false) | Err(broadcast::error::RecvError::Lagged(_)) => {},
                Err(broadcast::error::RecvError::Closed) => break,
            },
        }
    }
}

fn publish_events(outputs: &OutputPorts, events: Vec<MovementEvent>) {
    for event in events {
        match event {
            MovementEvent::ClickAccepted(point) => {
                let _ = outputs.way_point.send(point.clone());
                let _ = outputs.goal.send(point);
            }
            MovementEvent::ClickRejected { .. } => {}
            MovementEvent::CancelNavigation {
                publish_stop,
                publish_nan_goal,
            } => publish_cancel(outputs, publish_stop, publish_nan_goal),
            MovementEvent::CmdVel(command) => {
                let _ = outputs.cmd_vel.send(command);
            }
        }
    }
}

fn publish_cancel(outputs: &OutputPorts, publish_stop: bool, publish_nan_goal: bool) {
    if publish_stop {
        let _ = outputs.stop_command.send(MovementCommand::Stop);
        let _ = outputs.stop_movement.send(true);
    }
    if publish_nan_goal {
        let point = build_nan_goal_point();
        let _ = outputs.nan_goal_point.send(point.clone());
        let _ = outputs.way_point.send(point.clone());
        let _ = outputs.goal.send(point);
        let _ = outputs
            .legacy_nan_pose
            .send(MovementManagerModule::build_nan_goal_stamped());
    }
}

fn build_nan_goal_point() -> PointStamped {
    PointStamped::new(
        NavFrameId::Map,
        unix_timestamp_now(),
        Point3 {
            x: f64::NAN,
            y: f64::NAN,
            z: f64::NAN,
        },
    )
}

fn unix_timestamp_now() -> Timestamp {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    let nanos = duration.as_nanos().min(i64::MAX as u128) as i64;
    Timestamp {
        nanos,
        clock: nuwax_nav_types_messages::ClockKind::Unix,
    }
}
