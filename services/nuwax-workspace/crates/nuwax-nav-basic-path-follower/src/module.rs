//! Typed lifecycle wrapper for [`crate::BasicPathFollowerCore`].
//!
//! The module owns typed in-process ports for path, odometry, stop, command velocity,
//! and goal-reached messages. `start` binds the input receivers to a periodic control
//! task; transport adapters may forward decoded messages into these ports without
//! coupling the controller to a specific transport backend.

#![allow(clippy::module_name_repetitions)]

use std::sync::{
    atomic::{AtomicU64, Ordering},
    Arc, Mutex,
};
use std::time::Duration;

use serde::{Deserialize, Serialize};
use tokio::sync::{broadcast, mpsc, watch};
use tokio::task::JoinHandle;

use nuwax_nav_base::NavError;
use nuwax_nav_types_messages::{Odometry, Path2D, Point2 as MsgPoint2, PoseStamped, Twist2};
use nuwax_numeric::Point2 as NumPoint2;

use crate::BasicPathFollowerConfig;

/// Default controller frequency.
pub const DEFAULT_CONTROL_RATE_HZ: u32 = 10;

/// Default controller period.
pub const DEFAULT_TICK_PERIOD: Duration = Duration::from_millis(100);

const PORT_CAPACITY: usize = 16;

/// Module configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BasicPathFollowerModuleConfig {
    /// Robot identifier used by the process-level composition layer.
    pub robot_id: String,
    /// Control-loop frequency in Hz.
    pub control_rate_hz: u32,
    /// Command-velocity topic metadata for a transport adapter.
    pub cmd_vel_topic: String,
    /// Path topic metadata for a transport adapter.
    pub path_topic: String,
    /// Odometry topic metadata for a transport adapter.
    pub odom_topic: String,
    /// Stop topic metadata for a transport adapter.
    pub stop_topic: String,
    /// Controller algorithm configuration.
    pub core_config: BasicPathFollowerConfig,
}

impl Default for BasicPathFollowerModuleConfig {
    fn default() -> Self {
        Self {
            robot_id: "default_robot".to_string(),
            control_rate_hz: DEFAULT_CONTROL_RATE_HZ,
            cmd_vel_topic: "/nav_cmd_vel".to_string(),
            path_topic: "/plan_path".to_string(),
            odom_topic: "/odom".to_string(),
            stop_topic: "/stop_movement".to_string(),
            core_config: BasicPathFollowerConfig::default(),
        }
    }
}

/// Lifecycle and synchronous controller interface.
pub trait Module: Send + 'static {
    /// Start the input consumers and controller task.
    fn start(&mut self) -> Result<(), NavError>;
    /// Stop the task and publish a zero command.
    fn stop(&mut self) -> Result<(), NavError>;
    /// Inject a path synchronously.
    fn on_path(&mut self, path: Path2D);
    /// Inject a stamped pose synchronously.
    fn on_odom(&mut self, odom: PoseStamped);
    /// Return the most recently published velocity.
    fn cmd_vel(&self) -> Option<Twist2>;
    /// Return whether the most recent controller result reached the goal.
    fn goal_reached(&self) -> bool;
    /// Subscribe to command-velocity publications.
    fn subscribe_cmd_vel(&self) -> mpsc::Receiver<Twist2>;
}

#[derive(Default)]
struct SharedInputs {
    path: Option<Path2D>,
    odom: Option<PoseStamped>,
}

#[derive(Default)]
struct SharedOutputs {
    cmd_vel: Option<Twist2>,
    goal_reached: bool,
}

/// Path-follower module with typed lifecycle ports.
pub struct BasicPathFollowerModule {
    core_config: BasicPathFollowerConfig,
    config: BasicPathFollowerModuleConfig,
    running: bool,
    lifecycle_generation: Arc<AtomicU64>,
    control_guard: Arc<Mutex<()>>,
    inputs: Arc<Mutex<SharedInputs>>,
    outputs: Arc<Mutex<SharedOutputs>>,
    path_in: broadcast::Sender<Path2D>,
    odometry_in: broadcast::Sender<Odometry>,
    stop_in: broadcast::Sender<bool>,
    cmd_vel_tx: watch::Sender<Twist2>,
    goal_reached_tx: broadcast::Sender<bool>,
    lifecycle_handle: Option<JoinHandle<()>>,
}

impl BasicPathFollowerModule {
    /// Construct and validate a module.
    pub fn new(config: BasicPathFollowerModuleConfig) -> Result<Self, NavError> {
        if config.control_rate_hz == 0 {
            return Err(NavError::InvalidInput {
                field: "control_rate_hz",
                reason: "must be > 0".into(),
            });
        }
        config
            .core_config
            .validate()
            .map_err(|error| NavError::InvalidInput {
                field: "core_config",
                reason: error.to_string(),
            })?;

        let (path_in, _) = broadcast::channel(PORT_CAPACITY);
        let (odometry_in, _) = broadcast::channel(PORT_CAPACITY);
        let (stop_in, _) = broadcast::channel(PORT_CAPACITY);
        let (cmd_vel_tx, _) = watch::channel(Twist2::ZERO);
        let (goal_reached_tx, _) = broadcast::channel(PORT_CAPACITY);

        Ok(Self {
            core_config: config.core_config.clone(),
            config,
            running: false,
            lifecycle_generation: Arc::new(AtomicU64::new(0)),
            control_guard: Arc::new(Mutex::new(())),
            inputs: Arc::new(Mutex::new(SharedInputs::default())),
            outputs: Arc::new(Mutex::new(SharedOutputs::default())),
            path_in,
            odometry_in,
            stop_in,
            cmd_vel_tx,
            goal_reached_tx,
            lifecycle_handle: None,
        })
    }

    /// Command-velocity topic metadata.
    #[must_use]
    pub fn cmd_vel_topic(&self) -> &str {
        &self.config.cmd_vel_topic
    }

    /// Robot identifier.
    #[must_use]
    pub fn robot_id(&self) -> &str {
        &self.config.robot_id
    }

    /// Whether the controller lifecycle task is owned by this module.
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.running
    }

    /// Clone the typed path input port.
    #[must_use]
    pub fn path_input(&self) -> broadcast::Sender<Path2D> {
        self.path_in.clone()
    }

    /// Clone the typed odometry input port.
    #[must_use]
    pub fn odometry_input(&self) -> broadcast::Sender<Odometry> {
        self.odometry_in.clone()
    }

    /// Clone the typed stop input port. Only `true` cancels the active path.
    #[must_use]
    pub fn stop_input(&self) -> broadcast::Sender<bool> {
        self.stop_in.clone()
    }

    /// Subscribe to goal-reached events.
    #[must_use]
    pub fn subscribe_goal_reached(&self) -> broadcast::Receiver<bool> {
        self.goal_reached_tx.subscribe()
    }

    /// Run and publish one controller step synchronously.
    pub fn tick_and_publish(&mut self) -> Option<Twist2> {
        let control_guard = Arc::clone(&self.control_guard);
        let _control = control_guard
            .lock()
            .unwrap_or_else(|error| error.into_inner());
        let (path, odom) = snapshot_inputs(&self.inputs)?;
        let output = self.compute_step(&path, odom.as_ref());
        self.publish_step(output)
    }

    fn compute_step(&self, path: &Option<Path2D>, odom: Option<&PoseStamped>) -> crate::StepOutput {
        compute_step(&self.core_config, path, odom)
    }

    fn publish_step(&mut self, output: crate::StepOutput) -> Option<Twist2> {
        Some(publish_step(
            &self.inputs,
            &self.outputs,
            &self.cmd_vel_tx,
            &self.goal_reached_tx,
            output,
        ))
    }
}

impl Module for BasicPathFollowerModule {
    fn start(&mut self) -> Result<(), NavError> {
        if self.running {
            return Ok(());
        }
        let runtime = tokio::runtime::Handle::try_current().map_err(|error| {
            NavError::Internal(format!(
                "BasicPathFollowerModule::start requires a Tokio runtime: {error}"
            ))
        })?;

        let period = Duration::from_secs_f64(1.0 / f64::from(self.config.control_rate_hz));
        let mut path_rx = self.path_in.subscribe();
        let mut odometry_rx = self.odometry_in.subscribe();
        let mut stop_rx = self.stop_in.subscribe();
        let inputs = Arc::clone(&self.inputs);
        let outputs = Arc::clone(&self.outputs);
        let control_guard = Arc::clone(&self.control_guard);
        let lifecycle_generation = Arc::clone(&self.lifecycle_generation);
        let task_generation = lifecycle_generation.fetch_add(1, Ordering::AcqRel) + 1;
        let core_config = self.core_config.clone();
        let cmd_vel_tx = self.cmd_vel_tx.clone();
        let goal_reached_tx = self.goal_reached_tx.clone();

        self.lifecycle_handle = Some(runtime.spawn(async move {
            let mut ticker = tokio::time::interval(period);
            ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            ticker.tick().await;

            loop {
                tokio::select! {
                    biased;
                    received = stop_rx.recv() => match received {
                        Ok(true) => {
                            let _control = control_guard
                                .lock()
                                .unwrap_or_else(|error| error.into_inner());
                            if lifecycle_generation.load(Ordering::Acquire) != task_generation {
                                break;
                            }
                            cancel_path(&inputs, &outputs, &cmd_vel_tx);
                        }
                        Ok(false) | Err(broadcast::error::RecvError::Lagged(_)) => {},
                        Err(broadcast::error::RecvError::Closed) => break,
                    },
                    received = path_rx.recv() => match received {
                        Ok(path) => {
                            let _control = control_guard
                                .lock()
                                .unwrap_or_else(|error| error.into_inner());
                            if lifecycle_generation.load(Ordering::Acquire) != task_generation {
                                break;
                            }
                            set_path(&inputs, &outputs, &cmd_vel_tx, path);
                        }
                        Err(broadcast::error::RecvError::Lagged(_)) => {},
                        Err(broadcast::error::RecvError::Closed) => break,
                    },
                    received = odometry_rx.recv() => match received {
                        Ok(odometry) => {
                            let _control = control_guard
                                .lock()
                                .unwrap_or_else(|error| error.into_inner());
                            if lifecycle_generation.load(Ordering::Acquire) != task_generation {
                                break;
                            }
                            set_odometry(&inputs, odometry);
                        }
                        Err(broadcast::error::RecvError::Lagged(_)) => {},
                        Err(broadcast::error::RecvError::Closed) => break,
                    },
                    _ = ticker.tick() => {
                        let _control = control_guard
                            .lock()
                            .unwrap_or_else(|error| error.into_inner());
                        if lifecycle_generation.load(Ordering::Acquire) != task_generation {
                            break;
                        }
                        let Some((path, odom)) = snapshot_inputs(&inputs) else {
                            continue;
                        };
                        let output = compute_step(&core_config, &path, odom.as_ref());
                        publish_step(
                            &inputs,
                            &outputs,
                            &cmd_vel_tx,
                            &goal_reached_tx,
                            output,
                        );
                    }
                }
            }
        }));
        self.running = true;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), NavError> {
        let control_guard = Arc::clone(&self.control_guard);
        let _control = control_guard
            .lock()
            .unwrap_or_else(|error| error.into_inner());
        self.lifecycle_generation.fetch_add(1, Ordering::AcqRel);
        if let Some(handle) = self.lifecycle_handle.take() {
            handle.abort();
        }
        self.running = false;
        cancel_path(&self.inputs, &self.outputs, &self.cmd_vel_tx);
        Ok(())
    }

    fn on_path(&mut self, path: Path2D) {
        let control_guard = Arc::clone(&self.control_guard);
        let _control = control_guard
            .lock()
            .unwrap_or_else(|error| error.into_inner());
        set_path(&self.inputs, &self.outputs, &self.cmd_vel_tx, path);
    }

    fn on_odom(&mut self, odom: PoseStamped) {
        if let Ok(mut inputs) = self.inputs.lock() {
            inputs.odom = Some(odom);
        }
    }

    fn cmd_vel(&self) -> Option<Twist2> {
        self.outputs.lock().ok().and_then(|output| output.cmd_vel)
    }

    fn goal_reached(&self) -> bool {
        self.outputs
            .lock()
            .map(|output| output.goal_reached)
            .unwrap_or(false)
    }

    fn subscribe_cmd_vel(&self) -> mpsc::Receiver<Twist2> {
        let (sender, receiver) = mpsc::channel(PORT_CAPACITY);
        let mut watch_receiver = self.cmd_vel_tx.subscribe();
        tokio::spawn(async move {
            while watch_receiver.changed().await.is_ok() {
                let command = *watch_receiver.borrow_and_update();
                if sender.send(command).await.is_err() {
                    break;
                }
            }
        });
        receiver
    }
}

impl Drop for BasicPathFollowerModule {
    fn drop(&mut self) {
        self.lifecycle_generation.fetch_add(1, Ordering::AcqRel);
        if let Some(handle) = self.lifecycle_handle.take() {
            handle.abort();
        }
    }
}

fn snapshot_inputs(
    inputs: &Arc<Mutex<SharedInputs>>,
) -> Option<(Option<Path2D>, Option<PoseStamped>)> {
    let inputs = inputs.lock().ok()?;
    Some((inputs.path.clone(), inputs.odom.clone()))
}

fn set_path(
    inputs: &Arc<Mutex<SharedInputs>>,
    outputs: &Arc<Mutex<SharedOutputs>>,
    cmd_vel_tx: &watch::Sender<Twist2>,
    path: Path2D,
) {
    let empty = path.points.is_empty();
    if let Ok(mut inputs) = inputs.lock() {
        inputs.path = (!empty).then_some(path);
    }
    if let Ok(mut outputs) = outputs.lock() {
        outputs.goal_reached = false;
    }
    if empty {
        publish_twist(outputs, cmd_vel_tx, Twist2::ZERO);
    }
}

fn set_odometry(inputs: &Arc<Mutex<SharedInputs>>, odometry: Odometry) {
    if !odometry.is_finite() {
        return;
    }
    if let Ok(mut inputs) = inputs.lock() {
        inputs.odom = Some(PoseStamped {
            frame_id: odometry.frame_id,
            stamp: odometry.stamp,
            pose: odometry.pose,
        });
    }
}

fn cancel_path(
    inputs: &Arc<Mutex<SharedInputs>>,
    outputs: &Arc<Mutex<SharedOutputs>>,
    cmd_vel_tx: &watch::Sender<Twist2>,
) {
    if let Ok(mut inputs) = inputs.lock() {
        inputs.path = None;
    }
    if let Ok(mut outputs) = outputs.lock() {
        outputs.goal_reached = false;
    }
    publish_twist(outputs, cmd_vel_tx, Twist2::ZERO);
}

fn compute_step(
    config: &BasicPathFollowerConfig,
    path: &Option<Path2D>,
    odom: Option<&PoseStamped>,
) -> crate::StepOutput {
    let (Some(path), Some(odometry)) = (path.as_ref(), odom) else {
        return crate::StepOutput::no_path();
    };
    if path.frame_id != odometry.frame_id {
        return crate::StepOutput::no_path();
    }
    let mut core = match crate::BasicPathFollowerCore::new(config.clone()) {
        Ok(core) => core,
        Err(_) => return crate::StepOutput::no_path(),
    };
    if !path.points.is_empty() {
        core.set_path(
            path.points
                .iter()
                .map(|point| NumPoint2::new(point.x, point.y))
                .collect(),
        );
    }
    let position = NumPoint2::new(odometry.pose.position.x, odometry.pose.position.y);
    let yaw = odometry
        .pose
        .orientation
        .z
        .atan2(odometry.pose.orientation.w)
        * 2.0;
    core.step(position, yaw)
}

fn publish_step(
    inputs: &Arc<Mutex<SharedInputs>>,
    outputs: &Arc<Mutex<SharedOutputs>>,
    cmd_vel_tx: &watch::Sender<Twist2>,
    goal_reached_tx: &broadcast::Sender<bool>,
    output: crate::StepOutput,
) -> Twist2 {
    let command = if output.goal_reached {
        Twist2::ZERO
    } else {
        Twist2 {
            linear: MsgPoint2::new(output.command.linear, 0.0),
            angular_z: output.command.angular,
        }
    };
    if let Ok(mut shared) = outputs.lock() {
        shared.goal_reached = output.goal_reached;
    }
    if output.goal_reached {
        if let Ok(mut shared) = inputs.lock() {
            shared.path = None;
        }
        let _ = goal_reached_tx.send(true);
    }
    publish_twist(outputs, cmd_vel_tx, command);
    command
}

fn publish_twist(
    outputs: &Arc<Mutex<SharedOutputs>>,
    cmd_vel_tx: &watch::Sender<Twist2>,
    command: Twist2,
) {
    if let Ok(mut outputs) = outputs.lock() {
        outputs.cmd_vel = Some(command);
    }
    cmd_vel_tx.send_replace(command);
}
