//! Synchronous module boundary for [`crate::SimplePlanner`].
//!
//! A runtime supplies typed inputs and calls [`Module::tick`] at a fixed rate.
//! The adapter keeps transport concerns out of the planner while preserving the
//! original module's costmap, waypoint, path, and hold semantics.

use crate::{
    NavError, Point3, PointCloud, PointStamped, SimplePlanner, SimplePlannerConfig, Timestamp,
};
use nuwax_nav_types_messages::{NavFrameId, Path2D, PoseStamped};

/// Recommended lifecycle period (10 Hz).
pub const TICK_PERIOD_MS: u64 = 100;

/// Why the module emitted a stationary safety command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HoldReason {
    /// `stop_movement=true` cancelled the active goal.
    StopRequested,
    /// A* could not find a path through the current costmap.
    PlanningFailed,
    /// The robot entered the configured goal-reached radius.
    GoalReached,
    /// The lifecycle was stopped while a robot pose was available.
    LifecycleStopped,
}

/// A single lifecycle output.
#[derive(Debug, Clone, PartialEq)]
pub enum ModuleOutput {
    /// A fresh A* path and its leading lookahead waypoint.
    Plan {
        /// World-frame A* path.
        path: Path2D,
        /// Leading waypoint for the local planner.
        waypoint: PointStamped,
    },
    /// A refreshed waypoint from the cached path during the replan cooldown.
    Waypoint(PointStamped),
    /// A stationary waypoint plus an explicit diagnostic path.
    SafetyHold {
        /// Trigger for the hold.
        reason: HoldReason,
        /// Robot-position waypoint that commands no further motion.
        waypoint: PointStamped,
        /// One-point path for cancel/reached, or robot-to-goal for plan failure.
        path: Path2D,
    },
    /// No goal is active and no lifecycle output is pending.
    Idle,
}

/// Typed module input.
#[derive(Debug, Clone, PartialEq)]
pub enum ModuleInput {
    /// Robot pose normally supplied by the runtime's TF/odometry adapter.
    Odometry(PoseStamped),
    /// Navigation goal.
    Goal(PointStamped),
    /// Movement-cancellation signal.
    StopMovement(bool),
    /// Fast local terrain layered on top of the persistent costmap.
    TerrainMap(PointCloud),
    /// Persistent terrain snapshot that rebuilds the costmap.
    TerrainMapExt(PointCloud),
}

/// Lifecycle and typed input contract.
pub trait Module: Send + Sync + 'static {
    /// Start processing. Repeated calls while running are idempotent.
    fn start(&mut self) -> Result<(), NavError>;
    /// Stop processing and queue a stationary lifecycle hold.
    fn stop(&mut self) -> Result<(), NavError>;
    /// Update the robot pose.
    fn on_odometry(&mut self, pose: PoseStamped) -> Result<(), NavError>;
    /// Set or replace the active goal.
    fn on_goal(&mut self, goal: PointStamped) -> Result<(), NavError>;
    /// Cancel navigation when `stop` is true.
    fn on_stop_movement(&mut self, stop: bool) -> Result<(), NavError>;
    /// Layer a fast local terrain snapshot into the current costmap.
    fn on_terrain_map(&mut self, cloud: PointCloud) -> Result<(), NavError>;
    /// Rebuild the costmap from a persistent terrain snapshot.
    fn on_terrain_map_ext(&mut self, cloud: PointCloud) -> Result<(), NavError>;
    /// Advance planning by one lifecycle step.
    fn tick(&mut self) -> Result<ModuleOutput, NavError>;
}

/// Lifecycle wrapper around [`SimplePlanner`].
pub struct SimplePlannerModule {
    config: SimplePlannerConfig,
    planner: SimplePlanner,
    running: bool,
    clock_seconds: f64,
    last_pose: Option<PoseStamped>,
    active_goal: Option<PointStamped>,
    pending_hold: Option<HoldReason>,
}

impl SimplePlannerModule {
    /// Construct with the default planner configuration.
    #[must_use]
    pub fn new() -> Self {
        Self::with_config(SimplePlannerConfig::default())
            .expect("default SimplePlannerConfig must be valid")
    }

    /// Construct with an explicit planner configuration.
    ///
    /// # Errors
    /// Returns the same validation error as [`SimplePlanner::new`].
    pub fn with_config(config: SimplePlannerConfig) -> Result<Self, NavError> {
        Ok(Self {
            planner: SimplePlanner::new(config.clone())?,
            config,
            running: false,
            clock_seconds: 0.0,
            last_pose: None,
            active_goal: None,
            pending_hold: None,
        })
    }

    /// Whether the module is accepting input and normal ticks.
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.running
    }

    /// Dispatch a typed input through the validated runtime handlers.
    pub fn handle_input(&mut self, input: ModuleInput) -> Result<(), NavError> {
        match input {
            ModuleInput::Odometry(pose) => self.on_odometry(pose),
            ModuleInput::Goal(goal) => self.on_goal(goal),
            ModuleInput::StopMovement(stop) => self.on_stop_movement(stop),
            ModuleInput::TerrainMap(cloud) => self.on_terrain_map(cloud),
            ModuleInput::TerrainMapExt(cloud) => self.on_terrain_map_ext(cloud),
        }
    }

    fn ensure_running(&self) -> Result<(), NavError> {
        if self.running {
            Ok(())
        } else {
            Err(NavError::InvalidInput {
                field: "module",
                reason: "simple-planner module is not running".into(),
            })
        }
    }

    fn safety_hold(&self, reason: HoldReason) -> Result<ModuleOutput, NavError> {
        let pose = self.last_pose.as_ref().ok_or(NavError::InvalidInput {
            field: "odometry",
            reason: "cannot emit a safety hold without a robot pose".into(),
        })?;
        let robot = pose.pose.position.to_2d();
        let frame_id = NavFrameId::from_static(&self.config.world_frame);
        let stamp = pose.stamp;
        let waypoint = PointStamped::new(frame_id.clone(), stamp, pose.pose.position);
        let mut path = vec![robot];
        if reason == HoldReason::PlanningFailed {
            if let Some(goal) = self.active_goal.as_ref() {
                path.push(goal.point.to_2d());
            }
        }
        Ok(ModuleOutput::SafetyHold {
            reason,
            waypoint,
            path: Path2D::stamped(frame_id, stamp, path),
        })
    }
}

impl Default for SimplePlannerModule {
    fn default() -> Self {
        Self::new()
    }
}

impl Module for SimplePlannerModule {
    fn start(&mut self) -> Result<(), NavError> {
        if self.running {
            return Ok(());
        }
        self.planner = SimplePlanner::new(self.config.clone())?;
        self.clock_seconds = 0.0;
        self.last_pose = None;
        self.active_goal = None;
        self.pending_hold = None;
        self.running = true;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), NavError> {
        if !self.running {
            return Ok(());
        }
        self.planner.cancel_navigation();
        self.active_goal = None;
        if self.last_pose.is_some() {
            self.pending_hold = Some(HoldReason::LifecycleStopped);
        }
        self.running = false;
        Ok(())
    }

    fn on_odometry(&mut self, pose: PoseStamped) -> Result<(), NavError> {
        self.ensure_running()?;
        validate_frame(
            &pose.frame_id,
            &self.config.world_frame,
            "odometry.frame_id",
        )?;
        validate_point(pose.pose.position, "odometry")?;
        if !pose.pose.orientation.w.is_finite()
            || !pose.pose.orientation.x.is_finite()
            || !pose.pose.orientation.y.is_finite()
            || !pose.pose.orientation.z.is_finite()
        {
            return Err(NavError::InvalidInput {
                field: "odometry",
                reason: "orientation contains a non-finite value".into(),
            });
        }
        self.planner.update_odometry(pose.pose);
        self.last_pose = Some(pose);
        Ok(())
    }

    fn on_goal(&mut self, goal: PointStamped) -> Result<(), NavError> {
        self.ensure_running()?;
        validate_frame(&goal.frame_id, &self.config.world_frame, "goal.frame_id")?;
        validate_point(goal.point, "goal")?;
        self.planner.set_goal(goal.point, self.clock_seconds);
        self.active_goal = Some(goal);
        self.pending_hold = None;
        Ok(())
    }

    fn on_stop_movement(&mut self, stop: bool) -> Result<(), NavError> {
        self.ensure_running()?;
        if stop {
            self.planner.on_stop_movement(true);
            self.active_goal = None;
            self.pending_hold = Some(HoldReason::StopRequested);
        }
        Ok(())
    }

    fn on_terrain_map(&mut self, cloud: PointCloud) -> Result<(), NavError> {
        self.ensure_running()?;
        validate_cloud(&cloud, "terrain_map")?;
        if !cloud.is_empty() {
            self.planner.ingest_local_terrain(&cloud.points);
        }
        Ok(())
    }

    fn on_terrain_map_ext(&mut self, cloud: PointCloud) -> Result<(), NavError> {
        self.ensure_running()?;
        validate_cloud(&cloud, "terrain_map_ext")?;
        self.planner.ingest_world_terrain(&cloud.points);
        Ok(())
    }

    fn tick(&mut self) -> Result<ModuleOutput, NavError> {
        if let Some(reason) = self.pending_hold.take() {
            return self.safety_hold(reason);
        }
        self.ensure_running()?;
        self.clock_seconds += TICK_PERIOD_MS as f64 / 1_000.0;

        if let (Some(pose), Some(goal)) = (self.last_pose.as_ref(), self.active_goal.as_ref()) {
            if pose.pose.position.to_2d().distance(goal.point.to_2d())
                <= self.config.goal_reached_threshold
            {
                self.planner.cancel_navigation();
                self.active_goal = None;
                return self.safety_hold(HoldReason::GoalReached);
            }
        }

        let (plan, did_replan, _) = self.planner.replan_once(self.clock_seconds);
        match plan {
            Some(Some(path)) => {
                let mut waypoint = self.planner.update_waypoint().ok_or_else(|| {
                    NavError::Internal("planner produced a path without a waypoint".into())
                })?;
                waypoint.stamp = self
                    .last_pose
                    .as_ref()
                    .map_or_else(Timestamp::default, |pose| pose.stamp);
                Ok(ModuleOutput::Plan {
                    path: Path2D::stamped(waypoint.frame_id.clone(), waypoint.stamp, path),
                    waypoint,
                })
            }
            Some(None) => self.safety_hold(HoldReason::PlanningFailed),
            None if did_replan => Err(NavError::Internal(
                "planner failed before producing a path result".into(),
            )),
            None if self.active_goal.is_some() && self.planner.cached_path().is_some() => self
                .planner
                .update_waypoint()
                .map_or(Ok(ModuleOutput::Idle), |mut waypoint| {
                    waypoint.stamp = self
                        .last_pose
                        .as_ref()
                        .map_or_else(Timestamp::default, |pose| pose.stamp);
                    Ok(ModuleOutput::Waypoint(waypoint))
                }),
            None => Ok(ModuleOutput::Idle),
        }
    }
}

fn validate_frame(
    actual: &NavFrameId,
    expected: &str,
    field: &'static str,
) -> Result<(), NavError> {
    if actual.as_str() == expected {
        Ok(())
    } else {
        Err(NavError::InvalidInput {
            field,
            reason: format!(
                "frame `{}` does not match configured world frame `{expected}`",
                actual.as_str()
            ),
        })
    }
}

fn validate_cloud(cloud: &PointCloud, field: &'static str) -> Result<(), NavError> {
    cloud.validate().map_err(|reason| NavError::InvalidInput {
        field,
        reason: reason.into(),
    })?;
    for point in &cloud.points {
        validate_point(*point, field)?;
    }
    Ok(())
}

fn validate_point(point: Point3, field: &'static str) -> Result<(), NavError> {
    if point.x.is_finite() && point.y.is_finite() && point.z.is_finite() {
        Ok(())
    } else {
        Err(NavError::InvalidInput {
            field,
            reason: "contains a non-finite coordinate".into(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cloud(points: Vec<Point3>) -> PointCloud {
        PointCloud {
            stamp: Timestamp::default(),
            frame_id: NavFrameId::Map,
            points,
            intensities: Vec::new(),
        }
    }

    #[test]
    fn empty_persistent_snapshot_clears_previous_obstacles() {
        let mut module = SimplePlannerModule::new();
        module.start().expect("start");
        module
            .on_terrain_map_ext(cloud(vec![Point3::new(1.0, 1.0, 1.0)]))
            .expect("occupied snapshot");
        assert!(!module.planner.costmap().heights().is_empty());

        module
            .on_terrain_map_ext(cloud(Vec::new()))
            .expect("all-free snapshot");
        assert!(module.planner.costmap().heights().is_empty());
    }
}
