//! Synchronous module boundary for the replanning A* navigation stack.
//!
//! The runtime owns transport and scheduling. This adapter validates typed
//! odometry/costmap/goal inputs, drives [`GlobalPlanner`], and exposes every
//! control tick as one coherent navigation update.

use crate::global_planner::{
    GlobalPlanner, GlobalPlannerConfig, GlobalPlannerEvent, GlobalPlannerOutput,
};
use crate::helpers::{
    make_path2d_empty, make_twist2_zero, NavError, NavigationStatus, Path2D, PoseStamped, Twist2,
};
use crate::navigation_map::CostMap2D;

/// RPC-compatible navigation lifecycle.
pub trait NavigationInterface {
    /// Start the module. Repeated calls while running are idempotent.
    fn start(&mut self) -> Result<(), NavError>;
    /// Stop the planner and queue a zero-velocity safety update.
    fn stop(&mut self) -> Result<(), NavError>;
    /// Set or replace the active goal.
    fn set_goal(&mut self, goal: PoseStamped) -> Result<bool, NavError>;
    /// Cancel the active goal and queue a zero-velocity safety update.
    fn cancel_goal(&mut self) -> Result<bool, NavError>;
    /// Current navigation state.
    fn get_state(&self) -> &'static str;
    /// Whether the current goal was reached.
    fn is_goal_reached(&self) -> bool;
    /// Enable or disable automatic replanning.
    fn set_replanning_enabled(&mut self, enabled: bool) -> Result<(), NavError>;
    /// Override the safe-goal clearance in metres.
    fn set_safe_goal_clearance(&mut self, clearance: f64) -> Result<(), NavError>;
    /// Restore the safe-goal clearance derived from robot geometry.
    fn reset_safe_goal_clearance(&mut self) -> Result<(), NavError>;
}

/// One complete navigation-cycle result.
#[derive(Debug, Clone, PartialEq)]
pub struct NavigationUpdate {
    /// Current navigation state.
    pub status: NavigationStatus,
    /// Most recently published global path, if any.
    pub path: Option<Path2D>,
    /// Velocity command for this cycle.
    pub twist: Twist2,
    /// Whether the goal has been reached.
    pub goal_reached: bool,
    /// Planner event emitted by this cycle, including terminal failure pulses.
    pub event: Option<GlobalPlannerEvent>,
}

/// A single lifecycle output.
#[derive(Debug, Clone, PartialEq)]
pub enum ModuleOutput {
    /// Normal planner/control update.
    Navigation(NavigationUpdate),
    /// Explicit zero-velocity update produced by cancel or lifecycle stop.
    SafetyHold(NavigationUpdate),
}

/// Typed lifecycle input contract.
pub trait Module: Send + Sync + 'static {
    /// Advance the global and local planners by one control cycle.
    fn tick(&mut self) -> Result<ModuleOutput, NavError>;
    /// Update robot odometry.
    fn on_odom(&mut self, odom: PoseStamped) -> Result<(), NavError>;
    /// Replace the global occupancy costmap.
    fn on_costmap(&mut self, grid: CostMap2D) -> Result<(), NavError>;
    /// Set or replace the active goal.
    fn on_goal(&mut self, goal: PoseStamped) -> Result<(), NavError>;
    /// Cancel navigation when `stop` is true.
    fn on_stop_movement(&mut self, stop: bool) -> Result<(), NavError>;
}

/// Lifecycle wrapper around [`GlobalPlanner`].
pub struct ReplanningAStarPlannerModule {
    config: GlobalPlannerConfig,
    planner: GlobalPlanner,
    running: bool,
    has_odom: bool,
    costmap: Option<CostMap2D>,
    pending_hold: Option<NavigationStatus>,
}

impl ReplanningAStarPlannerModule {
    /// Construct an inactive module.
    #[must_use]
    pub fn new(config: GlobalPlannerConfig) -> Self {
        Self {
            planner: GlobalPlanner::new(config.clone()),
            config,
            running: false,
            has_odom: false,
            costmap: None,
            pending_hold: None,
        }
    }

    /// Access the underlying planner for diagnostics and advanced configuration.
    pub fn planner_mut(&mut self) -> &mut GlobalPlanner {
        &mut self.planner
    }

    /// Whether the module is accepting typed inputs and normal ticks.
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.running
    }

    fn ensure_running(&self) -> Result<(), NavError> {
        if self.running {
            Ok(())
        } else {
            Err(NavError::InvalidInput {
                field: "module",
                reason: "replanning A* module is not running".into(),
            })
        }
    }

    fn accept_goal(&mut self, goal: PoseStamped) -> Result<(), NavError> {
        self.ensure_running()?;
        validate_pose(&goal, "goal")?;
        if !self.has_odom {
            return Err(NavError::InvalidInput {
                field: "odometry",
                reason: "goal received before odometry".into(),
            });
        }
        let costmap = self.costmap.as_ref().ok_or(NavError::MapUnavailable)?;
        let (cell_x, cell_y) = costmap.world_to_cell(goal.pose.position.to_2d());
        if !costmap.in_bounds(cell_x, cell_y) {
            return Err(NavError::GoalOutOfBounds);
        }
        self.planner.handle_goal_request(goal);
        self.pending_hold = None;
        Ok(())
    }

    fn safety_hold(&self, status: NavigationStatus) -> ModuleOutput {
        ModuleOutput::SafetyHold(NavigationUpdate {
            status,
            path: Some(make_path2d_empty()),
            twist: make_twist2_zero(),
            goal_reached: self.planner.is_goal_reached(),
            event: None,
        })
    }
}

impl NavigationInterface for ReplanningAStarPlannerModule {
    fn start(&mut self) -> Result<(), NavError> {
        if self.running {
            return Ok(());
        }
        self.planner = GlobalPlanner::new(self.config.clone());
        self.has_odom = false;
        self.costmap = None;
        self.pending_hold = None;
        self.planner.start();
        self.running = true;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), NavError> {
        if !self.running {
            return Ok(());
        }
        self.planner.stop();
        self.pending_hold = Some(NavigationStatus::Cancelled);
        self.running = false;
        Ok(())
    }

    fn set_goal(&mut self, goal: PoseStamped) -> Result<bool, NavError> {
        self.accept_goal(goal)?;
        Ok(true)
    }

    fn cancel_goal(&mut self) -> Result<bool, NavError> {
        self.ensure_running()?;
        self.planner.cancel_goal(false);
        self.pending_hold = Some(NavigationStatus::Cancelled);
        Ok(true)
    }

    fn get_state(&self) -> &'static str {
        match self.planner.status() {
            NavigationStatus::Idle => "idle",
            NavigationStatus::Planning => "planning",
            NavigationStatus::Moving => "moving",
            NavigationStatus::Reached => "reached",
            NavigationStatus::Blocked => "blocked",
            NavigationStatus::Cancelled => "cancelled",
            NavigationStatus::Error => "error",
        }
    }

    fn is_goal_reached(&self) -> bool {
        self.planner.is_goal_reached()
    }

    fn set_replanning_enabled(&mut self, enabled: bool) -> Result<(), NavError> {
        self.ensure_running()?;
        self.planner.set_replanning_enabled(enabled);
        Ok(())
    }

    fn set_safe_goal_clearance(&mut self, clearance: f64) -> Result<(), NavError> {
        self.ensure_running()?;
        if !clearance.is_finite() || clearance < 0.0 {
            return Err(NavError::InvalidInput {
                field: "safe_goal_clearance",
                reason: "must be finite and non-negative".into(),
            });
        }
        self.config.safe_goal_clearance = clearance;
        self.planner.set_safe_goal_clearance(clearance);
        Ok(())
    }

    fn reset_safe_goal_clearance(&mut self) -> Result<(), NavError> {
        self.ensure_running()?;
        self.config.safe_goal_clearance = self.config.robot_rotation_diameter / 2.0;
        self.planner.reset_safe_goal_clearance();
        Ok(())
    }
}

impl Module for ReplanningAStarPlannerModule {
    fn tick(&mut self) -> Result<ModuleOutput, NavError> {
        if let Some(status) = self.pending_hold.take() {
            return Ok(self.safety_hold(status));
        }
        self.ensure_running()?;
        let costmap = self.costmap.clone().ok_or(NavError::MapUnavailable)?;
        let output = self.planner.tick(&costmap);
        Ok(ModuleOutput::Navigation(navigation_update(
            output,
            self.planner.is_goal_reached(),
        )))
    }

    fn on_odom(&mut self, odom: PoseStamped) -> Result<(), NavError> {
        self.ensure_running()?;
        validate_pose(&odom, "odometry")?;
        self.planner.handle_odom(odom);
        self.has_odom = true;
        Ok(())
    }

    fn on_costmap(&mut self, grid: CostMap2D) -> Result<(), NavError> {
        self.ensure_running()?;
        if grid.width() == 0
            || grid.height() == 0
            || !grid.resolution().is_finite()
            || grid.resolution() <= 0.0
        {
            return Err(NavError::InvalidInput {
                field: "global_costmap",
                reason: "width, height, and resolution must be positive".into(),
            });
        }
        self.planner.handle_global_costmap(grid.clone());
        self.costmap = Some(grid);
        Ok(())
    }

    fn on_goal(&mut self, goal: PoseStamped) -> Result<(), NavError> {
        self.accept_goal(goal)
    }

    fn on_stop_movement(&mut self, stop: bool) -> Result<(), NavError> {
        self.ensure_running()?;
        if stop {
            self.planner.cancel_goal(false);
            self.pending_hold = Some(NavigationStatus::Cancelled);
        }
        Ok(())
    }
}

/// Convert a planar twist into the two fields used by lightweight transports.
#[must_use]
pub fn twist_to_proto(twist: Twist2) -> (f64, f64) {
    (twist.linear.x, twist.angular_z)
}

fn navigation_update(output: GlobalPlannerOutput, goal_reached: bool) -> NavigationUpdate {
    NavigationUpdate {
        status: output.status,
        path: output.path,
        twist: output.twist,
        goal_reached,
        event: output.event,
    }
}

fn validate_pose(pose: &PoseStamped, field: &'static str) -> Result<(), NavError> {
    let position = pose.pose.position;
    let orientation = pose.pose.orientation;
    if position.x.is_finite()
        && position.y.is_finite()
        && position.z.is_finite()
        && orientation.w.is_finite()
        && orientation.x.is_finite()
        && orientation.y.is_finite()
        && orientation.z.is_finite()
    {
        Ok(())
    } else {
        Err(NavError::InvalidInput {
            field,
            reason: "pose contains a non-finite value".into(),
        })
    }
}
