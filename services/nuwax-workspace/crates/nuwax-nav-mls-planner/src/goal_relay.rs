// Copyright 2026 Nuwax Inc.
// SPDX-License-Identifier: Apache-2.0

//! Lifecycle adapter from odometry and point goals to stamped planner poses.

/// A stamped point goal.
#[derive(Clone, Debug, PartialEq)]
pub struct StampedPoint {
    /// Timestamp in seconds.
    pub timestamp: f64,
    /// Coordinate frame.
    pub frame_id: String,
    /// Point position.
    pub position: (f32, f32, f32),
}

/// Minimal odometry input required by the relay.
#[derive(Clone, Debug, PartialEq)]
pub struct RelayOdometry {
    /// Timestamp in seconds.
    pub timestamp: f64,
    /// Coordinate frame.
    pub frame_id: String,
    /// Robot position.
    pub position: (f32, f32, f32),
    /// Robot orientation `(x, y, z, w)`.
    pub orientation: (f32, f32, f32, f32),
}

/// Pose input consumed by the MLS worker.
#[derive(Clone, Debug, PartialEq)]
pub struct StampedPose {
    /// Timestamp in seconds.
    pub timestamp: f64,
    /// Coordinate frame.
    pub frame_id: String,
    /// Pose position.
    pub position: (f32, f32, f32),
    /// Pose orientation `(x, y, z, w)`.
    pub orientation: (f32, f32, f32, f32),
}

/// Goal relay lifecycle error.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum GoalRelayError {
    /// Input arrived outside the running lifecycle.
    #[error("goal relay is not running")]
    NotRunning,
    #[error("timestamp must be finite")]
    /// Timestamp is NaN or infinite.
    NonFiniteTimestamp,
    #[error("frame_id must not be empty")]
    /// Coordinate frame is empty or whitespace-only.
    EmptyFrameId,
    #[error("position must contain only finite values")]
    /// Position contains NaN or infinity.
    NonFinitePosition,
    #[error("orientation must be finite and have non-zero norm")]
    /// Quaternion contains non-finite values or has zero norm.
    InvalidQuaternion,
}

/// Synchronous owner for the Dimos goal-relay conversion boundary.
#[derive(Default)]
pub struct GoalRelay {
    running: bool,
}

impl GoalRelay {
    /// Construct a stopped relay.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Start the relay. Repeated starts are idempotent.
    pub fn start(&mut self) {
        self.running = true;
    }

    /// Stop the relay.
    pub fn stop(&mut self) {
        self.running = false;
    }

    /// Whether inputs are currently accepted.
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.running
    }

    /// Convert odometry to the planner start pose.
    pub fn relay_odometry(&self, odometry: RelayOdometry) -> Result<StampedPose, GoalRelayError> {
        self.ensure_running()?;
        validate_common(odometry.timestamp, &odometry.frame_id, odometry.position)?;
        validate_quaternion(odometry.orientation)?;
        Ok(StampedPose {
            timestamp: odometry.timestamp,
            frame_id: odometry.frame_id,
            position: odometry.position,
            orientation: odometry.orientation,
        })
    }

    /// Convert a point goal to an identity-orientation planner goal pose.
    pub fn relay_goal(&self, goal: StampedPoint) -> Result<StampedPose, GoalRelayError> {
        self.ensure_running()?;
        validate_common(goal.timestamp, &goal.frame_id, goal.position)?;
        Ok(StampedPose {
            timestamp: goal.timestamp,
            frame_id: goal.frame_id,
            position: goal.position,
            orientation: (0.0, 0.0, 0.0, 1.0),
        })
    }

    fn ensure_running(&self) -> Result<(), GoalRelayError> {
        self.running.then_some(()).ok_or(GoalRelayError::NotRunning)
    }
}

fn validate_common(
    timestamp: f64,
    frame_id: &str,
    position: (f32, f32, f32),
) -> Result<(), GoalRelayError> {
    if !timestamp.is_finite() {
        return Err(GoalRelayError::NonFiniteTimestamp);
    }
    if frame_id.trim().is_empty() {
        return Err(GoalRelayError::EmptyFrameId);
    }
    if ![position.0, position.1, position.2]
        .into_iter()
        .all(f32::is_finite)
    {
        return Err(GoalRelayError::NonFinitePosition);
    }
    Ok(())
}

fn validate_quaternion(quaternion: (f32, f32, f32, f32)) -> Result<(), GoalRelayError> {
    let values = [quaternion.0, quaternion.1, quaternion.2, quaternion.3];
    let norm_squared = values.into_iter().map(|value| value * value).sum::<f32>();
    if !values.into_iter().all(f32::is_finite)
        || !norm_squared.is_finite()
        || norm_squared <= f32::EPSILON
    {
        return Err(GoalRelayError::InvalidQuaternion);
    }
    Ok(())
}
