//! Composition contracts from model observations to arbitration or servo input.

use nuwax_nav_visual_query::session::{TargetObservation, TargetObservationState};
use thiserror::Error;

use crate::person_follow::{TrackedPerson, TrackingState};
use crate::{BBox, CameraIntrinsics};

/// Camera-frame navigation request that must pass through the canonical goal arbiter.
#[derive(Debug, Clone, PartialEq)]
pub struct GoalIntent {
    /// Stable source session.
    pub session_id: String,
    /// Camera optical frame.
    pub camera_frame: String,
    /// Relative camera-frame target `(forward, left, up)` in metres.
    pub relative_position: [f64; 3],
    /// Source image timestamp.
    pub timestamp: f64,
    /// Producer identity used by arbitration diagnostics.
    pub producer: &'static str,
}

/// Composition failures are rejected before reaching arbitration or servo control.
#[derive(Debug, Error, PartialEq)]
pub enum VisualCompositionError {
    /// Observation does not carry a tracked bbox.
    #[error("target observation is not tracked")]
    NotTracked,
    /// Camera model or target distance is invalid.
    #[error("invalid camera projection: {0}")]
    InvalidProjection(String),
}

/// Convert a tracked bbox to the Dimos `(z, -x, -y)` camera-frame goal intent.
///
/// This function only creates data. It deliberately has no transport or command sink;
/// callers must submit the result to the canonical navigation goal arbiter.
pub fn bbox_to_goal_intent(
    observation: &TargetObservation,
    intrinsics: CameraIntrinsics,
    goal_distance: f64,
) -> Result<GoalIntent, VisualCompositionError> {
    let bbox = tracked_bbox(observation)?;
    validate_intrinsics(observation, intrinsics)?;
    if !goal_distance.is_finite() || goal_distance <= 0.0 {
        return Err(VisualCompositionError::InvalidProjection(
            "goal_distance must be finite and positive".into(),
        ));
    }
    let (center_x, center_y) = bbox.center();
    let x = (center_x - intrinsics.cx) / intrinsics.fx * goal_distance;
    let y = (center_y - intrinsics.cy) / intrinsics.fy * goal_distance;
    Ok(GoalIntent {
        session_id: observation.session_id.clone(),
        camera_frame: observation.camera_frame.as_str().to_owned(),
        relative_position: [goal_distance, -x, -y],
        timestamp: observation.timestamp,
        producer: "nuwax-nav-visual-query",
    })
}

/// Adapt a model observation into the existing person-follow servo owner input.
pub fn target_observation_to_servo(
    observation: &TargetObservation,
    intrinsics: CameraIntrinsics,
) -> Result<TrackedPerson, VisualCompositionError> {
    if !intrinsics.is_valid() {
        return Err(VisualCompositionError::InvalidProjection(
            "camera intrinsics are invalid".into(),
        ));
    }
    let state = match observation.state {
        TargetObservationState::Tracked => {
            validate_image_dimensions(observation, intrinsics)?;
            TrackingState::Tracked
        }
        TargetObservationState::Lost { .. } => TrackingState::Lost,
        TargetObservationState::Stopped => TrackingState::Stop,
    };
    let bbox = if observation.state == TargetObservationState::Tracked {
        tracked_bbox(observation)?
    } else {
        BBox::new(0.0, 0.0, 0.0, 0.0)
    };
    Ok(TrackedPerson {
        session_id: observation.session_id.clone(),
        query: observation.object_description.clone(),
        image_timestamp: observation.timestamp,
        camera_frame: observation.camera_frame.as_str().to_owned(),
        image_width: if observation.image_width == 0 {
            intrinsics.width
        } else {
            observation.image_width
        },
        image_height: if observation.image_height == 0 {
            intrinsics.height
        } else {
            observation.image_height
        },
        intrinsics,
        bbox,
        state,
    })
}

fn tracked_bbox(observation: &TargetObservation) -> Result<BBox, VisualCompositionError> {
    if observation.state != TargetObservationState::Tracked {
        return Err(VisualCompositionError::NotTracked);
    }
    let bbox = observation
        .bbox
        .as_ref()
        .ok_or(VisualCompositionError::NotTracked)?;
    Ok(BBox::new(
        bbox.x1.into(),
        bbox.y1.into(),
        bbox.x2.into(),
        bbox.y2.into(),
    ))
}

fn validate_intrinsics(
    observation: &TargetObservation,
    intrinsics: CameraIntrinsics,
) -> Result<(), VisualCompositionError> {
    if !intrinsics.is_valid() {
        return Err(VisualCompositionError::InvalidProjection(
            "camera intrinsics are invalid".into(),
        ));
    }
    validate_image_dimensions(observation, intrinsics)
}

fn validate_image_dimensions(
    observation: &TargetObservation,
    intrinsics: CameraIntrinsics,
) -> Result<(), VisualCompositionError> {
    if observation.image_width != intrinsics.width || observation.image_height != intrinsics.height
    {
        return Err(VisualCompositionError::InvalidProjection(
            "image dimensions do not match camera intrinsics".into(),
        ));
    }
    Ok(())
}
