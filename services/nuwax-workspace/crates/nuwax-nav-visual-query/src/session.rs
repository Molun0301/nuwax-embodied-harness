//! Session, lost-target, and stop lifecycle for the visual model owner.

use nuwax_nav_types_messages::{BoundingBox2D, NavFrameId};

use crate::{get_object_bbox_from_image, ImageView, VisualQueryError, VlBBoxModel};

/// Query-session configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VisualQuerySessionConfig {
    /// End a session after this many consecutive lost observations.
    pub max_lost_frames: u32,
}

impl Default for VisualQuerySessionConfig {
    fn default() -> Self {
        Self {
            max_lost_frames: 15,
        }
    }
}

/// User or arbiter request that starts or stops visual target acquisition.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum VisualQueryIntent {
    /// Start or replace the active session.
    Start {
        /// Stable caller-owned session identifier.
        session_id: String,
        /// Natural-language target description.
        object_description: String,
    },
    /// Stop a matching active session.
    Stop {
        /// Session identifier to stop.
        session_id: String,
    },
}

/// Observable target lifecycle state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetObservationState {
    /// Model returned a valid bbox.
    Tracked,
    /// Model did not find a valid bbox.
    Lost {
        /// Consecutive lost observations.
        frames: u32,
        /// The session was disarmed by reaching the configured limit.
        terminal: bool,
    },
    /// Session was explicitly stopped.
    Stopped,
}

/// Model observation consumed by navigation arbitration or visual servoing.
#[derive(Debug, Clone, PartialEq)]
pub struct TargetObservation {
    /// Stable session identifier.
    pub session_id: String,
    /// Original natural-language query.
    pub object_description: String,
    /// Image timestamp in seconds.
    pub timestamp: f64,
    /// Image optical frame.
    pub camera_frame: NavFrameId,
    /// Image width.
    pub image_width: u32,
    /// Image height.
    pub image_height: u32,
    /// Valid bbox only in [`TargetObservationState::Tracked`].
    pub bbox: Option<BoundingBox2D>,
    /// Session lifecycle state.
    pub state: TargetObservationState,
}

#[derive(Debug, Clone)]
struct ActiveSession {
    session_id: String,
    object_description: String,
}

/// Stateful owner around any concrete [`VlBBoxModel`] backend.
pub struct VisualQuerySession {
    model: Box<dyn VlBBoxModel>,
    config: VisualQuerySessionConfig,
    active: Option<ActiveSession>,
    lost_frames: u32,
}

impl VisualQuerySession {
    /// Construct an idle session owner.
    #[must_use]
    pub fn new(model: Box<dyn VlBBoxModel>, config: VisualQuerySessionConfig) -> Self {
        Self {
            model,
            config,
            active: None,
            lost_frames: 0,
        }
    }

    /// Apply a start/stop intent. Stop only affects the matching session.
    pub fn apply_intent(&mut self, intent: VisualQueryIntent) -> Option<TargetObservation> {
        match intent {
            VisualQueryIntent::Start {
                session_id,
                object_description,
            } => {
                if session_id.is_empty() || object_description.trim().is_empty() {
                    return None;
                }
                self.active = Some(ActiveSession {
                    session_id,
                    object_description,
                });
                self.lost_frames = 0;
                None
            }
            VisualQueryIntent::Stop { session_id } => {
                let active = self.active.as_ref()?;
                if active.session_id != session_id {
                    return None;
                }
                let output = TargetObservation {
                    session_id: active.session_id.clone(),
                    object_description: active.object_description.clone(),
                    timestamp: 0.0,
                    camera_frame: NavFrameId::Camera,
                    image_width: 0,
                    image_height: 0,
                    bbox: None,
                    state: TargetObservationState::Stopped,
                };
                self.active = None;
                self.lost_frames = 0;
                Some(output)
            }
        }
    }

    /// Query one image. Idle sessions emit nothing.
    pub fn observe(
        &mut self,
        image: ImageView<'_>,
        timestamp: f64,
        camera_frame: NavFrameId,
    ) -> Result<Option<TargetObservation>, VisualQueryError> {
        let Some(active) = self.active.clone() else {
            return Ok(None);
        };
        if !timestamp.is_finite() || timestamp < 0.0 {
            return Err(VisualQueryError::ModelFailed(
                "image timestamp must be finite and non-negative".into(),
            ));
        }
        if image.width == 0 || image.height == 0 {
            return Err(VisualQueryError::ModelFailed(
                "image dimensions must be positive".into(),
            ));
        }
        let bbox =
            get_object_bbox_from_image(self.model.as_ref(), image, &active.object_description)?;
        let state = if bbox.is_some() {
            self.lost_frames = 0;
            TargetObservationState::Tracked
        } else {
            self.lost_frames = self.lost_frames.saturating_add(1);
            let terminal = self.lost_frames >= self.config.max_lost_frames;
            if terminal {
                self.active = None;
            }
            TargetObservationState::Lost {
                frames: self.lost_frames,
                terminal,
            }
        };
        Ok(Some(TargetObservation {
            session_id: active.session_id,
            object_description: active.object_description,
            timestamp,
            camera_frame,
            image_width: image.width,
            image_height: image.height,
            bbox,
            state,
        }))
    }

    /// Whether a query session is currently armed.
    #[must_use]
    pub fn is_active(&self) -> bool {
        self.active.is_some()
    }
}
