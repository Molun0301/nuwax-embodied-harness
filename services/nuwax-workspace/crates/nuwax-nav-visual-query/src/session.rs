//! Session, lost-target, and stop lifecycle for the visual model owner.

use std::collections::BTreeSet;

use nuwax_nav_types_messages::{BoundingBox2D, CameraInfo, NavFrameId, PoseStamped, Timestamp};
use serde::{Deserialize, Serialize};

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

const MAX_SESSION_IDS: usize = 1_024;

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

/// Strict identity frozen before one production worker invocation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VisualQueryFrameIdentity {
    /// Caller-owned request identifier.
    pub request_id: String,
    /// Stable query-session identifier.
    pub session_id: String,
    /// Stable target identifier within the session.
    pub target_id: String,
    /// Original natural-language query.
    pub object_description: String,
    /// Source image timestamp in nanoseconds.
    pub source_timestamp_ns: u64,
    /// Source image optical frame.
    pub camera_frame: NavFrameId,
    /// Calibration snapshot identifier.
    pub calibration_id: String,
    /// Calibration snapshot used for projection.
    pub camera_info: CameraInfo,
    /// Source image width.
    pub image_width: u32,
    /// Source image height.
    pub image_height: u32,
}

/// Observable target lifecycle state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
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
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TargetObservation {
    /// Caller-owned request identifier.
    pub request_id: String,
    /// Stable session identifier.
    pub session_id: String,
    /// Stable target identifier within the session.
    pub target_id: String,
    /// Original natural-language query.
    pub object_description: String,
    /// Exact source image timestamp in nanoseconds.
    pub source_timestamp_ns: u64,
    /// Image timestamp in seconds for legacy visual-servoing adapters.
    pub timestamp: f64,
    /// Image optical frame.
    pub camera_frame: NavFrameId,
    /// Calibration snapshot identifier.
    pub calibration_id: String,
    /// Calibration snapshot used by the BBox tracker.
    pub camera_info: CameraInfo,
    /// Image width.
    pub image_width: u32,
    /// Image height.
    pub image_height: u32,
    /// Valid bbox only in [`TargetObservationState::Tracked`].
    pub bbox: Option<BoundingBox2D>,
    /// Camera-frame target pose produced by the BBox tracker.
    pub target_pose: Option<PoseStamped>,
    /// Session lifecycle state.
    pub state: TargetObservationState,
}

#[derive(Debug, Clone)]
struct ActiveSession {
    session_id: String,
    target_id: String,
    object_description: String,
    camera_frame: NavFrameId,
    calibration_id: String,
    camera_info: CameraInfo,
    image_width: u32,
    image_height: u32,
    last_identity: Option<VisualQueryFrameIdentity>,
    pending: Option<VisualQueryFrameIdentity>,
}

impl ActiveSession {
    fn from_identity(identity: VisualQueryFrameIdentity) -> Self {
        Self {
            session_id: identity.session_id.clone(),
            target_id: identity.target_id.clone(),
            object_description: identity.object_description.clone(),
            camera_frame: identity.camera_frame.clone(),
            calibration_id: identity.calibration_id.clone(),
            camera_info: identity.camera_info,
            image_width: identity.image_width,
            image_height: identity.image_height,
            last_identity: None,
            pending: Some(identity),
        }
    }

    fn identity_matches(&self, identity: &VisualQueryFrameIdentity) -> bool {
        self.session_id == identity.session_id
            && self.target_id == identity.target_id
            && self.object_description == identity.object_description
            && self.camera_frame == identity.camera_frame
            && self.calibration_id == identity.calibration_id
            && self.camera_info == identity.camera_info
            && self.image_width == identity.image_width
            && self.image_height == identity.image_height
    }

    fn latest_identity(&self) -> Option<VisualQueryFrameIdentity> {
        self.pending
            .as_ref()
            .or(self.last_identity.as_ref())
            .cloned()
    }
}

/// Stateful owner around any concrete [`VlBBoxModel`] backend.
pub struct VisualQuerySession {
    model: Box<dyn VlBBoxModel>,
    config: VisualQuerySessionConfig,
    active: Option<ActiveSession>,
    known_session_ids: BTreeSet<String>,
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
            known_session_ids: BTreeSet::new(),
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
                if session_id.trim().is_empty() || object_description.trim().is_empty() {
                    return None;
                }
                self.known_session_ids.remove(&session_id);
                self.active = Some(ActiveSession {
                    session_id,
                    target_id: object_description.clone(),
                    object_description,
                    camera_frame: NavFrameId::Camera,
                    calibration_id: String::new(),
                    camera_info: CameraInfo::default(),
                    image_width: 0,
                    image_height: 0,
                    last_identity: None,
                    pending: None,
                });
                self.lost_frames = 0;
                None
            }
            VisualQueryIntent::Stop { session_id } => self.stop_matching(&session_id),
        }
    }

    /// Bind one public request to the active session before spawning its worker.
    ///
    /// A different session replaces the previous one and returns its `Stopped` observation.
    /// A terminal session identifier cannot be reused implicitly; callers must allocate a new
    /// session identifier or explicitly apply a new `Start` intent.
    pub fn begin_request(
        &mut self,
        identity: VisualQueryFrameIdentity,
    ) -> Result<Option<TargetObservation>, VisualQueryError> {
        validate_identity(&identity)?;
        let active_is_same = self
            .active
            .as_ref()
            .is_some_and(|active| active.session_id == identity.session_id);
        if self.known_session_ids.contains(&identity.session_id) && !active_is_same {
            return Err(session_error("session is already terminal"));
        }
        if !self.known_session_ids.contains(&identity.session_id)
            && self.known_session_ids.len() >= MAX_SESSION_IDS
        {
            return Err(session_error("session identity capacity is exhausted"));
        }

        let placeholder_same = self.active.as_ref().is_some_and(|active| {
            active.session_id == identity.session_id
                && active.pending.is_none()
                && active.last_identity.is_none()
        });
        if placeholder_same {
            self.active = Some(ActiveSession::from_identity(identity.clone()));
            self.known_session_ids.insert(identity.session_id);
            self.lost_frames = 0;
            return Ok(None);
        }

        let replaced =
            match self.active.as_mut() {
                Some(active) if active.session_id == identity.session_id => {
                    if !active.identity_matches(&identity) {
                        return Err(session_error(
                            "target, frame, calibration, or image identity changed within session",
                        ));
                    }
                    if active.pending.is_some() {
                        return Err(session_error("session already has a pending request"));
                    }
                    if active.last_identity.as_ref().is_some_and(|last| {
                        identity.source_timestamp_ns <= last.source_timestamp_ns
                    }) {
                        return Err(session_error("source timestamp is stale or reordered"));
                    }
                    active.pending = Some(identity);
                    return Ok(None);
                }
                Some(active) => active
                    .latest_identity()
                    .map(|identity| stopped_observation(&identity)),
                None => None,
            };

        if let Some(stopped) = replaced.as_ref() {
            debug_assert!(self.known_session_ids.contains(&stopped.session_id));
        }
        self.known_session_ids.insert(identity.session_id.clone());
        self.active = Some(ActiveSession::from_identity(identity));
        self.lost_frames = 0;
        Ok(replaced)
    }

    /// Consume a real worker result after BBox tracking has produced the matching pose.
    pub fn complete_request(
        &mut self,
        identity: &VisualQueryFrameIdentity,
        bbox: Option<BoundingBox2D>,
        target_pose: Option<PoseStamped>,
    ) -> Result<TargetObservation, VisualQueryError> {
        let active = self
            .active
            .as_mut()
            .ok_or_else(|| session_error("worker result has no active session"))?;
        let pending = active
            .pending
            .as_ref()
            .ok_or_else(|| session_error("worker result has no pending request"))?;
        if pending != identity {
            return Err(session_error(
                "worker result identity does not match pending request",
            ));
        }
        validate_result(identity, bbox.as_ref(), target_pose.as_ref())?;

        let state = if bbox.is_some() {
            self.lost_frames = 0;
            TargetObservationState::Tracked
        } else {
            self.lost_frames = self.lost_frames.saturating_add(1);
            TargetObservationState::Lost {
                frames: self.lost_frames,
                terminal: self.lost_frames >= self.config.max_lost_frames,
            }
        };
        let observation = observation_from_identity(identity, bbox, target_pose, state);
        active.last_identity = Some(identity.clone());
        active.pending = None;

        if matches!(state, TargetObservationState::Lost { terminal: true, .. }) {
            self.known_session_ids.insert(identity.session_id.clone());
            self.active = None;
        }
        Ok(observation)
    }

    /// Stop a matching active session and preserve its latest request identity.
    pub fn stop_matching(&mut self, session_id: &str) -> Option<TargetObservation> {
        let active = self.active.as_ref()?;
        if active.session_id != session_id {
            return None;
        }
        let identity = active
            .latest_identity()
            .unwrap_or_else(|| VisualQueryFrameIdentity {
                request_id: String::new(),
                session_id: active.session_id.clone(),
                target_id: active.target_id.clone(),
                object_description: active.object_description.clone(),
                source_timestamp_ns: 0,
                camera_frame: active.camera_frame.clone(),
                calibration_id: active.calibration_id.clone(),
                camera_info: active.camera_info,
                image_width: active.image_width,
                image_height: active.image_height,
            });
        self.active = None;
        self.known_session_ids.insert(session_id.to_string());
        self.lost_frames = 0;
        Some(stopped_observation(&identity))
    }

    /// Stop the active session, if any, and preserve its latest request identity.
    pub fn stop_active(&mut self) -> Option<TargetObservation> {
        let session_id = self.active.as_ref()?.session_id.clone();
        self.stop_matching(&session_id)
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
            return Err(session_error(
                "image timestamp must be finite and non-negative",
            ));
        }
        if image.width == 0 || image.height == 0 {
            return Err(session_error("image dimensions must be positive"));
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
                self.known_session_ids.insert(active.session_id.clone());
                self.active = None;
            }
            TargetObservationState::Lost {
                frames: self.lost_frames,
                terminal,
            }
        };
        let source_timestamp_ns = (timestamp * 1_000_000_000.0) as u64;
        Ok(Some(TargetObservation {
            request_id: String::new(),
            session_id: active.session_id,
            target_id: active.target_id,
            object_description: active.object_description,
            source_timestamp_ns,
            timestamp,
            camera_frame,
            calibration_id: active.calibration_id,
            camera_info: active.camera_info,
            image_width: image.width,
            image_height: image.height,
            bbox,
            target_pose: None,
            state,
        }))
    }

    /// Whether a query session is currently armed.
    #[must_use]
    pub fn is_active(&self) -> bool {
        self.active.is_some()
    }
}

fn validate_identity(identity: &VisualQueryFrameIdentity) -> Result<(), VisualQueryError> {
    if identity.request_id.trim().is_empty()
        || identity.session_id.trim().is_empty()
        || identity.target_id.trim().is_empty()
        || identity.object_description.trim().is_empty()
        || identity.calibration_id.trim().is_empty()
    {
        return Err(session_error(
            "request, session, target, query, and calibration identity must be non-empty",
        ));
    }
    if identity.source_timestamp_ns == 0 || identity.source_timestamp_ns > i64::MAX as u64 {
        return Err(session_error(
            "source timestamp must be within the positive sensor clock range",
        ));
    }
    if identity.camera_frame != NavFrameId::Camera {
        return Err(session_error(
            "source frame must be the canonical camera calibration frame",
        ));
    }
    if !identity.camera_info.is_valid()
        || identity.camera_info.width != identity.image_width
        || identity.camera_info.height != identity.image_height
    {
        return Err(session_error(
            "camera calibration is invalid or does not match image dimensions",
        ));
    }
    Ok(())
}

fn validate_result(
    identity: &VisualQueryFrameIdentity,
    bbox: Option<&BoundingBox2D>,
    target_pose: Option<&PoseStamped>,
) -> Result<(), VisualQueryError> {
    match (bbox, target_pose) {
        (Some(bbox), Some(pose)) => {
            if !bbox.is_valid()
                || bbox.x1 < 0.0
                || bbox.y1 < 0.0
                || bbox.x2 > identity.image_width as f32
                || bbox.y2 > identity.image_height as f32
            {
                return Err(session_error("worker bbox is outside the source image"));
            }
            if pose.frame_id != identity.camera_frame
                || pose.stamp != Timestamp::from_sensor(identity.source_timestamp_ns as i64)
            {
                return Err(session_error(
                    "tracked pose frame or timestamp identity mismatch",
                ));
            }
        }
        (None, None) => {}
        _ => {
            return Err(session_error(
                "tracked bbox and target pose must appear together",
            ))
        }
    }
    Ok(())
}

fn observation_from_identity(
    identity: &VisualQueryFrameIdentity,
    bbox: Option<BoundingBox2D>,
    target_pose: Option<PoseStamped>,
    state: TargetObservationState,
) -> TargetObservation {
    TargetObservation {
        request_id: identity.request_id.clone(),
        session_id: identity.session_id.clone(),
        target_id: identity.target_id.clone(),
        object_description: identity.object_description.clone(),
        source_timestamp_ns: identity.source_timestamp_ns,
        timestamp: identity.source_timestamp_ns as f64 / 1_000_000_000.0,
        camera_frame: identity.camera_frame.clone(),
        calibration_id: identity.calibration_id.clone(),
        camera_info: identity.camera_info,
        image_width: identity.image_width,
        image_height: identity.image_height,
        bbox,
        target_pose,
        state,
    }
}

fn stopped_observation(identity: &VisualQueryFrameIdentity) -> TargetObservation {
    observation_from_identity(identity, None, None, TargetObservationState::Stopped)
}

fn session_error(message: impl Into<String>) -> VisualQueryError {
    VisualQueryError::ModelFailed(message.into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::VlBBoxModel;

    struct UnusedModel;

    impl VlBBoxModel for UnusedModel {
        fn query(&self, _image: ImageView<'_>, _prompt: &str) -> Result<String, VisualQueryError> {
            panic!("production-result tests must not invoke a second model")
        }
    }

    fn identity(request: &str, session: &str, stamp: u64) -> VisualQueryFrameIdentity {
        VisualQueryFrameIdentity {
            request_id: request.into(),
            session_id: session.into(),
            target_id: "target-chair".into(),
            object_description: "chair".into(),
            source_timestamp_ns: stamp,
            camera_frame: NavFrameId::Camera,
            calibration_id: "front-camera-v3".into(),
            camera_info: CameraInfo::new(640, 480, 500.0, 500.0, 320.0, 240.0),
            image_width: 640,
            image_height: 480,
        }
    }

    fn session(max_lost_frames: u32) -> VisualQuerySession {
        VisualQuerySession::new(
            Box::new(UnusedModel),
            VisualQuerySessionConfig { max_lost_frames },
        )
    }

    fn tracked_pose(identity: &VisualQueryFrameIdentity) -> PoseStamped {
        let mut pose = PoseStamped::from_pose(
            nuwax_nav_types_messages::Pose3::default(),
            identity.camera_frame.clone(),
        );
        pose.stamp = Timestamp::from_sensor(identity.source_timestamp_ns as i64);
        pose
    }

    #[test]
    fn worker_result_preserves_strict_identity() {
        let mut owner = session(2);
        let frame = identity("request-1", "session-1", 42);
        assert!(owner.begin_request(frame.clone()).unwrap().is_none());
        let bbox = BoundingBox2D::from_xyxy(10.0, 20.0, 30.0, 40.0).unwrap();
        let observation = owner
            .complete_request(&frame, Some(bbox.clone()), Some(tracked_pose(&frame)))
            .unwrap();
        assert_eq!(observation.request_id, frame.request_id);
        assert_eq!(observation.target_id, frame.target_id);
        assert_eq!(observation.source_timestamp_ns, 42);
        assert_eq!(observation.camera_frame, frame.camera_frame);
        assert_eq!(observation.calibration_id, frame.calibration_id);
        assert_eq!(observation.camera_info, frame.camera_info);
        assert_eq!(observation.bbox, Some(bbox));
        assert_eq!(observation.state, TargetObservationState::Tracked);
    }

    #[test]
    fn lost_is_bounded_and_terminal_session_stays_silent() {
        let mut owner = session(2);
        let first = identity("request-1", "session-1", 42);
        owner.begin_request(first.clone()).unwrap();
        let lost = owner.complete_request(&first, None, None).unwrap();
        assert_eq!(
            lost.state,
            TargetObservationState::Lost {
                frames: 1,
                terminal: false
            }
        );

        let second = identity("request-2", "session-1", 43);
        owner.begin_request(second.clone()).unwrap();
        let terminal = owner.complete_request(&second, None, None).unwrap();
        assert_eq!(
            terminal.state,
            TargetObservationState::Lost {
                frames: 2,
                terminal: true
            }
        );
        assert!(!owner.is_active());
        assert!(owner
            .begin_request(identity("request-3", "session-1", 44))
            .is_err());
    }

    #[test]
    fn production_session_identity_capacity_is_bounded_and_reuse_stays_closed() {
        let mut owner = session(2);
        for index in 0..MAX_SESSION_IDS {
            let frame = identity(
                &format!("request-{index}"),
                &format!("session-{index}"),
                index as u64 + 1,
            );
            owner.begin_request(frame).unwrap();
        }
        assert_eq!(owner.known_session_ids.len(), MAX_SESSION_IDS);

        let overflow = identity("request-overflow", "session-overflow", 2_000);
        assert!(owner.begin_request(overflow).is_err());
        let reused = identity("request-reused", "session-0", 2_001);
        assert!(owner.begin_request(reused).is_err());
        assert_eq!(owner.known_session_ids.len(), MAX_SESSION_IDS);
    }

    #[test]
    fn stale_or_changed_identity_fails_without_consuming_pending_request() {
        let mut owner = session(2);
        let first = identity("request-1", "session-1", 42);
        owner.begin_request(first.clone()).unwrap();
        owner.complete_request(&first, None, None).unwrap();

        let stale = identity("request-2", "session-1", 42);
        assert!(owner.begin_request(stale).is_err());
        let mut changed = identity("request-3", "session-1", 43);
        changed.calibration_id = "other-calibration".into();
        assert!(owner.begin_request(changed).is_err());
        assert!(owner
            .begin_request(identity("request-4", "session-1", 43))
            .is_ok());
    }

    #[test]
    fn completed_request_identity_survives_stop_and_session_replacement() {
        let mut owner = session(3);
        let tracked = identity("request-1", "session-1", 42);
        owner.begin_request(tracked.clone()).unwrap();
        let bbox = BoundingBox2D::from_xyxy(10.0, 20.0, 30.0, 40.0).unwrap();
        owner
            .complete_request(&tracked, Some(bbox), Some(tracked_pose(&tracked)))
            .unwrap();
        let stopped = owner.stop_active().unwrap();
        assert_eq!(stopped.request_id, tracked.request_id);
        assert_eq!(stopped.target_id, tracked.target_id);
        assert_eq!(stopped.source_timestamp_ns, tracked.source_timestamp_ns);
        assert_eq!(stopped.camera_frame, tracked.camera_frame);
        assert_eq!(stopped.calibration_id, tracked.calibration_id);

        let mut owner = session(3);
        let lost = identity("request-2", "session-2", 43);
        owner.begin_request(lost.clone()).unwrap();
        owner.complete_request(&lost, None, None).unwrap();
        let next = identity("request-3", "session-3", 44);
        let replaced = owner.begin_request(next).unwrap().unwrap();
        assert_eq!(replaced.request_id, lost.request_id);
        assert_eq!(replaced.target_id, lost.target_id);
        assert_eq!(replaced.source_timestamp_ns, lost.source_timestamp_ns);
        assert_eq!(replaced.camera_frame, lost.camera_frame);
        assert_eq!(replaced.calibration_id, lost.calibration_id);
        assert_eq!(replaced.state, TargetObservationState::Stopped);
        assert!(owner
            .begin_request(identity("request-4", "session-2", 45))
            .is_err());
    }

    #[test]
    fn stop_preserves_latest_request_identity_and_disarms_session() {
        let mut owner = session(2);
        let frame = identity("request-1", "session-1", 42);
        owner.begin_request(frame.clone()).unwrap();
        let stopped = owner.stop_matching("session-1").unwrap();
        assert_eq!(stopped.request_id, frame.request_id);
        assert_eq!(stopped.source_timestamp_ns, frame.source_timestamp_ns);
        assert_eq!(stopped.calibration_id, frame.calibration_id);
        assert_eq!(stopped.state, TargetObservationState::Stopped);
        assert!(!owner.is_active());
        assert!(owner.stop_matching("session-1").is_none());
    }
}
